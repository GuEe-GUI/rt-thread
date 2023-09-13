/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-24     GuEe-GUI     first version
 */

#include <rthw.h>
#include <rtthread.h>

#define DBG_TAG "fw.raspberrypi"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <cpuport.h>
#include "firmware.h"

#define MBOX_MSG(chan, data28)  (((data28) & ~0xf) | ((chan) & 0xf))
#define MBOX_CHAN(msg)          ((msg) & 0xf)
#define MBOX_DATA28(msg)        ((msg) & ~0xf)
#define MBOX_CHAN_PROPERTY      8

struct rpi_firmware
{
    struct rt_mbox_client parent;

    /* The property channel. */
    struct rt_mbox_chan *chan;
    struct rt_completion done;
    rt_uint32_t enabled;

    struct ref consumers;
    rt_uint32_t get_throttled;

    struct rt_spinlock transaction_lock;
};

static rt_err_t rpi_firmware_transaction(struct rpi_firmware *rpi_fw,
        rt_uint32_t chan, rt_uint32_t data)
{
    rt_ssize_t res;
    rt_uint32_t message = MBOX_MSG(chan, data);

    rt_hw_spin_lock(&rpi_fw->transaction_lock.lock);

    res = rt_mbox_send(rpi_fw->chan, &message, RT_WAITING_FOREVER);

    if (res >= 0)
    {
        if (rt_completion_wait(&rpi_fw->done, 100))
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ETIMEOUT;
            LOG_W("Firmware transaction timeout");
        }
    }
    else
    {
        LOG_E("Mailbox send message error = %s", rt_strerror(res));
    }

    rt_hw_spin_unlock(&rpi_fw->transaction_lock.lock);

    return res;
}

rt_err_t rpi_firmware_property(struct rpi_firmware *rpi_fw,
        rt_uint32_t tag, void *tag_data, rt_size_t buf_size)
{
    rt_err_t err;
    struct rpi_firmware_property_tag_header *header;
    /*
     * Some mailboxes can use over 1k bytes. Rather than checking
     * size and using stack or kmalloc depending on requirements,
     * just use kmalloc. Mailboxes don't get called enough to worry
     * too much about the time taken in the allocation.
     */
    void *data = rt_malloc(sizeof(*header) + buf_size);

    if (!data)
    {
        return -RT_ENOMEM;
    }

    header = data;
    header->tag = tag;
    header->buf_size = buf_size;
    header->req_resp_size = 0;
    rt_memcpy(data + sizeof(*header), tag_data, buf_size);

    err = rpi_firmware_property_list(rpi_fw, data, buf_size + sizeof(*header));

    rt_memcpy(tag_data, data + sizeof(*header), buf_size);

    rt_free(data);

    return err;
}

rt_err_t rpi_firmware_property_list(struct rpi_firmware *rpi_fw,
        void *data, rt_size_t tag_size)
{
    rt_uint32_t *buf;
    rt_ubase_t bus_addr;
    rt_size_t size = tag_size + 12;
    rt_err_t err;

    /* Packets are processed a dword at a time. */
    if (size & 3)
    {
        return -RT_EINVAL;
    }

    buf = rt_dma_alloc_coherent(rpi_fw->parent.dev,
            RT_ALIGN(size, ARCH_PAGE_SIZE), &bus_addr);

    if (!buf)
    {
        return -RT_ENOMEM;
    }

    buf[0] = size;
    buf[1] = RPI_FIRMWARE_STATUS_REQUEST;
    rt_memcpy(&buf[2], data, tag_size);
    buf[size / 4 - 1] = RPI_FIRMWARE_PROPERTY_END;
    rt_hw_wmb();

    err = rpi_firmware_transaction(rpi_fw, MBOX_CHAN_PROPERTY, bus_addr);

    rt_hw_rmb();
    rt_memcpy(data, &buf[2], tag_size);

    if (err == 0 && buf[1] != RPI_FIRMWARE_STATUS_SUCCESS)
    {
        /*
         * The tag name here might not be the one causing the
         * error, if there were multiple tags in the request.
         * But single-tag is the most common, so go with it.
         */
        LOG_E("Request 0x%08x returned status 0x%08x", buf[2], buf[1]);
        err = -RT_EINVAL;
    }

    rt_dma_free_coherent(rpi_fw->parent.dev, RT_ALIGN(size, ARCH_PAGE_SIZE), buf, bus_addr);

    return err;
}

rt_uint32_t rpi_firmware_clk_get_max_rate(struct rpi_firmware *rpi_fw, rt_uint32_t id)
{
    struct rpi_firmware_clk_rate_request msg = RPI_FIRMWARE_CLK_RATE_REQUEST(id);

    if (rpi_firmware_property(rpi_fw, RPI_FIRMWARE_GET_MAX_CLOCK_RATE,
            &msg, sizeof(msg)))
    {
        return RT_UINT32_MAX;
    }

    return rt_le32_to_cpu(msg.rate);
}

static const struct rt_ofw_node_id rpi_firmware_ofw_ids[];

struct rt_ofw_node *rpi_firmware_find_node(void)
{
    return rt_ofw_find_node_by_ids(RT_NULL, rpi_firmware_ofw_ids);
}

struct rpi_firmware *rpi_firmware_get(struct rt_ofw_node *fw_np)
{
    struct rpi_firmware *rpi_fw = rt_ofw_data(fw_np);

    if (!rpi_fw)
    {
        return RT_NULL;
    }

    if (!ref_get_unless_zero(&rpi_fw->consumers))
    {
        return RT_NULL;
    }

    return rpi_fw;
}

static void rpi_firmware_release(struct ref *ref)
{
    struct rpi_firmware *rpi_fw = rt_container_of(ref, struct rpi_firmware, consumers);

    rt_mbox_free(rpi_fw->chan);
    rt_free(rpi_fw);
}

void rpi_firmware_put(struct rpi_firmware *rpi_fw)
{
    ref_put(&rpi_fw->consumers, rpi_firmware_release);
}

static rt_err_t rpi_firmware_notify_reboot(struct rt_device *dev, char *cmd)
{
    rt_uint32_t reboot_flags = 0;
    struct rpi_firmware *rpi_fw = dev->user_data;

    if (cmd && rt_strstr(cmd, "tryboot"))
    {
        reboot_flags |= 0x1;
    }

    if (reboot_flags)
    {
        rpi_firmware_property(rpi_fw, RPI_FIRMWARE_SET_REBOOT_FLAGS,
                &reboot_flags, sizeof(reboot_flags));
    }

    rpi_firmware_property(rpi_fw, RPI_FIRMWARE_NOTIFY_REBOOT, RT_NULL, 0);

    return RT_EOK;
}

static void rpi_firmware_rx_callback(struct rt_mbox_client *client, void *data)
{
    struct rpi_firmware *rpi_fw = rt_container_of(client, struct rpi_firmware, parent);

    rt_completion_done(&rpi_fw->done);
}

static void rpi_firmware_check_revision_info(struct rpi_firmware *rpi_fw)
{
    struct tm *tm;
    rt_uint32_t revision, variant, hash[5];
    static const char * const variant_strs[] =
    {
        "unknown",
        "start",
        "start_x",
        "start_db",
        "start_cd",
    };
    const char *variant_str = "cmd unsupported";

    if (rpi_firmware_property(rpi_fw, RPI_FIRMWARE_GET_FIRMWARE_REVISION,
            &revision, sizeof(revision)))
    {
        goto _check_hash;
    }

    tm = localtime((time_t *)&revision);

    if (!rpi_firmware_property(rpi_fw, RPI_FIRMWARE_GET_FIRMWARE_VARIANT,
            &variant, sizeof(variant)))
    {
        if (variant >= RT_ARRAY_SIZE(variant_strs))
        {
            variant = 0;
        }

        variant_str = variant_strs[variant];
    }

    LOG_I("Attached to firmware from %04d-%02d-%02d-%02d-%02d-%02d, variant %s",
            tm->tm_year, tm->tm_mon, tm->tm_mday,
            tm->tm_hour, tm->tm_min, tm->tm_sec, variant_str);
    (void)tm;
    (void)variant_str;

_check_hash:
    if (rpi_firmware_property(rpi_fw, RPI_FIRMWARE_GET_FIRMWARE_HASH,
            hash, sizeof(hash)))
    {
        return;
    }

    LOG_I("Firmware hash is %08x%08x%08x%08x%08x",
            hash[0], hash[1], hash[2], hash[3], hash[4]);
}

static rt_err_t rpi_firmware_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    rt_uint32_t packet;
    struct rt_device *dev = &pdev->parent;
    struct rpi_firmware *rpi_fw = rt_calloc(1, sizeof(*rpi_fw));

    if (!rpi_fw)
    {
        return -RT_ENOMEM;
    }

    dev->user_data = rpi_fw;
    rpi_fw->parent.dev = dev;
    rpi_fw->parent.rx_callback = rpi_firmware_rx_callback;

    rpi_fw->chan = rt_mbox_request_by_index(&rpi_fw->parent, 0);

    if (!rpi_fw->chan)
    {
        err = -RT_EINVAL;
        LOG_E("Request mailbox fail");

        goto _fail;
    }

    if ((err = rt_dm_reboot_mode_register(dev, &rpi_firmware_notify_reboot)))
    {
        goto _fail;
    }

    rt_completion_init(&rpi_fw->done);
    ref_init(&rpi_fw->consumers);
    rt_spin_lock_init(&rpi_fw->transaction_lock);

    rt_dm_dev_bind_fwdata(dev, RT_NULL, rpi_fw);

    rpi_firmware_check_revision_info(rpi_fw);

    if (!rpi_firmware_property(rpi_fw, RPI_FIRMWARE_GET_THROTTLED,
            &packet, sizeof(packet)))
    {
        struct rt_platform_device *voltage_monitor;

        if (!(voltage_monitor = rt_calloc(1, sizeof(*voltage_monitor))))
        {
            LOG_E("No memory to create voltage monitor");

            err = -RT_ENOMEM;
            goto _fail;
        }

        voltage_monitor->name = "raspberrypi-voltage-monitor";
        voltage_monitor->priv = dev;

        rt_bus_add_device(pdev->parent.bus, voltage_monitor);
    }

    return RT_EOK;

_fail:
    if (rpi_fw->chan)
    {
        rt_mbox_free(rpi_fw->chan);
    }

    rt_free(rpi_fw);

    return err;
}

static rt_err_t rpi_firmware_shutdown(struct rt_platform_device *pdev)
{
    struct rpi_firmware *rpi_fw = pdev->parent.user_data;

    rpi_firmware_property(rpi_fw, RPI_FIRMWARE_NOTIFY_REBOOT, RT_NULL, 0);

    return RT_EOK;
}

static const struct rt_ofw_node_id rpi_firmware_ofw_ids[] =
{
    { .compatible = "raspberrypi,bcm2835-firmware", },
    { /* sentinel */ }
};

static struct rt_platform_driver rpi_firmware_driver =
{
    .name = "raspberrypi-firmware",
    .ids = rpi_firmware_ofw_ids,

    .probe = rpi_firmware_probe,
    .shutdown = rpi_firmware_shutdown,
};

static int rpi_firmware_drv_register(void)
{
    rt_platform_driver_register(&rpi_firmware_driver);

    return 0;
}
INIT_PLATFORM_EXPORT(rpi_firmware_drv_register);
