/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-13     ErikChan      the first version
 */

#include <rtthread.h>
#include <virtio_console.h>
#include <virt.h>
#include <drivers/core/bus.h>

#ifdef BSP_USING_VIRTIO_BLK
#include <virtio_blk.h>
#endif
#ifdef BSP_USING_VIRTIO_NET
#include <virtio_net.h>
#endif
#ifdef BSP_USING_VIRTIO_CONSOLE
#include <virtio_console.h>
#endif
#ifdef BSP_USING_VIRTIO_GPU
#include <virtio_gpu.h>
#endif
#ifdef BSP_USING_VIRTIO_INPUT
#include <virtio_input.h>
#endif

static rt_thread_t virtio_thread = RT_NULL;

/* virtio interupt handlers */
static virtio_device_init_handler virtio_device_init_handlers[] =
    {
#ifdef BSP_USING_VIRTIO_BLK
        [VIRTIO_DEVICE_ID_BLOCK] = rt_virtio_blk_init,
#endif
#ifdef BSP_USING_VIRTIO_NET
        [VIRTIO_DEVICE_ID_NET] = rt_virtio_net_init,
#endif
#ifdef BSP_USING_VIRTIO_CONSOLE
        [VIRTIO_DEVICE_ID_CONSOLE] = rt_virtio_console_init,
#endif
#ifdef BSP_USING_VIRTIO_GPU
        [VIRTIO_DEVICE_ID_GPU] = rt_virtio_gpu_init,
#endif
#ifdef BSP_USING_VIRTIO_INPUT
        [VIRTIO_DEVICE_ID_INPUT] = rt_virtio_input_init,
#endif
};

rt_thread_t virtio_thread_self()
{
    return virtio_thread;
}

#ifdef RT_USING_DM

/**
 *  @brief This function match a virtio device/driver
 *
 *  @param drv the driver to be matched
 *
 *  @param dev the device to be matched
 *
 *  @return the error code, RT_TRUE on matcheded successfully.
 */
rt_bool_t virtio_match(struct rt_driver *drv, struct rt_device *dev)
{
    if ((!dev) || (!drv))
    {
        return -RT_EINVAL;
    }

    struct dtb_node *dtb_node = dev->dtb_node;

    /*1、match with dtb_node*/
    if (dtb_node)
    {
        const struct rt_device_id *id = drv->ids;
        const char *compatible = id->compatible;
        while (id->compatible)
        {
            rt_bool_t is_matched = dtb_node_get_dtb_node_compatible_match(dev->dtb_node, compatible);
            if (is_matched)
            {
                return RT_TRUE;
            }
            id++;
        }
    }
    /*2、match with name*/
    else if (dev->name)
    {
        return !strcmp(drv->name, dev->name);
    }

    return RT_FALSE;
}

static struct rt_bus virtio_bus = {
    .name = "virtio",
    .match = virtio_match,
};

/**
 *  @brief This function attach a virtio driver to virtio_bus
 *
 *  @param drv the driver to be attached
 *
 *  @return the error code, RT_TRUE on attached successfully.
 */
rt_err_t virtio_device_attach(struct rt_device *dev)
{
    rt_err_t ret = RT_EOK;

    ret = rt_bus_add_device(&virtio_bus, dev);

    return ret;
}

int virtio_driver_attach(struct rt_driver *drv)
{
    drv->bus = &virtio_bus;
    return 0;
}

int virtio_bus_init(void)
{
    rt_bus_register(&virtio_bus);
    return 0;
}

INIT_BUS_EXPORT(virtio_bus_init);

/**
 *  @brief This function create a virtio device
 *
 *  @param device_id the id of virtio device
 *
 *  @return device, a virtio device created
 */
struct rt_device *virtio_device_create(struct rt_device *dev)
{
    struct rt_bus *virtio_bus;
    rt_device_t device;

    virtio_bus = rt_bus_find_by_name("virtio");

    if (!virtio_bus)
    {
        return RT_NULL;
    }

    device = rt_device_find("virtio-console0");

    struct hw_virtio_device *virtio_device_priv = (struct hw_virtio_device *)(dev->user_data);
    rt_uint32_t device_id = virtio_device_priv->id;

#ifdef RT_USING_FDT
    device->dtb_node = dev->dtb_node;
#endif

    device->user_data = virtio_device_priv;

    switch (device_id)
    {
    case VIRTIO_DEVICE_ID_CONSOLE:
    {
        struct virtio_console_device *virtio_console_dev = rt_malloc(sizeof(struct virtio_console_device));
        if (virtio_console_dev == RT_NULL)
        {
            rt_kprintf("malloc virtio_console_dev failed...\n");
            return RT_NULL;
        }
        virtio_console_dev->parent = *device;
        device->priv = virtio_console_dev;
    }
    break;
    default:
        break;
    }

    if (device != RT_NULL)
    {
        rt_bus_add_device(virtio_bus, device);
        rt_device_close(device);
    }

    return device;
}

struct rt_virtio_ops virtio_ops =
    {
        .device_create = virtio_device_create,
        .device_destory = NULL,
};

/* init an ampty hw_virtio_device data */
static struct hw_virtio_device virtio_priv = {

};

static void virtio_create_console_entry(void *parameter)
{
    struct rt_device *dev = (struct rt_device *)parameter;
    struct rt_virtio_ops *ops = ((struct rt_virtio_driver *)(dev->drv))->ops;

    while (RT_TRUE)
    {
        rt_thread_suspend(rt_thread_self());
        rt_schedule();

        ops->device_create(dev);
    }
}

int virtio_probe(struct rt_device *dev)
{
    virtio_device_init_handler init_handler;
    struct virtio_mmio_config *mmio_config;

#ifdef RT_USING_FDT
    struct dtb_node *node = (struct dtb_node *)(dev->dtb_node);
    if (node)
    {
        size_t uart_reg_range = 0;
        void *uart_reg_addr = (void *)dtb_node_get_addr_size(node, "reg", &uart_reg_range);

        rt_ubase_t hw_base;
        if ((uart_reg_addr) && (uart_reg_range != 0))
        {
            hw_base = (rt_base_t)rt_ioremap(uart_reg_addr, uart_reg_range);
        }

        rt_uint32_t irqno = dtb_node_irq_get(node, 0) + 32;
        mmio_config = (struct virtio_mmio_config *)hw_base;

        if (mmio_config->magic != VIRTIO_MAGIC_VALUE ||
            mmio_config->version != RT_USING_VIRTIO_VERSION ||
            mmio_config->vendor_id != VIRTIO_VENDOR_ID)
        {
            return 0;
        }

        init_handler = virtio_device_init_handlers[mmio_config->device_id];

        if (mmio_config->device_id == VIRTIO_DEVICE_ID_CONSOLE)
        {
            virtio_thread = rt_thread_create("virtio",
                                             virtio_create_console_entry, dev, 4096, 10, 20);
        }

        if (init_handler != RT_NULL)
        {
            virtio_priv.id = mmio_config->device_id;
            dev->user_data = &virtio_priv;

            init_handler((rt_ubase_t *)hw_base, irqno);
        }
    }
#endif

    return 0;
}

rt_err_t rt_virtio_driver_register(struct rt_virtio_driver *drv)
{
    return rt_driver_register(&drv->parent);
}

struct rt_device_id virtio_ids[] =
    {
        {.compatible = "virtio,mmio"},
        {/* sentinel */}};

static struct rt_virtio_driver virtio_drv = {
    .parent = {
        .name = "virtio",
        .probe = virtio_probe,
        .ids = virtio_ids,
    },
    .ops = &virtio_ops,
};

VIRTIO_DRIVER_EXPORT(virtio_drv);

#endif
