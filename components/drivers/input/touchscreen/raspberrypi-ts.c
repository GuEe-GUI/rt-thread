/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-26     GuEe-GUI     first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "input.touchscreen.rpi-ts"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "../../firmware/raspberrypi/firmware.h"

#define RPI_TS_DEFAULT_WIDTH            800
#define RPI_TS_DEFAULT_HEIGHT           480

#define RPI_TS_MAX_SUPPORTED_POINTS     10

#define RPI_TS_FTS_TOUCH_DOWN           0
#define RPI_TS_FTS_TOUCH_CONTACT        2

#define RPI_TS_POLL_INTERVAL            17  /* 60fps */

#define RPI_TS_NPOINTS_REG_INVALIDATE   99

struct rpi_ts_regs
{
    rt_uint8_t device_mode;
    rt_uint8_t gesture_id;
    rt_uint8_t num_points;

    struct rpi_ts_touch
    {
        rt_uint8_t xh;
        rt_uint8_t xl;
        rt_uint8_t yh;
        rt_uint8_t yl;
        rt_uint8_t pressure;   /* Not supported */
        rt_uint8_t area;       /* Not supported */
    } point[RPI_TS_MAX_SUPPORTED_POINTS];
};

struct rpi_ts
{
    struct rt_touch_device parent;

    void *fw_regs;
    rt_ubase_t fw_regs_phys;

    int known_ids;
    struct rpi_ts_regs ts_regs;

    struct rt_timer poll_work;
};

#define raw_to_rpi_ts(raw) rt_container_of(raw, struct rpi_ts, parent)

static rt_size_t rpi_ts_touch_readpoint(struct rt_touch_device *touch, void *buf, rt_size_t touch_num)
{
    int touchid;
    int event_type;
    int modified_ids = 0;
    bitmap_t released_ids;
    int x, y, bit, points_report = 0;
    struct rt_touch_data *point = buf;
    struct rpi_ts *rts = raw_to_rpi_ts(touch);
    struct rpi_ts_regs *regs = &rts->ts_regs;

    for (int i = 0; i < regs->num_points; ++i)
    {
        x = (((int)regs->point[i].xh & 0xf) << 8) + regs->point[i].xl;
        y = (((int)regs->point[i].yh & 0xf) << 8) + regs->point[i].yl;
        touchid = (regs->point[i].yh >> 4) & 0xf;
        event_type = (regs->point[i].xh >> 6) & 0x03;

        modified_ids |= RT_BIT(touchid);

        if (event_type == RPI_TS_FTS_TOUCH_DOWN ||
            event_type == RPI_TS_FTS_TOUCH_CONTACT)
        {
            if (event_type == RPI_TS_FTS_TOUCH_DOWN)
            {
                point->event = RT_TOUCH_EVENT_DOWN;
            }
            else
            {
                point->event = RT_TOUCH_EVENT_MOVE;
            }

            point->track_id = touchid;
            point->width = 1;
            point->x_coordinate = x;
            point->y_coordinate = y;
            point->timestamp = rt_touch_get_ts();

            ++point;
            ++points_report;
        }
    }

    released_ids = rts->known_ids & ~modified_ids;
    bitmap_for_each_set_bit(&released_ids, bit, RPI_TS_MAX_SUPPORTED_POINTS)
    {
        if (rt_unlikely(points_report >= touch_num))
        {
            break;
        }

        modified_ids &= ~RT_BIT(bit);

        point->event = RT_TOUCH_EVENT_UP;
        point->track_id = bit;
        point->width = 1;
        point->x_coordinate = 0;
        point->y_coordinate = 0;
        point->timestamp = rt_touch_get_ts();

        ++point;
        ++points_report;
    }

    rts->known_ids = modified_ids;

    return points_report;
}

static rt_err_t rpi_ts_touch_control(struct rt_touch_device *touch, int cmd, void *arg)
{
    rt_err_t err = RT_EOK;
    struct rpi_ts *rts = raw_to_rpi_ts(touch);

    if (!arg)
    {
        return -RT_EINVAL;
    }

    switch (cmd)
    {
    case RT_TOUCH_CTRL_GET_ID:
        *(int *)arg = 0;
        break;

    case RT_TOUCH_CTRL_GET_INFO:
        rt_memcpy(arg, &rts->parent.info, sizeof(rts->parent.info));
        break;

    case RT_TOUCH_CTRL_DISABLE_INT:
        rt_timer_stop(&rts->poll_work);
        break;

    case RT_TOUCH_CTRL_ENABLE_INT:
        rt_timer_start(&rts->poll_work);
        break;

    default:
        err = -RT_ENOSYS;
        break;
    }

    return err;
}

const static struct rt_touch_ops rpi_ts_ops =
{
    .touch_readpoint = rpi_ts_touch_readpoint,
    .touch_control = rpi_ts_touch_control,
};

static void rpi_ts_poll(void *param)
{
    struct rpi_ts *rts = param;
    struct rpi_ts_regs *regs = &rts->ts_regs;

    rt_memcpy(regs, rts->fw_regs, sizeof(*regs));

    /*
     * We poll the memory based register copy of the touchscreen chip using
     * the number of points register to know whether the copy has been
     * updated (we write 99 to the memory copy, the GPU will write between
     * 0 - 10 points)
     */
    HWREG8(rts->fw_regs + (rt_size_t)&((struct rpi_ts_regs *)0)->num_points) =
            RPI_TS_NPOINTS_REG_INVALIDATE;

    if (regs->num_points == RPI_TS_NPOINTS_REG_INVALIDATE ||
        (regs->num_points == 0 && rts->known_ids == 0))
    {
        return;
    }

    rt_hw_touch_isr(&rts->parent);
}

static rt_err_t rpi_ts_probe(struct rt_platform_device *pdev)
{
    rt_err_t err = RT_EOK;
    const char *dev_name;
    rt_uint32_t touchbuf, tsx, tsy;
    struct rpi_firmware *rpi_fw = RT_NULL;
    struct rt_device *dev = &pdev->parent;
    struct rt_ofw_node *np = dev->ofw_node, *fw_np;
    struct rpi_ts *rts = rt_calloc(1, sizeof(*rts));

    if (!rts)
    {
        return -RT_ENOMEM;
    }

    fw_np = rt_ofw_parse_phandle(np, "firmware", 0);

    if (!fw_np)
    {
        err = -RT_EINVAL;
        goto _fail;
    }

    rpi_fw = rpi_firmware_get(fw_np);
    rt_ofw_node_put(fw_np);

    if (!rpi_fw)
    {
        err = -RT_EINVAL;
        goto _fail;
    }

    rts->fw_regs = rt_dma_alloc_coherent(dev, ARCH_PAGE_SIZE, &rts->fw_regs_phys);

    if (!rts->fw_regs)
    {
        LOG_E("Failed to dma buffer");

        goto _fail;
    }

    if (rts->fw_regs_phys > RT_UINT32_MAX)
    {
        LOG_E("64 bits dma buffer is not support");

        goto _fail;
    }

    touchbuf = (rt_uint32_t)rts->fw_regs_phys;

    if ((err = rpi_firmware_property(rpi_fw, RPI_FIRMWARE_FRAMEBUFFER_SET_TOUCHBUF,
            &touchbuf, sizeof(touchbuf))))
    {
        LOG_E("Set touchbuf from firmware error = %s", rt_strerror(err));

        goto _fail;
    }

    if (rt_ofw_prop_read_u32(np, "touchscreen-size-x", &tsx))
    {
        tsx = RPI_TS_DEFAULT_WIDTH;
    }

    if (rt_ofw_prop_read_u32(np, "touchscreen-size-y", &tsy))
    {
        tsy = RPI_TS_DEFAULT_HEIGHT;
    }

    rt_dm_dev_bind_fwdata(dev, RT_NULL, &rts->parent);
    dev->user_data = rts;

    rts->parent.info.type = RT_TOUCH_TYPE_CAPACITANCE;
    rts->parent.info.vendor = RT_TOUCH_VENDOR_UNKNOWN;
    rts->parent.info.point_num = RPI_TS_MAX_SUPPORTED_POINTS;
    rts->parent.info.range_x = tsx;
    rts->parent.info.range_y = tsy;
    rts->parent.ops = &rpi_ts_ops;

    rt_dm_dev_set_name_auto(&rts->parent.parent, "touch");
    dev_name = rt_dm_dev_get_name(&rts->parent.parent);

    rt_timer_init(&rts->poll_work, dev_name, rpi_ts_poll, rts,
            rt_tick_from_millisecond(RPI_TS_POLL_INTERVAL), RT_TIMER_FLAG_PERIODIC);

    rt_hw_touch_register(&rts->parent, dev_name, RT_DEVICE_FLAG_INT_RX, rts);

    return RT_EOK;

_fail:
    if (rts->fw_regs)
    {
        rt_dma_free_coherent(dev, ARCH_PAGE_SIZE, rts->fw_regs, rts->fw_regs_phys);
    }

    if (rpi_fw)
    {
        rpi_firmware_put(rpi_fw);
    }

    rt_free(rts);

    return err;
}

static rt_err_t rpi_ts_remove(struct rt_platform_device *pdev)
{
    struct rt_device *dev = &pdev->parent;
    struct rpi_ts *rts = dev->user_data;

    rt_timer_stop(&rts->poll_work);
    rt_timer_detach(&rts->poll_work);

    rt_dm_dev_unbind_fwdata(dev, RT_NULL);

    rt_dma_free_coherent(dev, ARCH_PAGE_SIZE, rts->fw_regs, rts->fw_regs_phys);

    rt_device_unregister(&rts->parent.parent);

    rt_free(rts);

    return RT_EOK;
}

static const struct rt_ofw_node_id rpi_ts_ofw_ids[] =
{
    { .compatible = "raspberrypi,firmware-ts" },
    { /* sentinel */ }
};

static struct rt_platform_driver rpi_ts_driver =
{
    .name = "touchscreen-rpi",
    .ids = rpi_ts_ofw_ids,

    .probe = rpi_ts_probe,
    .remove = rpi_ts_remove,
};
RT_PLATFORM_DRIVER_EXPORT(rpi_ts_driver);
