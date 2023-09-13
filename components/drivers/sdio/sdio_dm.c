/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-12-06     GuEe-GUI     first version
 */

#include <rtatomic.h>
#include "sdio_dm.h"

#define DBG_TAG "sdio.dm"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

int sdio_host_set_name(struct rt_mmcsd_host *host, char *out_devname)
{
    static volatile rt_atomic_t uid = 0;

    if (host)
    {
        int id = (int)rt_atomic_add(&uid, 1), res;

        res = rt_snprintf(host->name, RT_NAME_MAX, "sd%u", id);

        if (out_devname)
        {
            rt_strncpy(out_devname, host->name, RT_NAME_MAX);
        }

        return res;
    }

    return -RT_EINVAL;
}

#ifdef RT_USING_OFW
rt_err_t sdio_ofw_parse(struct rt_ofw_node *dev_np, struct rt_mmcsd_host *host)
{
    rt_uint32_t bus_width = 1;

    if (!dev_np)
    {
        return -RT_EINVAL;
    }

    host->flags = MMCSD_MUTBLKWRITE;
    rt_ofw_prop_read_u32(dev_np, "bus-width", &bus_width);

    switch (bus_width)
    {
    case 0x8:
        host->flags |= MMCSD_BUSWIDTH_8;
        break;

    case 0x4:
        host->flags |= MMCSD_BUSWIDTH_4;
        break;

    case 0x1:
        break;

    default:
        LOG_E("Invalid \"bus-width\" value %d", bus_width);
        return -RT_EIO;
    }

    rt_ofw_prop_read_u32(dev_np, "max-frequency", &host->freq_max);

    if (rt_ofw_prop_read_bool(dev_np, "non-removable"))
    {
        host->flags |= MMCSD_SUP_NONREMOVABLE;
    }

    if (rt_ofw_prop_read_bool(dev_np, "cap-sdio-irq"))
    {
        host->flags |= MMCSD_SUP_SDIO_IRQ;
    }

    if (rt_ofw_prop_read_bool(dev_np, "cap-sd-highspeed") ||
        rt_ofw_prop_read_bool(dev_np, "cap-mmc-highspeed"))
    {
        host->flags |= MMCSD_SUP_HIGHSPEED;
    }

    if (rt_ofw_prop_read_bool(dev_np, "mmc-ddr-3_3v"))
    {
        host->flags |= MMCSD_SUP_DDR_3V3;
    }
    if (rt_ofw_prop_read_bool(dev_np, "mmc-ddr-1_8v"))
    {
        host->flags |= MMCSD_SUP_DDR_1V8;
    }
    if (rt_ofw_prop_read_bool(dev_np, "mmc-ddr-1_2v"))
    {
        host->flags |= MMCSD_SUP_DDR_1V2;
    }

    if (rt_ofw_prop_read_bool(dev_np, "mmc-hs200-1_2v"))
    {
        host->flags |= MMCSD_SUP_HS200_1V2;
    }
    if (rt_ofw_prop_read_bool(dev_np, "mmc-hs200-1_8v"))
    {
        host->flags |= MMCSD_SUP_HS200_1V8;
    }

    return RT_EOK;
}
#endif /* RT_USING_OFW */
