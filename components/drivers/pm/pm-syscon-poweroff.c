/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     the first version
 */

#include <rtthread.h>

#define DBG_TAG "pm.syscon.poweroff"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <drivers/syscon.h>

static struct rt_syscon *syscon;
static rt_uint32_t offset, value, mask;

static void syscon_poweroff(void)
{
    /* Issue the poweroff */
    rt_syscon_update_bits(syscon, offset, mask, value);

    rt_thread_mdelay(1000);

    LOG_E("Unable to poweroff system");
}

static rt_err_t syscon_poweroff_probe(struct rt_platform_device *pdev)
{
    rt_err_t mask_err, value_err;
    struct rt_ofw_node *np = pdev->parent.ofw_node;

    syscon = rt_syscon_find_by_phandle(np, "regmap");

    if (!syscon)
    {
        return -RT_ERROR;
    }

    if (rt_ofw_prop_read_u32(np, "offset", &offset))
    {
        LOG_E("read 'offset' fail");

        return -RT_EINVAL;
    }

    value_err = rt_ofw_prop_read_u32(np, "value", &value);
    mask_err = rt_ofw_prop_read_u32(np, "mask", &mask);

    if (value_err && mask_err)
    {
        LOG_E("read 'value' and 'mask' fail");

        return -RT_EINVAL;
    }

    if (value_err)
    {
        /* support old binding */
        value = mask;
        mask = 0xffffffff;
    }
    else if (mask_err)
    {
        /* support value without mask */
        mask = 0xffffffff;
    }

    if (rt_pm_shutdown)
    {
        LOG_E("rt_pm_shutdown have hook %p", rt_pm_shutdown);

        return -RT_EBUSY;
    }

    rt_pm_shutdown = syscon_poweroff;

    return RT_EOK;
}

static const struct rt_ofw_node_id syscon_poweroff_ofw_ids[] =
{
    { .compatible = "syscon-poweroff" },
    { /* sentinel */ }
};

static struct rt_platform_driver syscon_poweroff_driver =
{
    .name = "pm-syscon-poweroff",
    .ids = syscon_poweroff_ofw_ids,

    .probe = syscon_poweroff_probe,
};
RT_PLATFORM_DRIVER_EXPORT(syscon_poweroff_driver);
