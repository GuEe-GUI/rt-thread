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
#include <rtdevice.h>

#define DBG_TAG "mfd.bcm2835-pm"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "bcm2835-pm.h"

static rt_err_t bcm2835_pm_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    const char *pm_name;
    rt_bool_t is_2712 = RT_FALSE;
    struct rt_device *dev = &pdev->parent;
    struct bcm2835_pm *pm = rt_malloc(sizeof(*pm));

    if (!pm)
    {
        return -RT_ENOMEM;
    }

#ifdef RT_USING_OFW
    is_2712 = rt_ofw_node_is_compatible(dev->ofw_node, "brcm,bcm2712-pm");

    if (rt_ofw_prop_read_bool(dev->ofw_node, "reg-names"))
    {
        if (!(pm->base = rt_dm_dev_iomap_by_name(dev, "pm")))
        {
            err = -RT_EIO;

            goto _fail;
        }

        pm->asb = rt_dm_dev_iomap_by_name(dev, "asb");
        pm->rpivid_asb = rt_dm_dev_iomap_by_name(dev, "rpivid_asb");
    }
    else
#endif /* RT_USING_OFW */
    {
        if (!(pm->base = rt_dm_dev_iomap(dev, 0)))
        {
            err = -RT_EIO;

            goto _fail;
        }

        pm->asb = rt_dm_dev_iomap(dev, 1);
        pm->rpivid_asb = rt_dm_dev_iomap(dev, 2);
    }

    pm->ofw_node = dev->ofw_node;

    if (pm->asb || is_2712)
    {
        pm_name = "bcm2835-power";
    }
    else
    {
        pm_name = "bcm2835-wdt";
    }

    pdev->name = pm_name;
    pdev->priv = pm;
    /* Ask bus to check drivers' name */
    pdev->parent.ofw_node = RT_NULL;

    return rt_bus_add_device(pdev->parent.bus, &pdev->parent);

_fail:
    rt_free(pm);

    return err;
}

static const struct rt_ofw_node_id bcm2835_pm_ofw_ids[] =
{
    { .compatible = "brcm,bcm2835-pm-wdt", },
    { .compatible = "brcm,bcm2835-pm", },
    { .compatible = "brcm,bcm2711-pm", },
    { .compatible = "brcm,bcm2712-pm", },
    { /* sentinel */ }
};

static struct rt_platform_driver bcm2835_pm_driver =
{
    .name = "pm-bcm2835",
    .ids = bcm2835_pm_ofw_ids,

    .probe = bcm2835_pm_probe,
};
RT_PLATFORM_DRIVER_EXPORT(bcm2835_pm_driver);
