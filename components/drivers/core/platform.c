/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-12     ErikChan      the first version
 */

#include <rtthread.h>
#include <rtdef.h>
#include <drivers/core/bus.h>
#include <drivers/core/platform.h>
#include <drivers/amba_bus.h>

static struct rt_bus platform_bus = {
    .name = "platform",
    .match = rt_platform_match,
};

static struct rt_device_id skipped_node_table[] =
    {
        {/* sentinel */}};

/**
 *  @brief This function create a platform device
 *
 *  @param node the device node from dtb
 *
 *  @return a new platform device
 */
rt_pdevice_t rt_pdevice_create(struct dtb_node *node)
{
    struct rt_platform_device *pdev;

    RT_ASSERT(node != RT_NULL);

    pdev = (rt_pdevice_t)rt_malloc(sizeof(struct rt_platform_device));
    pdev->dev.dtb_node = node;
    pdev->name = node->name;

    return pdev;
}

/**
 *  @brief This function check if the id of dtb_node existed in id_table
 *
 *  @param id the id table to check
 *
 *  @param dtb_node the device node to be matched
 * 
 *  @return the error code, RT_TRUE on matcheded successfully.
 */
static rt_bool_t rt_match_node(const struct rt_device_id *id, struct dtb_node *dtb_node)
{
    const char *compatible = id->compatible;
    while (id->compatible)
    {
        rt_bool_t is_matched = dtb_node_get_dtb_node_compatible_match(dtb_node, compatible);
        if (is_matched)
            return RT_TRUE;
        id++;
    }
    return RT_FALSE;
}

/**
 *  @brief This function match a platform device/driver
 *
 *  @param drv the driver to be matched
 *
 *  @param dev the device to be matched
 *
 *  @return the error code, RT_TRUE on matcheded successfully.
 */
rt_bool_t rt_platform_match(struct rt_driver *drv, struct rt_device *dev)
{
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(drv != RT_NULL);

    struct dtb_node *dtb_node = dev->dtb_node;
    /*1、match with dtb_node*/
    if (dtb_node)
    {
        const struct rt_device_id *id = drv->ids;
        return rt_match_node(id, dtb_node);
    }
    /*2、match with name*/
    else if (dev->name)
    {
        return !strcmp(drv->name, dev->name);
    }
    return RT_FALSE;
}

/**
 *  @brief This function create a platform device
 *
 *  @param node the dtb_node for a new device
 *
 *  @return the error code, RT_TRUE on matcheded successfully.
 */
static rt_err_t rt_platform_device_create(struct dtb_node *node)
{
    rt_err_t ret = RT_EOK;
    struct rt_platform_device *pdev;

    RT_ASSERT(node != RT_NULL);

    /* Skip nodes for which not contain property -- compatible */
    if(!dtb_node_get_property(node, "compatible", NULL))
    {
        return ret;
    }

    /* Skip nodes for which we don't want to create devices */
    if (rt_match_node(skipped_node_table, node))
    {
        rt_kprintf("%s() - skipping device node [%s]\n", __func__, node->name);
        return ret;
    }

    /* new amba device */
    if (dtb_node_get_dtb_node_compatible_match(node, "arm,primecell"))
    {
        struct rt_bus *amba_bus;

        amba_bus = rt_bus_find_by_name("amba");
        struct rt_amba_device *amba_device = rt_amba_device_create(node);

        if (amba_bus)
        {
            ret = rt_bus_add_device(amba_bus, &amba_device->dev);
        }
    }
    /* new platform devices */
    else
    {
        pdev =  rt_pdevice_create(node);
        ret = rt_bus_add_device(&platform_bus, &pdev->dev);
    }

    return ret;
}

/**
 *  @brief This function create platform devices by device nodes
 *
 *  @param node the root node for device tree
 *
 *  @return the error code, RT_TRUE on matcheded successfully.
 */
rt_err_t rt_platform_devices_init_with_dtb(struct dtb_node *device_root_node)
{
    struct dtb_node *node;

    RT_ASSERT(device_root_node != RT_NULL);

    node = (struct dtb_node *)device_root_node->child;
    while (node)
    {
        rt_platform_device_create(node);
        node = node->sibling;
    }
    return RT_EOK;
}
RTM_EXPORT(rt_devices_init_with_dtb);

int dbt_unflattern(void)
{
    struct dtb_node *device_root_node = get_dtb_node_head();
    if (device_root_node)
    {
        rt_platform_devices_init_with_dtb(device_root_node);
    }
    return 0;
}
INIT_BOARD_EXPORT(dbt_unflattern);

rt_err_t rt_platform_driver_register(rt_driver_t drv)
{
    return rt_driver_register(drv);
}

rt_err_t rt_timer_driver_register(rt_driver_t drv)
{
    return rt_driver_register(drv);
}

int platform_bus_init(void)
{
    rt_bus_register(&platform_bus);
    return 0;
}

INIT_BUS_EXPORT(platform_bus_init);
