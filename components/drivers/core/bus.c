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
#include <drivers/core/bus.h>

/*init the root bus*/
static struct rt_bus bus_root =
{
        .name = "root",
        .bus = NULL,
        .children = RT_LIST_OBJECT_INIT(bus_root.children),
};

/**
 * @brief This function get the root bus
 */
struct rt_bus rt_bus_get_root(void)
{
    return bus_root;
}

/**
 *  @brief This function match the device and driver, probe them if match successed
 *
 *  @param drv the drv to match/probe
 *
 *  @param dev the dev to match/probe
 *
 *  @return the result of probe, 1 on added successfully.
 */
static int rt_bus_probe(struct rt_driver *drv, struct rt_device *dev)
{
    int ret = 0;
    
    struct rt_bus *bus = drv->bus;

    if(!bus)
    {
        bus = dev->bus;
    }

    RT_ASSERT(bus != RT_NULL);

    if (bus->match(drv, dev))
    {
        ret = rt_driver_probe_device(drv, dev);
    }

    return ret;
}

/**
 *  @brief This function loop the dev_list of the bus, and call fn in each loop
 *
 *  @param bus the target bus
 *
 *  @param drv the target drv to be matched
 *
 *  @param fn  the function callback in each loop
 *
 *  @return the error code, RT_EOK on added successfully.
 */
rt_err_t rt_bus_for_each_dev(struct rt_bus *bus, struct rt_driver *drv,
                             int (*fn)(struct rt_driver *drv, struct rt_device *dev))
{
    struct rt_device *dev;
    rt_base_t lever;

    RT_ASSERT(bus != RT_NULL);
    RT_ASSERT(drv != RT_NULL);

    lever = rt_spin_lock_irqsave(&bus->spinlock);
    rt_list_for_each_entry(dev, &(bus->dev_list), node)
    {
        fn(drv, dev);
    }
    rt_spin_unlock_irqrestore(&bus->spinlock, lever);
    
    return RT_EOK;
}

/**
 *  @brief This function loop the drv_list of the bus, and call fn in each loop
 *
 *  @param bus the target bus
 *
 *  @param dev the target dev to be matched
 *
 *  @param fn  the function callback in each loop
 *
 *  @return the error code, RT_EOK on added successfully.
 */
rt_err_t rt_bus_for_each_drv(struct rt_bus *bus, struct rt_device *dev,
                                int (*fn)(struct rt_driver *drv, struct rt_device *dev))
{
    struct rt_driver *drv;
    rt_base_t lever;

    RT_ASSERT(bus != RT_NULL);
    RT_ASSERT(dev != RT_NULL);

    if (!rt_list_len(&(bus->drv_list)))
    {
        return RT_EOK;
    }

    lever = rt_spin_lock_irqsave(&bus->spinlock);
    rt_list_for_each_entry(drv, &(bus->drv_list), node)
    {
        if (fn(drv, dev))
        {
            return RT_EOK;
        }
    }
    rt_spin_unlock_irqrestore(&bus->spinlock, lever);

    return RT_ERROR;
}

/**
 *  @brief This function transfer dev_list and drv_list to the other bus
 *
 *  @param new_bus the bus to transfer
 *
 *  @param dev the target device
 *
 *  @return the error code, RT_EOK on added successfully.
 */
rt_err_t rt_bus_reload_driver_device(struct rt_bus *new_bus, struct rt_device *dev)
{
    rt_base_t lever;

    RT_ASSERT(new_bus != RT_NULL);
    RT_ASSERT(dev != RT_NULL);

    lever = rt_spin_lock_irqsave(&new_bus->spinlock);

    rt_list_remove(&(dev->node));
    rt_list_insert_after(&(new_bus->dev_list), &(dev->node));

#ifdef RT_USING_DM
    rt_list_remove(&(dev->drv->node));
    rt_list_insert_after(&(new_bus->drv_list), &(dev->drv->node));
#endif

    rt_spin_unlock_irqrestore(&new_bus->spinlock, lever);

    return RT_EOK;
}

/**
 *  @brief This function add a bus to the root
 *
 *  @param bus_node the bus to be added
 *
 *  @return the error code, RT_EOK on added successfully.
 */
rt_err_t rt_bus_add(struct rt_bus *bus_node)
{
    rt_base_t lever;

    RT_ASSERT(bus_node != RT_NULL);

    bus_node->bus = &bus_root;

    rt_list_init(&bus_node->list);

    lever = rt_spin_lock_irqsave(&bus_node->spinlock);
    rt_list_insert_after(&bus_root.children, &bus_node->list);
    rt_spin_unlock_irqrestore(&bus_node->spinlock, lever);

    return RT_EOK;
}

/**
 *  @brief This function add a driver to the drv_list of a specific bus
 *
 *  @param bus the bus to add
 *
 *  @param drv the driver to be added
 *
 *  @return the error code, RT_EOK on added successfully.
 */
rt_err_t rt_bus_add_driver(struct rt_bus *bus, struct rt_driver *drv)
{
    struct dtb_node *device_root_node;
    const struct rt_device_id *id;

    rt_base_t lever;

    RT_ASSERT(bus != RT_NULL);
    RT_ASSERT(drv != RT_NULL);

    id = drv->ids;
    device_root_node = get_dtb_node_head();

    /* loop the compatibles and decide which bus to depend*/
    while (id->compatible)
    {
        const char *compatible = id->compatible;
        struct dtb_node *dt_node = dtb_node_find_compatible_node(device_root_node, compatible);

        /* add driver to amba bus if compatible satisfied with `arm,primecell` */
        if (dtb_node_get_dtb_node_compatible_match(dt_node, "arm,primecell"))
        {
            bus = rt_bus_find_by_name("amba");
            break;
        }

        id++;
    }

    drv->bus = bus;

    lever = rt_spin_lock_irqsave(&bus->spinlock);
    rt_list_insert_after(&(bus->drv_list), &(drv->node));
    rt_spin_unlock_irqrestore(&bus->spinlock, lever);

    rt_bus_for_each_dev(drv->bus, drv, rt_bus_probe);

    return RT_EOK;
}

/**
 *  @brief This function add a device to the dev_list of a specific bus
 *
 *  @param bus the bus to add
 *
 *  @param dev the device to be added
 *
 *  @return the error code, RT_EOK on added successfully.
 */
rt_err_t rt_bus_add_device(struct rt_bus *bus, struct rt_device *dev)
{
    rt_base_t lever;

    RT_ASSERT(bus != RT_NULL);
    RT_ASSERT(dev != RT_NULL);

    dev->bus = bus;

    lever = rt_spin_lock_irqsave(&bus->spinlock);
    rt_list_insert_before(&(bus->dev_list), &(dev->node));
    rt_spin_unlock_irqrestore(&bus->spinlock, lever);
    
    rt_bus_for_each_drv(dev->bus, dev, rt_bus_probe);

    return RT_EOK;
}

/**
 *  @brief This function remove a driver from bus
 *
 *  @param drv the driver to be removed
 *
 *  @return the error code, RT_EOK on added successfully.
 */
rt_err_t rt_bus_remove_driver(struct rt_driver *drv)
{
    RT_ASSERT(drv->bus != RT_NULL);

    rt_kprintf("bus: '%s': remove driver %s\n", drv->bus->name, drv->name);
    rt_list_remove(&(drv->node));

    return RT_EOK;
}

/**
 *  @brief This function remove a device from bus
 *
 *  @param dev the device to be removed
 *
 *  @return the error code, RT_EOK on added successfully.
 */
rt_err_t rt_bus_remove_device(struct rt_device *dev)
{
    RT_ASSERT(dev->bus != RT_NULL);

    rt_kprintf("bus: '%s': remove device %s\n", dev->bus->name, dev->name);
    rt_list_remove(&(dev->node));

    return RT_EOK;
}

/**
 *  @brief This function find a bus by name
 *  @param bus the name to be finded
 *
 *  @return the bus finded by name.
 */
rt_bus_t rt_bus_find_by_name(char *name)
{
    struct rt_bus *bus = RT_NULL;
    struct rt_list_node *node = RT_NULL;

    if (!rt_list_isempty(&bus_root.children))
    {
        rt_list_for_each(node, &bus_root.children)
        {
            bus = rt_list_entry(node, struct rt_bus, list);
            if (rt_strncmp(bus->name, name, RT_NAME_MAX) == 0)
            {
                return bus;
            }
        }
    }

    return bus;
}

/**
 *  @brief This function register a bus
 *  @param bus the bus to be registered
 *
 *  @return the error code, RT_EOK on registeration successfully.
 */
rt_err_t rt_bus_register(struct rt_bus *bus)
{
    rt_list_init(&(bus->children));
    rt_list_init(&(bus->dev_list));
    rt_list_init(&(bus->drv_list));
    rt_bus_add(bus);
    return RT_EOK;
}

