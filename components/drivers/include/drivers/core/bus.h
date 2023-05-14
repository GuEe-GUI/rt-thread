/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-12     ErikChan      the first version
 */

#ifndef __BUS_H__
#define __BUS_H__

#include <rthw.h>
#include <drivers/core/device.h>
#include <drivers/core/driver.h>
#include <drivers/core/rtdm.h>

#ifdef RT_USING_FDT
#include <dtb_node.h>
#endif

typedef struct rt_bus *rt_bus_t; 

struct rt_bus
{
    struct rt_object          parent;                   /**< inherit from rt_object */

    char *name;
    struct rt_bus *bus;

    rt_list_t list;
    rt_list_t children;
    rt_list_t dev_list;
    rt_list_t drv_list;

    struct rt_spinlock spinlock;

    rt_bool_t (*match) (rt_driver_t drv, rt_device_t dev);
    rt_err_t (*probe) ();
};

struct rt_bus rt_bus_get_root(void);

rt_err_t rt_bus_init();
rt_err_t rt_bus_register(struct rt_bus *bus);

rt_err_t rt_bus_add(struct rt_bus *bus_node);
rt_err_t rt_bus_add_driver(struct rt_bus *bus, struct rt_driver *drv);
rt_err_t rt_bus_add_device(struct rt_bus *bus, struct rt_device *dev);
rt_err_t rt_bus_remove_driver(struct rt_driver *drv);
rt_err_t rt_bus_remove_device(struct rt_device *dev);

rt_bus_t rt_bus_find_by_name(char *name);
rt_err_t rt_bus_reload_driver_device(struct rt_bus *new_bus, struct rt_device *dev);

#endif /* __BUS_H__ */
