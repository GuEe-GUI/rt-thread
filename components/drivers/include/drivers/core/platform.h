/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-12     ErikChan      the first version
 */

#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include <drivers/core/rtdm.h>

#ifdef RT_USING_FDT
#include <dtb_node.h>
#endif

typedef struct rt_platform_device *rt_pdevice_t;

struct rt_platform_device {
    const char *name;
    struct rt_device dev;
    // TODO: add other properties such as resource, id_entry, etc...
};

#ifdef RT_USING_FDT

rt_err_t rt_platform_devices_init_with_dtb(struct dtb_node *device_root_node);

rt_pdevice_t rt_pdevice_create(struct dtb_node *node);

#endif

rt_bool_t rt_platform_match(struct rt_driver *drv, struct rt_device *dev);

rt_err_t rt_platform_driver_register(rt_driver_t drv);

rt_err_t rt_timer_driver_register(rt_driver_t drv);

#define PLATFORM_DRIVER_EXPORT(driver)  RT_DRIVER_EXPORT(driver, platform, BUILIN)

#define TIMER_DRIVER_EXPORT(driver)  RT_DRIVER_EXPORT(driver, platform, BUILIN)

#endif /* __PLATFORM_H__ */
