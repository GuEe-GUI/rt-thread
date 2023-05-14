/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-12     ErikChan      the first version
 */

#ifndef __AMBA_BUS_H__
#define __AMBA_BUS_H__

#include <rtthread.h>
#include <rtdef.h>
#include <rtconfig.h>
#include <drivers/core/rtdm.h>
#include <rtdevice.h>

#ifdef RT_USING_FDT
#include "dtb_node.h"
#endif

typedef struct rt_amba_device *rt_amba_dev_t;

struct rt_amba_device {
    const char *name;
    struct rt_device dev;
};

rt_amba_dev_t rt_amba_device_create(struct dtb_node *node);

rt_err_t rt_amba_driver_register(void *drv);

#define AMBA_DRIVER_EXPORT(driver)  RT_DRIVER_EXPORT(driver, amba, BUILIN)
#define AMBA_DRIVER_MODULE_EXPORT(driver)  RT_DRIVERS_EXPORT(driver, amba, MODULE)

#endif /* __AMBA_BUS_H__ */

