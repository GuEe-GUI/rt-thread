/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-12     ErikChan      the first version
 */

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <drivers/core/device.h>

typedef struct rt_driver *rt_driver_t;

/* driver object ptr structure */
struct rt_drv_object
{
    struct rt_driver *drv;
};

struct rt_driver
{
    /*new params*/
    struct rt_bus *bus;
    rt_list_t node;
    /*new params*/

#ifdef RT_USING_DEVICE_OPS
    const struct rt_device_ops *dev_ops;
#else
    /* common device interface */
    rt_err_t  (*init)   (rt_device_t dev);
    rt_err_t  (*open)   (rt_device_t dev, rt_uint16_t oflag);
    rt_err_t  (*close)  (rt_device_t dev);
    rt_ssize_t (*read)  (rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size);
    rt_ssize_t (*write) (rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size);
    rt_err_t  (*control)(rt_device_t dev, int cmd, void *args);
#endif
    const struct filesystem_ops *fops;
    const char *name;
    enum rt_device_class_type dev_type;
    int device_size;
    int flag;
    const struct rt_device_id *ids;
    int (*probe)(struct rt_device *dev);
    int (*probe_init)(struct rt_device *dev);
    int (*remove)(struct rt_device *dev);
    const void *ops;    /* driver-specific operations */
    void *drv_priv_data;
    void *priv;
};

int rt_driver_probe_device(struct rt_driver *drv, struct rt_device *dev);
void rt_system_drivers_init(void);

rt_err_t rt_driver_register(rt_driver_t drv);
rt_err_t rt_device_bind_with_driver(struct rt_device *device, struct rt_driver *driver);

#endif /* __DRIVER_H__ */
