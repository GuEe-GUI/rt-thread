/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <drivers/core/bus.h>

#if defined(RT_USING_POSIX_DEVIO)
#include <rtdevice.h> /* for wqueue_init */
#endif

#ifdef RT_USING_DM
/**
 * This function driver device match with id
 *
 * @param drv the pointer of driver structure
 * @param device_id the id of the device
 *
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_driver_match_with_id(const rt_driver_t drv,int device_id)
{
    rt_device_t device;
    int ret;
    if (!drv)
    {
        return -RT_EINVAL;
    }
    device = rt_device_create_since_driver(drv,device_id);
    if(!device)
    {
        return -RT_ERROR;
    }
    ret = rt_device_bind_driver(device,drv,RT_NULL);
    if(ret != 0)
    {
        return -RT_ERROR;
    }
    ret = rt_device_probe_and_init(device);
    if(ret != 0)
    {
        return -RT_ERROR;
    }
    return ret;
}

RTM_EXPORT(rt_driver_match_with_id);
#endif

#ifdef RT_USING_FDT
/**
 * This function driver device match with dtb_node
 *
 * @param drv the pointer of driver structure
 * @param from_node dtb node entry 
 * @param max_dev_num the max device support 
 * 
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_driver_match_with_dtb(const rt_driver_t drv,void *from_node,int max_dev_num)
{
    struct dtb_node** node_list; 
    rt_device_t device;
    int ret,i;
    int total_dev_num = 0;
    if ((!drv)||(!drv->ids)||(!drv->ids->compatible)||(!from_node)||(!drv->device_size))
    {
        return -RT_EINVAL;
    }

    node_list = rt_calloc(max_dev_num,sizeof(void *));
    if(!node_list)
    {
        return -RT_ERROR;
    }

    ret = dtb_node_find_all_compatible_node(from_node,drv->ids->compatible,node_list,max_dev_num,&total_dev_num);
    if((ret != 0) || (!total_dev_num))
    {
        return -RT_ERROR;
    }
    
    for(i = 0; i < total_dev_num; i ++)
    {
        if (!dtb_node_device_is_available(node_list[i]))
        {
            continue;
        }
        device = rt_device_create_since_driver(drv,i);
        if(!device)
        {
            continue;
        }
    
        ret = rt_device_bind_driver(device,drv,node_list[i]);
        if(ret != 0)
        {
            continue;
        }
        ret = rt_device_probe_and_init(device);
        if(ret != 0)
        {
            continue;
        }
    }
    rt_free(node_list);
    return ret;
}

RTM_EXPORT(rt_driver_match_with_dtb);


/**
 * This function match a dtb_node with driver
 *
 * @param drv the pointer of driver structure
 * @param from_node dtb node entry 
 * 
 * @return the error code, RT_EOK on successfully.
 */
struct dtb_node *dtb_match_node(const rt_driver_t drv, void *from_node)
{
    struct dtb_node** node_list;
    int ret,i;
    int total_dev_num = 0;
    if ((!drv)||(!drv->ids)||(!drv->ids->compatible)||(!from_node))
    {
        return RT_NULL;
    }

    node_list = rt_calloc(10,sizeof(void *));
    if(!node_list)
    {
        return RT_NULL;
    }

    ret = dtb_node_find_all_compatible_node(from_node,drv->ids->compatible,node_list, 10,&total_dev_num);
    if((ret != 0) || (!total_dev_num))
    {
        return RT_NULL;
    }

    struct dtb_node *dtb_node;
    for(i = 0; i < total_dev_num; i ++)
    {
        if (!dtb_node_device_is_available(node_list[i]))
        {
            continue;
        }
        dtb_node = node_list[i];
        break;
    }

    rt_free(node_list);
    return dtb_node;
}

/**
 * This function bind device and driver
 *
 * @param device the device to bind
 * @param driver the driver to bind
 *
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_device_bind_with_driver(struct rt_device *dev, struct rt_driver *drv)
{

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(drv != RT_NULL);

    dev->drv = drv;

    if(!dev->dtb_node)
    {
        void *root_node = get_dtb_node_head();
        struct dtb_node *dtb_node = dtb_match_node(drv, root_node);
        if (!dtb_node)
        {
            return -RT_ERROR;
        }
        dev->dtb_node = dtb_node;
    }

    return RT_EOK;
}
RTM_EXPORT(rt_device_bind_with_driver);

#endif  


/**
 * This function create device by driver
 *
 * @param drv the driver to create device
 *
 * @return the device created by driver
 */
rt_device_t rt_driver_create_device(struct rt_driver *drv)
{
    rt_device_t device;

    RT_ASSERT(drv != RT_NULL);

    device = rt_device_create(drv->dev_type, drv->device_size);

    rt_snprintf(device->parent.name, sizeof(device->parent.name), "%s", drv->name);

    return device;
}
RTM_EXPORT(rt_driver_create_device);

#ifdef RT_USING_DM
/**
 * This function probe device by driver
 *
 * @param device the device to be probed
 * @param driver the driver to probe device
 *
 * @return the result of probe, RT_EOK on successfully.
 */
int rt_driver_probe_device(struct rt_driver *drv, struct rt_device *dev)
{
    int ret = 0;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(drv != RT_NULL);

    rt_device_bind_with_driver(dev, drv);
    ret = drv->probe(dev);

    return ret;
}
RTM_EXPORT(rt_driver_probe_device);

/**
 * This function attach a driver to bus
 *
 * @param drv the driver to be attached
 */
rt_err_t rt_driver_register(rt_driver_t drv)
{
    rt_err_t ret;
    struct rt_bus *bus = RT_NULL;

    RT_ASSERT(drv != RT_NULL);

    /*TODO: check if drv in the bus*/

    if(drv->bus)
    {
        bus = drv->bus;
    }
    else
    {
        bus = rt_bus_find_by_name("platform");
    }
    
    if (bus)
    {
        ret = rt_bus_add_driver(bus, drv);
    }
    
    return ret;
}
RTM_EXPORT(rt_driver_register);

/**
 * This function remove driver from system.
 *
 * @param drv the driver to be removed
 */
rt_err_t rt_driver_unregister(rt_driver_t drv)
{
    rt_err_t ret;

    ret = rt_bus_remove_driver(drv);
    
    return ret;
}
RTM_EXPORT(rt_driver_register);

#endif
