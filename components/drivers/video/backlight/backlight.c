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

#define DBG_TAG "rtdm.backlight"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <stdlib.h>
#include <drivers/backlight.h>

static rt_ssize_t _backlight_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_ssize_t res;
    rt_uint32_t brightness;
    struct rt_backlight_device *bl = rt_container_of(dev, struct rt_backlight_device, parent);

    if ((res = rt_backlight_get_brightness(bl, &brightness)))
    {
        return res;
    }

    return rt_sprintf(buffer, "%u", brightness);
}

static rt_ssize_t _backlight_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_ssize_t res;
    rt_uint32_t brightness = atoi(buffer);
    struct rt_backlight_device *bl = rt_container_of(dev, struct rt_backlight_device, parent);

    if (brightness > bl->props.max_brightness)
    {
        LOG_D("%s: brightness(%u) > max_brightness(%u)",
                rt_dm_dev_get_name(dev), brightness, bl->props.max_brightness);

        return -RT_EINVAL;
    }

    if ((res = rt_backlight_set_brightness(bl, brightness)))
    {
        return res;
    }

    LOG_D("%s: brightness to %u", rt_dm_dev_get_name(dev), brightness);

    return size;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops _backlight_ops =
{
    .read = _backlight_read,
    .write = _backlight_write,
};
#endif

rt_err_t rt_hw_backlight_register(struct rt_backlight_device *bl)
{
    const char *dev_name;

    if (!bl || !bl->ops)
    {
        return -RT_EINVAL;
    }

    bl->parent.type = RT_Device_Class_Char;
#ifdef RT_USING_DEVICE_OPS
    bl->parent.ops = &_backlight_ops;
#else
    bl->parent.read = _backlight_read;
    bl->parent.write = _backlight_write;
#endif

    dev_name = rt_dm_dev_get_name(&bl->parent);
    rt_spin_lock_init(&bl->spinlock);

    return rt_device_register(&bl->parent, dev_name, RT_DEVICE_FLAG_RDWR);
}

rt_err_t rt_hw_backlight_unregister(struct rt_backlight_device *bl)
{
    if (!bl)
    {
        return -RT_EINVAL;
    }

    rt_backlight_set_power(bl, RT_BACKLIGHT_POWER_POWERDOWN);

    rt_device_unregister(&bl->parent);

    return RT_EOK;
}

rt_err_t rt_backlight_set_power(struct rt_backlight_device *bl, enum rt_backlight_power power)
{
    rt_err_t err;
    enum rt_backlight_power old_power;

    if (rt_unlikely(!bl || power >= RT_BACKLIGHT_POWER_NR))
    {
        return -RT_EINVAL;
    }

    rt_spin_lock(&bl->spinlock);

    old_power = bl->props.power;
    bl->props.power = power;

    if ((err = bl->ops->update_status(bl)))
    {
        bl->props.power = old_power;
    }

    rt_spin_unlock(&bl->spinlock);

    return err;
}

rt_err_t rt_backlight_get_power(struct rt_backlight_device *bl, enum rt_backlight_power *out_power)
{
    if (rt_unlikely(!bl || !out_power))
    {
        return -RT_EINVAL;
    }

    rt_spin_lock(&bl->spinlock);

    *out_power = bl->props.power;

    rt_spin_unlock(&bl->spinlock);

    return RT_EOK;
}

rt_err_t rt_backlight_set_brightness(struct rt_backlight_device *bl, rt_uint32_t brightness)
{
    rt_err_t err;
    rt_uint32_t old_brightness;

    if (rt_unlikely(!bl || brightness > bl->props.max_brightness))
    {
        return -RT_EINVAL;
    }

    rt_spin_lock(&bl->spinlock);

    old_brightness = bl->props.brightness;
    bl->props.brightness = brightness;

    if ((err = bl->ops->update_status(bl)))
    {
        bl->props.brightness = old_brightness;
    }

    rt_spin_unlock(&bl->spinlock);

    return err;
}

rt_err_t rt_backlight_get_brightness(struct rt_backlight_device *bl, rt_uint32_t *out_brightness)
{
    rt_err_t err;

    if (rt_unlikely(!bl || !out_brightness))
    {
        return -RT_EINVAL;
    }

    rt_spin_lock(&bl->spinlock);

    if (bl->ops->get_brightness)
    {
        err = bl->ops->get_brightness(bl, out_brightness);
    }
    else
    {
        *out_brightness = rt_backlight_power_brightness(bl);

        err = RT_EOK;
    }

    rt_spin_unlock(&bl->spinlock);

    return err;
}
