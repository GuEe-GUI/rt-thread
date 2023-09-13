/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-12-06     GuEe-GUI     first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define REG_CONTROL     0x00
#define REG_COUNTER_LO  0x04
#define REG_COUNTER_HI  0x08
#define REG_COMPARE(n)  (0x0c + (n) * 4)
#define MAX_TIMER       3
#define DEFAULT_TIMER   3

struct bcm2835_timer
{
    struct rt_hwtimer_device parent;

    int irq;
    void *base;
    void *control;
    void *compare;
    int match_mask;

    struct rt_hwtimer_info info;
};

#define raw_to_bcm2835_timer(raw) rt_container_of(raw, struct bcm2835_timer, parent)

static void bcm2835_timer_init(struct rt_hwtimer_device *timer, rt_uint32_t state)
{
}

static rt_err_t bcm2835_timer_start(struct rt_hwtimer_device *timer, rt_uint32_t cnt, rt_hwtimer_mode_t mode)
{
    rt_err_t err = RT_EOK;
    struct bcm2835_timer *bcm2835_timer = raw_to_bcm2835_timer(timer);

    switch (mode)
    {
    case HWTIMER_MODE_ONESHOT:
        HWREG32(bcm2835_timer->compare) = HWREG32(bcm2835_timer->base + REG_COUNTER_LO) + cnt;
        break;

    case HWTIMER_MODE_PERIOD:
        err = -RT_ENOSYS;
        break;

    default:
        err = -RT_EINVAL;
        break;
    }

    return err;
}

static void bcm2835_timer_stop(struct rt_hwtimer_device *timer)
{
}

static rt_uint32_t bcm2835_timer_count_get(struct rt_hwtimer_device *timer)
{
    struct bcm2835_timer *bcm2835_timer = raw_to_bcm2835_timer(timer);

    return HWREG32(bcm2835_timer->base + REG_COUNTER_LO);
}

static rt_err_t bcm2835_timer_ctrl(struct rt_hwtimer_device *timer, rt_uint32_t cmd, void *args)
{
    rt_err_t err = RT_EOK;
    struct bcm2835_timer *bcm2835_timer = raw_to_bcm2835_timer(timer);

    switch (cmd)
    {
    case HWTIMER_CTRL_FREQ_SET:
        err = -RT_ENOSYS;
        break;

    case HWTIMER_CTRL_STOP:
        break;

    case HWTIMER_CTRL_INFO_GET:
        if (args)
        {
            rt_memcpy(args, &bcm2835_timer->info, sizeof(bcm2835_timer->info));
        }
        else
        {
            err = -RT_ERROR;
        }
        break;

    case HWTIMER_CTRL_MODE_SET:
        err = -RT_ENOSYS;
        break;

    default:
        err = -RT_EINVAL;
        break;
    }

    return err;
}

const static struct rt_hwtimer_ops bcm2835_timer_ops =
{
    .init = bcm2835_timer_init,
    .start = bcm2835_timer_start,
    .stop = bcm2835_timer_stop,
    .count_get = bcm2835_timer_count_get,
    .control = bcm2835_timer_ctrl,
};

static void bcm2835_timer_isr(int irqno, void *param)
{
    struct bcm2835_timer *bcm2835_timer = (struct bcm2835_timer *)param;

    if (HWREG32(bcm2835_timer->control) & bcm2835_timer->match_mask)
    {
        HWREG32(bcm2835_timer->control) = bcm2835_timer->match_mask;

        rt_device_hwtimer_isr(&bcm2835_timer->parent);
    }
}

static rt_err_t bcm2835_timer_probe(struct rt_platform_device *pdev)
{
    rt_err_t err = RT_EOK;
    rt_uint32_t freq;
    const char *dev_name;
    struct rt_device *dev = &pdev->parent;
    struct bcm2835_timer *timer = rt_calloc(1, sizeof(*timer));

    if (!timer)
    {
        return -RT_ENOMEM;
    }

    timer->base = rt_dm_dev_iomap(dev, 0);

    if (!timer->base)
    {
        err = -RT_EIO;

        goto _fail;
    }

    timer->control = timer->base + REG_CONTROL;
    timer->compare = timer->base + REG_COMPARE(DEFAULT_TIMER);
    timer->match_mask = RT_BIT(DEFAULT_TIMER);

    timer->irq = rt_dm_dev_get_irq(dev, DEFAULT_TIMER);

    if (timer->irq < 0)
    {
        err = -RT_EIO;

        goto _fail;
    }

    if (rt_dm_dev_prop_read_u32(dev, "clock-frequency", &freq))
    {
        err = -RT_EIO;

        goto _fail;
    }

    dev->user_data = timer;

    timer->parent.ops = &bcm2835_timer_ops;
    timer->parent.info = &timer->info;

    timer->info.maxfreq = freq;
    timer->info.minfreq = freq;
    timer->info.maxcnt = 0xffffffff;
    timer->info.cntmode = HWTIMER_CNTMODE_UP;

    rt_dm_dev_set_name_auto(&timer->parent.parent, "timer");
    dev_name = rt_dm_dev_get_name(&timer->parent.parent);

    rt_device_hwtimer_register(&timer->parent, dev_name, RT_NULL);
    rt_hw_interrupt_install(timer->irq, bcm2835_timer_isr, timer, dev_name);
    rt_hw_interrupt_umask(timer->irq);

    return err;

_fail:
    if (timer->base)
    {
        rt_iounmap(timer->base);
    }
    rt_free(timer);

    return err;
}

static rt_err_t bcm2835_timer_remove(struct rt_platform_device *pdev)
{
    struct bcm2835_timer *timer = pdev->parent.user_data;

    bcm2835_timer_stop(&timer->parent);

    rt_hw_interrupt_mask(timer->irq);
    rt_pic_detach_irq(timer->irq, timer);

    rt_device_unregister(&timer->parent.parent);

    rt_iounmap(timer->base);
    rt_free(timer);

    return RT_EOK;
}

static const struct rt_ofw_node_id bcm2835_timer_ofw_ids[] =
{
    { .compatible = "brcm,bcm2835-system-timer", },
    { /* sentinel */ }
};

static struct rt_platform_driver bcm2835_timer_driver =
{
    .name = "hwtimer-bcm2835",
    .ids = bcm2835_timer_ofw_ids,

    .probe = bcm2835_timer_probe,
    .remove = bcm2835_timer_remove,
};

static int bcm2835_timer_drv_register(void)
{
    rt_platform_driver_register(&bcm2835_timer_driver);

    return 0;
}
INIT_DRIVER_EARLY_EXPORT(bcm2835_timer_drv_register);
