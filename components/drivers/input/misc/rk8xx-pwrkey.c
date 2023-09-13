/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-3-08      GuEe-GUI     the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

struct rk8xx_pwrkey
{
    int fall_irq, rise_irq;

    struct rt_thread *poweroff_task;
};

static void rk8xx_pwrkey_fall_isr(int irqno, void *param)
{
    /* Key down */
}

static void rk8xx_pwrkey_rise_isr(int irqno, void *param)
{
    struct rk8xx_pwrkey *pwr = param;

    /* Key up */
    rt_thread_resume(pwr->poweroff_task);
}

static void rk8xx_pwrkey_power_off_task(void *param)
{
    struct rk8xx_pwrkey *pwr = param;

    rt_thread_suspend(pwr->poweroff_task);
    rt_schedule();

    rt_hw_interrupt_mask(pwr->fall_irq);
    rt_hw_interrupt_mask(pwr->rise_irq);

    rt_hw_cpu_shutdown();
}

static rt_err_t rk8xx_pwrkey_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    struct rt_device *dev = &pdev->parent;
    struct rk8xx_pwrkey *pwr = rt_calloc(1, sizeof(*pwr));

    if (!pwr)
    {
        return -RT_ENOMEM;
    }

    if ((pwr->fall_irq = rt_dm_dev_get_irq(dev, 0)) < 0)
    {
        err = pwr->fall_irq;
        goto _fail;
    }

    if ((pwr->rise_irq = rt_dm_dev_get_irq(dev, 1)) < 0)
    {
        err = pwr->rise_irq;
        goto _fail;
    }

    pwr->poweroff_task = rt_thread_create("pwrkey-rk8xx", &rk8xx_pwrkey_power_off_task,
            pwr, SYSTEM_THREAD_STACK_SIZE, RT_THREAD_PRIORITY_MAX / 2, 10);

    if (!pwr->poweroff_task)
    {
        goto _fail;
    }

    rt_thread_startup(pwr->poweroff_task);

    dev->user_data = pwr;

    rt_hw_interrupt_install(pwr->fall_irq, rk8xx_pwrkey_fall_isr, pwr, "pwrkey-rk8xx-fall");
    rt_hw_interrupt_umask(pwr->fall_irq);

    rt_hw_interrupt_install(pwr->rise_irq, rk8xx_pwrkey_rise_isr, pwr, "pwrkey-rk8xx-rise");
    rt_hw_interrupt_umask(pwr->rise_irq);

    return RT_EOK;

_fail:
    rt_free(pwr);

    return err;
}

static rt_err_t rk8xx_pwrkey_remove(struct rt_platform_device *pdev)
{
    struct rk8xx_pwrkey *pwr = pdev->parent.user_data;

    rt_hw_interrupt_mask(pwr->fall_irq);
    rt_pic_detach_irq(pwr->fall_irq, pwr);

    rt_hw_interrupt_mask(pwr->rise_irq);
    rt_pic_detach_irq(pwr->rise_irq, pwr);

    rt_thread_delete(pwr->poweroff_task);

    rt_free(pwr);

    return RT_EOK;
}

static struct rt_platform_driver rk8xx_pwrkey_driver =
{
    .name = "rk8xx-pwrkey",
    .probe = rk8xx_pwrkey_probe,
    .remove = rk8xx_pwrkey_remove,
};
RT_PLATFORM_DRIVER_EXPORT(rk8xx_pwrkey_driver);
