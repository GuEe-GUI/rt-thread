/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-21     GuEe-GUI     first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "dvfs.governor.powersave"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static rt_err_t governor_powersave_start(struct rt_dvfs_scaling *dvfs)
{

}

static rt_err_t governor_powersave_stop(struct rt_dvfs_scaling *dvfs)
{

}

static rt_err_t governor_powersave_suspend(struct rt_dvfs_scaling *dvfs)
{

}

static rt_err_t governor_powersave_resume(struct rt_dvfs_scaling *dvfs)
{

}

static rt_err_t governor_powersave_set_interval(struct rt_dvfs_scaling *dvfs, rt_uint32_t interval_ms)
{

}

static rt_err_t governor_powersave_set_frequency(struct rt_dvfs_scaling *dvfs, rt_ubase_t *out_freq)
{

}

static struct rt_dvfs_governor governor_powersave =
{
    .name = "powersave",
    .type = RT_DVFS_GOVERNOR_TYPE_POWERSAVE,

    .start = governor_powersave_start,
    .stop = governor_powersave_stop,
    .suspend = governor_powersave_suspend,
    .resume = governor_powersave_resume,
    .set_interval = governor_powersave_set_interval,
    .set_frequency = governor_powersave_set_frequency,
};

static int governor_powersave_init(void)
{
    rt_dvfs_governor_register(&governor_powersave);

    return 0;
}
INIT_CORE_EXPORT(governor_powersave_init);
