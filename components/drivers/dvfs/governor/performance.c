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

#define DBG_TAG "dvfs.governor.performance"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static rt_err_t governor_performance_start(struct rt_dvfs_scaling *dvfs)
{

}

static rt_err_t governor_performance_stop(struct rt_dvfs_scaling *dvfs)
{

}

static rt_err_t governor_performance_suspend(struct rt_dvfs_scaling *dvfs)
{

}

static rt_err_t governor_performance_resume(struct rt_dvfs_scaling *dvfs)
{

}

static rt_err_t governor_performance_set_interval(struct rt_dvfs_scaling *dvfs, rt_uint32_t interval_ms)
{

}

static rt_err_t governor_performance_set_frequency(struct rt_dvfs_scaling *dvfs, rt_ubase_t *out_freq)
{

}

static struct rt_dvfs_governor governor_performance =
{
    .name = "performance",
    .type = RT_DVFS_GOVERNOR_TYPE_PERFORMANCE,

    .start = governor_performance_start,
    .stop = governor_performance_stop,
    .suspend = governor_performance_suspend,
    .resume = governor_performance_resume,
    .set_interval = governor_performance_set_interval,
    .set_frequency = governor_performance_set_frequency,
};

static int governor_performance_init(void)
{
    rt_dvfs_governor_register(&governor_performance);

    return 0;
}
INIT_CORE_EXPORT(governor_performance_init);
