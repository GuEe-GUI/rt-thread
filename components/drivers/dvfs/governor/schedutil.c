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

#define DBG_TAG "dvfs.governor.schedutil"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static rt_err_t governor_schedutil_start(struct rt_dvfs_scaling *dvfs)
{

}

static rt_err_t governor_schedutil_stop(struct rt_dvfs_scaling *dvfs)
{

}

static rt_err_t governor_schedutil_suspend(struct rt_dvfs_scaling *dvfs)
{

}

static rt_err_t governor_schedutil_resume(struct rt_dvfs_scaling *dvfs)
{

}

static rt_err_t governor_schedutil_set_interval(struct rt_dvfs_scaling *dvfs, rt_uint32_t interval_ms)
{

}

static rt_err_t governor_schedutil_set_frequency(struct rt_dvfs_scaling *dvfs, rt_ubase_t *out_freq)
{

}

static struct rt_dvfs_governor governor_schedutil =
{
    .name = "schedutil",
    .type = RT_DVFS_GOVERNOR_TYPE_SCHEDUTIL,

    .start = governor_schedutil_start,
    .stop = governor_schedutil_stop,
    .suspend = governor_schedutil_suspend,
    .resume = governor_schedutil_resume,
    .set_interval = governor_schedutil_set_interval,
    .set_frequency = governor_schedutil_set_frequency,
};

static int governor_schedutil_init(void)
{
    rt_dvfs_governor_register(&governor_schedutil);

    return 0;
}
INIT_CORE_EXPORT(governor_schedutil_init);
