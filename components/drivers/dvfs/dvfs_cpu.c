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

#define DBG_TAG "dvfs.cpu"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static void cpufreq_monitor_work(struct rt_work *work, void *work_data)
{
    struct rt_dvfs_cpufreq *cpufreq = rt_container_of(work, struct rt_dvfs_cpufreq, monitor_work);
}

rt_err_t rt_dvfs_cpufreq_register(struct rt_dvfs_cpufreq *cpufreq)
{
	rt_work_init(&cpufreq->monitor_work, cpufreq_monitor_work, cpufreq);
}

rt_err_t rt_dvfs_cpufreq_unregister(struct rt_dvfs_cpufreq *cpufreq)
{

}
