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

#define DBG_TAG "dvfs.ofw.cpufreq"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static rt_err_t cpu_suspend(struct rt_dvfs_scaling *dvfs)
{

}

static rt_err_t cpu_resume(struct rt_dvfs_scaling *dvfs)
{

}

static rt_err_t cpu_set_opp(struct rt_dvfs_scaling *dvfs, struct rt_dvfs_opp *opp)
{

}

static rt_err_t cpu_parse_opp(struct rt_dvfs_scaling *dvfs, struct rt_dvfs_opp *opp, void *fw_np)
{

}

struct rt_dvfs_scaling_ops dvfs_cpufreq_ofw_ops =
{
    .suspend = cpu_suspend,
    .resume = cpu_resume,
    .set_opp = cpu_set_opp,
    .parse_opp = cpu_parse_opp,
};

static void ofw_cpufreq_free(struct rt_dvfs_cpufreq *cpufreq)
{

}

static rt_err_t ofw_cpufreq_probe(struct rt_platform_device *pdev)
{
    struct rt_device *dev = &pdev->parent;
    struct rt_dvfs_scaling *dvfs;
    struct rt_dvfs_cpufreq *cpufreq = pdev->priv;

    if (!cpufreq)
    {
        return -RT_EINVAL;
    }
    dvfs = &cpufreq->parent;

    if (!dvfs->clk)
    {
        dvfs->clk = rt_clk_get_by_index(dev, 0);

        if (rt_is_err(dvfs->clk))
        {
            return rt_ptr_err(dvfs->clk);
        }
    }

    dvfs->ops = dvfs->ops ? : &dvfs_cpufreq_ofw_ops;

    return rt_dvfs_cpufreq_register(cpufreq);
}

static rt_err_t ofw_cpufreq_remove(struct rt_platform_device *pdev)
{
    struct rt_dvfs_cpufreq *cpufreq = pdev->priv;

    rt_dvfs_cpufreq_unregister(cpufreq);
    ofw_cpufreq_free(cpufreq);

    return RT_EOK;
}

static struct rt_platform_driver ofw_cpufreq_driver =
{
    .name = "ofw-cpufreq",
    .probe = ofw_cpufreq_probe,
    .remove = ofw_cpufreq_remove,
};
RT_PLATFORM_DRIVER_EXPORT(ofw_cpufreq_driver);
