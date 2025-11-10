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

#define DBG_TAG "rtdm.dvfs"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static RT_DEFINE_SPINLOCK(_dvfs_scaling_lock);

#ifdef RT_USING_OFW
static rt_err_t dvfs_ofw_parse_opp(struct rt_dvfs_scaling *dvfs)
{
    struct rt_dvfs_opp *opp;
    struct rt_ofw_node *opp_np, *opp_child_np;

    if (!dvfs->dev->ofw_node)
    {
        return RT_EOK;
    }

    opp_np = rt_ofw_parse_phandle(dvfs->dev->ofw_node, "operating-points-v2", 0);

    if (!opp_np)
    {
        return RT_EOK;
    }

    rt_ofw_foreach_child_node(opp_np, opp_child_np)
    {
        rt_uint64_t hz = 0;
        rt_uint32_t uvolt[3] = {0}, uvolt_nr = 0;

        if (rt_ofw_prop_read_u64(opp_child_np, "opp-hz", &hz))
        {
            continue;
        }

        uvolt_nr = rt_ofw_prop_read_u32_array_index(opp_child_np,
                "opp-microvolt", 0, RT_ARRAY_SIZE(uvolt), uvolt);

        if ((int)uvolt_nr < 0)
        {
            /* If previous voltage is unknown, assume 0 to ensure a voltage ramp-up */
            uvolt[0] = 0;
        }

        if (!(opp = rt_dvfs_scaling_add_opp(dvfs, (rt_ubase_t)hz, (rt_ubase_t)uvolt[0])))
        {
            continue;
        }

        if (dvfs->ops && dvfs->ops->parse_opp)
        {
            rt_err_t err = dvfs->ops->parse_opp(dvfs, opp, (void *)opp_child_np);

            LOG_W("%s: Parse OPP %s error = %s", rt_dm_dev_get_name(dvfs->dev),
                    rt_ofw_node_full_name(opp_child_np), rt_strerror(err));
        }
    }

    dvfs->opp_table->share = rt_ofw_prop_read_bool(opp_np, "opp-shared");
    dvfs->opp_table->priv = dvfs->opp_table->priv ? : opp_np; /* Default value, DVFS unused */

    return RT_EOK;
}
#endif /* RT_USING_OFW */

rt_err_t rt_dvfs_scaling_register(struct rt_dvfs_scaling *dvfs)
{
    rt_err_t err = RT_EOK;

    if (!dvfs || !dvfs->dev || !dvfs->ops)
    {
        return -RT_EINVAL;
    }

    RT_ASSERT(dvfs->ops->set_opp != RT_NULL);

#ifdef RT_USING_OFW
    if ((err = dvfs_ofw_parse_opp(dvfs)))
    {
        return err;
    }
#endif /* RT_USING_OFW */

    rt_dm_dev_bind_fwdata(dvfs->dev, RT_NULL, dvfs);
    dvfs->dev->dvfs_scaling = dvfs;

    return RT_EOK;
}

rt_err_t rt_dvfs_scaling_unregister(struct rt_dvfs_scaling *dvfs)
{
    if (!dvfs)
    {
        return -RT_EINVAL;
    }

    if (dvfs->gov)
    {
        if (dvfs->gov->stop)
        {
            dvfs->gov->stop(dvfs);
        }

        rt_dvfs_governor_put(dvfs->gov);
        dvfs->gov = RT_NULL;
    }

    dvfs->dev->dvfs_scaling = RT_NULL;
    rt_dm_dev_unbind_fwdata(dvfs->dev, RT_NULL);

    /* Free the OPP by Drivers */

    return RT_EOK;
}

void rt_dvfs_scaling_enter(struct rt_dvfs_scaling *dvfs)
{
    if (dvfs)
    {
        rt_spin_lock(&_dvfs_scaling_lock);
    }
}

void rt_dvfs_scaling_leave(struct rt_dvfs_scaling *dvfs)
{
    if (dvfs)
    {
        rt_spin_unlock(&_dvfs_scaling_lock);
    }
}

void rt_dvfs_ns_sleep(rt_uint32_t ns)
{
    rt_uint32_t us;

    if (!ns)
    {
        return;
    }

    us = (ns + 999) / 1000;

    if (us < 1000 || rt_hw_interrupt_is_disabled())
    {
        rt_hw_us_delay(us);
    }
    else
    {
        rt_thread_mdelay(us / 1000);
    }
}

rt_err_t rt_dvfs_scaling_suspend(struct rt_dvfs_scaling *dvfs)
{
    rt_err_t err = RT_EOK;

    if (!dvfs)
    {
        return -RT_EINVAL;
    }

    if (dvfs->suspend_freq)
    {
        if ((err = rt_dvfs_scaling_set_frequency(dvfs, dvfs->suspend_freq)))
        {
            LOG_W("%s: set suspend frequency(%lu) error = %s",
                    rt_dm_dev_get_name(dvfs->dev), dvfs->suspend_freq, rt_strerror(err));
        }
    }

    if (dvfs->ops && dvfs->ops->suspend)
    {
        rt_dvfs_scaling_enter(dvfs);
        err = dvfs->ops->suspend(dvfs);
        rt_dvfs_scaling_leave(dvfs);
    }

    return err;
}

rt_err_t rt_dvfs_scaling_resume(struct rt_dvfs_scaling *dvfs)
{
    rt_err_t err = RT_EOK;

    if (!dvfs)
    {
        return -RT_EINVAL;
    }

    if (dvfs->ops && dvfs->ops->resume)
    {
        rt_dvfs_scaling_enter(dvfs);
        err = dvfs->ops->resume(dvfs);
        rt_dvfs_scaling_leave(dvfs);
    }

    return err;
}

rt_err_t rt_dvfs_scaling_set_governor(struct rt_dvfs_scaling *dvfs, rt_uint32_t governor)
{
    rt_err_t err = RT_EOK;
    struct rt_dvfs_governor *gov;

    if (!dvfs)
    {
        return -RT_EINVAL;
    }

    if (!(gov = rt_dvfs_governor_get(governor)))
    {
        return -RT_ENOSYS;
    }

    if (dvfs->gov)
    {
        if (dvfs->gov->stop)
        {
            if ((err = dvfs->gov->stop(dvfs)))
            {
                rt_dvfs_governor_put(gov);
                return err;
            }
        }

        rt_dvfs_governor_put(dvfs->gov);
    }

    dvfs->gov = gov;

    if (dvfs->gov->start)
    {
        if ((err = dvfs->gov->start(dvfs)))
        {
            rt_dvfs_governor_put(dvfs->gov);
            dvfs->gov = RT_NULL;
        }
    }

    return err;
}

rt_err_t rt_dvfs_scaling_set_frequency(struct rt_dvfs_scaling *dvfs, rt_ubase_t frequency)
{
    rt_err_t err;
    struct rt_dvfs_opp *opp = RT_NULL;

    if (!dvfs || !dvfs->opp_table)
    {
        return -RT_EINVAL;
    }

    if (dvfs->min_freq && frequency < dvfs->min_freq)
    {
        frequency = dvfs->min_freq;
    }
    if (dvfs->max_freq && frequency > dvfs->max_freq)
    {
        frequency = dvfs->max_freq;
    }

    if (!(opp = rt_dvfs_scaling_find_opp(dvfs, frequency)))
    {
        if (!(opp = rt_dvfs_scaling_find_floor_opp(dvfs, frequency)))
        {
            opp = rt_dvfs_scaling_find_ceil_opp(dvfs, frequency);
        }
    }

    if (!opp || !opp->available)
    {
        return -RT_ENOENT;
    }

    err = rt_dvfs_scaling_apply_opp(dvfs, opp);

    return err;
}

static rt_err_t dvfs_regulator_set_voltage_retry(struct rt_regulator *supply,
        rt_ubase_t uvolt, rt_uint32_t retry_ns)
{
    for (int i = 0; i < RT_USING_DVFS_OPP_RETRY_MAX; ++i)
    {
        rt_err_t err = rt_regulator_set_voltage(supply, uvolt, uvolt);

        if (err == -RT_EBUSY)
        {
            rt_dvfs_ns_sleep(retry_ns);
            continue;
        }

        return err;
    }

    return -RT_EBUSY;
}

static rt_err_t dvfs_clk_set_rate_retry(struct rt_clk *clk,
        rt_ubase_t rate, rt_uint32_t retry_ns)
{
    for (int i = 0; i < RT_USING_DVFS_OPP_RETRY_MAX; ++i)
    {
        rt_err_t err = rt_clk_set_rate(clk, rate);

        if (err == -RT_EBUSY)
        {
            rt_dvfs_ns_sleep(retry_ns);
            continue;
        }

        return err;
    }

    return -RT_EBUSY;
}

rt_err_t rt_dvfs_scaling_apply_opp(struct rt_dvfs_scaling *dvfs, struct rt_dvfs_opp *opp)
{
    rt_err_t err;

    if (!dvfs || !opp || !dvfs->ops || !dvfs->ops->set_opp || !dvfs->opp_table)
    {
        return -RT_EINVAL;
    }

    if (!opp->available)
    {
        return -RT_EINVAL;
    }

    if ((dvfs->min_freq && opp->freq < dvfs->min_freq) ||
        (dvfs->max_freq && opp->freq > dvfs->max_freq))
    {
        return -RT_EINVAL;
    }

    if (dvfs->ops->set_opp)
    {
        for (int tries = 0; tries < RT_USING_DVFS_OPP_RETRY_MAX; ++tries)
        {
            err = dvfs->ops->set_opp(dvfs, opp);

            if (err == -RT_EBUSY)
            {
                rt_dvfs_ns_sleep(dvfs->retry_delay);
            }
            else if (err)
            {
                break;
            }
        }

        if (err)
        {
            return err;
        }

        rt_dvfs_ns_sleep(dvfs->transition_latency);

        if ((err = dvfs->ops->set_opp(dvfs, opp)))
        {
            return err;
        }
    }
    else
    {
        rt_uint32_t retry_delay = dvfs->retry_delay;
        rt_ubase_t old_uvolt, old_freq, new_uvolt, new_freq;
        struct rt_dvfs_opp *old = dvfs->opp_table->current_opp;

        /* If previous voltage is unknown, assume 0 to ensure a voltage ramp-up */
        old_uvolt = old ? old->uvolt : 0;
        old_freq = dvfs->cur_freq;

        new_uvolt = opp->uvolt;
        new_freq = opp->freq;

        if (new_freq > old_freq)
        {
            /* Scale up: raise voltage first, then increase frequency */
            if (dvfs->supply && new_uvolt > old_uvolt)
            {
                if ((err = dvfs_regulator_set_voltage_retry(dvfs->supply, new_uvolt, retry_delay)))
                {
                    return err;
                }
            }

            if (dvfs->clk)
            {
                if ((err = dvfs_clk_set_rate_retry(dvfs->clk, new_freq, retry_delay)))
                {
                    return err;
                }
            }
        }
        else if (new_freq < old_freq)
        {
            /* Scale down: lower frequency first, then lower voltage */
            if (dvfs->clk)
            {
                if ((err = dvfs_clk_set_rate_retry(dvfs->clk, new_freq, retry_delay)))
                {
                    return err;
                }
            }

            if (dvfs->supply && new_uvolt < old_uvolt)
            {
                if ((err = dvfs_regulator_set_voltage_retry(dvfs->supply, new_uvolt, retry_delay)))
                {
                    return err;
                }
            }
        }
        else
        {
            /* Frequency unchanged: adjust voltage only if needed */
            if (dvfs->supply && new_uvolt != old_uvolt)
            {
                if ((err = dvfs_regulator_set_voltage_retry(dvfs->supply, new_uvolt, retry_delay)))
                {
                    return err;
                }
            }
        }

        rt_dvfs_ns_sleep(dvfs->transition_latency);
    }

    rt_dvfs_scaling_enter(dvfs);
    dvfs->cur_freq = opp->freq;
    dvfs->opp_table->current_opp = opp;
    rt_dvfs_scaling_leave(dvfs);

    return RT_EOK;
}
