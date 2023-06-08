/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-26     GuEe-GUI     first version
 */

#include <rtdebug.h>
#include <rtthread.h>
#include <rtservice.h>

#define DBG_TAG "rtdm.clk"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <drivers/clk.h>
#include <drivers/core/rtdm.h>

static struct rt_spinlock _clk_lock = { 0 };
static rt_list_t _clk_nodes = RT_LIST_OBJECT_INIT(_clk_nodes);

static void clk_release(struct ref *r)
{
    struct rt_clk *clk = rt_container_of(r, struct rt_clk, ref);

    LOG_E("%s is release", clk->name);

    RT_ASSERT(0);
}

rt_inline struct rt_clk *clk_get(struct rt_clk *clk)
{
    ref_get(&clk->ref);

    return clk;
}

rt_inline void clk_put(struct rt_clk *clk)
{
    ref_put(&clk->ref, &clk_release);
}

static void clk_set_parent(struct rt_clk *clk, struct rt_clk *parent)
{
    rt_spin_lock(&_clk_lock);

    clk->parent = parent;

    rt_list_insert_after(&parent->children_nodes, &clk->list);

    rt_spin_unlock(&_clk_lock);
}

static const struct rt_clk_ops unused_clk_ops =
{
};

rt_err_t rt_clk_register(struct rt_clk *clk, struct rt_clk *parent)
{
    rt_err_t err = RT_EOK;

    if (clk)
    {
        if (!clk->ops)
        {
            clk->ops = &unused_clk_ops;
        }

        ref_init(&clk->ref);
        rt_list_init(&clk->list);
        rt_list_init(&clk->children_nodes);

        if (parent)
        {
            clk_set_parent(clk, parent);
        }
        else
        {
            clk->parent = RT_NULL;

            rt_spin_lock(&_clk_lock);

            rt_list_insert_after(&_clk_nodes, &clk->list);

            rt_spin_unlock(&_clk_lock);
        }
    }
    else
    {
        err = -RT_EINVAL;
    }

    return err;
}

rt_err_t rt_clk_unregister(struct rt_clk *clk)
{
    rt_err_t err = RT_EOK;

    if (clk)
    {
        err = -RT_EBUSY;

        rt_spin_lock(&_clk_lock);

        if (rt_list_isempty(&clk->children_nodes))
        {
            if (ref_read(&clk->ref) <= 1)
            {
                rt_list_remove(&clk->list);

                err = RT_EOK;
            }
        }

        rt_spin_unlock(&_clk_lock);
    }
    else
    {
        err = -RT_EINVAL;
    }

    return err;
}

void rt_clk_put(struct rt_clk *clk)
{
    if (clk)
    {
        clk_put(clk);
    }
}

struct rt_clk *rt_clk_get_parent(struct rt_clk *clk)
{
    struct rt_clk *parent = RT_NULL;

    if (clk)
    {
        rt_spin_lock(&_clk_lock);

        parent = clk->parent;

        rt_spin_unlock(&_clk_lock);
    }

    return parent;
}

static rt_err_t clk_prepare(struct rt_clk *clk)
{
    rt_err_t err = RT_EOK;

    if (clk->parent)
    {
        clk_prepare(clk->parent);
    }

    if (clk->ops->prepare)
    {
        err = clk->ops->prepare(clk);
    }

    return err;
}

rt_err_t rt_clk_prepare(struct rt_clk *clk)
{
    rt_err_t err = RT_EOK;

    RT_DEBUG_NOT_IN_INTERRUPT;

    if (clk)
    {
        rt_spin_lock(&_clk_lock);

        err = clk_prepare(clk);

        rt_spin_unlock(&_clk_lock);
    }
    else
    {
        err = -RT_EINVAL;
    }

    return err;
}

static void clk_unprepare(struct rt_clk *clk)
{
    if (clk->parent)
    {
        clk_unprepare(clk->parent);
    }

    if (clk->ops->unprepare)
    {
        clk->ops->unprepare(clk);
    }
}

rt_err_t rt_clk_unprepare(struct rt_clk *clk)
{
    rt_err_t err = RT_EOK;

    RT_DEBUG_NOT_IN_INTERRUPT;

    if (clk)
    {
        rt_spin_lock(&_clk_lock);

        clk_unprepare(clk);

        rt_spin_unlock(&_clk_lock);
    }
    else
    {
        err = -RT_EINVAL;
    }

    return err;
}

static rt_err_t clk_enable(struct rt_clk *clk)
{
    rt_err_t err = RT_EOK;

    if (clk->parent)
    {
        clk_enable(clk->parent);
    }

    if (clk->ops->enable)
    {
        err = clk->ops->enable(clk);
    }

    return err;
}

rt_err_t rt_clk_enable(struct rt_clk *clk)
{
    rt_err_t err = RT_EOK;

    if (clk)
    {
        rt_spin_lock(&_clk_lock);

        err = clk_enable(clk);

        rt_spin_unlock(&_clk_lock);
    }
    else
    {
        err = -RT_EINVAL;
    }

    return err;
}

static void clk_disable(struct rt_clk *clk)
{
    if (clk->parent)
    {
        clk_disable(clk->parent);
    }

    if (clk->ops->disable)
    {
        clk->ops->disable(clk);
    }
}

void rt_clk_disable(struct rt_clk *clk)
{
    if (clk)
    {
        rt_spin_lock(&_clk_lock);

        clk_disable(clk);

        rt_spin_unlock(&_clk_lock);
    }
}

rt_err_t rt_clk_prepare_enable(struct rt_clk *clk)
{
    rt_err_t err;

    RT_DEBUG_NOT_IN_INTERRUPT;

    if (clk)
    {
        err = rt_clk_prepare(clk);

        if (!err)
        {
            err = rt_clk_enable(clk);

            if (err)
            {
                rt_clk_unprepare(clk);
            }
        }
    }
    else
    {
        err = -RT_EINVAL;
    }

    return err;
}

void rt_clk_disable_unprepare(struct rt_clk *clk)
{
    RT_DEBUG_NOT_IN_INTERRUPT;

    if (clk)
    {
        rt_clk_disable(clk);
        rt_clk_unprepare(clk);
    }
}

rt_err_t rt_clk_set_rate_range(struct rt_clk *clk, rt_ubase_t min, rt_ubase_t max)
{
    rt_err_t err = RT_EOK;

    if (clk)
    {
        rt_spin_lock(&_clk_lock);

        if (clk->ops->set_rate)
        {
            rt_ubase_t rate = clk->rate;
            rt_ubase_t old_min = clk->min_rate;
            rt_ubase_t old_max = clk->max_rate;

            clk->min_rate = min;
            clk->max_rate = max;

            rate = rt_clamp(rate, min, max);
            err = clk->ops->set_rate(clk, rate, rt_clk_get_rate(clk->parent));

            if (err)
            {
                clk->min_rate = old_min;
                clk->max_rate = old_max;
            }
        }
        else
        {
            err = -RT_ENOSYS;
        }

        rt_spin_unlock(&_clk_lock);
    }
    else
    {
        err = -RT_EINVAL;
    }

    return err;
}

rt_err_t rt_clk_set_min_rate(struct rt_clk *clk, rt_ubase_t rate)
{
    rt_err_t err = RT_EOK;

    if (clk)
    {
        err = rt_clk_set_rate_range(clk, rate, clk->max_rate);
    }
    else
    {
        err = -RT_EINVAL;
    }

    return err;
}

rt_err_t rt_clk_set_max_rate(struct rt_clk *clk, rt_ubase_t rate)
{
    rt_err_t err = RT_EOK;

    if (clk)
    {
        err = rt_clk_set_rate_range(clk, clk->min_rate, rate);
    }
    else
    {
        err = -RT_EINVAL;
    }

    return err;
}

rt_err_t rt_clk_set_rate(struct rt_clk *clk, rt_ubase_t rate)
{
    rt_err_t err = RT_EOK;

    if (clk)
    {
        rt_spin_lock(&_clk_lock);

        if (clk->ops->set_rate)
        {
            err = clk->ops->set_rate(clk, rate, rt_clk_get_rate(clk->parent));
        }
        else
        {
            err = -RT_ENOSYS;
        }

        rt_spin_unlock(&_clk_lock);
    }
    else
    {
        err = -RT_EINVAL;
    }

    return err;
}

rt_ubase_t rt_clk_get_rate(struct rt_clk *clk)
{
    return clk ? clk->rate : -1UL;
}

#ifdef RT_USING_OFW
struct rt_clk *rt_ofw_get_clk(struct rt_ofw_node *np, int index)
{
    struct rt_clk *clk = RT_NULL;

    if (np && index >= 0)
    {
        rt_phandle phandle;

        rt_spin_lock(&_clk_lock);

        if (!rt_ofw_prop_read_u32_index(np, "clocks", index, (rt_uint32_t *)&phandle))
        {
            struct rt_ofw_node *clk_np = rt_ofw_find_node_by_phandle(phandle);

            if (clk_np)
            {
                clk = rt_ofw_data(clk_np);
                rt_ofw_node_put(clk_np);

                if (clk)
                {
                    clk = clk_get(clk);
                }
            }
        }

        rt_spin_unlock(&_clk_lock);
    }

    return clk;
}

struct rt_clk *rt_ofw_get_clk_by_name(struct rt_ofw_node *np, const char *name)
{
    struct rt_clk *clk = RT_NULL;

    if (np && name)
    {
        int index;
        struct rt_ofw_cell_args clk_args;

        rt_spin_lock(&_clk_lock);

        index = rt_ofw_prop_index_of_string(np, "clock-names", name);

        if (index >= 0 && !rt_ofw_parse_phandle_cells(np, "clocks", "#clock-cells", index, &clk_args))
        {
            clk = rt_ofw_data(clk_args.data);

            if (clk)
            {
                clk = clk_get(clk);
            }
        }

        rt_spin_unlock(&_clk_lock);
    }

    return clk;
}
#endif /* RT_USING_OFW */