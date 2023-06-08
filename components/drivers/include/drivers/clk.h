/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-26     GuEe-GUI     first version
 */

#ifndef __CLK_H__
#define __CLK_H__

#include <rthw.h>

#include <ref.h>
#include <drivers/ofw.h>

struct rt_clk_ops;

struct rt_clk
{
    rt_list_t list;
    rt_list_t children_nodes;

    const char *name;
    const struct rt_clk_ops *ops;

    struct rt_clk *parent;
    struct ref ref;

    rt_ubase_t rate;
    rt_ubase_t min_rate;
    rt_ubase_t max_rate;

    void *sysdata;
};

struct rt_clk_fixed_rate
{
    struct rt_clk clk;

    rt_ubase_t fixed_rate;
    rt_ubase_t fixed_accuracy;
};

struct rt_clk_ops
{
    rt_err_t    (*prepare)(struct rt_clk *);
    void        (*unprepare)(struct rt_clk *);
    rt_bool_t   (*is_prepared)(struct rt_clk *);
    rt_err_t    (*enable)(struct rt_clk *);
    void        (*disable)(struct rt_clk *);
    rt_bool_t   (*is_enabled)(struct rt_clk *);
    rt_err_t    (*set_rate)(struct rt_clk *, rt_ubase_t rate, rt_ubase_t parent_rate);
};

rt_err_t rt_clk_register(struct rt_clk *clk, struct rt_clk *parent);
rt_err_t rt_clk_unregister(struct rt_clk *clk);

void rt_clk_put(struct rt_clk *clk);
struct rt_clk *rt_clk_get_parent(struct rt_clk *clk);

rt_err_t rt_clk_prepare(struct rt_clk *clk);
rt_err_t rt_clk_unprepare(struct rt_clk *clk);

rt_err_t rt_clk_enable(struct rt_clk *clk);
void rt_clk_disable(struct rt_clk *clk);

rt_err_t rt_clk_prepare_enable(struct rt_clk *clk);
void rt_clk_disable_unprepare(struct rt_clk *clk);

rt_err_t rt_clk_set_rate_range(struct rt_clk *clk, rt_ubase_t min, rt_ubase_t max);
rt_err_t rt_clk_set_min_rate(struct rt_clk *clk, rt_ubase_t rate);
rt_err_t rt_clk_set_max_rate(struct rt_clk *clk, rt_ubase_t rate);
rt_err_t rt_clk_set_rate(struct rt_clk *clk, rt_ubase_t rate);
rt_ubase_t rt_clk_get_rate(struct rt_clk *clk);

#ifdef RT_USING_OFW
struct rt_clk *rt_ofw_get_clk(struct rt_ofw_node *np, int index);
struct rt_clk *rt_ofw_get_clk_by_name(struct rt_ofw_node *np, const char *name);
#else
struct rt_clk *rt_ofw_get_clk(struct rt_ofw_node *np, int index)
{
    return RT_NULL;
}
struct rt_clk *rt_ofw_get_clk_by_name(struct rt_ofw_node *np, const char *name)
{
    return RT_NULL;
}
#endif /* RT_USING_OFW */

#endif /* __CLK_H__ */
