/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-26     GuEe-GUI     first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "clk.rk8xx"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "../../mfd/rk8xx.h"

struct rk8xx_clkout
{
    struct rt_clk_node parent[2];

    struct rk8xx *rk8xx;
};

#define raw_to_rk8xx_clkout(raw, idx) &(rt_container_of(raw, struct rk8xx_clkout, parent[0])[-idx])

static rt_err_t rk808_clkout_init(struct rt_clk *clk, void *fw_data)
{
    clk->rate = 32768;

    return RT_EOK;
}

static const struct rt_clk_ops rk808_clkout1_ops =
{
    .init = rk808_clkout_init,
};

static rt_err_t rk808_clkout2_enable(struct rt_clk *clk, rt_bool_t enable)
{
    struct rk8xx_clkout *rk8xx_clkout = raw_to_rk8xx_clkout(clk->clk_np, 1);

    return rk8xx_update_bits(rk8xx_clkout->rk8xx, RK808_CLK32OUT_REG,
            CLK32KOUT2_EN, enable ? CLK32KOUT2_EN : 0);
}

static rt_err_t rk808_clkout2_prepare(struct rt_clk *clk)
{
    return rk808_clkout2_enable(clk, RT_TRUE);
}

static void rk808_clkout2_unprepare(struct rt_clk *clk)
{
    rk808_clkout2_enable(clk, RT_FALSE);
}

static rt_bool_t rk808_clkout2_is_prepared(struct rt_clk *clk)
{
    rt_uint32_t val;
    struct rk8xx_clkout *rk8xx_clkout = raw_to_rk8xx_clkout(clk->clk_np, 1);

    val = rk8xx_read(rk8xx_clkout->rk8xx, RK808_CLK32OUT_REG);

    if ((rt_err_t)val < 0)
    {
        return RT_FALSE;
    }

    return (val & CLK32KOUT2_EN) ? RT_TRUE : RT_FALSE;
}

static const struct rt_clk_ops rk808_clkout2_ops =
{
    .init = rk808_clkout_init,
    .prepare = rk808_clkout2_prepare,
    .unprepare = rk808_clkout2_unprepare,
    .is_prepared = rk808_clkout2_is_prepared,
};

static rt_err_t rk817_clkout2_enable(struct rt_clk *clk, rt_bool_t enable)
{
    struct rk8xx_clkout *rk8xx_clkout = raw_to_rk8xx_clkout(clk->clk_np, 1);

    return rk8xx_update_bits(rk8xx_clkout->rk8xx, RK817_SYS_CFG(1),
            RK817_CLK32KOUT2_EN, enable ? RK817_CLK32KOUT2_EN : 0);
}

static rt_err_t rk817_clkout2_prepare(struct rt_clk *clk)
{
    return rk817_clkout2_enable(clk, RT_TRUE);
}

static void rk817_clkout2_unprepare(struct rt_clk *clk)
{
    rk817_clkout2_enable(clk, RT_FALSE);
}

static rt_bool_t rk817_clkout2_is_prepared(struct rt_clk *clk)
{
    rt_uint32_t val;
    struct rk8xx_clkout *rk8xx_clkout = raw_to_rk8xx_clkout(clk->clk_np, 1);

    val = rk8xx_read(rk8xx_clkout->rk8xx, RK817_SYS_CFG(1));

    if ((rt_err_t)val < 0)
    {
        return RT_FALSE;
    }

    return (val & RK817_CLK32KOUT2_EN) ? RT_TRUE : RT_FALSE;
}

static const struct rt_clk_ops rk817_clkout2_ops =
{
    .init = rk808_clkout_init,
    .prepare = rk817_clkout2_prepare,
    .unprepare = rk817_clkout2_unprepare,
    .is_prepared = rk817_clkout2_is_prepared,
};

static rt_err_t rk8xx_clkout_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    struct rt_clk_node *clk_np;
    struct rk8xx *rk8xx = pdev->priv;
    struct rt_ofw_node *np = pdev->parent.ofw_node;
    struct rk8xx_clkout *rk8xx_clkout = rt_calloc(1, sizeof(*rk8xx_clkout));

    if (!rk8xx_clkout)
    {
        return -RT_ENOMEM;
    }

    rk8xx_clkout->rk8xx = rk8xx;

    /* clkout1 */
    clk_np = &rk8xx_clkout->parent[0];
    clk_np->ops = &rk808_clkout1_ops;
    clk_np->name = "rk8xx-clkout1";
    rt_ofw_prop_read_string(np, "clock-output-names", &clk_np->name);

    /* clkout2 */
    clk_np = &rk8xx_clkout->parent[1];
    switch (rk8xx->variant)
    {
    case RK809_ID:
    case RK817_ID:
        clk_np->ops = &rk817_clkout2_ops;
        break;

    /*
     * For the default case, it match the following PMIC type.
     * RK805_ID
     * RK808_ID
     * RK818_ID
     */
    default:
        clk_np->ops = &rk808_clkout2_ops;
        break;
    }
    clk_np->name = "rk8xx-clkout2";
    rt_ofw_prop_read_string(np, "clock-output-names", &clk_np->name);

    if ((err = rt_clk_register(&rk8xx_clkout->parent[0], RT_NULL)))
    {
        goto _fail;
    }

    if ((err = rt_clk_register(&rk8xx_clkout->parent[1], RT_NULL)))
    {
        rt_clk_unregister(&rk8xx_clkout->parent[0]);

        goto _fail;
    }

    rt_ofw_data(np) = &rk8xx_clkout->parent[0];

    return RT_EOK;

_fail:
    rt_free(rk8xx_clkout);

    return err;
}

static struct rt_platform_driver rk8xx_clkout_driver =
{
    .name = "rk8xx-clkout",
    .probe = rk8xx_clkout_probe,
};

static int rk8xx_clkout_register(void)
{
    rt_platform_driver_register(&rk8xx_clkout_driver);

    return 0;
}
INIT_SUBSYS_EXPORT(rk8xx_clkout_register);
