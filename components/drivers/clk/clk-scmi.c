/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-26     GuEe-GUI     first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "clk.scmi"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

struct scmi_clk
{
    struct rt_clk_node parent;

    struct rt_scmi_device *sdev;
};

#define raw_to_scmi_clk(raw) rt_container_of(raw, struct scmi_clk, parent)

static rt_err_t scmi_clk_op_gate(struct scmi_clk *sclk, int clk_id, rt_bool_t enable)
{
    struct scmi_clk_state_in in =
    {
        .clock_id = rt_cpu_to_le32(clk_id),
        .attributes = rt_cpu_to_le32(enable),
    };
    struct scmi_clk_state_out out;
    struct rt_scmi_msg msg = RT_SCMI_MSG_IN(SCMI_CLOCK_CONFIG_SET, in, out);

    return rt_scmi_process_msg(sclk->sdev, &msg);
}

static rt_base_t scmi_clk_op_get_rate(struct scmi_clk *sclk, int clk_id)
{
    rt_ubase_t res;
    struct scmi_clk_rate_get_in in =
    {
        .clock_id = rt_cpu_to_le32(clk_id),
    };
    struct scmi_clk_rate_get_out out;
    struct rt_scmi_msg msg = RT_SCMI_MSG_IN(SCMI_CLOCK_RATE_GET, in, out);

    res = rt_scmi_process_msg(sclk->sdev, &msg);

    if ((rt_base_t)res >= 0)
    {
        res = (rt_ubase_t)(((rt_uint64_t)out.rate_msb << 32) | out.rate_lsb);
    }

    return res;
}

static rt_base_t scmi_clk_op_set_rate(struct scmi_clk *sclk, int clk_id, rt_ubase_t rate)
{
    struct scmi_clk_rate_set_in in =
    {
        .clock_id = rt_cpu_to_le32(clk_id),
        .flags = rt_cpu_to_le32(SCMI_CLK_RATE_ROUND_CLOSEST),
        .rate_lsb = rt_cpu_to_le32((rt_uint32_t)rate),
        .rate_msb = rt_cpu_to_le32((rt_uint32_t)((rt_uint64_t)rate >> 32)),
    };
    struct scmi_clk_rate_set_out out;
    struct rt_scmi_msg msg = RT_SCMI_MSG_IN(SCMI_CLOCK_RATE_SET, in, out);

    return rt_scmi_process_msg(sclk->sdev, &msg);
}

static rt_err_t scmi_clk_init(struct rt_clk *clk, void *fw_data)
{
    struct scmi_clk *sclk = raw_to_scmi_clk(clk->clk_np);
    struct rt_ofw_cell_args *args = fw_data;
    rt_ubase_t clk_id = args->args[0];

    clk->rate = scmi_clk_op_get_rate(sclk, clk_id);
    clk->priv = (void *)clk_id;

    return RT_EOK;
}

static rt_err_t scmi_clk_enable(struct rt_clk *clk)
{
    struct scmi_clk *sclk = raw_to_scmi_clk(clk->clk_np);

    return scmi_clk_op_gate(sclk, (rt_ubase_t)clk->priv, RT_TRUE);
}

static void scmi_clk_disable(struct rt_clk *clk)
{
    struct scmi_clk *sclk = raw_to_scmi_clk(clk->clk_np);

    scmi_clk_op_gate(sclk, (rt_ubase_t)clk->priv, RT_FALSE);
}

static rt_err_t scmi_clk_set_rate(struct rt_clk *clk, rt_ubase_t rate, rt_ubase_t parent_rate)
{
    struct scmi_clk *sclk = raw_to_scmi_clk(clk->clk_np);

    return scmi_clk_op_set_rate(sclk, (rt_ubase_t)clk->priv, rate);
}

static const struct rt_clk_ops scmi_clk_ops =
{
    .init = scmi_clk_init,
    .enable = scmi_clk_enable,
    .disable = scmi_clk_disable,
    .set_rate = scmi_clk_set_rate,
};

static rt_err_t scmi_clk_probe(struct rt_scmi_device *sdev)
{
    rt_err_t err;
    struct rt_clk_node *clk_np;
    struct scmi_clk *sclk = rt_calloc(1, sizeof(*sclk));

    if (!sclk)
    {
        return -RT_ENOMEM;
    }

    sclk->sdev = sdev;

    clk_np = &sclk->parent;
    clk_np->ops = &scmi_clk_ops;

    if ((err = rt_clk_register(clk_np, RT_NULL)))
    {
        rt_free(sclk);

        goto _end;
    }

    rt_dm_dev_bind_fwdata(&sdev->parent, RT_NULL, &sclk->parent);

_end:
    return err;
}

static const struct rt_scmi_device_id scmi_clk_ids[] =
{
    { SCMI_PROTOCOL_ID_CLOCK, "clocks" },
    { /* sentinel */ },
};

static struct rt_scmi_driver scmi_clk_driver =
{
    .name = "clk-scmi",
    .ids = scmi_clk_ids,

    .probe = scmi_clk_probe,
};
RT_SCMI_DRIVER_EXPORT(scmi_clk_driver);
