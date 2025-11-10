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

struct scmi_clk_data
{
    int id;
    rt_bool_t rate_discrete;

    union
    {
        struct
        {
            int rates_nr;
            rt_uint64_t rates[];
        } list;
        struct
        {
            rt_uint64_t min_rate;
            rt_uint64_t max_rate;
            rt_uint64_t step_size;
        } range;
    } info;
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
    struct rt_scmi_msg msg = RT_SCMI_MSG_IN(SCMI_CLOCK_CONFIG_SET, &in, &out);

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
    struct rt_scmi_msg msg = RT_SCMI_MSG_IN(SCMI_CLOCK_RATE_GET, &in, &out);

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
    struct rt_scmi_msg msg = RT_SCMI_MSG_IN(SCMI_CLOCK_RATE_SET, &in, &out);

    return rt_scmi_process_msg(sclk->sdev, &msg);
}

static rt_err_t scmi_clk_init(struct rt_clk *clk, void *fw_data)
{
    rt_err_t err;
    struct scmi_clk *sclk = raw_to_scmi_clk(clk->clk_np);
    struct rt_ofw_cell_args *args = fw_data;
    struct scmi_clk_data *clk_data = RT_NULL;
    struct scmi_clk_describe_rates_out *out;
    struct scmi_clk_describe_rates_in in;
    struct rt_scmi_msg msg;
    rt_ubase_t clk_id = args->args[0];
    rt_uint32_t flags, rates_nr, rate_discrete;

    out = rt_malloc(rt_offsetof(struct scmi_clk_describe_rates_out,
            rate[SCMI_MAX_NUM_RATES]));

    if (!out)
    {
        err = -RT_ENOMEM;
        goto _end;
    }

    in.id = rt_cpu_to_le32(clk_id);
    in.rate_index = rt_cpu_to_le32(0);
    msg = RT_SCMI_MSG_IN(SCMI_CLOCK_DESCRIBE_RATES, &in, out);

    if ((err = rt_scmi_process_msg(sclk->sdev, &msg)))
    {
        goto _end;
    }

    flags = rt_le32_to_cpu(out->num_rates_flags);
    rates_nr = SCMI_NUM_REMAINING(flags);
    rate_discrete = SCMI_RATE_DISCRETE(flags);

    if (rate_discrete)
    {
        clk_data = rt_malloc(rt_offsetof(struct scmi_clk_data,
                info.list.rates[SCMI_MAX_NUM_RATES]));
    }
    else
    {
        clk_data = rt_malloc(sizeof(*clk_data));
    }

    if (!clk_data)
    {
        err = -RT_ENOMEM;
        goto _end;
    }

    if (rate_discrete)
    {
        for (int i = 0; i < rates_nr; ++i)
        {
            clk_data->info.list.rates[i] = SCMI_RATE_TO_U64(out->rate[i]);
        }

        clk_data->info.list.rates_nr = rates_nr;
    }
    else
    {
        clk_data->info.range.min_rate = SCMI_RATE_TO_U64(out->rate[0]);
        clk_data->info.range.max_rate = SCMI_RATE_TO_U64(out->rate[1]);
        clk_data->info.range.step_size = SCMI_RATE_TO_U64(out->rate[2]);
    }

    clk_data->rate_discrete = rate_discrete;
    clk_data->id = clk_id;
    clk->rate = scmi_clk_op_get_rate(sclk, clk_id);
    clk->priv = clk_data;

_end:
    if (err && clk_data)
    {
        rt_free(clk_data);
    }

    rt_free(out);

    return err;
}

static rt_err_t scmi_clk_enable(struct rt_clk *clk)
{
    struct scmi_clk_data *clk_data = clk->priv;
    struct scmi_clk *sclk = raw_to_scmi_clk(clk->clk_np);

    return scmi_clk_op_gate(sclk, clk_data->id, RT_TRUE);
}

static void scmi_clk_disable(struct rt_clk *clk)
{
    struct scmi_clk_data *clk_data = clk->priv;
    struct scmi_clk *sclk = raw_to_scmi_clk(clk->clk_np);

    scmi_clk_op_gate(sclk, clk_data->id, RT_FALSE);
}

static rt_err_t scmi_clk_set_rate(struct rt_clk *clk, rt_ubase_t rate, rt_ubase_t parent_rate)
{
    rt_err_t err;
    rt_ubase_t res_rate;
    struct scmi_clk_data *clk_data = clk->priv;
    struct scmi_clk *sclk = raw_to_scmi_clk(clk->clk_np);

    if (!(err = scmi_clk_op_set_rate(sclk, clk_data->id, rate)))
    {
        res_rate = scmi_clk_op_get_rate(sclk, clk_data->id);

        if ((rt_base_t)res_rate > 0)
        {
            clk->rate = res_rate;

            return RT_EOK;
        }

        err = (rt_err_t)res_rate;
    }

    return err;
}

static rt_base_t scmi_clk_round_rate(struct rt_clk *clk, rt_ubase_t drate, rt_ubase_t *prate)
{
    rt_uint64_t fmin, fmax, ftmp;
    struct scmi_clk_data *clk_data = clk->priv;

    if (clk_data->rate_discrete)
    {
        fmin = clk_data->info.list.rates[0];
        fmax = clk_data->info.list.rates[clk_data->info.list.rates_nr - 1];
    }
    else
    {
        fmin = clk_data->info.range.min_rate;
        fmax = clk_data->info.range.max_rate;
    }

    fmin = clk->clk_np->min_rate;
    fmax = clk->clk_np->max_rate;

    if (drate <= fmin)
    {
        return fmin;
    }

    if (drate >= fmax)
    {
        return fmax;
    }

    if (clk_data->rate_discrete)
    {
        for (int i = 0; i < clk_data->info.list.rates_nr; ++i)
        {
            if (drate >= clk_data->info.list.rates[i])
            {
                drate = clk_data->info.list.rates[i];
                break;
            }
        }

        return drate;
    }

    ftmp = drate - fmin;
    ftmp += clk_data->info.range.step_size - 1;
    rt_do_div(ftmp, clk_data->info.range.step_size);

    return ftmp * clk_data->info.range.step_size + fmin;
}

static const struct rt_clk_ops scmi_clk_ops =
{
    .init = scmi_clk_init,
    .enable = scmi_clk_enable,
    .disable = scmi_clk_disable,
    .set_rate = scmi_clk_set_rate,
    .round_rate = scmi_clk_round_rate,
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
