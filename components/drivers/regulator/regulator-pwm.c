/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-26     GuEe-GUI     first version
 */

#define DBG_TAG "regulator.pwm"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "regulator_dm.h"

struct pwm_continuous_reg_data
{
    rt_uint32_t min_uvolt_dutycycle;
    rt_uint32_t max_uvolt_dutycycle;
    rt_uint32_t dutycycle_unit;
};

struct pwm_voltages
{
    rt_uint32_t uvolt;
    rt_uint32_t dutycycle;
};

struct pwm_regulator
{
    struct rt_regulator_node parent;
    struct rt_regulator_param param;

    rt_bool_t enabled;
    struct rt_device_pwm *pwm_dev;
    struct rt_pwm_configuration pwm_conf;
    struct pwm_voltages *duty_cycle_table;
    struct pwm_continuous_reg_data continuous;

    int selector;
    rt_uint32_t n_voltages;

    rt_base_t enable_pin;
};

#define raw_to_pwm_regulator(raw) rt_container_of(raw, struct pwm_regulator, parent)

static rt_err_t pwm_regulator_enable(struct rt_regulator_node *reg_np)
{
    rt_err_t err;
    struct pwm_regulator *pr = raw_to_pwm_regulator(reg_np);

    if (pr->enable_pin >= 0)
    {
        rt_pin_write(pr->enable_pin, PIN_HIGH);
    }

    if (!(err = rt_pwm_enable(pr->pwm_dev, pr->pwm_conf.channel)))
    {
        pr->enabled = RT_TRUE;
    }

    return err;
}

static rt_err_t pwm_regulator_disable(struct rt_regulator_node *reg_np)
{
    rt_err_t err;
    struct pwm_regulator *pr = raw_to_pwm_regulator(reg_np);

    if ((err = rt_pwm_disable(pr->pwm_dev, pr->pwm_conf.channel)))
    {
        return err;
    }

    pr->enabled = RT_FALSE;

    if (pr->enable_pin >= 0)
    {
        rt_pin_write(pr->enable_pin, PIN_LOW);
    }

    return RT_EOK;
}

static rt_bool_t pwm_regulator_is_enabled(struct rt_regulator_node *reg_np)
{
    struct pwm_regulator *pr = raw_to_pwm_regulator(reg_np);

    if (pr->enable_pin >= 0 && rt_pin_read(pr->enable_pin) == PIN_LOW)
    {
        return RT_FALSE;
    }

    return pr->enabled;
}

static rt_err_t pwm_regulator_table_set_voltage(struct rt_regulator_node *reg_np,
        int min_uvolt, int max_uvolt)
{
    rt_err_t err;
    int selector = -1;
    rt_uint32_t duty_cycle;
    struct rt_pwm_configuration pwm_conf = {};
    struct pwm_regulator *pr = raw_to_pwm_regulator(reg_np);

    for (int i = 0; i < pr->n_voltages; ++i)
    {
        int uvolt = pr->duty_cycle_table[i].uvolt;

        if (uvolt >= min_uvolt && uvolt <= max_uvolt)
        {
            selector = i;
            break;
        }
    }

    if (selector < 0)
    {
        return -RT_EINVAL;
    }

    rt_pwm_get(pr->pwm_dev, &pwm_conf);
    pwm_conf.period = pr->pwm_conf.period;
    pwm_conf.complementary = pr->pwm_conf.complementary;

    duty_cycle = pr->duty_cycle_table[selector].dutycycle;
    duty_cycle = RT_DIV_ROUND_CLOSEST_ULL((rt_uint64_t)duty_cycle * pwm_conf.period, 100);

    pwm_conf.pulse = rt_pwm_conf_pulse(&pwm_conf, duty_cycle);

    if ((err = rt_pwm_set(pr->pwm_dev, pwm_conf.channel, pwm_conf.period, pwm_conf.pulse)))
    {
        return err;
    }

    pr->selector = selector;

    return RT_EOK;
}

static int pwm_regulator_table_get_voltage(struct rt_regulator_node *reg_np)
{
    struct pwm_regulator *pr = raw_to_pwm_regulator(reg_np);

    return pr->duty_cycle_table[pr->selector].uvolt;
}

static rt_err_t pwm_regulator_continuous_set_voltage(struct rt_regulator_node *reg_np,
        int min_uvolt, int max_uvolt)
{
    struct rt_pwm_configuration pwm_conf = {};
    struct pwm_regulator *pr = raw_to_pwm_regulator(reg_np);
    rt_uint32_t min_uvolt_duty = pr->continuous.min_uvolt_dutycycle;
    rt_uint32_t max_uvolt_duty = pr->continuous.max_uvolt_dutycycle;
    rt_uint32_t duty_unit = pr->continuous.dutycycle_unit, diff_duty, dutycycle;
    int fix_min_uvolt = reg_np->param->min_uvolt;
    int fix_max_uvolt = reg_np->param->max_uvolt;
    int diff_uvolt = fix_max_uvolt - fix_min_uvolt;

    rt_pwm_get(pr->pwm_dev, &pr->pwm_conf);
    pwm_conf.period = pr->pwm_conf.period;
    pwm_conf.complementary = pr->pwm_conf.complementary;

    if (max_uvolt_duty < min_uvolt_duty)
    {
        diff_duty = min_uvolt_duty - max_uvolt_duty;
    }
    else
    {
        diff_duty = max_uvolt_duty - min_uvolt_duty;
    }

    dutycycle = RT_DIV_ROUND_CLOSEST_ULL(
            (rt_uint64_t)(min_uvolt - fix_min_uvolt) * diff_duty, diff_uvolt);

    if (max_uvolt_duty < min_uvolt_duty)
    {
        dutycycle = min_uvolt_duty - dutycycle;
    }
    else
    {
        dutycycle = min_uvolt_duty + dutycycle;
    }

    pr->pwm_conf.pulse = rt_pwm_conf_pulse(&pwm_conf,
            RT_DIV_ROUND_CLOSEST_ULL((rt_uint64_t)dutycycle * pwm_conf.period, duty_unit));

    return rt_pwm_set(pr->pwm_dev, pwm_conf.channel, pwm_conf.period, pwm_conf.pulse);
}

static int pwm_regulator_continuous_get_voltage(struct rt_regulator_node *reg_np)
{
    struct pwm_regulator *pr = raw_to_pwm_regulator(reg_np);
    rt_uint32_t min_uvolt_duty = pr->continuous.min_uvolt_dutycycle;
    rt_uint32_t max_uvolt_duty = pr->continuous.max_uvolt_dutycycle;
    rt_uint32_t duty_unit = pr->continuous.dutycycle_unit, diff_duty;
    int min_uvolt = reg_np->param->min_uvolt;
    int max_uvolt = reg_np->param->max_uvolt;
    int uvolt, diff_uvolt = max_uvolt - min_uvolt;

    rt_pwm_get(pr->pwm_dev, &pr->pwm_conf);

    uvolt = pr->pwm_conf.period ? RT_DIV_ROUND_CLOSEST_ULL(
            (rt_uint64_t)rt_pwm_conf_duty_cycle(&pr->pwm_conf) * duty_unit,
            pr->pwm_conf.period) : 0;

    if (max_uvolt_duty < min_uvolt_duty)
    {
        uvolt = min_uvolt_duty - uvolt;
        diff_duty = min_uvolt_duty - max_uvolt_duty;
    }
    else
    {
        uvolt = uvolt - min_uvolt_duty;
        diff_duty = max_uvolt_duty - min_uvolt_duty;
    }

    uvolt = RT_DIV_ROUND_CLOSEST_ULL((rt_uint64_t)uvolt * diff_uvolt, diff_duty);

    return uvolt + min_uvolt;
}

static const struct rt_regulator_ops pwm_regulator_voltage_table_ops =
{
    .enable = pwm_regulator_enable,
    .disable = pwm_regulator_disable,
    .is_enabled = pwm_regulator_is_enabled,
    .set_voltage = pwm_regulator_table_set_voltage,
    .get_voltage = pwm_regulator_table_get_voltage,
};

static const struct rt_regulator_ops pwm_regulator_voltage_continuous_ops =
{
    .enable = pwm_regulator_enable,
    .disable = pwm_regulator_disable,
    .is_enabled = pwm_regulator_is_enabled,
    .set_voltage = pwm_regulator_continuous_set_voltage,
    .get_voltage = pwm_regulator_continuous_get_voltage,
};

static rt_err_t pwm_regulator_init_table(struct rt_ofw_node *np,
        struct pwm_regulator *pr)
{
    rt_err_t err;
    rt_ssize_t length = 0;
    rt_uint32_t dutycycle;
    struct pwm_voltages *duty_cycle_table;

    rt_ofw_prop_read_raw(np, "voltage-table", &length);

    if ((length < sizeof(*duty_cycle_table)) ||
        (length % sizeof(*duty_cycle_table)))
    {
        LOG_E("`voltage-table` length = %d is invalid", length);

        return -RT_EINVAL;
    }

    duty_cycle_table = rt_calloc(1, length);

    if (!duty_cycle_table)
    {
        return -RT_ENOMEM;
    }

    err = rt_ofw_prop_read_u32_array_index(np, "voltage-table",
            0, length / sizeof(rt_uint32_t), (rt_uint32_t *)duty_cycle_table);

    if (err < 0)
    {
        rt_free(duty_cycle_table);

        return err;
    }

    rt_pwm_get(pr->pwm_dev, &pr->pwm_conf);

    pr->selector = -1;
    pr->n_voltages = length / sizeof(*duty_cycle_table);

    dutycycle = pr->pwm_conf.period ? RT_DIV_ROUND_CLOSEST_ULL(
            (rt_uint64_t)rt_pwm_conf_duty_cycle(&pr->pwm_conf) * 100,
                pr->pwm_conf.period) : 0;

    for (int i = 0; i < pr->n_voltages; ++i)
    {
        if (dutycycle == pr->duty_cycle_table[i].dutycycle)
        {
            pr->selector = i;
            break;
        }
    }

    if (pr->selector < 0)
    {
        rt_free(duty_cycle_table);

        return -RT_EINVAL;
    }


    pr->parent.ops = &pwm_regulator_voltage_table_ops;

    return RT_EOK;
}

static rt_err_t pwm_regulator_init_continuous(struct rt_ofw_node *np,
        struct pwm_regulator *pr)
{
    rt_uint32_t dutycycle_unit = 100, dutycycle_range[2] = { 0, 100 };

    rt_ofw_prop_read_u32_array_index(np, "pwm-dutycycle-range", 0, 2,
            dutycycle_range);
    rt_ofw_prop_read_u32(np, "pwm-dutycycle-unit", &dutycycle_unit);

    if (dutycycle_range[0] > dutycycle_unit ||
        dutycycle_range[1] > dutycycle_unit)
    {
        return -RT_EINVAL;
    }

    pr->continuous.dutycycle_unit = dutycycle_unit;
    pr->continuous.min_uvolt_dutycycle = dutycycle_range[0];
    pr->continuous.max_uvolt_dutycycle = dutycycle_range[1];

    pr->parent.ops = &pwm_regulator_voltage_continuous_ops;

    return RT_EOK;
}

static rt_err_t pwm_regulator_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    struct rt_ofw_cell_args pwm_args;
    struct rt_ofw_node *np = pdev->parent.ofw_node;
    struct pwm_regulator *pr = rt_calloc(1, sizeof(*pr));
    struct rt_regulator_node *rgp;

    if (!pr)
    {
        return -RT_ENOMEM;
    }

    if ((err = regulator_ofw_parse(np, &pr->param)))
    {
        goto _fail;
    }

    if (rt_ofw_parse_phandle_cells(np, "pwms", "#pwm-cells", 0, &pwm_args))
    {
        goto _fail;
    }

    pr->pwm_dev = rt_ofw_data(pwm_args.data);
    rt_ofw_node_put(pwm_args.data);

    if (!pr->pwm_dev)
    {
        goto _fail;
    }

    pr->pwm_conf.channel = pwm_args.args[0];
    pr->pwm_conf.period = pwm_args.args[1];
    pr->pwm_conf.complementary = pwm_args.args[2];

    pr->enable_pin = rt_ofw_get_named_pin(np, "enable", 0, RT_NULL, RT_NULL);

    if (pr->enable_pin < 0 && pr->enable_pin != -RT_EEMPTY)
    {
        err = pr->enable_pin;
        goto _fail;
    }

    rgp = &pr->parent;
    rgp->supply_name = pr->param.name;
    rgp->param = &pr->param;
    rgp->dev = &pdev->parent;

    if (rt_ofw_prop_read_bool(np, "voltage-table"))
    {
        err = pwm_regulator_init_table(np, pr);
    }
    else
    {
        err = pwm_regulator_init_continuous(np, pr);
    }

    if (err || (err = rt_regulator_register(rgp)))
    {
        goto _fail;
    }

    return RT_EOK;

_fail:
    rt_free(pr);

    return err;
}

static const struct rt_ofw_node_id pwm_regulator_ofw_ids[] =
{
    { .compatible = "pwm-regulator" },
    { /* sentinel */ }
};

static struct rt_platform_driver pwm_regulator_driver =
{
    .name = "pwm-regulator",
    .ids = pwm_regulator_ofw_ids,

    .probe = pwm_regulator_probe,
};

static int pwm_regulator_register(void)
{
    rt_platform_driver_register(&pwm_regulator_driver);

    return 0;
}
INIT_DRIVER_EARLY_EXPORT(pwm_regulator_register);
