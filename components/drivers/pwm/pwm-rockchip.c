/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-3-08      GuEe-GUI     the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define PWM_CTRL_TIMER_EN       (1 << 0)
#define PWM_CTRL_OUTPUT_EN      (1 << 3)

#define PWM_ENABLE              (1 << 0)
#define PWM_CONTINUOUS          (1 << 1)
#define PWM_DUTY_POSITIVE       (1 << 3)
#define PWM_DUTY_NEGATIVE       (0 << 3)
#define PWM_INACTIVE_NEGATIVE   (0 << 4)
#define PWM_INACTIVE_POSITIVE   (1 << 4)
#define PWM_POLARITY_MASK       (PWM_DUTY_POSITIVE | PWM_INACTIVE_POSITIVE)
#define PWM_OUTPUT_LEFT         (0 << 5)
#define PWM_LOCK_EN             (1 << 6)
#define PWM_LP_DISABLE          (0 << 8)

#define NSEC_PER_SEC            1000000000L

struct rockchip_pwm_regs
{
    rt_ubase_t duty;
    rt_ubase_t period;
    rt_ubase_t cntr;
    rt_ubase_t ctrl;
};

struct rockchip_pwm_data
{
    struct rockchip_pwm_regs regs;

    rt_uint32_t prescaler;
    rt_bool_t supports_polarity;
    rt_bool_t supports_lock;
    rt_uint32_t enable_conf;
};

struct rockchip_pwm
{
    struct rt_device_pwm parent;
    void *base;

    struct rt_clk *clk;
    struct rt_clk *pclk;

    const struct rockchip_pwm_data *data;
};

#define raw_to_rockchip_pwm(raw) rt_container_of(raw, struct rockchip_pwm, parent)

static rt_err_t rockchip_pwm_enable(struct rockchip_pwm *rk_pwm, rt_bool_t enable)
{
    rt_uint32_t enable_conf = rk_pwm->data->enable_conf, val;

    if (enable)
    {
        rt_err_t err = rt_clk_enable(rk_pwm->clk);

        if (err)
        {
            return err;
        }
    }

    val = HWREG32(rk_pwm->base + rk_pwm->data->regs.ctrl);

    if (enable)
    {
        val |= enable_conf;
    }
    else
    {
        val &= ~enable_conf;
    }

    HWREG32(rk_pwm->base + rk_pwm->data->regs.ctrl) = val;

    if (!enable)
    {
        rt_clk_disable(rk_pwm->clk);
    }
    else
    {
        rt_pin_ctrl_confs_apply_by_name(&rk_pwm->parent.parent, "active");
    }

    return RT_EOK;
}

static void rockchip_pwm_config(struct rockchip_pwm *rk_pwm, struct rt_pwm_configuration *pwm_cfg)
{
    rt_ubase_t period, duty;
    rt_uint64_t clk_rate, div;
    rt_uint32_t ctrl;

    clk_rate = rt_clk_get_rate(rk_pwm->clk);

    /*
     * Since period and duty cycle registers have a width of 32 bits, every
     * possible input period can be obtained using the default prescaler value
     * for all practical clock rate values.
     */
    div = clk_rate * pwm_cfg->period;
    period = RT_DIV_ROUND_CLOSEST_ULL(div, rk_pwm->data->prescaler * NSEC_PER_SEC);

    div = clk_rate * rt_pwm_conf_duty_cycle(pwm_cfg);
    duty = RT_DIV_ROUND_CLOSEST_ULL(div, rk_pwm->data->prescaler * NSEC_PER_SEC);

    /*
     * Lock the period and duty of previous configuration, then change the duty
     * and period, that would not be effective.
     */
    ctrl = HWREG32(rk_pwm->base + rk_pwm->data->regs.ctrl);

    if (rk_pwm->data->supports_lock)
    {
        ctrl |= PWM_LOCK_EN;
        HWREG32(rk_pwm->base + rk_pwm->data->regs.ctrl) = ctrl;
    }

    HWREG32(rk_pwm->base + rk_pwm->data->regs.period) = period;
    HWREG32(rk_pwm->base + rk_pwm->data->regs.duty) = duty;

    if (rk_pwm->data->supports_polarity)
    {
        ctrl &= ~PWM_POLARITY_MASK;

        if (pwm_cfg->complementary)
        {
            ctrl |= PWM_DUTY_NEGATIVE | PWM_INACTIVE_POSITIVE;
        }
        else
        {
            ctrl |= PWM_DUTY_POSITIVE | PWM_INACTIVE_NEGATIVE;
        }
    }

    /*
     * Unlock and set polarity at the same time, the configuration of duty,
     * period and polarity would be effective together at next period.
     */
    if (rk_pwm->data->supports_lock)
    {
        ctrl &= ~PWM_LOCK_EN;
    }

    HWREG32(rk_pwm->base + rk_pwm->data->regs.ctrl) = ctrl;
}

static rt_err_t rockchip_pwm_get_state(struct rockchip_pwm *rk_pwm, struct rt_pwm_configuration *pwm_cfg)
{
    rt_err_t err;
    rt_uint32_t val;
    rt_uint64_t tmp;
    rt_ubase_t clk_rate;

    if ((err = rt_clk_enable(rk_pwm->pclk)))
    {
        return err;
    }

    if ((err = rt_clk_enable(rk_pwm->clk)))
    {
        return err;
    }

    clk_rate = rt_clk_get_rate(rk_pwm->clk);

    tmp = HWREG32(rk_pwm->base + rk_pwm->data->regs.period);
    tmp *= rk_pwm->data->prescaler * NSEC_PER_SEC;
    pwm_cfg->period = RT_DIV_ROUND_CLOSEST_ULL(tmp, clk_rate);

    tmp = HWREG32(rk_pwm->base + rk_pwm->data->regs.duty);
    tmp *= rk_pwm->data->prescaler * NSEC_PER_SEC;
    pwm_cfg->pulse = rt_pwm_conf_pulse(pwm_cfg, RT_DIV_ROUND_CLOSEST_ULL(tmp, clk_rate));

    val = HWREG32(rk_pwm->base + rk_pwm->data->regs.ctrl);

    if (rk_pwm->data->supports_polarity && !(val & PWM_DUTY_POSITIVE))
    {
        pwm_cfg->complementary = RT_TRUE;
    }
    else
    {
        pwm_cfg->complementary = RT_FALSE;
    }

    rt_clk_disable(rk_pwm->clk);
    rt_clk_disable(rk_pwm->pclk);

    return RT_EOK;
}

static rt_err_t rockchip_pwm_control(struct rt_device_pwm *pwm, int cmd, void *args)
{
    rt_err_t err = RT_EOK;
    struct rockchip_pwm *rk_pwm = raw_to_rockchip_pwm(pwm);
    struct rt_pwm_configuration *pwm_cfg = args;

    rt_clk_enable(rk_pwm->pclk);

    /* RT_PWM framework have check args */
    switch (cmd)
    {
    case PWM_CMD_ENABLE:
        err = rockchip_pwm_enable(rk_pwm, RT_TRUE);
        break;

    case PWM_CMD_DISABLE:
        err = rockchip_pwm_enable(rk_pwm, RT_FALSE);
        break;

    case PWM_CMD_SET:
    case PWM_CMD_SET_PERIOD:
    case PWM_CMD_SET_PULSE:
        rockchip_pwm_config(rk_pwm, pwm_cfg);
        break;

    case PWM_CMD_GET:
        err = rockchip_pwm_get_state(rk_pwm, pwm_cfg);
        break;

    default:
        err = -RT_EINVAL;
        break;
    }

    rt_clk_disable(rk_pwm->pclk);

    return err;
}

const static struct rt_pwm_ops rockchip_pwm_ops =
{
    .control = rockchip_pwm_control,
};

static void rockchip_pwm_free(struct rockchip_pwm *rk_pwm)
{
    if (rk_pwm->base)
    {
        rt_iounmap(rk_pwm->base);
    }

    if (!rt_is_err_or_null(rk_pwm->clk))
    {
        rt_clk_disable_unprepare(rk_pwm->clk);
        rt_clk_put(rk_pwm->clk);
    }

    if (!rt_is_err_or_null(rk_pwm->pclk))
    {
        rt_clk_disable_unprepare(rk_pwm->pclk);
        rt_clk_put(rk_pwm->pclk);
    }

    rt_free(rk_pwm);
}

static rt_err_t rockchip_pwm_probe(struct rt_platform_device *pdev)
{
    rt_err_t err = RT_EOK;
    rt_bool_t enabled;
    rt_uint32_t enable_conf, ctrl;
    struct rt_device *dev = &pdev->parent;
    struct rockchip_pwm *rk_pwm = rt_calloc(1, sizeof(*rk_pwm));
    const struct rockchip_pwm_data *pdata = pdev->id->data;

    if (!rk_pwm)
    {
        return -RT_ENOMEM;
    }

    rk_pwm->data = pdata;
    rk_pwm->base = rt_dm_dev_iomap(dev, 0);

    if (!rk_pwm->base)
    {
        err = -RT_EIO;
        goto _free_res;
    }

    rk_pwm->clk = rt_clk_get_by_name(dev, "pwm");

    if (rt_is_err(rk_pwm->clk))
    {
        err = rt_ptr_err(rk_pwm->clk);
        goto _free_res;
    }

#ifdef RT_USING_OFW
    if (rt_ofw_count_phandle_cells(dev->ofw_node, "clocks", "#clock-cells") == 2)
    {
        rk_pwm->pclk = rt_clk_get_by_name(dev, "pclk");

        if (rt_is_err(rk_pwm->pclk))
        {
            err = rt_ptr_err(rk_pwm->pclk);
            goto _free_res;
        }
    }
    else
#endif /* RT_USING_OFW */
    {
        rk_pwm->pclk = rk_pwm->clk;
    }

    if ((err = rt_clk_prepare_enable(rk_pwm->clk)))
    {
        goto _free_res;
    }

    if ((err = rt_clk_prepare_enable(rk_pwm->pclk)))
    {
        goto _free_res;
    }

    enable_conf = rk_pwm->data->enable_conf;
    ctrl = HWREG32(rk_pwm->base + rk_pwm->data->regs.ctrl);
    enabled = (ctrl & enable_conf) == enable_conf;

    /* Keep the PWM clk enabled if the PWM appears to be up and running. */
    if (!enabled)
    {
        rt_clk_disable(rk_pwm->clk);
    }

    rt_clk_disable(rk_pwm->pclk);

    dev->user_data = rk_pwm;

    rk_pwm->parent.parent.ofw_node = dev->ofw_node;

    rt_dm_dev_set_name_auto(&rk_pwm->parent.parent, "pwm");
    rt_device_pwm_register(&rk_pwm->parent, rt_dm_dev_get_name(&rk_pwm->parent.parent), &rockchip_pwm_ops, rk_pwm);

    rt_dm_dev_bind_fwdata(dev, RT_NULL, rk_pwm);

    return RT_EOK;

_free_res:
    rockchip_pwm_free(rk_pwm);

    return err;
}

static rt_err_t rockchip_pwm_remove(struct rt_platform_device *pdev)
{
    struct rt_device *dev = &pdev->parent;
    struct rockchip_pwm *rk_pwm = dev->user_data;

    rt_dm_dev_unbind_fwdata(dev, RT_NULL);

    rockchip_pwm_enable(rk_pwm, RT_FALSE);

    rt_device_unregister(&rk_pwm->parent.parent);

    rockchip_pwm_free(rk_pwm);

    return RT_EOK;
}

static const struct rockchip_pwm_data pwm_data_v1 =
{
    .regs =
    {
        .duty = 0x04,
        .period = 0x08,
        .cntr = 0x00,
        .ctrl = 0x0c,
    },
    .prescaler = 2,
    .supports_polarity = RT_FALSE,
    .supports_lock = RT_FALSE,
    .enable_conf = PWM_CTRL_OUTPUT_EN | PWM_CTRL_TIMER_EN,
};

static const struct rockchip_pwm_data pwm_data_v2 =
{
    .regs =
    {
        .duty = 0x08,
        .period = 0x04,
        .cntr = 0x00,
        .ctrl = 0x0c,
    },
    .prescaler = 1,
    .supports_polarity = RT_TRUE,
    .supports_lock = RT_FALSE,
    .enable_conf = PWM_OUTPUT_LEFT | PWM_LP_DISABLE | PWM_ENABLE | PWM_CONTINUOUS,
};

static const struct rockchip_pwm_data pwm_data_vop =
{
    .regs =
    {
        .duty = 0x08,
        .period = 0x04,
        .cntr = 0x0c,
        .ctrl = 0x00,
    },
    .prescaler = 1,
    .supports_polarity = RT_TRUE,
    .supports_lock = RT_FALSE,
    .enable_conf = PWM_OUTPUT_LEFT | PWM_LP_DISABLE | PWM_ENABLE | PWM_CONTINUOUS,
};

static const struct rockchip_pwm_data pwm_data_v3 =
{
    .regs =
    {
        .duty = 0x08,
        .period = 0x04,
        .cntr = 0x00,
        .ctrl = 0x0c,
    },
    .prescaler = 1,
    .supports_polarity = RT_TRUE,
    .supports_lock = RT_TRUE,
    .enable_conf = PWM_OUTPUT_LEFT | PWM_LP_DISABLE | PWM_ENABLE | PWM_CONTINUOUS,
};

static const struct rt_ofw_node_id rockchip_pwm_ofw_ids[] =
{
    { .compatible = "rockchip,rk2928-pwm", .data = &pwm_data_v1 },
    { .compatible = "rockchip,rk3288-pwm", .data = &pwm_data_v2 },
    { .compatible = "rockchip,vop-pwm", .data = &pwm_data_vop },
    { .compatible = "rockchip,rk3328-pwm", .data = &pwm_data_v3 },
    { /* sentinel */ }
};

static struct rt_platform_driver rockchip_pwm_driver =
{
    .name = "rockchip-pwm",
    .ids = rockchip_pwm_ofw_ids,

    .probe = rockchip_pwm_probe,
    .remove = rockchip_pwm_remove,
};

static int rockchip_pwm_register(void)
{
    rt_platform_driver_register(&rockchip_pwm_driver);

    return 0;
}
INIT_SUBSYS_LATER_EXPORT(rockchip_pwm_register);
