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

#define DBG_TAG "soc.rockchip.grf"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "rockchip.h"

struct rockchip_grf_value
{
    const char *desc;
    rt_uint32_t reg;
    rt_uint32_t val;
};

struct rockchip_grf_info
{
    const struct rockchip_grf_value *values;
    int values_nr;
};

#define RK3036_GRF_SOC_CON0 0x140

static const struct rockchip_grf_value rk3036_defaults[] =
{
    /*
     * Disable auto jtag/sdmmc switching that causes issues with the
     * clock-framework and the mmc controllers making them unreliable.
     */
    { "jtag switching", RK3036_GRF_SOC_CON0, HIWORD_UPDATE(0, 1, 11) },
};

static const struct rockchip_grf_info rk3036_grf =
{
    .values = rk3036_defaults,
    .values_nr = RT_ARRAY_SIZE(rk3036_defaults),
};

#define RK3128_GRF_SOC_CON0 0x140

static const struct rockchip_grf_value rk3128_defaults[] =
{
    { "jtag switching", RK3128_GRF_SOC_CON0, HIWORD_UPDATE(0, 1, 8) },
};

static const struct rockchip_grf_info rk3128_grf =
{
    .values = rk3128_defaults,
    .values_nr = RT_ARRAY_SIZE(rk3128_defaults),
};

#define RK3228_GRF_SOC_CON6 0x418

static const struct rockchip_grf_value rk3228_defaults[] =
{
    { "jtag switching", RK3228_GRF_SOC_CON6, HIWORD_UPDATE(0, 1, 8) },
};

static const struct rockchip_grf_info rk3228_grf =
{
    .values = rk3228_defaults,
    .values_nr = RT_ARRAY_SIZE(rk3228_defaults),
};

#define RK3288_GRF_SOC_CON0 0x244
#define RK3288_GRF_SOC_CON2 0x24c

static const struct rockchip_grf_value rk3288_defaults[] =
{
    { "jtag switching", RK3288_GRF_SOC_CON0, HIWORD_UPDATE(0, 1, 12) },
    { "pwm select", RK3288_GRF_SOC_CON2, HIWORD_UPDATE(1, 1, 0) },
};

static const struct rockchip_grf_info rk3288_grf =
{
    .values = rk3288_defaults,
    .values_nr = RT_ARRAY_SIZE(rk3288_defaults),
};

#define RK3328_GRF_SOC_CON4 0x410

static const struct rockchip_grf_value rk3328_defaults[] =
{
    { "jtag switching", RK3328_GRF_SOC_CON4, HIWORD_UPDATE(0, 1, 12) },
};

static const struct rockchip_grf_info rk3328_grf =
{
    .values = rk3328_defaults,
    .values_nr = RT_ARRAY_SIZE(rk3328_defaults),
};

#define RK3368_GRF_SOC_CON15 0x43c

static const struct rockchip_grf_value rk3368_defaults[] =
{
    { "jtag switching", RK3368_GRF_SOC_CON15, HIWORD_UPDATE(0, 1, 13) },
};

static const struct rockchip_grf_info rk3368_grf =
{
    .values = rk3368_defaults,
    .values_nr = RT_ARRAY_SIZE(rk3368_defaults),
};

#define RK3399_GRF_SOC_CON7 0xe21c

static const struct rockchip_grf_value rk3399_defaults[] =
{
    { "jtag switching", RK3399_GRF_SOC_CON7, HIWORD_UPDATE(0, 1, 12) },
};

static const struct rockchip_grf_info rk3399_grf =
{
    .values = rk3399_defaults,
    .values_nr = RT_ARRAY_SIZE(rk3399_defaults),
};

#define RK3566_GRF_USB3OTG0_CON1 0x0104

static const struct rockchip_grf_value rk3566_defaults[] =
{
    { "usb3otg port switch", RK3566_GRF_USB3OTG0_CON1, HIWORD_UPDATE(0, 1, 12) },
    { "usb3otg clock switch", RK3566_GRF_USB3OTG0_CON1, HIWORD_UPDATE(1, 1, 7) },
    { "usb3otg disable usb3", RK3566_GRF_USB3OTG0_CON1, HIWORD_UPDATE(1, 1, 0) },
};

static const struct rockchip_grf_info rk3566_pipegrf =
{
    .values = rk3566_defaults,
    .values_nr = RT_ARRAY_SIZE(rk3566_defaults),
};

#define RK3588_GRF_SOC_CON6 0x0318

static const struct rockchip_grf_value rk3588_defaults[] =
{
    { "jtag switching", RK3588_GRF_SOC_CON6, HIWORD_UPDATE(0, 1, 14) },
};

static const struct rockchip_grf_info rk3588_sysgrf =
{
    .values = rk3588_defaults,
    .values_nr = RT_ARRAY_SIZE(rk3588_defaults),
};

static rt_err_t rockchip_grf_probe(struct rt_platform_device *pdev)
{
    struct rt_syscon *grf;
    struct rt_ofw_node *np = pdev->parent.ofw_node;
    const struct rockchip_grf_info *grf_info = pdev->id->data;

    grf = rt_syscon_find_by_ofw_node(np);

    if (!grf)
    {
        return -RT_EINVAL;
    }

    for (int i = 0; i < grf_info->values_nr; ++i)
    {
        rt_err_t err;
        const struct rockchip_grf_value *val = &grf_info->values[i];

        err = rt_syscon_write(grf, val->reg, val->val);
        LOG_D("%s: adjusting %6x to %10x", val->desc, val->reg, val->val);

        if (err)
        {
            LOG_E("%s: write %6x to %10x fail", val->desc, val->reg, val->val);
        }
    }

    return RT_EOK;
}

static const struct rt_ofw_node_id rockchip_grf_ofw_ids[] =
{
    { .compatible = "rockchip,rk3036-grf", .data = &rk3036_grf, },
    { .compatible = "rockchip,rk3128-grf", .data = &rk3128_grf, },
    { .compatible = "rockchip,rk3228-grf", .data = &rk3228_grf, },
    { .compatible = "rockchip,rk3288-grf", .data = &rk3288_grf, },
    { .compatible = "rockchip,rk3328-grf", .data = &rk3328_grf, },
    { .compatible = "rockchip,rk3368-grf", .data = &rk3368_grf, },
    { .compatible = "rockchip,rk3399-grf", .data = &rk3399_grf, },
    { .compatible = "rockchip,rk3566-pipe-grf", .data = &rk3566_pipegrf, },
    { .compatible = "rockchip,rk3588-sys-grf", .data = &rk3588_sysgrf, },
    { /* sentinel */ }
};

static struct rt_platform_driver rockchip_grf_driver =
{
    .name = "rockchip-grf",
    .ids = rockchip_grf_ofw_ids,

    .probe = rockchip_grf_probe,
};

static int rockchip_grf_drv_register(void)
{
    rt_platform_driver_register(&rockchip_grf_driver);

    return 0;
}
INIT_FRAMEWORK_EXPORT(rockchip_grf_drv_register);
