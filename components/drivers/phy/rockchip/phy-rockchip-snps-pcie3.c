/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-24     GuEe-GUI     first version
 */

#include <cpuport.h>
#include <dt-bindings/phy/phy-snps-pcie3.h>

#include "../phy_dm.h"

#define DBG_TAG "phy.rockchip.snps-pcie3"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define GRF_ENABLE_SHIFT    16
#define GRF_ENABLE_MASK     RT_GENMASK(31, 16)
#define GRF_ENABLE          (GRF_ENABLE_MASK << GRF_ENABLE_SHIFT)

struct rockchip_pcie3_phy_soc_data;

struct rockchip_pcie3_phy
{
    struct rt_phy_device parent;

    void *regs;
    /* mode: RC, EP */
    int mode;
    /* pcie30_phymode: Aggregation, Bifurcation */
    int pcie30_phymode;
    rt_bool_t is_bifurcation;

    int num_lanes;
    rt_uint32_t lanes[4];

    struct rt_syscon *phy_grf;
    struct rt_syscon *pipe_grf;
    struct rt_reset_control *rstc;
    struct rt_clk_array *clk_arr;

    const struct rockchip_pcie3_phy_soc_data *soc_data;
};

#define raw_to_rockchip_pcie3_phy(raw) rt_container_of(raw, struct rockchip_pcie3_phy, parent)

struct rockchip_pcie3_phy_soc_data
{
    rt_err_t (*phy_init)(struct rockchip_pcie3_phy *);
};

#define RK3568_GRF_PCIE30PHY_CON1       0x4
#define RK3568_GRF_PCIE30PHY_CON6       0x18
#define RK3568_GRF_PCIE30PHY_CON9       0x24
#define RK3568_GRF_PCIE30PHY_DA_OCM     (RT_BIT(15) | RT_BIT(GRF_ENABLE_SHIFT + 15))
#define RK3568_GRF_PCIE30PHY_STATUS0    0x80
#define RK3568_GRF_PCIE30PHY_WR_EN      (0xf << 16)
#define RK3568_SRAM_INIT_DONE(reg)      (reg & RT_BIT(14))
#define RK3568_BIFURCATION_LANE_0_1     RT_BIT(0)

static rt_err_t rockchip_p3phy_rk3568_init(struct rockchip_pcie3_phy *rk_p3phy)
{
    rt_err_t err;
    rt_uint32_t reg = 0, timeout_us;
    rt_bool_t bifurcation = RT_FALSE;

    /* Deassert PCIe PMA output clamp mode */
    rt_syscon_write(rk_p3phy->phy_grf, RK3568_GRF_PCIE30PHY_CON9,
            RK3568_GRF_PCIE30PHY_DA_OCM);

    for (int i = 0; i < rk_p3phy->num_lanes; ++i)
    {
        if (rk_p3phy->lanes[i] > 1)
        {
            bifurcation = RT_TRUE;
            break;
        }
    }

    /* Set bifurcation if needed, and it doesn't care RC/EP */
    if (bifurcation)
    {
        LOG_D("%s: bifurcation %s",
                rt_dm_dev_get_name(&rk_p3phy->parent.parent), "enabled");

        rt_syscon_write(rk_p3phy->phy_grf, RK3568_GRF_PCIE30PHY_CON6,
                RK3568_GRF_PCIE30PHY_WR_EN | RK3568_BIFURCATION_LANE_0_1);
        rt_syscon_write(rk_p3phy->phy_grf, RK3568_GRF_PCIE30PHY_CON1,
                RK3568_GRF_PCIE30PHY_DA_OCM);
    }
    else
    {
        LOG_D("%s: bifurcation %s",
                rt_dm_dev_get_name(&rk_p3phy->parent.parent), "disabled");

        rt_syscon_write(rk_p3phy->phy_grf, RK3568_GRF_PCIE30PHY_CON6,
                RK3568_GRF_PCIE30PHY_WR_EN & ~RK3568_BIFURCATION_LANE_0_1);
    }

    rt_reset_control_deassert(rk_p3phy->rstc);

    timeout_us = 500;
    while (timeout_us --> 0)
    {
        err = rt_syscon_read(rk_p3phy->phy_grf, RK3568_GRF_PCIE30PHY_STATUS0, &reg);

        if (err || RK3568_SRAM_INIT_DONE(reg))
        {
            break;
        }

        rt_hw_us_delay(1);
        rt_hw_cpu_relax();
    }

    if (err)
    {
        LOG_E("%s: lock failed 0x%x, check input refclk and power supply",
                rt_dm_dev_get_name(&rk_p3phy->parent.parent), reg);
    }

    return err;
}

static const struct rockchip_pcie3_phy_soc_data rk3568_data =
{
    .phy_init = rockchip_p3phy_rk3568_init,
};

#define PHP_GRF_PCIESEL_CON                 0x100
#define RK3588_PCIE3PHY_GRF_CMN_CON0        0x0
#define RK3588_PCIE3PHY_GRF_DA_OCM          (RT_BIT(8) | RT_BIT(GRF_ENABLE_SHIFT + 8))
#define RK3588_PCIE3PHY_GRF_MODE_SHIFT      0x7
#define RK3588_PCIE3PHY_GRF_PHY0_STATUS1    0x904
#define RK3588_PCIE3PHY_GRF_PHY1_STATUS1    0xa04
#define RK3588_SRAM_INIT_DONE(reg)          (reg & RT_BIT(0))

#define RK3588_BIFURCATION_LANE_0_1         RT_BIT(0)
#define RK3588_BIFURCATION_LANE_2_3         RT_BIT(1)
#define RK3588_LANE_AGGREGATION             RT_BIT(2)

static rt_err_t rockchip_p3phy_rk3588_init(struct rockchip_pcie3_phy *rk_p3phy)
{
    rt_uint8_t mode = 0;
    rt_uint32_t reg = 0, timeout_us;
    rt_err_t err, err_res = RT_EOK;

    /* Deassert PCIe PMA output clamp mode */
    rt_syscon_write(rk_p3phy->phy_grf, RK3588_PCIE3PHY_GRF_CMN_CON0,
            RK3588_PCIE3PHY_GRF_DA_OCM);

    /* Set bifurcation if needed */
    for (int i = 0; i < rk_p3phy->num_lanes; ++i)
    {
        if (!rk_p3phy->lanes[i])
        {
            mode |= (RT_BIT(i) << 3);
        }
        if (rk_p3phy->lanes[i] > 1)
        {
            mode |= (RT_BIT(i) >> 1);
        }
    }

    if (!mode)
    {
        reg = RK3588_LANE_AGGREGATION;
    }
    else
    {
        if (mode & (RT_BIT(0) | RT_BIT(1)))
        {
            reg |= RK3588_BIFURCATION_LANE_0_1;
        }
        if (mode & (RT_BIT(2) | RT_BIT(3)))
        {
            reg |= RK3588_BIFURCATION_LANE_2_3;
        }
    }

    rt_syscon_write(rk_p3phy->phy_grf, RK3588_PCIE3PHY_GRF_CMN_CON0,
            (RK3588_PCIE3PHY_GRF_MODE_SHIFT << GRF_ENABLE_SHIFT) | reg);

    /* Set pcie1ln_sel in PHP_GRF_PCIESEL_CON */
    if (rk_p3phy->pipe_grf)
    {
        reg = (mode & (RT_BIT(6) | RT_BIT(7))) >> 6;

        if (reg)
        {
            rt_syscon_write(rk_p3phy->pipe_grf, PHP_GRF_PCIESEL_CON,
                    (reg << GRF_ENABLE_SHIFT) | reg);
        }
    }

    rt_reset_control_deassert(rk_p3phy->rstc);

    timeout_us = 500;
    while (timeout_us --> 0)
    {
        err = rt_syscon_read(rk_p3phy->phy_grf, RK3588_PCIE3PHY_GRF_PHY0_STATUS1, &reg);

        if (err || RK3588_SRAM_INIT_DONE(reg))
        {
            break;
        }

        rt_hw_us_delay(1);
        rt_hw_cpu_relax();
    }
    err_res |= err;

    timeout_us = 500;
    while (timeout_us --> 0)
    {
        err = rt_syscon_read(rk_p3phy->phy_grf, RK3588_PCIE3PHY_GRF_PHY1_STATUS1, &reg);

        if (err || RK3588_SRAM_INIT_DONE(reg))
        {
            break;
        }

        rt_hw_us_delay(1);
        rt_hw_cpu_relax();
    }
    err_res |= err;

    if (err_res)
    {
        LOG_E("%s: lock failed 0x%x, check input refclk and power supply",
                rt_dm_dev_get_name(&rk_p3phy->parent.parent), reg);
    }

    return err_res;
}

static const struct rockchip_pcie3_phy_soc_data rk3588_data =
{
    .phy_init = rockchip_p3phy_rk3588_init,
};

static rt_phy_status rockchip_pcie3_phy_init(struct rt_phy_device *phy_device,
        void *object, rt_uint32_t phy_addr, rt_uint32_t src_clock_hz)
{
    rt_err_t err;
    struct rockchip_pcie3_phy *rk_p3phy = raw_to_rockchip_pcie3_phy(phy_device);

    if ((err = rt_clk_array_prepare_enable(rk_p3phy->clk_arr)))
    {
        LOG_E("Enable PCIe bulk clks erros = %s", rt_strerror(err));

        return PHY_STATUS_FAIL;
    }

    rt_reset_control_assert(rk_p3phy->rstc);
    rt_hw_us_delay(1);

    if (rk_p3phy->soc_data->phy_init)
    {
        if ((err = rk_p3phy->soc_data->phy_init(rk_p3phy)))
        {
            rt_clk_array_disable_unprepare(rk_p3phy->clk_arr);

            LOG_E("Init PCIe PHY erros = %s", rt_strerror(err));

            return PHY_STATUS_FAIL;
        }
    }

    return PHY_STATUS_OK;
}

static rt_phy_status rockchip_pcie3_phy_exit(struct rt_phy_device *phy_device,
        void *object, rt_uint32_t phy_addr)
{
    struct rockchip_pcie3_phy *rk_p3phy = raw_to_rockchip_pcie3_phy(phy_device);

    rt_clk_array_disable_unprepare(rk_p3phy->clk_arr);
    rt_reset_control_assert(rk_p3phy->rstc);

    return PHY_STATUS_OK;
}

const static struct rt_phy_ops rockchip_pcie3_phy_ops =
{
    .init = rockchip_pcie3_phy_init,
    .exit = rockchip_pcie3_phy_exit,
};

static void rockchip_pcie3_phy_free(struct rockchip_pcie3_phy *rk_p3phy)
{
    if (rk_p3phy->regs)
    {
        rt_iounmap(rk_p3phy->regs);
    }

    if (!rt_is_err_or_null(rk_p3phy->clk_arr))
    {
        rt_clk_array_put(rk_p3phy->clk_arr);
    }

    if (!rt_is_err_or_null(rk_p3phy->rstc))
    {
        rt_reset_control_put(rk_p3phy->rstc);
    }

    rt_free(rk_p3phy);
}

static rt_err_t rockchip_pcie3_phy_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    rt_uint32_t val;
    const char *dev_name;
    struct rt_phy_device *phy;
    struct rt_device *dev = &pdev->parent;
    struct rockchip_pcie3_phy *rk_p3phy = rt_calloc(1, sizeof(*rk_p3phy));

    if (!rk_p3phy)
    {
        return -RT_ENOMEM;
    }

    rk_p3phy->regs = rt_dm_dev_iomap(dev, 0);

    if (!rk_p3phy->regs)
    {
        err = -RT_EIO;
        goto _fail;
    }

    rk_p3phy->phy_grf = rt_syscon_find_by_ofw_phandle(dev->ofw_node, "rockchip,phy-grf");

    if (!rk_p3phy->phy_grf)
    {
        err = -RT_EIO;
        goto _fail;
    }

    rk_p3phy->pipe_grf = rt_syscon_find_by_ofw_phandle(dev->ofw_node, "rockchip,pipe-grf");

    if (!rk_p3phy->pipe_grf && rt_dm_dev_prop_read_bool(dev, "rockchip,pipe-grf"))
    {
        err = -RT_EIO;
        goto _fail;
    }

    rk_p3phy->num_lanes = rt_dm_dev_prop_read_u32_array_index(dev,
            "data-lanes", 0, RT_ARRAY_SIZE(rk_p3phy->lanes), rk_p3phy->lanes);

    if (rk_p3phy->num_lanes == -RT_EEMPTY)
    {
        rk_p3phy->num_lanes = 1;
        rk_p3phy->lanes[0] = 1;
    }
    else if (rk_p3phy->num_lanes < 0)
    {
        err = rk_p3phy->num_lanes;
        LOG_E("Failed to read \"data-lanes\"");
        goto _fail;
    }

    rk_p3phy->rstc = rt_reset_control_get_by_name(dev, "phy");

    if (rt_is_err(rk_p3phy->rstc))
    {
        err = rt_ptr_err(rk_p3phy->rstc);
        goto _fail;
    }

    rk_p3phy->clk_arr = rt_clk_get_array(dev);

    if (rt_is_err(rk_p3phy->clk_arr))
    {
        err = rt_ptr_err(rk_p3phy->clk_arr);
        goto _fail;
    }

    if (!rt_dm_dev_prop_read_u32(dev, "rockchip,pcie30-phymode", &val) && val <= 4)
    {
        rk_p3phy->pcie30_phymode = val;
    }
    else
    {
        rk_p3phy->pcie30_phymode = PHY_MODE_PCIE_AGGREGATION;
    }

    rt_syscon_write(rk_p3phy->phy_grf, RK3588_PCIE3PHY_GRF_CMN_CON0,
            GRF_ENABLE | rk_p3phy->pcie30_phymode);

    if (rk_p3phy->pipe_grf)
    {
        rt_uint32_t reg = rk_p3phy->pcie30_phymode & 3;

        if (reg)
        {
            rt_syscon_write(rk_p3phy->pipe_grf, PHP_GRF_PCIESEL_CON,
                    (reg << GRF_ENABLE_SHIFT) | reg);
        }
    }

    rk_p3phy->soc_data = pdev->id->data;
    dev->user_data = rk_p3phy;

    phy = &rk_p3phy->parent;
    phy->ops = &rockchip_pcie3_phy_ops;

    rt_dm_dev_set_name_auto(&phy->parent, "phy");
    dev_name = rt_dm_dev_get_name(&phy->parent);

    if ((err = rt_hw_phy_register(phy, dev_name)))
    {
        goto _fail;
    }

    rt_dm_dev_bind_fwdata(dev, RT_NULL, phy);

    return RT_EOK;

_fail:
    rockchip_pcie3_phy_free(rk_p3phy);

    return err;
}

static rt_err_t rockchip_pcie3_phy_remove(struct rt_platform_device *pdev)
{
    struct rt_device *dev = &pdev->parent;
    struct rockchip_pcie3_phy *rk_p3phy = dev->user_data;

    rt_dm_dev_unbind_fwdata(dev, RT_NULL);

    rt_device_unregister(&rk_p3phy->parent.parent);

    rockchip_pcie3_phy_free(rk_p3phy);

    return RT_EOK;
}

static const struct rt_ofw_node_id rockchip_pcie3_phy_ofw_ids[] =
{
    { .compatible = "rockchip,rk3568-pcie3-phy", .data = &rk3568_data },
    { .compatible = "rockchip,rk3588-pcie3-phy", .data = &rk3588_data },
    { /* sentinel */ }
};

static struct rt_platform_driver rockchip_pcie3_phy_driver =
{
    .name = "phy-rockchip-snps-pcie3",
    .ids = rockchip_pcie3_phy_ofw_ids,

    .probe = rockchip_pcie3_phy_probe,
    .remove = rockchip_pcie3_phy_remove,
};

static int rockchip_pcie3_phy_drv_register(void)
{
    rt_platform_driver_register(&rockchip_pcie3_phy_driver);

    return 0;
}
INIT_DRIVER_EARLY_EXPORT(rockchip_pcie3_phy_drv_register);
