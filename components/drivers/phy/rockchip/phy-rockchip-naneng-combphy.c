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
#include <dt-bindings/phy/phy.h>

#include "../phy_dm.h"

#define DBG_TAG "phy.rockchip.naneng-combphy"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define HZ_PER_MHZ                  1000000UL
#define BIT_WRITEABLE_SHIFT         16
#define REF_CLOCK_24MHz             (24 * HZ_PER_MHZ)
#define REF_CLOCK_25MHz             (25 * HZ_PER_MHZ)
#define REF_CLOCK_100MHz            (100 * HZ_PER_MHZ)

/* COMBO PHY REG */
#define PHYREG6                     0x14
#define PHYREG6_PLL_DIV_MASK        RT_GENMASK(7, 6)
#define PHYREG6_PLL_DIV_SHIFT       6
#define PHYREG6_PLL_DIV_2           1

#define PHYREG7                     0x18
#define PHYREG7_TX_RTERM_MASK       RT_GENMASK(7, 4)
#define PHYREG7_TX_RTERM_SHIFT      4
#define PHYREG7_TX_RTERM_50OHM      8
#define PHYREG7_RX_RTERM_MASK       RT_GENMASK(3, 0)
#define PHYREG7_RX_RTERM_SHIFT      0
#define PHYREG7_RX_RTERM_44OHM      15

#define PHYREG8                     0x1c
#define PHYREG8_SSC_EN              RT_BIT(4)

#define PHYREG11                    0x28
#define PHYREG11_SU_TRIM_0_7        0xf0

#define PHYREG12                    0x2c
#define PHYREG12_PLL_LPF_ADJ_VALUE  4

#define PHYREG13                    0x30
#define PHYREG13_RESISTER_MASK      RT_GENMASK(5, 4)
#define PHYREG13_RESISTER_SHIFT     0x4
#define PHYREG13_RESISTER_HIGH_Z    3
#define PHYREG13_CKRCV_AMP0         RT_BIT(7)

#define PHYREG14                    0x34
#define PHYREG14_CKRCV_AMP1         RT_BIT(0)

#define PHYREG15                    0x38
#define PHYREG15_CTLE_EN            RT_BIT(0)
#define PHYREG15_SSC_CNT_MASK       RT_GENMASK(7, 6)
#define PHYREG15_SSC_CNT_SHIFT      6
#define PHYREG15_SSC_CNT_VALUE      1

#define PHYREG16                    0x3c
#define PHYREG16_SSC_CNT_VALUE      0x5f

#define PHYREG18                    0x44
#define PHYREG18_PLL_LOOP           0x32

#define PHYREG27                    0x6c
#define PHYREG27_RX_TRIM_RK3588     0x4c

#define PHYREG32                    0x7c
#define PHYREG32_SSC_MASK           RT_GENMASK(7, 4)
#define PHYREG32_SSC_DIR_SHIFT      4
#define PHYREG32_SSC_UPWARD         0
#define PHYREG32_SSC_DOWNWARD       1
#define PHYREG32_SSC_OFFSET_SHIFT   6
#define PHYREG32_SSC_OFFSET_500PPM  1

#define PHYREG33                    0x80
#define PHYREG33_PLL_KVCO_MASK      RT_GENMASK(4, 2)
#define PHYREG33_PLL_KVCO_SHIFT     2
#define PHYREG33_PLL_KVCO_VALUE     2

struct rockchip_combphy;

struct combphy_reg
{
    rt_uint16_t offset;
    rt_uint16_t bitend;
    rt_uint16_t bitstart;
    rt_uint16_t disable;
    rt_uint16_t enable;
};

struct rockchip_combphy_grfcfg
{
    struct combphy_reg pcie_mode_set;
    struct combphy_reg usb_mode_set;
    struct combphy_reg sgmii_mode_set;
    struct combphy_reg qsgmii_mode_set;
    struct combphy_reg pipe_rxterm_set;
    struct combphy_reg pipe_txelec_set;
    struct combphy_reg pipe_txcomp_set;
    struct combphy_reg pipe_clk_25m;
    struct combphy_reg pipe_clk_100m;
    struct combphy_reg pipe_phymode_sel;
    struct combphy_reg pipe_rate_sel;
    struct combphy_reg pipe_rxterm_sel;
    struct combphy_reg pipe_txelec_sel;
    struct combphy_reg pipe_txcomp_sel;
    struct combphy_reg pipe_clk_ext;
    struct combphy_reg pipe_sel_usb;
    struct combphy_reg pipe_sel_qsgmii;
    struct combphy_reg pipe_phy_status;
    struct combphy_reg con0_for_pcie;
    struct combphy_reg con1_for_pcie;
    struct combphy_reg con2_for_pcie;
    struct combphy_reg con3_for_pcie;
    struct combphy_reg con0_for_sata;
    struct combphy_reg con1_for_sata;
    struct combphy_reg con2_for_sata;
    struct combphy_reg con3_for_sata;
    struct combphy_reg pipe_con0_for_sata;
    struct combphy_reg pipe_con1_for_sata;
    struct combphy_reg pipe_xpcs_phy_ready;
    struct combphy_reg pipe_pcie1l0_sel;
    struct combphy_reg pipe_pcie1l1_sel;
};

struct rockchip_combphy_cfg
{
    const struct rockchip_combphy_grfcfg *grfcfg;
    rt_err_t (*combphy_cfg)(struct rockchip_combphy *rk_cphy);
};

struct rockchip_combphy
{
    struct rt_phy_device parent;

    void *regs;
    rt_uint8_t type;
    rt_bool_t enable_ssc;
    rt_bool_t ext_refclk;

    struct rt_syscon *pipe_grf;
    struct rt_syscon *phy_grf;
    struct rt_reset_control *rstc;
    struct rt_clk *refclk;
    struct rt_clk_array *clk_arr;
    const struct rockchip_combphy_cfg *cfg;
};

#define raw_to_rockchip_combphy(raw) rt_container_of(raw, struct rockchip_combphy, parent)

static void rockchip_combphy_updatel(struct rockchip_combphy *rk_cphy,
        int mask, int val, int offset)
{
    rt_uint32_t data;

    data = HWREG32(rk_cphy->regs + offset);
    data = (data & ~(mask)) | val;
    HWREG32(rk_cphy->regs + offset) = data;
}

static rt_err_t rockchip_combphy_param_write(struct rt_syscon *regmap,
        const struct combphy_reg *reg, bool en)
{
    rt_uint32_t val, mask, tmp;

    tmp = en ? reg->enable : reg->disable;
    mask = RT_GENMASK(reg->bitend, reg->bitstart);
    val = (tmp << reg->bitstart) | (mask << BIT_WRITEABLE_SHIFT);

    return rt_syscon_write(regmap, reg->offset, val);
}

static rt_uint32_t rockchip_combphy_is_ready(struct rockchip_combphy *rk_cphy)
{
    rt_uint32_t mask, val;
    const struct rockchip_combphy_grfcfg *cfg = rk_cphy->cfg->grfcfg;

    mask = RT_GENMASK(cfg->pipe_phy_status.bitend, cfg->pipe_phy_status.bitstart);

    rt_syscon_read(rk_cphy->phy_grf, cfg->pipe_phy_status.offset, &val);
    val = (val & mask) >> cfg->pipe_phy_status.bitstart;

    return val;
}

static rt_err_t rk3568_combphy_cfg(struct rockchip_combphy *rk_cphy)
{
    rt_uint32_t val;
    rt_ubase_t rate;
    const struct rockchip_combphy_grfcfg *cfg = rk_cphy->cfg->grfcfg;

    switch (rk_cphy->type)
    {
    case PHY_TYPE_PCIE:
        /* Set SSC downward spread spectrum. */
        rockchip_combphy_updatel(rk_cphy, PHYREG32_SSC_MASK,
                PHYREG32_SSC_DOWNWARD << PHYREG32_SSC_DIR_SHIFT, PHYREG32);

        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con0_for_pcie, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con1_for_pcie, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con2_for_pcie, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con3_for_pcie, RT_TRUE);
        break;

    case PHY_TYPE_USB3:
        /* Set SSC downward spread spectrum. */
        rockchip_combphy_updatel(rk_cphy, PHYREG32_SSC_MASK,
                PHYREG32_SSC_DOWNWARD << PHYREG32_SSC_DIR_SHIFT, PHYREG32);

        /* Enable adaptive CTLE for USB3.0 Rx. */
        val = HWREG32(rk_cphy->regs + PHYREG15);
        val |= PHYREG15_CTLE_EN;
        HWREG32(rk_cphy->regs + PHYREG15) = val;

        /* Set PLL KVCO fine tuning signals. */
        rockchip_combphy_updatel(rk_cphy, PHYREG33_PLL_KVCO_MASK,
                PHYREG33_PLL_KVCO_VALUE << PHYREG33_PLL_KVCO_SHIFT, PHYREG33);

        /* Enable controlling random jitter. */
        HWREG32(rk_cphy->regs + PHYREG12) = PHYREG12_PLL_LPF_ADJ_VALUE;

        /* Set PLL input clock divider 1/2. */
        rockchip_combphy_updatel(rk_cphy, PHYREG6_PLL_DIV_MASK,
                PHYREG6_PLL_DIV_2 << PHYREG6_PLL_DIV_SHIFT, PHYREG6);

        HWREG32(rk_cphy->regs + PHYREG18) = PHYREG18_PLL_LOOP;
        HWREG32(rk_cphy->regs + PHYREG11) = PHYREG11_SU_TRIM_0_7;

        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_sel_usb, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_txcomp_sel, RT_FALSE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_txelec_sel, RT_FALSE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->usb_mode_set, RT_TRUE);
        break;

    case PHY_TYPE_SATA:
        /* Enable adaptive CTLE for SATA Rx. */
        val = HWREG32(rk_cphy->regs + PHYREG15);
        val |= PHYREG15_CTLE_EN;
        HWREG32(rk_cphy->regs + PHYREG15) = val;
        /*
         * Set tx_rterm=50ohm and rx_rterm=44ohm for SATA.
         * 0: 60ohm, 8: 50ohm 15: 44ohm (by step abort 1ohm)
         */
        val = PHYREG7_TX_RTERM_50OHM << PHYREG7_TX_RTERM_SHIFT;
        val |= PHYREG7_RX_RTERM_44OHM << PHYREG7_RX_RTERM_SHIFT;
        HWREG32(rk_cphy->regs + PHYREG7) = val;

        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con0_for_sata, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con1_for_sata, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con2_for_sata, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con3_for_sata, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->pipe_grf, &cfg->pipe_con0_for_sata, RT_TRUE);
        break;

    case PHY_TYPE_SGMII:
        rockchip_combphy_param_write(rk_cphy->pipe_grf, &cfg->pipe_xpcs_phy_ready, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_phymode_sel, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_sel_qsgmii, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->sgmii_mode_set, RT_TRUE);
        break;

    case PHY_TYPE_QSGMII:
        rockchip_combphy_param_write(rk_cphy->pipe_grf, &cfg->pipe_xpcs_phy_ready, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_phymode_sel, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_rate_sel, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_sel_qsgmii, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->qsgmii_mode_set, RT_TRUE);
        break;

    default:
        LOG_E("Incompatible PHY type");
        return -RT_EINVAL;
    }

    rate = rt_clk_get_rate(rk_cphy->refclk);

    switch (rate)
    {
    case REF_CLOCK_24MHz:
        if (rk_cphy->type == PHY_TYPE_USB3 || rk_cphy->type == PHY_TYPE_SATA)
        {
            /* Set ssc_cnt[9:0]=0101111101 & 31.5KHz. */
            val = PHYREG15_SSC_CNT_VALUE << PHYREG15_SSC_CNT_SHIFT;
            rockchip_combphy_updatel(rk_cphy, PHYREG15_SSC_CNT_MASK, val, PHYREG15);

            HWREG32(rk_cphy->regs + PHYREG16) = PHYREG16_SSC_CNT_VALUE;
        }
        break;

    case REF_CLOCK_25MHz:
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_clk_25m, RT_TRUE);
        break;

    case REF_CLOCK_100MHz:
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_clk_100m, RT_TRUE);
        if (rk_cphy->type == PHY_TYPE_PCIE)
        {
            /* PLL KVCO  fine tuning. */
            val = PHYREG33_PLL_KVCO_VALUE << PHYREG33_PLL_KVCO_SHIFT;
            rockchip_combphy_updatel(rk_cphy, PHYREG33_PLL_KVCO_MASK, val, PHYREG33);

            /* Enable controlling random jitter. */
            HWREG32(rk_cphy->regs + PHYREG12) = PHYREG12_PLL_LPF_ADJ_VALUE;

            val = PHYREG6_PLL_DIV_2 << PHYREG6_PLL_DIV_SHIFT;
            rockchip_combphy_updatel(rk_cphy, PHYREG6_PLL_DIV_MASK,
                         val, PHYREG6);

            HWREG32(rk_cphy->regs + PHYREG18) = PHYREG18_PLL_LOOP;
            HWREG32(rk_cphy->regs + PHYREG11) = PHYREG11_SU_TRIM_0_7;
        }
        else if (rk_cphy->type == PHY_TYPE_SATA)
        {
            /* downward spread spectrum +500ppm */
            val = PHYREG32_SSC_DOWNWARD << PHYREG32_SSC_DIR_SHIFT;
            val |= PHYREG32_SSC_OFFSET_500PPM << PHYREG32_SSC_OFFSET_SHIFT;
            rockchip_combphy_updatel(rk_cphy, PHYREG32_SSC_MASK, val, PHYREG32);
        }
        break;

    default:
        LOG_E("Unsupported rate: %u", rate);
        return -RT_EINVAL;
    }

    if (rk_cphy->ext_refclk)
    {
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_clk_ext, RT_TRUE);

        if (rk_cphy->type == PHY_TYPE_PCIE && rate == REF_CLOCK_100MHz)
        {
            val = PHYREG13_RESISTER_HIGH_Z << PHYREG13_RESISTER_SHIFT;
            val |= PHYREG13_CKRCV_AMP0;
            rockchip_combphy_updatel(rk_cphy, PHYREG13_RESISTER_MASK, val, PHYREG13);

            val = HWREG32(rk_cphy->regs + PHYREG14);
            val |= PHYREG14_CKRCV_AMP1;
            HWREG32(rk_cphy->regs + PHYREG14) = val;
        }
    }

    if (rk_cphy->enable_ssc)
    {
        val = HWREG32(rk_cphy->regs + PHYREG8);
        val |= PHYREG8_SSC_EN;
        HWREG32(rk_cphy->regs + PHYREG8) = val;
    }

    return 0;
}

static const struct rockchip_combphy_grfcfg rk3568_combphy_grfcfgs =
{
    /* pipe-phy-grf */
    .pcie_mode_set       = { 0x0000,  5,  0, 0x00, 0x0011 },
    .usb_mode_set        = { 0x0000,  5,  0, 0x00, 0x0004 },
    .sgmii_mode_set      = { 0x0000,  5,  0, 0x00, 0x0001 },
    .qsgmii_mode_set     = { 0x0000,  5,  0, 0x00, 0x0021 },
    .pipe_rxterm_set     = { 0x0000, 12, 12, 0x00, 0x0001 },
    .pipe_txelec_set     = { 0x0004,  1,  1, 0x00, 0x0001 },
    .pipe_txcomp_set     = { 0x0004,  4,  4, 0x00, 0x0001 },
    .pipe_clk_25m        = { 0x0004, 14, 13, 0x00, 0x0001 },
    .pipe_clk_100m       = { 0x0004, 14, 13, 0x00, 0x0002 },
    .pipe_phymode_sel    = { 0x0008,  1,  1, 0x00, 0x0001 },
    .pipe_rate_sel       = { 0x0008,  2,  2, 0x00, 0x0001 },
    .pipe_rxterm_sel     = { 0x0008,  8,  8, 0x00, 0x0001 },
    .pipe_txelec_sel     = { 0x0008, 12, 12, 0x00, 0x0001 },
    .pipe_txcomp_sel     = { 0x0008, 15, 15, 0x00, 0x0001 },
    .pipe_clk_ext        = { 0x000c,  9,  8, 0x02, 0x0001 },
    .pipe_sel_usb        = { 0x000c, 14, 13, 0x00, 0x0001 },
    .pipe_sel_qsgmii     = { 0x000c, 15, 13, 0x00, 0x0007 },
    .pipe_phy_status     = { 0x0034,  6,  6, 0x01, 0x0000 },
    .con0_for_pcie       = { 0x0000, 15,  0, 0x00, 0x1000 },
    .con1_for_pcie       = { 0x0004, 15,  0, 0x00, 0x0000 },
    .con2_for_pcie       = { 0x0008, 15,  0, 0x00, 0x0101 },
    .con3_for_pcie       = { 0x000c, 15,  0, 0x00, 0x0200 },
    .con0_for_sata       = { 0x0000, 15,  0, 0x00, 0x0119 },
    .con1_for_sata       = { 0x0004, 15,  0, 0x00, 0x0040 },
    .con2_for_sata       = { 0x0008, 15,  0, 0x00, 0x80c3 },
    .con3_for_sata       = { 0x000c, 15,  0, 0x00, 0x4407 },
    /* pipe-grf */
    .pipe_con0_for_sata  = { 0x0000, 15,  0, 0x00, 0x2220 },
    .pipe_xpcs_phy_ready = { 0x0040,  2,  2, 0x00, 0x0001 },
};

static const struct rockchip_combphy_cfg rk3568_combphy_cfgs =
{
    .grfcfg = &rk3568_combphy_grfcfgs,
    .combphy_cfg = rk3568_combphy_cfg,
};

static rt_err_t rk3588_combphy_cfg(struct rockchip_combphy *rk_cphy)
{
    rt_uint32_t val;
    rt_ubase_t rate;
    const struct rockchip_combphy_grfcfg *cfg = rk_cphy->cfg->grfcfg;

    switch (rk_cphy->type)
    {
    case PHY_TYPE_PCIE:
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con0_for_pcie, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con1_for_pcie, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con2_for_pcie, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con3_for_pcie, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->pipe_grf, &cfg->pipe_pcie1l0_sel, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->pipe_grf, &cfg->pipe_pcie1l1_sel, RT_TRUE);
        break;

    case PHY_TYPE_USB3:
        /* Set SSC downward spread spectrum */
        rockchip_combphy_updatel(rk_cphy, PHYREG32_SSC_MASK,
                PHYREG32_SSC_DOWNWARD << PHYREG32_SSC_DIR_SHIFT, PHYREG32);

        /* Enable adaptive CTLE for USB3.0 Rx. */
        val = HWREG32(rk_cphy->regs + PHYREG15);
        val |= PHYREG15_CTLE_EN;
        HWREG32(rk_cphy->regs + PHYREG15) = val;

        /* Set PLL KVCO fine tuning signals. */
        rockchip_combphy_updatel(rk_cphy, PHYREG33_PLL_KVCO_MASK,
                PHYREG33_PLL_KVCO_VALUE << PHYREG33_PLL_KVCO_SHIFT, PHYREG33);

        /* Enable controlling random jitter. */
        HWREG32(rk_cphy->regs + PHYREG12) = PHYREG12_PLL_LPF_ADJ_VALUE;

        /* Set PLL input clock divider 1/2. */
        rockchip_combphy_updatel(rk_cphy, PHYREG6_PLL_DIV_MASK,
                PHYREG6_PLL_DIV_2 << PHYREG6_PLL_DIV_SHIFT, PHYREG6);

        HWREG32(rk_cphy->regs + PHYREG18) = PHYREG18_PLL_LOOP;
        HWREG32(rk_cphy->regs + PHYREG11) = PHYREG11_SU_TRIM_0_7;

        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_txcomp_sel, RT_FALSE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_txelec_sel, RT_FALSE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->usb_mode_set, RT_TRUE);
        break;

    case PHY_TYPE_SATA:
        /* Enable adaptive CTLE for SATA Rx. */
        val = HWREG32(rk_cphy->regs + PHYREG15);
        val |= PHYREG15_CTLE_EN;
        HWREG32(rk_cphy->regs + PHYREG15) = val;
        /*
         * Set tx_rterm=50ohm and rx_rterm=44ohm for SATA.
         * 0: 60ohm, 8: 50ohm 15: 44ohm (by step abort 1ohm)
         */
        val = PHYREG7_TX_RTERM_50OHM << PHYREG7_TX_RTERM_SHIFT;
        val |= PHYREG7_RX_RTERM_44OHM << PHYREG7_RX_RTERM_SHIFT;
        HWREG32(rk_cphy->regs + PHYREG7) = val;

        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con0_for_sata, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con1_for_sata, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con2_for_sata, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->con3_for_sata, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->pipe_grf, &cfg->pipe_con0_for_sata, RT_TRUE);
        rockchip_combphy_param_write(rk_cphy->pipe_grf, &cfg->pipe_con1_for_sata, RT_TRUE);
        break;

    case PHY_TYPE_SGMII:
    case PHY_TYPE_QSGMII:
    default:
        LOG_E("Incompatible PHY type");
        return -RT_EINVAL;
    }

    rate = rt_clk_get_rate(rk_cphy->refclk);

    switch (rate)
    {
    case REF_CLOCK_24MHz:
        if (rk_cphy->type == PHY_TYPE_USB3 || rk_cphy->type == PHY_TYPE_SATA)
        {
            /* Set ssc_cnt[9:0]=0101111101 & 31.5KHz. */
            val = PHYREG15_SSC_CNT_VALUE << PHYREG15_SSC_CNT_SHIFT;
            rockchip_combphy_updatel(rk_cphy, PHYREG15_SSC_CNT_MASK,
                         val, PHYREG15);

            HWREG32(rk_cphy->regs + PHYREG16) = PHYREG16_SSC_CNT_VALUE;
        }
        break;

    case REF_CLOCK_25MHz:
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_clk_25m, RT_TRUE);
        break;

    case REF_CLOCK_100MHz:
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_clk_100m, RT_TRUE);
        if (rk_cphy->type == PHY_TYPE_PCIE)
        {
            /* PLL KVCO fine tuning. */
            val = 4 << PHYREG33_PLL_KVCO_SHIFT;
            rockchip_combphy_updatel(rk_cphy, PHYREG33_PLL_KVCO_MASK, val, PHYREG33);

            /* Enable controlling random jitter. */
            HWREG32(rk_cphy->regs + PHYREG12) = PHYREG12_PLL_LPF_ADJ_VALUE;

            /* Set up rx_trim: PLL LPF C1 85pf R1 1.25kohm */
            HWREG32(rk_cphy->regs + PHYREG27) = PHYREG27_RX_TRIM_RK3588;

            /* Set up su_trim:  */
            HWREG32(rk_cphy->regs + PHYREG11) = PHYREG11_SU_TRIM_0_7;
        }
        else if (rk_cphy->type == PHY_TYPE_SATA)
        {
            /* downward spread spectrum +500ppm */
            val = PHYREG32_SSC_DOWNWARD << PHYREG32_SSC_DIR_SHIFT;
            val |= PHYREG32_SSC_OFFSET_500PPM << PHYREG32_SSC_OFFSET_SHIFT;
            rockchip_combphy_updatel(rk_cphy, PHYREG32_SSC_MASK, val, PHYREG32);
        }
        break;
    default:
        LOG_E("Unsupported rate: %u", rate);
        return -RT_EINVAL;
    }

    if (rk_cphy->ext_refclk)
    {
        rockchip_combphy_param_write(rk_cphy->phy_grf, &cfg->pipe_clk_ext, RT_TRUE);

        if (rk_cphy->type == PHY_TYPE_PCIE && rate == REF_CLOCK_100MHz)
        {
            val = PHYREG13_RESISTER_HIGH_Z << PHYREG13_RESISTER_SHIFT;
            val |= PHYREG13_CKRCV_AMP0;
            rockchip_combphy_updatel(rk_cphy, PHYREG13_RESISTER_MASK, val, PHYREG13);

            val = HWREG32(rk_cphy->regs + PHYREG14);
            val |= PHYREG14_CKRCV_AMP1;
            HWREG32(rk_cphy->regs + PHYREG14) = val;
        }
    }

    if (rk_cphy->enable_ssc)
    {
        val = HWREG32(rk_cphy->regs + PHYREG8);
        val |= PHYREG8_SSC_EN;
        HWREG32(rk_cphy->regs + PHYREG8) = val;
    }

    return RT_EOK;
}

static const struct rockchip_combphy_grfcfg rk3588_combphy_grfcfgs =
{
    /* pipe-phy-grf */
    .pcie_mode_set      = { 0x0000,  5,  0, 0x00, 0x0011 },
    .usb_mode_set       = { 0x0000,  5,  0, 0x00, 0x0004 },
    .pipe_rxterm_set    = { 0x0000, 12, 12, 0x00, 0x0001 },
    .pipe_txelec_set    = { 0x0004,  1,  1, 0x00, 0x0001 },
    .pipe_txcomp_set    = { 0x0004,  4,  4, 0x00, 0x0001 },
    .pipe_clk_25m       = { 0x0004, 14, 13, 0x00, 0x0001 },
    .pipe_clk_100m      = { 0x0004, 14, 13, 0x00, 0x0002 },
    .pipe_rxterm_sel    = { 0x0008,  8,  8, 0x00, 0x0001 },
    .pipe_txelec_sel    = { 0x0008, 12, 12, 0x00, 0x0001 },
    .pipe_txcomp_sel    = { 0x0008, 15, 15, 0x00, 0x0001 },
    .pipe_clk_ext       = { 0x000c,  9,  8, 0x02, 0x0001 },
    .pipe_phy_status    = { 0x0034,  6,  6, 0x01, 0x0000 },
    .con0_for_pcie      = { 0x0000, 15,  0, 0x00, 0x1000 },
    .con1_for_pcie      = { 0x0004, 15,  0, 0x00, 0x0000 },
    .con2_for_pcie      = { 0x0008, 15,  0, 0x00, 0x0101 },
    .con3_for_pcie      = { 0x000c, 15,  0, 0x00, 0x0200 },
    .con0_for_sata      = { 0x0000, 15,  0, 0x00, 0x0129 },
    .con1_for_sata      = { 0x0004, 15,  0, 0x00, 0x0000 },
    .con2_for_sata      = { 0x0008, 15,  0, 0x00, 0x80c1 },
    .con3_for_sata      = { 0x000c, 15,  0, 0x00, 0x0407 },
    /* pipe-grf */
    .pipe_con0_for_sata = { 0x0000, 11,  5, 0x00, 0x0022 },
    .pipe_con1_for_sata = { 0x0000,  2,  0, 0x00, 0x0002 },
    .pipe_pcie1l0_sel   = { 0x0100,  0,  0, 0x01, 0x0000 },
    .pipe_pcie1l1_sel   = { 0x0100,  1,  1, 0x01, 0x0000 },
};

static const struct rockchip_combphy_cfg rk3588_combphy_cfgs =
{
    .grfcfg = &rk3588_combphy_grfcfgs,
    .combphy_cfg = rk3588_combphy_cfg,
};

static rt_phy_status rockchip_combphy_init(struct rt_phy_device *phy_device,
        void *object, rt_uint32_t phy_addr, rt_uint32_t src_clock_hz)
{
    rt_err_t err;
    struct rockchip_combphy *rk_cphy = raw_to_rockchip_combphy(phy_device);
    const struct rockchip_combphy_grfcfg *cfg = rk_cphy->cfg->grfcfg;

    if ((err = rt_clk_array_prepare_enable(rk_cphy->clk_arr)))
    {
        LOG_E("Failed to enable clks error = %s", rt_strerror(err));

        return PHY_STATUS_FAIL;
    }

    switch (rk_cphy->type)
    {
    case PHY_TYPE_PCIE:
    case PHY_TYPE_USB3:
    case PHY_TYPE_SATA:
    case PHY_TYPE_SGMII:
    case PHY_TYPE_QSGMII:
        if (rk_cphy->cfg->combphy_cfg)
        {
            err = rk_cphy->cfg->combphy_cfg(rk_cphy);
        }
        break;

    default:
        LOG_E("Incompatible PHY type");
        err = -RT_EINVAL;
        break;
    }

    if (err)
    {
        LOG_E("Failed to init PHY for type %d", rk_cphy->type);

        goto _out_clk;
    }

    if ((err = rt_reset_control_deassert(rk_cphy->rstc)))
    {
        goto _out_clk;
    }

    if (rk_cphy->type == PHY_TYPE_USB3)
    {
        rt_uint32_t val;
        rt_int32_t timeout_us = 1000;

        while (timeout_us --> 0)
        {
            val = rockchip_combphy_is_ready(rk_cphy);

            if (val == cfg->pipe_phy_status.enable)
            {
                break;
            }

            rt_hw_us_delay(10);
            rt_hw_cpu_relax();
        }

        if (timeout_us <= 0)
        {
            LOG_W("Wait PHY status ready timeout");
        }
    }

    return PHY_STATUS_OK;

_out_clk:
    rt_clk_array_disable_unprepare(rk_cphy->clk_arr);

    return PHY_STATUS_FAIL;
}

static rt_phy_status rockchip_combphy_exit(struct rt_phy_device *phy_device,
        void *object, rt_uint32_t phy_addr)
{
    struct rockchip_combphy *rk_cphy = raw_to_rockchip_combphy(phy_device);

    rt_clk_array_disable_unprepare(rk_cphy->clk_arr);
    rt_reset_control_assert(rk_cphy->rstc);

    return PHY_STATUS_OK;
}

static rt_err_t rockchip_combphy_ofw_parse(struct rt_phy_device *phy_device,
        struct rt_ofw_cell_args *phy_args)
{
    struct rockchip_combphy *rk_cphy = raw_to_rockchip_combphy(phy_device);

    if (phy_args->args_count != 1)
    {
        LOG_E("Invalid number of arguments");

        return -RT_EINVAL;
    }

    if (rk_cphy->type != PHY_NONE && rk_cphy->type != phy_args->args[0])
    {
        LOG_W("PHY select type %d from type %d",
                phy_args->args[0], rk_cphy->type);
    }

    rk_cphy->type = phy_args->args[0];

    return RT_EOK;
}

const static struct rt_phy_ops rochchip_combphy_ops =
{
    .init = rockchip_combphy_init,
    .exit = rockchip_combphy_exit,
    .ofw_parse = rockchip_combphy_ofw_parse,
};

static void rockchip_combphy_free(struct rockchip_combphy *rk_cphy)
{
    if (rk_cphy->regs)
    {
        rt_iounmap(rk_cphy->regs);
    }

    if (!rt_is_err_or_null(rk_cphy->clk_arr))
    {
        rt_clk_array_put(rk_cphy->clk_arr);
    }

    if (!rt_is_err_or_null(rk_cphy->rstc))
    {
        rt_reset_control_put(rk_cphy->rstc);
    }

    rt_free(rk_cphy);
}

static rt_err_t rockchip_combphy_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    const char *dev_name;
    struct rt_phy_device *phy;
    struct rt_device *dev = &pdev->parent;
    struct rockchip_combphy *rk_cphy = rt_calloc(1, sizeof(*rk_cphy));
    const struct rockchip_combphy_cfg *phy_cfg;

    if (!rk_cphy)
    {
        return -RT_ENOMEM;
    }

    phy_cfg = pdev->id->data;
    rk_cphy->type = PHY_NONE;
    rk_cphy->cfg = phy_cfg;

    rk_cphy->regs = rt_dm_dev_iomap(dev, 0);
    if (!rk_cphy->regs)
    {
        err = -RT_EIO;
        goto _fail;
    }

    rk_cphy->clk_arr = rt_clk_get_array(dev);
    if (rt_is_err(rk_cphy->clk_arr))
    {
        err = rt_ptr_err(rk_cphy->clk_arr);
        goto _fail;
    }

    for (int i = rk_cphy->clk_arr->count - 1; i >= 0; --i)
    {
        if (!rt_strncmp(rk_cphy->clk_arr->clks[i]->con_id, "ref", 3))
        {
            rk_cphy->refclk = rk_cphy->clk_arr->clks[i];
        }
    }

    if (!rk_cphy->refclk)
    {
        err = -RT_EIO;
        goto _fail;
    }

    rk_cphy->pipe_grf = rt_syscon_find_by_ofw_phandle(dev->ofw_node, "rockchip,pipe-grf");
    if (!rk_cphy->pipe_grf)
    {
        err = -RT_EIO;
        goto _fail;
    }

    rk_cphy->phy_grf = rt_syscon_find_by_ofw_phandle(dev->ofw_node, "rockchip,pipe-phy-grf");
    if (!rk_cphy->phy_grf)
    {
        err = -RT_EIO;
        goto _fail;
    }

    rk_cphy->enable_ssc = rt_dm_dev_prop_read_bool(dev, "rockchip,enable-ssc");
    rk_cphy->ext_refclk = rt_dm_dev_prop_read_bool(dev, "rockchip,ext-refclk");

    rk_cphy->rstc = rt_reset_control_get_array(dev);
    if (rt_is_err(rk_cphy->rstc))
    {
        err = rt_ptr_err(rk_cphy->rstc);
        goto _fail;
    }

    err = rt_reset_control_assert(rk_cphy->rstc);
    if (err)
    {
        goto _fail;
    }

    dev->user_data = rk_cphy;

    phy = &rk_cphy->parent;
    phy->ops = &rochchip_combphy_ops;

    rt_dm_dev_set_name_auto(&phy->parent, "phy");
    dev_name = rt_dm_dev_get_name(&phy->parent);

    if ((err = rt_hw_phy_register(phy, dev_name)))
    {
        goto _fail;
    }

    rt_dm_dev_bind_fwdata(dev, RT_NULL, phy);

    return RT_EOK;

_fail:
    rockchip_combphy_free(rk_cphy);

    return err;
}

static rt_err_t rockchip_combphy_remove(struct rt_platform_device *pdev)
{
    struct rt_device *dev = &pdev->parent;
    struct rockchip_combphy *rk_cphy = dev->user_data;

    rt_dm_dev_unbind_fwdata(dev, RT_NULL);

    rt_device_unregister(&rk_cphy->parent.parent);

    rockchip_combphy_free(rk_cphy);

    return RT_EOK;
}

static const struct rt_ofw_node_id rockchip_combphy_ofw_ids[] =
{
    { .compatible = "rockchip,rk3568-naneng-combphy", .data = &rk3568_combphy_cfgs },
    { .compatible = "rockchip,rk3588-naneng-combphy", .data = &rk3588_combphy_cfgs },
    { /* sentinel */ }
};

static struct rt_platform_driver rockchip_combphy_driver =
{
    .name = "phy-rockchip-naneng-combphy",
    .ids = rockchip_combphy_ofw_ids,

    .probe = rockchip_combphy_probe,
    .remove = rockchip_combphy_remove,
};

static int rockchip_combphy_drv_register(void)
{
    rt_platform_driver_register(&rockchip_combphy_driver);

    return 0;
}
INIT_DRIVER_EARLY_EXPORT(rockchip_combphy_drv_register);
