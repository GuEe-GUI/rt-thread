/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <dt-bindings/size.h>

#include "../sdio_dm.h"

#define SDHCI_DWCMSHC_ARG2_STUFF        RT_GENMASK(31, 16)

/* DWCMSHC specific Mode Select value */
#define DWCMSHC_CTRL_HS400              0x7

/* DWC IP vendor area 1 pointer */
#define DWCMSHC_P_VENDOR_AREA1          0xe8
#define DWCMSHC_AREA1_MASK              RT_GENMASK(11, 0)
/* Offset inside the  vendor area 1 */
#define DWCMSHC_HOST_CTRL3              0x8
#define DWCMSHC_EMMC_CONTROL            0x2c
#define DWCMSHC_CARD_IS_EMMC            RT_BIT(0)
#define DWCMSHC_ENHANCED_STROBE         RT_BIT(8)
#define DWCMSHC_EMMC_ATCTRL             0x40

/* Rockchip specific Registers */
#define DWCMSHC_EMMC_DLL_CTRL           0x800
#define DWCMSHC_EMMC_DLL_RXCLK          0x804
#define DWCMSHC_EMMC_DLL_TXCLK          0x808
#define DWCMSHC_EMMC_DLL_STRBIN         0x80c
#define DECMSHC_EMMC_DLL_CMDOUT         0x810
#define DWCMSHC_EMMC_DLL_STATUS0        0x840
#define DWCMSHC_EMMC_DLL_START          RT_BIT(0)
#define DWCMSHC_EMMC_DLL_LOCKED         RT_BIT(8)
#define DWCMSHC_EMMC_DLL_TIMEOUT        RT_BIT(9)
#define DWCMSHC_EMMC_DLL_RXCLK_SRCSEL   29
#define DWCMSHC_EMMC_DLL_START_POINT    16
#define DWCMSHC_EMMC_DLL_INC            8
#define DWCMSHC_EMMC_DLL_BYPASS         RT_BIT(24)
#define DWCMSHC_EMMC_DLL_DLYENA         RT_BIT(27)
#define DLL_TXCLK_TAPNUM_DEFAULT        0x10
#define DLL_TXCLK_TAPNUM_90_DEGREES     0xA
#define DLL_TXCLK_TAPNUM_FROM_SW        RT_BIT(24)
#define DLL_STRBIN_TAPNUM_DEFAULT       0x8
#define DLL_STRBIN_TAPNUM_FROM_SW       RT_BIT(24)
#define DLL_STRBIN_DELAY_NUM_SEL        RT_BIT(26)
#define DLL_STRBIN_DELAY_NUM_OFFSET     16
#define DLL_STRBIN_DELAY_NUM_DEFAULT    0x16
#define DLL_RXCLK_NO_INVERTER           1
#define DLL_RXCLK_INVERTER              0
#define DLL_CMDOUT_TAPNUM_90_DEGREES    0x8
#define DLL_RXCLK_ORI_GATE              RT_BIT(31)
#define DLL_CMDOUT_TAPNUM_FROM_SW       RT_BIT(24)
#define DLL_CMDOUT_SRC_CLK_NEG          RT_BIT(28)
#define DLL_CMDOUT_EN_SRC_CLK_NEG       RT_BIT(29)

#define DLL_LOCK_WO_TMOUT(x) \
        ((((x) & DWCMSHC_EMMC_DLL_LOCKED) == DWCMSHC_EMMC_DLL_LOCKED) && \
            (((x) & DWCMSHC_EMMC_DLL_TIMEOUT) == 0))

#define BOUNDARY_OK(addr, len) \
        ((addr | ((128 * SIZE_MB) - 1)) == ((addr + len - 1) | ((128 * SIZE_MB) - 1)))

struct sdwcmshc_soc_data
{
    const struct rt_mmcsd_host_ops *ops;
};

struct dwcmshc_sdhci
{
    struct rt_clk *bus_clk;
    /* P_VENDOR_SPECIFIC_AREA reg */
    int vendor_specific_area1;

    void *priv;
};

enum dwcmshc_rk_type
{
    DWCMSHC_RK3568,
    DWCMSHC_RK3588,
};

struct rk35xx_priv
{
    /* Rockchip specified optional clocks */
    struct rt_clk_array *clk_arr;
    struct reset_control *rstc;

    enum dwcmshc_rk_type devtype;
    rt_uint8_t txclk_tapnum;
};

static rt_err_t sdhci_dwcmshc_probe(struct rt_platform_device *pdev)
{
    return RT_EOK;
}

static const struct sdwcmshc_soc_data sdhci_dwcmshc_data =
{
};

static const struct sdwcmshc_soc_data sdhci_dwcmshc_rk35xx_data =
{
};

static const struct rt_ofw_node_id sdhci_dwcmshc_ofw_ids[] =
{
    { .compatible = "rockchip,rk3588-dwcmshc", .data = &sdhci_dwcmshc_rk35xx_data, },
    { .compatible = "rockchip,rk3568-dwcmshc", .data = &sdhci_dwcmshc_rk35xx_data, },
    { .compatible = "rockchip,dwcmshc-sdhci", .data = &sdhci_dwcmshc_rk35xx_data, },
    { .compatible = "snps,dwcmshc-sdhci", .data = &sdhci_dwcmshc_data, },
    { /* sentinel */ }
};

static struct rt_platform_driver sdhci_dwcmshc_driver =
{
    .name = "sdhci-dwcmshc",
    .ids = sdhci_dwcmshc_ofw_ids,

    .probe = sdhci_dwcmshc_probe,
};
RT_PLATFORM_DRIVER_EXPORT(sdhci_dwcmshc_driver);
