/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-09-23     GuEe-GUI     first version
 */

#define DBG_TAG "pci.dw.rockchip"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "pcie-dw.h"

/*
 * The upper 16 bits of PCIE_CLIENT_CONFIG are a write
 * mask for the lower 16 bits.
 */
#define HIWORD_UPDATE(mask, val)        (((mask) << 16) | (val))
#define HIWORD_UPDATE_BIT(val)          HIWORD_UPDATE(val, val)
#define HIWORD_DISABLE_BIT(val)         HIWORD_UPDATE(val, ~val)

#define PCIE_CLIENT_RC_MODE             HIWORD_UPDATE_BIT(0x40)
#define PCIE_CLIENT_ENABLE_LTSSM        HIWORD_UPDATE_BIT(0xc)
#define PCIE_SMLH_LINKUP                RT_BIT(16)
#define PCIE_RDLH_LINKUP                RT_BIT(17)
#define PCIE_LINKUP                     (PCIE_SMLH_LINKUP | PCIE_RDLH_LINKUP)
#define PCIE_L0S_ENTRY                  0x11
#define PCIE_CLIENT_GENERAL_CONTROL     0x0
#define PCIE_CLIENT_INTR_STATUS_LEGACY  0x8
#define PCIE_CLIENT_INTR_MASK_LEGACY    0x1c
#define PCIE_CLIENT_GENERAL_DEBUG       0x104
#define PCIE_CLIENT_HOT_RESET_CTRL      0x180
#define PCIE_CLIENT_LTSSM_STATUS        0x300
#define PCIE_LTSSM_ENABLE_ENHANCE       RT_BIT(4)
#define PCIE_LTSSM_STATUS_MASK          RT_GENMASK(5, 0)

struct rockchip_pcie
{
    struct dw_pcie pci;
    void *apb_base;

    rt_base_t rst_pin;
    struct rt_phy_device *phy;
    struct rt_clk_array *clk_arr;
    struct rt_reset_control *rstc;
    struct rt_regulator *vpcie3v3;

    int intx_irq;
    struct rt_pic intx_pic;
};

#define to_rockchip_pcie(dw_pcie) rt_container_of(dw_pcie, struct rockchip_pcie, pci)

rt_inline rt_uint32_t rockchip_pcie_readl_apb(struct rockchip_pcie *rk_pcie,
        int offset)
{
    return HWREG32(rk_pcie->apb_base + offset);
}

rt_inline void rockchip_pcie_writel_apb(struct rockchip_pcie *rk_pcie,
        rt_uint32_t val, int offset)
{
    HWREG32(rk_pcie->apb_base + offset) = val;
}

static void rockchip_pcie_enable_ltssm(struct rockchip_pcie *rk_pcie)
{
    rockchip_pcie_writel_apb(rk_pcie, PCIE_CLIENT_ENABLE_LTSSM,
            PCIE_CLIENT_GENERAL_CONTROL);
}

static rt_bool_t rockchip_pcie_link_up(struct dw_pcie *pci)
{
    rt_uint32_t val;
    struct rockchip_pcie *rk_pcie = to_rockchip_pcie(pci);

    val = rockchip_pcie_readl_apb(rk_pcie, PCIE_CLIENT_LTSSM_STATUS);

    if ((val & PCIE_LINKUP) == PCIE_LINKUP &&
        (val & PCIE_LTSSM_STATUS_MASK) == PCIE_L0S_ENTRY)
    {
        return RT_TRUE;
    }

    return RT_FALSE;
}

static rt_err_t rockchip_pcie_start_link(struct dw_pcie *pci)
{
    struct rockchip_pcie *rk_pcie = to_rockchip_pcie(pci);

    /* Reset device */
    if (rk_pcie->rst_pin >= 0)
    {
        rt_pin_write(rk_pcie->rst_pin, PIN_LOW);
    }

    rockchip_pcie_enable_ltssm(rk_pcie);

    /*
     * PCIe requires the refclk to be stable for 100µs prior to releasing
     * PERST. See table 2-4 in section 2.6.2 AC Specifications of the PCI
     * Express Card Electromechanical Specification, 1.1. However, we don't
     * know if the refclk is coming from RC's PHY or external OSC. If it's
     * from RC, so enabling LTSSM is the just right place to release #PERST.
     * We need more extra time as before, rather than setting just
     * 100us as we don't know how long should the device need to reset.
     */
    rt_thread_mdelay(100);

    if (rk_pcie->rst_pin >= 0)
    {
        rt_pin_write(rk_pcie->rst_pin, PIN_HIGH);
    }

    return RT_EOK;
}

static const struct dw_pcie_ops dw_pcie_ops =
{
    .link_up = rockchip_pcie_link_up,
    .start_link = rockchip_pcie_start_link,
};

static void rockchip_pcie_intx_mask(struct rt_pic_irq *pirq)
{
    struct rockchip_pcie *rk_pcie = pirq->pic->priv_data;

    rockchip_pcie_writel_apb(rk_pcie, HIWORD_UPDATE_BIT(RT_BIT(pirq->hwirq)),
            PCIE_CLIENT_INTR_MASK_LEGACY);
}

static void rockchip_pcie_intx_unmask(struct rt_pic_irq *pirq)
{
    struct rockchip_pcie *rk_pcie = pirq->pic->priv_data;

    rockchip_pcie_writel_apb(rk_pcie, HIWORD_DISABLE_BIT(RT_BIT(pirq->hwirq)),
            PCIE_CLIENT_INTR_MASK_LEGACY);
}

static int rockchip_intx_irq_map(struct rt_pic *pic, int hwirq, rt_uint32_t mode)
{
    int irq;
    struct rt_pic_irq *pirq = rt_pic_find_irq(pic, hwirq);

    if (pirq)
    {
        if (pirq->irq >= 0)
        {
            irq = pirq->irq;
        }
        else
        {
            struct rockchip_pcie *rk_pcie = pic->priv_data;

            irq = rt_pic_config_irq(pic, hwirq, hwirq);
            rt_pic_cascade(pirq, rk_pcie->intx_irq);
            rt_pic_irq_set_triger_mode(irq, mode);
        }
    }
    else
    {
        irq = -1;
    }

    return irq;
}

static rt_err_t rockchip_intx_irq_parse(struct rt_pic *pic,
        struct rt_ofw_cell_args *args, struct rt_pic_irq *out_pirq)
{
    rt_err_t err = RT_EOK;

    if (args->args_count == 1)
    {
        out_pirq->hwirq = args->args[1];
    }
    else
    {
        err = -RT_EINVAL;
    }

    return err;
}

const static struct rt_pic_ops rockchip_intx_ops =
{
    .name = "RK-INTx",
    .irq_mask = rockchip_pcie_intx_mask,
    .irq_unmask = rockchip_pcie_intx_unmask,
    .irq_map = rockchip_intx_irq_map,
    .irq_parse = rockchip_intx_irq_parse,
};

static void rk_pcie_intx_isr(int irqno, void *param)
{
    rt_uint32_t ints;
    struct rt_pic_irq *pirq;
    struct rockchip_pcie *rk_pcie = param;

    ints = rockchip_pcie_readl_apb(rk_pcie, PCIE_CLIENT_INTR_STATUS_LEGACY);

    for (int pin = 0; ints && pin < RT_PCI_INTX_PIN_MAX; ++pin, ints >>= 1)
    {
        if ((ints & 1))
        {
            pirq = rt_pic_find_irq(&rk_pcie->intx_pic, pin);

            rt_pic_handle_isr(pirq);
        }
    }
}

static rt_err_t rockchip_pcie_init_irq(struct rockchip_pcie *rk_pcie)
{
    struct rt_ofw_node *np = rk_pcie->pci.dev->ofw_node, *intx_np;
    struct rt_pic *intx_pic = &rk_pcie->intx_pic;

    intx_np = rt_ofw_get_child_by_tag(np, "legacy-interrupt-controller");

    if (!intx_np)
    {
        LOG_E("INTx ofw node not found");

        return -RT_EIO;
    }

    rk_pcie->intx_irq = rt_ofw_get_irq(intx_np, 0);

    rt_ofw_node_put(intx_np);

    if (rk_pcie->intx_irq < 0)
    {
        rk_pcie->intx_irq = rt_ofw_get_irq_by_name(np, "legacy");
    }

    if (rk_pcie->intx_irq < 0)
    {
        LOG_E("INTx irq get fail");

        return rk_pcie->intx_irq;
    }

    intx_pic->priv_data = rk_pcie;
    intx_pic->ops = &rockchip_intx_ops;

    rt_pic_linear_irq(intx_pic, RT_PCI_INTX_PIN_MAX);

    rt_pic_user_extends(intx_pic);

    rt_ofw_data(np) = intx_pic;

    rt_hw_interrupt_install(rk_pcie->intx_irq, rk_pcie_intx_isr, rk_pcie, "rk-pcie-INTx");
    rt_hw_interrupt_umask(rk_pcie->intx_irq);

    return RT_EOK;
}

static rt_err_t rockchip_pcie_host_init(struct dw_pcie_rp *pp)
{
    rt_err_t err;
    struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
    struct rockchip_pcie *rk_pcie = to_rockchip_pcie(pci);
    rt_uint32_t val = HIWORD_UPDATE_BIT(PCIE_LTSSM_ENABLE_ENHANCE);

    if ((err = rockchip_pcie_init_irq(rk_pcie)))
    {
        return err;
    }

    /* LTSSM enable control mode */
    rockchip_pcie_writel_apb(rk_pcie, val, PCIE_CLIENT_HOT_RESET_CTRL);
    rockchip_pcie_writel_apb(rk_pcie, PCIE_CLIENT_RC_MODE, PCIE_CLIENT_GENERAL_CONTROL);

    return RT_EOK;
}

static const struct dw_pcie_host_ops rockchip_pcie_host_ops =
{
    .host_init = rockchip_pcie_host_init,
};

static rt_err_t rockchip_pcie_phy_init(struct rockchip_pcie *rk_pcie)
{
    rt_phy_status phy_status;
    struct rt_device *dev = rk_pcie->pci.dev;

    rk_pcie->phy = rt_phy_get_by_name(dev, "pcie-phy");
    if (!rk_pcie->phy)
    {
        LOG_E("Missing PHY");
        return -RT_ERROR;
    }

    phy_status = rt_phy_init(rk_pcie->phy, RT_NULL, 0, 0);
    if (phy_status)
    {
        rk_pcie->phy = RT_NULL;
        LOG_E("%s PHY fail", "Init");

        return -RT_EIO;
    }

    phy_status = rt_phy_power_on(rk_pcie->phy);
    if (phy_status)
    {
        rt_phy_exit(rk_pcie->phy, RT_NULL, 0);
        rk_pcie->phy = RT_NULL;

        LOG_E("%s PHY fail", "Power on");

        return -RT_EIO;
    }

    return RT_EOK;
}

static rt_err_t rockchip_pcie_resource_init(struct rockchip_pcie *rk_pcie)
{
    struct rt_device *dev = rk_pcie->pci.dev;

    rk_pcie->apb_base = rt_dm_dev_iomap_by_name(dev, "apb");
    if (!rk_pcie->apb_base)
    {
        rk_pcie->apb_base = rt_dm_dev_iomap_by_name(dev, "pcie-apb");
    }

    if (!rk_pcie->apb_base)
    {
        return -RT_EIO;
    }

    rk_pcie->rst_pin = rt_pin_get_named_pin(dev, "reset", 0, RT_NULL, RT_NULL);
    if (rk_pcie->rst_pin < 0 && rk_pcie->rst_pin != -RT_EEMPTY)
    {
        return (rt_err_t)rk_pcie->rst_pin;
    }

    if (rk_pcie->rst_pin >= 0)
    {
        rt_pin_mode(rk_pcie->rst_pin, PIN_MODE_OUTPUT);
        rt_pin_write(rk_pcie->rst_pin, PIN_HIGH);
    }

    rk_pcie->rstc = rt_reset_control_get_array(dev);

    if (rt_is_err(rk_pcie->rstc))
    {
        return rt_ptr_err(rk_pcie->rstc);
    }

    return RT_EOK;
}

static void rockchip_pcie_phy_deinit(struct rockchip_pcie *rk_pcie)
{
    rt_phy_exit(rk_pcie->phy, RT_NULL, 0);
    rt_phy_power_off(rk_pcie->phy);
}

static rt_err_t rockchip_pcie_clk_init(struct rockchip_pcie *rk_pcie)
{
    struct rt_device *dev = rk_pcie->pci.dev;

    rk_pcie->clk_arr = rt_clk_get_array(dev);

    if (rt_is_err(rk_pcie->clk_arr))
    {
        return rt_ptr_err(rk_pcie->clk_arr);
    }

    return rt_clk_array_prepare_enable(rk_pcie->clk_arr);
}

static void rockchip_pcie_free(struct rockchip_pcie *rk_pcie)
{
    if (rk_pcie->apb_base)
    {
        rt_iounmap(rk_pcie->apb_base);
    }

    if (rk_pcie->rstc)
    {
        rt_reset_control_put(rk_pcie->rstc);
    }

    if (rk_pcie->vpcie3v3)
    {
        rt_regulator_disable(rk_pcie->vpcie3v3);
    }

    if (rk_pcie->phy)
    {
        rockchip_pcie_phy_deinit(rk_pcie);
    }

    if (rk_pcie->clk_arr)
    {
        rt_clk_array_disable_unprepare(rk_pcie->clk_arr);
        rt_clk_array_put(rk_pcie->clk_arr);
    }

    rt_free(rk_pcie);
}

static rt_err_t rockchip_pcie_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    struct rt_device *dev = &pdev->parent;
    struct rockchip_pcie *rk_pcie = rt_calloc(1, sizeof(*rk_pcie));

    if (!rk_pcie)
    {
        return -RT_EINVAL;
    }

    rk_pcie->pci.dev = dev;
    rk_pcie->pci.ops = &dw_pcie_ops;
    rk_pcie->pci.pp.ops = &rockchip_pcie_host_ops;
    rk_pcie->intx_irq = -RT_EEMPTY;

    if ((err = rockchip_pcie_resource_init(rk_pcie)))
    {
        goto _free_res;
    }

    if ((err = rt_reset_control_assert(rk_pcie->rstc)))
    {
        goto _free_res;
    }

    rk_pcie->vpcie3v3 = rt_regulator_get_optional(dev, "vpcie3v3");

    if (!rt_is_err(rk_pcie->vpcie3v3))
    {
        if ((err = rt_regulator_enable(rk_pcie->vpcie3v3)))
        {
            goto _free_res;
        }
    }

    if ((err = rockchip_pcie_phy_init(rk_pcie)))
    {
        goto _free_res;
    }

    if ((err = rt_reset_control_deassert(rk_pcie->rstc)))
    {
        goto _free_res;
    }

    if ((err = rockchip_pcie_clk_init(rk_pcie)))
    {
        goto _free_res;
    }

    if ((err = dw_pcie_host_init(&rk_pcie->pci.pp)))
    {
        goto _free_res;
    }

    dev->user_data = rk_pcie;

    return RT_EOK;

_free_res:
    rockchip_pcie_free(rk_pcie);

    return err;
}

static rt_err_t rockchip_pcie_remove(struct rt_platform_device *pdev)
{
    struct rockchip_pcie *rk_pcie = pdev->parent.user_data;

    if (rk_pcie->intx_irq >= 0)
    {
        rt_hw_interrupt_mask(rk_pcie->intx_irq);
        rt_pic_detach_irq(rk_pcie->intx_irq, rk_pcie);
    }

    rockchip_pcie_free(rk_pcie);

    return RT_EOK;
}

static const struct rt_ofw_node_id rockchip_pcie_ofw_ids[] =
{
    { .compatible = "rockchip,rk3568-pcie" },
    { /* sentinel */ }
};

static struct rt_platform_driver rockchip_pcie_driver =
{
    .name = "rockchip-dw-pcie",
    .ids = rockchip_pcie_ofw_ids,

    .probe = rockchip_pcie_probe,
    .remove = rockchip_pcie_remove,
};
RT_PLATFORM_DRIVER_EXPORT(rockchip_pcie_driver);
