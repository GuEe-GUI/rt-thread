/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-25     GuEe-GUI     first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "pic.mip-msi"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define MIP_INT_RAISED          0x00
#define MIP_INT_CLEARED         0x10
#define MIP_INT_CFGL_HOST       0x20
#define MIP_INT_CFGH_HOST       0x30
#define MIP_INT_MASKL_HOST      0x40
#define MIP_INT_MASKH_HOST      0x50
#define MIP_INT_MASKL_VPU       0x60
#define MIP_INT_MASKH_VPU       0x70
#define MIP_INT_STATUSL_HOST    0x80
#define MIP_INT_STATUSH_HOST    0x90
#define MIP_INT_STATUSL_VPU     0xa0
#define MIP_INT_STATUSH_VPU     0xb0

struct mip_msi
{
    struct rt_pic parent;
    struct rt_pic *ppic;
    struct rt_ofw_node *pic_np;

    void *base;
    rt_ubase_t msg_addr;
    rt_uint32_t msi_base;       /* The SGI number that MSIs start */
    rt_uint32_t num_msis;       /* The number of SGIs for MSIs */
    rt_uint32_t msi_offset;     /* Shift the allocated msi up by N */
    rt_ubase_t *msi_map;

    struct rt_spinlock msi_map_lock;
};

#define raw_to_mip_msi(raw) rt_container_of(raw, struct mip_msi, parent)

static rt_err_t mip_msi_irq_init(struct rt_pic *pic)
{
    struct mip_msi *mip = raw_to_mip_msi(pic);

    mip->ppic = rt_ofw_data(mip->pic_np);

    return mip->ppic ? RT_EOK : -RT_ENOSYS;
}

static void mip_msi_irq_mask(struct rt_pic_irq *pirq)
{
    rt_pci_msi_mask_irq(pirq);
    rt_pic_irq_parent_mask(pirq);
}

static void mip_msi_irq_unmask(struct rt_pic_irq *pirq)
{
    rt_pci_msi_unmask_irq(pirq);
    rt_pic_irq_parent_unmask(pirq);
}

static void mip_msi_compose_msi_msg(struct rt_pic_irq *pirq, struct rt_pci_msi_msg *msg)
{
    struct mip_msi *mip = raw_to_mip_msi(pirq->pic);

    msg->address_hi = rt_upper_32_bits(mip->msg_addr);
    msg->address_lo = rt_lower_32_bits(mip->msg_addr);
    msg->data = pirq->hwirq;
}

static int mip_msi_irq_alloc_msi(struct rt_pic *pic, struct rt_pci_msi_desc *msi_desc)
{
    int irq, parent_irq, hwirq, hwirq_index;
    struct rt_pic_irq *pirq;
    struct mip_msi *mip = raw_to_mip_msi(pic);

    rt_spin_lock(&mip->msi_map_lock);

    hwirq_index = bitmap_next_clear_bit(mip->msi_map, 0, mip->num_msis);

    if (hwirq_index >= mip->num_msis)
    {
        irq = -RT_EEMPTY;
        goto _out_lock;
    }

    hwirq = hwirq_index + mip->msi_offset;

    parent_irq = mip->ppic->ops->irq_map(mip->ppic, hwirq, RT_IRQ_MODE_EDGE_RISING);
    if (parent_irq < 0)
    {
        irq = parent_irq;
        goto _out_lock;
    }

    irq = rt_pic_config_irq(pic, hwirq_index, hwirq);
    if (irq < 0)
    {
        goto _out_lock;
    }
    pirq = rt_pic_find_irq(pic, hwirq_index);

    rt_pic_cascade(pirq, parent_irq);

    bitmap_set_bit(mip->msi_map, hwirq_index);

_out_lock:
    rt_spin_unlock(&mip->msi_map_lock);

    return irq;
}

static void mip_msi_irq_free_msi(struct rt_pic *pic, int irq)
{
    struct rt_pic_irq *pirq;
    struct mip_msi *mip = raw_to_mip_msi(pic);

    pirq = rt_pic_find_pirq(pic, irq);

    if (!pirq)
    {
        return;
    }

    rt_spin_lock(&mip->msi_map_lock);

    rt_pic_uncascade(pirq);
    bitmap_clear_bit(mip->msi_map, pirq->hwirq - mip->msi_offset);

    rt_spin_unlock(&mip->msi_map_lock);
}

const static struct rt_pic_ops mip_msi_ops =
{
    .name = "MIP-MSI",
    .irq_init = mip_msi_irq_init,
    .irq_mask = mip_msi_irq_mask,
    .irq_unmask = mip_msi_irq_unmask,
    .irq_eoi = rt_pic_irq_parent_eoi,
    .irq_set_affinity = rt_pic_irq_parent_set_affinity,
    .irq_set_triger_mode = rt_pic_irq_parent_set_triger_mode,
    .irq_compose_msi_msg = mip_msi_compose_msi_msg,
    .irq_alloc_msi = mip_msi_irq_alloc_msi,
    .irq_free_msi = mip_msi_irq_free_msi,
    .flags = RT_PIC_F_IRQ_ROUTING,
};

static rt_err_t mip_msi_ofw_init(struct rt_ofw_node *np, const struct rt_ofw_node_id *id)
{
    rt_err_t err;
    struct mip_msi *mip = rt_calloc(1, sizeof(*mip));

    if (!mip)
    {
        return -RT_ENOMEM;
    }

    mip->pic_np = rt_ofw_find_irq_parent(np, RT_NULL);

    if (!mip->pic_np)
    {
        LOG_E("Unable to find PIC parent");
        err = -RT_EINVAL;
        goto _fail;
    }

    mip->base = rt_ofw_iomap(np, 0);

    if (!mip->base)
    {
        err = -RT_EIO;
        goto _fail;
    }

    if (rt_ofw_prop_read_u32(np, "brcm,msi-base-spi", &mip->msi_base))
    {
        LOG_E("Unable to parse MSI base");
        err = -RT_EINVAL;
        goto _fail;
    }

    if (rt_ofw_prop_read_u32(np, "brcm,msi-num-spis", &mip->num_msis))
    {
        LOG_E("Unable to parse MSI numbers");
        err = -RT_EINVAL;
        goto _fail;
    }

    if (rt_ofw_prop_read_u32(np, "brcm,msi-offset", &mip->msi_offset))
    {
        mip->msi_offset = 0;
    }

    if (rt_ofw_prop_read_u64(np, "brcm,msi-pci-addr", &mip->msg_addr))
    {
        LOG_E("Unable to parse MSI address");
        err = -RT_EINVAL;
        goto _fail;
    }

    mip->msi_map = rt_calloc(RT_BITS_TO_LONGS(mip->num_msis), sizeof(*mip->msi_map));

    if (!mip->msi_map)
    {
        err = -RT_ENOMEM;
        goto _fail;
    }

    /*
     * Begin with all MSI-Xs masked in for the host, masked out for the VPU,
     * and edge-triggered.
     */
    HWREG32(mip->base + MIP_INT_MASKL_HOST) = 0;
    HWREG32(mip->base + MIP_INT_MASKH_HOST) = 0;
    HWREG32(mip->base + MIP_INT_MASKL_VPU) = ~0;
    HWREG32(mip->base + MIP_INT_MASKH_VPU) = ~0;
    HWREG32(mip->base + MIP_INT_CFGL_HOST) = ~0;
    HWREG32(mip->base + MIP_INT_CFGH_HOST) = ~0;

    LOG_D("Found %d MSIx, starting at %d", mip->num_msis, mip->msi_base);

    rt_spin_lock_init(&mip->msi_map_lock);

    mip->parent.priv_data = mip;
    mip->parent.ops = &mip_msi_ops;

    rt_pic_linear_irq(&mip->parent, RT_BITS_TO_LONGS(mip->num_msis));
    rt_pic_user_extends(&mip->parent);

    rt_ofw_data(np) = &mip->parent;
    rt_ofw_node_set_flag(np, RT_OFW_F_READLY);

    return RT_EOK;

_fail:
    if (mip->base)
    {
        rt_iounmap(mip->base);
    }

    if (mip->msi_map)
    {
        rt_free(mip->msi_map);
    }

    rt_free(mip);

    return err;
}

static const struct rt_ofw_node_id mip_msi_ofw_ids[] =
{
    { .compatible = "brcm,bcm2712-mip-intc" },
    { /* sentinel */ }
};
RT_PIC_OFW_DECLARE(mip_msi, mip_msi_ofw_ids, mip_msi_ofw_init);
