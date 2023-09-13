/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-01-30     GuEe-GUI     first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "pic.gic-v3-its"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <mmu.h>
#include <cpuport.h>
#include <dt-bindings/size.h>
#include "pic-gicv3.h"
#include "pic-gic-common.h"

#define ITS_CMD_QUEUE_SIZE                  (64 * SIZE_KB)
#define ITS_CMD_QUEUE_ALIGN                 (64 * SIZE_KB)

#define ITS_MAX_LPI_NIRQS                   (64 * SIZE_KB)
#define ITS_MAX_LPI_NRBITS                  16           /* 64K => 1 << 16 */
#define ITS_LPI_CONFIG_TABLE_ALIGN          (64 * SIZE_KB)
#define ITS_LPI_CONFIG_PROP_DEFAULT_PRIO    GICD_INT_DEF_PRI
#define ITS_LPI_CONFIG_PROP_SHIFT           2
#define ITS_LPI_CONFIG_PROP_MASK            RT_GENMASK(7, ITS_LPI_CONFIG_PROP_SHIFT)

#define RDIST_FLAGS_PROPBASE_NEEDS_FLUSHING (1 << 0)
#define RDIST_FLAGS_RD_TABLES_PREALLOCATED  (1 << 1)
#define RDIST_FLAGS_FORCE_NON_SHAREABLE     (1 << 2)

#define ITS_FLAGS_CMDQ_NEEDS_FLUSHING       (1ULL << 0)
#define ITS_FLAGS_WORKAROUND_CAVIUM_22375   (1ULL << 1)
#define ITS_FLAGS_FORCE_NON_SHAREABLE       (1ULL << 2)

#define RD_LOCAL_LPI_ENABLED                RT_BIT(0)
#define RD_LOCAL_PENDTABLE_PREALLOCATED     RT_BIT(1)
#define RD_LOCAL_MEMRESERVE_DONE            RT_BIT(2)

struct gicv3_its
{
    struct rt_pic parent;
    rt_list_t list;

    void *base;
    void *base_phy;

    void *cmd_base;
    rt_ubase_t cmd_idx;
    rt_uint32_t flags;

    rt_size_t lpis;
    rt_size_t lpis_irq_base;

    void *lpi_table;
    void *lpi_pending_table;
    rt_size_t lpi_table_size;

    struct rt_spinlock lock;

    struct gicv3 *gic;
    struct rt_ofw_node *np;
};

#define raw_to_gicv3_its(raw) rt_container_of(raw, struct gicv3_its, parent)

static rt_list_t its_nodes = RT_LIST_OBJECT_INIT(its_nodes);

rt_inline void its_readq(struct gicv3_its *its, int off, rt_uint64_t *out_value)
{
    *out_value = HWREG32(its->base + off);
    *out_value |= (rt_uint64_t)HWREG32(its->base + off + 4) << 32;
}

rt_inline void its_writeq(struct gicv3_its *its, int off, rt_uint64_t value)
{
    HWREG32(its->base + off) = (rt_uint32_t)value;
    HWREG32(its->base + off + 4) = (rt_uint32_t)(value >> 32);
}

rt_inline void its_readl(struct gicv3_its *its, int off, rt_uint32_t *out_value)
{
    *out_value = HWREG32(its->base + off);
}

rt_inline void its_writel(struct gicv3_its *its, int off, rt_uint32_t value)
{
    HWREG32(its->base + off) = value;
}

rt_inline void *lpi_base_config(struct gicv3_its *its, int index)
{
    return &((rt_uint8_t *)its->lpi_table)[index + its->lpis_irq_base - 8192];
}

rt_inline void lpi_flush_config(struct gicv3_its *its, rt_uint8_t *conf)
{
    if ((its->gic->redist_flags & RDIST_FLAGS_PROPBASE_NEEDS_FLUSHING))
    {
        /* Clean D-cache under command. */
        rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, conf, sizeof(*conf));
    }
    else
    {
        /* DSB inner shareable, store */
        rt_hw_wmb();
    }
}

rt_inline void *gicr_rd_base(struct gicv3_its *its)
{
    return its->gic->redist_percpu_base[rt_hw_cpu_id()];
}

rt_inline rt_uint64_t *gicr_rd_flags(struct gicv3_its *its)
{
    return &its->gic->redist_percpu_flags[rt_hw_cpu_id()];
}

static rt_bool_t gicr_supports_plpis(struct gicv3_its *its)
{
    return !!(HWREG64(gicr_rd_base(its) + GICR_TYPER) & GICR_TYPER_PLPIS);
}

static rt_err_t redist_disable_lpis(struct gicv3_its *its)
{
    void *gicr = gicr_rd_base(its);
    rt_uint64_t timeout = 1000000L, val;

    if (!gicr_supports_plpis(its))
    {
        LOG_I("CPU#%d: LPIs not supported", rt_hw_cpu_id());
        return -RT_ENOSYS;
    }

    val = HWREG32(gicr + GICR_CTLR);
    if (!(val & GICR_CTLR_ENABLE_LPIS))
    {
        return RT_EOK;
    }

    /*
     * If coming via a CPU hotplug event, we don't need to disable
     * LPIs before trying to re-enable them. They are already
     * configured and all is well in the world.
     *
     * If running with preallocated tables, there is nothing to do.
     */
    if ((*gicr_rd_flags(its) & RD_LOCAL_LPI_ENABLED) ||
        (its->gic->flags & RDIST_FLAGS_RD_TABLES_PREALLOCATED))
    {
        return RT_EOK;
    }

    /*
     * From that point on, we only try to do some damage control.
     */
    LOG_W("CPU%d: Booted with LPIs enabled, memory probably corrupted", rt_hw_cpu_id());

    /* Disable LPIs */
    val &= ~GICR_CTLR_ENABLE_LPIS;
    HWREG32(gicr + GICR_CTLR) = val;

    /* Make sure any change to GICR_CTLR is observable by the GIC */
    rt_hw_barrier(dsb, sy);

    /*
     * Software must observe RWP==0 after clearing GICR_CTLR.EnableLPIs
     * from 1 to 0 before programming GICR_PEND{PROP}BASER registers.
     * Error out if we time out waiting for RWP to clear.
     */
    while (HWREG32(gicr + GICR_CTLR) & GICR_CTLR_RWP)
    {
        if (!timeout)
        {
            LOG_E("CPU#%d: Timeout while disabling LPIs", rt_hw_cpu_id());

            return -RT_ETIMEOUT;
        }

        rt_hw_us_delay(1);
        --timeout;
    }

    /*
     * After it has been written to 1, it is IMPLEMENTATION
     * DEFINED whether GICR_CTLR.EnableLPI becomes RES1 or can be
     * cleared to 0. Error out if clearing the bit failed.
     */
    if (HWREG32(gicr + GICR_CTLR) & GICR_CTLR_ENABLE_LPIS)
    {
        LOG_E("CPU%d: Failed to disable LPIs", rt_hw_cpu_id());

        return -RT_EBUSY;
    }

    return RT_EOK;
}

static void gicv3_its_cpu_init_lpis(struct gicv3_its *its)
{
    void *gicr;
    rt_ubase_t paddr;
    rt_uint64_t val, tmp;

    if (*gicr_rd_flags(its) & RD_LOCAL_LPI_ENABLED)
    {
        return;
    }

    gicr = gicr_rd_base(its);

    val = HWREG32(gicr + GICR_CTLR);

    if ((its->gic->redist_flags & RDIST_FLAGS_RD_TABLES_PREALLOCATED) &&
        (val & GICR_CTLR_ENABLE_LPIS))
    {
        *gicr_rd_flags(its) |= RD_LOCAL_PENDTABLE_PREALLOCATED;

        goto _out;
    }

    paddr = (rt_ubase_t)rt_kmem_v2p(its->lpi_pending_table);

    /* set PROPBASE */
    val = ((rt_ubase_t)rt_kmem_v2p(its->lpi_table) |
            GITS_CBASER_InnerShareable |
            GITS_CBASER_RaWaWb |
            ((ITS_MAX_LPI_NRBITS - 1) & GICR_PROPBASER_IDBITS_MASK));

    HWREG64(gicr + GICR_PROPBASER) = val;
    tmp = HWREG64(gicr + GICR_PROPBASER);

    if (its->gic->redist_flags & RDIST_FLAGS_FORCE_NON_SHAREABLE)
    {
        tmp &= ~GICR_PBASER_SHARE_MASK_ALL;
    }

    if ((tmp ^ val) & GICR_PBASER_SHARE_MASK_ALL)
    {
        if (!(tmp & GICR_PBASER_SHARE_MASK_ALL))
        {
            /* The HW reports non-shareable, we must remove the cacheability attributes as well */
            val &= ~(GICR_PBASER_SHARE_MASK_ALL | GICR_PBASER_INNER_MASK_ALL);
            val |= GICR_PBASER_nC;
            HWREG64(gicr + GICR_PROPBASER) = val;
        }

        LOG_I("Using cache flushing for LPI property table");
        its->gic->redist_flags |= RDIST_FLAGS_PROPBASE_NEEDS_FLUSHING;
    }

    val = (paddr | GICR_PBASER_InnerShareable | GICR_PBASER_RaWaWb);

    HWREG64(gicr + GICR_PENDBASER) = val;
    tmp = HWREG64(gicr + GICR_PENDBASER);

    if (its->gic->redist_flags & RDIST_FLAGS_FORCE_NON_SHAREABLE)
    {
        tmp &= ~GICR_PBASER_SHARE_MASK_ALL;
    }

    if (!(tmp & GICR_PBASER_SHARE_MASK_ALL))
    {
        /*
         * The HW reports non-shareable, we must remove the
         * cacheability attributes as well.
         */
        val &= ~(GICR_PBASER_SHARE_MASK_ALL | GICR_PBASER_INNER_MASK_ALL);
        val |= GICR_PBASER_nC;
        HWREG64(gicr + GICR_PENDBASER) = val;
    }

    /* Enable LPIs */
    val = HWREG32(gicr + GICR_CTLR);
    val |= GICR_CTLR_ENABLE_LPIS;
    HWREG32(gicr + GICR_CTLR) = val;

    rt_hw_barrier(dsb, sy);

_out:
    *gicr_rd_flags(its) |= RD_LOCAL_LPI_ENABLED;
}

static rt_err_t gicv3_its_irq_init(struct rt_pic *pic)
{
    rt_err_t err;
    struct gicv3_its *its = raw_to_gicv3_its(pic);

    if ((err = redist_disable_lpis(its)))
    {
        return err;
    }

    gicv3_its_cpu_init_lpis(its);

    return RT_EOK;
}

static void gicv3_its_irq_mask(struct rt_pic_irq *pirq)
{
    struct gicv3_its *its = raw_to_gicv3_its(pirq->pic);
    rt_uint8_t *conf = lpi_base_config(its, pirq->hwirq);

    *conf &= ~GITS_LPI_CFG_ENABLED;
    lpi_flush_config(its, conf);

    // its_cmd_inv(dev, girq->gi_its_dev, girq);
}

static void gicv3_its_irq_unmask(struct rt_pic_irq *pirq)
{
    struct gicv3_its *its = raw_to_gicv3_its(pirq->pic);
    rt_uint8_t *conf = lpi_base_config(its, pirq->hwirq);

    *conf |= GITS_LPI_CFG_ENABLED;
    lpi_flush_config(its, conf);

    // its_cmd_inv(dev, girq->gi_its_dev, girq);
}

static rt_err_t gicv3_its_set_priority(struct rt_pic_irq *pirq, rt_uint32_t priority)
{
    struct gicv3_its *its = raw_to_gicv3_its(pirq->pic);
    rt_uint8_t *conf = lpi_base_config(its, pirq->hwirq);

    *conf = (priority << ITS_LPI_CONFIG_PROP_SHIFT) | (*conf & (~ITS_LPI_CONFIG_PROP_MASK));
    lpi_flush_config(its, conf);

    // its_cmd_inv(dev, girq->gi_its_dev, girq);

    return RT_EOK;
}

static rt_err_t gicv3_its_set_affinity(struct rt_pic_irq *pirq, bitmap_t *affinity)
{
    rt_err_t err = RT_EOK;

    return err;
}

static void gicv3_its_irq_compose_msi_msg(struct rt_pic_irq *pirq, struct rt_pci_msi_msg *msg)
{
}

static int gicv3_its_irq_alloc_msi(struct rt_pic *pic, struct rt_pci_msi_desc *msi_desc)
{
    int irq = -1;

    return irq;
}

static void gicv3_its_irq_free_msi(struct rt_pic *pic, int irq)
{
}

const static struct rt_pic_ops gicv3_its_ops =
{
    .name = "GICv3-ITS",
    .irq_init = gicv3_its_irq_init,
    .irq_ack = rt_pic_irq_parent_ack,
    .irq_mask = gicv3_its_irq_mask,
    .irq_unmask = gicv3_its_irq_unmask,
    .irq_eoi = rt_pic_irq_parent_eoi,
    .irq_set_priority = gicv3_its_set_priority,
    .irq_set_affinity = gicv3_its_set_affinity,
    .irq_compose_msi_msg = gicv3_its_irq_compose_msi_msg,
    .irq_alloc_msi = gicv3_its_irq_alloc_msi,
    .irq_free_msi = gicv3_its_irq_free_msi,
    .flags = RT_PIC_F_IRQ_ROUTING,
};

static rt_err_t its_cmd_queue_init(struct gicv3_its *its)
{
    void *cmd_phy_base;
    rt_uint32_t ctlr;
    rt_uint64_t baser, tmp;

    its->cmd_base = rt_malloc_align(ITS_CMD_QUEUE_SIZE, ITS_CMD_QUEUE_ALIGN);

    if (!its->cmd_base)
    {
        return -RT_ENOMEM;
    }

    its->cmd_idx = 0;
    its->flags = 0;

    cmd_phy_base = rt_kmem_v2p(its->cmd_base);

    baser = GITS_CBASER_VALID | GITS_CBASER_RaWaWb | GITS_CBASER_InnerShareable | \
            ((rt_uint64_t)cmd_phy_base) | (ITS_CMD_QUEUE_SIZE / (4 * SIZE_KB) - 1);

    its_writeq(its, GITS_CBASER, baser);

    its_readq(its, GITS_CBASER, &tmp);

    if (its->flags & ITS_FLAGS_FORCE_NON_SHAREABLE)
    {
        tmp &= ~GITS_CBASER_SHARE_MASK_ALL;
    }

    if ((tmp ^ baser) & GITS_CBASER_SHARE_MASK_ALL)
    {
        if (!(tmp & GITS_CBASER_SHARE_MASK_ALL))
        {
            /* The HW reports non-shareable, we must remove the cacheability attributes as well */
            baser &= ~(GITS_CBASER_SHARE_MASK_ALL | GITS_CBASER_INNER_MASK_ALL);
            baser |= GITS_CBASER_nC;

            its_writeq(its, GITS_CBASER, baser);
        }

        LOG_I("Using cache flushing for cmd queue");
        its->flags |= ITS_FLAGS_CMDQ_NEEDS_FLUSHING;
    }

    /* Get the next command from the start of the buffer */
    its_writeq(its, GITS_CWRITER, 0);

    its_readl(its, GITS_CTLR, &ctlr);
    ctlr |= GITS_CTLR_ENABLE;
    its_writel(its, GITS_CTLR, ctlr);

    return RT_EOK;
}

static rt_err_t its_lpi_table_init(struct gicv3_its *its)
{
    rt_uint32_t lpis, id_bits, numlpis = 1UL << GICD_TYPER_NUM_LPIS(its->gic->gicd_typer);

    if ((its->gic->redist_flags & RDIST_FLAGS_RD_TABLES_PREALLOCATED))
    {
        rt_uint64_t val = HWREG64(gicr_rd_base(its) + GICR_PROPBASER);
        id_bits = (val & GICR_PROPBASER_IDBITS_MASK) + 1;
    }
    else
    {
        id_bits = rt_min_t(rt_uint32_t, GICD_TYPER_ID_BITS(its->gic->gicd_typer), ITS_MAX_LPI_NRBITS);
    }

    lpis = (1UL << id_bits) - 8192;

    if (numlpis > 2 && numlpis > lpis)
    {
        lpis = numlpis;
        LOG_W("Using hypervisor restricted LPI range [%u]", lpis);
    }

    its->lpis = lpis;
    its->lpi_table_size = lpis; /* LPI Configuration table entry is 1 byte */
    its->lpi_table = rt_malloc_align(its->lpi_table_size * 2, ITS_LPI_CONFIG_TABLE_ALIGN);

    if (!its->lpi_table)
    {
        return -RT_ENOMEM;
    }

    its->lpi_pending_table = its->lpi_table + its->lpi_table_size;

    /* Set the default configuration */
    rt_memset(its->lpi_table, ITS_LPI_CONFIG_PROP_DEFAULT_PRIO | GITS_LPI_CFG_GROUP1, its->lpi_table_size);

    /* Initializing the allocator is just the same as freeing the full range of LPIs. */

    /* Flush the table to memory */
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, its->lpi_table, its->lpi_table_size);

    LOG_D("ITS: Allocator initialized for %u LPIs", lpis);

    return RT_EOK;
}

static void its_init_fail(struct gicv3_its *its)
{
    if (its->base)
    {
        rt_iounmap(its->base);
    }

    if (its->cmd_base)
    {
        rt_free_align(its->cmd_base);
    }

    rt_free(its);
}

static rt_err_t its_quirk_cavium_22375(void *data)
{
    struct gicv3_its *its = data;

    its->flags |= ITS_FLAGS_WORKAROUND_CAVIUM_22375;

    return RT_EOK;
}

static rt_err_t its_enable_rk3588001(void *data)
{
    struct gicv3_its *its = data;
    struct gicv3 *gic = its->gic;

    if (!rt_ofw_machine_is_compatible("rockchip,rk3588") &&
        !rt_ofw_machine_is_compatible("rockchip,rk3588s"))
    {
        return -RT_EINVAL;
    }

    its->flags |= ITS_FLAGS_FORCE_NON_SHAREABLE;
    gic->redist_flags |= RDIST_FLAGS_FORCE_NON_SHAREABLE;

    return RT_EOK;
}

static rt_err_t its_set_non_coherent(void *data)
{
    struct gicv3_its *its = data;

    if (!rt_ofw_prop_read_bool(its->np, "dma-noncoherent"))
    {
        return -RT_EINVAL;
    }

    its->flags |= ITS_FLAGS_FORCE_NON_SHAREABLE;

    return RT_EOK;
}

static const struct gic_quirk _its_quirks[] =
{
    {
        .desc       = "ITS: Cavium ThunderX errata: 22375, 24313",
        .iidr       = 0xa100034c,
        .iidr_mask  = 0xffff0fff,
        .init       = its_quirk_cavium_22375,
    },
    {
        .desc       = "ITS: Rockchip erratum RK3588001",
        .iidr       = 0x0201743b,
        .iidr_mask  = 0xffffffff,
        .init       = its_enable_rk3588001,
    },
    {
        .desc       = "ITS: non-coherent attribute",
        .compatible = "arm,gic-v3-its",
        .init       = its_set_non_coherent,
    },
    { /* sentinel */ }
};

static const struct rt_ofw_node_id gicv3_its_ofw_match[] =
{
    { .compatible = "arm,gic-v3-its" },
    { /* sentinel */ }
};

rt_err_t gicv3_its_ofw_probe(struct rt_ofw_node *np, const struct rt_ofw_node_id *id)
{
    rt_err_t err = -RT_EEMPTY;
    struct gicv3_its *its;
    struct rt_ofw_node *its_np;

    rt_ofw_foreach_available_child_node(np, its_np)
    {
        if (!rt_ofw_node_match(its_np, gicv3_its_ofw_match))
        {
            continue;
        }

        if (!rt_ofw_prop_read_bool(its_np, "msi-controller"))
        {
            continue;
        }

        if (!(its = rt_malloc(sizeof(struct gicv3_its))))
        {
            rt_ofw_node_put(its_np);

            err = -RT_ENOMEM;
            goto _free_all;
        }

        its->base = rt_ofw_iomap(its_np, 0);

        if (!its->base)
        {
            LOG_E("%s: IO map failed", rt_ofw_node_full_name(its_np));
            its_init_fail(its);
            continue;
        }

        /* Make sure ALL the ITS are reset before we probe any, as they may be sharing memory */
        for (int i = 0; i < GITS_TRANSLATION_TABLE_DESCRIPTORS_NR; ++i)
        {
            its_writeq(its, GITS_BASER + (i << 3), 0);
        }

        its->np = its_np;
        rt_list_init(&its->list);
        rt_list_insert_before(&its_nodes, &its->list);
    }

    rt_list_for_each_entry(its, &its_nodes, list)
    {
        its->base_phy = rt_kmem_v2p(its->base);
        its->gic = rt_ofw_data(np);

        if ((err = its_lpi_table_init(its)))
        {
            goto _fail;
        }

        gic_common_init_quirk_hw(HWREG32(its->base + GITS_IIDR), _its_quirks, its);
        gic_common_init_quirk_ofw(its->np, _its_quirks, its);

        if ((err = its_cmd_queue_init(its)))
        {
            goto _fail;
        }

        rt_spin_lock_init(&its->lock);

        its->parent.priv_data = its;
        its->parent.ops = &gicv3_its_ops;

        rt_pic_linear_irq(&its->parent, its->gic->lpi_nr);
        rt_pic_user_extends(&its->parent);

        its_np = its->np;
        rt_ofw_data(its_np) = &its->parent;
        rt_ofw_node_set_flag(its_np, RT_OFW_F_READLY);

        continue;

    _fail:
        its_init_fail(its);

        if (err == -RT_ENOMEM)
        {
            break;
        }
    }

    return err;

_free_all:
    rt_list_for_each_entry(its, &its_nodes, list)
    {
        rt_free(its);
    }

    return err;
}
