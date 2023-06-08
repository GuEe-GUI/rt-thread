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

#define DBG_TAG "irqchip.gic-v3-its"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <mmu.h>
#include <cpuport.h>
#include <ioremap.h>

#include <msi.h>
#include <rtof.h>
#include <rt_bitmap.h>
#include <rt_irqchip.h>
#include <irq-gicv3.h>
#include <irq-gic-common.h>
#include <dt-bindings/size.h>

#define ITS_CMD_QUEUE_SIZE  (64 * KB)
#define ITS_CMD_QUEUE_ALIGN (64 * KB)

#define ITS_MAX_LPI_NIRQS                   (64 * KB)
#define ITS_MAX_LPI_NRBITS                  16           /* 64K => 1 << 16 */
#define ITS_LPI_CONFIG_TABLE_ALIGN          (64 * KB)
#define ITS_LPI_CONFIG_PROP_DEFAULT_PRIO    GICD_INT_DEF_PRI

#define RDIST_FLAGS_PROPBASE_NEEDS_FLUSHING (1 << 0)
#define RDIST_FLAGS_RD_TABLES_PREALLOCATED  (1 << 1)

#define ITS_FLAGS_CMDQ_NEEDS_FLUSHING       (1ULL << 0)
#define ITS_FLAGS_WORKAROUND_CAVIUM_22375   (1ULL << 1)
#define ITS_FLAGS_WORKAROUND_CAVIUM_23144   (1ULL << 2)

struct gicv3_its
{
    struct rt_irq_chip_desc parent;
    rt_list_t list;

    void *base;
    void *base_phy;

    void *cmd_base;
    rt_ubase_t cmd_idx;
    rt_uint32_t flags;

    rt_size_t lpis;
    void *conf_table;
    rt_size_t conf_table_size;

    struct rt_spinlock lock;

    struct gicv3 *gic;
};

static rt_list_t _its_nodes = RT_LIST_OBJECT_INIT(_its_nodes);

rt_inline void its_readq(struct gicv3_its *its, int off, rt_uint64_t *out_value)
{
    *out_value = HWREG64(its->base + off);
}

rt_inline void its_writeq(struct gicv3_its *its, int off, rt_uint64_t value)
{
    HWREG64(its->base + off) = value;
}

rt_inline void its_readl(struct gicv3_its *its, int off, rt_uint32_t *out_value)
{
    *out_value = HWREG32(its->base + off);
}

rt_inline void its_writel(struct gicv3_its *its, int off, rt_uint32_t value)
{
    HWREG32(its->base + off) = value;
}

static rt_err_t gicv3_its_irq_init(void)
{
    return RT_EOK;
}

static void gicv3_its_irq_mask(struct rt_irq_data *data)
{
    rt_uint8_t *conf;
    struct gicv3_its *its = data->chip_data;
}

static void gicv3_its_irq_unmask(struct rt_irq_data *data)
{

}

static rt_err_t gicv3_its_set_priority(struct rt_irq_data *data, rt_uint32_t priority)
{
    rt_err_t ret = RT_EOK;

    return ret;
}

static rt_err_t gicv3_its_set_affinity(struct rt_irq_data *data, rt_bitmap_t *affinity)
{
    rt_err_t ret = RT_EOK;

    return ret;
}

static void gicv3_its_irq_compose_msi_msg(struct rt_irq_data *data, struct msi_msg *msg)
{
}

static int gicv3_its_irq_alloc_msi(struct rt_irq_chip_desc *chip_desc, struct msi_desc *msi_desc)
{
    int ret = -1;

    return ret;
}

static void gicv3_its_irq_free_msi(struct rt_irq_chip_desc *chip_desc, int irq)
{
}

static struct rt_irq_chip gicv3_its_irq_chip =
{
    .name = "GICv3-ITS",
    .irq_init = gicv3_its_irq_init,
    .irq_ack = rt_irqchip_irq_parent_ack,
    .irq_mask = gicv3_its_irq_mask,
    .irq_unmask = gicv3_its_irq_unmask,
    .irq_eoi = rt_irqchip_irq_parent_eoi,
    .irq_set_priority = gicv3_its_set_priority,
    .irq_set_affinity = gicv3_its_set_affinity,
    .irq_compose_msi_msg = gicv3_its_irq_compose_msi_msg,
    .irq_alloc_msi = gicv3_its_irq_alloc_msi,
    .irq_free_msi = gicv3_its_irq_free_msi,
};

static rt_err_t its_cmd_queue_init(struct gicv3_its *its)
{
    rt_err_t ret = RT_EOK;

    its->cmd_base = rt_malloc_align(ITS_CMD_QUEUE_SIZE, ITS_CMD_QUEUE_ALIGN);

    if (its->cmd_base)
    {
        void *cmd_phy_base;
        rt_uint64_t baser, tmp;

        its->cmd_idx = 0;
        its->flags = 0;

        cmd_phy_base = rt_kmem_v2p(its->cmd_base);

        baser = GITS_CBASER_VALID | GITS_CBASER_RaWaWb | GITS_CBASER_InnerShareable | \
                ((rt_uint64_t)cmd_phy_base) | (ITS_CMD_QUEUE_SIZE / (4 * KB) - 1);

        its_writeq(its, GITS_CBASER, baser);

        its_readq(its, GITS_CBASER, &tmp);

        if ((tmp & GITS_BASER_SHAREABILITY(GITS_CBASER, SHARE_MASK)) != GITS_CBASER_InnerShareable)
        {
        }

        /* Get the next command from the start of the buffer */
        its_writeq(its, GITS_CWRITER, 0);
    }
    else
    {
        ret = -RT_ENOMEM;
    }

    return ret;
}

static rt_err_t its_conftable_init(struct gicv3_its *its)
{
    rt_err_t ret = RT_EOK;
    // void *conf_table;
    // rt_uint32_t lpis, id_bits, numlpis = 1UL << GICD_TYPER_NUM_LPIS(its->gic->gicd_typer);

    // if (its->gic->redist_flags & RDIST_FLAGS_RD_TABLES_PREALLOCATED)
    // {
    //     rt_uint64_t val = gicr_read_propbaser(gic_data_rdist_rd_base() + GICR_PROPBASER);
    //     id_bits = (val & GICR_PROPBASER_IDBITS_MASK) + 1;
    // }
    // else
    // {
    //     id_bits = min_t(rt_uint32_t, GICD_TYPER_ID_BITS(its->gic->gicd_typer), ITS_MAX_LPI_NRBITS);
    // }

    // lpis = (1UL << id_bits) - 8192;

    // if (numlpis > 2 && numlpis > lpis)
    // {
    //     lpis = numlpis;
    //     LOG_W("Using hypervisor restricted LPI range [%u]", lpis);
    // }

    // its->lpis = lpis;
    // its->conf_table_size = lpis; /* LPI Configuration table entry is 1 byte */
    // its->conf_table = rt_malloc_align(its->conf_table_size, ITS_LPI_CONFIG_TABLE_ALIGN);

    // if (its->conf_table)
    // {
    //     /* Set the default configuration */
    //     rt_memset(its->conf_table, ITS_LPI_CONFIG_PROP_DEFAULT_PRIO | GITS_LPI_CFG_GROUP1, its->conf_table_size);

    //     /* Initializing the allocator is just the same as freeing the full range of LPIs. */

    //     /* Flush the table to memory */
    //     rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, its->conf_table, its->conf_table_size);

    //     LOG_D("ITS: Allocator initialized for %u LPIs", lpis);
    // }
    // else
    // {
    //     ret = -RT_ENOMEM;
    // }

    return ret;
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

static const struct gic_quirk _its_quirks[] =
{
    { /* sentinel */ }
};

static const struct rt_of_device_id gicv3_its_of_match[] =
{
    { .compatible = "arm,gic-v3-its" },
    { /* sentinel */ }
};

rt_err_t gicv3_its_of_probe(struct rt_device_node *node, const struct rt_of_device_id *id)
{
    rt_err_t ret = -RT_EEMPTY;
    rt_uint64_t regs[2];
    struct rt_device_node *its_np;

    rt_of_for_each_child_of_node(node, its_np)
    {
        struct gicv3_its *its;

        if (!rt_of_device_is_available(its_np))
        {
            continue;
        }

        if (!rt_of_match_node(gicv3_its_of_match, its_np))
        {
            continue;
        }

        if (!rt_of_find_property(its_np, "msi-controller", RT_NULL))
        {
            continue;
        }

        rt_memset(regs, 0, sizeof(regs));

        if (rt_of_address_to_array(its_np, regs) < 0)
        {
            continue;
        }

        if (!(its = rt_malloc(sizeof(struct gicv3_its))))
        {
            ret = -RT_ENOMEM;
            break;
        }

        rt_list_init(&its->list);
        its->base_phy = (void *)regs[0];
        its->base = rt_ioremap(its->base_phy, regs[1]);

        its->parent.chip = &gicv3_its_irq_chip;
        its->parent.chip_data = its;
        its->gic = rt_of_data(node);

        if (its_conftable_init(its))
        {
            its_init_fail(its);
            continue;
        }

        gic_common_init_quirk_hw(HWREG32(its->base + GITS_IIDR), _its_quirks, its->gic);

        if (its_cmd_queue_init(its))
        {
            its_init_fail(its);
            continue;
        }

        rt_spin_lock_init(&its->lock);
        rt_list_insert_after(&_its_nodes, &its->list);
        rt_of_data(its_np) = &its->parent;

        rt_of_node_set_flag(its_np, OF_F_READY);

        ret = RT_EOK;
    }

    if (!ret)
    {
        /* Install forced that call `gicv3_its_irq_init` auto */
        rt_irqchip_install_forced(&gicv3_its_irq_chip);
    }

    return ret;
}
