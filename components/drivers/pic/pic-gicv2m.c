/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-07     GuEe-GUI     first version
 */

#include <rthw.h>
#include <rtthread.h>

#define DBG_TAG "irqchip.gic-v2m"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <cpuport.h>
#include <ioremap.h>
#include <board.h>

#include <msi.h>
#include <rtof.h>
#include <rt_bitmap.h>
#include <rt_irqchip.h>
#include <irq-gicv2.h>
#include <irq-gicv3.h>

/*
* MSI_TYPER:
*   [31:26] Reserved
*   [25:16] lowest SPI assigned to MSI
*   [15:10] Reserved
*   [9:0]   Numer of SPIs assigned to MSI
*/
#define V2M_MSI_TYPER                   0x008
#define V2M_MSI_TYPER_BASE_SHIFT        16
#define V2M_MSI_TYPER_BASE_MASK         0x3ff
#define V2M_MSI_TYPER_NUM_MASK          0x3ff
#define V2M_MSI_SETSPI_NS               0x040
#define V2M_MIN_SPI                     32
#define V2M_MAX_SPI                     1019
#define V2M_MSI_IIDR                    0xfcc

#define V2M_MSI_TYPER_BASE_SPI(x)       (((x) >> V2M_MSI_TYPER_BASE_SHIFT) & V2M_MSI_TYPER_BASE_MASK)
#define V2M_MSI_TYPER_NUM_SPI(x)        ((x) & V2M_MSI_TYPER_NUM_MASK)

/* APM X-Gene with GICv2m MSI_IIDR register value */
#define XGENE_GICV2M_MSI_IIDR           0x06000170
/* Broadcom NS2 GICv2m MSI_IIDR register value */
#define BCM_NS2_GICV2M_MSI_IIDR         0x0000013f

/* List of flags for specific v2m implementation */
#define GICV2M_NEEDS_SPI_OFFSET         0x00000001
#define GICV2M_GRAVITON_ADDRESS_ONLY    0x00000002

struct gicv2m
{
    struct rt_irq_chip_desc parent;

    rt_list_t list;
    void *base;
    void *base_phy;
    rt_uint32_t spi_start;  /* The SPI number that MSIs start */
    rt_uint32_t spis_nr;    /* The number of SPIs for MSIs */
    rt_uint32_t spi_offset; /* Offset to be subtracted from SPI number */

    rt_bitmap_t *vectors;   /* MSI vector bitmap */
    rt_uint32_t flags;      /* Flags for v2m's specific implementation */

    void *gic;
};

static rt_list_t _v2m_nodes = RT_LIST_OBJECT_INIT(_v2m_nodes);
static struct rt_spinlock _v2m_lock = { 0 };

static rt_ubase_t gicv2m_get_msi_addr(struct gicv2m *v2m, int hwirq)
{
    rt_ubase_t ret;

    if (v2m->flags & GICV2M_GRAVITON_ADDRESS_ONLY)
    {
        ret = (rt_ubase_t)v2m->base_phy | ((hwirq - 32) << 3);
    }
    else
    {
        ret = (rt_ubase_t)v2m->base_phy + V2M_MSI_SETSPI_NS;
    }

    return ret;
}

static rt_bool_t is_msi_spi_valid(rt_uint32_t base, rt_uint32_t num)
{
    rt_bool_t ret = RT_TRUE;

    if (base < V2M_MIN_SPI)
    {
        LOG_E("Invalid MSI base SPI (base:%u)", base);

        ret = RT_FALSE;
    }
    else if ((num == 0) || (base + num > V2M_MAX_SPI))
    {
        LOG_E("Number of SPIs (%u) exceed maximum (%u)", num, V2M_MAX_SPI - V2M_MIN_SPI + 1);

        ret = RT_FALSE;
    }

    return ret;
}

void gicv2m_irq_mask(struct rt_irq_data *data)
{
    pci_msi_mask_irq(data);
    rt_irqchip_irq_parent_mask(data);
}

void gicv2m_irq_unmask(struct rt_irq_data *data)
{
    pci_msi_unmask_irq(data);
    rt_irqchip_irq_parent_unmask(data);
}

void gicv2m_compose_msi_msg(struct rt_irq_data *data, struct msi_msg *msg)
{
    struct gicv2m *v2m = data->chip_data;

    if (v2m)
    {
        rt_ubase_t addr = gicv2m_get_msi_addr(v2m, data->hwirq);

        msg->address_hi = ((rt_uint32_t)(((addr) >> 16) >> 16));
        msg->address_lo = ((rt_uint32_t)((addr) & RT_UINT32_MAX));

        if (v2m->flags & GICV2M_GRAVITON_ADDRESS_ONLY)
        {
            msg->data = 0;
        }
        else
        {
            msg->data = data->hwirq;
        }

        if (v2m->flags & GICV2M_NEEDS_SPI_OFFSET)
        {
            msg->data -= v2m->spi_offset;
        }
    }
}

int gicv2m_irq_alloc_msi(struct rt_irq_chip_desc *chip_desc, struct msi_desc *msi_desc)
{
    int irq = -1;
    struct gicv2m *v2m;
    rt_ubase_t level = rt_spin_lock_irqsave(&_v2m_lock);

    rt_list_for_each_entry(v2m, &_v2m_nodes, list)
    {
        int vector = rt_bitmap_next_clear_bit(v2m->vectors, 0, v2m->spis_nr);

        if (vector >= 0)
        {
            struct rt_irq_info spi =
            {
                .mode = RT_IRQ_MODE_EDGE_RISING,
                .type = RT_IRQ_TYPE_EXTERNAL,
                .msi_desc = msi_desc,
            };

            irq = rt_irqchip_chip_parent_map(chip_desc, v2m->gic, v2m->spi_start + v2m->spi_offset + vector, &spi);

            if (irq < 0)
            {
                continue;
            }

            rt_bitmap_set_bit(v2m->vectors, vector);

            break;
        }
    }

    rt_spin_unlock_irqrestore(&_v2m_lock, level);

    return irq;
}

void gicv2m_irq_free_msi(struct rt_irq_chip_desc *chip_desc, int irq)
{
    rt_ubase_t level = rt_spin_lock_irqsave(&_v2m_lock);

    if (!rt_list_isempty(&_v2m_nodes))
    {
        struct rt_irq_data irqdata = { .irq = -1 };

        rt_irqchip_search_irq(irq, &irqdata);

        if (irqdata.irq >= 0)
        {
            struct gicv2m *v2m = irqdata.chip_data;

            rt_bitmap_clear_bit(v2m->vectors, irqdata.hwirq - (v2m->spi_start + v2m->spi_offset));

            rt_irqchip_chip_parent_unmap(chip_desc, irq);
        }
    }

    rt_spin_unlock_irqrestore(&_v2m_lock, level);
}

static struct rt_irq_chip gicv2m_irq_chip =
{
    .name = "GICv2m",
    .irq_ack = rt_irqchip_irq_parent_ack,
    .irq_mask = gicv2m_irq_mask,
    .irq_unmask = gicv2m_irq_unmask,
    .irq_eoi = rt_irqchip_irq_parent_eoi,
    .irq_set_priority = rt_irqchip_irq_parent_set_priority,
    .irq_set_affinity = rt_irqchip_irq_parent_set_affinity,
    .irq_compose_msi_msg = gicv2m_compose_msi_msg,
    .irq_alloc_msi = gicv2m_irq_alloc_msi,
    .irq_free_msi = gicv2m_irq_free_msi,
};

static const struct rt_of_device_id gicv2m_of_match[] =
{
    { .compatible = "arm,gic-v2m-frame" },
    { /* sentinel */ }
};

rt_err_t gicv2m_of_probe(struct rt_device_node *node, const struct rt_of_device_id *id)
{
    rt_err_t ret = RT_EOK;
    rt_uint64_t regs[2];
    struct rt_device_node *v2m_np;

    rt_of_for_each_child_of_node(node, v2m_np)
    {
        rt_bool_t init_ok = RT_TRUE;
        struct gicv2m *v2m;
        rt_size_t bitmap_size;
        rt_uint32_t spi_start = 0, spis_nr = 0;

        if (!rt_of_device_is_available(v2m_np))
        {
            continue;
        }

        if (!rt_of_match_node(gicv2m_of_match, v2m_np))
        {
            continue;
        }

        if (!rt_of_find_property(v2m_np, "msi-controller", RT_NULL))
        {
            continue;
        }

        rt_memset(regs, 0, sizeof(regs));

        if (rt_of_address_to_array(v2m_np, regs) < 0)
        {
            continue;
        }

        if (!(v2m = rt_malloc(sizeof(*v2m))))
        {
            ret = -RT_ENOMEM;
            break;
        }

        rt_list_init(&v2m->list);
        v2m->base_phy = (void *)regs[0];
        v2m->base = rt_ioremap(v2m->base_phy, regs[1]);
        v2m->flags = 0;

        if (!rt_of_property_read_u32(v2m_np, "arm,msi-base-spi", &spi_start) &&
            !rt_of_property_read_u32(v2m_np, "arm,msi-num-spis", &spis_nr))
        {
            LOG_I("DT overriding V2M MSI_TYPER (base:%u, num:%u)", spi_start, spis_nr);
        }

        do {
            if (spi_start && spis_nr)
            {
                v2m->spi_start = spi_start;
                v2m->spis_nr = spis_nr;
            }
            else
            {
                rt_uint32_t typer;

                /* Graviton should always have explicit spi_start/spis_nr */
                if (v2m->flags & GICV2M_GRAVITON_ADDRESS_ONLY)
                {
                    init_ok = RT_FALSE;
                    break;
                }
                typer = HWREG32(v2m->base + V2M_MSI_TYPER);

                v2m->spi_start = V2M_MSI_TYPER_BASE_SPI(typer);
                v2m->spis_nr = V2M_MSI_TYPER_NUM_SPI(typer);
            }

            if (!is_msi_spi_valid(v2m->spi_start, v2m->spis_nr))
            {
                init_ok = RT_FALSE;
                break;
            }

            /*
             * APM X-Gene GICv2m implementation has an erratum where
             * the MSI data needs to be the offset from the spi_start
             * in order to trigger the correct MSI interrupt. This is
             * different from the standard GICv2m implementation where
             * the MSI data is the absolute value within the range from
             * spi_start to (spi_start + num_spis).
             *
             * Broadcom NS2 GICv2m implementation has an erratum where the MSI data
             * is 'spi_number - 32'
             *
             * Reading that register fails on the Graviton implementation
             */
            if (!(v2m->flags & GICV2M_GRAVITON_ADDRESS_ONLY))
            {
                switch (HWREG32(v2m->base + V2M_MSI_IIDR))
                {
                case XGENE_GICV2M_MSI_IIDR:
                    v2m->flags |= GICV2M_NEEDS_SPI_OFFSET;
                    v2m->spi_offset = v2m->spi_start;
                    break;
                case BCM_NS2_GICV2M_MSI_IIDR:
                    v2m->flags |= GICV2M_NEEDS_SPI_OFFSET;
                    v2m->spi_offset = 32;
                    break;
                }
            }

            bitmap_size = RT_BITMAP_LEN(v2m->spis_nr) * sizeof(rt_bitmap_t);

            if (!(v2m->vectors = rt_malloc(bitmap_size)))
            {
                ret = -RT_ENOMEM;
                init_ok = RT_FALSE;
                break;
            }

            rt_memset(v2m->vectors, 0, bitmap_size);
        } while (0);

        if (!init_ok)
        {
            rt_iounmap(v2m->base);
            rt_free(v2m);

            if (ret)
            {
                break;
            }
        }

        v2m->parent.chip = &gicv2m_irq_chip;
        v2m->parent.chip_data = v2m;
        v2m->gic = rt_of_data(node);

        rt_spin_lock(&_v2m_lock);
        rt_list_insert_after(&_v2m_nodes, &v2m->list);
        rt_spin_unlock(&_v2m_lock);

        rt_of_data(v2m_np) = &v2m->parent;

        rt_of_node_set_flag(v2m_np, OF_F_READY);
    }

    return ret;
}
