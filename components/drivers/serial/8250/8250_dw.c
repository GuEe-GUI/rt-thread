/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-22     GuEe-GUI     first version
 */

/*
 * The Synopsys DesignWare 8250 has an extra feature whereby it detects if the
 * LCR is written whilst busy. If it is, then a busy detect interrupt is
 * raised, the LCR needs to be rewritten and the uart status register read.
 */

#include <rtthread.h>

#include <ioremap.h>

#include <rtof.h>
#include <rt_irqchip.h>
#include <serial8250.h>
#include <serial_internal.h>

/* Offsets for the DesignWare specific registers */
#define DW_UART_USR         0x1f    /* UART Status Register */
#define DW_UART_DMASA       0xa8    /* DMA Software Ack */

#define OCTEON_UART_USR     0x27    /* UART Status Register */

#define RZN1_UART_TDMACR    0x10c   /* DMA Control Register Transmit Mode */
#define RZN1_UART_RDMACR    0x110   /* DMA Control Register Receive Mode */

/* DesignWare specific register fields */
#define DW_UART_MCR_SIRE    RT_BIT(6)

/* Renesas specific register fields */
#define RZN1_UART_xDMACR_DMA_EN         RT_BIT(0)
#define RZN1_UART_xDMACR_1_WORD_BURST   (0 << 1)
#define RZN1_UART_xDMACR_4_WORD_BURST   (1 << 1)
#define RZN1_UART_xDMACR_8_WORD_BURST   (2 << 1)
#define RZN1_UART_xDMACR_BLK_SZ(x)      ((x) << 3)

/* Quirks */
#define DW_UART_QUIRK_OCTEON            RT_BIT(0)
#define DW_UART_QUIRK_ARMADA_38X        RT_BIT(1)
#define DW_UART_QUIRK_SKIP_SET_RATE     RT_BIT(2)
#define DW_UART_QUIRK_IS_DMA_FC         RT_BIT(3)

struct dw8250_platform_data
{
    rt_uint8_t usr_reg;
    rt_uint32_t cpr_val;
    rt_uint32_t quirks;
};

struct dw8250
{
    struct serial8250 parent;
    struct rt_spinlock spinlock;

    rt_bool_t uart_16550_compatible;
    struct dw8250_platform_data *platform_data;
};

#define to_dw8250(serial8250) rt_container_of(serial8250, struct dw8250, parent)

static void dw8250_check_lcr(struct serial8250 *serial, int value)
{
    void *offset = (void *)(serial->base + (UART_LCR << serial->regshift));
    int tries = 1000;

    /* Make sure LCR write wasn't ignored */
    while (tries--)
    {
        rt_uint32_t lcr = serial->serial_in(serial, UART_LCR);

        if ((value & ~UART_LCR_SPAR) == (lcr & ~UART_LCR_SPAR))
        {
            break;
        }

        serial->serial_out(serial, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
        serial->serial_in(serial, UART_RX);

        if (serial->iotype == PORT_MMIO32)
        {
            HWREG32(offset) = value;
        }
        else if (serial->iotype == PORT_MMIO32BE)
        {
            HWREG32(offset) = rt_cpu_to_be32(value);
        }
        else
        {
            HWREG8(offset) = value;
        }
    }
}

static void dw8250_serial_out32(struct serial8250 *serial, int offset, int value)
{
    struct dw8250 *dw8250 = to_dw8250(serial);

    HWREG32(serial->base + (offset << serial->regshift)) = value;

    if (offset == UART_LCR && !dw8250->uart_16550_compatible)
    {
        dw8250_check_lcr(serial, value);
    }
}

static rt_uint32_t dw8250_serial_in32(struct serial8250 *serial, int offset)
{
    return HWREG32(serial->base + (offset << serial->regshift));
}

static rt_err_t dw8250_isr(struct serial8250 *serial, int irq)
{
    unsigned int iir, status;
    struct dw8250 *dw8250 = to_dw8250(serial);

    iir = serial8250_in(serial, UART_IIR);

    /* If don't do this in non-DMA mode then the "RX TIMEOUT" interrupt will fire forever. */
    if ((iir & 0x3f) == UART_IIR_RX_TIMEOUT)
    {
        rt_base_t level = rt_spin_lock_irqsave(&dw8250->spinlock);

        status = serial8250_in(serial, UART_LSR);

        if (!(status & (UART_LSR_DR | UART_LSR_BI)))
        {
            serial8250_in(serial, UART_RX);
        }

        rt_spin_unlock_irqrestore(&dw8250->spinlock, level);
    }

    if (!(iir & UART_IIR_NO_INT))
    {
        rt_hw_serial_isr(&serial->parent, RT_SERIAL_EVENT_RX_IND);
    }

    if ((iir & UART_IIR_BUSY) == UART_IIR_BUSY)
    {
        /* Clear the USR */
        serial8250_in(serial, dw8250->platform_data->usr_reg);
    }

    return RT_EOK;
}

static void dw8250_remove(struct serial8250 *serial)
{
    rt_free(rt_container_of(serial, struct dw8250, parent));
}

static rt_err_t dw8250_probe(struct rt_device_node *node, const struct rt_of_device_id *id)
{
    rt_err_t ret = RT_EOK;
    struct serial8250 *serial = RT_NULL;
    struct dw8250 *dw8250 = serial8250_alloc(dw8250);

    if (dw8250)
    {
        struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

        // TODO: remove
        config.baud_rate = 1500000;
        serial = &dw8250->parent;
        serial->base = rt_of_iomap(node, 0);

        if (serial->base)
        {
            rt_uint32_t val;

            serial->irq = rt_of_irq_get(node, 0);

            serial->parent.ops = &serial8250_uart_ops;
            serial->parent.config = config;
            // TODO "clock-frequency"
            serial->freq = 24000000;
            rt_of_property_read_u32(node, "reg-shift", &serial->regshift);
            if (!rt_of_property_read_u32(node, "reg-io-width", &val) && val == 4)
            {
                serial->iotype = PORT_MMIO32;
                serial->serial_in = &dw8250_serial_in32;
                serial->serial_out = &dw8250_serial_out32;
            }
            serial->handle_irq = &dw8250_isr;
            serial->remove = &dw8250_remove;
            serial->data = dw8250;

            rt_spin_lock_init(&dw8250->spinlock);
            dw8250->uart_16550_compatible = rt_of_property_read_bool(node, "snps,uart-16550-compatible");
            dw8250->platform_data = (struct dw8250_platform_data *)id->data;
            rt_of_data(node) = &serial->parent;

            ret = serial8250_setup(serial);
        }
        else
        {
            rt_free(dw8250);
            ret = -RT_ERROR;
        }
    }
    else
    {
        ret = -RT_ENOMEM;
    }

    return ret;
}

static const struct dw8250_platform_data dw8250_dw_apb =
{
    .usr_reg = DW_UART_USR,
};

static const struct dw8250_platform_data dw8250_octeon_3860_data =
{
    .usr_reg = OCTEON_UART_USR,
    .quirks = DW_UART_QUIRK_OCTEON,
};

static const struct dw8250_platform_data dw8250_armada_38x_data =
{
    .usr_reg = DW_UART_USR,
    .quirks = DW_UART_QUIRK_ARMADA_38X,
};

static const struct dw8250_platform_data dw8250_renesas_rzn1_data =
{
    .usr_reg = DW_UART_USR,
    .cpr_val = 0x00012f32,
    .quirks = DW_UART_QUIRK_IS_DMA_FC,
};

static const struct dw8250_platform_data dw8250_starfive_jh7100_data =
{
    .usr_reg = DW_UART_USR,
    .quirks = DW_UART_QUIRK_SKIP_SET_RATE,
};

static const struct rt_of_device_id dw8250_of_match[] =
{
    { .type = "ttyS", .compatible = "snps,dw-apb-uart", .data = &dw8250_dw_apb },
    { .type = "ttyS", .compatible = "cavium,octeon-3860-uart", .data = &dw8250_octeon_3860_data },
    { .type = "ttyS", .compatible = "marvell,armada-38x-uart", .data = &dw8250_armada_38x_data },
    { .type = "ttyS", .compatible = "renesas,rzn1-uart", .data = &dw8250_renesas_rzn1_data },
    { .type = "ttyS", .compatible = "starfive,jh7100-uart", .data = &dw8250_starfive_jh7100_data },
    { /* sentinel */ }
};
RT_OF_DECLARE_DEVICE(dw8250_of_match, dw8250_probe);
