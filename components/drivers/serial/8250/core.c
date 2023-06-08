/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-16     GuEe-GUI     first version
 */

#include <ioremap.h>

#include <rt_irqchip.h>
#include <serial8250.h>
#include <serial_internal.h>

rt_err_t serial8250_config(struct serial8250 *serial, const char *options)
{
    rt_err_t ret = -RT_EINVAL;

    if (serial)
    {
        char *arg;
        rt_bool_t has_iotype = RT_FALSE;

        /*
         *  uart8250,io,<addr>[,options]
         *  uart8250,mmio,<addr>[,options]
         *  uart8250,mmio16,<addr>[,options]
         *  uart8250,mmio32,<addr>[,options]
         *  uart8250,mmio32be,<addr>[,options]
         *  uart8250,0x<addr>[,options]
         */
        serial_for_each_args(arg, options)
        {
            if (!rt_strcmp(arg, "uart8250"))
            {
                ret = RT_EOK;
                continue;
            }
            /* user call error */
            if (ret)
            {
                break;
            }
            if (!rt_strncmp(arg, "0x", 2))
            {
                serial->base = serial_base_from_args(arg);
                continue;
            }
            if (!has_iotype)
            {
                const struct
                {
                    char *param;
                    int type;
                } iotype_table[] =
                {
                    { "io", PORT_IO },
                    { "mmio", PORT_MMIO },
                    { "mmio16", PORT_MMIO16 },
                    { "mmio32", PORT_MMIO32 },
                    { "mmio32be", PORT_MMIO32BE },
                };

                serial->iotype = PORT_MMIO32;

                for (int i = 0; i < rt_array_size(iotype_table); ++i)
                {
                    if (!rt_strcmp(arg, iotype_table[i].param))
                    {
                        serial->iotype = iotype_table[i].type;
                        break;
                    }
                }

                has_iotype = RT_TRUE;
                continue;
            }

            serial->parent.config = serial_cfg_from_args(arg);
            break;
        }

        if (!serial->size)
        {
            serial->size = 0x1000;
        }
    }

    return ret;
}

static void serial8250_isr(int irqno, void *param)
{
    struct serial8250 *serial = (struct serial8250 *)param;

    if (serial->handle_irq)
    {
        serial->handle_irq(serial, irqno);
    }
    else
    {
        rt_hw_serial_isr(&serial->parent, RT_SERIAL_EVENT_RX_IND);
    }
}

rt_err_t serial8250_setup(struct serial8250 *serial)
{
    rt_err_t ret = RT_EOK;
    char dev_name[RT_NAME_MAX];

    if (serial)
    {
        rt_hw_spin_lock_init(&serial->spinlock);

        serial->serial_in = serial->serial_in ? serial->serial_in : &serial8250_in;
        serial->serial_out = serial->serial_out ? serial->serial_out : &serial8250_out;

        rt_sprintf(dev_name, "uart%d", serial_next_id());

        rt_hw_serial_register(&serial->parent, dev_name, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, serial->data);
        rt_hw_interrupt_install(serial->irq, serial8250_isr, serial, dev_name);
    }
    else
    {
        ret = -RT_EINVAL;
    }

    return ret;
}

rt_uint32_t serial8250_in(struct serial8250 *serial, int offset)
{
    rt_uint32_t ret = 0;
    offset <<= serial->regshift;

    switch (serial->iotype)
    {
    case PORT_MMIO:
        ret = HWREG8(serial->base + offset);
        break;
    case PORT_MMIO16:
        ret = HWREG16(serial->base + offset);
        break;
    case PORT_MMIO32:
        ret = HWREG32(serial->base + offset);
        break;
    case PORT_MMIO32BE:
        ret = rt_cpu_to_be32(HWREG32(serial->base + offset));
        break;
#ifdef ARCH_SUPPORT_PIO
    case PORT_IO:
        ret = inb(serial->base + offset, value);
        break;
#endif
    default:
        break;
    }

    return ret;
}

void serial8250_out(struct serial8250 *serial, int offset, int value)
{
    offset <<= serial->regshift;

    switch (serial->iotype)
    {
    case PORT_MMIO:
        HWREG8(serial->base + offset) = value;
        break;
    case PORT_MMIO16:
        HWREG16(serial->base + offset) = value;
        break;
    case PORT_MMIO32:
        HWREG32(serial->base + offset) = value;
        break;
    case PORT_MMIO32BE:
        HWREG32(serial->base + offset) = rt_cpu_to_be32(value);
        break;
#ifdef ARCH_SUPPORT_PIO
    case PORT_IO:
        outb(serial->base + offset, value);
        break;
#endif
    default:
        break;
    }
}

rt_err_t serial8250_uart_configure(struct rt_serial_device *raw_serial, struct serial_configure *cfg)
{
    rt_err_t ret = RT_EOK;
    struct serial8250 *serial = raw_to_serial8250(raw_serial);

    serial->serial_out(serial, UART_IER, !UART_IER_RDI);

    /* Enable FIFO, Clear FIFO*/
    serial->serial_out(serial, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);

    /* Disable interrupt */
    serial->serial_out(serial, UART_IER, 0);

    /* DTR + RTS */
    serial->serial_out(serial, UART_MCR, 0x3);
    if (serial->freq)
    {
        unsigned char lcr = serial8250_in(serial, UART_LCR);
        rt_uint32_t wlen = cfg->data_bits + (UART_LCR_WLEN5 - DATA_BITS_5);
    #if 0
        rt_uint32_t divisor = serial->freq / 16 / cfg->baud_rate;

        /* Enable access DLL & DLH */
        serial->serial_out(serial, UART_LCR, lcr | UART_LCR_DLAB);
        serial->serial_out(serial, UART_DLL, (divisor & 0xff));
        serial->serial_out(serial, UART_DLM, (divisor >> 8) & 0xff);
        /* Clear DLAB bit */
        serial->serial_out(serial, UART_LCR, lcr & (~UART_LCR_DLAB));
    #endif

        serial->serial_out(serial, UART_LCR, (lcr & (~wlen)) | wlen);
        serial->serial_out(serial, UART_LCR, lcr & (~UART_LCR_STOP));
        serial->serial_out(serial, UART_LCR, lcr & (~UART_LCR_PARITY));
    }

    serial->serial_out(serial, UART_IER, UART_IER_RDI);

    return ret;
}

rt_err_t serial8250_uart_control(struct rt_serial_device *raw_serial, int cmd, void *arg)
{
    rt_err_t ret = RT_EOK;
    struct serial8250 *serial = raw_to_serial8250(raw_serial);

    switch (cmd)
    {
        case RT_DEVICE_CTRL_CLR_INT:
            /* disable rx irq */
            serial->serial_out(serial, UART_IER, !UART_IER_RDI);
            rt_hw_interrupt_mask(serial->irq);
            break;

        case RT_DEVICE_CTRL_SET_INT:
            /* enable rx irq */
            serial->serial_out(serial, UART_IER, UART_IER_RDI);
            rt_hw_interrupt_umask(serial->irq);
            break;

        case RT_DEVICE_CTRL_SHUTDOWN:
            rt_iounmap((void *)serial->base);
            rt_hw_interrupt_mask(serial->irq);
            rt_hw_interrupt_uninstall(serial->irq, serial8250_isr, serial);

            rt_device_unregister(&serial->parent.parent);

            if (serial->remove)
            {
                serial->remove(serial);
            }
            break;
    }

    return ret;
}

int serial8250_uart_putc(struct rt_serial_device *raw_serial, char c)
{
    struct serial8250 *serial = raw_to_serial8250(raw_serial);

    while (!(serial->serial_in(serial, UART_LSR) & 0x20))
    {
    }

    serial->serial_out(serial, UART_TX, c);

    return 1;
}

int serial8250_uart_getc(struct rt_serial_device *raw_serial)
{
    int ch = -1;
    struct serial8250 *serial = raw_to_serial8250(raw_serial);

    if ((serial->serial_in(serial, UART_LSR) & 0x1))
    {
        ch = serial->serial_in(serial, UART_RX) & 0xff;
    }

    return ch;
}

const struct rt_uart_ops serial8250_uart_ops =
{
    serial8250_uart_configure,
    serial8250_uart_control,
    serial8250_uart_putc,
    serial8250_uart_getc,
};
