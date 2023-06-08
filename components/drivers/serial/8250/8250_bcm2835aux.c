/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-16     GuEe-GUI     first version
 */

#include <rtthread.h>

#include <ioremap.h>

#include <rtof.h>
#include <rt_irqchip.h>
#include <serial8250.h>
#include <serial_internal.h>

#define BCM2835_AUX_UART_CNTL           8
#define BCM2835_AUX_UART_CNTL_RXEN      0x01 /* Receiver enable */
#define BCM2835_AUX_UART_CNTL_TXEN      0x02 /* Transmitter enable */
#define BCM2835_AUX_UART_CNTL_AUTORTS   0x04 /* RTS set by RX fill level */
#define BCM2835_AUX_UART_CNTL_AUTOCTS   0x08 /* CTS stops transmitter */
#define BCM2835_AUX_UART_CNTL_RTS3      0x00 /* RTS set until 3 chars left */
#define BCM2835_AUX_UART_CNTL_RTS2      0x10 /* RTS set until 2 chars left */
#define BCM2835_AUX_UART_CNTL_RTS1      0x20 /* RTS set until 1 chars left */
#define BCM2835_AUX_UART_CNTL_RTS4      0x30 /* RTS set until 4 chars left */
#define BCM2835_AUX_UART_CNTL_RTSINV    0x40 /* Invert auto RTS polarity */
#define BCM2835_AUX_UART_CNTL_CTSINV    0x80 /* Invert auto CTS polarity */

struct bcm2835aux
{
    struct serial8250 parent;
    rt_uint32_t cntl;
};

static void bcm2835aux_remove(struct serial8250 *serial)
{
    rt_free(rt_container_of(serial, struct bcm2835aux, parent));
}

static rt_err_t bcm2835aux_probe(struct rt_device_node *node, const struct rt_of_device_id *id)
{
    rt_err_t ret = RT_EOK;
    struct serial8250 *serial = RT_NULL;
    struct bcm2835aux *bcm2835aux = serial8250_alloc(bcm2835aux);

    if (bcm2835aux)
    {
        struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

        serial = &bcm2835aux->parent;
        serial->base = rt_of_iomap(node, 0);

        if (serial->base)
        {
            serial->clk = clk_of_get(node, 0);
            clk_enable(serial->clk);
            serial->freq = clk_get_rate(serial->clk);

            if (!serial->freq)
            {
                rt_iounmap(serial->base);
                serial->base = RT_NULL;
            }
        }

        if (serial->base)
        {
            serial->irq = rt_of_irq_get(node, 0);

            serial->parent.ops = &serial8250_uart_ops;
            serial->parent.config = config;
            serial->regshift = 2;
            serial->iotype = PORT_MMIO;
            serial->remove = &bcm2835aux_remove;
            serial->data = bcm2835aux;

            bcm2835aux->cntl = BCM2835_AUX_UART_CNTL_RXEN | BCM2835_AUX_UART_CNTL_TXEN;
            rt_of_data(node) = &serial->parent;

            ret = serial8250_setup(serial);
        }
        else
        {
            rt_free(bcm2835aux);
            ret = -RT_ERROR;
        }
    }
    else
    {
        ret = -RT_ENOMEM;
    }

    return ret;
}

static const struct rt_of_device_id bcm2835aux_of_match[] =
{
    { .type = "ttyS", .compatible = "brcm,bcm2835-aux-uart" },
    { /* sentinel */ }
};
RT_OF_DECLARE_DEVICE(bcm2835aux_of_match, bcm2835aux_probe);
