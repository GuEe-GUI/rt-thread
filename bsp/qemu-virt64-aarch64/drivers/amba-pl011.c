#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <rthw.h>

#include "board.h"
#include "drivers/amba_bus.h"

#ifdef RT_USING_FDT
#include "dtb_node.h"
#endif

struct hw_uart_device
{
    rt_ubase_t hw_base;
    rt_uint32_t irqno;
};

static void rt_hw_uart_isr(int irqno, void *param)
{
    struct rt_serial_device *serial = (struct rt_serial_device *)param;

    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
}

static rt_err_t uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    return RT_EOK;
}

static rt_err_t uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct hw_uart_device *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct hw_uart_device *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        UART_IMSC(uart->hw_base) &= ~UARTIMSC_RXIM;
        break;

    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        UART_IMSC(uart->hw_base) |= UARTIMSC_RXIM;
        rt_hw_interrupt_umask(uart->irqno);
        break;
    }

    return RT_EOK;
}

static int uart_putc(struct rt_serial_device *serial, char c)
{
    struct hw_uart_device *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct hw_uart_device *)serial->parent.user_data;

    while (UART_FR(uart->hw_base) & UARTFR_TXFF)
        ;
    UART_DR(uart->hw_base) = c;

    return 1;
}

static int uart_getc(struct rt_serial_device *serial)
{
    int ch;
    struct hw_uart_device *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct hw_uart_device *)serial->parent.user_data;

    ch = -1;
    if (!(UART_FR(uart->hw_base) & UARTFR_RXFE))
    {
        ch = UART_DR(uart->hw_base) & 0xff;
    }

    return ch;
}

struct rt_serial_device *pl011_device_create(struct rt_device *dev)
{
    struct rt_serial_device *serial;

    serial = rt_malloc(sizeof(struct rt_serial_device));
    if (serial == RT_NULL)
    {
        rt_kprintf("malloc serial failed...\n");
        return serial;
    }
    serial->parent = *dev;
    
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    serial->config = config;

    return serial;
}

static struct rt_uart_ops pl011_uart_ops =
    {
        .device_create = pl011_device_create,
        .configure = uart_configure,
        .control = uart_control,
        .putc = uart_putc,
        .getc = uart_getc,
};

int pl011_probe(struct rt_device *dev)
{
    size_t uart_reg_range = 0;
    void *uart_reg_addr = RT_NULL;

    struct dtb_node *node = (struct dtb_node *)(dev->dtb_node);

    struct hw_uart_device *uart_priv = (struct hw_uart_device *)(rt_calloc(1, sizeof(struct hw_uart_device)));
    if (node)
    {
        uart_reg_addr = (void *)dtb_node_get_addr_size(node, "reg", &uart_reg_range);
        if ((uart_reg_addr) && (uart_reg_range != 0))
        {
            uart_priv->hw_base = (rt_base_t)rt_ioremap(uart_reg_addr, uart_reg_range);
        }

        uart_priv->irqno = dtb_node_irq_get(node, 0) + IRQ_SPI_OFFSET;
        
        struct rt_uart_ops *uart_ops = ((struct rt_uart_driver *)(dev->drv))->ops;
        struct rt_serial_device *serial = uart_ops->device_create(dev);

        const char *uart_name = dev->drv->name;
        dev->priv = serial;
        rt_hw_serial_register(serial, uart_name,
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DEACTIVATE,
                              uart_priv);
        rt_hw_interrupt_install(uart_priv->irqno, rt_hw_uart_isr, serial, uart_name);
        UART_CR(uart_priv->hw_base) = (1 << 0) | (1 << 8) | (1 << 9);
    }

    return RT_EOK;
}

struct rt_device_id pl011_ids[] =
    {
        {.compatible = "arm,pl011"},
        {/* sentinel */}};

struct rt_uart_driver pl011_drv = {
    .parent = {
        .name = RT_CONSOLE_DEVICE_NAME,
        .probe = pl011_probe,
        .ids = pl011_ids,
    },
    .ops = &pl011_uart_ops,
};

AMBA_DRIVER_EXPORT(pl011_drv);
