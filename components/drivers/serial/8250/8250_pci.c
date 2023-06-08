/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-09     GuEe-GUI     first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include <cpuport.h>
#include <ioremap.h>

#include <pci.h>
#include <serial8250.h>
#include <serial_internal.h>

#define IO_PORT_BAR     0

#define DW_UART_USR     0x1f    /* UART Status Register */

enum
{
    PCI_SERIAL  = 0,
    PCI_SERIAL2 = 2,
    PCI_SERIAL4 = 4,
};

enum
{
    SERIAL_8250 = 0,
    SERIAL_16450,
    SERIAL_16550,
    SERIAL_16650,
    SERIAL_16750,
    SERIAL_16850,
    SERIAL_16950,
};

struct pci_serial
{
    struct serial8250 parent;
    struct rt_spinlock spinlock;

    struct pci_device *dev;

    rt_uint8_t type;
    rt_uint8_t compat;
};

#define raw_to_pci_serial(raw) rt_container_of(raw, struct pci_serial, parent)

static rt_err_t pci_serial_isr(struct serial8250 *serial, int irq)
{
    unsigned int iir;
    void *base = serial->base;

    iir = HWREG8(base) = UART_IIR;

    if (!(iir & UART_IIR_NO_INT))
    {
        rt_hw_serial_isr(&serial->parent, RT_SERIAL_EVENT_RX_IND);
    }

    if ((iir & UART_IIR_BUSY) == UART_IIR_BUSY)
    {
        /* Clear the USR */
        HWREG8(base) = DW_UART_USR;
    }

    return RT_EOK;
}

static void pci_serial_8250_remove(struct serial8250 *serial)
{
    struct pci_serial *pci_serial = raw_to_pci_serial(serial);

    pci_serial->dev->dev = RT_NULL;
    rt_free(pci_serial);
}

static rt_err_t pci_serial_probe(struct pci_device *dev, const struct pci_device_id *id)
{
    rt_err_t ret = RT_EOK;
    struct serial8250 *serial = RT_NULL;
    struct pci_serial *pci_serial = serial8250_alloc(pci_serial);

    if (pci_serial)
    {
        struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

        serial = &pci_serial->parent;
        serial->size = dev->resource[IO_PORT_BAR].size;
        serial->base = rt_ioremap((void *)dev->resource[IO_PORT_BAR].base, serial->size);

        if (serial->base)
        {
            serial->irq = dev->irq;

            serial->parent.ops = &serial8250_uart_ops;
            serial->parent.config = config;
            if (dev->bus->cur_bus_speed == PCI_SPEED_UNKNOWN)
            {
                serial->freq = 1843200;
            }
            else
            {
                /* TODO */
            }
            serial->handle_irq = &pci_serial_isr;
            serial->iotype = PORT_MMIO;
            serial->remove = &pci_serial_8250_remove;
            serial->data = pci_serial;

            pci_serial->dev = dev;
            pci_serial->type = (rt_ubase_t)id->data;
            rt_spin_lock_init(&pci_serial->spinlock);
            dev->dev = (typeof(dev->dev))&pci_serial->parent;
            pci_read_config_byte(dev, PCI_CLASS_PROG, &pci_serial->compat);

            ret = serial8250_setup(serial);

            if (!ret)
            {
                pci_intx(pci_serial->dev, RT_TRUE);
            }
        }
        else
        {
            rt_free(pci_serial);
            ret = -RT_EIO;
        }
    }
    else
    {
        ret = -RT_ENOMEM;
    }

    return ret;
}

static void pci_serial_remove(struct pci_device *dev)
{
    pci_serial_8250_remove((struct serial8250 *)dev->dev);
}

static struct pci_device_id pci_serial_pci_ids[] =
{
    { PCI_DEVICE(PCI_VENDOR_ID_REDHAT, 0x0002), .data = (void *)PCI_SERIAL, },
    { PCI_DEVICE(PCI_VENDOR_ID_REDHAT, 0x0003), .data = (void *)PCI_SERIAL2, },
    { PCI_DEVICE(PCI_VENDOR_ID_REDHAT, 0x0004), .data = (void *)PCI_SERIAL4, },
    { /* sentinel */ }
};

static struct pci_driver pci_serial_drv =
{
    .name = "pci-serial",
    .id_table = pci_serial_pci_ids,
    .probe = pci_serial_probe,
    .remove = pci_serial_remove,
};
PCI_DECLARE(pci_serial, pci_serial_drv);
