/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-24     GuEe-GUI     first version
 */

#include <rthw.h>
#include <rtthread.h>

#define DBG_TAG "pci.ofw"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <drivers/pci.h>
#include <drivers/ofw.h>
#include <drivers/ofw_io.h>
#include <drivers/ofw_irq.h>
#include <drivers/ofw_fdt.h>

static rt_err_t pci_ofw_irq_parse(struct rt_pci_device *pdev, struct rt_ofw_cell_args *out_irq)
{
    rt_err_t err = RT_EOK;
    rt_uint8_t pin;
    fdt32_t map_addr[4];
    struct rt_pci_device *p2pdev;
    struct rt_ofw_node *dev_np, *p2pnode = RT_NULL;

    /* Parse device tree if dev have a device node */
    dev_np = pdev->parent.ofw_node;

    if (dev_np)
    {
        err = rt_ofw_parse_irq_cells(dev_np, 0, out_irq);

        if (err)
        {
            return err;
        }
    }

    /* Assume #interrupt-cells is 1 */
    if ((err = rt_pci_read_config_u8(pdev, PCIR_INTPIN, &pin)))
    {
        goto _err;
    }

    /* No pin, exit with no error message. */
    if (pin == 0)
    {
        return -RT_ENOSYS;
    }

    /* Try local interrupt-map in the device node */
    if (rt_ofw_prop_read_raw(dev_np, "interrupt-map", RT_NULL))
    {
        pin = rt_pci_irq_intx(pdev, pin);
        p2pnode = dev_np;
    }

    /* Walk up the PCI tree */
    while (!p2pnode)
    {
        p2pdev = pdev->bus->self;

        /* Is the root bus -> host bridge */
        if (rt_pci_is_root_bus(pdev->bus))
        {
            struct rt_pci_host_bridge *host_bridge = pdev->bus->host_bridge;

            p2pnode = host_bridge->parent.ofw_node;

            if (!p2pnode)
            {
                err = -RT_EINVAL;

                goto _err;
            }
        }
        else
        {
            /* Is P2P bridge */
            p2pnode = p2pdev->parent.ofw_node;
        }

        if (p2pnode)
        {
            break;
        }

        /* Try get INTx in P2P */
        pin = rt_pci_irq_intx(pdev, pin);
        pdev = p2pdev;
    }

    /* For more format detail, please read `components/drivers/ofw/irq.c:ofw_parse_irq_map` */
    out_irq->data = map_addr;
    out_irq->args_count = 2;
    out_irq->args[0] = 3;
    out_irq->args[1] = 1;

    /* In addr cells */
    map_addr[0] = cpu_to_fdt32((pdev->bus->number << 16) | (pdev->devfn << 8));
    map_addr[1] = cpu_to_fdt32(0);
    map_addr[2] = cpu_to_fdt32(0);
    /* In pin cells */
    map_addr[3] = cpu_to_fdt32(pin);

    err = rt_ofw_parse_irq_map(p2pnode, out_irq);

_err:
    if (err == -RT_EEMPTY)
    {
        LOG_W("PCI-Device<%s> no interrupt-map found, INTx interrupts not available",
                rt_dm_dev_get_name(&pdev->parent));
        LOG_W("PCI-Device<%s> possibly some PCI slots don't have level triggered interrupts capability",
                rt_dm_dev_get_name(&pdev->parent));
    }
    else if (err && err != -RT_ENOSYS)
    {
        LOG_E("PCI-Device<%s> irq parse failed with err = %s",
                rt_dm_dev_get_name(&pdev->parent), rt_strerror(err));
    }

    return err;
}

int rt_pci_ofw_irq_parse_and_map(struct rt_pci_device *pdev, rt_uint8_t slot, rt_uint8_t pin)
{
    int irq = -1;
    rt_err_t status;
    struct rt_ofw_cell_args irq_args;

    if (!pdev)
    {
        goto _end;
    }

    status = pci_ofw_irq_parse(pdev, &irq_args);

    if (status)
    {
        goto _end;
    }

    irq = rt_ofw_map_irq(&irq_args);

_end:
    return irq;
}

rt_err_t rt_pci_ofw_parse_ranges(struct rt_ofw_node *dev_np, struct rt_pci_host_bridge *host_bridge)
{
    const fdt32_t *cell;
    rt_ssize_t total_cells;
    int groups, space_code;
    rt_uint32_t phy_addr[3];
    rt_uint64_t cpu_addr, phy_addr_size;
    int phy_addr_cells = -1, phy_size_cells = -1, cpu_addr_cells;

    if (!dev_np || !host_bridge)
    {
        return -RT_EINVAL;
    }

    cpu_addr_cells = rt_ofw_io_addr_cells(dev_np);
    rt_ofw_prop_read_s32(dev_np, "#address-cells", &phy_addr_cells);
    rt_ofw_prop_read_s32(dev_np, "#size-cells", &phy_size_cells);

    if (phy_addr_cells != 3 || phy_size_cells < 1 || cpu_addr_cells < 1)
    {
        return -RT_EINVAL;
    }

    cell = rt_ofw_prop_read_raw(dev_np, "ranges", &total_cells);

    if (!cell)
    {
        return -RT_EINVAL;
    }

    groups = total_cells / sizeof(*cell) / (phy_addr_cells + phy_size_cells + cpu_addr_cells);
    host_bridge->bus_regions = rt_malloc(groups * sizeof(struct rt_pci_bus_region));

    if (!host_bridge->bus_regions)
    {
        return -RT_ENOMEM;
    }

    host_bridge->bus_regions_nr = 0;

    for (int i = 0; i < groups; ++i)
    {
        /*
         * ranges:
         *  phys.hi  cell: npt000ss bbbbbbbb dddddfff rrrrrrrr
         *  phys.low cell: llllllll llllllll llllllll llllllll
         *  phys.mid cell: hhhhhhhh hhhhhhhh hhhhhhhh hhhhhhhh
         *
         *  n: relocatable region flag (doesn't play a role here)
         *  p: prefetchable (cacheable) region flag
         *  t: aliased address flag (doesn't play a role here)
         *  ss: space code
         *      00: configuration space
         *      01: I/O space
         *      10: 32 bit memory space
         *      11: 64 bit memory space
         *  bbbbbbbb: The PCI bus number
         *  ddddd: The device number
         *  fff: The function number. Used for multifunction PCI devices.
         *  rrrrrrrr: Register number; used for configuration cycles.
         */

        for (int j = 0; j < phy_addr_cells; ++j)
        {
            phy_addr[j] = rt_fdt_read_number(cell++, 1);
        }

        space_code = (phy_addr[0] >> 24) & 0x3;

        cpu_addr = rt_fdt_read_number(cell, cpu_addr_cells);
        cell += cpu_addr_cells;
        phy_addr_size = rt_fdt_read_number(cell, phy_size_cells);
        cell += phy_size_cells;

        host_bridge->bus_regions[i].phy_addr = ((rt_uint64_t)phy_addr[1] << 32) | phy_addr[2];
        host_bridge->bus_regions[i].cpu_addr = cpu_addr;
        host_bridge->bus_regions[i].size = phy_addr_size;

        host_bridge->bus_regions[i].bus_start = host_bridge->bus_regions[i].phy_addr;

        if (space_code & 2)
        {
            host_bridge->bus_regions[i].flags = phy_addr[0] & (1U << 30) ?
                    PCI_BUS_REGION_F_PREFETCH : PCI_BUS_REGION_F_MEM;
        }
        else if (space_code & 1)
        {
            host_bridge->bus_regions[i].flags = PCI_BUS_REGION_F_IO;
        }
        else
        {
            continue;
        }

        ++host_bridge->bus_regions_nr;
    }

    return rt_pci_region_setup(host_bridge);
}

rt_err_t rt_pci_ofw_host_bridge_init(struct rt_ofw_node *dev_np, struct rt_pci_host_bridge *host_bridge)
{
    rt_err_t err;
    const char *propname;

    if (!dev_np || !host_bridge)
    {
        return -RT_EINVAL;
    }

    host_bridge->irq_slot = rt_pci_irq_slot;
    host_bridge->irq_map = rt_pci_ofw_irq_parse_and_map;

    if (rt_ofw_prop_read_u32_array_index(dev_np, "bus-range", 0, 2, host_bridge->bus_range) < 0)
    {
        return -RT_EIO;
    }

    propname = rt_ofw_get_prop_fuzzy_name(dev_np, ",pci-domain$");
    rt_ofw_prop_read_u32(dev_np, propname, &host_bridge->domain);

    err = rt_pci_ofw_parse_ranges(dev_np, host_bridge);

    return err;
}

rt_err_t rt_pci_ofw_bus_init(struct rt_pci_bus *bus)
{
    rt_err_t err = RT_EOK;

    return err;
}

rt_err_t rt_pci_ofw_device_init(struct rt_pci_device *pdev)
{
    struct rt_ofw_node *np = RT_NULL;

    if (!pdev)
    {
        return -RT_EINVAL;
    }

    if (pdev->bus->parent)
    {
        np = pdev->bus->self->parent.ofw_node;
    }

    if (!np)
    {
        return RT_EOK;
    }

    pdev->parent.ofw_node = np;

    return RT_EOK;
}
