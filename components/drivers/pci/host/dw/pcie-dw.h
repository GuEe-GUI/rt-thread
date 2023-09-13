/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-09-23     GuEe-GUI     first version
 */

#ifndef __PCIE_DESIGNWARE_H__
#define __PCIE_DESIGNWARE_H__

#include <rtthread.h>
#include <rtdevice.h>

struct dw_pcie_ops;
struct dw_pcie_host_ops;

struct dw_pcie_rp
{
    rt_bool_t has_msi_ctrl:1;
    rt_bool_t cfg0_io_shared:1;

    rt_uint64_t cfg0_base;
    void *va_cfg0_base;
    rt_uint32_t cfg0_size;

    rt_size_t io_base;
    rt_ubase_t io_bus_addr;
    rt_uint32_t io_size;

    int irq;
    const struct dw_pcie_host_ops *ops;
};

struct dw_pcie_host_ops
{
    rt_err_t (*host_init)(struct dw_pcie_rp *pp);
    void (*host_deinit)(struct dw_pcie_rp *pp);
    rt_err_t (*msi_host_init)(struct dw_pcie_rp *pp);
};

struct dw_pcie_ep
{
    const struct dw_pcie_ep_ops *ops;

    rt_ubase_t phys_base;
    rt_size_t addr_size;
    rt_size_t page_size;
    rt_uint8_t bar_to_atu[PCI_STD_NUM_BARS];
    rt_ubase_t *outbound_addr;
    rt_ubase_t *ib_window_map;
    rt_ubase_t *ob_window_map;
    void *msi_mem;
    rt_ubase_t msi_mem_phys;
};

struct dw_pcie
{
    struct rt_device *dev;

    void *dbi_base;
    void *dbi_base2;
    void *atu_base;

    rt_size_t atu_size;
    rt_uint32_t num_ib_windows;
    rt_uint32_t num_ob_windows;
    rt_uint32_t region_align;
    rt_uint64_t region_limit;

    struct dw_pcie_rp pp;
    struct dw_pcie_ep ep;
    const struct dw_pcie_ops *ops;

    void *priv;
};

struct dw_pcie_ops
{
    rt_uint64_t (*cpu_addr_fixup)(struct dw_pcie *pcie, rt_uint64_t cpu_addr);
    rt_uint32_t (*read_dbi)(struct dw_pcie *pcie, void *base, rt_uint32_t reg, rt_size_t size);
    void        (*write_dbi)(struct dw_pcie *pcie, void *base, rt_uint32_t reg, rt_size_t size, rt_uint32_t val);
    void        (*write_dbi2)(struct dw_pcie *pcie, void *base, rt_uint32_t reg, rt_size_t size, rt_uint32_t val);
    rt_bool_t   (*link_up)(struct dw_pcie *pcie);
    rt_err_t    (*start_link)(struct dw_pcie *pcie);
    void        (*stop_link)(struct dw_pcie *pcie);
};

#define to_dw_pcie_from_pp(port)        rt_container_of((port), struct dw_pcie, pp)
#define to_dw_pcie_from_ep(endpoint)    rt_container_of((endpoint), struct dw_pcie, ep)

rt_err_t dw_pcie_host_init(struct dw_pcie_rp *pp);

#endif /* __PCIE_DESIGNWARE_H__ */
