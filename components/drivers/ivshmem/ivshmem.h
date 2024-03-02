/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-27     GuEe-GUI     first version
 */

#ifndef __IVSHMEM_H__
#define __IVSHMEM_H__

#include <rtthread.h>
#include <rtdevice.h>

/* Registers */
#define IVSHMEM_INTR_MASK       0
#define IVSHMEM_INTR_STATUS     4
#define IVSHMEM_IV_POSITION     8
#define IVSHMEM_DOORBELL        12

/* BAR */
#define IVSHMEM_REG_BAR         0
#define IVSHMEM_MSIX_BAR        1
#define IVSHMEM_SHARE_MEM_BAR   2

#define IVSHMEM_MAX_VECTORS     255

/* JailHouse support */
#define IVSHMEM_JAILHOUSE_SHMEM_ADDR_LO 0x40
#define IVSHMEM_JAILHOUSE_SHMEM_ADDR_HI 0x44
#define IVSHMEM_JAILHOUSE_SHMEM_SIZE_LO 0x48
#define IVSHMEM_JAILHOUSE_SHMEM_SIZE_HI 0x4c

struct ivshmem_cmd
{
    rt_bool_t is_read;
    rt_uint32_t reg_off;
    rt_uint32_t value;
};

struct ivshmem_device
{
    struct rt_device parent;
    struct rt_pci_device *pdev;

    void *reg;
    int irq;

    void *shmem;
    rt_size_t shmem_size;

    int nvectors;
    struct rt_pci_msix_entry *msix_entries;

    rt_err_t (*handle_irq)(struct ivshmem_device *, int irq);
};

#define to_ivshmem(dev) rt_container_of(dev, struct ivshmem_device, parent)

rt_inline rt_uint32_t ivshmem_doorbell(int peer_id, int vector_index)
{
    return (rt_uint32_t)((peer_id << 16) | (vector_index & IVSHMEM_MAX_VECTORS));
}

rt_err_t ivshmem_install_msix_vectors(struct ivshmem_device *ivdev, int nvectors, const char *name);
rt_err_t ivshmem_install_intx_vector(struct ivshmem_device *ivdev, const char *name);

rt_err_t ivshmem_pci_probe(struct rt_pci_device *pdev, struct ivshmem_device *ivdev);
struct ivshmem_device *ivshmem_pci_remove(struct rt_pci_device *pdev);

#endif /* __IVSHMEM_H__ */
