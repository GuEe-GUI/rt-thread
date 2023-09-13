/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-07     GuEe-GUI     first version
 */

#ifndef __VIRTIO_PCI_H__
#define __VIRTIO_PCI_H__

#include <drivers/pci.h>

#define VIRTIO_PCI_CAP_COMMON_CFG           1   /* Common configuration */
#define VIRTIO_PCI_CAP_NOTIFY_CFG           2   /* Notifications */
#define VIRTIO_PCI_CAP_ISR_CFG              3   /* ISR Status */
#define VIRTIO_PCI_CAP_DEVICE_CFG           4   /* Device specific configuration */
#define VIRTIO_PCI_CAP_PCI_CFG              5   /* PCI configuration access */
#define VIRTIO_PCI_CAP_SHARED_MEMORY_CFG    8   /* Shared memory region */
#define VIRTIO_PCI_CAP_VENDOR_CFG           9   /* Vendor-specific data */

struct virtio_pci_cap
{
    rt_uint8_t cap_vndr;    /* Generic PCI field: PCI_CAP_ID_VNDR */
    rt_uint8_t cap_next;    /* Generic PCI field: next ptr. */
    rt_uint8_t cap_len;     /* Generic PCI field: capability length */
    rt_uint8_t cfg_type;    /* Identifies the structure. */
    rt_uint8_t bar;         /* Where to find it. */
    rt_uint8_t id;          /* Multiple capabilities of the same type */
    rt_uint8_t padding[2];  /* Pad to full dword. */
    rt_uint32_t offset;     /* Offset within bar. */
    rt_uint32_t length;     /* Length of the structure, in bytes. */
};

struct virtio_pci_cap64
{
    struct virtio_pci_cap cap;
    rt_uint32_t offset_hi;
    rt_uint32_t length_hi;
};

struct virtio_pci_common_cfg
{
    /* About the whole device. */
    rt_uint32_t device_feature_select;  /* Read-write */
    rt_uint32_t device_feature;         /* Read-only for driver */
    rt_uint32_t driver_feature_select;  /* Read-write */
    rt_uint32_t driver_feature;         /* Read-write */
    rt_uint16_t config_msix_vector;     /* Read-write */
    rt_uint16_t num_queues;             /* Read-only for driver */
    rt_uint8_t device_status;           /* Read-write */
    rt_uint8_t config_generation;       /* Read-only for driver */

    /* About a specific virtqueue. */
    rt_uint16_t queue_select;           /* Read-write */
    rt_uint16_t queue_size;             /* Read-write */
    rt_uint16_t queue_msix_vector;      /* Read-write */
    rt_uint16_t queue_enable;           /* Read-write */
    rt_uint16_t queue_notify_off;       /* Read-only for driver */
    rt_uint64_t queue_desc;             /* Read-write */
    rt_uint64_t queue_driver;           /* Read-write */
    rt_uint64_t queue_device;           /* Read-write */
    rt_uint16_t queue_notify_data;      /* Read-only for driver */
    rt_uint16_t queue_reset;            /* Read-write */
};

struct virtio_pci_notify_cap
{
    struct virtio_pci_cap cap;
    rt_uint32_t notify_off_multiplier;  /* Multiplier for queue_notify_off. */
};

struct virtio_pci_vndr_data
{
    rt_uint8_t cap_vndr;                /* Generic PCI field: PCI_CAP_ID_VNDR */
    rt_uint8_t cap_next;                /* Generic PCI field: next ptr. */
    rt_uint8_t cap_len;                 /* Generic PCI field: capability length */
    rt_uint8_t cfg_type;                /* Identifies the structure. */
    rt_uint16_t vendor_id;              /* Identifies the vendor-specific format. */
    /* For Vendor Definition */
    /* Pads structure to a multiple of 4 bytes */
    /* Reads must not have side effects */
};

struct virtio_pci_cfg_cap
{
    struct virtio_pci_cap cap;
    rt_uint8_t pci_cfg_data[4];         /* Data for BAR access. */
};


#define VIRTIO_PCI_DEVICE_FEATURES  0x00 /* <RO> Flags representing features the device supports */
#define VIRTIO_PCI_DRIVER_FEATURES  0x04 /* <WO> Device features understood and activated by the driver */
#define VIRTIO_PCI_QUEUE_PFN        0x08 /* <RW> Guest physical page number of the virtual queue */
#define VIRTIO_PCI_QUEUE_NUM        0x0c /* <WO> Virtual queue size */
#define VIRTIO_PCI_QUEUE_SEL        0x0e /* <WO> Virtual queue index */
#define VIRTIO_PCI_QUEUE_NOTIFY     0x10 /* <WO> Queue notifier */
#define VIRTIO_PCI_STATUS           0x12 /* <RW> Device status */
#define VIRTIO_PCI_INTERRUPT_STATUS 0x13 /* <RO> Interrupt status */

#endif /* __VIRTIO_PCI_H__ */
