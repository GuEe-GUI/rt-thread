/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     the first version
 */

#ifndef __VIRTIO_IOMMU_H__
#define __VIRTIO_IOMMU_H__

#include <drivers/core/dm.h>

#define VIRTIO_IOMMU_F_INPUT_RANGE      0
#define VIRTIO_IOMMU_F_DOMAIN_RANGE     1
#define VIRTIO_IOMMU_F_MAP_UNMAP        2
#define VIRTIO_IOMMU_F_BYPASS           3
#define VIRTIO_IOMMU_F_PROBE            4
#define VIRTIO_IOMMU_F_MMIO             5
#define VIRTIO_IOMMU_F_BYPASS_CONFIG    6

struct virtio_iommu_range_64
{
    rt_le64_t start;
    rt_le64_t end;
};

struct virtio_iommu_range_32
{
    rt_le32_t start;
    rt_le32_t end;
};

struct virtio_iommu_config
{
    /* Supported page sizes */
    rt_le64_t page_size_mask;
    /* Supported IOVA range */
    struct virtio_iommu_range_64 input_range;
    /* Max domain ID size */
    struct virtio_iommu_range_32 domain_range;
    /* Probe buffer size */
    rt_le32_t probe_size;
    rt_uint8_t bypass;
    rt_uint8_t reserved[3];
};

/* Request types */
#define VIRTIO_IOMMU_T_ATTACH           0x01
#define VIRTIO_IOMMU_T_DETACH           0x02
#define VIRTIO_IOMMU_T_MAP              0x03
#define VIRTIO_IOMMU_T_UNMAP            0x04
#define VIRTIO_IOMMU_T_PROBE            0x05

/* Status types */
#define VIRTIO_IOMMU_S_OK               0x00
#define VIRTIO_IOMMU_S_IOERR            0x01
#define VIRTIO_IOMMU_S_UNSUPP           0x02
#define VIRTIO_IOMMU_S_DEVERR           0x03
#define VIRTIO_IOMMU_S_INVAL            0x04
#define VIRTIO_IOMMU_S_RANGE            0x05
#define VIRTIO_IOMMU_S_NOENT            0x06
#define VIRTIO_IOMMU_S_FAULT            0x07
#define VIRTIO_IOMMU_S_NOMEM            0x08

struct virtio_iommu_req_head
{
    rt_uint8_t type;
    rt_uint8_t reserved[3];
};

struct virtio_iommu_req_tail
{
    rt_uint8_t status;
    rt_uint8_t reserved[3];
};

#define VIRTIO_IOMMU_ATTACH_F_BYPASS    (1 << 0)

struct virtio_iommu_req_attach
{
    struct virtio_iommu_req_head head;
    rt_le32_t domain;
    rt_le32_t endpoint;
    rt_le32_t flags;
    rt_uint8_t reserved[4];
    struct virtio_iommu_req_tail tail;
};

struct virtio_iommu_req_detach
{
    struct virtio_iommu_req_head head;
    rt_le32_t domain;
    rt_le32_t endpoint;
    rt_uint8_t reserved[8];
    struct virtio_iommu_req_tail tail;
};

#define VIRTIO_IOMMU_MAP_F_READ         (1 << 0)
#define VIRTIO_IOMMU_MAP_F_WRITE        (1 << 1)
#define VIRTIO_IOMMU_MAP_F_MMIO         (1 << 2)
#define VIRTIO_IOMMU_MAP_F_MASK         (VIRTIO_IOMMU_MAP_F_READ | VIRTIO_IOMMU_MAP_F_WRITE | VIRTIO_IOMMU_MAP_F_MMIO)

struct virtio_iommu_req_map
{
    struct virtio_iommu_req_head head;
    rt_le32_t domain;
    rt_le64_t virt_start;
    rt_le64_t virt_end;
    rt_le64_t phys_start;
    rt_le32_t flags;
    struct virtio_iommu_req_tail tail;
};

struct virtio_iommu_req_unmap
{
    struct virtio_iommu_req_head head;
    rt_le32_t domain;
    rt_le64_t virt_start;
    rt_le64_t virt_end;
    rt_uint8_t reserved[4];
    struct virtio_iommu_req_tail tail;
};

#define VIRTIO_IOMMU_PROBE_T_NONE           0
#define VIRTIO_IOMMU_PROBE_T_RESV_MEM       1
#define VIRTIO_IOMMU_PROBE_T_MASK           0xfff

struct virtio_iommu_probe_property
{
    rt_le16_t type;
    rt_le16_t length;
};

#define VIRTIO_IOMMU_RESV_MEM_T_RESERVED    0
#define VIRTIO_IOMMU_RESV_MEM_T_MSI         1

struct virtio_iommu_probe_resv_mem
{
    struct virtio_iommu_probe_property head;
    rt_uint8_t subtype;
    rt_uint8_t reserved[3];
    rt_le64_t start;
    rt_le64_t end;
};

struct virtio_iommu_req_probe
{
    struct virtio_iommu_req_head head;
    rt_le32_t endpoint;
    rt_uint8_t reserved[64];

    rt_uint8_t properties[];
    /*
     * Tail follows the variable-length properties array. No padding,
     * property lengths are all aligned on 8 bytes.
     */
};

/* Fault types */
#define VIRTIO_IOMMU_FAULT_R_UNKNOWN        0
#define VIRTIO_IOMMU_FAULT_R_DOMAIN         1
#define VIRTIO_IOMMU_FAULT_R_MAPPING        2

#define VIRTIO_IOMMU_FAULT_F_READ           (1 << 0)
#define VIRTIO_IOMMU_FAULT_F_WRITE          (1 << 1)
#define VIRTIO_IOMMU_FAULT_F_EXEC           (1 << 2)
#define VIRTIO_IOMMU_FAULT_F_ADDRESS        (1 << 8)

struct virtio_iommu_fault
{
    rt_uint8_t reason;
    rt_uint8_t reserved[3];
    rt_le32_t flags;
    rt_le32_t endpoint;
    rt_uint8_t reserved2[4];
    rt_le64_t address;
};

#endif /* __VIRTIO_IOMMU_H__ */
