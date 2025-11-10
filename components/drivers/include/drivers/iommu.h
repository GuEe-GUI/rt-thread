/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     the first version
 */

#ifndef __IOMMU_H__
#define __IOMMU_H__

#include <mm_flag.h>
#include <drivers/ofw.h>
#include <drivers/core/dm.h>

struct rt_iommu_ops;
struct rt_iommu_fault;
struct rt_iommu_group;
struct rt_iommu_domain;

struct rt_iommu_device
{
    struct rt_device *dev;

    const struct rt_iommu_ops *ops;

    rt_size_t page_size;
    rt_size_t max_pasids;
    rt_bitmap_t *pasids_maps;

    rt_list_t group_nodes;
    struct rt_spinlock lock;
};

struct rt_iommu_ops
{
    rt_err_t (*attach)(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain);
    rt_err_t (*detach)(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain);

    rt_err_t (*map)(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain,
            void *iopa, void *iova, rt_size_t size, mm_flag_t flags);
    rt_err_t (*unmap)(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain,
            void *iova, rt_size_t size);
    rt_err_t (*translate)(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain,
            void *iova, void **out_iopa);

    rt_err_t (*iotlb_flush)(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain);
    rt_err_t (*iotlb_sync)(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain,
            void *iova, rt_size_t size);

    int (*ofw_parse)(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain,
            struct rt_ofw_cell_args *iommu_args);
};

struct rt_iommu_page
{
    rt_list_t list;

    void *iova;
    void *iopa;
    rt_size_t size;
    mm_flag_t flags;

    rt_uint32_t ref;
};

struct rt_iommu_page_shadow
{
    rt_list_t list;

    rt_size_t offset;
    rt_size_t size;
    struct rt_iommu_page *page;
};

struct rt_iommu_group
{
    rt_list_t list;
    struct rt_iommu_device *iommu;

#define RT_IOMMU_PASID_INVALID  (-1U)
    rt_uint32_t pasid;  /* Process Address Space ID */

    rt_bool_t mapping;
    rt_list_t page_nodes;
    rt_list_t domain_nodes;

    void *priv;
};

struct rt_iommu_domain
{
    rt_list_t list;
    rt_list_t page_shadow_nodes;

    struct rt_iommu_group *group;
    struct rt_device *dev;

    void (*fault_handle)(struct rt_iommu_domain *domain, struct rt_iommu_fault *fault);

    void *priv;
    void *user_data;    /* Process, ASID ... */
};

/* Requested page permissions */
#define RT_IOMMU_FAULT_PERM_READ                    RT_BIT(0)   /* Read */
#define RT_IOMMU_FAULT_PERM_WRITE                   RT_BIT(1)   /* Write */
#define RT_IOMMU_FAULT_PERM_EXEC                    RT_BIT(2)   /* Exec */
#define RT_IOMMU_FAULT_PERM_PRIV                    RT_BIT(3)   /* Privileged */

struct rt_iommu_fault_unrecoverable
{
#define RT_IOMMU_FAULT_REASON_UNKNOWN               0
#define RT_IOMMU_FAULT_REASON_PASID_FETCH           1   /* Could not access the PASID table (fetch caused external abort) */
#define RT_IOMMU_FAULT_REASON_BAD_PASID_ENTRY       2   /* PASID entry is invalid or has configuration errors */
#define RT_IOMMU_FAULT_REASON_PASID_INVALID         3   /* PASID is out of range or disabled. */
#define RT_IOMMU_FAULT_REASON_WALK_EABT             4   /* An external abort occurred fetching (or updating) a translation table descriptor */
#define RT_IOMMU_FAULT_REASON_PTE_FETCH             5   /* Could not access the page table entry (Bad address), actual translation fault */
#define RT_IOMMU_FAULT_REASON_PERMISSION            6   /* Protection flag check failed */
#define RT_IOMMU_FAULT_REASON_ACCESS                7   /* Access flag check failed */
#define RT_IOMMU_FAULT_REASON_OOR_ADDRESS           8   /* Output address of a translation stage caused Address Size fault */
    rt_uint32_t reason;
#define RT_IOMMU_FAULT_UNRECOV_PASID_VALID          RT_BIT(0)
#define RT_IOMMU_FAULT_UNRECOV_ADDR_VALID           RT_BIT(1)
#define RT_IOMMU_FAULT_UNRECOV_FETCH_ADDR_VALID     RT_BIT(2)
    rt_uint32_t flags;
    rt_uint32_t pasid;
    rt_uint32_t perm;
    rt_uint64_t addr;
    rt_uint64_t fetch_addr;
};

struct rt_iommu_fault_page_request
{
#define RT_IOMMU_FAULT_PAGE_REQUEST_PASID_VALID     RT_BIT(0)
#define RT_IOMMU_FAULT_PAGE_REQUEST_LAST_PAGE       RT_BIT(1)
#define RT_IOMMU_FAULT_PAGE_REQUEST_PRIV_DATA       RT_BIT(2)
#define RT_IOMMU_FAULT_PAGE_RESPONSE_NEEDS_PASID    RT_BIT(3)
    rt_uint32_t flags;
    rt_uint32_t pasid;
    rt_uint32_t grpid;  /* Page Request Group Index */
    rt_uint32_t perm;
    rt_uint64_t addr;
};

struct rt_iommu_fault
{
#define RT_IOMMU_FAULT_UNRECOVERABLE    1
#define RT_IOMMU_FAULT_PAGE_REQUEST     2
    rt_uint32_t type;

    union
    {
        struct rt_iommu_fault_unrecoverable unrecoverable;
        struct rt_iommu_fault_page_request page_request;
    };
};

rt_err_t rt_iommu_device_register(struct rt_iommu_device *iommu);
rt_err_t rt_iommu_device_unregister(struct rt_iommu_device *iommu);

void rt_iommu_device_fault(struct rt_iommu_device *iommu, struct rt_iommu_fault *fault);

struct rt_iommu_group *rt_iommu_device_get_group(struct rt_iommu_device *iommu, rt_uint32_t pasid);
void rt_iommu_device_put_group(struct rt_iommu_group *group);

rt_err_t rt_iommu_domain_map(struct rt_iommu_domain *domain, void *iopa, void *iova, rt_size_t size, mm_flag_t flags);
rt_err_t rt_iommu_domain_unmap(struct rt_iommu_domain *domain, void *iova, rt_size_t size);
rt_err_t rt_iommu_domain_unmap_all(struct rt_iommu_domain *domain);
rt_err_t rt_iommu_domain_translate(struct rt_iommu_domain *domain, void *iova, void **out_iopa);

rt_err_t rt_iommu_domain_iotlb_flush(struct rt_iommu_domain *domain);
rt_err_t rt_iommu_domain_iotlb_sync(struct rt_iommu_domain *domain, void *iova, rt_size_t size);

struct rt_iommu_domain *rt_iommu_domain_get_by_index(struct rt_device *dev, int index);
struct rt_iommu_domain *rt_iommu_domain_get_by_name(struct rt_device *dev, const char *name);
void rt_iommu_domain_put(struct rt_iommu_domain *node);

rt_err_t rt_iommu_attach(struct rt_device *dev);
rt_err_t rt_iommu_detach(struct rt_device *dev);

rt_inline rt_err_t rt_iommu_map(struct rt_device *dev, void *iopa, void *iova, rt_size_t size, mm_flag_t flags)
{
    return rt_iommu_domain_map(dev->iommu_domain, iopa, iova, size, flags);
}

rt_inline rt_err_t rt_iommu_unmap(struct rt_device *dev, void *iova, rt_size_t size)
{
    return rt_iommu_domain_unmap(dev->iommu_domain, iova, size);
}

rt_inline rt_err_t rt_iommu_unmap_all(struct rt_device *dev, void *iova)
{
    return rt_iommu_domain_unmap_all(dev->iommu_domain);
}

rt_inline rt_err_t rt_iommu_translate(struct rt_device *dev, void *iova, void **out_iopa)
{
    return rt_iommu_domain_translate(dev->iommu_domain, iova, out_iopa);
}

rt_inline rt_err_t rt_iommu_iotlb_flush(struct rt_device *dev)
{
    return rt_iommu_domain_iotlb_flush(dev->iommu_domain);
}

rt_inline rt_err_t rt_iommu_iotlb_sync(struct rt_device *dev, void *iova, rt_size_t size)
{
    return rt_iommu_domain_iotlb_sync(dev->iommu_domain, iova, size);
}

#endif /* __IOMMU_H__ */
