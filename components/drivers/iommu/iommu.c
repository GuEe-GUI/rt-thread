/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "rtdm.iommu"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

rt_err_t rt_iommu_device_register(struct rt_iommu_device *iommu)
{
    if (!iommu || !iommu->dev || !iommu->ops || !iommu->max_pasids)
    {
        return -RT_EINVAL;
    }

    if (!iommu->page_size)
    {
        iommu->page_size = ARCH_PAGE_SIZE;
    }

    iommu->pasids_maps = rt_calloc(RT_BITMAP_LEN(iommu->max_pasids), sizeof(rt_bitmap_t));

    if (!iommu->pasids_maps)
    {
        return -RT_ENOMEM;
    }

    rt_list_init(&iommu->group_nodes);
    rt_spin_lock_init(&iommu->lock);

    if (iommu->dev->ofw_node)
    {
        rt_dm_dev_bind_fwdata(iommu->dev, RT_NULL, iommu);
    }

    return RT_EOK;
}

rt_err_t rt_iommu_device_unregister(struct rt_iommu_device *iommu)
{
    rt_ubase_t level;
    rt_err_t err = RT_EOK;

    if (!iommu)
    {
        return -RT_EINVAL;
    }

    level = rt_spin_lock_irqsave(&iommu->lock);

    if (rt_list_isempty(&iommu->group_nodes))
    {
        if (iommu->dev->ofw_node)
        {
            rt_dm_dev_unbind_fwdata(iommu->dev, RT_NULL);
        }
    }
    else
    {
        err = -RT_EBUSY;
    }

    rt_spin_unlock_irqrestore(&iommu->lock, level);

    if (!err)
    {
        rt_free(iommu->pasids_maps);
    }

    return err;
}

static void iommu_dump_fault(struct rt_iommu_domain *domain, struct rt_iommu_fault *fault)
{
    LOG_W("%s Fault (no handle)", rt_dm_dev_get_name(domain->dev));

    switch (fault->type)
    {
    case RT_IOMMU_FAULT_UNRECOVERABLE:
        LOG_W("Type: %s", "unrecoverable");
        LOG_W("Reason: %u", fault->unrecoverable.reason);
        LOG_W("Flags: %u", fault->unrecoverable.flags);
        LOG_W("PASID: %u", fault->unrecoverable.pasid);
        LOG_W("Permissions: %u", fault->unrecoverable.perm);
        LOG_W("Address(IOVA): 0x%8x.%8x",
                fault->unrecoverable.addr >> 32,
                fault->unrecoverable.addr & RT_UINT32_MAX);
        LOG_W("Fetch Address: 0x%8x.%8x",
                fault->unrecoverable.fetch_addr >> 32,
                fault->unrecoverable.fetch_addr & RT_UINT32_MAX);
        break;

    case RT_IOMMU_FAULT_PAGE_REQUEST:
        LOG_W("Type: %s", "page request");
        LOG_W("Flags: %u", fault->page_request.flags);
        LOG_W("PASID: %u", fault->page_request.pasid);
        LOG_W("Group ID: %u", fault->page_request.grpid);
        LOG_W("Permissions: %u", fault->page_request.perm);
        LOG_W("Address(IOVA): 0x%8x.%8x",
                fault->page_request.addr >> 32,
                fault->page_request.addr & RT_UINT32_MAX);
        break;
    }
}

void rt_iommu_device_fault(struct rt_iommu_device *iommu, struct rt_iommu_fault *fault)
{
    rt_ubase_t level;
    rt_uint64_t addr;
    rt_uint32_t pasid;
    struct rt_iommu_page *page;
    struct rt_iommu_group *group;
    struct rt_iommu_domain *domain, *domain_next;

    RT_ASSERT(iommu != RT_NULL);
    RT_ASSERT(fault != RT_NULL);

    switch (fault->type)
    {
    case RT_IOMMU_FAULT_UNRECOVERABLE:
        pasid = fault->unrecoverable.pasid;
        addr = fault->unrecoverable.addr;
        break;

    case RT_IOMMU_FAULT_PAGE_REQUEST:
        pasid = fault->page_request.pasid;
        addr = fault->page_request.addr;
        break;

    default:
        pasid = RT_UINT32_MAX;
        addr = RT_UINT64_MAX;
        LOG_E("%s: Unknow fault type = %d", rt_dm_dev_get_name(iommu->dev), fault->type);
        RT_ASSERT(0);
        break;
    }

    level = rt_spin_lock_irqsave(&iommu->lock);

    rt_list_for_each_entry(group, &iommu->group_nodes, list)
    {
        if (group->pasid == pasid)
        {
            goto _found_pasid;
        }
    }

    LOG_W("%s: PASID %d group not found", rt_dm_dev_get_name(iommu->dev), pasid);
    goto _out_lock;

_found_pasid:
    rt_list_for_each_entry(page, &group->page_nodes, list)
    {
        rt_ubase_t iova = (rt_ubase_t)page->iova;

        if (iova <= addr && iova + page->size > addr)
        {
            goto _found_addr;
        }
    }

    LOG_W("%s: IOVA 0x%8x.%8x not found", rt_dm_dev_get_name(iommu->dev),
            addr >> 32, addr & RT_UINT32_MAX);
    goto _out_lock;

_found_addr:
    rt_list_for_each_entry_safe(domain, domain_next, &group->domain_nodes, list)
    {
        struct rt_iommu_page_shadow *page_shadow;

        rt_list_for_each_entry(page_shadow, &domain->page_shadow_nodes, list)
        {
            if (page_shadow->page == page)
            {
                if (domain->fault_handle)
                {
                    /* Domain can be destroyed or map lazy from here */
                    rt_spin_unlock_irqrestore(&iommu->lock, level);

                    domain->fault_handle(domain, fault);

                    level = rt_spin_lock_irqsave(&iommu->lock);
                }
                else
                {
                    iommu_dump_fault(domain, fault);
                }

                break;
            }
        }
    }

_out_lock:
    rt_spin_unlock_irqrestore(&iommu->lock, level);
}

struct rt_iommu_group *rt_iommu_device_get_group(struct rt_iommu_device *iommu, rt_uint32_t pasid)
{
    rt_ubase_t level;
    struct rt_iommu_group *group = RT_NULL, *group_tmp;

    if (pasid > iommu->max_pasids)
    {
        return rt_err_ptr(-RT_EINVAL);
    }

    level = rt_spin_lock_irqsave(&iommu->lock);
    rt_list_for_each_entry(group_tmp, &iommu->group_nodes, list)
    {
        if (group_tmp->pasid == pasid)
        {
            group = group_tmp;
            break;
        }
    }
    rt_spin_unlock_irqrestore(&iommu->lock, level);

    if (!group)
    {
        group = rt_calloc(1, sizeof(*group));

        if (!group)
        {
            return rt_err_ptr(-RT_ENOMEM);
        }

        group->iommu = iommu;
        group->pasid = pasid;
        rt_list_init(&group->list);
        rt_list_init(&group->page_nodes);
        rt_list_init(&group->domain_nodes);

        level = rt_spin_lock_irqsave(&iommu->lock);

        rt_bitmap_set_bit(iommu->pasids_maps, group->pasid);
        rt_list_insert_before(&iommu->group_nodes, &group->list);

        rt_spin_unlock_irqrestore(&iommu->lock, level);
    }

    return group;
}

void rt_iommu_device_put_group(struct rt_iommu_group *group)
{
    rt_ubase_t level;
    struct rt_iommu_device *iommu;

    if (!group)
    {
        return;
    }

    if (!rt_list_isempty(&group->domain_nodes))
    {
        return;
    }
    iommu = group->iommu;

    if (!rt_list_isempty(&group->page_nodes))
    {
        LOG_E("%s: It is impossible that the pages exists when group release",
                rt_dm_dev_get_name(iommu->dev));
        RT_ASSERT(0);
    }

    level = rt_spin_lock_irqsave(&iommu->lock);

    rt_list_remove(&group->list);
    rt_bitmap_clear_bit(iommu->pasids_maps, group->pasid);

    rt_spin_unlock_irqrestore(&iommu->lock, level);

    rt_free(group);
}

static rt_bool_t domain_mapped_iova(struct rt_iommu_domain *domain,
        void *iova, rt_size_t size, struct rt_iommu_page_shadow **out_page_shadow)
{
    void *start_iova;
    struct rt_iommu_page *page;
    struct rt_iommu_page_shadow *page_shadow;

    rt_list_for_each_entry(page_shadow, &domain->page_shadow_nodes, list)
    {
        page = page_shadow->page;
        start_iova = page->iova + page_shadow->offset;

        if (start_iova <= iova && start_iova + page_shadow->size > iova + size)
        {
            if (out_page_shadow)
            {
                if (start_iova != iova || page_shadow->size != size)
                {
                    continue;
                }

                *out_page_shadow = page_shadow;
            }

            return RT_TRUE;
        }
    }

    return RT_FALSE;
}

rt_err_t rt_iommu_domain_map(struct rt_iommu_domain *domain,
        void *iopa, void *iova, rt_size_t size, mm_flag_t flags)
{
    rt_err_t err;
    void *end_iova;
    rt_bool_t new_page, cross;
    rt_list_t *next_page_list;
    struct rt_iommu_group *group;
    struct rt_iommu_device *iommu;
    struct rt_iommu_page *page, *next_page;
    struct rt_iommu_page_shadow *page_shadow;

    if (!domain || !size)
    {
        return -RT_EINVAL;
    }

    group = domain->group;
    iommu = group->iommu;

_retry:
    rt_spin_lock(&iommu->lock);

    if (group->mapping)
    {
        rt_spin_unlock(&iommu->lock);

        rt_thread_yield();
        goto _retry;
    }

    err = RT_EOK;
    cross = RT_FALSE;
    new_page = RT_TRUE;
    page_shadow = RT_NULL;
    next_page_list = &group->page_nodes;
    end_iova = (void *)RT_ALIGN((rt_ubase_t)(iova + size), iommu->page_size);

    /*
     * Check area:
     *  Because of `page_shadow` point to the `page`, we can not realloc `page`,
     *  or rebase `page` -> iova, merge the next `page`, but we can increase
     *  the size of `page` before the next `page` (flags is equl).
     *
     *  If `domain` just map a area included the `page`, alloc a new page_shadow
     *  is OK.
     *
     *      +---------------+                 +---------------+
     *      |     Page0     |                 |     Page1     |
     *      +---------------+-----------------+---------------+
     *      |   Included    |     Realloc     |
     *      +---------------+-----------------+
     */
    rt_list_for_each_entry(page, &group->page_nodes, list)
    {
        rt_size_t realloc_size;
        void *page_start_iova, *page_end_iova;

        page_start_iova = page->iova;
        page_end_iova = page_start_iova + page->size;

        if (page_start_iova >= end_iova)
        {
            next_page_list = &page->list;
            continue;
        }

        if (page_end_iova < iova)
        {
            if (!cross)
            {
                next_page_list = &page->list;
                cross = RT_TRUE;
            }
            continue;
        }

        if (iova < page_start_iova)
        {
            LOG_E("%s: IOVA(%p) is overflow", rt_dm_dev_get_name(domain->dev), iova);
            err = -RT_EINVAL;
            goto _out_lock;
        }

        new_page = RT_FALSE;

        /* Check PV offset */
        if (iopa - iova != page->iopa - page->iova)
        {
            LOG_E("%s: IOPA and IOVA(%p) offset is not equl",
                    rt_dm_dev_get_name(domain->dev), iova);
            err = -RT_EINVAL;
            goto _out_lock;
        }

        /* Check map flags */
        if (flags != page->flags)
        {
            LOG_E("%s: IOPA and IOVA(%p) flags is not equl",
                    rt_dm_dev_get_name(domain->dev), iova);
            err = -RT_EINVAL;
            goto _out_lock;
        }

        /* Mapped Included? */
        if (page_end_iova > iova + size)
        {
            break;
        }

        next_page = rt_list_first_entry(&page->list, struct rt_iommu_page, list);
        realloc_size = RT_ALIGN(iova + size - page_end_iova, iommu->page_size);

        /* Not the last page */
        if (&next_page->list != &group->page_nodes)
        {
            if (next_page->iova <= page_start_iova + realloc_size)
            {
                LOG_E("%s: IOVA(%p) is overflow", rt_dm_dev_get_name(domain->dev), iova);
                err = -RT_EINVAL;
                goto _out_lock;
            }
        }

        /* Start to realloc */
        err = iommu->ops->map(iommu, domain, page->iopa + page->size,
                page_end_iova, realloc_size, flags);

        if (err)
        {
            goto _out_lock;
        }

        /* Update page */
        page->size += realloc_size;
        break;
    }

    /* Start a new map */
    group->mapping = RT_TRUE;
    rt_spin_unlock(&iommu->lock);

    if (new_page)
    {
        if (!(page = rt_malloc(sizeof(*page))))
        {
            err = -RT_ENOMEM;
            goto _lock;
        }
    }

    if (!(page_shadow = rt_malloc(sizeof(*page_shadow))))
    {
        err = -RT_ENOMEM;
        goto _lock;
    }

_lock:
    rt_spin_lock(&iommu->lock);
    group->mapping = RT_FALSE;

    if (err)
    {
        goto _out_lock;
    }

    if (new_page)
    {
        rt_list_init(&page->list);
        page->iova = (void *)RT_ALIGN_DOWN((rt_ubase_t)iova, iommu->page_size);
        page->iopa = (void *)RT_ALIGN_DOWN((rt_ubase_t)iopa, iommu->page_size);
        page->size = RT_ALIGN(iova + size - page->iova, iommu->page_size);
        page->flags = flags;
        page->ref = 1;

        err = iommu->ops->map(iommu, domain, page->iopa, page->iopa, page->size, flags);

        if (err)
        {
            goto _out_lock;
        }

        rt_list_insert_after(next_page_list, &page->list);
    }
    else
    {
        ++page->ref;
    }

    rt_list_init(&page_shadow->list);
    page_shadow->offset = iova - page->iova;
    page_shadow->size = size;
    page_shadow->page = page;
    rt_list_insert_before(&domain->page_shadow_nodes, &page_shadow->list);

_out_lock:
    rt_spin_unlock(&iommu->lock);

    if (err)
    {
        if (new_page && page)
        {
            rt_free(page);
        }

        if (page_shadow)
        {
            rt_free(page_shadow);
        }
    }

    return err;
}

rt_err_t rt_iommu_domain_unmap(struct rt_iommu_domain *domain, void *iova, rt_size_t size)
{
    rt_err_t err = RT_EOK;
    struct rt_iommu_device *iommu;
    struct rt_iommu_page *page = RT_NULL;
    struct rt_iommu_page_shadow *page_shadow;

    if (!domain)
    {
        return -RT_EINVAL;
    }
    iommu = domain->group->iommu;

    rt_spin_lock(&iommu->lock);

    if (!domain_mapped_iova(domain, iova, size, &page_shadow))
    {
        LOG_D("%s: IOVA %p is not mapped", rt_dm_dev_get_name(domain->dev), iova);
        err = -RT_EINVAL;
        goto _out_lock;
    }

    if (page_shadow->page->ref > 1)
    {
        --page_shadow->page->ref;
    }
    else
    {
        err = iommu->ops->unmap(iommu, domain, page->iova, page->size);

        if (!err)
        {
            page = page_shadow->page;

            rt_list_remove(&page->list);
        }
    }

    if (!err)
    {
        rt_list_remove(&page_shadow->list);
    }

_out_lock:
    rt_spin_unlock(&iommu->lock);

    if (!err)
    {
        rt_free(page_shadow);

        if (page)
        {
            rt_free(page);
        }
    }

    return err;
}

rt_err_t rt_iommu_domain_unmap_all(struct rt_iommu_domain *domain)
{
    rt_err_t err = RT_EOK;
    struct rt_iommu_device *iommu;
    struct rt_iommu_page_shadow *page_shadow, *page_shadow_next;

    if (!domain)
    {
        return -RT_EINVAL;
    }
    iommu = domain->group->iommu;

    rt_spin_lock(&iommu->lock);

    rt_list_for_each_entry_safe(page_shadow, page_shadow_next, &domain->page_shadow_nodes, list)
    {
        void *iova = page_shadow->page->iova + page_shadow->offset;

        rt_spin_unlock(&iommu->lock);

        err |= rt_iommu_domain_unmap(domain, iova, page_shadow->size);

        rt_spin_lock(&iommu->lock);
    }

    rt_spin_unlock(&iommu->lock);

    return err;
}

rt_err_t rt_iommu_domain_translate(struct rt_iommu_domain *domain, void *iova, void **out_iopa)
{
    rt_err_t err = RT_EOK;
    struct rt_iommu_device *iommu;
    struct rt_iommu_page_shadow *page_shadow;

    if (!domain || !out_iopa)
    {
        return -RT_EINVAL;
    }
    iommu = domain->group->iommu;

    rt_spin_lock(&iommu->lock);

    if (!domain_mapped_iova(domain, iova, iommu->page_size, &page_shadow))
    {
        LOG_D("%s: IOVA %p is not mapped", rt_dm_dev_get_name(domain->dev), iova);
        err = -RT_EINVAL;
        goto _out_lock;
    }

    if (iommu->ops->translate)
    {
        err = iommu->ops->translate(iommu, domain, iova, out_iopa);
    }
    else
    {
        struct rt_iommu_page *page = page_shadow->page;

        *out_iopa = page->iopa + page_shadow->offset;
        *out_iopa += iova - (page->iova + page_shadow->offset);
    }

_out_lock:
    rt_spin_unlock(&iommu->lock);

    return err;
}

rt_err_t rt_iommu_domain_iotlb_flush(struct rt_iommu_domain *domain)
{
    rt_err_t err = RT_EOK;
    struct rt_iommu_device *iommu;

    if (!domain)
    {
        return -RT_EINVAL;
    }
    iommu = domain->group->iommu;

    rt_spin_lock(&iommu->lock);

    if (iommu->ops->iotlb_flush)
    {
        err = iommu->ops->iotlb_flush(iommu, domain);
    }

    rt_spin_unlock(&iommu->lock);

    return err;
}

rt_err_t rt_iommu_domain_iotlb_sync(struct rt_iommu_domain *domain, void *iova, rt_size_t size)
{
    rt_err_t err = RT_EOK;
    struct rt_iommu_device *iommu;

    if (!domain || !size)
    {
        return -RT_EINVAL;
    }
    iommu = domain->group->iommu;

    rt_spin_lock(&iommu->lock);

    if (!domain_mapped_iova(domain, iova, iommu->page_size, RT_NULL))
    {
        LOG_D("%s: IOVA %p is not mapped", rt_dm_dev_get_name(domain->dev), iova);
        err = -RT_EINVAL;
        goto _out_lock;
    }

    if (iommu->ops->iotlb_sync)
    {
        err = iommu->ops->iotlb_sync(iommu, domain, iova, size);
    }

_out_lock:
    rt_spin_unlock(&iommu->lock);

    return err;
}

struct rt_iommu_domain *rt_iommu_domain_get_by_index(struct rt_device *dev, int index)
{
    rt_err_t err;
    struct rt_iommu_domain *domain;
    struct rt_iommu_device *iommu = RT_NULL;
#ifdef RT_USING_OFW
    struct rt_ofw_cell_args args;
#endif

    if (!dev || index < 0)
    {
        return rt_err_ptr(-RT_EINVAL);
    }

#ifdef RT_USING_OFW
    if (dev->ofw_node)
    {
        struct rt_ofw_node *np;

        if (!rt_ofw_parse_phandle_cells(dev->ofw_node, "iommus", "#iommu-cells", index, &args))
        {
            np = args.data;

            if (!rt_ofw_data(np))
            {
                rt_platform_ofw_request(np);
            }

            iommu = rt_ofw_data(np);
            rt_ofw_node_put(np);
        }
    }
#endif /* RT_USING_OFW */

    if (!iommu)
    {
        return RT_NULL;
    }

    domain = rt_calloc(1, sizeof(*domain));

    if (!domain)
    {
        return rt_err_ptr(-RT_ENOMEM);
    }
    domain->dev = dev;
    rt_list_init(&domain->list);
    rt_list_init(&domain->page_shadow_nodes);

#ifdef RT_USING_OFW
    if (iommu->ops->ofw_parse)
    {
        if ((err = iommu->ops->ofw_parse(iommu, domain, &args)))
        {
            goto _fail;
        }
    }
#endif

    if ((err = iommu->ops->attach(iommu, domain)))
    {
        goto _fail;
    }

    if (!domain->group)
    {
        LOG_E("%s: No set domain(%s) group", rt_dm_dev_get_name(iommu->dev),
                rt_dm_dev_get_name(domain->dev));
        err = -RT_ERROR;
        goto _fail;
    }

    return domain;

_fail:
    rt_free(domain);

    return rt_err_ptr(err);
}

struct rt_iommu_domain *rt_iommu_domain_get_by_name(struct rt_device *dev, const char *name)
{
    int index;
    struct rt_ofw_node *np;

    if (!dev || !name)
    {
        return rt_err_ptr(-RT_EINVAL);
    }

    np = dev->ofw_node;
    index = rt_ofw_prop_index_of_string(np, "iommu-names", name);

    if (index < 0)
    {
        return RT_NULL;
    }

    return rt_iommu_domain_get_by_index(dev, index);
}

void rt_iommu_domain_put(struct rt_iommu_domain *domain)
{
    rt_err_t err;
    rt_ubase_t level;
    struct rt_iommu_device *iommu;

    if (!domain)
    {
        return;
    }
    iommu = domain->group->iommu;

    rt_iommu_domain_unmap_all(domain);

    if ((err = iommu->ops->detach(iommu, domain)))
    {
        LOG_W("%s: Domain detach error = %s", rt_dm_dev_get_name(domain->dev), rt_strerror(err));
    }

    level = rt_spin_lock_irqsave(&iommu->lock);
    rt_list_remove(&domain->list);
    rt_spin_unlock_irqrestore(&iommu->lock, level);

    rt_iommu_device_put_group(domain->group);
    rt_free(domain);
}

rt_err_t rt_iommu_attach(struct rt_device *dev)
{
    int iommus;

    if (!dev)
    {
        return -RT_EINVAL;
    }

#ifdef RT_USING_OFW
    iommus = rt_ofw_count_phandle_cells(dev->ofw_node, "iommus", "#iommu-cells");

    if (iommus > 1)
    {
        LOG_D("%s ignore IOMMU domains", rt_dm_dev_get_name(dev));
        return RT_EOK;
    }
    else if (iommus < 0)
    {
        if (iommus != -RT_EINVAL)
        {
            return -RT_EEMPTY;
        }

        return iommus;
    }
#else
    (void)iommus;
#endif

    dev->iommu_domain = rt_iommu_domain_get_by_index(dev, 0);

    if (rt_is_err(dev->iommu_domain))
    {
        rt_err_t err = rt_ptr_err(dev->iommu_domain);

        dev->iommu_domain = RT_NULL;

        return err;
    }

    return RT_EOK;
}

rt_err_t rt_iommu_detach(struct rt_device *dev)
{
    if (dev)
    {
        return -RT_EINVAL;
    }

    if (!dev->iommu_domain)
    {
        return RT_EOK;
    }

    rt_iommu_domain_put(dev->iommu_domain);
    dev->iommu_domain = RT_NULL;

    return RT_EOK;
}
