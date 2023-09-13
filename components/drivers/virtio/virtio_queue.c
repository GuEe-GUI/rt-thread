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

#define DBG_TAG "virtio.queue"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <mm_aspace.h>

#include <drivers/virtq.h>
#include <drivers/virtio.h>

#include "virtio_internal.h"

struct virtqueue_split
{
    struct virtq virtq;

    rt_uint16_t avail_flags_shadow;
};

struct virtqueue_packed
{
    struct virtq virtq;
};

static rt_err_t vq_virtq_alloc(struct virtq *virtq, int num, rt_uint32_t align)
{
    void *pages;
    rt_size_t size;

    size = virtq_size(RT_NULL, num, align);
    pages = rt_malloc_align(size, align);

    if (!pages)
    {
        return -RT_ENOMEM;
    }

    virtq_init(virtq, num, pages, align);

    rt_memset(pages, 0, size);

    return RT_EOK;
}

static rt_err_t vq_virtq_free(struct virtq *virtq)
{
    rt_free_align((void *)virtq->desc);

    return RT_EOK;
}

static rt_uint32_t vq_packed_enable_callback(struct rt_virtqueue *vq)
{
    return -RT_ENOSYS;
}

static struct rt_virtqueue *vq_split_create(struct rt_virtio_device *vdev, const char *name, int index,
        int num, rt_uint32_t align, rt_virtqueue_notifier notify, rt_virtqueue_callback callback, void *ext_cfg)
{
    struct virtqueue_split *vq_split;
    struct rt_virtqueue *vq = RT_NULL;

    /*
     * Layout:
     * +-----------------+
     * |   rt_virtqueue  |
     * +-----------------+
     * | virtqueue_split |
     * +-----------------+
     * |   buffer_hash   |
     * +-----------------+
     */
    vq = rt_malloc(sizeof(*vq) + sizeof(struct virtqueue_split) + sizeof(void *) * num);

    if (!vq)
    {
        return RT_NULL;
    }

    vq_split = (void *)vq + sizeof(*vq);

    if (vq_virtq_alloc(&vq_split->virtq, num, align))
    {
        rt_free(vq);

        return RT_NULL;
    }

    vq_split->avail_flags_shadow = 0;

    vq->name = name;
    vq->index = index;
    vq->num_free = num;
    vq->buffer_hash = (void *)vq_split + sizeof(*vq_split);
    rt_memset(vq->buffer_hash, 0, sizeof(void *) * num);

    vq->callback = callback;

    vq->vq_split = vq_split;
    vq->packed_ring = RT_FALSE;

    vq->notify = notify;
    vq->event = rt_virtio_has_feature(vdev, VIRTIO_F_RING_EVENT_IDX);
    vq->event_triggered = RT_FALSE;
    vq->free_head = 0;
    vq->num_added = 0;
    vq->last_used_idx = 0;

    vq->vdev = vdev;

    return vq;
}

static rt_err_t vq_split_delete(struct rt_virtqueue *vq)
{
    rt_err_t err;
    struct virtqueue_split *vq_split = vq->vq_split;

    err = vq_virtq_free(&vq_split->virtq);

    rt_free(vq);

    return err;
}

static rt_err_t vq_split_add_buf(struct rt_virtqueue *vq, void *dma_buf, rt_size_t size, rt_bool_t is_out)
{
    struct virtq *virtq;
    struct virtq_desc *desc;
    rt_size_t virtq_size;
    void *dma_buf_pa = rt_kmem_v2p(dma_buf);
    int head, next, flags = VIRTQ_DESC_F_NEXT;

    if (!dma_buf_pa)
    {
        return -RT_ERROR;
    }

    if (!vq->num_free)
    {
        LOG_D("%s.virtqueue[%s(%d)] add buffer.len = %d fail",
                rt_dm_dev_get_name(&vq->vdev->parent), vq->name, vq->index);

        /* Recvice buffer NOW! */
        if (is_out)
        {
            vq->notify(vq);
        }

        return -RT_EFULL;
    }

    virtq = &vq->vq_split->virtq;
    virtq_size = virtq->num;

    head = vq->free_head++;
    head %= virtq_size;
    rt_hw_dsb();

    next = vq->free_head;
    next %= virtq_size;
    desc = &virtq->desc[head];

    if (!is_out)
    {
        flags |= VIRTQ_DESC_F_WRITE;
    }

    desc->addr = cpu_to_virtio64(vq->vdev, (rt_uint64_t)dma_buf_pa);
    desc->len = cpu_to_virtio32(vq->vdev, size);
    desc->flags = cpu_to_virtio16(vq->vdev, flags);
    desc->next = cpu_to_virtio16(vq->vdev, next);

    vq->buffer_hash[head] = dma_buf;

    /* Update index  */
    vq->num_added++;
    vq->num_free--;

    LOG_D("%s.virtqueue[%s(%d)] add buffer(%p, size = %d) head = %d",
            rt_dm_dev_get_name(&vq->vdev->parent), vq->name, vq->index, dma_buf, size, head);

    if (vq->num_added == RT_UINT16_MAX)
    {
        rt_virtqueue_kick(vq);
    }

    return RT_EOK;
}

static rt_bool_t vq_split_submit(struct rt_virtqueue *vq)
{
    int head, prev;
    struct virtq *virtq = &vq->vq_split->virtq;
    rt_size_t virtq_num = virtq->num;

    head = vq->free_head;
    prev = (head - 1) % virtq_num;
    head = (head - vq->num_added) % virtq_num;

    LOG_D("%s.virtqueue[%s(%d)] submit head = %d, num_added = %d, idx = %d",
            rt_dm_dev_get_name(&vq->vdev->parent), vq->name, vq->index, head, vq->num_added, virtq->avail->idx + 1);

    /* Reset list info */
    vq->num_added = 0;

    /* Clear last "next" flags */
    virtq->desc[prev].flags &= ~cpu_to_virtio16(vq->vdev, VIRTQ_DESC_F_NEXT);
    virtq->desc[prev].next = 0;

    /* Tell the device the first index in our chain of descriptors */
    virtq->avail->ring[virtq->avail->idx % virtq_num] = cpu_to_virtio32(vq->vdev, head);
    rt_hw_dsb();

    /* Tell the device another avail ring entry is available */
    virtq->avail->idx++;
    rt_hw_dsb();

    return RT_TRUE;
}

static rt_bool_t vq_split_poll(struct rt_virtqueue *vq, rt_uint32_t last_used_idx)
{
    return (rt_uint16_t)last_used_idx != virtio16_to_cpu(vq->vdev, vq->vq_split->virtq.used->idx);
}

rt_inline rt_bool_t vq_split_pending(struct rt_virtqueue *vq)
{
    return vq->last_used_idx != virtio16_to_cpu(vq->vdev, vq->vq_split->virtq.used->idx);
}

static void vq_split_disable_callback(struct rt_virtqueue *vq)
{
    struct virtqueue_split *vq_split = vq->vq_split;
    struct virtq *virtq = &vq_split->virtq;

    if (!(vq_split->avail_flags_shadow & VIRTQ_AVAIL_F_NO_INTERRUPT))
    {
        vq_split->avail_flags_shadow |= VIRTQ_AVAIL_F_NO_INTERRUPT;

        if (vq->event)
        {
            *virtq_used_event(virtq) = 0;
        }
        else
        {
            virtq->avail->flags |= cpu_to_virtio16(vq->vdev, vq_split->avail_flags_shadow);
        }
    }
}

static rt_uint32_t vq_split_enable_callback(struct rt_virtqueue *vq)
{
    struct virtqueue_split *vq_split = vq->vq_split;
    struct virtq *virtq = &vq_split->virtq;
    rt_uint16_t last_used_idx = vq->last_used_idx;

    if ((vq_split->avail_flags_shadow & VIRTQ_AVAIL_F_NO_INTERRUPT))
    {
        vq_split->avail_flags_shadow &= ~VIRTQ_AVAIL_F_NO_INTERRUPT;

        if (!vq->event)
        {
            virtq->avail->flags &= ~cpu_to_virtio16(vq->vdev, vq_split->avail_flags_shadow);
        }
    }

    *virtq_used_event(virtq) = cpu_to_virtio16(vq->vdev, last_used_idx);

    return last_used_idx;
}

static void *vq_split_read_buf(struct rt_virtqueue *vq, rt_size_t *out_len)
{
    void *buf;
    rt_uint32_t idx;
    rt_uint16_t last_used, next, flags;
    struct virtq *virtq;

    if (!vq_split_pending(vq))
    {
        LOG_D("%s.virtqueue[%s(%d)] read buffer empty",
                rt_dm_dev_get_name(&vq->vdev->parent), vq->name, vq->index);

        return RT_NULL;
    }

    virtq = &vq->vq_split->virtq;

    last_used = vq->last_used_idx & (virtq->num - 1);
    next = idx = virtio32_to_cpu(vq->vdev, virtq->used->ring[last_used].id);
    *out_len = virtio32_to_cpu(vq->vdev, virtq->used->ring[last_used].len);
    rt_hw_dsb();

    buf = vq->buffer_hash[idx];

    LOG_D("%s.virtqueue[%s(%d)] read head = %d, buffer(%p, size = %d)",
            rt_dm_dev_get_name(&vq->vdev->parent), vq->name, vq->index, idx, buf, *out_len);

    do {
        idx = next;

        flags = virtq->desc[idx].flags;
        next = virtq->desc[idx].next;

        vq->num_free++;

    } while ((flags & VIRTQ_DESC_F_NEXT));

    vq->last_used_idx++;
    rt_hw_dsb();

    return buf;
}

rt_inline rt_bool_t virtqueue_pending(struct rt_virtqueue *vq)
{
    rt_bool_t res = RT_FALSE;

    if (vq->packed_ring)
    {

    }
    else
    {
        res = vq_split_pending(vq);
    }

    return res;
}

struct rt_virtqueue *rt_virtqueue_create(struct rt_virtio_device *vdev, const char *name, int index,
        int num, rt_uint32_t align, rt_virtqueue_notifier notify, rt_virtqueue_callback callback, void *ext_cfg)
{
    struct rt_virtqueue *vq = RT_NULL;

    RT_ASSERT(vdev != RT_NULL);
    RT_ASSERT(num != 0);
    RT_ASSERT(notify != RT_NULL);
    RT_ASSERT(name != RT_NULL);

    if (rt_virtio_has_feature(vdev, VIRTIO_F_RING_PACKED))
    {
    }
    else
    {
        vq = vq_split_create(vdev, name, index, num, align, notify, callback, ext_cfg);
    }

    if (vq)
    {
        if (!callback)
        {
            rt_virtqueue_disable_callback(vq);
        }

        rt_base_t level = rt_spin_lock_irqsave(&vdev->vq_lock);

        rt_list_init(&vq->list);
        rt_list_insert_before(&vdev->vq_node, &vq->list);

        rt_spin_unlock_irqrestore(&vdev->vq_lock, level);
    }

    return vq;
}

rt_err_t rt_virtqueue_delete(struct rt_virtio_device *vdev, struct rt_virtqueue *vq)
{
    rt_ubase_t level;
    rt_err_t err = RT_EOK;

    RT_ASSERT(vdev != RT_NULL);
    RT_ASSERT(vq != RT_NULL);

    level = rt_spin_lock_irqsave(&vdev->vq_lock);

    while (virtqueue_pending(vq))
    {
        rt_spin_unlock_irqrestore(&vdev->vq_lock, level);

        rt_thread_yield();

        level = rt_spin_lock_irqsave(&vdev->vq_lock);
    }

    rt_list_remove(&vq->list);

    if (vq->packed_ring)
    {
    }
    else
    {
        vq_split_delete(vq);
    }

    rt_spin_unlock_irqrestore(&vdev->vq_lock, level);

    return err;
}

rt_err_t rt_virtqueue_add_outbuf(struct rt_virtqueue *vq, void *dma_buf, rt_size_t size)
{
    rt_err_t err = -RT_ENOSYS;

    RT_ASSERT(vq != RT_NULL);
    RT_ASSERT(dma_buf != RT_NULL);
    RT_ASSERT(size != 0);

    if (vq->packed_ring)
    {
    }
    else
    {
        err = vq_split_add_buf(vq, dma_buf, size, RT_TRUE);
    }

    return err;
}

rt_err_t rt_virtqueue_add_inbuf(struct rt_virtqueue *vq, void *dma_buf, rt_size_t size)
{
    rt_err_t err = -RT_ENOSYS;

    RT_ASSERT(vq != RT_NULL);
    RT_ASSERT(dma_buf != RT_NULL);
    RT_ASSERT(size != 0);

    if (vq->packed_ring)
    {
    }
    else
    {
        err = vq_split_add_buf(vq, dma_buf, size, RT_FALSE);
    }

    return err;
}

rt_bool_t rt_virtqueue_prepare(struct rt_virtqueue *vq, rt_uint32_t nr)
{
    return vq->num_free >= nr;
}

rt_uint32_t rt_virtqueue_next_buf_index(struct rt_virtqueue *vq)
{
    RT_ASSERT(vq != RT_NULL);

    return vq->free_head % (vq->packed_ring ? vq->vq_packed->virtq.num : vq->vq_split->virtq.num);
}

rt_bool_t rt_virtqueue_submit(struct rt_virtqueue *vq)
{
    rt_bool_t res = RT_FALSE;

    RT_ASSERT(vq != RT_NULL);

    if (vq->packed_ring)
    {
    }
    else
    {
        res = vq_split_submit(vq);
    }

    return res;
}

rt_bool_t rt_virtqueue_notify(struct rt_virtqueue *vq)
{
    RT_ASSERT(vq != RT_NULL);

    if (!vq->notify(vq))
    {
        return RT_FALSE;
    }

    return RT_TRUE;
}

rt_bool_t rt_virtqueue_kick(struct rt_virtqueue *vq)
{
    RT_ASSERT(vq != RT_NULL);

    if (rt_virtqueue_submit(vq))
    {
        return rt_virtqueue_notify(vq);
    }

    return RT_TRUE;
}

void rt_virtqueue_isr(int irq, struct rt_virtqueue *vq)
{
    if (!virtqueue_pending(vq))
    {
        LOG_D("%s.virtqueue[%s(%d)] no buffer pending in %s",
                rt_dm_dev_get_name(&vq->vdev->parent), vq->name, vq->index, "isr");

        return;
    }

    if (vq->event)
    {
        vq->event_triggered = RT_TRUE;
    }

    if (vq->callback)
    {
        vq->callback(vq);
    }
}

rt_bool_t rt_virtqueue_poll(struct rt_virtqueue *vq, rt_uint32_t last_used_idx)
{
    rt_bool_t res = RT_FALSE;

    RT_ASSERT(vq != RT_NULL);

    if (vq->packed_ring)
    {
    }
    else
    {
        res = vq_split_poll(vq, last_used_idx);
    }

    return res;
}

void rt_virtqueue_disable_callback(struct rt_virtqueue *vq)
{
    if (vq->event_triggered)
    {
        return;
    }

    if (vq->packed_ring)
    {
    }
    else
    {
        vq_split_disable_callback(vq);
    }
}

rt_bool_t rt_virtqueue_enable_callback(struct rt_virtqueue *vq, rt_uint32_t *out_last_used_idx)
{
    rt_uint32_t last_used_idx;

    if (vq->event_triggered)
    {
        vq->event_triggered = RT_FALSE;
    }

    if (vq->packed_ring)
    {
        last_used_idx = vq_packed_enable_callback(vq);
    }
    else
    {
        last_used_idx = vq_split_enable_callback(vq);
    }

    if (out_last_used_idx)
    {
        *out_last_used_idx = last_used_idx;
    }

    return !rt_virtqueue_poll(vq, last_used_idx);
}

void *rt_virtqueue_read_buf(struct rt_virtqueue *vq, rt_size_t *out_len)
{
    void *buf = RT_NULL;
    rt_size_t len = 0;

    RT_ASSERT(vq != RT_NULL);

    if (vq->packed_ring)
    {
    }
    else
    {
        buf = vq_split_read_buf(vq, &len);
    }

    if (len && out_len)
    {
        *out_len = len;
    }

    return buf;
}

rt_size_t rt_virtqueue_get_virtq_size(struct rt_virtqueue *vq)
{
    RT_ASSERT(vq != RT_NULL);

    if (vq->packed_ring)
    {
        return (rt_ubase_t)vq->vq_packed->virtq.num;
    }
    else
    {
        return (rt_ubase_t)vq->vq_split->virtq.num;
    }
}

rt_ubase_t rt_virtqueue_get_desc_addr(struct rt_virtqueue *vq)
{
    RT_ASSERT(vq != RT_NULL);

    if (vq->packed_ring)
    {
        return (rt_ubase_t)vq->vq_packed->virtq.desc;
    }
    else
    {
        return (rt_ubase_t)vq->vq_split->virtq.desc;
    }
}

rt_ubase_t rt_virtqueue_get_avail_addr(struct rt_virtqueue *vq)
{
    RT_ASSERT(vq != RT_NULL);

    if (vq->packed_ring)
    {
        return (rt_ubase_t)vq->vq_packed->virtq.avail;
    }
    else
    {
        return (rt_ubase_t)vq->vq_split->virtq.avail;
    }
}

rt_ubase_t rt_virtqueue_get_used_addr(struct rt_virtqueue *vq)
{
    RT_ASSERT(vq != RT_NULL);

    if (vq->packed_ring)
    {
        return (rt_ubase_t)vq->vq_packed->virtq.used;
    }
    else
    {
        return (rt_ubase_t)vq->vq_split->virtq.used;
    }
}
