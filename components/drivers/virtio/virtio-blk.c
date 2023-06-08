/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-9-16      GuEe-GUI     the first version
 * 2021-11-11     GuEe-GUI     using virtio common interface
 * 2023-02-25     GuEe-GUI     using virtio dm
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "virtio.dev.blk"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <cpuport.h>

#include "virtio-blk.h"

#define VIRTIO_BLK_VQS_NR   1

struct virtio_blk_request
{
#define VIRTIO_BLK_REQUEST_SPLIT_NR 3
    /*
     * Layout:
     * +----------------+
     * | virtio_blk_req |
     * +----------------+
     * |     data[]     |
     * +----------------+
     * |     status     |
     * +----------------+
     */

    struct virtio_blk_req req;
    rt_uint8_t status;

    /* user data */
    rt_bool_t done;
};

struct virtio_blk
{
    struct rt_device parent;
    struct rt_virtio_device *vdev;

    rt_le32_t blk_size;

    struct rt_virtqueue *vqs[VIRTIO_BLK_VQS_NR];

    rt_size_t virtq_nr;
    rt_size_t request_nr;
    struct virtio_blk_request *request;
};

#define raw_to_virtio_blk(raw) rt_container_of(raw, struct virtio_blk, parent)

static void virtio_blk_rw(struct virtio_blk *vblk, rt_off_t pos, void *buffer, rt_size_t count, int flags)
{
    rt_size_t size;
    rt_base_t level;
    struct rt_virtqueue *vq = RT_NULL;
    struct virtio_blk_request *request;

    for (;;)
    {
        level = rt_spin_lock_irqsave(&vblk->vdev->vq_lock);

        for (int i = 0; i < RT_ARRAY_SIZE(vblk->vqs); ++i)
        {
            if (rt_virtqueue_prepare(vblk->vqs[i], VIRTIO_BLK_REQUEST_SPLIT_NR))
            {
                vq = vblk->vqs[i];

                break;
            }
        }

        if (vq)
        {
            break;
        }

        rt_spin_unlock_irqrestore(&vblk->vdev->vq_lock, level);

        rt_thread_yield();
    }

    size = count * vblk->blk_size;

    request = &vblk->request[vq->index * vblk->virtq_nr + rt_virtqueue_next_buf_index(vq)];
    request->done = RT_FALSE;
    request->req.type = flags;
    request->req.ioprio = 0;
    request->req.sector = pos * (vblk->blk_size / 512);
    request->status = 0xff;

    rt_virtqueue_add_outbuf(vq, &request->req, sizeof(request->req));

    if (flags == VIRTIO_BLK_T_OUT)
    {
        rt_virtqueue_add_outbuf(vq, buffer, size);
    }
    else
    {
        rt_virtqueue_add_inbuf(vq, buffer, size);
    }

    rt_virtqueue_add_inbuf(vq, &request->status, sizeof(request->status));

    rt_virtqueue_kick(vq);

    rt_spin_unlock_irqrestore(&vblk->vdev->vq_lock, level);

    while (!request->done)
    {
        rt_hw_cpu_relax();
    }
}

static rt_ssize_t virtio_blk_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t count)
{
    struct virtio_blk *vblk = raw_to_virtio_blk(dev);

    virtio_blk_rw(vblk, pos, buffer, count, VIRTIO_BLK_T_IN);

    return count;
}

static rt_ssize_t virtio_blk_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t count)
{
    struct virtio_blk *vblk = raw_to_virtio_blk(dev);

    virtio_blk_rw(vblk, pos, (void *)buffer, count, VIRTIO_BLK_T_OUT);

    return count;
}

static rt_err_t virtio_blk_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t err = RT_EOK;
    rt_le64_t capacity;
    struct rt_device_blk_geometry *geometry;
    struct virtio_blk *vblk = raw_to_virtio_blk(dev);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_BLK_GETGEOME:
        geometry = args;

        if (!geometry)
        {
            err = -RT_EINVAL;
            break;
        }

        rt_virtio_read_config(vblk->vdev, struct virtio_blk_config, capacity, &capacity);

        geometry->bytes_per_sector = 512;
        geometry->block_size = vblk->blk_size;
        geometry->sector_count = capacity;
        break;

    default:
        err = -RT_EINVAL;
        break;
    }

    return err;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops virtio_blk_ops =
{
    .read = virtio_blk_read,
    .write = virtio_blk_write,
    .control = virtio_blk_control,
};
#endif

static void virtio_blk_done(struct rt_virtqueue *vq)
{
    struct virtio_blk_request *request;
    rt_ubase_t level = rt_spin_lock_irqsave(&vq->vdev->vq_lock);

    while ((request = rt_virtqueue_read_buf(vq, RT_NULL)))
    {
        request->done = RT_TRUE;

        if (request->status != VIRTIO_BLK_S_OK)
        {
            LOG_E("request status = %d is fail", request->status);
            RT_ASSERT(0);
        }
    }

    rt_spin_unlock_irqrestore(&vq->vdev->vq_lock, level);
}

static rt_err_t virtio_blk_vq_init(struct virtio_blk *vblk)
{
    rt_size_t vqs_nr = VIRTIO_BLK_VQS_NR;
    const char *names[VIRTIO_BLK_VQS_NR];
    rt_virtqueue_callback cbs[VIRTIO_BLK_VQS_NR];

    if (vqs_nr > 1 && rt_virtio_has_feature(vblk->vdev, VIRTIO_BLK_F_MQ))
    {
        vqs_nr = 1;

        LOG_W("%s-%s not support %s", rt_dm_get_dev_name(&vblk->vdev->parent), "blk", "VIRTIO_BLK_F_MQ");
    }

    vblk->virtq_nr = rt_min(RT_CPUS_NR, 1024) * VIRTIO_BLK_REQUEST_SPLIT_NR;
    vblk->request_nr = (vblk->virtq_nr / VIRTIO_BLK_REQUEST_SPLIT_NR) * vqs_nr;
    vblk->request = rt_malloc(sizeof(struct virtio_blk_request) * vblk->request_nr);

    if (!vblk->request)
    {
        return -RT_ENOMEM;
    }

    for (int i = 0; i < vqs_nr; ++i)
    {
        names[i] = "req";
        cbs[i] = &virtio_blk_done;

        rt_virtio_virtqueue_preset_max(&vblk->vqs[i], vblk->virtq_nr);
    }

    return rt_virtio_virtqueue_install(vblk->vdev, vqs_nr, vblk->vqs, names, cbs);
}

static void virtio_blk_vq_finit(struct virtio_blk *vblk)
{
    if (vblk->request)
    {
        if (vblk->vqs[0])
        {
            rt_virtio_virtqueue_release(vblk->vdev);
        }

        rt_free(vblk->request);
    }
}

static rt_err_t virtio_blk_probe(struct rt_virtio_device *vdev)
{
    rt_err_t err;
    struct virtio_blk *vblk = rt_calloc(1, sizeof(*vblk));

    if (!vblk)
    {
        return -RT_ENOMEM;
    }

    vblk->vdev = vdev;

    vblk->parent.type = RT_Device_Class_Block;
#ifdef RT_USING_DEVICE_OPS
    vblk->parent.ops = &virtio_blk_ops;
#else
    vblk->parent.read = virtio_blk_read;
    vblk->parent.write = virtio_blk_write;
    vblk->parent.control = virtio_blk_control;
#endif

    if ((err = virtio_blk_vq_init(vblk)))
    {
        goto _fail;
    }

    if ((err = rt_dm_set_dev_name_auto(&vblk->parent, "block")) < 0)
    {
        goto _fail;
    }

    if ((err = rt_device_register(&vblk->parent, rt_dm_get_dev_name(&vblk->parent), RT_DEVICE_FLAG_RDWR)))
    {
        goto _fail;
    }

    rt_virtio_read_config(vblk->vdev, struct virtio_blk_config, blk_size, &vblk->blk_size);

    return RT_EOK;

_fail:
    virtio_blk_vq_finit(vblk);
    rt_free(vblk);

    return err;
}

static const struct rt_virtio_device_id virtio_blk_ids[] =
{
    { VIRTIO_DEVICE_ID_BLOCK, VIRTIO_DEVICE_ANY_ID },
    { /* sentinel */ }
};

static struct rt_virtio_driver virtio_blk_driver =
{
    .ids = virtio_blk_ids,
    .features =
        RT_BIT(VIRTIO_BLK_F_SIZE_MAX)
      | RT_BIT(VIRTIO_BLK_F_GEOMETRY)
      | RT_BIT(VIRTIO_BLK_F_BLK_SIZE)
      | RT_BIT(VIRTIO_BLK_F_TOPOLOGY)
      | RT_BIT(VIRTIO_BLK_F_MQ)
      | RT_BIT(VIRTIO_BLK_F_DISCARD)
      | RT_BIT(VIRTIO_BLK_F_WRITE_ZEROES),

    .probe = virtio_blk_probe,
};
RT_VIRTIO_DRIVER_EXPORT(virtio_blk_driver);
