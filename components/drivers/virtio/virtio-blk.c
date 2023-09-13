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
#include "virtio_internal.h"

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

    /* user data */
    rt_bool_t done;
};

struct virtio_blk
{
    struct rt_blk_disk parent;
    struct rt_virtio_device *vdev;

    rt_le32_t blk_size;

    struct rt_virtqueue *vqs[VIRTIO_BLK_VQS_NR];

    rt_size_t virtq_nr;
    rt_size_t request_nr;
    struct virtio_blk_request *request;
};

#define raw_to_virtio_blk(raw) rt_container_of(raw, struct virtio_blk, parent)

static rt_err_t virtio_blk_rw(struct virtio_blk *vblk,
        rt_off_t sector, void *buffer, rt_size_t sector_count, int type)
{
    rt_size_t size;
    rt_base_t level;
    rt_uint8_t status = 0xff;
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

    size = sector_count * vblk->blk_size;

    request = &vblk->request[vq->index * vblk->virtq_nr + rt_virtqueue_next_buf_index(vq)];
    request->done = RT_FALSE;
    request->req.type = cpu_to_virtio32(vblk->vdev, type);
    request->req.ioprio = cpu_to_virtio32(vblk->vdev, 0);
    request->req.sector = cpu_to_virtio64(vblk->vdev, sector * (vblk->blk_size / 512));

    rt_virtqueue_add_outbuf(vq, &request->req, sizeof(request->req));

    if (type == VIRTIO_BLK_T_OUT)
    {
        rt_virtqueue_add_outbuf(vq, buffer, size);
    }
    else if (type == VIRTIO_BLK_T_IN)
    {
        rt_virtqueue_add_inbuf(vq, buffer, size);
    }

    rt_virtqueue_add_inbuf(vq, &status, sizeof(status));

    rt_virtqueue_kick(vq);

    rt_spin_unlock_irqrestore(&vblk->vdev->vq_lock, level);

    while (!request->done)
    {
        rt_hw_cpu_relax();
    }

    switch (status)
    {
    case VIRTIO_BLK_S_OK:
        return RT_EOK;

    case VIRTIO_BLK_S_UNSUPP:
        return -RT_ENOSYS;

    case VIRTIO_BLK_S_ZONE_OPEN_RESOURCE:
        return 1;

    case VIRTIO_BLK_S_ZONE_ACTIVE_RESOURCE:
        return 2;

    case VIRTIO_BLK_S_IOERR:
    case VIRTIO_BLK_S_ZONE_UNALIGNED_WP:
    default:
        return -RT_EIO;
    }
}

static rt_ssize_t virtio_blk_read(struct rt_blk_disk *disk, rt_off_t sector, void *buffer, rt_size_t sector_count)
{
    rt_ssize_t res;
    struct virtio_blk *vblk = raw_to_virtio_blk(disk);

    res = virtio_blk_rw(vblk, sector, buffer, sector_count, VIRTIO_BLK_T_IN);

    return res >= 0 ? sector_count : res;
}

static rt_ssize_t virtio_blk_write(struct rt_blk_disk *disk, rt_off_t sector, const void *buffer, rt_size_t sector_count)
{
    rt_ssize_t res;
    struct virtio_blk *vblk = raw_to_virtio_blk(disk);

    res = virtio_blk_rw(vblk, sector, (void *)buffer, sector_count, VIRTIO_BLK_T_OUT);

    return res >= 0 ? sector_count : res;
}

static rt_err_t virtio_blk_getgeome(struct rt_blk_disk *disk, struct rt_device_blk_geometry *geometry)
{
    rt_le64_t capacity;
    struct virtio_blk *vblk = raw_to_virtio_blk(disk);

    rt_virtio_read_config(vblk->vdev, struct virtio_blk_config, capacity, &capacity);

    geometry->bytes_per_sector = 512;
    geometry->block_size = vblk->blk_size;
    geometry->sector_count = rt_le64_to_cpu(capacity);

    return RT_EOK;
}

static rt_err_t virtio_blk_sync(struct rt_blk_disk *disk)
{
    struct virtio_blk *vblk = raw_to_virtio_blk(disk);

    return virtio_blk_rw(vblk, 0, RT_NULL, 0, VIRTIO_BLK_T_FLUSH);
}

static rt_err_t virtio_blk_erase(struct rt_blk_disk *disk)
{
    struct virtio_blk *vblk = raw_to_virtio_blk(disk);

    return virtio_blk_rw(vblk, 0, RT_NULL, 0, VIRTIO_BLK_T_SECURE_ERASE);
}

static rt_err_t virtio_blk_autorefresh(struct rt_blk_disk *disk, rt_bool_t is_auto)
{
    rt_uint8_t writeback = !is_auto;
    struct virtio_blk *vblk = raw_to_virtio_blk(disk);

    /*
     * 0: write through
     * 1: write back
     */
    rt_virtio_write_config(vblk->vdev, struct virtio_blk_config, writeback, &writeback);

    return RT_EOK;
}

static const struct rt_blk_disk_ops virtio_blk_ops =
{
    .read = virtio_blk_read,
    .write = virtio_blk_write,
    .getgeome = virtio_blk_getgeome,
    .sync = virtio_blk_sync,
    .erase = virtio_blk_erase,
    .autorefresh = virtio_blk_autorefresh,
};

static void virtio_blk_done(struct rt_virtqueue *vq)
{
    struct virtio_blk_request *request;
    rt_ubase_t level = rt_spin_lock_irqsave(&vq->vdev->vq_lock);

    while ((request = rt_virtqueue_read_buf(vq, RT_NULL)))
    {
        request->done = RT_TRUE;
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

        LOG_W("%s-%s not support %s", rt_dm_dev_get_name(&vblk->vdev->parent), "blk", "VIRTIO_BLK_F_MQ");
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
    static int index = 0;
    struct virtio_blk *vblk = rt_calloc(1, sizeof(*vblk));

    if (!vblk)
    {
        return -RT_ENOMEM;
    }

    vblk->vdev = vdev;
    vblk->parent.parallel_io = RT_TRUE;
    vblk->parent.ops = &virtio_blk_ops;
    vblk->parent.max_partitions = RT_BLK_PARTITION_MAX;

    if ((err = virtio_blk_vq_init(vblk)))
    {
        goto _fail;
    }

    rt_virtio_read_config(vblk->vdev, struct virtio_blk_config, blk_size, &vblk->blk_size);

    rt_dm_dev_set_name(&vblk->parent.parent, "vd%c", 'a' + index);

    if ((err = rt_hw_blk_disk_register(&vblk->parent)))
    {
        goto _fail;
    }

    ++index;

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
      | RT_BIT(VIRTIO_BLK_F_CONFIG_WCE)
      | RT_BIT(VIRTIO_BLK_F_MQ)
      | RT_BIT(VIRTIO_BLK_F_DISCARD)
      | RT_BIT(VIRTIO_BLK_F_WRITE_ZEROES)
      | RT_BIT(VIRTIO_BLK_F_SECURE_ERASE),

    .probe = virtio_blk_probe,
};
RT_VIRTIO_DRIVER_EXPORT(virtio_blk_driver);
