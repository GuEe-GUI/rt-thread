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

#define DBG_TAG "virtio.dev.rng"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <cpuport.h>

#include "virtio-rng.h"

struct virtio_rng_request
{
    /*
     * Layout:
     * +----------------+
     * |     data[]     |
     * +----------------+
     */

    rt_uint8_t data[sizeof(rt_uint32_t)];

    /* user data */
    rt_bool_t done;
    rt_uint32_t *rand_ptr;
};

struct virtio_rng
{
    struct rt_hwcrypto_device parent;
    struct rt_virtio_device *vdev;

    struct rt_virtqueue *vqs[1];

    rt_size_t virtq_nr;
    rt_size_t request_nr;
    struct virtio_rng_request *request;
};

static rt_uint32_t virtio_rng_rand(struct hwcrypto_rng *ctx)
{
    rt_base_t level;
    rt_uint32_t rand;
    struct rt_virtqueue *vq = RT_NULL;
    struct virtio_rng_request *request;
    struct virtio_rng *vrng = rt_container_of(ctx->parent.device, struct virtio_rng, parent);

    for (;;)
    {
        level = rt_spin_lock_irqsave(&vrng->vdev->vq_lock);

        for (int i = 0; i < RT_ARRAY_SIZE(vrng->vqs); ++i)
        {
            if (rt_virtqueue_prepare(vrng->vqs[i], 1))
            {
                vq = vrng->vqs[i];

                break;
            }
        }

        if (vq)
        {
            break;
        }

        rt_spin_unlock_irqrestore(&vrng->vdev->vq_lock, level);

        rt_thread_yield();
    }

    request = &vrng->request[vq->index * vrng->virtq_nr + rt_virtqueue_next_buf_index(vq)];
    request->done = RT_FALSE;
    request->rand_ptr = &rand;

    rt_virtqueue_add_inbuf(vq, &request->data, sizeof(request->data));

    rt_virtqueue_kick(vq);

    rt_spin_unlock_irqrestore(&vrng->vdev->vq_lock, level);

    while (!request->done)
    {
        rt_hw_cpu_relax();
    }

    return rand;
}

static const struct hwcrypto_rng_ops rng_ops =
{
    .update = virtio_rng_rand,
};

static rt_err_t virtio_rng_create(struct rt_hwcrypto_ctx *ctx)
{
    rt_err_t res = RT_EOK;
    struct hwcrypto_rng *rng;

    switch (ctx->type & HWCRYPTO_MAIN_TYPE_MASK)
    {
    case HWCRYPTO_TYPE_RNG:
        ctx->contex = RT_NULL;

        rng = rt_container_of(ctx, struct hwcrypto_rng, parent);
        rng->ops = &rng_ops;
        break;

    default:
        res = -RT_ENOSYS;
        break;
    }

    return res;
}

static void virtio_rng_destroy(struct rt_hwcrypto_ctx *ctx)
{
    rt_free(ctx->contex);
}

static rt_err_t virtio_rng_copy(struct rt_hwcrypto_ctx *des, const struct rt_hwcrypto_ctx *src)
{
    rt_err_t err = RT_EOK;

    switch (src->type & HWCRYPTO_MAIN_TYPE_MASK)
    {
    case HWCRYPTO_TYPE_RNG:
        break;
    default:
        err = -RT_ENOSYS;
        break;
    }

    return err;
}

static void virtio_rng_reset(struct rt_hwcrypto_ctx *ctx)
{
}

static const struct rt_hwcrypto_ops virtio_rng_ops =
{
    .create = virtio_rng_create,
    .destroy = virtio_rng_destroy,
    .copy = virtio_rng_copy,
    .reset = virtio_rng_reset,
};

static void virtio_rng_done(struct rt_virtqueue *vq)
{
    struct virtio_rng_request *request;
    rt_ubase_t level = rt_spin_lock_irqsave(&vq->vdev->vq_lock);

    while ((request = rt_virtqueue_read_buf(vq, RT_NULL)))
    {
        rt_memcpy(request->rand_ptr, request->data, sizeof(request->data));

        request->done = RT_TRUE;
    }

    rt_spin_unlock_irqrestore(&vq->vdev->vq_lock, level);
}

static rt_err_t virtio_rng_vq_init(struct virtio_rng *vrng)
{
    const char *names[] =
    {
        "req",
    };
    rt_virtqueue_callback cbs[] =
    {
        &virtio_rng_done,
    };

    vrng->virtq_nr = rt_min(RT_CPUS_NR, 1024);
    vrng->request_nr = vrng->virtq_nr;
    vrng->request = rt_malloc(sizeof(struct virtio_rng_request) * vrng->request_nr);

    if (!vrng->request)
    {
        return -RT_ENOMEM;
    }

    for (int i = 0; i < RT_ARRAY_SIZE(names); ++i)
    {
        rt_virtio_virtqueue_preset_max(&vrng->vqs[i], vrng->virtq_nr);
    }

    return rt_virtio_virtqueue_install(vrng->vdev, RT_ARRAY_SIZE(names), vrng->vqs, names, cbs);
}

static void virtio_rng_vq_finit(struct virtio_rng *vrng)
{
    if (vrng->request)
    {
        if (vrng->vqs[0])
        {
            rt_virtio_virtqueue_release(vrng->vdev);
        }

        rt_free(vrng->request);
    }
}

static rt_err_t virtio_rng_probe(struct rt_virtio_device *vdev)
{
    rt_err_t err;
    struct virtio_rng *vrng = rt_calloc(1, sizeof(*vrng));

    if (!vrng)
    {
        return -RT_ENOMEM;
    }

    vrng->vdev = vdev;

    if ((err = virtio_rng_vq_init(vrng)))
    {
        goto _fail;
    }

    vrng->parent.ops = &virtio_rng_ops;
    vrng->parent.id = vdev->id.vendor;

    if ((err = rt_hwcrypto_register(&vrng->parent, "hwrng")))
    {
        goto _fail;
    }

    return RT_EOK;

_fail:
    virtio_rng_vq_finit(vrng);
    rt_free(vrng);

    return err;
}

static const struct rt_virtio_device_id virtio_rng_ids[] =
{
    { VIRTIO_DEVICE_ID_RNG, VIRTIO_DEVICE_ANY_ID },
    { /* sentinel */ }
};

static struct rt_virtio_driver virtio_rng_driver =
{
    .ids = virtio_rng_ids,
    .probe = virtio_rng_probe,
};
RT_VIRTIO_DRIVER_EXPORT(virtio_rng_driver);
