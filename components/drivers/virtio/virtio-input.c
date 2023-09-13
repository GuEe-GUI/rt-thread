/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-11     GuEe-GUI     the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "virtio.dev.input"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "virtio-input.h"

/* Event types */
#define EV_SYN          0x00            /* Synchronization events */
#define EV_KEY          0x01            /* Keys and buttons type */
#define EV_REL          0x02            /* Relative axes events */
#define EV_ABS          0x03            /* Absolute axes events */
#define EV_MSC          0x04            /* Misc events */
#define EV_SW           0x05            /* Switch events */
#define EV_LED          0x11            /* LEDs events */
#define EV_SND          0x12            /* Sounds events */
#define EV_REP          0x14            /* Repeat events */
#define EV_FF           0x15            /* Force feedback events */
#define EV_PWR          0x16            /* Power management events */
#define EV_FF_STATUS    0x17            /* Force feedback state */
#define EV_MAX          0x1f            /* Maximum number of events */
#define EV_CNT          (EV_MAX + 1)    /* Event count */

#define VIRTIO_INPUT_QUEUE_EVENTS   0
#define VIRTIO_INPUT_QUEUE_STATUS   1

struct virtio_input_request
{
    /*
     * Layout:
     * +--------------------+
     * | virtio_input_event |
     * +--------------------+
     */
    struct virtio_input_event events;
    struct virtio_input_event status;

    /* user data */
    struct virtio_input_event cache;
};

struct virtio_input
{
    struct rt_device parent;
    struct rt_virtio_device *vdev;

    struct rt_virtqueue *vqs[2];

    int queue_idx;
    int queue_last_idx;
    int queue_size;
    rt_size_t virtq_nr;
    rt_size_t request_nr;
    struct virtio_input_request *request;
};

#define raw_to_virtio_input(raw) rt_container_of(raw, struct virtio_input, parent)

rt_inline void virtio_input_enqueue(struct virtio_input *vinput, struct virtio_input_event *event)
{
    rt_memcpy(&vinput->request[vinput->queue_idx].cache, event, sizeof(*event));

    vinput->queue_idx++;
    vinput->queue_idx %= vinput->queue_size;
}

rt_inline struct virtio_input_event *virtio_input_dequeue(struct virtio_input *vinput)
{
    struct virtio_input_event *cache = RT_NULL;

    if (vinput->queue_last_idx != vinput->queue_idx)
    {
        cache = &vinput->request[vinput->queue_last_idx].cache;

        vinput->queue_last_idx++;
        vinput->queue_last_idx %= vinput->queue_size;
    }

    return cache;
}

static rt_ssize_t virtio_input_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_ubase_t level;
    struct rt_virtio_device *vdev;
    struct virtio_input_event *in_event, *out_event;
    struct virtio_input *vinput = raw_to_virtio_input(dev);

    if (pos >= vinput->request_nr || pos + size >= vinput->request_nr)
    {
        return -RT_EINVAL;
    }

    vdev = vinput->vdev;
    level = rt_spin_lock_irqsave(&vdev->vq_lock);

    out_event = buffer;

    for (int i = 0; i < size; ++i)
    {
        in_event = virtio_input_dequeue(vinput);

        if (!in_event)
        {
            size = i;

            break;
        }

        out_event->type = rt_le16_to_cpu(in_event->type);
        out_event->code = rt_le16_to_cpu(in_event->code);
        out_event->value = rt_le32_to_cpu(in_event->value);

        ++out_event;
        ++pos;
    }

    rt_spin_unlock_irqrestore(&vdev->vq_lock, level);

    return size;
}

static rt_ssize_t virtio_input_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_ubase_t level;
    struct rt_virtqueue *vq;
    struct rt_virtio_device *vdev;
    struct virtio_input_event *status;
    struct virtio_input_request *request;
    struct virtio_input *vinput = raw_to_virtio_input(dev);

    if (pos >= vinput->request_nr || pos + size >= vinput->request_nr)
    {
        return -RT_EINVAL;
    }

    vdev = vinput->vdev;
    level = rt_spin_lock_irqsave(&vdev->vq_lock);

    status = (struct virtio_input_event *)buffer;

    for (int i = 0; i < size; ++i)
    {
        for (;;)
        {
            for (int i = 0; i < RT_ARRAY_SIZE(vinput->vqs); ++i)
            {
                if (rt_virtqueue_prepare(vinput->vqs[VIRTIO_INPUT_QUEUE_STATUS], 1))
                {
                    vq = vinput->vqs[i];

                    break;
                }
            }

            if (vq)
            {
                break;
            }

            rt_spin_unlock_irqrestore(&vdev->vq_lock, level);

            rt_thread_yield();

            level = rt_spin_lock_irqsave(&vdev->vq_lock);
        }

        request = &vinput->request[rt_virtqueue_next_buf_index(vq)];

        request->status.type = rt_cpu_to_le16(status->type);
        request->status.code = rt_cpu_to_le16(status->code);
        request->status.value = rt_cpu_to_le32(status->value);

        rt_virtqueue_add_outbuf(vq, &request->status, sizeof(request->status));

        rt_virtqueue_kick(vq);

        ++status;
        ++pos;
    }

    rt_spin_unlock_irqrestore(&vdev->vq_lock, level);

    return size;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops virtio_input_ops =
{
    .read = virtio_input_read,
    .write = virtio_input_write,
};
#endif

static void virtio_input_events_done(struct rt_virtqueue *vq)
{
    struct virtio_input_request *request;
    struct virtio_input *vinput = vq->vdev->priv;
    rt_ubase_t level = rt_spin_lock_irqsave(&vq->vdev->vq_lock);

    while ((request = rt_virtqueue_read_buf(vq, RT_NULL)))
    {
        virtio_input_enqueue(vinput, &request->events);
    }

    rt_spin_unlock_irqrestore(&vq->vdev->vq_lock, level);
}

static void virtio_input_status_done(struct rt_virtqueue *vq)
{
    rt_virtqueue_read_buf(vq, RT_NULL);
}

static rt_uint8_t virtio_input_cfg_select(struct virtio_input *vinput, rt_uint8_t select, rt_uint8_t subsel)
{
    rt_uint8_t size = 0;
    struct rt_virtio_device *vdev = vinput->vdev;

    rt_virtio_write_config(vdev, struct virtio_input_config, select, &select);
    rt_virtio_write_config(vdev, struct virtio_input_config, subsel, &subsel);
    rt_virtio_read_config(vdev, struct virtio_input_config, size, &size);

    return size;
}

static rt_err_t virtio_input_vq_init(struct virtio_input *vinput)
{
    const char *names[] =
    {
        "events",
        "status"
    };
    rt_virtqueue_callback cbs[] =
    {
        &virtio_input_events_done,
        &virtio_input_status_done,
    };

    vinput->virtq_nr = rt_min(RT_CPUS_NR * 16, 1024);
    vinput->request_nr = vinput->virtq_nr;
    vinput->request = rt_malloc(sizeof(struct virtio_input_request) * vinput->request_nr);

    if (!vinput->request)
    {
        return -RT_ENOMEM;
    }

    for (int i = 0; i < RT_ARRAY_SIZE(names); ++i)
    {
        rt_virtio_virtqueue_preset_max(&vinput->vqs[i], vinput->virtq_nr);
    }

    return rt_virtio_virtqueue_install(vinput->vdev, RT_ARRAY_SIZE(names), vinput->vqs, names, cbs);
}

static void virtio_input_vq_finit(struct virtio_input *vinput)
{
    if (vinput->request)
    {
        if (vinput->vqs[0])
        {
            rt_virtio_virtqueue_release(vinput->vdev);
        }

        rt_free(vinput->request);
    }
}

static rt_err_t virtio_input_probe(struct rt_virtio_device *vdev)
{
    rt_err_t err;
    const char *dev_name;
    struct virtio_input_devids devids;
    struct virtio_input *vinput = rt_calloc(1, sizeof(*vinput));

    if (!vinput)
    {
        return -RT_ENOMEM;
    }

    vdev->priv = vinput;
    vinput->vdev = vdev;

    virtio_input_cfg_select(vinput, VIRTIO_INPUT_CFG_ID_DEVIDS, 0);
    rt_virtio_read_config(vinput->vdev, struct virtio_input_config, ids.product, &devids.product);

    if (devids.product == EV_ABS)
    {
        vinput->parent.type = RT_Device_Class_Touch;
    }
    else
    {
        /* Replace it to "KeyBoard" or "Mouse" if support in the future */
        if (devids.product == EV_KEY)
        {
            vinput->parent.type = RT_Device_Class_Miscellaneous;
        }
        else
        {
            vinput->parent.type = RT_Device_Class_Miscellaneous;
        }
    }

#ifdef RT_USING_DEVICE_OPS
    vinput->parent.ops = &virtio_input_ops;
#else
    vinput->parent.read = virtio_input_read;
    vinput->parent.write = virtio_input_write;
#endif

    if ((err = virtio_input_vq_init(vinput)) < 0)
    {
        goto _fail;
    }

    if ((err = rt_dm_dev_set_name_auto(&vinput->parent, "input-event")) < 0)
    {
        goto _fail;
    }

    dev_name = rt_dm_dev_get_name(&vinput->parent);

    if ((err = rt_device_register(&vinput->parent, dev_name, RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX)))
    {
        goto _fail;
    }

    return RT_EOK;

_fail:
    virtio_input_vq_finit(vinput);
    rt_free(vinput);

    return err;
}

static const struct rt_virtio_device_id virtio_input_ids[] =
{
    { VIRTIO_DEVICE_ID_INPUT, VIRTIO_DEVICE_ANY_ID },
    { /* sentinel */ }
};

static struct rt_virtio_driver virtio_input_driver =
{
    .ids = virtio_input_ids,
    .probe = virtio_input_probe,
};
RT_VIRTIO_DRIVER_EXPORT(virtio_input_driver);
