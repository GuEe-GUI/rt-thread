/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-11     GuEe-GUI     the first version
 * 2023-02-25     GuEe-GUI     using virtio dm
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "virtio.dev.net"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <mm_aspace.h>
#include <netif/ethernetif.h>

#include "virtio-net.h"

#define VIRTIO_NET_VQS_NR       2
#define VIRTIO_NET_QUEUE_RX(n)  (n)
#define VIRTIO_NET_QUEUE_TX(n)  (n + 1)

struct virtio_net_request
{
#define VIRTIO_NET_REQUEST_SPLIT_NR 1
    /*
     * Layout:
     * +--------------------------+
     * | virtio_net_hdr + net MSS |
     * +--------------------------+
     */
    struct
    {
        struct virtio_net_hdr hdr;
        rt_uint8_t mss[VIRTIO_NET_MSS];
    } rx, tx;

    /* user data */
    struct rt_virtqueue *vq;
};

struct virtio_net
{
    struct eth_device parent;
    struct rt_virtio_device *vdev;

    rt_uint16_t status;

    struct rt_virtqueue *vqs[VIRTIO_NET_VQS_NR];

    int queue_idx;
    int queue_last_idx;
    int queue_size;
    rt_size_t virtq_nr;
    rt_size_t request_nr;
    struct virtio_net_request *request;
};

#define raw_to_virtio_net(raw) \
        rt_container_of(rt_container_of(raw, struct eth_device, parent), struct virtio_net, parent)

rt_inline void virtio_net_enqueue(struct virtio_net *vnet, struct rt_virtqueue *vq)
{
    struct virtio_net_request *request = &vnet->request[vnet->queue_idx];

    request->vq = vq;

    vnet->queue_idx++;
    vnet->queue_idx %= vnet->queue_size;
}

rt_inline struct rt_virtqueue *virtio_net_dequeue(struct virtio_net *vnet)
{
    struct rt_virtqueue *vq = RT_NULL;

    if (vnet->queue_last_idx != vnet->queue_idx)
    {
        vq = vnet->request[vnet->queue_last_idx].vq;

        vnet->queue_last_idx++;
        vnet->queue_last_idx %= vnet->queue_size;
    }

    return vq;
}

static rt_err_t virtio_net_tx(rt_device_t dev, struct pbuf *p)
{
    rt_base_t level;
    struct rt_virtqueue *vq = RT_NULL;
    struct virtio_net_request *request;
    struct virtio_net *vnet = raw_to_virtio_net(dev);

    for (;;)
    {
        level = rt_spin_lock_irqsave(&vnet->vdev->vq_lock);

        for (int i = 0; VIRTIO_NET_QUEUE_TX(i) < RT_ARRAY_SIZE(vnet->vqs); i += 2)
        {
            if (rt_virtqueue_prepare(vnet->vqs[VIRTIO_NET_QUEUE_TX(i)], 2))
            {
                vq = vnet->vqs[VIRTIO_NET_QUEUE_TX(i)];

                break;
            }
        }

        if (vq)
        {
            break;
        }

        rt_spin_unlock_irqrestore(&vnet->vdev->vq_lock, level);

        rt_thread_yield();
    }

    request = &vnet->request[((vq->index / 2) + VIRTIO_NET_QUEUE_TX(0)) * vnet->virtq_nr];
    request = &request[rt_virtqueue_next_buf_index(vq) / VIRTIO_NET_REQUEST_SPLIT_NR];
    request->tx.hdr.flags = 0;
    request->tx.hdr.gso_type = 0;
    request->tx.hdr.hdr_len = 0;
    request->tx.hdr.gso_size = 0;
    request->tx.hdr.csum_start = 0;
    request->tx.hdr.csum_offset = 0;
    request->tx.hdr.num_buffers = 0;

    RT_ASSERT(p->tot_len < sizeof(request->tx.mss));

    pbuf_copy_partial(p, &request->tx.mss, p->tot_len, 0);

    rt_virtqueue_add_outbuf(vq, &request->tx, sizeof(request->tx.hdr) + p->tot_len);

    rt_virtqueue_kick(vq);

    rt_spin_unlock_irqrestore(&vnet->vdev->vq_lock, level);

    return RT_EOK;
}

static struct pbuf *virtio_net_rx(rt_device_t dev)
{
    struct rt_virtqueue *vq;
    struct virtio_net_request *request;
    struct pbuf *p = RT_NULL, *new, *ret = RT_NULL;
    struct virtio_net *vnet = raw_to_virtio_net(dev);
    rt_ubase_t level = rt_spin_lock_irqsave(&vnet->vdev->vq_lock);

    while ((vq = virtio_net_dequeue(vnet)))
    {
        rt_size_t size;

        request = rt_virtqueue_read_buf(vq, &size);
        size -= sizeof(request->rx.hdr);

        rt_spin_unlock_irqrestore(&vnet->vdev->vq_lock, level);

        if (size > sizeof(request->rx.mss))
        {
            LOG_W("%s receive buffer's size = %u > %u is overflow",
                    rt_dm_dev_get_name(&vnet->parent.parent), size, sizeof(request->rx.mss));

            size = sizeof(request->rx.mss);
        }

        new = pbuf_alloc(PBUF_RAW, size, PBUF_RAM);

        if (p)
        {
            p->next = new;
            p = p->next;
        }
        else
        {
            p = new;
            ret = p;
        }

        if (!p)
        {
            goto _ret;
        }

        rt_memcpy(p->payload, &request->rx.mss, size);

        level = rt_spin_lock_irqsave(&vnet->vdev->vq_lock);

        rt_virtqueue_kick(vq);
    }

    rt_spin_unlock_irqrestore(&vnet->vdev->vq_lock, level);

_ret:
    return ret;
}

static rt_err_t virtio_net_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t err = RT_EOK;
    rt_uint8_t *mac;
    struct virtio_net *vnet = raw_to_virtio_net(dev);

    switch (cmd)
    {
    case NIOCTL_GADDR:
        mac = args;

        if (!mac)
        {
            err = -RT_EINVAL;
            break;
        }

        for (int i = 0; i < 6; ++i)
        {
            rt_virtio_read_config(vnet->vdev, struct virtio_net_config, mac[i], &mac[i]);
        }
        break;

    default:
        err = -RT_EINVAL;
        break;
    }

    return err;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops virtio_net_ops =
{
    .control = virtio_net_control,
};
#endif

static void virtio_net_rx_done(struct rt_virtqueue *vq)
{
    struct virtio_net *vnet = vq->vdev->priv;
    rt_ubase_t level = rt_spin_lock_irqsave(&vnet->vdev->vq_lock);

    virtio_net_enqueue(vnet, vq);

    rt_spin_unlock_irqrestore(&vnet->vdev->vq_lock, level);

    eth_device_ready(&vnet->parent);
}

static void virtio_net_tx_done(struct rt_virtqueue *vq)
{
    rt_virtqueue_read_buf(vq, RT_NULL);
}

static void virtio_net_config_changed(struct rt_virtio_device *vdev)
{
    rt_uint16_t status;
    rt_bool_t link_up;
    struct virtio_net *vnet = vdev->priv;

    if (!rt_virtio_has_feature(vdev, VIRTIO_NET_F_STATUS))
    {
        return;
    }

    rt_virtio_read_config(vdev, struct virtio_net_config, status, &status);

    /* Remove other status bit */
    status &= VIRTIO_NET_S_LINK_UP;

    if (vnet->status == status)
    {
        /* Status no change */
        return;
    }

    link_up = !!((vnet->status = status) & VIRTIO_NET_S_LINK_UP);

    LOG_D("%s linkchange to %s", rt_dm_dev_get_name(&vdev->parent), link_up ? "up" : "down");

    eth_device_linkchange(&vnet->parent, link_up);
}

static rt_err_t virtio_net_vq_init(struct virtio_net *vnet)
{
    rt_err_t err;
    rt_size_t vqs_nr = VIRTIO_NET_VQS_NR, fill_count;
    const char *names[VIRTIO_NET_VQS_NR];
    rt_virtqueue_callback cbs[VIRTIO_NET_VQS_NR];

    vnet->virtq_nr = rt_min(RT_CPUS_NR * 4, 1024);
    vnet->request_nr = (vnet->virtq_nr / VIRTIO_NET_REQUEST_SPLIT_NR) * vqs_nr;
    vnet->request = rt_malloc(sizeof(struct virtio_net_request) * vnet->request_nr);

    if (!vnet->request)
    {
        return -RT_ENOMEM;
    }

    for (int i = 0; VIRTIO_NET_QUEUE_TX(i) < vqs_nr; i += 2)
    {
        int rx = VIRTIO_NET_QUEUE_RX(i), tx = VIRTIO_NET_QUEUE_TX(i);

        names[rx] = "rx";
        names[tx] = "tx";

        cbs[rx] = &virtio_net_rx_done;
        cbs[tx] = &virtio_net_tx_done;

        rt_virtio_virtqueue_preset_max(&vnet->vqs[rx], vnet->virtq_nr);
        rt_virtio_virtqueue_preset_max(&vnet->vqs[tx], vnet->virtq_nr);
    }

    err = rt_virtio_virtqueue_install(vnet->vdev, vqs_nr, vnet->vqs, names, cbs);

    if (err)
    {
        return err;
    }

    fill_count = vnet->request_nr / vqs_nr;

    for (int i = 0; VIRTIO_NET_QUEUE_TX(i) < vqs_nr; i += 2)
    {
        struct virtio_net_request *request;
        struct rt_virtqueue *vq = vnet->vqs[VIRTIO_NET_QUEUE_RX(i)];

        request = &vnet->request[((vq->index / 2) + VIRTIO_NET_QUEUE_RX(0)) * vnet->virtq_nr];

        for (int idx = 0; idx < fill_count; ++idx)
        {
            rt_virtqueue_add_inbuf(vq, &request[idx].rx, sizeof(request->rx));

            rt_virtqueue_submit(vq);
        }

        rt_virtqueue_notify(vq);
    }

    return RT_EOK;
}

static void virtio_net_vq_finit(struct virtio_net *vnet)
{
    if (vnet->request)
    {
        if (vnet->vqs[0])
        {
            rt_virtio_virtqueue_release(vnet->vdev);
        }

        rt_free(vnet->request);
    }
}

static rt_err_t virtio_net_probe(struct rt_virtio_device *vdev)
{
    rt_err_t err = RT_EOK;
    struct virtio_net *vnet = rt_calloc(1, sizeof(*vnet));

    if (!vnet)
    {
        return -RT_ENOMEM;
    }

    vdev->priv = vnet;
    vnet->vdev = vdev;

#ifdef RT_USING_DEVICE_OPS
    vnet->parent.parent.ops = &virtio_net_ops;
#else
    vnet->parent.parent.control = virtio_net_control;
#endif
    vnet->parent.eth_tx = virtio_net_tx;
    vnet->parent.eth_rx = virtio_net_rx;

    if ((err = virtio_net_vq_init(vnet)))
    {
        goto _fail;
    }

    if ((err = rt_dm_dev_set_name_auto(&vnet->parent.parent, "e")) < 0)
    {
        goto _fail;
    }

    if ((err = eth_device_init(&vnet->parent, rt_dm_dev_get_name(&vnet->parent.parent))))
    {
        goto _fail;
    }

    eth_device_linkchange(&vnet->parent, RT_TRUE);

    return RT_EOK;

_fail:
    virtio_net_vq_finit(vnet);
    rt_free(vnet);

    return err;
}

static const struct rt_virtio_device_id virtio_net_ids[] =
{
    { VIRTIO_DEVICE_ID_NET, VIRTIO_DEVICE_ANY_ID },
    { /* sentinel */ }
};

static struct rt_virtio_driver virtio_net_driver =
{
    .ids = virtio_net_ids,
    .features =
        RT_BIT(VIRTIO_NET_F_MTU)
      | RT_BIT(VIRTIO_NET_F_MAC)
      | RT_BIT(VIRTIO_NET_F_MRG_RXBUF)
      | RT_BIT(VIRTIO_NET_F_STATUS)
      | RT_BIT(VIRTIO_NET_F_CTRL_RX)
      | RT_BIT(VIRTIO_NET_F_CTRL_VLAN)
      | RT_BIT(VIRTIO_NET_F_CTRL_RX_EXTRA)
      | RT_BIT(VIRTIO_NET_F_GUEST_ANNOUNCE)
      | RT_BIT(VIRTIO_NET_F_MQ)
      | RT_BIT(VIRTIO_NET_F_CTRL_MAC_ADDR)
      | RT_BIT(VIRTIO_NET_F_SPEED_DUPLEX)
      | RT_BIT(VIRTIO_F_ANY_LAYOUT)
      | RT_BIT(VIRTIO_F_RING_INDIRECT_DESC),

    .probe = virtio_net_probe,
    .config_changed = virtio_net_config_changed,
};
RT_VIRTIO_DRIVER_EXPORT(virtio_net_driver);
