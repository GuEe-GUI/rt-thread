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

#define DBG_TAG "virtio.dev.sound"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static rt_err_t virtio_sound_probe(struct rt_virtio_device *vdev)
{
    return RT_EOK;
}

static const struct rt_virtio_device_id virtio_sound_ids[] =
{
    { VIRTIO_DEVICE_ID_AUDIO, VIRTIO_DEVICE_ANY_ID },
    { /* sentinel */ }
};

static struct rt_virtio_driver virtio_sound_driver =
{
    .ids = virtio_sound_ids,
    .features =
        RT_BIT(VIRTIO_F_ANY_LAYOUT)
      | RT_BIT(VIRTIO_F_RING_INDIRECT_DESC),

    .probe = virtio_sound_probe,
};
RT_VIRTIO_DRIVER_EXPORT(virtio_sound_driver);
