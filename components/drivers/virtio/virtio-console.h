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

#ifndef __VIRTIO_CONSOLE_H__
#define __VIRTIO_CONSOLE_H__

#define VIRTIO_CONSOLE_F_SIZE           0   /* Does host provide console size? */
#define VIRTIO_CONSOLE_F_MULTIPORT      1   /* Does host provide multiple ports? */
#define VIRTIO_CONSOLE_F_EMERG_WRITE    2   /* Does host support emergency write? */

struct virtio_console_config
{
    rt_uint16_t cols;
    rt_uint16_t rows;
    rt_uint32_t max_nr_ports;
    rt_uint32_t emerg_wr;
} __attribute__((packed));

struct virtio_console_control
{
#define VIRTIO_CONSOLE_BAD_ID       (~(rt_uint32_t)0)
    rt_uint32_t id;     /* Port number */

#define VIRTIO_CONSOLE_DEVICE_READY 0
#define VIRTIO_CONSOLE_PORT_ADD     1
#define VIRTIO_CONSOLE_PORT_REMOVE  2
#define VIRTIO_CONSOLE_PORT_READY   3
#define VIRTIO_CONSOLE_CONSOLE_PORT 4
#define VIRTIO_CONSOLE_RESIZE       5
#define VIRTIO_CONSOLE_PORT_OPEN    6
#define VIRTIO_CONSOLE_PORT_NAME    7
    rt_uint16_t event;  /* The kind of control event */
    rt_uint16_t value;  /* Extra information for the event */
};

struct virtio_console_resize
{
    rt_uint16_t cols;
    rt_uint16_t rows;
};

#endif /* __VIRTIO_CONSOLE_H__ */
