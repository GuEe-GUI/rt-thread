/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-13     ErikChan      the first version
 */

#include <rtthread.h>
#include <virtio_console.h>
#include <virt.h>
#include <drivers/core/bus.h>

#ifdef RT_USING_FDT
#include "dtb_node.h"
#endif

int virtio_mmio_probe(struct rt_device *dev)
{
    struct virtio_mmio_config *mmio_config;

    struct dtb_node *node = (struct dtb_node *)(dev->dtb_node);
    if (node)
    {
        size_t uart_reg_range = 0;
        void *uart_reg_addr = (void *)dtb_node_get_addr_size(node, "reg", &uart_reg_range);

        rt_ubase_t hw_base;
        if ((uart_reg_addr) && (uart_reg_range != 0))
        {
            hw_base = (rt_base_t)rt_ioremap(uart_reg_addr, uart_reg_range);
        }

        mmio_config = (struct virtio_mmio_config *)hw_base;

        if (mmio_config->magic != VIRTIO_MAGIC_VALUE ||
            mmio_config->version != RT_USING_VIRTIO_VERSION ||
            mmio_config->vendor_id != VIRTIO_VENDOR_ID)
        {
            return 0;
        }

        struct hw_virtio_device *virtio_device_priv = (struct hw_virtio_device *)(dev->user_data);
        rt_uint32_t device_id = virtio_device_priv->id;

        switch (device_id)
        {
        case VIRTIO_DEVICE_ID_CONSOLE:
        {
            /* Create vport0p1 */
            struct virtio_console_device *virtio_console_dev = (struct virtio_console_device *)(dev);
            virtio_console_port_create(virtio_console_dev);
        }
        break;
        default:
            break;
        }
    }

    return 0;
}

struct rt_device_id virtio_mmio_ids[] =
    {
        {.compatible = "virtio,mmio"},
        {/* sentinel */}};

static struct rt_virtio_driver virtio_mmio_drv = {
    .parent = {
        .name = "virtio_mmio",
        .probe = virtio_mmio_probe,
        .ids = virtio_mmio_ids,
    },
};

int virtio_drv_init(void)
{
    struct rt_bus *virtio_bus = rt_bus_find_by_name("virtio");
    virtio_mmio_drv.parent.bus = virtio_bus;

    rt_virtio_driver_register(&virtio_mmio_drv);
    return 0;
}

INIT_ENV_EXPORT(virtio_drv_init);
