/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-07     GuEe-GUI     first version
 */

#include "virtio_pci_legacy.h"

static rt_err_t virtio_pci_legacy_probe(struct rt_pci_device *pdev)
{
    rt_err_t err = RT_EOK;

    return err;
}

static struct rt_pci_device_id virtio_pci_legacy_ids[VIRTIO_PCI_DEVICE_END - VIRTIO_PCI_DEVICE_START] =
{
};

static struct rt_pci_driver virtio_pci_legacy_driver =
{
    .name = "virtio-pci-legacy",

    .ids = virtio_pci_legacy_ids,
    .probe = virtio_pci_legacy_probe,
};

static int virtio_pci_legacy_driver_register(void)
{
    rt_uint16_t id = VIRTIO_PCI_DEVICE_START;

    for (int i = 0; i < RT_ARRAY_SIZE(virtio_pci_legacy_ids); ++i, ++id)
    {
        virtio_pci_legacy_ids[i] = (struct rt_pci_device_id)
        {
            RT_PCI_DEVICE_ID(PCI_VENDOR_ID_REDHAT_QUMRANET, id)
        };
    }

    rt_pci_driver_register(&virtio_pci_legacy_driver);

    return 0;
}
INIT_DEVICE_EXPORT(virtio_pci_legacy_driver_register);
