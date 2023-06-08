/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-9-16      GuEe-GUI     the first version
 * 2021-11-11     GuEe-GUI     modify to virtio common interface
 * 2023-02-25     GuEe-GUI     support DM
 */

#ifndef __VIRTIO_H__
#define __VIRTIO_H__

#include <mm_page.h>

#include <drivers/core/rtdm.h>
#include <drivers/core/device.h>
#include <drivers/core/driver.h>

#define VIRTIO_STATUS_ACKNOWLEDGE   RT_BIT(0)
#define VIRTIO_STATUS_DRIVER        RT_BIT(1)
#define VIRTIO_STATUS_DRIVER_OK     RT_BIT(2)
#define VIRTIO_STATUS_FEATURES_OK   RT_BIT(3)
#define VIRTIO_STATUS_NEEDS_RESET   RT_BIT(6)
#define VIRTIO_STATUS_FAILED        RT_BIT(7)

#define VIRTIO_F_NOTIFY_ON_EMPTY    24
#define VIRTIO_F_ANY_LAYOUT         27
#define VIRTIO_F_RING_INDIRECT_DESC 28
#define VIRTIO_F_RING_EVENT_IDX     29
#define VIRTIO_F_VERSION_1          32
#define VIRTIO_F_ACCESS_PLATFORM    33
#define VIRTIO_F_RING_PACKED        34
#define VIRTIO_F_IN_ORDER           35
#define VIRTIO_F_ORDER_PLATFORM     36
#define VIRTIO_F_SR_IOV             37
#define VIRTIO_F_NOTIFICATION_DATA  38
#define VIRTIO_F_NOTIF_CONFIG_DATA  39
#define VIRTIO_F_RING_RESET         40

#define VIRTIO_TRANSPORT_F_START    VIRTIO_F_RING_INDIRECT_DESC
#define VIRTIO_TRANSPORT_F_END      VIRTIO_F_RING_RESET

enum
{
    /* virtio 1.0 */
    VIRTIO_DEVICE_ID_INVALID        = 0,    /* Invalid device */
    VIRTIO_DEVICE_ID_NET            = 1,    /* Net */
    VIRTIO_DEVICE_ID_BLOCK          = 2,    /* Block */
    VIRTIO_DEVICE_ID_CONSOLE        = 3,    /* Console */
    VIRTIO_DEVICE_ID_RNG            = 4,    /* Rng */
    VIRTIO_DEVICE_ID_BALLOON        = 5,    /* Balloon */
    VIRTIO_DEVICE_ID_IOMEM          = 6,    /* IO memory */
    VIRTIO_DEVICE_ID_RPMSG          = 7,    /* Remote processor messaging */
    VIRTIO_DEVICE_ID_SCSI           = 8,    /* SCSI */
    VIRTIO_DEVICE_ID_9P             = 9,    /* 9p console */
    VIRTIO_DEVICE_ID_MAC80211_WLAN  = 10,   /* Mac80211 wlan */
    VIRTIO_DEVICE_ID_RPROC_SERIAL   = 11,   /* Remoteproc serial link */
    VIRTIO_DEVICE_ID_CAIF           = 12,   /* CAIF */
    VIRTIO_DEVICE_ID_MEM_BALLOON    = 13,   /* Memory balloon */
    VIRTIO_DEVICE_ID_GPU            = 16,   /* GPU */
    VIRTIO_DEVICE_ID_TIME           = 17,   /* Timer/clock device */
    VIRTIO_DEVICE_ID_INPUT          = 18,   /* Input */
    /* virtio 1.1 */
    VIRTIO_DEVICE_ID_SOCKET         = 19,   /* Socket device */
    VIRTIO_DEVICE_ID_CRYPTO         = 20,   /* Crypto device */
    VIRTIO_DEVICE_ID_SIG_DIS_MOD    = 21,   /* Signal Distribution Module */
    VIRTIO_DEVICE_ID_PSTORE         = 22,   /* Pstore device */
    VIRTIO_DEVICE_ID_IOMMU          = 23,   /* IOMMU device */
    VIRTIO_DEVICE_ID_MEM            = 24,   /* Memory device */
    /* virtio 1.2 */
    VIRTIO_DEVICE_ID_AUDIO          = 25,   /* Audio device */
    VIRTIO_DEVICE_ID_FS             = 26,   /* File system device */
    VIRTIO_DEVICE_ID_PMEM           = 27,   /* PMEM device */
    VIRTIO_DEVICE_ID_RPMB           = 28,   /* RPMB device */
    VIRTIO_DEVICE_ID_MAC80211_HWSIM = 29,   /* Mac80211 hwsim wireless simulation device */
    VIRTIO_DEVICE_ID_VIDEO_ENCODER  = 30,   /* Video encoder device */
    VIRTIO_DEVICE_ID_VIDEO_DECODER  = 31,   /* Video decoder device */
    VIRTIO_DEVICE_ID_SCMI           = 32,   /* SCMI device */
    VIRTIO_DEVICE_ID_NITRO_SEC_MOD  = 33,   /* NitroSecureModule */
    VIRTIO_DEVICE_ID_I2C_ADAPTER    = 34,   /* I2C adapter */
    VIRTIO_DEVICE_ID_WATCHDOG       = 35,   /* Watchdog */
    VIRTIO_DEVICE_ID_CAN            = 36,   /* CAN device */
    VIRTIO_DEVICE_ID_DMABUF         = 37,   /* Virtio dmabuf */
    VIRTIO_DEVICE_ID_PARAM_SERV     = 38,   /* Parameter Server */
    VIRTIO_DEVICE_ID_AUDIO_POLICY   = 39,   /* Audio policy device */
    VIRTIO_DEVICE_ID_BT             = 40,   /* Bluetooth device */
    VIRTIO_DEVICE_ID_GPIO           = 41,   /* GPIO device */
    VIRTIO_DEVICE_ID_RDMA           = 42,   /* RDMA device */

    VIRTIO_DEVICE_ID_MAX
};

enum
{
    VIRTIO_PCI_SUBSYSTEM_DEVICE_ID_NET      = 0x1000,  /* Network card */
    VIRTIO_PCI_SUBSYSTEM_DEVICE_ID_BLOCK    = 0x1001,  /* Block device */
    VIRTIO_PCI_SUBSYSTEM_DEVICE_ID_BALLOON  = 0x1002,  /* Memory ballooning (traditional) */
    VIRTIO_PCI_SUBSYSTEM_DEVICE_ID_CONSOLE  = 0x1003,  /* Console */
    VIRTIO_PCI_SUBSYSTEM_DEVICE_ID_SCSI     = 0x1004,  /* SCSI host */
    VIRTIO_PCI_SUBSYSTEM_DEVICE_ID_RNG      = 0x1005,  /* Entropy source */
    VIRTIO_PCI_SUBSYSTEM_DEVICE_ID_9P       = 0x1009,  /* 9P transport */

    VIRTIO_PCI_SUBSYSTEM_DEVICE_ID_MAX
};

struct rt_virtio_device_id
{
#define VIRTIO_DEVICE_ANY_ID 0xffffffff
    rt_uint32_t device;
    rt_uint32_t vendor;
};

struct rt_virtqueue;
struct rt_virtio_transport;

typedef rt_bool_t (*rt_virtqueue_notifier)(struct rt_virtqueue *vq);
typedef void (*rt_virtqueue_callback)(struct rt_virtqueue *vq);

struct rt_virtio_device
{
    struct rt_device parent;

    int idx;
    struct rt_virtio_device_id id;

    rt_uint64_t features;
    const struct rt_virtio_transport *trans;

    rt_list_t vq_node;
    struct rt_spinlock vq_lock;

    void *priv;
};

struct virtqueue_split;
struct virtqueue_packed;

struct rt_virtqueue
{
    rt_list_t list;

    const char *name;
    rt_uint32_t index;
    rt_uint32_t num_max;
    rt_uint32_t num_free;
    void **buffer_hash;

    rt_virtqueue_callback callback;

    union
    {
        struct virtqueue_split *vq_split;
        struct virtqueue_packed *vq_packed;
    };
    rt_bool_t packed_ring;

    rt_virtqueue_notifier notify;

    rt_bool_t event;
    rt_bool_t event_triggered;
    rt_uint32_t free_head;
    rt_uint32_t num_added;
    rt_uint16_t last_used_idx;

    struct rt_virtio_device *vdev;
};

struct rt_virtio_shm_region
{
    rt_uint8_t id;

    rt_uint64_t base;
    rt_uint64_t size;
};

#define RT_VIRTQUEUE_CTL_IRQ_AFFINITY       0
#define RT_VIRTQUEUE_CTL_READ_SHM_REGION    1

struct rt_virtio_transport
{
    rt_err_t (*get_status)(struct rt_virtio_device *vdev, rt_uint8_t *out_status);
    rt_err_t (*set_status)(struct rt_virtio_device *vdev, rt_uint8_t status);
    rt_err_t (*get_features)(struct rt_virtio_device *vdev, rt_uint64_t *out_features);
    rt_err_t (*set_features)(struct rt_virtio_device *vdev);
    rt_err_t (*get_config)(struct rt_virtio_device *vdev, rt_uint32_t offset, void *dst, int length);
    rt_err_t (*set_config)(struct rt_virtio_device *vdev, rt_uint32_t offset, const void *src, int length);
    rt_err_t (*install_vqs)(struct rt_virtio_device *vdev, int vqs_nr,
                           struct rt_virtqueue *vqs[], const char *names[], rt_virtqueue_callback cbs[]);
    rt_err_t (*release_vqs)(struct rt_virtio_device *vdev);
    rt_err_t (*control_vqs)(struct rt_virtio_device *vdev, rt_uint32_t cfg, void *data);
    rt_err_t (*generation)(struct rt_virtio_device *vdev, rt_uint32_t *out_counter);
    rt_err_t (*reset)(struct rt_virtio_device *vdev);
};

struct rt_virtio_driver
{
    struct rt_driver parent;

    const struct rt_virtio_device_id *ids;

    rt_uint64_t features;

    rt_err_t (*probe)(struct rt_virtio_device *vdev);
    void (*config_changed)(struct rt_virtio_device *vdev);
};

struct rt_virtqueue *rt_virtqueue_create(struct rt_virtio_device *vdev, const char *name, int index,
        int num, rt_uint32_t align, rt_virtqueue_notifier notify, rt_virtqueue_callback callback, void *ext_cfg);

rt_err_t rt_virtqueue_delete(struct rt_virtio_device *vdev, struct rt_virtqueue *vq);

rt_err_t rt_virtqueue_add_outbuf(struct rt_virtqueue *vq, void *dma_buf, rt_size_t size);
rt_err_t rt_virtqueue_add_inbuf(struct rt_virtqueue *vq, void *dma_buf, rt_size_t size);

rt_bool_t rt_virtqueue_prepare(struct rt_virtqueue *vq, rt_uint32_t nr);
rt_uint32_t rt_virtqueue_next_buf_index(struct rt_virtqueue *vq);

rt_bool_t rt_virtqueue_submit(struct rt_virtqueue *vq);
rt_bool_t rt_virtqueue_notify(struct rt_virtqueue *vq);
rt_bool_t rt_virtqueue_kick(struct rt_virtqueue *vq);

void rt_virtqueue_isr(int irq, struct rt_virtqueue *vq);
rt_bool_t rt_virtqueue_poll(struct rt_virtqueue *vq, rt_uint32_t last_used_idx);

void rt_virtqueue_disable_callback(struct rt_virtqueue *vq);
rt_bool_t rt_virtqueue_enable_callback(struct rt_virtqueue *vq, rt_uint32_t *out_last_used_idx);

void *rt_virtqueue_read_buf(struct rt_virtqueue *vq, rt_size_t *out_len);

rt_size_t rt_virtqueue_get_virtq_size(struct rt_virtqueue *vq);
rt_ubase_t rt_virtqueue_get_desc_addr(struct rt_virtqueue *vq);
rt_ubase_t rt_virtqueue_get_avail_addr(struct rt_virtqueue *vq);
rt_ubase_t rt_virtqueue_get_used_addr(struct rt_virtqueue *vq);

void rt_virtio_device_ready(struct rt_virtio_device *vdev);
void rt_virtio_device_reset(struct rt_virtio_device *vdev);
void rt_virtio_device_config_changed(struct rt_virtio_device *vdev);

void rt_virtio_add_status(struct rt_virtio_device *vdev, rt_uint32_t status);

rt_bool_t rt_virtio_has_status(struct rt_virtio_device *vdev, rt_uint8_t status_bit);
rt_bool_t rt_virtio_has_feature(struct rt_virtio_device *vdev, rt_uint32_t feature_bit);

rt_err_t rt_virtio_get_config(struct rt_virtio_device *vdev, rt_uint32_t offset, void *dst, int length);
rt_err_t rt_virtio_set_config(struct rt_virtio_device *vdev, rt_uint32_t offset, const void *src, int length);

#define rt_virtio_read_config(vdev, type, member, out_value) \
    rt_virtio_get_config(vdev, (rt_ubase_t)&((type *)0)->member, out_value, sizeof(((type *)0)->member))

#define rt_virtio_write_config(vdev, type, member, in_value) \
    rt_virtio_set_config(vdev, (rt_ubase_t)&((type *)0)->member, in_value, sizeof(((type *)0)->member))

#define rt_virtio_virtqueue_preset_max(vq_ptr, num_max) ({ *vq_ptr = (void *)num_max; })
#define rt_virtio_virtqueue_preget_max(vq_ptr) ({ (rt_uint32_t)(rt_ubase_t)((void *)*vq_ptr); })

rt_err_t rt_virtio_virtqueue_install(struct rt_virtio_device *vdev, int vqs_nr,
        struct rt_virtqueue *vqs[], const char *names[], rt_virtqueue_callback cbs[]);
rt_err_t rt_virtio_virtqueue_release(struct rt_virtio_device *vdev);
rt_err_t rt_virtio_virtqueue_control(struct rt_virtio_device *vdev, rt_uint32_t cfg, void *data);

const char *rt_virtio_device_id_name(struct rt_virtio_device *vdev);

rt_err_t rt_virtio_driver_register(struct rt_virtio_driver *vdrv);
rt_err_t rt_virtio_device_register(struct rt_virtio_device *vdev);

#define RT_VIRTIO_DRIVER_EXPORT(driver)  RT_DRIVER_EXPORT(driver, virtio, BUILIN)

#endif /* __VIRTIO_H__ */
