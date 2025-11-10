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

#include "virtio_config/virtio-sound.h"

#define QUEUE_CONTROL   0
#define QUEUE_EVENT     1
#define QUEUE_TX        2
#define QUEUE_RX        3

struct virtio_sound_jack
{
    rt_uint32_t nid;
    rt_uint32_t features;
    rt_uint32_t defconf;
    rt_uint32_t caps;
    rt_bool_t connected;
    int type;
};

struct virtio_sound_rate
{
    rt_uint32_t alsa_bit;
    rt_uint32_t rate;
};

struct virtio_sound
{
    struct rt_audio_device parent;

    struct rt_virtio_device *vdev;

    struct rt_virtqueue *vqs[4];

    struct virtio_snd_event *event_msgs;

    rt_uint32_t jacks_nr;
    rt_uint32_t chmaps_nr;
    rt_uint32_t substreams_nr;
    struct virtio_sound_jack *jacks;
    struct virtio_pcm_substream *substreams;
    struct virtio_snd_chmap_info *chmaps;

    struct rt_spinlock tx_lock;
    struct rt_spinlock rx_lock;
};

#define raw_to_virtio_sound(raw) rt_container_of(raw, struct virtio_sound, parent)

// static const rt_uint32_t format_map[] =
// {
//     [VIRTIO_SND_PCM_FMT_IMA_ADPCM] = SNDRV_PCM_FORMAT_IMA_ADPCM,
//     [VIRTIO_SND_PCM_FMT_MU_LAW] = SNDRV_PCM_FORMAT_MU_LAW,
//     [VIRTIO_SND_PCM_FMT_A_LAW] = SNDRV_PCM_FORMAT_A_LAW,
//     [VIRTIO_SND_PCM_FMT_S8] = SNDRV_PCM_FORMAT_S8,
//     [VIRTIO_SND_PCM_FMT_U8] = SNDRV_PCM_FORMAT_U8,
//     [VIRTIO_SND_PCM_FMT_S16] = SNDRV_PCM_FORMAT_S16_LE,
//     [VIRTIO_SND_PCM_FMT_U16] = SNDRV_PCM_FORMAT_U16_LE,
//     [VIRTIO_SND_PCM_FMT_S18_3] = SNDRV_PCM_FORMAT_S18_3LE,
//     [VIRTIO_SND_PCM_FMT_U18_3] = SNDRV_PCM_FORMAT_U18_3LE,
//     [VIRTIO_SND_PCM_FMT_S20_3] = SNDRV_PCM_FORMAT_S20_3LE,
//     [VIRTIO_SND_PCM_FMT_U20_3] = SNDRV_PCM_FORMAT_U20_3LE,
//     [VIRTIO_SND_PCM_FMT_S24_3] = SNDRV_PCM_FORMAT_S24_3LE,
//     [VIRTIO_SND_PCM_FMT_U24_3] = SNDRV_PCM_FORMAT_U24_3LE,
//     [VIRTIO_SND_PCM_FMT_S20] = SNDRV_PCM_FORMAT_S20_LE,
//     [VIRTIO_SND_PCM_FMT_U20] = SNDRV_PCM_FORMAT_U20_LE,
//     [VIRTIO_SND_PCM_FMT_S24] = SNDRV_PCM_FORMAT_S24_LE,
//     [VIRTIO_SND_PCM_FMT_U24] = SNDRV_PCM_FORMAT_U24_LE,
//     [VIRTIO_SND_PCM_FMT_S32] = SNDRV_PCM_FORMAT_S32_LE,
//     [VIRTIO_SND_PCM_FMT_U32] = SNDRV_PCM_FORMAT_U32_LE,
//     [VIRTIO_SND_PCM_FMT_FLOAT] = SNDRV_PCM_FORMAT_FLOAT_LE,
//     [VIRTIO_SND_PCM_FMT_FLOAT64] = SNDRV_PCM_FORMAT_FLOAT64_LE,
//     [VIRTIO_SND_PCM_FMT_DSD_U8] = SNDRV_PCM_FORMAT_DSD_U8,
//     [VIRTIO_SND_PCM_FMT_DSD_U16] = SNDRV_PCM_FORMAT_DSD_U16_LE,
//     [VIRTIO_SND_PCM_FMT_DSD_U32] = SNDRV_PCM_FORMAT_DSD_U32_LE,
//     [VIRTIO_SND_PCM_FMT_IEC958_SUBFRAME] = SNDRV_PCM_FORMAT_IEC958_SUBFRAME_LE
// };

static const struct virtio_sound_rate rate_map[] =
{
    /* VIRTIO_SND_PCM_RATE_5512 */
    [VIRTIO_SND_PCM_RATE_8000] = { AUDIO_SAMP_RATE_8K, 8000 },
    [VIRTIO_SND_PCM_RATE_11025] = { AUDIO_SAMP_RATE_11K, 11025 },
    [VIRTIO_SND_PCM_RATE_16000] = { AUDIO_SAMP_RATE_16K, 16000 },
    [VIRTIO_SND_PCM_RATE_22050] = { AUDIO_SAMP_RATE_22K, 22050 },
    [VIRTIO_SND_PCM_RATE_32000] = { AUDIO_SAMP_RATE_32K, 32000 },
    [VIRTIO_SND_PCM_RATE_44100] = { AUDIO_SAMP_RATE_44K, 44100 },
    [VIRTIO_SND_PCM_RATE_48000] = { AUDIO_SAMP_RATE_48K, 48000 },
    /* VIRTIO_SND_PCM_RATE_64000 */
    /* VIRTIO_SND_PCM_RATE_88200 */
    [VIRTIO_SND_PCM_RATE_96000] = { AUDIO_SAMP_RATE_96K, 96000 },
    /* VIRTIO_SND_PCM_RATE_176400 */
    [VIRTIO_SND_PCM_RATE_192000] = { AUDIO_SAMP_RATE_192K, 192000 }
};

static rt_err_t virtio_sound_audio_getcaps(struct rt_audio_device *audio,
        struct rt_audio_caps *caps)
{
    struct virtio_sound *vsound = raw_to_virtio_sound(audio);

    switch (caps->main_type)
    {
    case AUDIO_TYPE_OUTPUT:
    case AUDIO_TYPE_INPUT:
        switch(caps->sub_type)
        {
        // case AUDIO_DSP_PARAM:
        //     caps->udata.config.channels = i2s->audio_config.channels;
        //     caps->udata.config.samplebits = i2s->audio_config.samplebits;
        //     caps->udata.config.samplerate = i2s->audio_config.samplerate;
        //     break;

        // case AUDIO_DSP_SAMPLERATE:
        //     caps->udata.config.samplerate = i2s->audio_config.samplerate;
        //     break;

        // case AUDIO_DSP_CHANNELS:
        //     caps->udata.config.channels = i2s->audio_config.channels;
        //     break;

        // case AUDIO_DSP_SAMPLEBITS:
        //     caps->udata.config.samplebits = i2s->audio_config.samplebits;
        //     break;

        default:
            return -RT_ENOSYS;
        }
        break;

    case AUDIO_TYPE_MIXER:
        break;

    default:
        return -RT_ENOSYS;
    }

    return RT_EOK;

    return RT_EOK;
}

static rt_err_t virtio_sound_audio_configure(struct rt_audio_device *audio,
        struct rt_audio_caps *caps)
{
    return RT_EOK;
}

static rt_err_t virtio_sound_audio_init(struct rt_audio_device *audio)
{
    return RT_EOK;
}

static rt_err_t virtio_sound_audio_start(struct rt_audio_device *audio, int stream)
{
    return RT_EOK;
}

static rt_err_t virtio_sound_audio_stop(struct rt_audio_device *audio, int stream)
{
    return RT_EOK;
}

static rt_ssize_t virtio_sound_audio_transmit(struct rt_audio_device *audio,
        const void *write_buf, void *read_buf, rt_size_t size)
{
    return RT_EOK;
}

static void virtio_sound_audio_buffer_info(struct rt_audio_device *audio,
        struct rt_audio_buf_info *info)
{
}

const static struct rt_audio_ops virtio_sound_audio_ops =
{
    .getcaps = virtio_sound_audio_getcaps,
    .configure = virtio_sound_audio_configure,
    .init = virtio_sound_audio_init,
    .start = virtio_sound_audio_start,
    .stop = virtio_sound_audio_stop,
    .transmit = virtio_sound_audio_transmit,
    .buffer_info = virtio_sound_audio_buffer_info,
};

static void virtio_sound_control_done(struct rt_virtqueue *vq)
{

}

static void virtio_sound_event_done(struct rt_virtqueue *vq)
{
}

static void virtio_sound_tx_done(struct rt_virtqueue *vq)
{
    // rt_audio_tx_complete(&hda->parent);
}

static void virtio_sound_rx_done(struct rt_virtqueue *vq)
{
}

static rt_err_t virtio_sound_vq_init(struct virtio_sound *vsound)
{
    const char *names[] =
    {
        "control",
        "event",
        "tx",
        "rx",
    };
    rt_virtqueue_callback cbs[] =
    {
        &virtio_sound_control_done,
        &virtio_sound_event_done,
        &virtio_sound_tx_done,
        &virtio_sound_rx_done,
    };

    return rt_virtio_virtqueue_install(vsound->vdev,
            RT_ARRAY_SIZE(names), vsound->vqs, names, cbs);
}

static void virtio_sound_vq_finit(struct virtio_sound *vsound)
{
    if (vsound->vqs[0])
    {
        rt_virtio_virtqueue_release(vsound->vdev);
    }
}

static rt_err_t virtio_sound_probe(struct rt_virtio_device *vdev)
{
    rt_err_t err;
    const char *audio_name;
    struct virtio_sound *vsound = rt_calloc(1, sizeof(*vsound));

    if (!vsound)
    {
        return -RT_ENOMEM;
    }

    vdev->priv = vsound;
    vsound->vdev = vdev;
    vdev->parent.user_data = vsound;

    if ((err = virtio_sound_vq_init(vsound)) < 0)
    {
        goto _fail;
    }

    rt_virtio_read_config(vdev, struct virtio_snd_config, jacks, &vsound->jacks_nr);
    rt_virtio_read_config(vdev, struct virtio_snd_config, streams, &vsound->substreams_nr);
    rt_virtio_read_config(vdev, struct virtio_snd_config, chmaps, &vsound->chmaps_nr);

    rt_spin_lock_init(&vsound->tx_lock);
    rt_spin_lock_init(&vsound->rx_lock);

    rt_dm_dev_set_name_auto(&vsound->parent.parent, "sound");
    audio_name = rt_dm_dev_get_name(&vsound->parent.parent);

    vsound->parent.ops = (struct rt_audio_ops *)&virtio_sound_audio_ops;

    if ((err = rt_audio_register(&vsound->parent, audio_name, RT_DEVICE_FLAG_RDWR, vsound)))
    {
        goto _fail;
    }

    return RT_EOK;

_fail:
    virtio_sound_vq_finit(vsound);
    rt_free(vsound);

    return err;
}

static rt_err_t virtio_sound_remove(struct rt_virtio_device *vdev)
{
    struct virtio_sound *vsound = vdev->parent.user_data;

    rt_device_unregister(&vsound->parent.parent);
    virtio_sound_vq_finit(vsound);
    rt_free(vsound);

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
        RT_BIT(VIRTIO_SOUND_F_CTLS)
      | RT_BIT(VIRTIO_F_ANY_LAYOUT),

    .probe = virtio_sound_probe,
    .remove = virtio_sound_remove,
};
RT_VIRTIO_DRIVER_EXPORT(virtio_sound_driver);
