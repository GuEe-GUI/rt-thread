/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-24     GuEe-GUI     first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#include <cpuport.h>

#define DBG_TAG "audio.hda"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "hda-defs.h"

#define ICH6_REG_BAR 0

#define CORB_ALIGNED 128

struct hda_bdl_entry
{
    rt_uint32_t paddr;
    rt_uint32_t length;
    rt_uint32_t flags;
} rt_packed;

struct hda_output
{
    rt_uint8_t codec;
    rt_uint16_t node_id;

    rt_uint32_t sample_rate;
    int amp_gain_steps;
    int num_channels;
};

struct intel_hda
{
    struct rt_audio_device parent;

    void *ich6_reg_base;
    struct rt_pci_device *pdev;

    int stream;

    struct hda_output output;

    void *ring_buffer;
    rt_uint32_t *corb;              /* command outbound ring buffer */
    rt_uint64_t *rirb;              /* response inbound ring buffer */
    struct hda_bdl_entry *bdl;      /* buffer descriptor list */
    rt_uint64_t *dma_pos;           /* dma position in current buffer */

    rt_size_t corb_entries;         /* number of CORB entries */
    rt_size_t rirb_entries;         /* number of RIRB entries */
    rt_uint16_t rirb_read_pointer;  /* RIRB read pointer */

    rt_uint32_t *buffer;
    rt_uint32_t buffers_completed;
};
#define raw_to_intel_hda(raw) rt_container_of(raw, struct intel_hda, parent)

#define intel_hda_writel(hda, reg, val)    HWREG32((hda)->ich6_reg_base + ICH6_REG_##reg) = (val)
#define intel_hda_writew(hda, reg, val)    HWREG16((hda)->ich6_reg_base + ICH6_REG_##reg) = (val)
#define intel_hda_writeb(hda, reg, val)    HWREG8((hda)->ich6_reg_base + ICH6_REG_##reg) = (val)

#define intel_hda_readl(hda, reg)          HWREG32((hda)->ich6_reg_base + ICH6_REG_##reg)
#define intel_hda_readw(hda, reg)          HWREG16((hda)->ich6_reg_base + ICH6_REG_##reg)
#define intel_hda_readb(hda, reg)          HWREG8((hda)->ich6_reg_base + ICH6_REG_##reg)

static rt_err_t intel_hda_audio_getcaps(struct rt_audio_device *audio,
        struct rt_audio_caps *caps)
{
    return RT_EOK;
}

static rt_err_t intel_hda_audio_configure(struct rt_audio_device *audio,
        struct rt_audio_caps *caps)
{
    return RT_EOK;
}

static rt_err_t intel_hda_audio_init(struct rt_audio_device *audio)
{
    return RT_EOK;
}

static rt_err_t intel_hda_audio_start(struct rt_audio_device *audio, int stream)
{
    return RT_EOK;
}

static rt_err_t intel_hda_audio_stop(struct rt_audio_device *audio, int stream)
{
    return RT_EOK;
}

static rt_ssize_t intel_hda_audio_transmit(struct rt_audio_device *audio,
        const void *write_buf, void *read_buf, rt_size_t size)
{
    return RT_EOK;
}

static void intel_hda_audio_buffer_info(struct rt_audio_device *audio,
        struct rt_audio_buf_info *info)
{
}

const static struct rt_audio_ops intel_hda_audio_ops =
{
    .getcaps = intel_hda_audio_getcaps,
    .configure = intel_hda_audio_configure,
    .init = intel_hda_audio_init,
    .start = intel_hda_audio_start,
    .stop = intel_hda_audio_stop,
    .transmit = intel_hda_audio_transmit,
    .buffer_info = intel_hda_audio_buffer_info,
};

static void intel_hda_isr(int irqno, void *param)
{
    rt_uint8_t sts;
    rt_uint32_t isr;
    struct intel_hda *hda = param;

    isr = intel_hda_readl(hda, INTSTS);
    sts = intel_hda_readb(hda, SDO0_CTLL);

    if (sts & SDO0_STS_BCIS)
    {
        if (hda->stream == AUDIO_STREAM_REPLAY)
        {
            rt_audio_tx_complete(&hda->parent);

            hda->buffers_completed++;
            hda->buffers_completed %= BDL_SIZE;
        }
        else if (hda->stream == AUDIO_STREAM_RECORD)
        {
            // rt_audio_rx_done();
        }
        else
        {
            LOG_E("Unknow stream = %d", hda->stream);
        }
    }

    /* Reset interrupt status registers */
    intel_hda_writel(hda, INTSTS, isr);
    intel_hda_writeb(hda, SDO0_CTLL, sts);
}

static void intel_hda_setup_corb(struct intel_hda *hda)
{

}

static void intel_hda_setup_rirb(struct intel_hda *hda)
{

}

static void intel_hda_reset(struct intel_hda *hda)
{
    /* Clear CORB/RIRB RUN bits before reset */
    intel_hda_writel(hda, CORBCTL, 0);
    intel_hda_writel(hda, RIRBCTL, 0);

    while ((intel_hda_readl(hda, CORBCTL) & ICH6_CORBCTL_RUN) ||
        (intel_hda_readl(hda, RIRBCTL) & ICH6_RBCTL_DMA_EN))
    {
        rt_hw_cpu_relax();
    }

    /* Reset the CRST bit and wait until hardware is in reset */
    intel_hda_writel(hda, GCTL, 0);

    while (intel_hda_readl(hda, GCTL) & ICH6_GCTL_RESET)
    {
        rt_hw_cpu_relax();
    }

    rt_hw_dmb();

    /* Take the hardware out of reset */
    intel_hda_writel(hda, GCTL, ICH6_GCTL_RESET);

    while ((intel_hda_readl(hda, GCTL) & ICH6_GCTL_RESET) == 0)
    {
        rt_hw_cpu_relax();
    }

    /* Enable all interrupts */
    intel_hda_writew(hda, WAKEEN, 0xffff);
    intel_hda_writel(hda, INTCTL, 0x800000ff);

    intel_hda_setup_corb(hda);
    intel_hda_setup_rirb(hda);

    /* Need to wait 25 frames (521 us) before enumerating codecs */
    rt_hw_us_delay(521);
}

static rt_err_t intel_hda_probe(struct rt_pci_device *pdev)
{
    rt_err_t err;
    const char *audio_name;
    char dev_name[RT_NAME_MAX];
    struct intel_hda *hda = rt_calloc(1, sizeof(*hda));

    if (!hda)
    {
        return -RT_ENOMEM;
    }

    hda->ich6_reg_base = rt_pci_iomap(pdev, ICH6_REG_BAR);

    if (!hda->ich6_reg_base)
    {
        err = -RT_EIO;

        goto _free_res;
    }

    // hda->ring_buffer = rt_malloc_align(, CORB_ALIGNED);

    intel_hda_reset(hda);

    rt_dm_dev_set_name_auto(&hda->parent.parent, "audio");
    audio_name = rt_dm_dev_get_name(&hda->parent.parent);

    hda->parent.ops = (struct rt_audio_ops *)&intel_hda_audio_ops;
    if ((err = rt_audio_register(&hda->parent, audio_name, RT_DEVICE_FLAG_RDWR, hda)))
    {
        goto _free_res;
    }

    pdev->parent.user_data = hda;

    rt_snprintf(dev_name, sizeof(dev_name), "%s-hda", audio_name);
    rt_hw_interrupt_install(pdev->irq, intel_hda_isr, hda, dev_name);
    rt_pci_irq_unmask(pdev);

    LOG_I("Intel HD Audio Controller v%d.%d (ich%s)",
            intel_hda_readb(hda, VMAJ), intel_hda_readb(hda, VMIN), pdev->id->data);

    return RT_EOK;

_free_res:
    if (hda->ich6_reg_base)
    {
        rt_iounmap(hda->ich6_reg_base);
    }

    rt_free(hda);

    return err;
}

static rt_err_t intel_hda_remove(struct rt_pci_device *pdev)
{
    struct intel_hda *hda = pdev->parent.user_data;

    rt_pci_irq_mask(pdev);
    rt_pic_detach_irq(pdev->irq, hda);

    rt_device_unregister(&hda->parent.parent);

    rt_iounmap(hda->ich6_reg_base);
    rt_free(hda);

    return RT_EOK;
}

static struct rt_pci_device_id intel_hda_pci_ids[] =
{
    { RT_PCI_DEVICE_ID(PCI_VENDOR_ID_INTEL, 0x2668), .data = "6" },
    { RT_PCI_DEVICE_ID(PCI_VENDOR_ID_INTEL, 0x293e), .data = "9" },
    { /* sentinel */ }
};

static struct rt_pci_driver intel_hda_driver =
{
    .name = "intel-hda",

    .ids = intel_hda_pci_ids,
    .probe = intel_hda_probe,
    .remove = intel_hda_remove,
};
RT_PCI_DRIVER_EXPORT(intel_hda_driver);
