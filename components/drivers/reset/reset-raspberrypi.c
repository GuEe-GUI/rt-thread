/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-26     GuEe-GUI     first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "reset.brcmstb"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <dt-bindings/reset/raspberrypi,firmware-reset.h>
#include "../firmware/raspberrypi/firmware.h"

struct rpi_reset
{
    struct rt_reset_controller parent;

    struct rpi_firmware *rpi_fw;
};

#define raw_to_rpi_reset(raw) rt_container_of(raw, struct rpi_reset, parent)

static rt_err_t rpi_reset_reset(struct rt_reset_control *rstc)
{
    rt_err_t err = RT_EOK;
    rt_uint32_t dev_addr;
    struct rpi_reset *rrst = rstc->rstcer->priv;

    switch (rstc->id) {
    case RASPBERRYPI_FIRMWARE_RESET_ID_USB:
        /*
         * The Raspberry Pi 4 gets its USB functionality from VL805, a
         * PCIe chip that implements xHCI. After a PCI reset, VL805's
         * firmware may either be loaded directly from an EEPROM or, if
         * not present, by the SoC's co-processor, VideoCore. rpi's
         * VideoCore OS contains both the non public firmware load
         * logic and the VL805 firmware blob. This triggers the
         * aforementioned process.
         *
         * The pci device address is expected is expected by the
         * firmware encoded like this:
         *
         *  PCI_BUS << 20 | PCI_SLOT << 15 | PCI_FUNC << 12
         *
         * But since rpi's PCIe is hardwired, we know the address in
         * advance.
         */
        dev_addr = 0x100000;

        if ((err = rpi_firmware_property(rrst->rpi_fw, RPI_FIRMWARE_NOTIFY_XHCI_RESET,
                &dev_addr, sizeof(dev_addr))))
        {
            break;
        }

        /* Wait for vl805 to startup */
        rt_hw_us_delay(500);
        break;

    default:
        err = -RT_EINVAL;
        break;
    }

    return err;
}

const static struct rt_reset_control_ops rpi_reset_ops =
{
    .reset = rpi_reset_reset,
};

static rt_err_t rpi_reset_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    struct rt_reset_controller *rstcer;
    struct rt_ofw_node *np = pdev->parent.ofw_node, *fw_np;
    struct rpi_reset *rrst = rt_calloc(1, sizeof(*rrst));

    if (!rrst)
    {
        return -RT_ENOMEM;
    }

    fw_np = rt_ofw_parse_phandle(np, "firmware", 0);

    if (!fw_np)
    {
        err = -RT_EINVAL;
        goto _fail;
    }

    rrst->rpi_fw = rpi_firmware_get(fw_np);
    rt_ofw_node_put(fw_np);

    if (!rrst->rpi_fw)
    {
        err = -RT_EINVAL;
        goto _fail;
    }

    rstcer = &rrst->parent;

    rstcer->priv = rrst;
    rstcer->ofw_node = np;
    rstcer->ops = &rpi_reset_ops;

    if ((err = rt_reset_controller_register(rstcer)))
    {
        goto _fail;
    }

    return RT_EOK;

_fail:
    rt_free(rrst);

    return err;
}

static const struct rt_ofw_node_id rpi_reset_ofw_ids[] =
{
    { .compatible = "raspberrypi,firmware-reset" },
    { /* sentinel */ }
};

static struct rt_platform_driver rpi_reset_driver =
{
    .name = "reset-raspberrypi",
    .ids = rpi_reset_ofw_ids,

    .probe = rpi_reset_probe,
};

static int rpi_reset_register(void)
{
    rt_platform_driver_register(&rpi_reset_driver);

    return 0;
}
INIT_SUBSYS_EXPORT(rpi_reset_register);
