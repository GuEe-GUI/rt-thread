/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-25     GuEe-GUI     first version
 */

#include "macb.h"

#define DBG_TAG "eth.macb"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static rt_err_t macb_eth_tx(rt_device_t dev, struct pbuf *p)
{

}

static struct pbuf *macb_eth_rx(rt_device_t dev)
{

}

static rt_err_t macb_eth_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t err = RT_EOK;
    struct macb_eth *eth = raw_to_macb_eth(dev);

    switch (cmd)
    {
    case NIOCTL_GADDR:
        if (args)
        {
            rt_memcpy(args, eth->mac, sizeof(eth->mac));
        }
        else
        {
            err = -RT_EINVAL;
        }
        break;

    default:
        err = -RT_EINVAL;
        break;
    }

    return err;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops macb_eth_ops =
{
    .control = macb_eth_control,
};
#endif

static void macb_eth_isr(int irq, void *param)
{

}

#if xx
ethernet@100000 {
    reg = <0xc0 0x40100000 0x00 0x4000>;
    compatible = "cdns,macb";
    #address-cells = <0x01>;
    #size-cells = <0x00>;
    interrupts = <0x06 0x04>;
    clocks = <0x49 0x4a 0x33 0x1d>;
    clock-names = "pclk","hclk","tsu_clk";
    phy-mode = "rgmii-id";
    cdns,aw2w-max-pipe = [08];
    cdns,ar2r-max-pipe = [08];
    cdns,use-aw2b-fill;
    local-mac-address = [d8 3a dd c4 d3 0b];
    status = "okay";
    phy-handle = <0x4b>;
    phy-reset-gpios = <0x38 0x20 0x01>;
    phy-reset-duration = <0x05>;
    phandle = <0xeb>;

    ethernet-phy@1 {
        reg = <0x01>;
        brcm,powerdown-enable;
        phandle = <0x4b>;
    };
};
#endif

rt_err_t macb_eth_common_probe(struct macb_eth *eth)
{
    rt_err_t err;

#ifdef RT_USING_OFW
    // if (rt_ofw_get_mac_addr(eth->dev->ofw_node, eth->mac))
#endif
    {
        ethernet_random_addr(&eth->parent, eth->mac);
    }

#ifdef RT_USING_DEVICE_OPS
    eth->parent.parent.ops = &macb_eth_ops;
#else
    eth->parent.parent.control = macb_eth_control;
#endif
    eth->parent.eth_tx = macb_eth_tx;
    eth->parent.eth_rx = macb_eth_rx;

    if ((err = rt_dm_dev_set_name_auto(&eth->parent.parent, "e")) < 0)
    {
        goto _fail;
    }

    if ((err = eth_device_init(&eth->parent, rt_dm_dev_get_name(&eth->parent.parent))))
    {
        goto _fail;
    }

    eth_device_linkchange(&eth->parent, RT_TRUE);

    return RT_EOK;

_fail:
    return err;
}

rt_err_t macb_eth_common_remove(struct macb_eth *eth)
{

}

static rt_err_t macb_eth_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    struct rt_device *dev = &pdev->parent;
    struct macb_eth *eth = rt_calloc(1, sizeof(*eth));

    if (!eth)
    {
        return -RT_ENOMEM;
    }

    eth->dev = dev;

    // if ((err = macb_eth_common_probe(eth)))
    // {
    //     goto _fail;
    // }

    return RT_EOK;

_fail:
    rt_free(eth);

    return err;
}

static rt_err_t macb_eth_remove(struct rt_platform_device *pdev)
{

}

static const struct rt_ofw_node_id macb_eth_ofw_ids[] =
{
    { .compatible = "cdns,macb" },
    // { .compatible = "cdns,gem", .data = &pc302gem_config },
    // { .compatible = "cdns,emac", .data = &emac_config },
    // { .compatible = "cdns,at91sam9260-macb", .data = &at91sam9260_config },
    // { .compatible = "cdns,np4-macb", .data = &np4_config },
    // { .compatible = "cdns,pc302-gem", .data = &pc302gem_config },
    // { .compatible = "cdns,sam9x60-macb", .data = &at91sam9260_config },
    // { .compatible = "atmel,sama5d2-gem", .data = &sama5d2_config },
    // { .compatible = "atmel,sama5d29-gem", .data = &sama5d29_config },
    // { .compatible = "atmel,sama5d3-gem", .data = &sama5d3_config },
    // { .compatible = "atmel,sama5d3-macb", .data = &sama5d3macb_config },
    // { .compatible = "atmel,sama5d4-gem", .data = &sama5d4_config },
    // { .compatible = "cdns,at91rm9200-emac", .data = &emac_config },
    // { .compatible = "cdns,zynqmp-gem", .data = &zynqmp_config},
    // { .compatible = "cdns,zynq-gem", .data = &zynq_config },
    // { .compatible = "sifive,fu540-c000-gem", .data = &fu540_c000_config },
    // { .compatible = "microchip,mpfs-macb", .data = &mpfs_config },
    // { .compatible = "microchip,sama7g5-gem", .data = &sama7g5_gem_config },
    // { .compatible = "microchip,sama7g5-emac", .data = &sama7g5_emac_config },
    // { .compatible = "xlnx,zynqmp-gem", .data = &zynqmp_config},
    // { .compatible = "xlnx,zynq-gem", .data = &zynq_config },
    // { .compatible = "xlnx,versal-gem", .data = &versal_config},
    { /* sentinel */ }
};

static struct rt_platform_driver macb_eth_driver =
{
    .name = "eth-macb",
    .ids = macb_eth_ofw_ids,

    .probe = macb_eth_probe,
    .remove = macb_eth_remove,
};
RT_PLATFORM_DRIVER_EXPORT(macb_eth_driver);
