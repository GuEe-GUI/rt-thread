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

#define DBG_TAG "eth.macb.pci"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define GEM_PCLK_RATE 50000000
#define GEM_HCLK_RATE 50000000

static struct rt_clk_fixed_rate (*macb_pci_clk)[2];

static rt_err_t macb_pci_clk_init(void)
{
    rt_err_t err;

    if (macb_pci_clk)
    {
        return RT_EOK;
    }

    macb_pci_clk = rt_calloc(2, sizeof(struct rt_clk_fixed_rate));

    if (!macb_pci_clk)
    {
        return -RT_ENOMEM;
    }

    macb_pci_clk[0]->clk.name = "macb_pci_pclk";
    macb_pci_clk[0]->clk.rate = GEM_PCLK_RATE;
    macb_pci_clk[0]->clk.min_rate = GEM_PCLK_RATE;
    macb_pci_clk[0]->clk.max_rate = GEM_PCLK_RATE;
    macb_pci_clk[0]->fixed_rate = GEM_PCLK_RATE;

    macb_pci_clk[1]->clk.name = "macb_pci_hclk";
    macb_pci_clk[1]->clk.rate = GEM_PCLK_RATE;
    macb_pci_clk[1]->clk.min_rate = GEM_PCLK_RATE;
    macb_pci_clk[1]->clk.max_rate = GEM_PCLK_RATE;
    macb_pci_clk[1]->fixed_rate = GEM_PCLK_RATE;

    if ((err = rt_clk_register(&macb_pci_clk[0]->clk, RT_NULL)))
    {
        goto _fail;
    }

    if ((err = rt_clk_register(&macb_pci_clk[1]->clk, RT_NULL)))
    {
        goto _unregister_pclk;
    }

    return RT_EOK;

_unregister_pclk:
    rt_clk_unregister(&macb_pci_clk[1]->clk);

_fail:
    rt_free(macb_pci_clk);

    macb_pci_clk = RT_NULL;

    return err;
}

static rt_err_t macb_pci_probe(struct rt_pci_device *pdev)
{
    rt_err_t err;
    struct macb_eth *eth;

    if ((err = macb_pci_clk_init()))
    {
        return err;
    }

    if (!(eth = rt_calloc(1, sizeof(*eth))))
    {
        return -RT_ENOMEM;
    }

    rt_pci_set_master(pdev);

    eth->dev = &pdev->parent;

    eth->regs = rt_pci_iomap(pdev, 0);

    eth->irq = pdev->irq;
    rt_pci_irq_unmask(pdev);

    eth->pclk = rt_clk_get_by_name(eth->dev, "macb_pci_pclk");

    if (rt_is_err_or_null(eth->pclk))
    {
        if (eth->pclk)
        {
            err = rt_ptr_err(eth->pclk);
        }
        else
        {
            err = -RT_EIO;
        }

        goto _fail;
    }

    eth->hclk = rt_clk_get_by_name(eth->dev, "macb_pci_hclk");

    if (rt_is_err_or_null(eth->hclk))
    {
        if (eth->hclk)
        {
            err = rt_ptr_err(eth->hclk);
        }
        else
        {
            err = -RT_EIO;
        }

        goto _fail;
    }

    if ((err = macb_eth_common_probe(eth)))
    {
        goto _fail;
    }

    return RT_EOK;

_fail:
    if (eth->regs)
    {
        rt_iounmap(eth);
    }

    rt_pci_irq_mask(pdev);

    rt_clk_put(eth->pclk);
    rt_clk_put(eth->hclk);

    rt_free(eth);

    return err;
}

static rt_err_t macb_pci_remove(struct rt_pci_device *pdev)
{
    struct macb_eth *eth = pdev->parent.user_data;

    /* INTx is shared, don't mask all */
    rt_hw_interrupt_umask(pdev->irq);
    rt_pci_irq_mask(pdev);

    macb_eth_common_remove(eth);

    rt_pci_clear_master(pdev);

    rt_iounmap(eth->regs);
    rt_free(eth);

    return RT_EOK;
}

static const struct rt_pci_device_id macb_pci_ids[] =
{
    { RT_PCI_DEVICE_ID(PCI_VENDOR_ID_CDNS, 0xe007), },
    { /* sentinel */ }
};

static struct rt_pci_driver macb_pci_driver =
{
    .name = "macb-pci",

    .ids = macb_pci_ids,
    .probe = macb_pci_probe,
    .remove = macb_pci_remove,
};
RT_PCI_DRIVER_EXPORT(macb_pci_driver);
