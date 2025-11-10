/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-11-25     GuEe-GUI     first version
 */

#ifndef __MACB_H__
#define __MACB_H__

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "../ethernet_dm.h"

struct macb_eth
{
    struct eth_device parent;
#ifdef RT_ETHERNET_CADENCE_PTP
    struct rt_ptp_clock ptp_parent;
#endif
    struct rt_device *dev;

    int irq;
    void *regs;

    struct rt_clk *pclk;
    struct rt_clk *hclk;

    rt_uint8_t mac[6];
};
#define raw_to_macb_eth(raw) \
    rt_container_of(rt_container_of(raw, struct eth_device, parent), struct macb_eth, parent)
#define raw_to_macb_ptp(raw) \
    rt_container_of(raw, struct macb_eth, ptp_parent)

rt_err_t macb_eth_common_probe(struct macb_eth *eth);
rt_err_t macb_eth_common_remove(struct macb_eth *eth);

#endif /* __MACB_H__ */
