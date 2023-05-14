/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020/10/7      bernard      the first version
 */

#include <stdio.h>
#include <rtthread.h>
#include "drivers/core/bus.h"

#ifdef RT_USING_FDT
#include "dtb_node.h"
#endif

struct rt_device timer_device = {
    .name = "timer_virt64",
};

// static void timer_init(void)
// {
//     rt_device_attach(&timer_device);
// }

int main(void)
{
    /* example: init timer device statically */
    // timer_init();

    rt_uint8_t times = 0;

    printf("hello rt-thread\n");

    /*test timer*/
    while (times++ < 2)
    {
        rt_thread_mdelay(500);
        printf("hello rt-thread\n");
    }

    return 0;
}
