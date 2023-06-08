/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020/10/7      bernard      the first version
 */

#include <rthw.h>
#include <rtthread.h>

int main(int argc, char** argv)
{
    rt_ubase_t level = rt_hw_interrupt_disable();

    rt_kprintf("Hi, this is RT-Thread!!\n");

    rt_hw_interrupt_enable(level);

    return 0;
}
