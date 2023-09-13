/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-3-08      GuEe-GUI     the first version
 * 2023-02-21     GuEe-GUI     move common init to setup
 */

#include <setup.h>
#include <board.h>

void rt_hw_board_init(void)
{
    rt_fdt_commit_memregion_early(&(rt_region_t)
    {
        .name = "memheap",
        .start = (rt_size_t)rt_kmem_v2p(HEAP_BEGIN),
        .end = (rt_size_t)rt_kmem_v2p(HEAP_END),
    }, RT_TRUE);

    rt_hw_common_setup();
}
