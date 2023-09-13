/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     first version
 */

#ifndef __BLK_DM_H__
#define __BLK_DM_H__

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define to_blk_disk(dev)    rt_container_of(dev, struct rt_blk_disk, parent)
#define to_blk(dev)         rt_container_of(dev, struct rt_blk_device, parent)

#define to_disk_name(disk)  rt_dm_dev_get_name(&disk->parent)
#define to_blk_name(blk)    rt_dm_dev_get_name(&blk->parent)

rt_inline void spin_lock(struct rt_spinlock *spinlock)
{
    rt_hw_spin_lock(&spinlock->lock);
}

rt_inline void spin_unlock(struct rt_spinlock *spinlock)
{
    rt_hw_spin_unlock(&spinlock->lock);
}

rt_err_t blk_dev_initialize(struct rt_blk_device *blk);
rt_err_t disk_add_blk_dev(struct rt_blk_disk *disk, struct rt_blk_device *blk);
rt_err_t disk_remove_blk_dev(struct rt_blk_device *blk, rt_bool_t lockless);

#endif /* __BLK_DM_H__ */
