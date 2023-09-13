/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     the first version
 */

#define DBG_TAG "rtdm.blk"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "blk_dm.h"

static void blk_remove_all(struct rt_blk_disk *disk)
{
    struct rt_blk_device *blk, *blk_next;

    /* Remove all partitions */
    rt_list_for_each_entry_safe(blk, blk_next, &disk->part_nodes, list)
    {
        disk_remove_blk_dev(blk, RT_TRUE);
    }
}

static rt_ssize_t blk_read(rt_device_t dev, rt_off_t sector,
        void *buffer, rt_size_t sector_count)
{
    rt_ssize_t res;
    struct rt_blk_disk *disk = to_blk_disk(dev);

    rt_sem_take(&disk->usr_lock, RT_WAITING_FOREVER);

    res = disk->ops->read(disk, sector, buffer, sector_count);

    rt_sem_release(&disk->usr_lock);

    return res;
}

static rt_ssize_t blk_write(rt_device_t dev, rt_off_t sector,
        const void *buffer, rt_size_t sector_count)
{
    rt_ssize_t res;
    struct rt_blk_disk *disk = to_blk_disk(dev);

    if (rt_unlikely(disk->read_only))
    {
        return -RT_ENOSYS;
    }

    rt_sem_take(&disk->usr_lock, RT_WAITING_FOREVER);

    res = disk->ops->write(disk, sector, buffer, sector_count);

    rt_sem_release(&disk->usr_lock);

    return res;
}

static rt_ssize_t blk_parallel_read(rt_device_t dev, rt_off_t sector,
        void *buffer, rt_size_t sector_count)
{
    struct rt_blk_disk *disk = to_blk_disk(dev);

    return disk->ops->read(disk, sector, buffer, sector_count);
}

static rt_ssize_t blk_parallel_write(rt_device_t dev, rt_off_t sector,
        const void *buffer, rt_size_t sector_count)
{
    struct rt_blk_disk *disk = to_blk_disk(dev);

    if (rt_unlikely(disk->read_only))
    {
        return -RT_ENOSYS;
    }

    return disk->ops->write(disk, sector, buffer, sector_count);
}

static rt_err_t blk_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t err;
    struct rt_blk_disk *disk = to_blk_disk(dev);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_BLK_GETGEOME:
        if (rt_unlikely(!args))
        {
            err = -RT_EINVAL;
            break;
        }

        err = disk->ops->getgeome(disk, args);
        break;

    case RT_DEVICE_CTRL_BLK_SYNC:
        if (disk->ops->sync)
        {
            rt_sem_take(&disk->usr_lock, RT_WAITING_FOREVER);

            spin_lock(&disk->lock);

            err = disk->ops->sync(disk);

            spin_unlock(&disk->lock);

            rt_sem_release(&disk->usr_lock);
        }
        else
        {
            err = -RT_ENOSYS;
        }
        break;

    case RT_DEVICE_CTRL_BLK_ERASE:
        if (disk->ops->erase)
        {
            rt_sem_take(&disk->usr_lock, RT_WAITING_FOREVER);

            spin_lock(&disk->lock);

            if (disk->parent.ref_count != 1)
            {
                err = -RT_EBUSY;
                goto _unlock;
            }

            blk_remove_all(disk);

            err = disk->ops->erase(disk);

        _unlock:
            spin_unlock(&disk->lock);

            rt_sem_release(&disk->usr_lock);
        }
        else
        {
            err = -RT_ENOSYS;
        }
        break;

    case RT_DEVICE_CTRL_BLK_AUTOREFRESH:
        if (disk->ops->autorefresh)
        {
            err = disk->ops->autorefresh(disk, !!args);
        }
        else
        {
            err = -RT_ENOSYS;
        }
        break;

    default:
        err = -RT_EINVAL;
        break;
    }

    return err;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops blk_ops =
{
    .read = blk_read,
    .write = blk_write,
    .control = blk_control,
};

const static struct rt_device_ops blk_parallel_ops =
{
    .read = blk_parallel_read,
    .write = blk_parallel_write,
    .control = blk_control,
};
#endif

rt_err_t rt_hw_blk_disk_register(struct rt_blk_disk *disk)
{
    rt_err_t err;
    const char *disk_name;
    rt_uint16_t flags = RT_DEVICE_FLAG_RDONLY;

    if (!disk || !disk->ops)
    {
        return -RT_EINVAL;
    }

#if RT_NAME_MAX > 0
    if (disk->parent.parent.name[0] == '\0')
#else
    if (disk->parent.parent.name)
#endif
    {
        return -RT_EINVAL;
    }

    disk_name = to_disk_name(disk);

    err = rt_sem_init(&disk->usr_lock, disk_name, 1, RT_IPC_FLAG_PRIO);

    if (err)
    {
        LOG_E("%s: Init user mutex error = %s", rt_strerror(err));

        return err;
    }

    rt_list_init(&disk->part_nodes);
    rt_spin_lock_init(&disk->lock);

    disk->parent.type = RT_Device_Class_Block;
#ifdef RT_USING_DEVICE_OPS
    if (disk->parallel_io)
    {
        disk->parent.ops = &blk_parallel_ops;
    }
    else
    {
        disk->parent.ops = &blk_ops;
    }
#else
    if (disk->parallel_io)
    {
        disk->parent.read = blk_parallel_read;
        disk->parent.write = blk_parallel_write;
    }
    else
    {
        disk->parent.read = blk_read;
        disk->parent.write = blk_write;
    }
    disk->parent.control = blk_control;
#endif

    if (!disk->ops->write)
    {
        disk->read_only = RT_TRUE;
    }

    if (!disk->read_only)
    {
        flags |= RT_DEVICE_FLAG_WRONLY;
    }

    err = rt_device_register(&disk->parent, disk_name, flags);

    if (err)
    {
        rt_sem_detach(&disk->usr_lock);
    }

    return err;
}

rt_err_t rt_hw_blk_disk_unregister(struct rt_blk_disk *disk)
{
    rt_err_t err;

    if (!disk)
    {
        return -RT_EINVAL;
    }

    spin_lock(&disk->lock);

    if (disk->parent.ref_count != 1)
    {
        err = -RT_EBUSY;
        goto _unlock;
    }

    /* Flush all data */
    if (disk->ops->sync)
    {
        err = disk->ops->sync(disk);

        if (err)
        {
            LOG_E("%s: Sync error = %s", to_disk_name(disk), rt_strerror(err));

            goto _unlock;
        }
    }

    rt_sem_detach(&disk->usr_lock);

    blk_remove_all(disk);

    err = rt_device_unregister(&disk->parent);

_unlock:
    spin_unlock(&disk->lock);

    return err;
}

rt_ssize_t rt_blk_disk_get_capacity(struct rt_blk_disk *disk)
{
    rt_ssize_t res;
    struct rt_device_blk_geometry geometry;

    if (!disk)
    {
        return -RT_EINVAL;
    }

    res = disk->ops->getgeome(disk, &geometry);

    if (!res)
    {
        return geometry.sector_count;
    }

    return res;
}

rt_ssize_t rt_blk_disk_get_logical_block_size(struct rt_blk_disk *disk)
{
    rt_ssize_t res;
    struct rt_device_blk_geometry geometry;

    if (!disk)
    {
        return -RT_EINVAL;
    }

    res = disk->ops->getgeome(disk, &geometry);

    if (!res)
    {
        return geometry.bytes_per_sector;
    }

    return res;
}
