/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     first version
 */

#include "blk_dm.h"

static rt_err_t blk_dev_open(rt_device_t dev, rt_uint16_t oflag)
{
    struct rt_blk_device *blk = to_blk(dev);

    return rt_device_open(&blk->disk->parent, oflag);
}

static rt_err_t blk_dev_close(rt_device_t dev)
{
    struct rt_blk_device *blk = to_blk(dev);

    return rt_device_close(&blk->disk->parent);
}

static rt_ssize_t blk_dev_read(rt_device_t dev, rt_off_t sector,
        void *buffer, rt_size_t sector_count)
{
    struct rt_blk_device *blk = to_blk(dev);

    if (rt_unlikely(sector > blk->sector_start + blk->sector_count))
    {
        return -RT_EINVAL;
    }

    if (rt_unlikely(sector_count > blk->sector_count))
    {
        return -RT_EINVAL;
    }

    return rt_device_read(&blk->disk->parent,
            blk->sector_start + sector, buffer, sector_count);
}

static rt_ssize_t blk_dev_write(rt_device_t dev, rt_off_t sector,
        const void *buffer, rt_size_t sector_count)
{
    struct rt_blk_device *blk = to_blk(dev);

    if (rt_unlikely(sector > blk->sector_start + blk->sector_count))
    {
        return -RT_EINVAL;
    }

    if (rt_unlikely(sector_count > blk->sector_count))
    {
        return -RT_EINVAL;
    }

    return rt_device_write(&blk->disk->parent,
            blk->sector_start + sector, buffer, sector_count);
}

static rt_err_t blk_dev_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t err;
    struct rt_blk_device *blk = to_blk(dev);
    struct rt_device_blk_geometry disk_geometry, *geometry;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_BLK_GETGEOME:
        if (rt_unlikely(!(geometry = args)))
        {
            err = -RT_EINVAL;
            break;
        }

        err = blk->disk->ops->getgeome(blk->disk, &disk_geometry);

        if (rt_unlikely(err))
        {
            break;
        }

        geometry->bytes_per_sector = disk_geometry.bytes_per_sector;
        geometry->block_size = disk_geometry.block_size;
        geometry->sector_count = blk->sector_count;
        break;

    case RT_DEVICE_CTRL_BLK_SYNC:
        rt_device_control(&blk->disk->parent, cmd, args);
        break;

    case RT_DEVICE_CTRL_BLK_ERASE:
    case RT_DEVICE_CTRL_BLK_AUTOREFRESH:
        if (blk->disk->partitions <= 1)
        {
            rt_device_control(&blk->disk->parent, cmd, args);
        }
        else
        {
            err = -RT_EIO;
        }
        break;

    case RT_DEVICE_CTRL_BLK_PARTITION:
        if (rt_unlikely(!args))
        {
            err = -RT_EINVAL;
            break;
        }

        rt_memcpy(args, &blk->partition, sizeof(blk->partition));
        break;

    default:
        err = -RT_EINVAL;
        break;
    }

    return err;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops blk_dev_ops =
{
    .open = blk_dev_open,
    .close = blk_dev_close,
    .read = blk_dev_read,
    .write = blk_dev_write,
    .control = blk_dev_control,
};
#endif

rt_err_t blk_dev_initialize(struct rt_blk_device *blk)
{
    struct rt_device *dev;

    if (!blk)
    {
        return -RT_EINVAL;
    }

    dev = &blk->parent;
    dev->type = RT_Device_Class_Block;
#ifdef RT_USING_DEVICE_OPS
    dev->ops = &blk_dev_ops;
#else
    dev->open = blk_dev_open;
    dev->close = blk_dev_close;
    dev->read = blk_dev_read;
    dev->write = blk_dev_write;
    dev->control = blk_dev_control;
#endif

    return RT_EOK;
}

rt_err_t disk_add_blk_dev(struct rt_blk_disk *disk, struct rt_blk_device *blk)
{
    rt_err_t err;
    const char *disk_name, *name_fmt;

    if (!disk || !blk)
    {
        return -RT_EINVAL;
    }

    blk->disk = disk;
    rt_list_init(&blk->list);

    disk_name = to_disk_name(disk);

    /* End is [a-zA-Z] or [0-9] */
    if (disk_name[rt_strlen(disk_name) - 1] < 'a')
    {
        name_fmt = "%sp%d";
    }
    else
    {
        name_fmt = "%s%d";
    }

    rt_dm_dev_set_name(&blk->parent, name_fmt, disk_name, blk->partno);

    err = rt_device_register(&blk->parent, to_blk_name(blk),
            disk->parent.flag & RT_DEVICE_FLAG_RDWR);

    if (err)
    {
        return err;
    }

    spin_lock(&disk->lock);

    rt_list_insert_before(&disk->part_nodes, &blk->list);

    spin_unlock(&disk->lock);

    return RT_EOK;
}

rt_err_t disk_remove_blk_dev(struct rt_blk_device *blk, rt_bool_t lockless)
{
    struct rt_blk_disk *disk;

    if (!blk)
    {
        return -RT_EINVAL;
    }

    disk = blk->disk;

    if (!disk)
    {
        return -RT_EINVAL;
    }

    rt_device_unregister(&blk->parent);

    if (!lockless)
    {
        spin_lock(&disk->lock);
    }

    rt_list_remove(&blk->list);

    if (!lockless)
    {
        spin_unlock(&disk->lock);
    }

    return RT_EOK;
}
