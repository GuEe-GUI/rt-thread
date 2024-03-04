/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-01     GuEe-GUI     first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtservice.h>

#include "dfs_directfs.h"

static struct rt_spinlock _directfs_lock = { 0 };
static struct rt_object _directfs_root =
{
    .child_nodes = RT_LIST_OBJECT_INIT(_directfs_root.child_nodes),
};
static struct directfs_bin_attribute _directfs_default_attr =
{
    .attr =
    {
        .name = RT_NULL,
    },
};

rt_inline void directfs_lock(void)
{
    rt_hw_spin_lock(&_directfs_lock.lock);
}

rt_inline void directfs_unlock(void)
{
    rt_hw_spin_unlock(&_directfs_lock.lock);
}

rt_object_t dfs_directfs_find_object(rt_object_t parent, const char *path)
{
    rt_object_t obj = RT_NULL, obj_tmp;
    rt_size_t path_part_len = 0;
    const char *split;

    if (!path)
    {
        return obj;
    }

    parent = parent ? : &_directfs_root;
    for (split = path; *split && *split != '/'; ++split)
    {
    }
    path_part_len = split - path;

    directfs_lock();

    rt_list_for_each_entry(obj_tmp, &parent->child_nodes, node)
    {
        if (rt_strlen(obj_tmp->name) == path_part_len &&
            !rt_strncmp(obj_tmp->name, path, path_part_len))
        {
            obj = obj_tmp;
            break;
        }
    }

    directfs_unlock();

    path += path_part_len;

    return obj ? (*path == '/' ? dfs_directfs_find_object(obj, path + 1) : obj) : RT_NULL;
}

rt_err_t dfs_directfs_create_link(rt_object_t parent, rt_object_t object, const char *name)
{
    if (!object || !name)
    {
        return -RT_EINVAL;
    }

    parent = parent ? : &_directfs_root;

    rt_strncpy(object->name, name, RT_NAME_MAX);

    object->parent = parent;
    object->objectfs = &_directfs_default_attr;

    rt_list_init(&object->node);
    rt_list_init(&object->child_nodes);

    directfs_lock();

    rt_list_insert_before(&parent->child_nodes, &object->node);

    directfs_unlock();

    return RT_EOK;
}

rt_err_t dfs_directfs_destroy_link(rt_object_t object)
{
    rt_err_t err;

    directfs_lock();

    if (rt_list_isempty(&object->child_nodes))
    {
        rt_list_remove(&object->node);

        err = RT_EOK;
    }
    else
    {
        err = -RT_EBUSY;
    }

    directfs_unlock();

    return err;
}

rt_err_t dfs_directfs_create_bin_file(rt_object_t object, struct directfs_bin_attribute *attr)
{
    if (!object || !attr)
    {
        return -RT_EINVAL;
    }

    object->objectfs = attr;

    directfs_lock();

    rt_list_remove(&object->node);
    rt_list_insert_after(&object->parent->child_nodes, &object->node);

    directfs_unlock();

    return RT_EOK;
}

static int dfs_directfs_open(struct dfs_file *file)
{
    rt_object_t obj = RT_NULL;

    if (rt_strcmp(file->vnode->path, "/"))
    {
        struct directfs_bin_attribute *battr;
        obj = dfs_directfs_find_object(RT_NULL, file->vnode->path + 1);

        if (!obj)
        {
            return -EIO;
        }

        battr = obj->objectfs;

        if ((file->flags & O_DIRECTORY) && battr->attr.name)
        {
            return -ENOENT;
        }

        file->vnode->size = battr->attr.name ? 0 : rt_list_len(&obj->child_nodes);
    }
    else
    {
        file->vnode->size = rt_list_len(&_directfs_root.child_nodes);
    }

    file->vnode->data = obj;
    file->pos = 0;

    return RT_EOK;
}

static int dfs_directfs_close(struct dfs_file *file)
{
    int ret = -EIO;
    rt_object_t obj = file->vnode->data;

    file->vnode->data = RT_NULL;

    if (obj)
    {
        ret = 0;
    }

    return ret;
}

static int dfs_directfs_ioctl(struct dfs_file *file, int cmd, void *args)
{
    int ret = 0;
    rt_object_t obj = file->vnode->data;
    struct directfs_bin_attribute *battr = obj->objectfs;

    switch (cmd)
    {
    case RT_FIOMMAP2:
        if (battr->mmap2 && args)
        {
            ret = (int)battr->mmap2(obj, battr, args);
        }
        break;

    default:
        ret = -RT_EIO;
        break;
    }

    return ret;
}

static int dfs_directfs_read(struct dfs_file *file, void *buf, size_t count)
{
    int ret = 0;
    rt_object_t obj = file->vnode->data;
    struct directfs_bin_attribute *battr = obj->objectfs;

    if (battr->read && count > 0)
    {
        ret = battr->read(obj, battr, buf, file->pos, count);

        if (ret > 0)
        {
            /* update file current position */
            file->pos += ret;
        }
    }

    return ret;
}

static int dfs_directfs_write(struct dfs_file *file, const void *buf, size_t count)
{
    int ret = 0;
    rt_object_t obj = file->vnode->data;
    struct directfs_bin_attribute *battr = obj->objectfs;

    if (battr->write && count > 0)
    {
        ret = battr->write(obj, battr, (void *)buf, file->pos, count);

        if (ret > 0)
        {
            /* update file current position */
            file->pos += ret;
        }
    }

    return ret;
}

static int dfs_directfs_lseek(struct dfs_file *file, off_t offset)
{
    int ret = -EIO;

    if (offset <= file->vnode->size)
    {
        file->pos = offset;
        ret = file->pos;
    }

    return ret;
}

static int dfs_directfs_getdents(struct dfs_file *file, struct dirent *dirp, uint32_t count)
{
    int ret = -EIO;
    rt_list_t *root = &_directfs_root.child_nodes;
    rt_object_t obj = file->vnode->data, child;

    /* make integer count */
    count = (count / sizeof(struct dirent));

    if (obj)
    {
        root = &obj->child_nodes;
    }

    if (count)
    {
        struct dirent *d;
        rt_size_t index = 0, end = file->pos + count, count = 0;

        rt_spin_lock(&_directfs_lock);

        rt_list_for_each_entry(child, root, node)
        {
            if (index >= (rt_size_t)file->pos)
            {
                struct directfs_bin_attribute *battr = child->objectfs;

                d = dirp + count;

                /* fill dirent */
                if (!battr->attr.name)
                {
                    d->d_type = DT_DIR;
                }
                else
                {
                    d->d_type = DT_REG;
                }

                d->d_namlen = RT_NAME_MAX;
                d->d_reclen = (rt_uint16_t)sizeof(struct dirent);
                rt_strncpy(d->d_name, child->name, RT_NAME_MAX);

                ++count;

                /* move to next position */
                ++file->pos;
            }

            ++index;

            if (index >= end)
            {
                break;
            }
        }

        rt_spin_unlock(&_directfs_lock);

        ret = count * sizeof(struct dirent);
    }
    else
    {
        ret = -EINVAL;
    }

    return ret;
}

static const struct dfs_file_ops _directfs_fops =
{
    .open       = dfs_directfs_open,
    .close      = dfs_directfs_close,
    .ioctl      = dfs_directfs_ioctl,
    .read       = dfs_directfs_read,
    .write      = dfs_directfs_write,
    .lseek      = dfs_directfs_lseek,
    .getdents   = dfs_directfs_getdents,
};

static int dfs_directfs_mount(struct dfs_filesystem *fs, unsigned long rwflag, const void *data)
{
    return RT_EOK;
}

static int dfs_directfs_unmount(struct dfs_filesystem *fs)
{
    return RT_EOK;
}

static int dfs_directfs_stat(struct dfs_filesystem *fs, const char *path, struct stat *st)
{
    rt_size_t size = 0;
    rt_object_t obj = dfs_directfs_find_object(RT_NULL, path + 1);
    struct directfs_bin_attribute *battr = obj->objectfs;

    st->st_dev = 0;
    st->st_mode = S_IFREG | S_IRUSR | S_IRGRP | S_IROTH |
                  S_IWUSR | S_IWGRP | S_IWOTH;

    if (!battr->attr.name)
    {
        st->st_mode &= ~S_IFREG;
        st->st_mode |= S_IFDIR | S_IXUSR | S_IXGRP | S_IXOTH;
    }
    else
    {
        size = rt_list_len(&obj->child_nodes);
    }

    st->st_size = size;
    st->st_mtime = 0;

    return RT_EOK;
}

static const struct dfs_filesystem_ops _directfs =
{
    .name       = "direct",
    .flags      = DFS_FS_FLAG_DEFAULT,
    .fops       = &_directfs_fops,

    .mount      = dfs_directfs_mount,
    .unmount    = dfs_directfs_unmount,
    .stat       = dfs_directfs_stat,
};

int dfs_directfs_init(void)
{
    /* register direct file system */
    return dfs_register(&_directfs);
}
INIT_COMPONENT_EXPORT(dfs_directfs_init);
