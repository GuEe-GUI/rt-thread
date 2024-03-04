/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-01     GuEe-GUI     first version
 */

#ifndef __DFS_DIRECTFS_H__
#define __DFS_DIRECTFS_H__

#include <rtdef.h>

#include <dfs.h>
#include <dfs_fs.h>
#include <dfs_file.h>

struct rt_varea;

struct directfs_attribute
{
    const char *name;
    rt_uint32_t mode;
};

struct directfs_bin_attribute
{
    struct directfs_attribute attr;
    rt_size_t size;

    rt_ssize_t (*read)(rt_object_t, struct directfs_bin_attribute *, char *, rt_off_t, rt_size_t);
    rt_ssize_t (*write)(rt_object_t, struct directfs_bin_attribute *, char *, rt_off_t, rt_size_t);
    rt_err_t (*mmap2)(rt_object_t, struct directfs_bin_attribute *, struct dfs_mmap2_args *);
};

rt_object_t dfs_directfs_find_object(rt_object_t parent, const char *path);
rt_err_t dfs_directfs_create_link(rt_object_t parent, rt_object_t object, const char *name);
rt_err_t dfs_directfs_destroy_link(rt_object_t object);
rt_err_t dfs_directfs_create_bin_file(rt_object_t object, struct directfs_bin_attribute *attr);

int dfs_directfs_init(void);

#endif /* __DFS_SYSFS_H__ */
