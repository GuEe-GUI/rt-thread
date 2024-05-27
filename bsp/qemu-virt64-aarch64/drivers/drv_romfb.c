/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

static int romfb_init(void)
{
    rt_err_t err;
    struct fb_info *info;
    struct fb_var_screeninfo var = {};
    struct fb_fix_screeninfo fix = {};
    rt_device_t rom_fbdev = rt_device_find("fb0");

    if (!rom_fbdev || rt_device_open(rom_fbdev, 0))
    {
        return -RT_EIO;
    }

    var.xres = 800;
    var.yres = 600;
    var.xres_virtual = var.xres;
    var.yres_virtual = var.yres;
    var.bits_per_pixel = 32;
    var.red.offset = 16;
    var.red.length = 8;
    var.green.offset = 8;
    var.green.length = 8;
    var.blue.offset = 0;
    var.blue.length = 8;
    var.transp.offset = 24;
    var.transp.length = 8;

    if (!(err = rt_device_control(rom_fbdev, FBIOPUT_VSCREENINFO, &var)))
    {
        if (!(err = rt_device_control(rom_fbdev, FBIOGET_FSCREENINFO, &fix)))
        {
            info = rom_fbdev->user_data;

            rt_memset((void *)info->screen_base, 0, fix.smem_len);
        }
    }

    rt_device_close(rom_fbdev);

    return err;
}
INIT_SUBSYS_LATER_EXPORT(romfb_init);
