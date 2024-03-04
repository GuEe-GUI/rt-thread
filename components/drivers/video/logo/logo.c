/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "video.logo"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static const rt_uint8_t startup_logo[] =
{
    #include "logo.inc"
};

static int startup_logo_width = __RT_THREAD_STARTUP_LOGO_WIDTH__;
static int startup_logo_height = __RT_THREAD_STARTUP_LOGO_HEIGHT__;
static int startup_logo_gray = __RT_THREAD_STARTUP_LOGO_GRAY__;

static int logo_rendering(void)
{
    rt_err_t err = RT_EOK;
    const rt_uint8_t *logo;
    rt_uint8_t *fb, alpha, bit32 = 0;
    int bpp, xlate, xoffset, yoffset;
    int red_id, green_id, blue_id, alpha_size, color_nr;
    rt_uint32_t red_off, green_off, blue_off;
    rt_uint32_t red_mask, green_mask, blue_mask;
    struct fb_info *info;
    struct fb_var_screeninfo var = {};
    struct fb_fix_screeninfo fix = {};
    rt_device_t fbdev = rt_device_find(RT_VIDEO_LOGO_FBDEV);

    if (!fbdev || (err = rt_device_open(fbdev, 0)))
    {
        return err ? : (int)-RT_EIO;
    }

    if ((err = rt_device_control(fbdev, FBIOGET_VSCREENINFO, &var)))
    {
        LOG_E("Get framebuffer %s information error = %s", "var", rt_strerror(err));

        goto _close_fbdev;
    }

    if (var.bits_per_pixel == 32)
    {
        bit32 = 1;
        alpha_size = 1;
    }
    else if (var.bits_per_pixel == 24)
    {
        bit32 = 1;
        alpha_size = 0;
    }
    else if (var.bits_per_pixel != 16)
    {
        LOG_E("Only supported 32, 24 or 16 bits");

        err = (int)-RT_EINVAL;
        goto _close_fbdev;
    }

    if ((err = rt_device_control(fbdev, FBIOGET_FSCREENINFO, &fix)))
    {
        LOG_E("Get framebuffer %s information error = %s", "fix", rt_strerror(err));

        goto _close_fbdev;
    }

    if (startup_logo_width > var.xres || startup_logo_height > var.yres)
    {
        LOG_E("PPM logo[%u, %u] Out of screen[%u, %u]",
                startup_logo_width, startup_logo_height, var.xres, var.yres);

        err = (int)-RT_EINVAL;
        goto _close_fbdev;
    }

    bpp = var.bits_per_pixel / 8;
    xlate = (var.xres - startup_logo_width) * bpp;
    xoffset = (var.xres - startup_logo_width) >> 1;
    yoffset = (var.yres - startup_logo_height) >> 1;

    info = fbdev->user_data;
    fb = (void *)info->screen_base;
    fb += xoffset * bpp + yoffset * fix.line_length;

    logo = startup_logo;

    red_off = var.red.offset;
    red_mask = RT_GENMASK(var.red.length, 0);
    green_off = var.green.offset;
    green_mask = RT_GENMASK(var.green.length, 0);
    blue_off = var.blue.offset;
    blue_mask = RT_GENMASK(var.blue.length, 0);

    alpha = (0xff & RT_GENMASK(var.transp.length, 0)) << var.transp.offset;

    if (!startup_logo_gray)
    {
        color_nr = 3;
        red_id = 0;
        green_id = 1;
        blue_id = 2;
    }
    else
    {
        color_nr = 1;
        red_id = green_id = blue_id = 0;
    }

    for (int dy = 0; dy < startup_logo_height; ++dy)
    {
        for (int dx = 0; dx < startup_logo_width; ++dx)
        {
            rt_uint32_t color = ((logo[red_id] & red_mask) << red_off) |
                                 ((logo[green_id] & green_mask) << green_off) |
                                 ((logo[blue_id] & blue_mask) << blue_off) |
                                 alpha;

            if (rt_unlikely(bit32))
            {
                *(rt_uint32_t *)fb = color;
                fb += alpha_size + 3;
            }
            else
            {
                *(rt_uint16_t *)fb = (rt_uint16_t)color;
                fb += 2;
            }

            logo += color_nr;
        }

        fb += xlate;
    }

    rt_device_control(fbdev, FBIO_WAITFORVSYNC, RT_NULL);

_close_fbdev:
    rt_device_close(fbdev);

    return (int)err;
}
#ifdef RT_VIDEO_LOGO_RENDERING_STAGE_DRIVER_EARLY
#define LOGO_RENDERING_STAGE INIT_DRIVER_EARLY_EXPORT
#endif
#ifdef RT_VIDEO_LOGO_RENDERING_STAGE_DRIVER
#define LOGO_RENDERING_STAGE INIT_DEVICE_EXPORT
#endif
#ifdef RT_VIDEO_LOGO_RENDERING_STAGE_DRIVER_LATER
#define LOGO_RENDERING_STAGE INIT_DRIVER_LATER_EXPORT
#endif
#ifdef RT_VIDEO_LOGO_RENDERING_STAGE_COMPONENT
#define LOGO_RENDERING_STAGE INIT_COMPONENT_EXPORT
#endif
#ifdef RT_VIDEO_LOGO_RENDERING_STAGE_ENV
#define LOGO_RENDERING_STAGE INIT_ENV_EXPORT
#endif
#ifdef RT_VIDEO_LOGO_RENDERING_STAGE_APP
#define LOGO_RENDERING_STAGE INIT_APP_EXPORT
#endif
LOGO_RENDERING_STAGE(logo_rendering);
