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

#include <mmu.h>

#define DBG_TAG "fb.float"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/*
 * Float framebuffer device driver
 *
 * Device Tree node, eg.:
 *
 *      framebuffer {
 *          compatible = "rt-thread,float-fb";
 *          reg = <...>;
 *          format = "...";
 *          offset-x = <X>;
 *          offset-y = <Y>;
 *          stride = <(BPP / 8 * WIDTH)>;
 *          height = <WIDTH>;
 *          width = <HEIGH>;
 *      };
 */

struct floatfb_format
{
    const char *name;
    rt_uint32_t mode;
    rt_uint32_t bits_per_pixel;
};

struct floatfb
{
    struct rt_graphic_device parent;

    void *peer_fb;
    void *render_fb;
    void *screen_base;
    rt_size_t screen_size;
    rt_uint32_t render_stride;

    rt_uint32_t offset_x;
    rt_uint32_t offset_y;
    rt_uint32_t stride;
    rt_uint32_t height;
    rt_uint32_t width;
};

static struct floatfb_format floatfb_formats[] =
{
    { "r8g8b8",   RTGRAPHIC_PIXEL_FORMAT_RGB888 , 24 },
    { "x8r8g8b8", RTGRAPHIC_PIXEL_FORMAT_ARGB888, 32 },
    { "a8r8g8b8", RTGRAPHIC_PIXEL_FORMAT_ARGB888, 32 },
    { "x8b8g8r8", RTGRAPHIC_PIXEL_FORMAT_ABGR888, 32 },
    { "a8b8g8r8", RTGRAPHIC_PIXEL_FORMAT_ABGR888, 32 },
};

static rt_err_t floatfb_plane_fb_update(struct rt_graphic_plane *plane,
        struct rt_device_rect_info *rect)
{
    int bpp;
    rt_size_t size;
    void *to_fb, *from_fb;
    struct floatfb *ffb = rt_container_of(plane->graphic, struct floatfb, parent);

    bpp = plane->bits_per_pixel / 8;
    size = bpp * rect->width;
    to_fb = ffb->render_fb + rect->x * bpp + rect->y * ffb->stride;
    from_fb = ffb->screen_base + rect->x * bpp + rect->y * ffb->render_stride;

    for (int y = 0; y < rect->height; ++y)
    {
        rt_memcpy(to_fb, from_fb, size);
        to_fb += ffb->stride;
        from_fb += ffb->render_stride;
    }

    return RT_EOK;
}

static rt_err_t floatfb_plane_fb_remap(struct rt_graphic_plane *plane,
        rt_uint32_t mode, struct rt_device_rect_info *rect)
{
    struct floatfb *ffb = rt_container_of(plane->graphic, struct floatfb, parent);

    plane->line_length = ffb->render_stride;
    plane->bits_per_pixel = rt_graphic_mode_bpp(mode);

    plane->framebuffer = ffb->screen_base;
    plane->screen_len = ffb->screen_size;
    plane->framebuffer_len = ffb->screen_size;

    return RT_EOK;
}

static const struct rt_graphic_plane_ops floatfb_plane_ops =
{
    .update = floatfb_plane_fb_update,
    .fb_remap = floatfb_plane_fb_remap,
};

static rt_err_t floatfb_probe(struct rt_platform_device *pdev)
{
    rt_err_t err = RT_EOK;
    const char *format;
    struct floatfb_format *params = RT_NULL;
    struct rt_device *dev = &pdev->parent;
    struct floatfb *ffb = rt_calloc(1, sizeof(*ffb));

    if (!ffb)
    {
        return -RT_ENOMEM;
    }

    err |= rt_dm_dev_prop_read_string(dev, "format", &format);
    err |= rt_dm_dev_prop_read_u32(dev, "offset-x", &ffb->offset_x);
    err |= rt_dm_dev_prop_read_u32(dev, "offset-y", &ffb->offset_y);
    err |= rt_dm_dev_prop_read_u32(dev, "stride", &ffb->stride);
    err |= rt_dm_dev_prop_read_u32(dev, "height", &ffb->height);
    err |= rt_dm_dev_prop_read_u32(dev, "width", &ffb->width);

    if (err)
    {
        goto _fail;
    }

    for (int i = 0; i < RT_ARRAY_SIZE(floatfb_formats); ++i)
    {
        if (rt_strcmp(format, floatfb_formats[i].name))
        {
            continue;
        }

        params = &floatfb_formats[i];
        break;
    }

    if (!params)
    {
        goto _fail;
    }

    ffb->peer_fb = rt_dm_dev_iomap(dev, 0);

    if (!ffb->peer_fb)
    {
        err = -RT_EIO;
        goto _fail;
    }
    ffb->render_fb = ffb->peer_fb;
    ffb->render_fb += params->bits_per_pixel / 8 * ffb->offset_x;
    ffb->render_fb += ffb->stride * ffb->offset_y;
    ffb->render_stride = params->bits_per_pixel / 8 * ffb->width;

    ffb->screen_size = ffb->height * ffb->stride;
    ffb->screen_base = rt_malloc_align(ffb->screen_size, ARCH_PAGE_SIZE);

    if (!ffb->screen_base)
    {
        goto _free_fb;
    }

    if ((err = rt_graphic_device_simple_register(&ffb->parent,
            ffb->width, ffb->height, 0, &floatfb_plane_ops,
            &params->mode, 1)))
    {
        goto _fail;
    }

    pdev->parent.user_data = ffb;

    return RT_EOK;

_free_fb:
    rt_iounmap(ffb->peer_fb);

_fail:
    rt_free(ffb);

    return err;
}

static rt_err_t floatfb_remove(struct rt_platform_device *pdev)
{
    struct floatfb *ffb = pdev->parent.user_data;

    rt_graphic_device_simple_unregister(&ffb->parent);
    rt_iounmap(ffb->peer_fb);

    rt_free_align(ffb->screen_base);
    rt_free(ffb);

    return RT_EOK;
}

static const struct rt_ofw_node_id floatfb_ofw_ids[] =
{
    { .compatible = "rt-thread,float-fb" },
    { /* sentinel */ }
};

static struct rt_platform_driver floatfb_driver =
{
    .name = "float-framebuffer",
    .ids = floatfb_ofw_ids,

    .probe = floatfb_probe,
    .remove = floatfb_remove,
};
RT_PLATFORM_DRIVER_EXPORT(floatfb_driver);
