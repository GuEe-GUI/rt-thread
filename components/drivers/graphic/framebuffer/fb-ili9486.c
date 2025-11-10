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

#define DBG_TAG "fb.ili9486"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define ILI9486_WIDTH       320
#define ILI9486_HEIGHT      480
#define ILI9486_BPP         16
#define ILI9486_STRIDE      (ILI9486_HEIGHT * (ILI9486_BPP / 8))

struct gpiod
{
    rt_ubase_t pin;
    rt_uint8_t active_val;
};

struct ili9486fb
{
    struct rt_graphic_device parent;
    struct rt_spi_device *spi;

    struct gpiod reset;
    struct gpiod dc;

    rt_uint8_t buffer[ILI9486_WIDTH * ILI9486_HEIGHT * 3];
    rt_uint8_t framebuffer[ILI9486_WIDTH * ILI9486_STRIDE];
};

static void ili9486fb_write_data(struct ili9486fb *ifb, rt_uint8_t data)
{
    rt_uint8_t send_data[2];

    send_data[0] = (data >> 8) & 0xff;
    send_data[1] = data & 0xff;

    rt_spi_transfer(ifb->spi, send_data, RT_NULL, sizeof(send_data));
}

#define ili9486fb_write_data_list(ifb, ...)                 \
do {                                                        \
    rt_uint8_t __data[] = { __VA_ARGS__ };                  \
    for (int __i = 0; __i < RT_ARRAY_SIZE(__data); ++__i)   \
    {                                                       \
        ili9486fb_write_data(ifb, __data[__i]);             \
    }                                                       \
} while (0)

static void ili9486fb_write_cmd(struct ili9486fb *ifb, rt_uint8_t cmd)
{
    /* CMD mode */
    rt_pin_write(ifb->reset.pin, !ifb->reset.active_val);

    ili9486fb_write_data(ifb, cmd);

    /* Data mode */
    rt_pin_write(ifb->reset.pin, ifb->reset.active_val);
}

static rt_err_t ili9486fb_plane_update(struct rt_graphic_plane *plane,
        struct rt_device_rect_info *rect)
{
    rt_uint8_t *buffer, r, g, b;
    rt_uint16_t *pixels, color;
    struct ili9486fb *ifb = rt_container_of(plane->graphic, struct ili9486fb, parent);

    /* Column address set */
    ili9486fb_write_data_list(ifb, 0x2a,
            (rect->x >> 8) & 0xff,
            rect->x & 0xff,
            ((rect->x + rect->width - 1) >> 8) & 0xff,
            (rect->x + rect->width - 1) & 0xff);

    /* Row adress set */
    ili9486fb_write_data_list(ifb, 0x2b,
            (rect->y >> 8) & 0xff,
            rect->y & 0xff,
            ((rect->y + rect->height - 1) >> 8) & 0xff,
            (rect->y + rect->height - 1) & 0xff);

    /* Memory write */
    ili9486fb_write_data(ifb, 0x2c);

    buffer = ifb->buffer;
    pixels = (void *)ifb->framebuffer +
            rect->y * ILI9486_STRIDE + rect->x * (ILI9486_BPP / 8);

    for (int y = 0; y < rect->height; ++y)
    {
        for (int x = 0; x < rect->width; ++x)
        {
            color = pixels[x];

            r = (color & RT_GENMASK(15, 11)) >> 11;
            g = (color & RT_GENMASK(10, 5)) >> 5;
            b = color & RT_GENMASK(4, 0);

            *buffer++ = (r & 0x1f) << 3;
            *buffer++ = (g & 0x3f) << 2;
            *buffer++ = (b & 0x1f) << 3;
        }

        pixels += ILI9486_STRIDE;
    }

    rt_spi_transfer(ifb->spi, ifb->buffer, RT_NULL,
            rect->width * rect->height * (ILI9486_BPP / 8));

    return RT_EOK;
}

static rt_err_t ili9486fb_plane_fb_remap(struct rt_graphic_plane *plane,
        rt_uint32_t mode, struct rt_device_rect_info *rect)
{
    struct ili9486fb *ifb = rt_container_of(plane->graphic, struct ili9486fb, parent);

    plane->line_length = ILI9486_STRIDE;
    plane->bits_per_pixel = rt_graphic_mode_bpp(mode);

    plane->framebuffer = ifb->framebuffer;
    plane->screen_len = sizeof(ifb->framebuffer);
    plane->framebuffer_len = plane->screen_len;

    return RT_EOK;
}

static const struct rt_graphic_plane_ops ili9486fb_plane_ops =
{
    .update = ili9486fb_plane_update,
    .fb_remap = ili9486fb_plane_fb_remap,
};

static rt_uint32_t ili9486fb_modes[] =
{
    RTGRAPHIC_PIXEL_FORMAT_RGB565,
};

static void ili9486fb_reset(struct ili9486fb *ifb)
{
    rt_pin_mode(ifb->reset.pin, PIN_MODE_OUTPUT);

    rt_pin_write(ifb->reset.pin, !ifb->reset.active_val);
    rt_hw_us_delay(20);

    rt_pin_write(ifb->reset.pin, ifb->reset.active_val);
    rt_thread_mdelay(120);
}

static void ili9486fb_init(struct ili9486fb *ifb, rt_uint32_t rotation)
{
    /* Software reset */
    ili9486fb_write_cmd(ifb, 0x00);
    rt_thread_mdelay(5);

    /* Display off */
    ili9486fb_write_cmd(ifb, 0x28);
    rt_thread_mdelay(150);

    /* Interface Pixel Format */
    ili9486fb_write_cmd(ifb, 0x3a);
    /* 16 bit/pixe */
    ili9486fb_write_data(ifb, 0x55);

    /* Interface Pixel Format */
    ili9486fb_write_cmd(ifb, 0xc2);
    ili9486fb_write_data(ifb, 0x44);

    /* VCOM Control */
    ili9486fb_write_cmd(ifb, 0xc5);
    ili9486fb_write_data_list(ifb, 0x00, 0x00, 0x00, 0x00);

    /* PGAMCTRL (Positive Gamma Control) */
    ili9486fb_write_cmd(ifb, 0xe0);
    ili9486fb_write_data_list(ifb, 0x0f, 0x1f, 0x1c, 0x0c, 0x0f, 0x08, 0x48,
            0x98, 0x37, 0x0a, 0x13, 0x04, 0x11, 0x0d, 0x00);

    /* NGAMCTRL (Negative Gamma Correction) */
    ili9486fb_write_cmd(ifb, 0xe1);
    ili9486fb_write_data_list(ifb, 0x0f, 0x32, 0x2e, 0x0b, 0x0d, 0x05, 0x47,
            0x75, 0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00);

    /* Sleep out, also SW reset */
    ili9486fb_write_cmd(ifb, 0x11);
    rt_thread_mdelay(120);

    /* Set rotation */
    if (rotation == 0 || rotation == 180)
    {
        if (rotation == 0)
        {
            ili9486fb_write_data(ifb, 0x48);
        }
        else
        {
            ili9486fb_write_data(ifb, 0x98);
        }

        ili9486fb_write_cmd(ifb, 0x2a);
        ili9486fb_write_data_list(ifb, 0x00, 0x00, 0x01, 0x3f);

        ili9486fb_write_cmd(ifb, 0x2b);
        ili9486fb_write_data_list(ifb, 0x00, 0x00, 0x01, 0xe0);
    }
    else if (rotation == 180 || rotation == 270)
    {
        if (rotation == 180)
        {
            ili9486fb_write_data(ifb, 0x28);
        }
        else
        {
            ili9486fb_write_data(ifb, 0xf8);
        }

        ili9486fb_write_cmd(ifb, 0x2b);
        ili9486fb_write_data_list(ifb, 0x00, 0x00, 0x01, 0x3f);

        ili9486fb_write_cmd(ifb, 0x2a);
        ili9486fb_write_data_list(ifb, 0x00, 0x00, 0x01, 0xe0);
    }

    /* Display on */
    ili9486fb_write_cmd(ifb, 0x29);
    rt_thread_mdelay(20);
}

static rt_err_t ili9486fb_probe(struct rt_spi_device *spi_dev)
{
    rt_err_t err;
    rt_uint32_t rotation = 0;
    struct rt_device *dev = &spi_dev->parent;
    struct ili9486fb *ifb = rt_calloc(1, sizeof(*ifb));

    if (!ifb)
    {
        return -RT_ENOMEM;
    }
    ifb->spi = spi_dev;

    ifb->reset.pin = rt_pin_get_named_pin(dev, "reset", 0,
            RT_NULL, &ifb->reset.active_val);

    if (ifb->reset.pin < 0 && ifb->reset.pin != -RT_EEMPTY)
    {
        err = ifb->reset.pin;
        goto _fail;
    }

    ifb->dc.pin = rt_pin_get_named_pin(dev, "dc", 0,
            RT_NULL, &ifb->dc.active_val);

    if (ifb->dc.pin < 0 && ifb->dc.pin != -RT_EEMPTY)
    {
        err = ifb->dc.pin;
        goto _fail;
    }

    rt_dm_dev_prop_read_u32(dev, "rotation", &rotation);

    ili9486fb_reset(ifb);
    ili9486fb_init(ifb, rotation);

    if ((err = rt_graphic_device_simple_register(&ifb->parent,
            ILI9486_WIDTH, ILI9486_HEIGHT, 0, &ili9486fb_plane_ops,
            ili9486fb_modes, RT_ARRAY_SIZE(ili9486fb_modes))))
    {
        goto _fail;
    }

    spi_dev->parent.user_data = ifb;

    return RT_EOK;

_fail:
    rt_free(ifb);

    return err;
}

static rt_err_t ili9486fb_remove(struct rt_spi_device *spi_dev)
{
    struct ili9486fb *ifb = spi_dev->parent.user_data;

    rt_graphic_device_simple_unregister(&ifb->parent);

    ili9486fb_reset(ifb);

    rt_free(ifb);

    return RT_EOK;
}

static const struct rt_spi_device_id ili9486fb_ids[] =
{
    { .name = "ili9486" },
    { .name = "rpi-lcd-35" },
    { .name = "piscreen" },
    { /* sentinel */ },
};

static const struct rt_ofw_node_id ili9486fb_ofw_ids[] =
{
    { .compatible = "waveshare,rpi-lcd-35" },
    { .compatible = "ozzmaker,piscreen" },
    { .compatible = "ilitek,ili9486" },
    { /* sentinel */ },
};

static struct rt_spi_driver ili9486fb_driver =
{
    .ids = ili9486fb_ids,
    .ofw_ids = ili9486fb_ofw_ids,

    .probe = ili9486fb_probe,
    .remove = ili9486fb_remove,
};
RT_SPI_DRIVER_EXPORT(ili9486fb_driver);
