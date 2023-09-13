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
#include <dt-bindings/input/event-codes.h>

#define DBG_TAG "serial.virtual"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "render.h"

struct virtual_serial
{
    struct rt_serial_device parent;

    struct rt_device *fbdev;
    struct rt_device *kdbdev;

    struct render_point start, end;

    int ctrl;
    int ctrl_index;
    rt_bool_t is_ctrl;

    int param;
    rt_bool_t is_escape;
    rt_bool_t is_bracket;
    struct rt_spinlock spinlock;
};

#define raw_to_virtual_serial(raw)  rt_container_of(raw, struct virtual_serial, parent)

__asm__ (
        ".section .text\n"
    #ifdef ARCH_CPU_64BIT
        "    .align 8\n"
    #else
        "    .align 4\n"
    #endif
        "    .globl __virtual_font_start\n"
        "__virtual_font_start:\n"
        "    .incbin \"" RT_STRINGIFY(__RT_THREAD_VIRTUAL_FONT_PATH__) "\"\n"
        "    .globl __virtual_font_start\n"
        "__virtual_font_end:");

extern rt_uint8_t __virtual_font_start;
extern rt_uint8_t __virtual_font_end;

enum
{
    COLOR_BLACK,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_YELLOW,
    COLOR_BLUE,
    COLOR_MAGENTA,
    COLOR_CYAN,
    COLOR_LIGHT_GRAY,
    COLOR_DARK_GRAY,
    COLOR_LIGHT_RED,
    COLOR_LIGHT_GREEN,
    COLOR_LIGHT_YELLOW,
    COLOR_LIGHT_BLUE,
    COLOR_LIGHT_MAGENTA,
    COLOR_LIGHT_CYAN,
    COLOR_WHITE,
};

static struct render_color font_colors[] =
{                             /*  R    G    B    A */
    [COLOR_BLACK]           = {   0,   0,   0, 255 },
    [COLOR_RED]             = { 205,   0,   0, 255 },
    [COLOR_GREEN]           = {   0, 205,   0, 255 },
    [COLOR_YELLOW]          = { 205, 205,   0, 255 },
    [COLOR_BLUE]            = {   0,   0, 238, 255 },
    [COLOR_MAGENTA]         = { 205,   0, 205, 255 },
    [COLOR_CYAN]            = {   0, 205, 205, 255 },
    [COLOR_LIGHT_GRAY]      = { 229, 229, 229, 255 },
    [COLOR_DARK_GRAY]       = { 127, 127, 127, 255 },
    [COLOR_LIGHT_RED]       = { 255,   0,   0, 255 },
    [COLOR_LIGHT_GREEN]     = {   0, 255,   0, 255 },
    [COLOR_LIGHT_YELLOW]    = { 255, 255,   0, 255 },
    [COLOR_LIGHT_BLUE]      = {  92,  92, 255, 255 },
    [COLOR_LIGHT_MAGENTA]   = { 255,   0, 255, 255 },
    [COLOR_LIGHT_CYAN]      = {   0, 255, 255, 255 },
    [COLOR_WHITE]           = { 255, 255, 255, 255 },
};

static const rt_uint8_t unix_color_map[] =
{
    /* \033[Xm */
    [0]  = COLOR_WHITE,
    [30] = COLOR_BLACK,
    [31] = COLOR_RED,
    [32] = COLOR_GREEN,
    [33] = COLOR_YELLOW,
    [34] = COLOR_BLUE,
    [35] = COLOR_MAGENTA,
    [36] = COLOR_CYAN,
    [37] = COLOR_LIGHT_GRAY,
    [90] = COLOR_DARK_GRAY,
    [91] = COLOR_LIGHT_RED,
    [92] = COLOR_LIGHT_GREEN,
    [93] = COLOR_LIGHT_YELLOW,
    [94] = COLOR_LIGHT_BLUE,
    [95] = COLOR_LIGHT_MAGENTA,
    [96] = COLOR_LIGHT_CYAN,
    [97] = COLOR_WHITE,
};

static enum cursor cursor_shape = CURSOR_BLOCK;

static struct virtual_serial _vs = {};

static rt_err_t virtual_serial_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    return RT_EOK;
}

static rt_err_t virtual_serial_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct virtual_serial *vs = raw_to_virtual_serial(serial);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        if (vs->kdbdev)
        {
            rt_device_control(vs->kdbdev, RT_DEVICE_CTRL_CLR_INT, RT_NULL);
        }
        break;

    case RT_DEVICE_CTRL_SET_INT:
        if (vs->kdbdev)
        {
            rt_device_control(vs->kdbdev, RT_DEVICE_CTRL_SET_INT, RT_NULL);
        }
        break;
    }

    return RT_EOK;
}

static rt_bool_t parse_ctrl(struct virtual_serial *vs, char c)
{
    if (vs->is_bracket && c == 'm')
    {
        render_set_foreground(&font_colors[unix_color_map[vs->param]]);

        return RT_TRUE;
    }

    if (c == 'J' && vs->param == 2)
    {
        render_clear_display();

        return RT_TRUE;
    }

    if (c == 'K' && vs->param == 2)
    {
        rt_uint32_t old_col;
        struct render_point point;

        render_current_cursor(&point);
        old_col = point.col;

        for (int i = vs->start.col; i < old_col; ++i)
        {
            --point.col;
            render_move_cursor(&point);
            render_put_char(' ');
        }

        point.col = old_col;
        render_move_cursor(&point);

        return RT_TRUE;
    }

    if (c == 'H')
    {
        render_move_cursor(&vs->start);

        return RT_TRUE;
    }

    return RT_FALSE;
}

static int virtual_serial_putc(struct rt_serial_device *serial, char c)
{
    int res = 1;
    struct virtual_serial *vs = raw_to_virtual_serial(serial);

    rt_spin_lock(&vs->spinlock);

    if (rt_unlikely(vs->is_escape))
    {
        if (c == '[')
        {
            vs->param = 0;
            vs->is_bracket = RT_TRUE;
        }
        else if (vs->is_bracket && (c >= '0' && c <= '9'))
        {
            vs->param *= 10;
            vs->param += (c - '0');
        }
        else
        {
            rt_bool_t ok = parse_ctrl(vs, c);

            vs->is_bracket = RT_FALSE;
            vs->is_escape = RT_FALSE;

            if (!ok)
            {
                goto _end_escape;
            }
        }

        res = 0;
        goto _out_lock;
    }

_end_escape:
    if (rt_likely(c >= ' '))
    {
        render_put_char(c);
    }
    else
    {
        struct render_point point;

        switch (c)
        {
        case '\n':
            render_return_cursor(RT_NULL);
            break;

        case '\033':
            vs->is_escape = RT_TRUE;
            break;

        case '\t':
            render_current_cursor(&point);
            point.col = (point.col / 4 + 1) * 4;

            if (point.col > vs->end.col)
            {
                point.col = point.col - vs->end.col;

                if (point.row + 1 > vs->end.row)
                {
                    render_return_cursor(RT_NULL);
                }
                else
                {
                    ++point.row;
                }
            }
            render_move_cursor(&point);
            break;

        case '\r':
            render_reset_cursor(RT_NULL);
            break;

        case '\b':
            render_current_cursor(&point);
            if (point.col > 0)
            {
                --point.col;
            }
            render_move_cursor(&point);
            break;

        default:
            break;
        }
    }

_out_lock:
    rt_spin_unlock(&vs->spinlock);

    return res;
}

static int virtual_serial_getc(struct rt_serial_device *serial)
{
    int ch = -1;
    static rt_uint8_t uart_ctrl_map[][3] =
    {
        { 0x1b, 0x5b, 0x41 }, /* up key */
        { 0x1b, 0x5b, 0x42 }, /* down key */
        { 0x1b, 0x5b, 0x43 }, /* right key */
        { 0x1b, 0x5b, 0x44 }, /* left key */
    };
    struct virtual_serial *vs = raw_to_virtual_serial(serial);

    if (rt_unlikely(vs->ctrl))
    {
        goto _ctrl;
    }

    rt_device_read(vs->kdbdev, 0, &ch, 1);

    if (rt_unlikely(ch == KEY_UP))
    {
        vs->ctrl = 0;
        goto _ctrl;
    }
    else if (rt_unlikely(ch == KEY_LEFT))
    {
        vs->ctrl = 1;
        goto _ctrl;
    }
    else if (rt_unlikely(ch == KEY_RIGHT))
    {
        vs->ctrl = 2;
        goto _ctrl;
    }
    else if (rt_unlikely(ch == KEY_DOWN))
    {
        vs->ctrl = 3;
        goto _ctrl;
    }

    return ch;

_ctrl:
    vs->ctrl = RT_TRUE;
    ch = uart_ctrl_map[vs->ctrl][vs->ctrl_index++];

    if (vs->ctrl_index >= RT_ARRAY_SIZE(uart_ctrl_map[0]))
    {
        vs->ctrl_index = 0;
        vs->ctrl = RT_FALSE;
    }

    return ch;
}

static const struct rt_uart_ops virtual_serial_ops =
{
    .configure = virtual_serial_configure,
    .control = virtual_serial_control,
    .putc = virtual_serial_putc,
    .getc = virtual_serial_getc,
};

static void virtual_serial_kdb_notify(struct rt_device *dev)
{
    struct virtual_serial *vs;

    vs = raw_to_virtual_serial(rt_container_of(dev, struct rt_serial_device, parent));

    rt_hw_serial_isr(&vs->parent, RT_SERIAL_EVENT_RX_IND);
}

static int virtual_serial_setup(void)
{
    rt_err_t err;
    void *psf_data = (void *)&__virtual_font_start;
    rt_size_t psf_size = &__virtual_font_end - &__virtual_font_start;

    rt_memset(&_vs, 0, sizeof(_vs));

    _vs.fbdev = rt_device_find(RT_SERIAL_VIRTUAL_FBDEV);
    _vs.kdbdev = rt_device_find(RT_SERIAL_VIRTUAL_KDBDEV);

    if (!_vs.fbdev)
    {
        return (int)-RT_EIO;
    }

    if ((err = render_load_fbdev(_vs.fbdev)))
    {
        LOG_E("Load fbdev error = %s", rt_strerror(err));

        return (int)err;
    }

    if ((err = render_load_font(psf_data, psf_size,
            &font_colors[COLOR_WHITE], &font_colors[COLOR_BLACK],
            &_vs.start, &_vs.end)))
    {
        LOG_E("Load PSF font error = %s", rt_strerror(err));

        return (int)err;
    }

    render_select_cursor(cursor_shape);

    rt_device_open(_vs.fbdev, 0);

    if (_vs.kdbdev)
    {
        rt_device_open(_vs.kdbdev, 0);
        rt_device_control(_vs.kdbdev, RT_DEVICE_CTRL_NOTIFY_SET, &virtual_serial_kdb_notify);
    }

    rt_spin_lock_init(&_vs.spinlock);

    _vs.parent.ops = &virtual_serial_ops;
    rt_hw_serial_register(&_vs.parent, "vuart", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, &_vs);

    return 0;
}
INIT_DEVICE_EXPORT(virtual_serial_setup);

#if defined(RT_USING_CONSOLE) && defined(RT_USING_MSH)
static int virtual_serial_cmd_color(int argc, char**argv)
{
    const char *color;
    static const char * const color_names[] =
    {
        [COLOR_BLACK]           = "black",
        [COLOR_RED]             = "red",
        [COLOR_GREEN]           = "green",
        [COLOR_YELLOW]          = "yellow",
        [COLOR_BLUE]            = "blue",
        [COLOR_MAGENTA]         = "magenta",
        [COLOR_CYAN]            = "cyan",
        [COLOR_LIGHT_GRAY]      = "light gray",
        [COLOR_DARK_GRAY]       = "dark gray",
        [COLOR_LIGHT_RED]       = "light red",
        [COLOR_LIGHT_GREEN]     = "light green",
        [COLOR_LIGHT_YELLOW]    = "light yellow",
        [COLOR_LIGHT_BLUE]      = "light blue",
        [COLOR_LIGHT_MAGENTA]   = "light magenta",
        [COLOR_LIGHT_CYAN]      = "light cyan",
        [COLOR_WHITE]           = "white",
    };

    if (argc != 2)
    {
        goto _help;
    }

    color = argv[1];

    if (!((color[0] >= '0' && color[0] <= '9') || (color[0] >= 'a' && color[0] <= 'f')) ||
        !((color[1] >= '0' && color[1] <= '9') || (color[1] >= 'a' && color[1] <= 'f')))
    {
        goto _help;
    }

    if (color[0] == color[1])
    {
        rt_kprintf("foreground cannot equal background\n");

        return (int)-RT_EINVAL;
    }

    render_set_foreground(&font_colors[color[0] - (color[0] >= 'a' ? ('a' - 10) : '0')]);
    render_set_background(&font_colors[color[1] - (color[1] >= 'a' ? ('a' - 10) : '0')]);

    return 0;

_help:
    rt_kprintf("Usage: color [attr]\nattr:\n");

    for (int i = 0; i < RT_ARRAY_SIZE(font_colors); ++i)
    {
        rt_kprintf("\t%x = %s\n", i, color_names[i]);
    }

    return (int)-RT_EINVAL;
}
MSH_CMD_EXPORT_ALIAS(virtual_serial_cmd_color, color, set virtual serial foreground and background);
#endif /* RT_USING_CONSOLE && RT_USING_MSH */
