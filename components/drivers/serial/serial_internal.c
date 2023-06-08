/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-16     GuEe-GUI     first version
 */

#include <rtatomic.h>

#include "serial_internal.h"

int serial_dev_set_name(struct rt_serial_device *sdev)
{
    int id;
    static volatile rt_atomic_t uid = 0;

    RT_ASSERT(sdev != RT_NULL)

    id = (int)rt_hw_atomic_add(&uid, 1);

    return rt_dm_set_dev_name(&sdev->parent, "uart%u", id);
}

void *serial_base_from_args(char *str)
{
    rt_ubase_t base = 0;

    while (*str && !(*str == 'x' || *str == 'X'))
    {
        ++str;
    }
    ++str;

    /* The str may get from bootargs that we need check it */
    while (*str && *str != ',' && *str != ' ')
    {
        base <<= 4;

        if ((*str >= 'a' && *str <= 'f') || (*str >= 'A' && *str <= 'F'))
        {
            base += ((*str | ' ') - 'a') + 10;
        }
        else
        {
            base += *str - '0';
        }

        ++str;
    }

    return (void *)base;
}

struct serial_configure serial_cfg_from_args(char *str)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;

    /* Format baudrate/parity/bits/flow (BBBBPNF), Default is 115200n8 */
    if (str && *str)
    {
        rt_uint32_t baudrate = 0;

    #ifdef RT_USING_OFW
        *(str - 1) = RT_FDT_EARLYCON_OPTION_SIGNATURE;
    #endif

        /* BBBB is the speed */
        while (*str && (*str >= '0' && *str <= '9'))
        {
            baudrate *= 10;
            baudrate += *str - '0';
            ++str;
        }

        if (baudrate)
        {
            cfg.baud_rate = baudrate;
        }

        /* P is parity (n/o/e) */
        switch (*str)
        {
        case 'n':
            cfg.parity = PARITY_NONE;
            break;
        case 'o':
            cfg.parity = PARITY_ODD;
            break;
        case 'e':
            cfg.parity = PARITY_EVEN;
            break;
        default:
            --str;
            break;
        }
        ++str;

        /* N is number of bits */
        if (*str && (*str >= '0' && *str <= '9'))
        {
            cfg.data_bits = *str;
            ++str;
        }

        /* F is flow ontrol ('r' for RTS) */
        if (*str)
        {
            cfg.flowcontrol = (*str == 'r' ? RT_SERIAL_FLOWCONTROL_CTSRTS : RT_SERIAL_FLOWCONTROL_NONE);
        }
    }

    return cfg;
}
