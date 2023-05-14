/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-20     ErikChan      the first version
 */

#ifndef __RT_DM_H__
#define __RT_DM_H__

/* init early before board_int, such as unflatten device tree */
#define INIT_EARLY_EXPORT(fn)       INIT_EXPORT(fn, "0.end.0")

/* init buses */
#define INIT_BUS_EXPORT(fn)         INIT_EXPORT(fn, "0.end.1")

/* init subsystems */
#define INIT_SUBSYS_EXPORT(fn)      INIT_EXPORT(fn, "1.0")

/* init drivers */
#define INIT_DRIVER_BUILIN_EXPORT(fn)     INIT_EXPORT(fn, "1.1")

#define RT_DRIVER_EXPORT(driver, bus_name, mode)      \
static int ___##driver##_register(void)               \
{                                                     \
    rt_##bus_name##_driver_register(&driver);         \
    return 0;                                         \
}                                                     \
INIT_DRIVER_##mode##_EXPORT(___##driver##_register);  \

#endif /* __RT_DM_H__ */
