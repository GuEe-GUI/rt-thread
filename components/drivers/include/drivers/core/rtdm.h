/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-20     ErikChan     the first version
 */

#ifndef __RT_DM_H__
#define __RT_DM_H__

#include <rthw.h>
#include <rtdef.h>
#include <ioremap.h>
#include <drivers/misc.h>
#include <drivers/byteorder.h>

#ifndef RT_CPUS_NR
#define RT_CPUS_NR 1
#endif

#ifndef RT_USING_SMP
extern int rt_hw_cpu_id(void);
#endif

void rt_dm_secondary_cpu_init(void);

int rt_dm_dev_set_name_auto(rt_device_t dev, const char *prefix);
int rt_dm_dev_get_name_id(rt_device_t dev);

int rt_dm_dev_set_name(rt_device_t dev, const char *format, ...);
const char *rt_dm_dev_get_name(rt_device_t dev);

int rt_dm_dev_get_address_count(rt_device_t dev);
rt_err_t rt_dm_dev_get_address(rt_device_t dev, int index,
        rt_uint64_t *out_address, rt_uint64_t *out_size);
rt_err_t rt_dm_dev_get_address_by_name(rt_device_t dev, const char *name,
        rt_uint64_t *out_address, rt_uint64_t *out_size);
int rt_dm_dev_get_address_array(rt_device_t dev, int nr, rt_uint64_t *out_regs);

void *rt_dm_dev_iomap(rt_device_t dev, int index);
void *rt_dm_dev_iomap_by_name(rt_device_t dev, const char *name);

int rt_dm_dev_get_irq_count(rt_device_t dev);
int rt_dm_dev_get_irq(rt_device_t dev, int index);
int rt_dm_dev_get_irq_by_name(rt_device_t dev, const char *name);

void rt_dm_dev_bind_fwdata(rt_device_t dev, void *fw_np, void *data);
void rt_dm_dev_unbind_fwdata(rt_device_t dev, void *fw_np);

int rt_dm_dev_prop_read_u8_array_index(rt_device_t dev, const char *propname,
        int index, int nr, rt_uint8_t *out_values);
int rt_dm_dev_prop_read_u16_array_index(rt_device_t dev, const char *propname,
        int index, int nr, rt_uint16_t *out_values);
int rt_dm_dev_prop_read_u32_array_index(rt_device_t dev, const char *propname,
        int index, int nr, rt_uint32_t *out_values);
int rt_dm_dev_prop_read_u64_array_index(rt_device_t dev, const char *propname,
        int index, int nr, rt_uint64_t *out_values);
int rt_dm_dev_prop_read_string_array_index(rt_device_t dev, const char *propname,
        int index, int nr, const char **out_strings);

int rt_dm_dev_prop_count_of_size(rt_device_t dev, const char *propname, int size);
int rt_dm_dev_prop_index_of_string(rt_device_t dev, const char *propname, const char *string);

rt_bool_t rt_dm_dev_prop_read_bool(rt_device_t dev, const char *propname);

rt_inline rt_err_t rt_dm_dev_prop_read_u8_index(rt_device_t dev, const char *propname,
        int index, rt_uint8_t *out_value)
{
    int nr = rt_dm_dev_prop_read_u8_array_index(dev, propname, index, 1, out_value);

    return nr > 0 ? RT_EOK : (rt_err_t)nr;
}

rt_inline rt_err_t rt_dm_dev_prop_read_u16_index(rt_device_t dev, const char *propname,
        int index, rt_uint16_t *out_value)
{
    int nr = rt_dm_dev_prop_read_u16_array_index(dev, propname, index, 1, out_value);

    return nr > 0 ? RT_EOK : (rt_err_t)nr;
}

rt_inline rt_err_t rt_dm_dev_prop_read_u32_index(rt_device_t dev, const char *propname,
        int index, rt_uint32_t *out_value)
{
    int nr = rt_dm_dev_prop_read_u32_array_index(dev, propname, index, 1, out_value);

    return nr > 0 ? RT_EOK : (rt_err_t)nr;
}

rt_inline rt_err_t rt_dm_dev_prop_read_u64_index(rt_device_t dev, const char *propname,
        int index, rt_uint64_t *out_value)
{
    int nr = rt_dm_dev_prop_read_u64_array_index(dev, propname, index, 1, out_value);

    return nr > 0 ? RT_EOK : (rt_err_t)nr;
}

rt_inline rt_err_t rt_dm_dev_prop_read_string_index(rt_device_t dev, const char *propname,
        int index, const char **out_string)
{
    int nr = rt_dm_dev_prop_read_string_array_index(dev, propname, index, 1, out_string);

    return nr > 0 ? RT_EOK : (rt_err_t)nr;
}

rt_inline rt_err_t rt_dm_dev_prop_read_u8(rt_device_t dev, const char *propname,
        rt_uint8_t *out_value)
{
    return rt_dm_dev_prop_read_u8_index(dev, propname, 0, out_value);
}

rt_inline rt_err_t rt_dm_dev_prop_read_u16(rt_device_t dev, const char *propname,
        rt_uint16_t *out_value)
{
    return rt_dm_dev_prop_read_u16_index(dev, propname, 0, out_value);
}

rt_inline rt_err_t rt_dm_dev_prop_read_u32(rt_device_t dev, const char *propname,
        rt_uint32_t *out_value)
{
    return rt_dm_dev_prop_read_u32_index(dev, propname, 0, out_value);
}

rt_inline rt_err_t rt_dm_dev_prop_read_s32(rt_device_t dev, const char *propname,
        rt_int32_t *out_value)
{
    return rt_dm_dev_prop_read_u32_index(dev, propname, 0, (rt_uint32_t *)out_value);
}

rt_inline rt_err_t rt_dm_dev_prop_read_u64(rt_device_t dev, const char *propname,
        rt_uint64_t *out_value)
{
    return rt_dm_dev_prop_read_u64_index(dev, propname, 0, out_value);
}

rt_inline rt_err_t rt_dm_dev_prop_read_string(rt_device_t dev, const char *propname,
        const char **out_string)
{
    return rt_dm_dev_prop_read_string_index(dev, propname, 0, out_string);
}

rt_inline int rt_dm_dev_prop_count_of_u8(rt_device_t dev, const char *propname)
{
    return rt_dm_dev_prop_count_of_size(dev, propname, sizeof(rt_uint8_t));
}

rt_inline int rt_dm_dev_prop_count_of_u16(rt_device_t dev, const char *propname)
{
    return rt_dm_dev_prop_count_of_size(dev, propname, sizeof(rt_uint16_t));
}

rt_inline int rt_dm_dev_prop_count_of_u32(rt_device_t dev, const char *propname)
{
    return rt_dm_dev_prop_count_of_size(dev, propname, sizeof(rt_uint32_t));
}

rt_inline int rt_dm_dev_prop_count_of_u64(rt_device_t dev, const char *propname)
{
    return rt_dm_dev_prop_count_of_size(dev, propname, sizeof(rt_uint64_t));
}

/* init cpu, memory, interrupt-controller, bus... */
#define INIT_CORE_EXPORT(fn)            INIT_EXPORT(fn, "1.0")
/* init pci/pcie, usb platform driver... */
#define INIT_FRAMEWORK_EXPORT(fn)       INIT_EXPORT(fn, "1.1")
/* init platform, user code... */
#define INIT_PLATFORM_EXPORT(fn)        INIT_EXPORT(fn, "1.2")
/* init sys-timer, clk, pinctrl... */
#define INIT_SUBSYS_EXPORT(fn)          INIT_EXPORT(fn, "1.3")
/* init subsystem if depends more... */
#define INIT_SUBSYS_LATER_EXPORT(fn)    INIT_EXPORT(fn, "1.3.1")
/* init early drivers */
#define INIT_DRIVER_EARLY_EXPORT(fn)    INIT_EXPORT(fn, "1.4")
/* init later drivers */
#define INIT_DRIVER_LATER_EXPORT(fn)    INIT_EXPORT(fn, "3.0")
/* init in secondary_cpu_c_start */
#define INIT_SECONDARY_CPU_EXPORT(fn)   INIT_EXPORT(fn, "7")
/* init after mount fs */
#define INIT_FS_EXPORT(fn)              INIT_EXPORT(fn, "6.0")

#endif /* __RT_DM_H__ */
