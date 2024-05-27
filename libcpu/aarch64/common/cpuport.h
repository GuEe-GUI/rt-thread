/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-21     GuEe-GUI     rename barrier and add cpu relax
 */

#ifndef  CPUPORT_H__
#define  CPUPORT_H__

#include <armv8.h>
#include <rtdef.h>

#ifdef RT_USING_SMP
typedef union {
    unsigned long slock;
    struct __arch_tickets {
        unsigned short owner;
        unsigned short next;
    } tickets;
} rt_hw_spinlock_t;
#endif

#define rt_hw_barrier(cmd, ...) asm volatile (RT_STRINGIFY(cmd) " "RT_STRINGIFY(__VA_ARGS__):::"memory")

#define rt_hw_isb() rt_hw_barrier(isb)
#define rt_hw_dmb() rt_hw_barrier(dmb, ish)
#define rt_hw_wmb() rt_hw_barrier(dmb, ishst)
#define rt_hw_rmb() rt_hw_barrier(dmb, ishld)
#define rt_hw_dsb() rt_hw_barrier(dsb, ish)

#define rt_hw_wfi() rt_hw_barrier(wfi)
#define rt_hw_wfe() rt_hw_barrier(wfe)
#define rt_hw_sev() rt_hw_barrier(sev)

#define rt_hw_cpu_relax() rt_hw_barrier(yield)

#define sysreg_write(sysreg, val) \
    __asm__ volatile ("msr "RT_STRINGIFY(sysreg)", %0"::"r"((rt_uint64_t)(val)))

#define sysreg_read(sysreg, val) \
    __asm__ volatile ("mrs %0, "RT_STRINGIFY(sysreg)"":"=r"((val)))

rt_inline unsigned long __rt_clz(unsigned long word)
{
#ifdef __GNUC__
    return __builtin_clz(word);
#else
    unsigned long val;

    __asm__ volatile ("clz %0, %1"
        :"=r"(val)
        :"r"(word));

    return val;
#endif
}

rt_inline unsigned long __rt_ffsl(unsigned long word)
{
#ifdef __GNUC__
    return __builtin_ffsl(word);
#else
    if (!word)
    {
        return 0;
    }

    __asm__ volatile ("rbit %0, %0" : "+r" (word));

    return __rt_clz(word);
#endif
}

#endif  /*CPUPORT_H__*/
