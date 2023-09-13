/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-09-15     Bernard      first version
 * 2023-07-13     GuEe-GUI     append fpu: Q16 ~ Q31
 */

#ifndef __ARMV8_H__
#define __ARMV8_H__

#include <rtdef.h>

typedef struct { rt_uint64_t value[2]; } rt_uint128_t;

/* the exception stack without VFP registers */
struct rt_hw_exp_stack
{
    rt_uint64_t pc;
    rt_uint64_t cpsr;
    rt_uint64_t sp_el0;
    rt_uint64_t x30;
    rt_uint64_t fpcr;
    rt_uint64_t fpsr;
    rt_uint64_t x28;
    rt_uint64_t x29;
    rt_uint64_t x26;
    rt_uint64_t x27;
    rt_uint64_t x24;
    rt_uint64_t x25;
    rt_uint64_t x22;
    rt_uint64_t x23;
    rt_uint64_t x20;
    rt_uint64_t x21;
    rt_uint64_t x18;
    rt_uint64_t x19;
    rt_uint64_t x16;
    rt_uint64_t x17;
    rt_uint64_t x14;
    rt_uint64_t x15;
    rt_uint64_t x12;
    rt_uint64_t x13;
    rt_uint64_t x10;
    rt_uint64_t x11;
    rt_uint64_t x8;
    rt_uint64_t x9;
    rt_uint64_t x6;
    rt_uint64_t x7;
    rt_uint64_t x4;
    rt_uint64_t x5;
    rt_uint64_t x2;
    rt_uint64_t x3;
    rt_uint64_t x0;
    rt_uint64_t x1;

    rt_uint128_t fpu[32];
};

#define SP_ELx     ((unsigned long)0x01)
#define SP_EL0     ((unsigned long)0x00)
#define PSTATE_EL1 ((unsigned long)0x04)
#define PSTATE_EL2 ((unsigned long)0x08)
#define PSTATE_EL3 ((unsigned long)0x0c)

rt_ubase_t rt_hw_get_current_el(void);
void rt_hw_set_elx_env(void);
void rt_hw_set_current_vbar(rt_ubase_t addr);

#endif
