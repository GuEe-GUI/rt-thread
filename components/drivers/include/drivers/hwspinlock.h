/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-09-23     GuEe-GUI     first version
 */

#ifndef __HWSPINLOCK_H__
#define __HWSPINLOCK_H__

#include <ref.h>
#include <rthw.h>
#include <rtthread.h>

enum rt_hwspinlock_mode
{
    RT_HWSPINLOCK_NONE = 0,
    RT_HWSPINLOCK_IRQSTATE,
    RT_HWSPINLOCK_RAW,
    RT_HWSPINLOCK_IN_ATOMIC,    /* Called while in atomic context */
};

struct rt_hwspinlock;
struct rt_hwspinlock_ops;
struct rt_hwspinlock_bank;

rt_err_t rt_hwspinlock_bank_register(struct rt_hwspinlock_bank *bank);
rt_err_t rt_hwspinlock_bank_unregister(struct rt_hwspinlock_bank *bank);

rt_err_t rt_hwspin_trylock_mode(struct rt_hwspinlock *hwlock, int mode,
        rt_ubase_t *out_level);
rt_err_t rt_hwspin_lock_timeout_mode(struct rt_hwspinlock *hwlock, int mode,
        rt_uint32_t timeout_ms, rt_ubase_t *out_level);
void rt_hwspin_unlock_mode(struct rt_hwspinlock *hwlock, int mode,
        rt_ubase_t *out_level);

rt_inline rt_err_t rt_hwspin_trylock(struct rt_hwspinlock *hwlock)
{
    return rt_hwspin_trylock_mode(hwlock, RT_HWSPINLOCK_NONE, RT_NULL);
}

rt_inline rt_err_t rt_hwspin_trylock_irqsave(struct rt_hwspinlock *hwlock,
        rt_ubase_t *out_level)
{
    return rt_hwspin_trylock_mode(hwlock, RT_HWSPINLOCK_IRQSTATE, out_level);
}

rt_inline rt_err_t rt_hwspin_trylock_raw(struct rt_hwspinlock *hwlock)
{
    return rt_hwspin_trylock_mode(hwlock, RT_HWSPINLOCK_RAW, RT_NULL);
}

rt_inline rt_err_t rt_hwspin_trylock_in_atomic(struct rt_hwspinlock *hwlock)
{
    return rt_hwspin_trylock_mode(hwlock, RT_HWSPINLOCK_IN_ATOMIC, RT_NULL);
}

rt_inline rt_err_t rt_hwspin_lock_timeout(struct rt_hwspinlock *hwlock,
        rt_uint32_t timeout_ms)
{
    return rt_hwspin_lock_timeout_mode(hwlock, RT_HWSPINLOCK_NONE, timeout_ms,
            RT_NULL);
}

rt_inline rt_err_t rt_hwspin_lock_timeout_irqsave(struct rt_hwspinlock *hwlock,
        rt_uint32_t timeout_ms, rt_ubase_t *out_level)
{
    return rt_hwspin_lock_timeout_mode(hwlock, RT_HWSPINLOCK_IRQSTATE,
            timeout_ms, out_level);
}

rt_inline rt_err_t rt_hwspin_lock_timeout_raw(struct rt_hwspinlock *hwlock,
        rt_uint32_t timeout_ms)
{
    return rt_hwspin_lock_timeout_mode(hwlock, RT_HWSPINLOCK_RAW, timeout_ms,
            RT_NULL);
}

rt_inline rt_err_t rt_hwspin_lock_timeout_in_atomic(struct rt_hwspinlock *hwlock,
        rt_uint32_t timeout_ms)
{
    return rt_hwspin_lock_timeout_mode(hwlock, RT_HWSPINLOCK_IN_ATOMIC,
            timeout_ms, RT_NULL);
}

rt_inline void rt_hwspin_unlock(struct rt_hwspinlock *hwlock)
{
    rt_hwspin_unlock_mode(hwlock, RT_HWSPINLOCK_NONE, RT_NULL);
}

rt_inline void rt_hwspin_unlock_irqsave(struct rt_hwspinlock *hwlock,
        rt_ubase_t *out_level)
{
    rt_hwspin_unlock_mode(hwlock, RT_HWSPINLOCK_IRQSTATE, out_level);
}

rt_inline void rt_hwspin_unlock_raw(struct rt_hwspinlock *hwlock)
{
    rt_hwspin_unlock_mode(hwlock, RT_HWSPINLOCK_RAW, RT_NULL);
}

rt_inline void rt_hwspin_unlock_in_atomic(struct rt_hwspinlock *hwlock)
{
    rt_hwspin_unlock_mode(hwlock, RT_HWSPINLOCK_IN_ATOMIC, RT_NULL);
}

struct rt_hwspinlock *rt_hwspinlock_request(void);
struct rt_hwspinlock *rt_hwspinlock_request_id(int id);
rt_err_t rt_hwspinlock_free(struct rt_hwspinlock *hwlock);

int rt_ofw_get_hwspinlock_id(struct rt_ofw_node *np, int index);
int rt_ofw_get_hwspinlock_id_byname(struct rt_ofw_node *np, const char *name);

#endif /* __HWSPINLOCK_H__ */
