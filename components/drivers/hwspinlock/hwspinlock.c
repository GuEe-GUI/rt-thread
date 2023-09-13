/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-09-23     GuEe-GUI     first version
 */

#include <cpuport.h>

#define DBG_TAG "rtdm.hwspinlock"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "hwspinlock_dm.h"

static struct rt_spinlock hwspinlock_ops_lock = {};
static rt_list_t hwspinlock_bank_nodes = RT_LIST_OBJECT_INIT(hwspinlock_bank_nodes);

rt_err_t rt_hwspinlock_bank_register(struct rt_hwspinlock_bank *bank)
{
    struct rt_hwspinlock *hwlock;

    if (!bank || !bank->ops || bank->locks_nr <= 0 || !bank->dev)
    {
        return -RT_EINVAL;
    }

    rt_list_init(&bank->list);
    ref_init(&bank->ref);

    hwlock = &bank->locks[0];

    for (int i = bank->locks_nr - 1; i >= 0; --i, ++hwlock)
    {
        rt_spin_lock_init(&hwlock->lock);
        hwlock->used = RT_FALSE;
    }

    rt_spin_lock(&hwspinlock_ops_lock);

    rt_list_insert_after(&hwspinlock_bank_nodes, &bank->list);

    rt_spin_unlock(&hwspinlock_ops_lock);

    rt_dm_dev_bind_fwdata(bank->dev, RT_NULL, bank);

    return RT_EOK;
}

rt_err_t rt_hwspinlock_bank_unregister(struct rt_hwspinlock_bank *bank)
{
    rt_err_t err;

    if (!bank)
    {
        return -RT_EINVAL;
    }

    rt_spin_lock(&hwspinlock_ops_lock);

    if (ref_read(&bank->ref) == 1)
    {
        rt_dm_dev_unbind_fwdata(bank->dev, RT_NULL);

        err = RT_EOK;
    }
    else
    {
        err = -RT_EBUSY;
    }

    rt_spin_unlock(&hwspinlock_ops_lock);

    return err;
}

rt_err_t rt_hwspin_trylock_mode(struct rt_hwspinlock *hwlock, int mode,
        rt_ubase_t *out_level)
{
    rt_err_t err;

    if (rt_unlikely(!hwlock || (!out_level && mode == RT_HWSPINLOCK_IRQSTATE)))
    {
        return -RT_EINVAL;
    }

    switch (mode)
    {
    case RT_HWSPINLOCK_IRQSTATE:
        err = rt_spin_trylock_irqsave(&hwlock->lock, out_level);
        break;

    case RT_HWSPINLOCK_RAW:
    case RT_HWSPINLOCK_IN_ATOMIC:
        err = RT_EOK;
        break;

    default:
        err = rt_spin_trylock(&hwlock->lock);
        break;
    }

    if (err < 0)
    {
        return -RT_EBUSY;
    }

    err = hwlock->bank->ops->trylock(hwlock);

    if (!err)
    {
        switch (mode)
        {
        case RT_HWSPINLOCK_IRQSTATE:
            rt_spin_unlock_irqrestore(&hwlock->lock, *out_level);
            break;

        case RT_HWSPINLOCK_RAW:
        case RT_HWSPINLOCK_IN_ATOMIC:
            break;

        default:
            rt_spin_unlock(&hwlock->lock);
            break;
        }

        return -RT_EBUSY;
    }

    rt_hw_dmb();

    return err;
}

rt_err_t rt_hwspin_lock_timeout_mode(struct rt_hwspinlock *hwlock, int mode,
        rt_uint32_t timeout_ms, rt_ubase_t *out_level)
{
    rt_err_t err;
    rt_base_t us_timeout;
    rt_tick_t timeout = rt_tick_get() + rt_tick_from_millisecond(timeout_ms);

    if (mode == RT_HWSPINLOCK_IN_ATOMIC)
    {
        us_timeout = timeout_ms * 1000;
    }

    for (;;)
    {
        err = rt_hwspin_trylock_mode(hwlock, mode, out_level);

        if (err != -RT_EBUSY)
        {
            break;
        }

        if (mode == RT_HWSPINLOCK_IN_ATOMIC)
        {
            /* timeout_ms is ms unit, so delay 1/10 ms */
            const int retry_us = 100;

            rt_hw_us_delay(retry_us);
            us_timeout -= retry_us;

            if (us_timeout < 0)
            {
                return -RT_ETIMEOUT;
            }
        }
        else if (timeout > rt_tick_get())
        {
            return -RT_ETIMEOUT;
        }

        if (hwlock->bank->ops->relax)
        {
            hwlock->bank->ops->relax(hwlock);
        }
    }

    return err;
}

void rt_hwspin_unlock_mode(struct rt_hwspinlock *hwlock, int mode,
        rt_ubase_t *out_level)
{
    if (rt_unlikely(!hwlock || (!out_level && mode == RT_HWSPINLOCK_IRQSTATE)))
    {
        return;
    }

    rt_hw_dmb();

    hwlock->bank->ops->unlock(hwlock);

    switch (mode)
    {
    case RT_HWSPINLOCK_IRQSTATE:
        rt_spin_unlock_irqrestore(&hwlock->lock, *out_level);
        break;

        case RT_HWSPINLOCK_RAW:
        case RT_HWSPINLOCK_IN_ATOMIC:
        break;

    default:
        rt_spin_unlock(&hwlock->lock);
        break;
    }
}

static struct rt_hwspinlock *hwspinlock_request(int id)
{
    struct rt_hwspinlock_bank *bank;
    struct rt_hwspinlock *hwlock = RT_NULL;

    rt_spin_lock(&hwspinlock_ops_lock);

    rt_list_for_each_entry(bank, &hwspinlock_bank_nodes, list)
    {
        if (id <= 0)
        {
            for (int i = 0; i < bank->locks_nr; ++i)
            {
                if (!bank->locks[i].used)
                {
                    hwlock = &bank->locks[i];
                    break;
                }
            }
        }
        else if (id >= bank->base_id && id <= bank->base_id + bank->locks_nr)
        {
            int offset = id - bank->base_id;

            if (!bank->locks[offset].used)
            {
                hwlock = &bank->locks[offset];
                break;
            }
        }
    }

    if (hwlock)
    {
        hwlock->used = RT_TRUE;
    }

    rt_spin_unlock(&hwspinlock_ops_lock);

    return hwlock;
}

struct rt_hwspinlock *rt_hwspinlock_request(void)
{
    return hwspinlock_request(-1);
}

struct rt_hwspinlock *rt_hwspinlock_request_id(int id)
{
    return hwspinlock_request(id);
}

static void hwspinlock_release(struct ref *r)
{
    struct rt_hwspinlock_bank *bank = rt_container_of(r, struct rt_hwspinlock_bank, ref);

    LOG_E("%s is release", rt_dm_dev_get_name(bank->dev));
    (void)bank;

    RT_ASSERT(0);
}

rt_err_t rt_hwspinlock_free(struct rt_hwspinlock *hwlock)
{
    if (!hwlock)
    {
        return -RT_EINVAL;
    }

    ref_put(&hwlock->bank->ref, &hwspinlock_release);
    hwlock->used = RT_TRUE;

    return RT_EOK;
}

int rt_ofw_get_hwspinlock_id(struct rt_ofw_node *np, int index)
{
    int id;
    rt_err_t err;
    struct rt_ofw_cell_args args;
    struct rt_hwspinlock_bank *bank;

    if (rt_unlikely(!np && index < 0))
    {
        return -RT_EINVAL;
    }

    rt_spin_lock(&hwspinlock_ops_lock);

    err = rt_ofw_parse_phandle_cells(np, "hwlocks", "#hwlock-cells", index, &args);

    if (rt_unlikely(err))
    {
        goto _out_lock;
    }

    bank = rt_ofw_data(args.data);
    rt_ofw_node_put(args.data);

    if (rt_unlikely(!bank || args.args_count != 1))
    {
        err = -RT_ENOSYS;
    }
    else
    {
        id = bank->base_id + args.args[0];
    }

_out_lock:
    rt_spin_unlock(&hwspinlock_ops_lock);

    return err < 0 ? err : id;
}

int rt_ofw_get_hwspinlock_id_byname(struct rt_ofw_node *np, const char *name)
{
    int index;

    if (rt_unlikely(!np && !name))
    {
        return -RT_EINVAL;
    }

    index = rt_ofw_prop_index_of_string(np, "hwlock-names", name);

    if (rt_unlikely(index < 0))
    {
        return index;
    }

    return rt_ofw_get_hwspinlock_id(np, index);
}
