/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-3-1       zhouxiaohu   first version
 * 2023-5-18      GuEe-GUI     implemented by rtatomic
 */

#ifndef __UTIL_REF_H__
#define __UTIL_REF_H__

#include <rtatomic.h>

/**
 * struct ref must be embedded in an object.
 * it acts as a reference counter for the object.
 */
struct ref
{
    rt_atomic_t refcount;
};

#define REF_INIT(n)  { .refcount = n, }

rt_inline void ref_init(struct ref *r)
{
    rt_atomic_store(&r->refcount, 1);
}

rt_inline unsigned int ref_read(struct ref *r)
{
    return rt_atomic_load(&r->refcount);
}

/**
 * ref_get
 * increment reference counter for object.
 */
rt_inline void ref_get(struct ref *r)
{
    rt_atomic_add(&r->refcount, 1);
}

/**
 * ref_put
 * decrement reference counter for object.
 * If the reference counter is zero, call release().
 *
 * Return 1 means the object's reference counter is zero and release() is called.
 */
rt_inline int ref_put(struct ref *r, void (*release)(struct ref *r))
{
    if (rt_atomic_dec_and_test(&r->refcount))
    {
        release(r);

        return 1;
    }

    return 0;
}

/**
 * ref_get_unless_zero - Increment refcount for object unless it is zero.
 * Return non-zero if the increment succeeded. Otherwise return 0.
 */
rt_inline int ref_get_unless_zero(struct ref *r)
{
    return (int)rt_atomic_inc_not_zero(&r->refcount);
}

#endif /* __UTIL_REF_H__ */
