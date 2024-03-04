/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-6-27      GuEe-GUI     first version
 */

#ifndef __UTIL_BITMAP_H__
#define __UTIL_BITMAP_H__

#include <rtdef.h>

typedef rt_ubase_t bitmap_t;

#define BITMAP_BITS_MIN             (sizeof(bitmap_t) * 8)
#define BITMAP_LEN(bits)            (((bits) + (BITMAP_BITS_MIN) - 1) / (BITMAP_BITS_MIN))
#define BITMAP_BIT_LEN(nr)          (nr * BITMAP_BITS_MIN)
#define DECLARE_BITMAP(name, bits)  bitmap_t name[BITMAP_LEN(bits)]

#define __BIT(bit) (1UL << (bit))

rt_inline void bitmap_set_bit(bitmap_t *bitmap, rt_uint32_t bit)
{
    bitmap[bit / BITMAP_BITS_MIN] |= __BIT(bit & (BITMAP_BITS_MIN - 1));
}

rt_inline rt_bool_t bitmap_test_bit(bitmap_t *bitmap, int bit)
{
    return !!(bitmap[bit / BITMAP_BITS_MIN] & __BIT(bit & (BITMAP_BITS_MIN - 1)));
}

rt_inline void bitmap_clear_bit(bitmap_t *bitmap, rt_uint32_t bit)
{
    bitmap[bit / BITMAP_BITS_MIN] &= ~__BIT(bit & (BITMAP_BITS_MIN - 1));
}

#undef __BIT

rt_inline rt_size_t bitmap_next_set_bit(bitmap_t *bitmap, rt_size_t start, rt_size_t limit)
{
    rt_size_t bit;

    for (bit = start; bit < limit && !bitmap_test_bit(bitmap, bit); ++bit)
    {
    }

    return bit;
}

rt_inline rt_size_t bitmap_next_clear_bit(bitmap_t *bitmap, rt_size_t start, rt_size_t limit)
{
    rt_size_t bit;

    for (bit = start; bit < limit && bitmap_test_bit(bitmap, bit); ++bit)
    {
    }

    return bit;
}

#define bitmap_for_each_bit_from(state, bitmap, from, bit, limit)       \
    for ((bit) = bitmap_next_##state##_bit((bitmap), (from), (limit));  \
         (bit) < (limit);                                               \
         (bit) = bitmap_next_##state##_bit((bitmap), (bit + 1), (limit)))

#define bitmap_for_each_set_bit_from(bitmap, from, bit, limit) \
    bitmap_for_each_bit_from(set, bitmap, from, bit, limit)

#define bitmap_for_each_set_bit(bitmap, bit, limit) \
    bitmap_for_each_set_bit_from(bitmap, 0, bit, limit)

#define bitmap_for_each_clear_bit_from(bitmap, from, bit, limit) \
    bitmap_for_each_bit_from(clear, bitmap, from, bit, limit)

#define bitmap_for_each_clear_bit(bitmap, bit, limit) \
    bitmap_for_each_clear_bit_from(bitmap, 0, bit, limit)

#endif /* __UTIL_BITMAP_H__ */
