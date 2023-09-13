/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     the first version
 */

#ifndef __CRASH_CORE_H__
#define __CRASH_CORE_H__

#include <mmu.h>
#include <lwp_elf.h>

#define CRASH_CORE_NOTE_HEAD_BYTES  RT_ALIGN(sizeof(struct elf_note), 4)

#define VMCOREINFO_BYTES            ARCH_PAGE_SIZE
#define VMCOREINFO_NOTE_NAME        "VMCOREINFO"
#define VMCOREINFO_NOTE_NAME_BYTES  RT_ALIGN(sizeof(VMCOREINFO_NOTE_NAME), 4)
#define VMCOREINFO_NOTE_SIZE        ((CRASH_CORE_NOTE_HEAD_BYTES * 2) + VMCOREINFO_NOTE_NAME_BYTES + VMCOREINFO_BYTES)

void vmcoreinfo_append_str(const char *format, ...);
rt_ubase_t vmcoreinfo_note_paddr(void);

#define VMCOREINFO_OSRELEASE(value) \
        vmcoreinfo_append_str("OSRELEASE=%s\n", value)
#define VMCOREINFO_BUILD_ID() \
        vmcoreinfo_append_str("BUILD-ID=%40s\n", rtthread_build_id)
#define VMCOREINFO_PAGESIZE(value) \
        vmcoreinfo_append_str("PAGESIZE=%ld\n", value)
#define VMCOREINFO_SYMBOL(name) \
        vmcoreinfo_append_str("SYMBOL(%s)=%lx\n", #name, (rt_ubase_t)&name)
#define VMCOREINFO_SYMBOL_ARRAY(name) \
        vmcoreinfo_append_str("SYMBOL(%s)=%lx\n", #name, (rt_ubase_t)name)
#define VMCOREINFO_SIZE(name) \
        vmcoreinfo_append_str("SIZE(%s)=%lu\n", #name, (rt_ubase_t)sizeof(name))
#define VMCOREINFO_STRUCT_SIZE(name) \
        vmcoreinfo_append_str("SIZE(%s)=%lu\n", #name, (rt_ubase_t)sizeof(struct name))
#define VMCOREINFO_OFFSET(name, field) \
        vmcoreinfo_append_str("OFFSET(%s.%s)=%lu\n", #name, #field, (rt_ubase_t)&((struct name *)0)->field)
#define VMCOREINFO_TYPE_OFFSET(name, field) \
        vmcoreinfo_append_str("OFFSET(%s.%s)=%lu\n", #name, #field, (rt_ubase_t)&((name *)0)->field)
#define VMCOREINFO_LENGTH(name, value) \
        vmcoreinfo_append_str("LENGTH(%s)=%lu\n", #name, (rt_ubase_t)value)
#define VMCOREINFO_NUMBER(name) \
        vmcoreinfo_append_str("NUMBER(%s)=%ld\n", #name, (rt_base_t)name)
#define VMCOREINFO_CONFIG(name) \
        vmcoreinfo_append_str("CONFIG_%s=y\n", #name)

#endif /* __CRASH_CORE_H__ */
