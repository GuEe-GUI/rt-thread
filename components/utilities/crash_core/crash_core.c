/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     the first version
 */

#define DBG_TAG "crash.core"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <mm_page.h>
#include <crash_core.h>

#ifndef DIV_ROUND_UP
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#endif

extern char __build_id_start;
extern char __build_id_end;

char rtthread_build_id[40] = {};

/* vmcoreinfo stuff */
rt_uint8_t *vmcoreinfo_data = RT_NULL;
rt_size_t vmcoreinfo_size = 0;
rt_uint32_t *vmcoreinfo_note = RT_NULL;

void vmcoreinfo_append_str(const char *format, ...)
{
    va_list args;
    rt_size_t r;
    char buf[0x50];
    const rt_size_t max_size = (rt_size_t)VMCOREINFO_BYTES - vmcoreinfo_size;

    va_start(args, format);
    r = rt_vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if (r > max_size)
    {
        r = max_size;
    }

    rt_memcpy(&vmcoreinfo_data[vmcoreinfo_size], buf, r);

    vmcoreinfo_size += r;

    if (vmcoreinfo_size == VMCOREINFO_BYTES)
    {
        LOG_W("vmcoreinfo data exceeds allocated size, truncating");
    }
}

rt_weak rt_ubase_t vmcoreinfo_note_paddr(void)
{
    void *ptr = vmcoreinfo_note;

#ifdef ARCH_MM_MMU
    ptr = rt_kmem_v2p(ptr);
#endif

    return (rt_ubase_t)ptr;
}

rt_weak void rt_hw_crash_save_vmcoreinfo(void)
{
}

static void update_vmcoreinfo_note(void)
{
    rt_uint32_t *buf;
    struct elf_note *note;

    if (!vmcoreinfo_size)
    {
        return;
    }

    buf = vmcoreinfo_note;
    note = (struct elf_note *)buf;

    note->n_namesz = rt_strlen(VMCOREINFO_NOTE_NAME) + 1;
    note->n_descsz = vmcoreinfo_size;
    note->n_type = 0;
    buf += DIV_ROUND_UP(sizeof(*note), sizeof(Elf_Word));
    rt_memcpy(buf, VMCOREINFO_NOTE_NAME, note->n_namesz);
    buf += DIV_ROUND_UP(note->n_namesz, sizeof(Elf_Word));
    rt_memcpy(buf, vmcoreinfo_data, vmcoreinfo_size);
    buf += DIV_ROUND_UP(vmcoreinfo_size, sizeof(Elf_Word));

    rt_memset(buf, 0, sizeof(struct elf_note));
}

static int crash_save_vmcoreinfo_init(void)
{
    const char *build_id;

    vmcoreinfo_data = rt_pages_alloc(rt_page_bits(ARCH_PAGE_SIZE));

    if (!vmcoreinfo_data)
    {
        LOG_W("Memory allocation for vmcoreinfo_%s failed", "data");

        return -RT_ENOMEM;
    }

    vmcoreinfo_note = rt_pages_alloc(rt_page_bits(ARCH_PAGE_SIZE));

    if (!vmcoreinfo_note)
    {
        rt_pages_free(vmcoreinfo_data, rt_page_bits(ARCH_PAGE_SIZE));
        vmcoreinfo_data = RT_NULL;

        LOG_W("Memory allocation for vmcoreinfo_%s failed", "note");

        return -RT_ENOMEM;
    }

    rt_memset(vmcoreinfo_data, 0, ARCH_PAGE_SIZE);

    /* Skip GNU header (16 bytes) */
    build_id = &__build_id_start + 16;

    for (int i = 0; build_id < &__build_id_end; ++build_id)
    {
        const char *hex = "0123456789abcdef";

        rtthread_build_id[i++] = hex[*build_id >> 4];
        rtthread_build_id[i++] = hex[*build_id & 0xf];
    }

    VMCOREINFO_BUILD_ID();
    VMCOREINFO_PAGESIZE(ARCH_PAGE_SIZE);

    rt_hw_crash_save_vmcoreinfo();
    update_vmcoreinfo_note();

    return 0;
}
INIT_BOARD_EXPORT(crash_save_vmcoreinfo_init);
