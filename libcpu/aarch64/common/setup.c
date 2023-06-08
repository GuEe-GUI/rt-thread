/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-21     GuEe-GUI     first version
 */

#include <rtthread.h>

#define DBG_TAG "cpu.aa64"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <cpu.h>
#include <mmu.h>
#include <cpuport.h>
#include <interrupt.h>

#ifdef RT_USING_DFS
#include <dfs_fs.h>
#endif
#ifdef RT_USING_SMART
#include <lwp_arch.h>
#endif
#include <ioremap.h>
#include <drivers/pic.h>
#include <drivers/ofw.h>
#include <drivers/ofw_fdt.h>
#include <drivers/ofw_raw.h>
#include <drivers/core/rtdm.h>
#include <dt-bindings/size.h>

extern void _secondary_cpu_entry(void);
extern void rt_hw_builtin_fdt();

/* symbol in entry_point.S and link.ld */
extern rt_ubase_t _start, _end;

extern void *system_vectors;

static void *fdt_ptr = RT_NULL;
static rt_size_t fdt_size = 0;
static rt_uint64_t initrd_ranges[3] = { };

#ifdef RT_USING_SMP
extern struct cpu_ops_t cpu_psci_ops;
extern struct cpu_ops_t cpu_spin_table_ops;
#else
extern int rt_hw_cpu_id(void);
#endif

rt_uint64_t rt_cpu_mpidr_table[] =
{
    [RT_CPUS_NR] = 0,
};

static struct cpu_ops_t *cpu_ops[] =
{
#ifdef RT_USING_SMP
    &cpu_psci_ops,
    &cpu_spin_table_ops,
#endif
};

static struct rt_ofw_node *cpu_np[RT_CPUS_NR] = { };

void rt_hw_fdt_install_early(void *fdt)
{
    if (fdt != RT_NULL && !fdt_check_header(fdt))
    {
        fdt_ptr = fdt;
        fdt_size = fdt_totalsize(fdt_ptr);
    }
}

rt_err_t rt_fdt_boot_dump(void)
{
    rt_uint64_t mpidr, midr_el1;

    sysreg_read(mpidr_el1, mpidr);
    sysreg_read(midr_el1, midr_el1);

    LOG_I("Booting RT-Thread on physical CPU 0x%010x [0x%08x]", mpidr & MPIDR_AFFINITY_MASK, midr_el1);

    return RT_EOK;
}

void rt_hw_console_output(const char *str)
{
    rt_fdt_earlycon_output(str);
}

rt_weak void rt_hw_idle_wfi(void)
{
    __asm__ volatile ("wfi");
}

static void system_vectors_init(void)
{
    rt_hw_set_current_vbar((rt_ubase_t)&system_vectors);
}

static void cpu_info_init(void)
{
    int i = 0;
    rt_uint64_t mpidr;
    struct rt_ofw_node *np;

    /* get boot cpu info */
    sysreg_read(mpidr_el1, mpidr);

    rt_ofw_foreach_cpu_node(np)
    {
        rt_uint64_t hwid = rt_ofw_get_cpu_hwid(np, 0);

        if ((mpidr & MPIDR_AFFINITY_MASK) != hwid)
        {
            /* Only save affinity and res make smp boot can check */
            hwid |= 1ULL << 31;
        }
        else
        {
            hwid = mpidr;

            sysreg_write(tpidr_el1, i);
        }

        cpu_np[i] = np;
        rt_cpu_mpidr_table[i] = hwid;

        rt_ofw_data(np) = (void *)hwid;

        for (int idx = 0; idx < RT_ARRAY_SIZE(cpu_ops); ++idx)
        {
            struct cpu_ops_t *ops = cpu_ops[idx];

            if (ops->cpu_init)
            {
                ops->cpu_init(i, np);
            }
        }

        if (++i >= RT_CPUS_NR)
        {
            break;
        }
    }

    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, rt_cpu_mpidr_table, sizeof(rt_cpu_mpidr_table));
}

void rt_hw_common_setup(void)
{
    const char *bootargs;
    rt_ubase_t aspace_base;
    rt_size_t mem_region_nr;
    rt_region_t *mem_region;
    rt_size_t page_best_start;
    rt_region_t platform_mem_region;
    static struct mem_desc platform_mem_desc;
    void *kernel_start, *kernel_end, *memheap_start = RT_NULL, *memheap_end = RT_NULL;

    system_vectors_init();

#ifdef RT_USING_SMART
    aspace_base = 0 - ((rt_ubase_t)ARCH_ASPACE_SIZE + 1);
#else
    aspace_base = 0xffffd0000000;
#endif

    /* Init kernel address space */
    rt_hw_mmu_map_init(&rt_kernel_space, (void *)aspace_base, (rt_ubase_t)ARCH_ASPACE_SIZE + 1, MMUTable);

    /* Image ARM64 header is 64 bytes */
    kernel_start = rt_kmem_v2p((void *)&_start) - 64;
    kernel_end = rt_kmem_v2p((void *)&_end);

    if (!rt_fdt_commit_memregion_request(&mem_region, &mem_region_nr, RT_TRUE))
    {
        const char *name = "memheap";

        while (mem_region_nr --> 0)
        {
            if (mem_region->name == name || !rt_strcmp(mem_region->name, name))
            {
                memheap_start = (void *)mem_region->start;
                memheap_end = (void *)mem_region->end;

                break;
            }
        }
    }

    /* We maybe use the area in end of memheap as page pool */
    page_best_start = (rt_size_t)(memheap_end ? : kernel_end);

#ifdef RT_USING_BUILTIN_FDT
    fdt_ptr = &rt_hw_builtin_fdt;
    fdt_size = fdt_totalsize(fdt_ptr);
#else
    /*
     * The bootloader doesn't know the region of our memheap so maybe load the
     * device tree to the memheap, it's safety to move to the end of memheap
     */
    if (memheap_end && fdt_ptr > kernel_start)
    {
        rt_memmove(memheap_end, fdt_ptr - PV_OFFSET, fdt_size);

        fdt_ptr = memheap_end;

        /* Now, we use page in the end of fdt */
        page_best_start = (rt_size_t)fdt_ptr + fdt_size;
    }

    /* Reserved fdt region */
    rt_fdt_commit_memregion_early(&(rt_region_t)
    {
        .name = "fdt",
        .start = (rt_size_t)fdt_ptr,
        .end = (rt_size_t)(fdt_ptr + fdt_size),
    }, RT_TRUE);

    /* Fixup fdt pointer to kernel address space */
    fdt_ptr -= PV_OFFSET;

#endif /* !RT_USING_BUILTIN_FDT */

    /* Reserved kernel image region */
    rt_fdt_commit_memregion_early(&(rt_region_t)
    {
        .name = "kernel",
        .start = (rt_size_t)kernel_start,
        .end = (rt_size_t)kernel_end,
    }, RT_TRUE);

    if (rt_fdt_prefetch(fdt_ptr))
    {
        /* Platform cannot be initialized */
        RT_ASSERT(0);
    }

    /* Setup earlycon */
    rt_fdt_scan_chosen_stdout();

    rt_fdt_scan_initrd(initrd_ranges);

    /* Reserved your memory block before here */
    rt_fdt_scan_memory();

    /* Init system memheap */
    if (memheap_start && memheap_end)
    {
        rt_system_heap_init(memheap_start - PV_OFFSET, memheap_end - PV_OFFSET);
    }

    /* Find SoC memory limit */
    platform_mem_region.start = ~0UL;
    platform_mem_region.end = 0;

    if (!rt_fdt_commit_memregion_request(&mem_region, &mem_region_nr, RT_TRUE))
    {
        LOG_I("Reserved memory:");

        while (mem_region_nr --> 0)
        {
            if (platform_mem_region.start > mem_region->start)
            {
                platform_mem_region.start = mem_region->start;
            }

            if (platform_mem_region.end < mem_region->end)
            {
                platform_mem_region.end = mem_region->end;
            }

            LOG_I("  %-*.s [%p, %p]", RT_NAME_MAX, mem_region->name, mem_region->start, mem_region->end);

            ++mem_region;
        }
    }

    if (!rt_fdt_commit_memregion_request(&mem_region, &mem_region_nr, RT_FALSE))
    {
        rt_ubase_t best_offset = ~0UL;
        rt_region_t *usable_mem_region = mem_region, *page_region = RT_NULL, init_page_region = { 0 };

        LOG_I("Usable memory:");

        /* Now, we will find the best page region by for each usable memory */
        for (int i = 0; i < mem_region_nr; ++i, ++mem_region)
        {
            if (!mem_region->name)
            {
                continue;
            }

            if (platform_mem_region.start > mem_region->start)
            {
                platform_mem_region.start = mem_region->start;
            }

            if (platform_mem_region.end < mem_region->end)
            {
                platform_mem_region.end = mem_region->end;
            }

            if (mem_region->start >= page_best_start &&
                mem_region->start - page_best_start < best_offset &&
                /* MUST >= 1MB */
                mem_region->end - mem_region->start >= SIZE_MB)
            {
                page_region = mem_region;

                best_offset = page_region->start - page_best_start;
            }

            LOG_I("  %-*.s [%p, %p]", RT_NAME_MAX, mem_region->name, mem_region->start, mem_region->end);
        }

        /* Init kernel page pool */
        RT_ASSERT(page_region != RT_NULL);

        init_page_region.start = page_region->start - PV_OFFSET;
        init_page_region.end = page_region->end - PV_OFFSET;
        rt_page_init(init_page_region);

        /* Init mmu address space config */
        RT_ASSERT(platform_mem_region.end - platform_mem_region.start != 0);

        platform_mem_desc.paddr_start = platform_mem_region.start;
        platform_mem_desc.vaddr_start = platform_mem_region.start - PV_OFFSET;
        platform_mem_desc.vaddr_end = platform_mem_region.end - PV_OFFSET - 1;
        platform_mem_desc.attr = NORMAL_MEM;
        rt_hw_mmu_setup(&rt_kernel_space, &platform_mem_desc, 1);

        /* MMU config was changed, update the mmio map in earlycon */
        rt_fdt_earlycon_kick(FDT_EARLYCON_KICK_UPDATE);

        /* Install all usable memory to memory system */
        mem_region = usable_mem_region;

        for (int i = 0; i < mem_region_nr; ++i, ++mem_region)
        {
            if (mem_region != page_region)
            {
                rt_page_install(*mem_region);
            }
        }
    }

    rt_fdt_unflatten();

    cpu_info_init();

    /* Init hardware interrupt */
    rt_pic_init();
    rt_hw_interrupt_init();

    if (!rt_ofw_prop_read_string(rt_ofw_find_node_by_path("/chosen"), "bootargs", &bootargs))
    {
        LOG_I("Command line: %s", bootargs);
    }

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
    rt_ofw_console_setup();
#endif

    rt_thread_idle_sethook(rt_hw_idle_wfi);

#ifdef RT_USING_SMP
    /* install IPI handle */
    rt_hw_ipi_handler_install(RT_SCHEDULE_IPI, rt_scheduler_ipi_handler);
    rt_hw_ipi_handler_install(RT_STOP_IPI, rt_scheduler_ipi_handler);
    rt_hw_interrupt_umask(RT_SCHEDULE_IPI);
    rt_hw_interrupt_umask(RT_STOP_IPI);
#endif
}

#ifdef RT_USING_DFS
static int rootfs_mnt_init(void)
{
    void *fsdata = RT_NULL;
    const char *dev = rt_ofw_bootargs_select("root=", 0);
    const char *fstype = rt_ofw_bootargs_select("rootfstype=", 0);

    if ((!dev || !fstype) && initrd_ranges[0] && initrd_ranges[1])
    {
        void *base = (void *)initrd_ranges[0];
        size_t size = (void *)initrd_ranges[1] - base;

        fsdata = rt_ioremap(base, size);

        if (fsdata)
        {
            fstype = "cpio";
            initrd_ranges[3] = (rt_uint64_t)fsdata;
        }
    }

    if (fstype)
    {
        if (!dfs_mount(dev, "/", fstype, 0, fsdata))
        {
            LOG_I("Mount root %s%s type=%s %s", dev ? "on /dev/" : "", dev ? dev : "", fstype, "done");
        }
        else
        {
            LOG_W("Mount root %s%s type=%s %s", dev ? "on /dev/" : "", dev ? dev : "", fstype, "fail");
        }
    }

    return 0;
}
INIT_ENV_EXPORT(rootfs_mnt_init);
#endif /* RT_USING_DFS */

#ifdef RT_USING_SMP
rt_weak void rt_hw_secondary_cpu_up(void)
{
    int cpu_id = rt_hw_cpu_id();
    rt_uint64_t entry = (rt_uint64_t)rt_kmem_v2p(_secondary_cpu_entry);

    if (!entry)
    {
        LOG_E("Failed to translate '_secondary_cpu_entry' to physical address");
        RT_ASSERT(0);
    }

    /* Maybe we are no in the first cpu */
    for (int i = 0; i < RT_ARRAY_SIZE(cpu_np); ++i)
    {
        int err;
        const char *enable_method;

        if (!cpu_np[i] || i == cpu_id)
        {
            continue;
        }

        err = rt_ofw_prop_read_string(cpu_np[i], "enable-method", &enable_method);

        for (int idx = 0; !err && idx < RT_ARRAY_SIZE(cpu_ops); ++idx)
        {
            struct cpu_ops_t *ops = cpu_ops[idx];

            if (ops->method && !rt_strcmp(ops->method, enable_method) && ops->cpu_boot)
            {
                err = ops->cpu_boot(i, entry);

                break;
            }
        }

        if (err)
        {
            LOG_W("Call cpu %d on %s", i, "failed");
        }
    }
}

rt_weak void secondary_cpu_c_start(void)
{
    int cpu_id = rt_hw_cpu_id();

    system_vectors_init();

    rt_hw_spin_lock(&_cpus_lock);

    /* Save all mpidr */
    sysreg_read(mpidr_el1, rt_cpu_mpidr_table[cpu_id]);

    rt_hw_mmu_ktbl_set((unsigned long)MMUTable);

    rt_hw_interrupt_init();

    rt_dm_secondary_cpu_init();
    rt_hw_interrupt_umask(RT_SCHEDULE_IPI);
    rt_hw_interrupt_umask(RT_STOP_IPI);

    LOG_I("Call cpu %d on %s", cpu_id, "success");

    rt_system_scheduler_start();
}

rt_weak void rt_hw_secondary_cpu_idle_exec(void)
{
    rt_hw_wfe();
}
#endif /* RT_USING_SMP */
