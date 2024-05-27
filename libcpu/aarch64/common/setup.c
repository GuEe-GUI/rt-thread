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
#ifdef RT_USING_DFS_DIRECTFS
#include <dfs_directfs.h>
#endif
#ifdef RT_USING_FINSH
#include <msh.h>
#endif
#endif
#ifdef RT_USING_SMART
#include <lwp_arch.h>
#endif
#include <stdlib.h>
#include <ioremap.h>
#include <drivers/pic.h>
#include <drivers/ofw.h>
#include <drivers/ofw_fdt.h>
#include <drivers/ofw_raw.h>
#include <drivers/core/rtdm.h>
#include <dt-bindings/size.h>

#define phys_to_virt(pa)    ((pa) - PV_OFFSET)

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

static void read_cpuid(char *ids, rt_uint64_t midr)
{
    const char *id;
#define IS_CPU(ID, NAME)  case MIDR_##ID: id = NAME; break
    switch (midr & ~(MIDR_VARIANT_MASK | MIDR_REVISION_MASK))
    {
    IS_CPU(CORTEX_A53, "Cortex-A53");
    IS_CPU(CORTEX_A57, "Cortex-A57");
    IS_CPU(CORTEX_A72, "Cortex-A72");
    IS_CPU(CORTEX_A73, "Cortex-A73");
    IS_CPU(CORTEX_A75, "Cortex-A75");
    IS_CPU(CORTEX_A35, "Cortex-A35");
    IS_CPU(CORTEX_A55, "Cortex-A55");
    IS_CPU(CORTEX_A76, "Cortex-A76");
    IS_CPU(NEOVERSE_N1, "Neoverse-N1");
    IS_CPU(CORTEX_A77, "Cortex-A77");
    IS_CPU(NEOVERSE_V1, "Neoverse-V1");
    IS_CPU(CORTEX_A78, "Cortex-A78");
    IS_CPU(CORTEX_A78AE, "Cortex-A78AE");
    IS_CPU(CORTEX_X1, "Cortex-X1");
    IS_CPU(CORTEX_A510, "Cortex-A510");
    IS_CPU(CORTEX_A710, "Cortex-A710");
    IS_CPU(CORTEX_A715, "Cortex-A715");
    IS_CPU(CORTEX_X2, "Cortex-X2");
    IS_CPU(NEOVERSE_N2, "Neoverse-N2");
    IS_CPU(CORTEX_A78C, "Cortex-A78C");
    IS_CPU(THUNDERX, "Thunderx");
    IS_CPU(THUNDERX_81XX, "Thunderx-81XX");
    IS_CPU(THUNDERX_83XX, "Thunderx-83XX");
    IS_CPU(OCTX2_98XX, "Octx2-98XX");
    IS_CPU(OCTX2_96XX, "Octx2-96XX");
    IS_CPU(OCTX2_95XX, "Octx2-95XX");
    IS_CPU(OCTX2_95XXN, "Octx2-95XXN");
    IS_CPU(OCTX2_95XXMM, "Octx2-95XXMM");
    IS_CPU(OCTX2_95XXO, "Octx2-95XXO");
    IS_CPU(CAVIUM_THUNDERX2, "Cavium-Thunderx2");
    IS_CPU(BRAHMA_B53, "Brahma-B53");
    IS_CPU(BRCM_VULCAN, "Brcm-Vulcan");
    IS_CPU(QCOM_FALKOR, "Qcom-Falkor");
    IS_CPU(QCOM_KRYO, "Qcom-Kryo");
    IS_CPU(QCOM_KRYO_2XX_GOLD, "Qcom-Kryo-2XX-Gold");
    IS_CPU(QCOM_KRYO_2XX_SILVER, "Qcom-Kryo-2XX-Silver");
    IS_CPU(QCOM_KRYO_3XX_SILVER, "qcom-Kryo-3XX-Silver");
    IS_CPU(QCOM_KRYO_4XX_GOLD, "Qcom-Kryo-4XX-Gold");
    IS_CPU(QCOM_KRYO_4XX_SILVER, "Qcom-Kryo-4XX-Silver");
    IS_CPU(NVIDIA_DENVER, "Nvidia_Denver");
    IS_CPU(NVIDIA_CARMEL, "Nvidia_Carmel");
    IS_CPU(FUJITSU_A64FX, "Fujitsu_A64FX");
    IS_CPU(HISI_TSV110, "Hisi_Tsv110");
    IS_CPU(APPLE_M1_ICESTORM, "Apple-M1-Icestorm");
    IS_CPU(APPLE_M1_FIRESTORM, "Apple-M1-Firestorm");
    IS_CPU(APPLE_M1_ICESTORM_PRO, "Apple-M1-Icestorm-Pro");
    IS_CPU(APPLE_M1_FIRESTORM_PRO, "Apple-M1-Firestorm-Pro");
    IS_CPU(APPLE_M1_ICESTORM_MAX, "Apple-M1-Icestorm-Max");
    IS_CPU(APPLE_M1_FIRESTORM_MAX, "Apple-M1-Firestorm-Max");
    IS_CPU(APPLE_M2_BLIZZARD, "Apple-M2-Blizzard");
    IS_CPU(APPLE_M2_AVALANCHE, "Apple-M2-Avalanche");
    IS_CPU(APPLE_M2_BLIZZARD_PRO, "Apple-M2-Blizzard-Pro");
    IS_CPU(APPLE_M2_AVALANCHE_PRO, "Apple-M2-Avalanche-Pro");
    IS_CPU(APPLE_M2_BLIZZARD_MAX, "Apple-M2-Blizzard-Max");
    IS_CPU(APPLE_M2_AVALANCHE_MAX, "Apple-M2-Avalanche-Max");
    IS_CPU(AMPERE1, "Ampere1");
    IS_CPU(QEMU, "Qemu");
    default:
        id = "ARMv8";
        break;
    }

    if (midr & (MIDR_VARIANT_MASK | MIDR_REVISION_MASK))
    {
        rt_sprintf(ids, "%s v%dr%d", id,
                (midr & MIDR_VARIANT_MASK) >> MIDR_VARIANT_SHIFT,
                (midr & MIDR_REVISION_MASK) >> MIDR_REVISION_SHIFT);
    }
    else
    {
        rt_sprintf(ids, "%s", id);
    }
#undef IS_CPU
}

rt_err_t rt_fdt_boot_dump(void)
{
    rt_uint64_t mpidr, midr;

    sysreg_read(mpidr_el1, mpidr);
    sysreg_read(midr_el1, midr);

    LOG_I("Booting RT-Thread on physical CPU 0x%010x [0x%08x]", mpidr & MPIDR_AFFINITY_MASK, midr);

    return RT_EOK;
}

void rt_hw_console_output(const char *str)
{
    rt_fdt_earlycon_output(str);
}

#ifdef RT_USING_HWTIMER
static rt_ubase_t loops_per_tick[RT_CPUS_NR];

static rt_ubase_t cpu_get_cycles(void)
{
    rt_ubase_t cycles;

    sysreg_read(cntpct_el0, cycles);

    return cycles;
}

static void cpu_loops_per_tick_init(void)
{
    rt_ubase_t offset;
    volatile rt_ubase_t freq, step, cycles_end1, cycles_end2;
    volatile rt_uint32_t cycles_count1 = 0, cycles_count2 = 0;

    sysreg_read(cntfrq_el0, freq);
    step = freq / RT_TICK_PER_SECOND;

    cycles_end1 = cpu_get_cycles() + step;

    while (cpu_get_cycles() < cycles_end1)
    {
        __asm__ volatile ("nop");
        __asm__ volatile ("add %0, %0, #1":"=r"(cycles_count1));
    }

    cycles_end2 = cpu_get_cycles() + step;

    while (cpu_get_cycles() < cycles_end2)
    {
        __asm__ volatile ("add %0, %0, #1":"=r"(cycles_count2));
    }

    if ((rt_int32_t)(cycles_count2 - cycles_count1) > 0)
    {
        offset = cycles_count2 - cycles_count1;
    }
    else
    {
        /* Impossible, but prepared for any eventualities */
        offset = cycles_count2 / 4;
    }

    loops_per_tick[rt_hw_cpu_id()] = offset;
}

static void cpu_us_delay(rt_uint32_t us)
{
    volatile rt_base_t start = cpu_get_cycles(), cycles;

    cycles = ((us * 0x10c7UL) * loops_per_tick[rt_hw_cpu_id()] * RT_TICK_PER_SECOND) >> 32;

    while ((cpu_get_cycles() - start) < cycles)
    {
        rt_hw_cpu_relax();
    }
}
#endif /* RT_USING_HWTIMER */

rt_weak void rt_hw_idle_wfi(void)
{
    __asm__ volatile ("wfi");
}

static void system_vectors_init(void)
{
    rt_hw_set_current_vbar((rt_ubase_t)&system_vectors);
}

rt_inline void cpu_info_init(void)
{
    int i = 0;
    char cpuid[32];
    rt_uint64_t mpidr, midr;
    struct rt_ofw_node *np;

    /* get boot cpu info */
    sysreg_read(mpidr_el1, mpidr);
    sysreg_read(midr_el1, midr);

    read_cpuid(cpuid, midr);
    LOG_I("Boot Processor ID: %s", cpuid);

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

#ifdef RT_USING_HWTIMER
    cpu_loops_per_tick_init();

    if (!rt_device_hwtimer_us_delay)
    {
        rt_device_hwtimer_us_delay = &cpu_us_delay;
    }
#endif /* RT_USING_HWTIMER */
}

rt_inline void directfs_init(void)
{
#ifdef RT_USING_DFS_DIRECTFS
    static char *directfs_root[] =
    {
        "bus",
        "firmware",
    };
    static struct rt_object objs[RT_ARRAY_SIZE(directfs_root)];

    for (int i = 0; i < RT_ARRAY_SIZE(directfs_root); ++i)
    {
        dfs_directfs_create_link(RT_NULL, &objs[i], directfs_root[i]);
    }
#endif
}

rt_inline rt_bool_t is_kernel_aspace(const char *name)
{
    static char * const names[] =
    {
        "kernel",
        "memheap",
    };

    if (!name)
    {
        return RT_FALSE;
    }

    for (int i = 0; i < RT_ARRAY_SIZE(names); ++i)
    {
        if (!rt_strcmp(names[i], name))
        {
            return RT_TRUE;
        }
    }

    return RT_FALSE;
}

void rt_hw_common_setup(void)
{
    const char *bootargs;
    rt_ubase_t aspace_base;
    rt_size_t mem_region_nr;
    rt_region_t *mem_region;
    rt_size_t page_best_start;
    rt_region_t platform_mem_region;
    struct rt_ofw_node *ofw_chosen_node;
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

            ++mem_region;
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
     * device tree to the memheap, it's safety to move to the end of the memheap
     */
    if (memheap_end && fdt_ptr > kernel_start)
    {
        rt_memmove(phys_to_virt(memheap_end), phys_to_virt(fdt_ptr), fdt_size);

        fdt_ptr = memheap_end;

        /* Now, we use the page in the end of the fdt */
        page_best_start = (rt_size_t)fdt_ptr + fdt_size;
    }

    /* Reserved the fdt region */
    rt_fdt_commit_memregion_early(&(rt_region_t)
    {
        .name = "fdt",
        .start = (rt_size_t)fdt_ptr,
        .end = (rt_size_t)(fdt_ptr + fdt_size),
    }, RT_TRUE);

    /* Fixup the fdt pointer to kernel address space */
    fdt_ptr = phys_to_virt(fdt_ptr);
#endif /* !RT_USING_BUILTIN_FDT */

    /* Reserved the kernel image region */
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

    /* Init the system memheap */
    if (memheap_start && memheap_end)
    {
        rt_system_heap_init(phys_to_virt(memheap_start), phys_to_virt(memheap_end));
    }

    /* Find the SoC memory limit */
    platform_mem_region.start = ~0UL;
    platform_mem_region.end = 0;

    if (!rt_fdt_commit_memregion_request(&mem_region, &mem_region_nr, RT_TRUE))
    {
        LOG_I("Reserved memory:");

        while (mem_region_nr --> 0)
        {
            if (is_kernel_aspace(mem_region->name))
            {
                if (platform_mem_region.start > mem_region->start)
                {
                    platform_mem_region.start = mem_region->start;
                }

                if (platform_mem_region.end < mem_region->end)
                {
                    platform_mem_region.end = mem_region->end;
                }
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

        /* Now, we will find the best page region by for each the usable memory */
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

        /* Init the kernel page pool */
        RT_ASSERT(page_region != RT_NULL);


        init_page_region.start = phys_to_virt(page_region->start);
        init_page_region.end = phys_to_virt(page_region->end);
        rt_page_init(init_page_region);

        /* Init the mmu address space config */
        platform_mem_region.start = RT_ALIGN(platform_mem_region.start, ARCH_PAGE_SIZE);
        platform_mem_region.end = RT_ALIGN_DOWN(platform_mem_region.end, ARCH_PAGE_SIZE);
        RT_ASSERT(platform_mem_region.end - platform_mem_region.start != 0);

        platform_mem_desc.paddr_start = platform_mem_region.start;
        platform_mem_desc.vaddr_start = phys_to_virt(platform_mem_region.start);
        platform_mem_desc.vaddr_end = phys_to_virt(platform_mem_region.end) - 1;
        platform_mem_desc.attr = NORMAL_MEM;

        rt_hw_mmu_setup(&rt_kernel_space, &platform_mem_desc, 1);

        /* MMU config was changed, update the mmio map in earlycon */
        rt_fdt_earlycon_kick(FDT_EARLYCON_KICK_UPDATE);

        /* Install all usable memory into memory system */
        mem_region = usable_mem_region;

        for (int i = 0; i < mem_region_nr; ++i, ++mem_region)
        {
            if (mem_region != page_region && mem_region->name)
            {
                rt_page_install(*mem_region);
            }
        }
    }

    directfs_init();

    rt_fdt_unflatten();

    cpu_info_init();

    /* Init the hardware interrupt */
    rt_pic_init();
    rt_hw_interrupt_init();

    ofw_chosen_node = rt_ofw_find_node_by_path("/chosen");

    if (!rt_ofw_prop_read_string(ofw_chosen_node, "bootargs", &bootargs))
    {
        LOG_I("Command line: %s", bootargs);
    }

    rt_ofw_node_put(ofw_chosen_node);

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
    rt_ofw_console_setup();
#endif

    rt_thread_idle_sethook(rt_hw_idle_wfi);

#ifdef RT_USING_SMP
    /* Install the IPI handle */
    rt_hw_ipi_handler_install(RT_SCHEDULE_IPI, rt_scheduler_ipi_handler);
    rt_hw_ipi_handler_install(RT_STOP_IPI, rt_scheduler_ipi_handler);
    rt_hw_interrupt_umask(RT_SCHEDULE_IPI);
    rt_hw_interrupt_umask(RT_STOP_IPI);
#endif
}

#ifdef RT_USING_DFS
static int rootfs_mnt_init(void)
{
    rt_err_t err = -RT_ERROR;
    void *fsdata = RT_NULL;
    const char *cpio_type = "cpio";
    const char *dev = rt_ofw_bootargs_select("root=", 0);
    const char *fstype = rt_ofw_bootargs_select("rootfstype=", 0);
    const char *rw = rt_ofw_bootargs_select("rw", 0);

    if ((!dev || !fstype) && initrd_ranges[0] && initrd_ranges[1])
    {
        void *base = (void *)initrd_ranges[0];
        size_t size = (void *)initrd_ranges[1] - base;

        fsdata = rt_ioremap(base, size);

        if (fsdata)
        {
            fstype = cpio_type;
            initrd_ranges[2] = (rt_uint64_t)fsdata;
        }
    }

    if (fstype != cpio_type && dev)
    {
        rt_tick_t timeout = 0;
        const char *rootwait, *rootdelay = RT_NULL;

        rootwait = rt_ofw_bootargs_select("rootwait", 0);

        /* Maybe it is undefined or 'rootwaitABC' */
        if (!rootwait || *rootwait)
        {
            rootdelay = rt_ofw_bootargs_select("rootdelay=", 0);

            if (rootdelay)
            {
                timeout = rt_tick_from_millisecond(atoi(rootdelay));
            }

            rootwait = RT_NULL;
        }

        /*
         * Delays in boot flow is a terrible behavior in RTOS, but the RT-Thread
         * SDIO framework init the devices in a task that we need to wait for
         * SDIO devices to init complete...
         *
         * WHAT THE F*CK PROBLEMS WILL HAPPENED?
         *
         * Your main PE, applications, services that depend on the root FS and
         * the multi cores setup, init will delay, too...
         *
         * So, you can try to link this function to `INIT_APP_EXPORT` even later
         * and remove the delays if you want to optimize the boot time and mount
         * the FS auto.
         */
        for (; rootdelay || rootwait; --timeout)
        {
            if (!rootwait && timeout == 0)
            {
                LOG_E("Wait for /dev/%s init time out", dev);

                /*
                 * We don't return at once because the device driver may init OK
                 * when we break from this point, might as well give it another
                 * try.
                 */
                break;
            }

            if (rt_device_find(dev))
            {
                break;
            }

            rt_thread_mdelay(1);
        }
    }

    if (fstype)
    {
        if (!(err = dfs_mount(dev, "/", fstype, rw ? 0 : ~0, fsdata)))
        {
            LOG_I("Mount root %s%s type=%s %s",
                (dev && *dev) ? "on /dev/" : "",
                (dev && *dev) ? dev : "\b",
                fstype, "done");
        }
        else
        {
            LOG_W("Mount root %s%s type=%s %s",
                (dev && *dev) ? "on /dev/" : "",
                (dev && *dev) ? dev : "\b",
                fstype, "fail");
        }
    }

    return 0;
}
INIT_ENV_EXPORT(rootfs_mnt_init);

static int fstab_mnt_init(void)
{
#ifdef RT_USING_DFS_DIRECTFS
    mkdir("/direct", 0755);

    if (!dfs_mount(RT_NULL, "/direct", "direct", 0, RT_NULL))
    {
        LOG_I("Mount %s %s%s type=%s %s", "direct", "/", "direct", "direct", "done");
    }
#endif

    mkdir("/mnt", 0755);

#ifdef RT_USING_FINSH
    /* Try mount by table */
    msh_exec_script("fstab.sh", 16);
#endif

    return 0;
}
INIT_FS_EXPORT(fstab_mnt_init);
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
            LOG_W("Call CPU%d on failed", i);
        }
    }
}

rt_weak void secondary_cpu_c_start(void)
{
    char cpuid[32];
    rt_uint64_t midr;
    int cpu_id = rt_hw_cpu_id();

    system_vectors_init();

    rt_hw_spin_lock(&_cpus_lock);

    /* Save all mpidr */
    sysreg_read(mpidr_el1, rt_cpu_mpidr_table[cpu_id]);
    sysreg_read(midr_el1, midr);

    rt_hw_mmu_ktbl_set((unsigned long)MMUTable);

#ifdef RT_USING_HWTIMER
    if (rt_device_hwtimer_us_delay == &cpu_us_delay)
    {
        cpu_loops_per_tick_init();
    }
#endif

    rt_hw_interrupt_init();

    rt_dm_secondary_cpu_init();
    rt_hw_interrupt_umask(RT_SCHEDULE_IPI);
    rt_hw_interrupt_umask(RT_STOP_IPI);

    read_cpuid(cpuid, midr);
    LOG_I("Call CPU%d [%s] on success", cpu_id, cpuid);

    rt_system_scheduler_start();
}

rt_weak void rt_hw_secondary_cpu_idle_exec(void)
{
    rt_hw_wfe();
}
#endif /* RT_USING_SMP */
