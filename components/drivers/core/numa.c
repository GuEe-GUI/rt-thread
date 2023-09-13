/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-24     GuEe-GUI     the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "rtdm.numa"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

struct numa_memory
{
    rt_list_t list;

    int nid;
    rt_ubase_t start;
    rt_ubase_t end;

    union
    {
        void *ofw_node;
    };
};

static int cpu_numa_map[RT_CPUS_NR] =
{
    [0 ... RT_CPUS_NR - 1] = -RT_ENOSYS,
};
static rt_list_t numa_memory_nodes = RT_LIST_OBJECT_INIT(numa_memory_nodes);

int rt_numa_cpu_id(int cpuid)
{
    return cpuid < RT_ARRAY_SIZE(cpu_numa_map) ? cpu_numa_map[cpuid] : -RT_EINVAL;
}

int rt_numa_device_id(struct rt_device *dev)
{
    rt_uint32_t nid = (rt_uint32_t)-RT_ENOSYS;

    return rt_dm_dev_prop_read_u32(dev, "numa-node-id", &nid) ? : (int)nid;
}

rt_err_t rt_numa_memory_affinity(rt_ubase_t phy_addr, bitmap_t *out_affinity)
{
    struct numa_memory *nm;

    if (!out_affinity)
    {
        return -RT_EINVAL;
    }

    rt_memset(out_affinity, 0, sizeof(*out_affinity) * BITMAP_LEN(RT_CPUS_NR));

    rt_list_for_each_entry(nm, &numa_memory_nodes, list)
    {
        if (phy_addr >= nm->start && phy_addr < nm->end)
        {
            for (int i = 0; i < RT_ARRAY_SIZE(cpu_numa_map); ++i)
            {
                if (cpu_numa_map[i] == nm->nid)
                {
                    bitmap_set_bit(out_affinity, i);
                }
            }

            return RT_EOK;
        }
    }

    return -RT_EEMPTY;
}

#ifdef RT_USING_OFW
static int numa_ofw_init(void)
{
    int i = 0;
    rt_uint32_t nid;
    const char *numa_conf;
    struct rt_ofw_node *np = RT_NULL;

    numa_conf = rt_ofw_bootargs_select("numa=", 0);

    if (!numa_conf || rt_strcmp(numa_conf, "on"))
    {
        return (int)RT_EOK;
    }

    rt_ofw_foreach_cpu_node(np)
    {
        rt_ofw_prop_read_u32(np, "numa-node-id", (rt_uint32_t *)&cpu_numa_map[i]);

        if (++i >= RT_CPUS_NR)
        {
            break;
        }
    }

    rt_ofw_foreach_node_by_type(np, "memory")
    {
        if (!rt_ofw_prop_read_u32(np, "numa-node-id", &nid))
        {
            int mem_nr = rt_ofw_get_address_count(np);

            for (i = 0; i < mem_nr; ++i)
            {
                rt_uint64_t addr, size;
                struct numa_memory *nm;

                if (rt_ofw_get_address(np, i, &addr, &size))
                {
                    continue;
                }

                nm = rt_malloc(sizeof(*nm));

                if (rt_unlikely(!nm))
                {
                    LOG_E("No memory to record NUMA[%d] memory[%p, %p] info",
                            nid, addr, addr + size);

                    return (int)-RT_ENOMEM;
                }

                nm->start = (rt_ubase_t)addr;
                nm->end = (rt_ubase_t)(addr + size);
                nm->ofw_node = np;

                rt_list_init(&nm->list);
                rt_list_insert_before(&numa_memory_nodes, &nm->list);
            }
        }
    }

    return 0;
}
INIT_CORE_EXPORT(numa_ofw_init);
#endif /* RT_USING_OFW */
