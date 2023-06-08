/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-24     GuEe-GUI     first version
 */

#include <rthw.h>
#include <rtthread.h>

#define DBG_TAG "irqchip.bcm2836"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <cpuport.h>

#include <bitmap.h>

#include "pic-bcm2836.h"

struct bcm2836_arm_intc
{
    struct rt_pic parent;

    void *base;
};

static struct bcm2836_arm_intc intc;

static void bcm2836_arm_irqchip_mask_ipi(rt_uint32_t reg_offset, rt_uint32_t bit, int cpu)
{
    void *reg = intc.base + reg_offset + 4 * cpu;

    HWREG32(reg) &= ~RT_BIT(bit);
}

static void bcm2836_arm_irqchip_unmask_ipi(rt_uint32_t reg_offset, rt_uint32_t bit, int cpu)
{
    void *reg = intc.base + reg_offset + 4 * cpu;

    HWREG32(reg) |= RT_BIT(bit);
}

static void bcm2836_arm_irqchip_mask_timer_irq(int hwirq)
{
    bcm2836_arm_irqchip_mask_ipi(LOCAL_TIMER_INT_CONTROL0, hwirq - LOCAL_IRQ_CNTPSIRQ, rt_hw_cpu_id());
}

static void bcm2836_arm_irqchip_unmask_timer_irq(int hwirq)
{
    bcm2836_arm_irqchip_unmask_ipi(LOCAL_TIMER_INT_CONTROL0, hwirq - LOCAL_IRQ_CNTPSIRQ, rt_hw_cpu_id());
}

static void bcm2836_arm_irqchip_mask_gpu_irq(void)
{
}

static void bcm2836_arm_irqchip_unmask_gpu_irq(void)
{
}

static void bcm2836_arm_irqchip_mask_pmu_irq(void)
{
    HWREG32(intc.base + LOCAL_PM_ROUTING_CLR) = 1 << rt_hw_cpu_id();
}

static void bcm2836_arm_irqchip_unmask_pmu_irq(void)
{
    HWREG32(intc.base + LOCAL_PM_ROUTING_SET) = 1 << rt_hw_cpu_id();
}

static void bcm2836_arm_irqchip_irq_ops(int hwirq, rt_bool_t mask)
{
#define CALL_OPS(name, ...)                                 \
    if (mask) {                                             \
        bcm2836_arm_irqchip_mask_##name(__VA_ARGS__);       \
    } else {                                                \
        bcm2836_arm_irqchip_unmask_##name(__VA_ARGS__);     \
    }

    switch (hwirq)
    {
    case LOCAL_IRQ_MAILBOX0:
    case LOCAL_IRQ_MAILBOX1:
    case LOCAL_IRQ_MAILBOX2:
    case LOCAL_IRQ_MAILBOX3:
        CALL_OPS(ipi, LOCAL_MAILBOX_INT_CONTROL0, 0, rt_hw_cpu_id());
        break;
    case LOCAL_IRQ_CNTPSIRQ:
    case LOCAL_IRQ_CNTPNSIRQ:
    case LOCAL_IRQ_CNTHPIRQ:
    case LOCAL_IRQ_CNTVIRQ:
        CALL_OPS(timer_irq, hwirq);
        break;
    case LOCAL_IRQ_GPU_FAST:
        CALL_OPS(gpu_irq);
        break;
    case LOCAL_IRQ_PMU_FAST:
        CALL_OPS(pmu_irq);
        break;
    default:
        LOG_W("Unexpected hwirq = %d", hwirq);
        break;
    }
}

static void bcm2836_arm_l1_intc_irq_ack(struct rt_pic_irq *pirq)
{
    switch (pirq->hwirq)
    {
    case LOCAL_IRQ_MAILBOX0:
    case LOCAL_IRQ_MAILBOX1:
    case LOCAL_IRQ_MAILBOX2:
    case LOCAL_IRQ_MAILBOX3:
        HWREG32(intc.base + LOCAL_MAILBOX0_CLR0 + 16 * rt_hw_cpu_id()) = RT_BIT(data->hwirq);
        break;
    default:
        break;
    }
}

static void bcm2836_arm_l1_intc_irq_mask(struct rt_pic_irq *pirq)
{
    bcm2836_arm_irqchip_irq_ops(data->hwirq, RT_TRUE);
}

static void bcm2836_arm_l1_intc_irq_unmask(struct rt_pic_irq *pirq)
{
    bcm2836_arm_irqchip_irq_ops(pirq->hwirq, RT_FALSE);
}

static void bcm2836_arm_l1_intc_irq_send_ipi(struct rt_pic_irq *pirq, bitmap_t *cpumask)
{
    int cpu, hwirq = pirq->hwirq;
    void *mailbox0_base = intc.base + LOCAL_MAILBOX0_SET0;

    /*
     * Ensure that stores to normal memory are visible to the other CPUs before
     * issuing the IPI.
     */
    rt_hw_wmb();

    bitmap_for_each_set_bit(cpumask, cpu, RT_CPUS_NR)
    {
        HWREG32(mailbox0_base + 16 * cpu) = RT_BIT(hwirq);
    }
}

static int bcm2836_arm_l1_intc_irq_map(struct rt_irq_chip_desc *chip_desc, int hwirq, struct rt_irq_info *init_info)
{
    return rt_pic_config_irq(pic, hwirq, hwirq);
}

static rt_err_t bcm2836_arm_l1_intc_irq_parse(struct rt_pic *pic,
        struct rt_ofw_cell_args *args, struct rt_pic_irq *out_pirq)
{
    rt_err_t ret = RT_EOK;

    if (args->args_count == 2)
    {
        out_pirq->hwirq = args->args[0];
        out_pirq->mode = args->args[1];
    }
    else
    {
        ret = -RT_EINVAL;
    }

    return ret;
}

static struct rt_pic_ops bcm2836_arm_l1_intc_ops =
{
    .name = "BCM2836-ARM-L1-INTC",
    .irq_ack = bcm2836_arm_l1_intc_irq_ack,
    .irq_mask = bcm2836_arm_l1_intc_irq_mask,
    .irq_unmask = bcm2836_arm_l1_intc_irq_unmask,
    .irq_send_ipi = bcm2836_arm_l1_intc_irq_send_ipi,
    .irq_map = bcm2836_arm_l1_intc_irq_map,
    .irq_parse = bcm2836_arm_l1_intc_irq_parse,
};

static rt_bool_t bcm2836_arm_l1_intc_handler(void *data)
{
    rt_uint32_t hwirq;
    rt_bool_t res = RT_FALSE;
    int cpu_id = rt_hw_cpu_id();
    rt_uint32_t status = HWREG32(intc.base + LOCAL_IRQ_PENDING0 + 4 * cpu_id);

    if (!status)
    {
        status = HWREG32(intc.base + LOCAL_MAILBOX0_CLR0 + 16 * cpu_id);
    }

    if (status)
    {
        struct rt_pic_irq *pirq;

        hwirq = __rt_ffs(status) - 1;

        pirq = rt_pic_find_irq(&intc.parent, hwirq);

        rt_pic_handle_isr(pirq);

        res = RT_TRUE;
    }

    return res;
}

/*
 * The LOCAL_IRQ_CNT* timer firings are based off of the external oscillator
 * with some scaling.  The firmware sets up CNTFRQ to report 19.2Mhz, but
 * doesn't set up the scaling registers.
 */
static void bcm2835_init_local_timer_frequency(void)
{
    /*
     * Set the timer to source from the 19.2Mhz crystal clock (bit 8 unset), and
     * only increment by 1 instead of 2 (bit 9 unset).
     */
    HWREG32(intc.base + LOCAL_CONTROL) = 0;

    /* Set the timer prescaler to 1:1 (timer freq = input freq * 2**31 / prescaler) */
    HWREG32(intc.base + LOCAL_PRESCALER) = 0x80000000;
}

static rt_err_t bcm2836_arm_l1_intc_init(struct rt_ofw_node *np, const struct rt_ofw_node_id *id)
{
    rt_err_t err = RT_EOK;

    intc.base = rt_ofw_iomap(node, 0);

    if (intc.base)
    {
        bcm2835_init_local_timer_frequency();

        intc.parent.priv_data = &intc;
        intc.parent.ops = &bcm2836_arm_l1_intc_ops;
        rt_ofw_data(np) = &intc.parent;

        rt_pic_linear_irq(&intc.parent, LAST_IRQ + 1);

        rt_pic_add_traps(bcm2836_arm_l1_intc_handler, &intc);
    }
    else
    {
        err = -RT_ERROR;
    }

    return err;
}

static const struct rt_ofw_ofw_id bcm2836_arm_l1_intc_ofw_ids[] =
{
    { .compatible = "brcm,bcm2836-l1-intc" },
    { /* sentinel */ }
};
RT_OF_DECLARE_CORE(bcm2836_arm_l1_intc, bcm2836_arm_l1_intc_ofw_ids, bcm2836_arm_l1_intc_init);
