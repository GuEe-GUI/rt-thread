/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-07-06     Bernard      first version
 * 2018-11-22     Jesven       add smp support
 * 2023-02-21     GuEe-GUI     replace with pic
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtservice.h>

#include <armv8.h>
#include <interrupt.h>

#include <drivers/pic.h>

#ifndef RT_USING_SMP
/* Those variables will be accessed in ISR, so we need to share them. */
rt_ubase_t rt_interrupt_from_thread        = 0;
rt_ubase_t rt_interrupt_to_thread          = 0;
rt_ubase_t rt_thread_switch_interrupt_flag = 0;
#endif

#ifdef RT_USING_SMP
#define rt_interrupt_nest rt_cpu_self()->irq_nest
#else
extern volatile rt_uint8_t rt_interrupt_nest;
#endif

/**
 * This function will initialize hardware interrupt
 */
void rt_hw_interrupt_init(void)
{
    /* initialize pic */
    rt_pic_irq_init();
}

/**
 * This function will mask a interrupt.
 * @param vector the interrupt number
 */
void rt_hw_interrupt_mask(int vector)
{
    rt_pic_irq_mask(vector);
}

/**
 * This function will un-mask a interrupt.
 * @param vector the interrupt number
 */
void rt_hw_interrupt_umask(int vector)
{
    rt_pic_irq_unmask(vector);
}

/**
 * This function will install a interrupt service routine to a interrupt.
 * @param vector the interrupt number
 * @param new_handler the interrupt service routine to be installed
 * @param old_handler the old interrupt service routine
 */
rt_isr_handler_t rt_hw_interrupt_install(int vector, rt_isr_handler_t handler,
        void *param, const char *name)
{
    rt_pic_attach_irq(vector, handler, param, name, RT_IRQ_F_NONE);

    return RT_NULL;
}

/**
 * This function will install a interrupt service routine to a interrupt.
 * @param vector the interrupt number
 * @param new_handler the interrupt service routine to be installed
 * @param old_handler the old interrupt service routine
 */
void rt_hw_interrupt_uninstall(int vector, rt_isr_handler_t handler, void *param)
{
    rt_pic_detach_irq(vector, param);
}

#ifdef RT_USING_SMP
void rt_hw_ipi_send(int ipi_vector, unsigned int cpu_mask)
{
    DECLARE_BITMAP(cpu_masks, RT_CPUS_NR) = { cpu_mask };

    rt_pic_irq_send_ipi(ipi_vector, cpu_masks);
}

void rt_hw_ipi_handler_install(int ipi_vector, rt_isr_handler_t ipi_isr_handler)
{
    /* note: ipi_vector maybe different with irq_vector */
    rt_hw_interrupt_install(ipi_vector, ipi_isr_handler, 0, "IPI_HANDLER");
}
#endif
