#include <rthw.h>
#include <rtthread.h>
#include <stdint.h>
#include <drivers/core/platform.h>

#include "cp15.h"
#include "board.h"
#include "gtimer.h"
#include "drivers/core/rtdm.h"

#ifdef RT_USING_FDT
#include "dtb_node.h"
#endif

static rt_uint64_t timerStep = 0;

static void rt_hw_timer_isr(int vector, void *param)
{
    rt_hw_set_gtimer_val(timerStep);
    rt_tick_increase();
}

void rt_hw_timer_enable(rt_uint32_t irqno)
{
    rt_hw_set_gtimer_val(timerStep);
    rt_hw_interrupt_umask(irqno);
    rt_hw_gtimer_enable();
}

int timer_probe(struct rt_device *dev)
{
    struct dtb_node *node = (struct dtb_node *)(dev->dtb_node);
    if (node)
    {
        rt_uint32_t irqno = dtb_node_irq_get(node, 1) + IRQ_PPI_OFFSET;
        const char *name = dev->drv->name;
        rt_device_register(dev, name, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_DEACTIVATE);
        rt_hw_interrupt_install(irqno, rt_hw_timer_isr, RT_NULL, name);
        __ISB();
        timerStep = rt_hw_get_gtimer_frq();
        __DSB();
        timerStep /= RT_TICK_PER_SECOND;
        rt_hw_timer_enable(irqno);
    }
    return 0;
}

struct rt_device_id timer_ids[] =
    {
        {.compatible = "arm,armv7-timer"},
        {/* sentinel */}
    };

struct rt_driver timer_drv = {
    .ids = timer_ids,
    .name = "timer_virt64",
    .dev_type = RT_Device_Class_Timer,
    .device_size = sizeof(struct rt_device),
    .probe = timer_probe,
};

TIMER_DRIVER_EXPORT(timer_drv);
