#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <rthw.h>

#include "board.h"
#include "drivers/amba_bus.h"

#ifdef RT_USING_FDT
#include "dtb_node.h"
#endif

#define GPIODIR 0x400
#define GPIOIS  0x404
#define GPIOIBE 0x408
#define GPIOIEV 0x40c
#define GPIOIE  0x410
#define GPIORIS 0x414
#define GPIOMIS 0x418
#define GPIOIC  0x41c

#define BIT(x)  (1 << (x))

#define PL061_GPIO_NR   8

static struct pl061
{
#ifdef RT_USING_SMP
    struct rt_spinlock spinlock;
#endif
    void (*(hdr[PL061_GPIO_NR]))(void *args);
    void *args[PL061_GPIO_NR];
    rt_ubase_t hw_base;
    rt_uint32_t irqno;
} _pl061;

rt_inline rt_uint8_t pl061_read8(rt_ubase_t offset)
{
    return HWREG8(_pl061.hw_base + offset);
}

rt_inline void pl061_write8(rt_ubase_t offset, rt_uint8_t value)
{
    HWREG8(_pl061.hw_base + offset) = value;
}

static void pl061_pin_mode(struct rt_device *device, rt_base_t pin, rt_uint8_t mode)
{
    int value;
    rt_uint8_t gpiodir;

#ifdef RT_USING_SMP
    rt_base_t level;
#endif

    if (pin < 0 || pin >= PL061_GPIO_NR)
    {
        return;
    }

#ifdef RT_USING_SMP
    level = rt_spin_lock_irqsave(&_pl061.spinlock);
#endif

    switch (mode)
    {
    case PIN_MODE_OUTPUT:

        value = !!pl061_read8((BIT(pin + 2)));

        pl061_write8(BIT(pin + 2), 0 << pin);
        gpiodir = pl061_read8(GPIODIR);
        gpiodir |= BIT(pin);
        pl061_write8(GPIODIR, gpiodir);

        /*
         * gpio value is set again, because pl061 doesn't allow to set value of
         * a gpio pin before configuring it in OUT mode.
         */
        pl061_write8((BIT(pin + 2)), value << pin);

        break;
    case PIN_MODE_INPUT:

        gpiodir = pl061_read8(GPIODIR);
        gpiodir &= ~(BIT(pin));
        pl061_write8(GPIODIR, gpiodir);

        break;
    }

#ifdef RT_USING_SMP
    rt_spin_unlock_irqrestore(&_pl061.spinlock, level);
#endif
}

static void pl061_pin_write(struct rt_device *device, rt_base_t pin, rt_uint8_t value)
{
    pl061_write8(BIT(pin + 2), !!value << pin);
}

static rt_int8_t pl061_pin_read(struct rt_device *device, rt_base_t pin)
{
    return !!pl061_read8((BIT(pin + 2)));
}

static rt_err_t pl061_pin_attach_irq(struct rt_device *device, rt_base_t pin, rt_uint8_t mode, void (*hdr)(void *args), void *args)
{
    rt_uint8_t gpiois, gpioibe, gpioiev;
    rt_uint8_t bit = BIT(mode);
#ifdef RT_USING_SMP
    rt_base_t level;
#endif

    if (pin < 0 || pin >= PL061_GPIO_NR)
    {
        return -RT_EINVAL;
    }

#ifdef RT_USING_SMP
    level = rt_spin_lock_irqsave(&_pl061.spinlock);
#endif

    gpioiev = pl061_read8(GPIOIEV);
    gpiois  = pl061_read8(GPIOIS);
    gpioibe = pl061_read8(GPIOIBE);

    if (mode == PIN_IRQ_MODE_HIGH_LEVEL || pin == PIN_IRQ_MODE_LOW_LEVEL)
    {
        rt_bool_t polarity = (mode == PIN_IRQ_MODE_HIGH_LEVEL);

        /* Disable edge detection */
        gpioibe &= ~bit;
        /* Enable level detection */
        gpiois |= bit;

        /* Select polarity */
        if (polarity)
        {
            gpioiev |= bit;
        }
        else
        {
            gpioiev &= ~bit;
        }
    }
    else if (mode == PIN_IRQ_MODE_RISING_FALLING)
    {
        /* Disable level detection */
        gpiois &= ~bit;
        /* Select both edges, setting this makes GPIOEV be ignored */
        gpioibe |= bit;
    }
    else if (mode == PIN_IRQ_MODE_RISING || mode == PIN_IRQ_MODE_FALLING)
    {
        rt_bool_t rising = (mode == PIN_IRQ_MODE_RISING);

        /* Disable level detection */
        gpiois &= ~bit;
        /* Clear detection on both edges */
        gpioibe &= ~bit;

        /* Select edge */
        if (rising)
        {
            gpioiev |= bit;
        }
        else
        {
            gpioiev &= ~bit;
        }
    }
    else
    {
        /* No trigger: disable everything */
        gpiois  &= ~bit;
        gpioibe &= ~bit;
        gpioiev &= ~bit;
    }

    pl061_write8(GPIOIS, gpiois);
    pl061_write8(GPIOIBE, gpioibe);
    pl061_write8(GPIOIEV, gpioiev);

    _pl061.hdr[pin] = hdr;
    _pl061.args[pin] = args;

#ifdef RT_USING_SMP
    rt_spin_unlock_irqrestore(&_pl061.spinlock, level);
#endif

    return RT_EOK;
}

static rt_err_t pl061_pin_detach_irq(struct rt_device *device, rt_base_t pin)
{
    if (pin < 0 || pin >= PL061_GPIO_NR)
    {
        return -RT_EINVAL;
    }

    _pl061.hdr[pin] = RT_NULL;
    _pl061.args[pin] = RT_NULL;

    return RT_EOK;
}

static rt_err_t pl061_pin_irq_enable(struct rt_device *device, rt_base_t pin, rt_uint8_t enabled)
{
    rt_uint8_t mask = BIT(pin);
    rt_uint8_t gpioie;

#ifdef RT_USING_SMP
    rt_base_t level;
#endif

    if (pin < 0 || pin >= PL061_GPIO_NR)
    {
        return -RT_EINVAL;
    }

#ifdef RT_USING_SMP
    level = rt_spin_lock_irqsave(&_pl061.spinlock);
#endif

    if (enabled)
    {
        gpioie = pl061_read8(GPIOIE) | mask;
    }
    else
    {
        gpioie = pl061_read8(GPIOIE) & ~mask;
    }

    pl061_write8(GPIOIE, gpioie);

#ifdef RT_USING_SMP
    rt_spin_unlock_irqrestore(&_pl061.spinlock, level);
#endif

    return RT_EOK;
}

static struct rt_pin_ops pl061_ops =
{
    pl061_pin_mode,
    pl061_pin_write,
    pl061_pin_read,
    pl061_pin_attach_irq,
    pl061_pin_detach_irq,
    pl061_pin_irq_enable,
    RT_NULL,
};

static void rt_hw_gpio_isr(int irqno, void *param)
{
    rt_uint8_t mask;
    unsigned long pending;

#ifdef RT_USING_SMP
    rt_base_t level;
#endif

    pending = pl061_read8(GPIOMIS);

    if (pending)
    {
        rt_base_t pin;

        for (pin = 0; pin < PL061_GPIO_NR; ++pin)
        {
            if (pending & BIT(pin))
            {
                mask |= BIT(pin);

                if (_pl061.hdr[pin] != RT_NULL)
                {
                    _pl061.hdr[pin](_pl061.args[pin]);
                }
            }
        }
    }

#ifdef RT_USING_SMP
    level = rt_spin_lock_irqsave(&_pl061.spinlock);
#endif

    pl061_write8(GPIOIC, mask);

#ifdef RT_USING_SMP
    rt_spin_unlock_irqrestore(&_pl061.spinlock, level);
#endif
}

int pl061_probe(struct rt_device *dev)
{
    size_t reg_range = 0;
    void *reg_addr = RT_NULL;

    struct dtb_node *node = (struct dtb_node *)(dev->dtb_node);

#ifdef RT_USING_SMP
    rt_spin_lock_init(&_pl061.spinlock);
#endif

    if (node)
    {
        reg_addr = (void *)dtb_node_get_addr_size(node, "reg", &reg_range);
        if ((reg_addr) && (reg_range != 0))
        {
            _pl061.hw_base = (rt_base_t)rt_ioremap(reg_addr, reg_range);
        }

        _pl061.irqno = dtb_node_irq_get(node, 0) + IRQ_SPI_OFFSET;

        struct rt_pin_ops *ops = ((struct rt_pin_driver *)(dev->drv))->ops;

        rt_device_pin_register(dev->drv->name, ops, RT_NULL);
        rt_hw_interrupt_install(_pl061.irqno, rt_hw_gpio_isr, RT_NULL, dev->drv->name);
        rt_hw_interrupt_umask(_pl061.irqno);
    }

    return RT_EOK;
}

struct rt_device_id pl061_ids[] =
    {
        {.compatible = "arm,pl061"},
        {/* sentinel */}};

struct rt_pin_driver pl061_drv = {
    .parent = {
        .name = "pl061_gpio",
        .probe = pl061_probe,
        .ids = pl061_ids,
    },
    .ops = &pl061_ops,
};

AMBA_DRIVER_EXPORT(pl061_drv);
