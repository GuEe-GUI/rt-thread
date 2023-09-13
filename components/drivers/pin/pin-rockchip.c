/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-26     GuEe-GUI     first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "pin_dm.h"
#include "../pinctrl/pinctrl-rockchip.h"

#define RK_PIN_MAX RK_GPIO4_D7

struct rk_gpio
{
    struct rt_device parent;
    rt_bool_t init_ok;

    struct
    {
        struct rt_pin_irq_hdr hdr;
        rt_uint32_t mask_cache;
    } irq_info[RK_PIN_MAX];

    struct rockchip_pinctrl_device *pinctrl_dev;
};

static struct rk_gpio rk_gpio =
{
    .init_ok = RT_FALSE,
    .irq_info[0 ... RK_PIN_MAX - 1] =
    {
        .mask_cache = 0xffffffffU,
    }
};

#define raw_pin_to_bank(raw, pin)                                           \
({                                                                          \
    struct rockchip_pin_bank *___pin_bank = RT_NULL;                        \
    struct rockchip_pinctrl_device *___pinctrl_dev = (raw)->user_data;      \
    struct rockchip_pin_ctrl *___pinctrl = ___pinctrl_dev->pinctrl;         \
    if (pin <= RK_PIN_MAX && ___pinctrl->pin_banks[(pin) / 32].gpio_regs)   \
    {                                                                       \
        ___pin_bank = &___pinctrl->pin_banks[(pin) / 32];                   \
    }                                                                       \
    ___pin_bank;                                                            \
})
#define raw_pin_to_id(pin) (pin % 32)

#define GPIO_TYPE_V1    0
#define GPIO_TYPE_V2    0x01000c2b
#define GPIO_TYPE_V2_1  0x0101157c

static const struct rockchip_gpio_regs _gpio_regs_v1 =
{
    .port_dr        = 0x00,
    .port_ddr       = 0x04,
    .int_en         = 0x30,
    .int_mask       = 0x34,
    .int_type       = 0x38,
    .int_polarity   = 0x3c,
    .int_status     = 0x40,
    .int_rawstatus  = 0x44,
    .debounce       = 0x48,
    .port_eoi       = 0x4c,
    .ext_port       = 0x50,
};

static const struct rockchip_gpio_regs _gpio_regs_v2 =
{
    .port_dr        = 0x00,
    .port_ddr       = 0x08,
    .int_en         = 0x10,
    .int_mask       = 0x18,
    .int_type       = 0x20,
    .int_polarity   = 0x28,
    .int_bothedge   = 0x30,
    .int_status     = 0x50,
    .int_rawstatus  = 0x58,
    .debounce       = 0x38,
    .dbclk_div_en   = 0x40,
    .dbclk_div_con  = 0x48,
    .port_eoi       = 0x60,
    .ext_port       = 0x70,
    .version_id     = 0x78,
};

rt_inline void gpio_writel_v2(void *base, rt_uint32_t val)
{
    HWREG32(base) = (val & 0xffff) | 0xffff0000U;
    HWREG32(base + 0x4) = (val >> 16) | 0xffff0000U;
}

rt_inline rt_uint32_t gpio_readl_v2(void *base)
{
    return HWREG32(base + 0x4) << 16 | HWREG32(base);
}

rt_inline void rockchip_gpio_writel(struct rockchip_pin_bank *pin_bank, rt_uint32_t value, int offset)
{
    void *base = pin_bank->reg_base + offset;

    if (pin_bank->gpio_type == GPIO_TYPE_V2)
    {
        gpio_writel_v2(base, value);
    }
    else
    {
        HWREG32(base) = value;
    }
}

rt_inline rt_uint32_t rockchip_gpio_readl(struct rockchip_pin_bank *pin_bank, int offset)
{
    rt_uint32_t value;
    void *base = pin_bank->reg_base + offset;

    if (pin_bank->gpio_type == GPIO_TYPE_V2)
    {
        value = gpio_readl_v2(base);
    }
    else
    {
        value = HWREG32(base);
    }

    return value;
}

rt_inline void rockchip_gpio_writel_bit(struct rockchip_pin_bank *pin_bank,
        rt_uint32_t bit, rt_uint32_t value, int offset)
{
    rt_uint32_t data;
    void *base = pin_bank->reg_base + offset;

    if (pin_bank->gpio_type == GPIO_TYPE_V2)
    {
        if (value)
        {
            data = RT_BIT(bit % 16) | RT_BIT(bit % 16 + 16);
        }
        else
        {
            data = RT_BIT(bit % 16 + 16);
        }
        HWREG32(bit >= 16 ? base + 0x4 : base) = data;
    }
    else
    {
        data = HWREG32(base);
        data &= ~RT_BIT(bit);

        if (value)
        {
            data |= RT_BIT(bit);
        }
        HWREG32(base) = data;
    }
}

rt_inline rt_uint32_t rockchip_gpio_readl_bit(struct rockchip_pin_bank *pin_bank,
        rt_uint32_t bit, int offset)
{
    rt_uint32_t data;
    void *base = pin_bank->reg_base + offset;

    if (pin_bank->gpio_type == GPIO_TYPE_V2)
    {
        data = HWREG32(bit >= 16 ? base + 0x4 : base);
        data >>= bit % 16;
    }
    else
    {
        data = HWREG32(base);
        data >>= bit;
    }

    return data & 1;
}

static void rk_pin_mode(struct rt_device *device, rt_base_t pin, rt_uint8_t mode)
{
    rt_base_t level;
    struct rockchip_pinctrl_device *pinctrl_dev = device->user_data;
    struct rockchip_pin_bank *pin_bank = raw_pin_to_bank(device, pin);

    if (!pin_bank)
    {
        return;
    }

    switch (mode)
    {
    case PIN_MODE_OUTPUT:
    case PIN_MODE_OUTPUT_OD:
        pinctrl_dev->pinctrl->set_mux(pin_bank, pin, RK_FUNC_GPIO);

        level = rt_spin_lock_irqsave(&pin_bank->spinlock);
        rockchip_gpio_writel_bit(pin_bank, raw_pin_to_id(pin), 1, pin_bank->gpio_regs->port_ddr);
        rt_spin_unlock_irqrestore(&pin_bank->spinlock, level);
        break;

    case PIN_MODE_INPUT:
    case PIN_MODE_INPUT_PULLUP:
    case PIN_MODE_INPUT_PULLDOWN:
        pinctrl_dev->pinctrl->set_mux(pin_bank, pin, RK_FUNC_GPIO);

        level = rt_spin_lock_irqsave(&pin_bank->spinlock);
        rockchip_gpio_writel_bit(pin_bank, raw_pin_to_id(pin), 0, pin_bank->gpio_regs->port_ddr);
        rt_spin_unlock_irqrestore(&pin_bank->spinlock, level);
        break;

    default:
        break;
    }
}

static void rk_pin_write(struct rt_device *device, rt_base_t pin, rt_uint8_t value)
{
    struct rockchip_pin_bank *pin_bank = raw_pin_to_bank(device, pin);

    if (!pin_bank)
    {
        return;
    }

    rockchip_gpio_writel_bit(pin_bank, raw_pin_to_id(pin), value, pin_bank->gpio_regs->port_dr);
}

static rt_int8_t rk_pin_read(struct rt_device *device, rt_base_t pin)
{
    rt_uint32_t data;
    struct rockchip_pin_bank *pin_bank = raw_pin_to_bank(device, pin);

    if (!pin_bank)
    {
        return -1;
    }

    data = HWREG32(pin_bank->reg_base + pin_bank->gpio_regs->ext_port);
    data >>= raw_pin_to_id(pin);

    return data & 1;
}

static rt_err_t rk_pin_irq_mode(struct rt_device *device, rt_base_t pin, rt_uint8_t mode);

static rt_err_t rk_pin_attach_irq(struct rt_device *device, rt_base_t pin, rt_uint8_t mode,
        void (*hdr)(void *args), void *args)
{
    rt_err_t err;
    typeof(rk_gpio.irq_info[0]) *irq_info;
    struct rockchip_pin_bank *pin_bank = raw_pin_to_bank(device, pin);

    if (!pin_bank || !hdr)
    {
        return -RT_EINVAL;
    }

    if ((err = rk_pin_irq_mode(device, pin, mode)))
    {
        return err;
    }

    irq_info = &rk_gpio.irq_info[pin];

    irq_info->hdr.mode = mode;
    irq_info->hdr.args = args;
    irq_info->hdr.hdr = hdr;

    return RT_EOK;
}

static rt_err_t rk_pin_detach_irq(struct rt_device *device, rt_base_t pin)
{
    typeof(rk_gpio.irq_info[0]) *irq_info;
    struct rockchip_pin_bank *pin_bank = raw_pin_to_bank(device, pin);

    if (!pin_bank)
    {
        return -RT_EINVAL;
    }

    irq_info = &rk_gpio.irq_info[pin];

    irq_info->hdr.hdr = RT_NULL;
    irq_info->hdr.args = RT_NULL;

    return RT_EOK;
}

static rt_err_t rk_pin_irq_enable(struct rt_device *device, rt_base_t pin, rt_uint8_t enabled)
{
    rt_base_t level;
    typeof(rk_gpio.irq_info[0]) *irq_info;
    struct rockchip_pin_bank *pin_bank = raw_pin_to_bank(device, pin);

    if (!pin_bank)
    {
        return -RT_EINVAL;
    }

    level = rt_spin_lock_irqsave(&pin_bank->spinlock);
    irq_info = &rk_gpio.irq_info[pin];

    if (enabled)
    {
        irq_info->mask_cache &= ~RT_BIT(raw_pin_to_id(pin));
    }
    else
    {
        irq_info->mask_cache |= RT_BIT(raw_pin_to_id(pin));
    }
    rockchip_gpio_writel(pin_bank, irq_info->mask_cache, pin_bank->gpio_regs->int_mask);

    rt_spin_unlock_irqrestore(&pin_bank->spinlock, level);

    return RT_EOK;
}

static rt_err_t rk_pin_irq_mode(struct rt_device *device, rt_base_t pin, rt_uint8_t mode)
{
    rt_base_t level;
    rt_err_t err = RT_EOK;
    rt_uint32_t mask, int_level, polarity, data;
    struct rockchip_pin_bank *pin_bank = raw_pin_to_bank(device, pin);

    if (!pin_bank)
    {
        return -RT_EINVAL;
    }

    level = rt_spin_lock_irqsave(&pin_bank->spinlock);

    rockchip_gpio_writel_bit(pin_bank, raw_pin_to_id(pin), 0, pin_bank->gpio_regs->port_ddr);

    mask = RT_BIT(raw_pin_to_id(pin));
    int_level = rockchip_gpio_readl(pin_bank, pin_bank->gpio_regs->int_type);
    polarity = rockchip_gpio_readl(pin_bank, pin_bank->gpio_regs->int_polarity);

    if (mode == PIN_IRQ_MODE_RISING_FALLING)
    {
        if (pin_bank->gpio_type == GPIO_TYPE_V2)
        {
            rockchip_gpio_writel_bit(pin_bank, raw_pin_to_id(pin), 1, pin_bank->gpio_regs->int_bothedge);
            goto _end;
        }
        else
        {
            pin_bank->toggle_edge_mode |= mask;
            int_level &= ~mask;

            /*
             * Determine gpio state. If 1 next interrupt should be
             * low otherwise high.
             */
            data = HWREG32(pin_bank->reg_base + pin_bank->gpio_regs->ext_port);

            if (data & mask)
            {
                polarity &= ~mask;
            }
            else
            {
                polarity |= mask;
            }
        }
    }
    else
    {
        if (pin_bank->gpio_type == GPIO_TYPE_V2)
        {
            rockchip_gpio_writel_bit(pin_bank, raw_pin_to_id(pin), 0, pin_bank->gpio_regs->int_bothedge);
        }
        else
        {
            pin_bank->toggle_edge_mode &= ~mask;
        }

        switch (mode)
        {
        case PIN_IRQ_MODE_RISING:
            int_level |= mask;
            polarity |= mask;
            break;

        case PIN_IRQ_MODE_FALLING:
            int_level |= mask;
            polarity &= ~mask;
            break;

        case PIN_IRQ_MODE_HIGH_LEVEL:
            int_level &= ~mask;
            polarity |= mask;
            break;

        case PIN_IRQ_MODE_LOW_LEVEL:
            int_level &= ~mask;
            polarity &= ~mask;
            break;

        default:
            err = -RT_EINVAL;
            goto _end;
        }
    }

    rockchip_gpio_writel(pin_bank, int_level, pin_bank->gpio_regs->int_type);
    rockchip_gpio_writel(pin_bank, polarity, pin_bank->gpio_regs->int_polarity);

_end:
    rt_spin_unlock_irqrestore(&pin_bank->spinlock, level);

    return err;
}

static rt_ssize_t rk_pin_parse(struct rt_device *device, struct rt_ofw_cell_args *args, rt_uint32_t *flags)
{
    rt_ssize_t pin;
    struct rt_device_pin *pin_dev = rt_ofw_data(args->data);

    pin = pin_dev->irqchip.pin_range[0] + args->args[0];

    if (flags)
    {
        *flags = args->args[1];
    }

    return pin;
}

static const struct rt_pin_ops rk_pin_ops =
{
    .pin_mode = rk_pin_mode,
    .pin_write = rk_pin_write,
    .pin_read = rk_pin_read,
    .pin_attach_irq = rk_pin_attach_irq,
    .pin_detach_irq = rk_pin_detach_irq,
    .pin_irq_enable = rk_pin_irq_enable,
    .pin_irq_mode = rk_pin_irq_mode,
    .pin_parse = rk_pin_parse,
};

static void rk_pin_isr(int irqno, void *param)
{
    rt_uint32_t pending;
    struct rockchip_pin_bank *pin_bank = param;

    pending = HWREG32(pin_bank->reg_base + pin_bank->gpio_regs->int_status);

    for (rt_ubase_t pin = 0; pin < 32 && pending; ++pin)
    {
        rt_uint32_t clr = RT_BIT(pin);
        typeof(rk_gpio.irq_info[0]) *irq_info;

        if (!(clr & pending))
        {
            continue;
        }

        if (clr & pin_bank->toggle_edge_mode)
        {
            rt_uint32_t data, data_old, polarity;

            data = HWREG32(pin_bank->reg_base + pin_bank->gpio_regs->ext_port);

            do {
                rt_ubase_t level = rt_spin_lock_irqsave(&pin_bank->spinlock);

                polarity = HWREG32(pin_bank->reg_base + pin_bank->gpio_regs->int_polarity);

                if (data & clr)
                {
                    polarity &= ~clr;
                }
                else
                {
                    polarity |= clr;
                }
                HWREG32(pin_bank->reg_base + pin_bank->gpio_regs->int_polarity) = polarity;

                rt_spin_unlock_irqrestore(&pin_bank->spinlock, level);

                data_old = data;
                data = HWREG32(pin_bank->reg_base + pin_bank->gpio_regs->ext_port);
            } while ((data & clr) != (data_old & clr));
        }

        pin_pic_handle_isr(&pin_bank->parent, pin);

        irq_info = &rk_gpio.irq_info[pin_bank->parent.irqchip.pin_range[0] + pin];

        if (irq_info->hdr.hdr)
        {
            irq_info->hdr.hdr(irq_info->hdr.args);
        }

        rockchip_gpio_writel_bit(pin_bank, pin, 1, pin_bank->gpio_regs->port_eoi);

        /* clear this pin irq */
        pending &= ~clr;
    }
}

static rt_err_t rockchip_gpio_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    int id, version;
    const char *name;
    struct rockchip_pin_bank *pin_bank;
    struct rt_ofw_node *np = pdev->parent.ofw_node;
    struct rt_ofw_node *npp = rt_ofw_get_parent(np);
    struct rockchip_pinctrl_device *pinctrl_dev = rt_ofw_data(npp);
    struct rockchip_pin_ctrl *pinctrl = pinctrl_dev->pinctrl;
    static int gpio_uid = 0;

    rt_ofw_node_put(npp);

    id = rt_ofw_get_alias_id(np, "gpio");

    if (id < 0)
    {
        id = gpio_uid++;
    }

    pin_bank = &pinctrl->pin_banks[id];

    pin_bank->reg_base = rt_ofw_iomap(np, 0);

    if (!pin_bank->reg_base)
    {
        err = -RT_EIO;
        goto _out_res;
    }

    pin_bank->irq = rt_ofw_get_irq(np, 0);

    if (pin_bank->irq < 0)
    {
        err = pin_bank->irq;
        goto _out_res;
    }

    pin_bank->clk = rt_ofw_get_clk(np, 0);

    if (rt_is_err(pin_bank->clk))
    {
        err = rt_ptr_err(pin_bank->clk);
        goto _out_res;
    }

    rt_clk_prepare_enable(pin_bank->clk);

    version = HWREG32(pin_bank->reg_base + _gpio_regs_v2.version_id);

    if (version == GPIO_TYPE_V2 || version == GPIO_TYPE_V2_1)
    {
        pin_bank->gpio_regs = &_gpio_regs_v2;
        pin_bank->gpio_type = GPIO_TYPE_V2;

        pin_bank->db_clk = rt_ofw_get_clk(np, 1);

        if (rt_is_err(pin_bank->db_clk))
        {
            err = rt_ptr_err(pin_bank->db_clk);
            goto _out_res;
        }
    }
    else
    {
        pin_bank->gpio_regs = &_gpio_regs_v1;
        pin_bank->gpio_type = GPIO_TYPE_V1;
    }

    rt_dm_dev_set_name_auto(&rk_gpio.parent, "gpio");
    name = rt_dm_dev_get_name(&rk_gpio.parent);

    rt_hw_interrupt_install(pin_bank->irq, rk_pin_isr, pin_bank, name);
    rt_hw_interrupt_umask(pin_bank->irq);

    rockchip_gpio_writel(pin_bank, 0xffffffffU, pin_bank->gpio_regs->int_mask);
    rockchip_gpio_writel(pin_bank, 0xffffffffU, pin_bank->gpio_regs->port_eoi);
    rockchip_gpio_writel(pin_bank, 0xffffffffU, pin_bank->gpio_regs->int_en);

    rt_spin_lock_init(&pin_bank->spinlock);

    pin_bank->parent.irqchip.irq = pin_bank->irq;
    pin_bank->parent.irqchip.pin_range[0] = id * 32;
    pin_bank->parent.irqchip.pin_range[1] = pin_bank->parent.irqchip.pin_range[0] + RK_PD7;
    pin_bank->parent.ops = &rk_pin_ops;
    pin_bank->parent.parent.user_data = pinctrl_dev;
    pin_pic_init(&pin_bank->parent);

    rt_ofw_data(np) = &pin_bank->parent;

    if (!rk_gpio.init_ok)
    {
        rk_gpio.init_ok = RT_TRUE;
        rk_gpio.pinctrl_dev = pinctrl_dev;

        rt_device_pin_register("gpio", &rk_pin_ops, pinctrl_dev);
    }

    return RT_EOK;

_out_res:
    if (pin_bank->reg_base)
    {
        rt_iounmap(pin_bank->reg_base);
    }

    if (rt_is_err_or_null(pin_bank->clk))
    {
        rt_clk_disable_unprepare(pin_bank->clk);
        rt_clk_put(pin_bank->clk);
    }

    if (rt_is_err_or_null(pin_bank->db_clk))
    {
        rt_clk_disable_unprepare(pin_bank->db_clk);
        rt_clk_put(pin_bank->db_clk);
    }

    return err;
}

static const struct rt_ofw_node_id rockchip_gpio_ofw_ids[] =
{
    { .compatible = "rockchip,gpio-bank" },
    { /* sentinel */ }
};

static struct rt_platform_driver rockchip_gpio_driver =
{
    .name = "rockchip-gpio",
    .ids = rockchip_gpio_ofw_ids,

    .probe = rockchip_gpio_probe,
};

static int rockchip_gpio_register(void)
{
    int idx = 0;
    struct rt_ofw_node *np = rt_ofw_find_node_by_path("/pinctrl"), *gpio_np;

    if (!np || !rt_ofw_data(np))
    {
        goto _end;
    }

    rt_platform_driver_register(&rockchip_gpio_driver);

    rt_ofw_foreach_available_child_node(np, gpio_np)
    {
        if (idx > RK_GPIO4)
        {
            rt_ofw_node_put(gpio_np);

            break;
        }

        if (!rt_ofw_prop_read_bool(gpio_np, "compatible"))
        {
            continue;
        }

        if (rt_ofw_data(gpio_np))
        {
            continue;
        }

        ++idx;
        rt_platform_ofw_device_probe_child(gpio_np);
    }

_end:
    rt_ofw_node_put(np);

    return 0;
}
INIT_SUBSYS_EXPORT(rockchip_gpio_register);
