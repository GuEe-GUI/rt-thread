/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-3-08      GuEe-GUI     the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <dt-bindings/input/event-codes.h>

#define DBG_TAG "input.keyboard.gpio"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

struct gpio_key
{
    rt_base_t pin;
    rt_uint8_t mode;
    rt_uint32_t code;
};

typedef void (*gpio_key_handler)(void *);

static rt_err_t gpio_attach_event(struct gpio_key *gkey, void (*handler)(void *))
{
    rt_err_t err;

    err = rt_pin_attach_irq(gkey->pin, gkey->mode, handler, gkey);

    if (!err)
    {
        rt_pin_irq_enable(gkey->pin, RT_TRUE);
    }

    return err;
}

static rt_err_t ofw_append_gpio_key(struct rt_ofw_node *np)
{
    rt_err_t err;
    const char *propname;
    struct gpio_key *gkey = rt_calloc(1, sizeof(*gkey));

    gkey->pin = rt_ofw_get_named_pin(np, RT_NULL, 0, &gkey->mode, RT_NULL);

    if (gkey->pin < 0)
    {
        err = gkey->pin;

        goto _fail;
    }

    if ((propname = rt_ofw_get_prop_fuzzy_name(np, ",code$")) &&
        !rt_ofw_prop_read_u32(np, propname, &gkey->code))
    {
        switch (gkey->code)
        {
        case KEY_POWER:
            err = gpio_attach_event(gkey, (gpio_key_handler)rt_hw_cpu_shutdown);
            break;

        case KEY_RESTART:
            err = gpio_attach_event(gkey, (gpio_key_handler)rt_hw_cpu_reset);
            break;

        default:
            err = -RT_EINVAL;
            LOG_W("Unsupported event code = %d", gkey->code);
            break;
        }
    }

    if (err)
    {
        goto _fail;
    }

    rt_ofw_data(np) = gkey;

    return RT_EOK;

_fail:
    rt_free(gkey);

    return err;
}

static rt_err_t gpio_key_probe(struct rt_platform_device *pdev)
{
    rt_err_t err = RT_EOK;
    struct rt_ofw_node *key_np, *np = pdev->parent.ofw_node;

    rt_ofw_foreach_available_child_node(np, key_np)
    {
        rt_err_t err = ofw_append_gpio_key(key_np);

        if (err == -RT_ENOMEM)
        {
            rt_ofw_node_put(key_np);

            return err;
        }
        else if (err)
        {
            LOG_E("%s: create KEY fail", rt_ofw_node_full_name(key_np));
            continue;
        }
    }

    return err;
}

static rt_err_t gpio_key_remove(struct rt_platform_device *pdev)
{
    struct rt_ofw_node *key_np, *np = pdev->parent.ofw_node;

    rt_ofw_foreach_available_child_node(np, key_np)
    {
        struct gpio_key *gkey = rt_ofw_data(key_np);

        if (!gkey)
        {
            continue;
        }

        rt_ofw_data(key_np) = RT_NULL;

        rt_pin_irq_enable(gkey->pin, RT_FALSE);
        rt_pin_detach_irq(gkey->pin);

        rt_free(gkey);
    }

    return RT_EOK;
}

static const struct rt_ofw_node_id gpio_key_ofw_ids[] =
{
    { .compatible = "gpio-keys" },
    { /* sentinel */ }
};

static struct rt_platform_driver gpio_key_driver =
{
    .name = "gpio-keys",
    .ids = gpio_key_ofw_ids,

    .probe = gpio_key_probe,
    .remove = gpio_key_remove,
};

static int gpio_key_drv_register(void)
{
    rt_platform_driver_register(&gpio_key_driver);

    return 0;
}
INIT_DRIVER_LATER_EXPORT(gpio_key_drv_register);
