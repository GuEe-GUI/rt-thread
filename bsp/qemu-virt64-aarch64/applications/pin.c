/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2022-6-30      GuEe-GUI       first version
 * 2023-2-25      GuEe-GUI       using gpio-keys ofw in dm
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "app.pin"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef RT_USING_PIN

#define LINUX_KEY_POWER 116 /* SC System Power Down */

void linux_key_poweroff(void *args)
{
    LOG_I("Power off the machine by [%s]", args);

    rt_hw_cpu_shutdown();
}

static int pin_init()
{
    struct rt_ofw_node *gpio_keys_np = rt_ofw_find_node_by_compatible(RT_NULL, "gpio-keys"), *key;

    if (!gpio_keys_np)
    {
        return -1;
    }

    rt_ofw_foreach_child_node(gpio_keys_np, key)
    {
        struct rt_ofw_cell_args args;

        if (!rt_ofw_parse_phandle_cells(key, "gpios", "#gpio-cells", 0, &args))
        {
            rt_uint32_t linux_code;
            void *key_args = RT_NULL;
            void (*fn_ptr)(void *) = RT_NULL;

            rt_ofw_prop_read_u32(key, "linux,code", &linux_code);

            switch (linux_code)
            {
            case LINUX_KEY_POWER:
                fn_ptr = &linux_key_poweroff;
                rt_ofw_prop_read_string(key, "label", (const char **)&key_args);
                break;
            default:
                break;
            }

            if (fn_ptr)
            {
                rt_pin_attach_irq(args.args[0], args.args[1], fn_ptr, key_args);
                rt_pin_irq_enable(args.args[0], RT_TRUE);
            }

            rt_ofw_node_put(args.data);
        }
    }

    rt_ofw_node_put(gpio_keys_np);

    return 0;
}
INIT_APP_EXPORT(pin_init);

#endif /* RT_USING_PIN */
