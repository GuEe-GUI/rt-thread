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

#define DBG_TAG "input.keyboard.adc"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

struct adc_keys_button
{
    rt_uint32_t voltage;
    rt_uint32_t keycode;
};

struct adc_keys
{
    struct rt_adc_device *adc_dev;

    struct rt_timer poll_work;

    int channel;
    rt_uint32_t num_keys;
    rt_uint32_t last_key;
    rt_uint32_t keyup_voltage;
    struct adc_keys_button kbtn[];
};

static void adc_keys_poll(void *param)
{
    int value, keycode = 0;
    rt_uint32_t diff, closest = 0xffffffff;
    struct adc_keys *tk = param;

    value = rt_adc_read(tk->adc_dev, tk->channel);

    if (rt_unlikely(value < 0))
    {
        /* Forcibly release key if any was pressed */
        value = tk->keyup_voltage;
    }
    else
    {
        for (int i = 0; i < tk->num_keys; ++i)
        {
            diff = rt_abs(tk->kbtn[i].voltage - value);

            if (diff < closest)
            {
                closest = diff;
                keycode = tk->kbtn[i].keycode;
            }
        }
    }

    if (rt_abs(tk->keyup_voltage - value) < closest)
    {
        keycode = 0;
    }

    if (tk->last_key && tk->last_key != keycode)
    {
        /* Key up */
        switch (tk->last_key)
        {
        case KEY_POWER:
            rt_hw_cpu_shutdown();
            break;

        case KEY_RESTART:
            rt_hw_cpu_reset();
            break;

        default:
            LOG_W("Unsupported keycode = %d", tk->last_key);
            break;
        }
    }

    if (keycode)
    {
        /* Key down */
    }

    tk->last_key = keycode;
}

static rt_err_t adc_key_probe(struct rt_platform_device *pdev)
{
    int i = 0;
    rt_err_t err;
    rt_uint32_t interval;
    rt_uint32_t num_keys;
    struct adc_keys *tk;
    struct rt_device *dev = &pdev->parent;
    struct rt_ofw_node *np = dev->ofw_node, *key_np;

    num_keys = rt_ofw_get_child_count(np);

    if (!num_keys)
    {
        LOG_E("Keymap is missing");

        return -RT_EINVAL;
    }

    tk = rt_calloc(1, sizeof(*tk) + sizeof(struct adc_keys_button) * num_keys);

    if (!tk)
    {
        return -RT_ENOMEM;
    }

    tk->adc_dev = rt_iio_channel_get_by_name(dev, "buttons", &tk->channel);

    if (!tk->adc_dev)
    {
        LOG_E("ADC device not found");

        err = -RT_EINVAL;
        goto _fail;
    }

    rt_ofw_foreach_child_node(np, key_np)
    {
        const char *propname;

        if (rt_ofw_prop_read_u32(key_np, "press-threshold-microvolt",
                &tk->kbtn[i].voltage))
        {
            LOG_E("%s: Key with invalid or missing %s",
                    rt_ofw_node_full_name(key_np), "voltage");
            rt_ofw_node_put(key_np);

            err = -RT_EINVAL;
            goto _fail;
        }

        tk->kbtn[i].voltage /= 1000;

        if (!(propname = rt_ofw_get_prop_fuzzy_name(key_np, ",code$")) ||
            rt_ofw_prop_read_u32(key_np, propname, &tk->kbtn[i].keycode))
        {
            LOG_E("%s: Key with invalid or missing %s",
                    rt_ofw_node_full_name(key_np), "*,code");
            rt_ofw_node_put(key_np);

            err = -RT_EINVAL;
            goto _fail;
        }

        ++i;
    }

    dev->user_data = tk;

    if (rt_ofw_prop_read_u32(np, "keyup-threshold-microvolt", &tk->keyup_voltage))
    {
        LOG_E("Invalid or missing keyup voltage");

        err = -RT_EINVAL;
        goto _fail;
    }

    if (rt_ofw_prop_read_u32(np, "poll-interval", &interval))
    {
        interval = 200;
    }

    rt_timer_init(&tk->poll_work, "adc-keys", adc_keys_poll, tk,
            rt_tick_from_millisecond(interval), RT_TIMER_FLAG_PERIODIC);
    rt_timer_start(&tk->poll_work);

    return RT_EOK;

_fail:
    rt_free(tk);

    return err;
}

static rt_err_t adc_key_remove(struct rt_platform_device *pdev)
{
    struct adc_keys *tk = pdev->parent.user_data;

    rt_timer_stop(&tk->poll_work);
    rt_timer_detach(&tk->poll_work);

    rt_free(tk);

    return RT_EOK;
}

static const struct rt_ofw_node_id adc_key_ofw_ids[] =
{
    { .compatible = "adc-keys" },
    { /* sentinel */ }
};

static struct rt_platform_driver adc_key_driver =
{
    .name = "adc-keys",
    .ids = adc_key_ofw_ids,

    .probe = adc_key_probe,
    .remove = adc_key_remove,
};

static int adc_key_drv_register(void)
{
    rt_platform_driver_register(&adc_key_driver);

    return 0;
}
INIT_DRIVER_LATER_EXPORT(adc_key_drv_register);
