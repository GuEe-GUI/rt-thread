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

#define DBG_TAG "input.js.adc"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

struct adc_joystick_axis
{
    rt_uint32_t code;
    rt_uint32_t range[2];
    rt_uint32_t fuzz;
    rt_uint32_t flat;
    rt_uint32_t channel;

    struct rt_adc_device *adc_dev;
};

struct adc_joysticks
{
    struct rt_input_device parent;

    rt_uint32_t num_axis;
    struct adc_joystick_axis axis[];
};

static void adc_joysticks_poll(struct rt_input_device *idev)
{
    int value;
    struct adc_joysticks *aj = rt_container_of(idev, struct adc_joysticks, parent);

    for (int i = 0; i < aj->num_axis; ++i)
    {
        struct adc_joystick_axis *axis = &aj->axis[i];

        value = rt_adc_read(axis->adc_dev, axis->channel);

        rt_input_report_abs(&aj->parent, axis->code, value);
    }

    rt_input_sync(&aj->parent);
}

static rt_err_t adc_joystick_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    rt_uint32_t interval;
    rt_uint32_t num_axis;
    struct adc_joysticks *aj;
    struct adc_joystick_axis *axis;
    struct rt_device *dev = &pdev->parent;
    struct rt_ofw_node *np = dev->ofw_node, *axis_np;

    num_axis = rt_ofw_get_child_count(np);

    if (!num_axis)
    {
        LOG_E("Keymap is missing");

        return -RT_EINVAL;
    }

    aj = rt_calloc(1, sizeof(*aj) + sizeof(struct adc_joysticks_button) * num_axis);

    if (!aj)
    {
        return -RT_ENOMEM;
    }

    rt_ofw_foreach_child_node(np, axis_np)
    {
        rt_uint32_t reg;
        const char *propname;

        if (rt_ofw_prop_read_u32(axis_np, "reg", &reg))
        {
            rt_ofw_node_put(axis_np);

            err = -RT_EINVAL;
            goto _fail;
        }
        axis = &aj->axis[reg];

        axis->adc_dev = rt_iio_channel_get_by_index(dev, reg, &axis->channel);

        if (!axis->adc_dev)
        {
            rt_ofw_node_put(axis_np);

            LOG_E("ADC device not found");
            err = -RT_EINVAL;
            goto _fail;
        }

        if (rt_ofw_prop_read_u32(axis_np, "abs-flat", &axis->flat))
        {
            rt_ofw_node_put(axis_np);

            LOG_E("%s: Axis[%d] missing %s", rt_ofw_node_full_name(axis_np), reg, "abs-flat");
            err = -RT_EINVAL;
            goto _fail;
        }

        if (rt_ofw_prop_read_u32(axis_np, "abs-fuzz", &axis->fuzz))
        {
            rt_ofw_node_put(axis_np);

            LOG_E("%s: Axis[%d] missing %s", rt_ofw_node_full_name(axis_np), reg, "abs-fuzz");
            err = -RT_EINVAL;
            goto _fail;
        }

        if (rt_ofw_prop_read_u32_array_index(axis_np, "abs-range", 0, 2, axis->range))
        {
            rt_ofw_node_put(axis_np);

            LOG_E("%s: Axis[%d] missing %s", rt_ofw_node_full_name(axis_np), reg, "abs-range");
            err = -RT_EINVAL;
            goto _fail;
        }

        if (!(propname = rt_ofw_get_prop_fuzzy_name(key_np, ",code$")) ||
            rt_ofw_prop_read_u32(key_np, propname, &axis->code))
        {
            rt_ofw_node_put(key_np);

            LOG_E("%s: Axis[%d] missing %s", rt_ofw_node_full_name(axis_np), reg, "*,code");
            err = -RT_EINVAL;
            goto _fail;
        }

        rt_input_set_abs_params(&aj->parent, axes->code,
                axes->range[0], axes->range[1], axes->fuzz, axes->flat);
        rt_input_set_capability(&aj->parent, EV_ABS, axis->code);
    }

    if (rt_ofw_prop_read_u32(np, "poll-interval", &interval))
    {
        err = -RT_ENOSYS;
        goto _fail;
    }

    if ((err = rt_input_setup_polling(&aj->parent, adc_joysticks_poll)))
    {
        goto _fail;
    }

    rt_input_set_poll_interval(&aj->parent, interval);

    if ((err = rt_input_device_register(&aj->parent)))
    {
        goto _fail;
    }

    dev->user_data = aj;

    return RT_EOK;

_fail:
    rt_free(aj);

    return err;
}

static rt_err_t adc_joystick_remove(struct rt_platform_device *pdev)
{
    struct adc_joysticks *aj = pdev->parent.user_data;

    rt_input_device_unregister(&aj->parent);

    rt_free(aj);

    return RT_EOK;
}

static const struct rt_ofw_node_id adc_joystick_ofw_ids[] =
{
    { .compatible = "adc-joystick" },
    { /* sentinel */ }
};

static struct rt_platform_driver adc_joystick_driver =
{
    .name = "adc-joystick",
    .ids = adc_joystick_ofw_ids,

    .probe = adc_joystick_probe,
    .remove = adc_joystick_remove,
};
RT_PLATFORM_DRIVER_EXPORT(adc_joystick_driver);
