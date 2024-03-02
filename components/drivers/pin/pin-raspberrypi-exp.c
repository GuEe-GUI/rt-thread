/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-3-08      GuEe-GUI     the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "pin.raspberrypi-exp"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "pin_dm.h"
#include "../firmware/raspberrypi/firmware.h"

#define RPI_EXP_GPIO_BASE       128

#define RPI_EXP_GPIO_DIR_IN     0
#define RPI_EXP_GPIO_DIR_OUT    1

#define RPI_EXP_GPIO_NR         8

struct gpio_set_config
{
    rt_uint32_t gpio;
    rt_uint32_t direction;
    rt_uint32_t polarity;
    rt_uint32_t term_en;
    rt_uint32_t term_pull_up;
    rt_uint32_t state;
};

struct gpio_get_config
{
    rt_uint32_t gpio;
    rt_uint32_t direction;
    rt_uint32_t polarity;
    rt_uint32_t term_en;
    rt_uint32_t term_pull_up;
};

struct gpio_get_set_state
{
    rt_uint32_t gpio;
    rt_uint32_t state;
};

struct rpi_exp_gpio
{
    struct rt_device_pin parent;

    struct rpi_firmware *rpi_fw;
};

#define raw_to_rpi_exp_gpio(raw) rt_container_of(raw, struct rpi_exp_gpio, parent)

static int rpi_exp_gpio_get_polarity(struct rpi_exp_gpio *regpio, rt_base_t pin)
{
    rt_err_t err;
    struct gpio_get_config get;

    get.gpio = pin + RPI_EXP_GPIO_BASE;

    if ((err = rpi_firmware_property(regpio->rpi_fw, RPI_FIRMWARE_GET_GPIO_CONFIG,
            &get, sizeof(get))) || get.gpio != 0)
    {
        LOG_E("Failed to get PIN %u config (%s, %x)", pin, rt_strerror(err), get.gpio);

        return err ? err : -RT_EIO;
    }

    return get.polarity;
}

static void rpi_exp_gpio_pin_mode(struct rt_device *device, rt_base_t pin, rt_uint8_t mode)
{
    rt_err_t err;
    rt_int32_t polarity;
    union
    {
        struct gpio_set_config in;
        struct gpio_set_config out;
    } set;
    struct rpi_exp_gpio *regpio = raw_to_rpi_exp_gpio(device);

    if (pin >= RPI_EXP_GPIO_NR)
    {
        return;
    }

    switch (mode)
    {
    case PIN_MODE_OUTPUT:
        set.out.gpio = pin + RPI_EXP_GPIO_BASE;
        set.out.direction = RPI_EXP_GPIO_DIR_OUT;
        set.out.term_en = 0;            /* n/a as an output */
        set.out.term_pull_up = 0;       /* n/a as termination disabled */
        set.out.state = 1;              /* Output state */

        polarity = rpi_exp_gpio_get_polarity(regpio, pin);
        if (polarity < 0)
        {
            LOG_E("Failed to get PIN %u polarity error = %s", rt_strerror(polarity));

            break;
        }
        set.out.polarity = polarity;    /* Retain existing setting */

        if ((err = rpi_firmware_property(regpio->rpi_fw, RPI_FIRMWARE_SET_GPIO_CONFIG,
                &set.out, sizeof(set.out))) || set.out.gpio != 0)
        {
            LOG_E("Failed to set PIN %u to output (%s, %x)", pin, rt_strerror(err), set.out.gpio);
        }

        break;

    case PIN_MODE_INPUT:
        set.in.gpio = pin + RPI_EXP_GPIO_BASE;
        set.in.direction = RPI_EXP_GPIO_DIR_IN;
        set.in.term_en = 0;             /* Termination disabled */
        set.in.term_pull_up = 0;        /* n/a as termination disabled */
        set.in.state = 0;               /* n/a as configured as an input */

        polarity = rpi_exp_gpio_get_polarity(regpio, pin);
        if (polarity < 0)
        {
            LOG_E("Failed to get PIN %u polarity error = %s", rt_strerror(polarity));

            break;
        }
        set.in.polarity = polarity;     /* Retain existing setting */

        if ((err = rpi_firmware_property(regpio->rpi_fw, RPI_FIRMWARE_SET_GPIO_CONFIG,
                &set.in, sizeof(set.in))) || set.in.gpio != 0)
        {
            LOG_E("Failed to set PIN %u to input (%s, %x)", pin, rt_strerror(err), set.in.gpio);
        }
        break;

    default:
        break;
    }
}

static void rpi_exp_gpio_pin_write(struct rt_device *device, rt_base_t pin, rt_uint8_t value)
{
    rt_err_t err;
    struct gpio_get_set_state set;
    struct rpi_exp_gpio *regpio = raw_to_rpi_exp_gpio(device);

    if (pin >= RPI_EXP_GPIO_NR)
    {
        return;
    }

    set.gpio = pin + RPI_EXP_GPIO_BASE;
    set.state = value;

    if ((err = rpi_firmware_property(regpio->rpi_fw, RPI_FIRMWARE_SET_GPIO_STATE,
            &set, sizeof(set))) || set.gpio != 0)
    {
        LOG_E("Failed to set PIN %u state (%s, %x)", pin, rt_strerror(err), set.gpio);
    }
}

static rt_int8_t rpi_exp_gpio_pin_read(struct rt_device *device, rt_base_t pin)
{
    rt_err_t err;
    struct gpio_get_set_state get;
    struct rpi_exp_gpio *regpio = raw_to_rpi_exp_gpio(device);

    if (pin >= RPI_EXP_GPIO_NR)
    {
        return -RT_EINVAL;
    }

    get.gpio = pin + RPI_EXP_GPIO_BASE;
    get.state = 0;

    if ((err = rpi_firmware_property(regpio->rpi_fw, RPI_FIRMWARE_GET_GPIO_STATE,
            &get, sizeof(get))) || get.gpio != 0)
    {
        LOG_E("Failed to get PIN %u state (%s, %x)", pin, rt_strerror(err), get.gpio);

        return err ? err : -RT_EIO;
    }

    return !!get.state;
}

static const struct rt_pin_ops rpi_exp_gpio_pin_ops =
{
    .pin_mode = rpi_exp_gpio_pin_mode,
    .pin_write = rpi_exp_gpio_pin_write,
    .pin_read = rpi_exp_gpio_pin_read,
};

static rt_err_t rpi_exp_gpio_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    struct rt_device *dev = &pdev->parent;
    struct rt_ofw_node *np = dev->ofw_node, *fw_np;
    struct rpi_exp_gpio *rpi_gpio = rt_calloc(1, sizeof(*rpi_gpio));

    if (!rpi_gpio)
    {
        return -RT_ENOMEM;
    }

    fw_np = rt_ofw_get_parent(np);

    if (!fw_np)
    {
        err = -RT_EINVAL;
        goto _fail;
    }

    rpi_gpio->rpi_fw = rpi_firmware_get(fw_np);
    rt_ofw_node_put(fw_np);

    if (!rpi_gpio->rpi_fw)
    {
        err = -RT_EINVAL;
        goto _fail;
    }

    rpi_gpio->parent.ops = &rpi_exp_gpio_pin_ops;

    rt_dm_dev_bind_fwdata(dev, RT_NULL, &rpi_gpio->parent);

    return RT_EOK;

_fail:
    if (rpi_gpio->rpi_fw)
    {
        rpi_firmware_put(rpi_gpio->rpi_fw);
    }

    rt_free(rpi_gpio);

    return err;
}

static const struct rt_ofw_node_id rpi_exp_gpio_ofw_ids[] =
{
    { .compatible = "raspberrypi,firmware-gpio" },
    { /* sentinel */ }
};

static struct rt_platform_driver rpi_exp_gpio_driver =
{
    .name = "raspberrypi-exp-gpio",
    .ids = rpi_exp_gpio_ofw_ids,

    .probe = rpi_exp_gpio_probe,
};
RT_PLATFORM_DRIVER_EXPORT(rpi_exp_gpio_driver);
