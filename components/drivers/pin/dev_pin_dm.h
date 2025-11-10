/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-26     GuEe-GUI     first version
 */

#ifndef __PIN_DM_H__
#define __PIN_DM_H__

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

rt_err_t pin_api_init(struct rt_device_pin *gpio, rt_size_t pin_nr);

rt_err_t pin_pic_init(struct rt_device_pin *gpio, int pin_irq);
rt_err_t pin_pic_handle_isr(struct rt_device_pin *gpio, rt_base_t pin);

#ifdef RT_USING_PINCTRL
rt_err_t pin_gpio_request(struct rt_device_pin *pinctrl, rt_base_t gpio, rt_uint32_t flags);
#endif

#endif /* __DEV_PIN_DM_H__ */
