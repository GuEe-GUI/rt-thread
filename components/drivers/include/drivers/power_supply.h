/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-21     GuEe-GUI     first version
 */

#ifndef __POWER_SUPPLY_H__
#define __POWER_SUPPLY_H__

#include <rtthread.h>
#include <drivers/led.h>
#include <drivers/thermal.h>

enum rt_power_supply_type
{
    RT_POWER_SUPPLY_TYPE_UNKNOWN = 0,
    RT_POWER_SUPPLY_TYPE_BATTERY,
    RT_POWER_SUPPLY_TYPE_UPS,
    RT_POWER_SUPPLY_TYPE_MAINS,
    RT_POWER_SUPPLY_TYPE_USB,               /* Standard Downstream Port */
    RT_POWER_SUPPLY_TYPE_USB_DCP,           /* Dedicated Charging Port */
    RT_POWER_SUPPLY_TYPE_USB_CDP,           /* Charging Downstream Port */
    RT_POWER_SUPPLY_TYPE_USB_ACA,           /* Accessory Charger Adapters */
    RT_POWER_SUPPLY_TYPE_USB_TYPE_C,        /* Type C Port */
    RT_POWER_SUPPLY_TYPE_USB_PD,            /* Power Delivery Port */
    RT_POWER_SUPPLY_TYPE_USB_PD_DRP,        /* PD Dual Role Port */
    RT_POWER_SUPPLY_TYPE_APPLE_BRICK_ID,    /* Apple Charging Method */
    RT_POWER_SUPPLY_TYPE_WIRELESS,          /* Wireless */
};

enum rt_power_supply_property
{
    /* Properties of type `int' */
    RT_POWER_SUPPLY_PROP_STATUS = 0,
    RT_POWER_SUPPLY_PROP_CHARGE_TYPE,
    RT_POWER_SUPPLY_PROP_HEALTH,
    RT_POWER_SUPPLY_PROP_PRESENT,
    RT_POWER_SUPPLY_PROP_ONLINE,
    RT_POWER_SUPPLY_PROP_AUTHENTIC,
    RT_POWER_SUPPLY_PROP_TECHNOLOGY,
    RT_POWER_SUPPLY_PROP_CYCLE_COUNT,
    RT_POWER_SUPPLY_PROP_VOLTAGE_MAX,
    RT_POWER_SUPPLY_PROP_VOLTAGE_MIN,
    RT_POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
    RT_POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
    RT_POWER_SUPPLY_PROP_VOLTAGE_NOW,
    RT_POWER_SUPPLY_PROP_VOLTAGE_AVG,
    RT_POWER_SUPPLY_PROP_VOLTAGE_OCV,
    RT_POWER_SUPPLY_PROP_VOLTAGE_BOOT,
    RT_POWER_SUPPLY_PROP_CURRENT_MAX,
    RT_POWER_SUPPLY_PROP_CURRENT_NOW,
    RT_POWER_SUPPLY_PROP_CURRENT_AVG,
    RT_POWER_SUPPLY_PROP_CURRENT_BOOT,
    RT_POWER_SUPPLY_PROP_POWER_NOW,
    RT_POWER_SUPPLY_PROP_POWER_AVG,
    RT_POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
    RT_POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN,
    RT_POWER_SUPPLY_PROP_CHARGE_FULL,
    RT_POWER_SUPPLY_PROP_CHARGE_EMPTY,
    RT_POWER_SUPPLY_PROP_CHARGE_NOW,
    RT_POWER_SUPPLY_PROP_CHARGE_AVG,
    RT_POWER_SUPPLY_PROP_CHARGE_COUNTER,
    RT_POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
    RT_POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
    RT_POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
    RT_POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
    RT_POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
    RT_POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
    RT_POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD, /* in percents! */
    RT_POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD, /* in percents! */
    RT_POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
    RT_POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
    RT_POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
    RT_POWER_SUPPLY_PROP_INPUT_POWER_LIMIT,
    RT_POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
    RT_POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN,
    RT_POWER_SUPPLY_PROP_ENERGY_FULL,
    RT_POWER_SUPPLY_PROP_ENERGY_EMPTY,
    RT_POWER_SUPPLY_PROP_ENERGY_NOW,
    RT_POWER_SUPPLY_PROP_ENERGY_AVG,
    RT_POWER_SUPPLY_PROP_CAPACITY, /* in percents! */
    RT_POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN, /* in percents! */
    RT_POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX, /* in percents! */
    RT_POWER_SUPPLY_PROP_CAPACITY_ERROR_MARGIN, /* in percents! */
    RT_POWER_SUPPLY_PROP_CAPACITY_LEVEL,
    RT_POWER_SUPPLY_PROP_TEMP,
    RT_POWER_SUPPLY_PROP_TEMP_MAX,
    RT_POWER_SUPPLY_PROP_TEMP_MIN,
    RT_POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
    RT_POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
    RT_POWER_SUPPLY_PROP_TEMP_AMBIENT,
    RT_POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MIN,
    RT_POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MAX,
    RT_POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
    RT_POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
    RT_POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
    RT_POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
    RT_POWER_SUPPLY_PROP_TYPE, /* use power_supply.type instead */
    RT_POWER_SUPPLY_PROP_USB_TYPE,
    RT_POWER_SUPPLY_PROP_SCOPE,
    RT_POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
    RT_POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
    RT_POWER_SUPPLY_PROP_CALIBRATE,
    RT_POWER_SUPPLY_PROP_MANUFACTURE_YEAR,
    RT_POWER_SUPPLY_PROP_MANUFACTURE_MONTH,
    RT_POWER_SUPPLY_PROP_MANUFACTURE_DAY,
    /* Properties of type `const char *' */
    RT_POWER_SUPPLY_PROP_MODEL_NAME,
    RT_POWER_SUPPLY_PROP_MANUFACTURER,
    RT_POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

union rt_power_supply_property_val
{
    int intval;
    const char *strval;
};

struct rt_power_supply_ops;

struct rt_power_supply
{
    struct rt_device parent;

    enum power_supply_type type;

    rt_size_t properties_nr;
    enum rt_power_supply_property *properties;

    const struct rt_power_supply_ops *ops;

#ifdef RT_USING_THERMAL
    struct rt_thermal_zone_device *thermal_dev;
    struct rt_thermal_cooling_device *thermal_cool_dev;
#endif

#ifdef RT_USING_LED
    struct rt_led_device *charging_full_led;
    struct rt_led_device *charging_led;
    struct rt_led_device *full_led;
    struct rt_led_device *online_led;
    struct rt_led_device *charging_blink_full_solid_led;
#endif /* RT_USING_LED */
}

struct rt_power_supply_ops
{
    rt_err_t (*get_property)(struct rt_power_supply *,
            enum power_supply_property prop, union rt_power_supply_property_val *val);
    rt_err_t (*set_property)(struct rt_power_supply *,
            enum power_supply_property prop, const union rt_power_supply_property_val *val);
};

rt_err_t rt_power_supply_register(struct rt_power_supply *psy);
rt_err_t rt_power_supply_unregister(struct rt_power_supply *psy);

void rt_power_supply_changed(struct rt_power_supply *psy);

#endif /* __POWER_SUPPLY_H__ */
