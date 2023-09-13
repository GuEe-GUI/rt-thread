/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-3-08      GuEe-GUI     the first version
 */

#ifndef __THERMAL_H__
#define __THERMAL_H__

#include <rtdef.h>

/* No upper/lower limit requirement */
#define RT_THERMAL_NO_LIMIT     ((rt_uint32_t)~0)

struct rt_thermal_zone_ops;
struct rt_thermal_cooling_device;
struct rt_thermal_cooling_device_ops;

struct rt_thermal_trip
{
    int temperature;    /* temperature value in millidegree celsius */
    int hysteresis;     /* relative hysteresis in millidegree celsius */
    void *priv;
};

struct rt_thermal_zone_params
{
    int sustainable_power; /* Sustainable power (heat) that this thermal zone
                            * can dissipate in mW
                            */

    int slope;  /* slope of a linear temperature adjustment curve */
    int offset; /* offset of a linear temperature adjustment curve */
};

struct rt_thermal_zone_device
{
    struct rt_device parent;

    int zone_id;
    const struct rt_thermal_zone_ops *ops;

    rt_size_t trips_nr;
    struct rt_thermal_trip *trips;
    struct rt_thermal_zone_params params;

    rt_list_t notifier_nodes;

    rt_list_t zone_list;
    rt_list_t cooling_list;
    struct rt_thermal_cooling_device *cooling_dev;

    void *priv;
};

struct rt_thermal_zone_ops
{
    rt_err_t (*bind)(struct rt_thermal_zone_device *, struct rt_thermal_cooling_device *);
    rt_err_t (*unbind)(struct rt_thermal_zone_device *, struct rt_thermal_cooling_device *);
    rt_err_t (*get_temp)(struct rt_thermal_zone_device *, int *out_temp);
    rt_err_t (*set_trips)(struct rt_thermal_zone_device *, int low_temp, int high_temp);
    rt_err_t (*set_trip_temp)(struct rt_thermal_zone_device *, int trip_id, int temp);
    rt_err_t (*set_trip_hyst)(struct rt_thermal_zone_device *, int trip_id, int hyst);
};

struct rt_thermal_cooling_device
{
    struct rt_device parent;

    const struct rt_thermal_cooling_device_ops *ops;

    rt_ubase_t max_level;   /* The cooling capacity indicator */

    rt_list_t zone_nodes;

    void *priv;
};

struct rt_thermal_cooling_device_ops
{
    rt_err_t (*get_max_level)(struct rt_thermal_cooling_device *, rt_ubase_t *out_level);
    rt_err_t (*get_cur_level)(struct rt_thermal_cooling_device *, rt_ubase_t *out_level);
    rt_err_t (*set_cur_level)(struct rt_thermal_cooling_device *, rt_ubase_t level);
};

struct rt_thermal_notifier;

#define RT_THERMAL_MSG_EVENT_UNSPECIFIED                RT_BIT(0) /* Unspecified event */
#define RT_THERMAL_MSG_EVENT_TEMP_SAMPLE                RT_BIT(0) /* New Temperature sample */
#define RT_THERMAL_MSG_TRIP_VIOLATED                    RT_BIT(0) /* TRIP Point violation */
#define RT_THERMAL_MSG_TRIP_CHANGED                     RT_BIT(0) /* TRIP Point temperature changed */
#define RT_THERMAL_MSG_DEVICE_DOWN                      RT_BIT(0) /* Thermal device is down */
#define RT_THERMAL_MSG_DEVICE_UP                        RT_BIT(0) /* Thermal device is up after a down event */
#define RT_THERMAL_MSG_DEVICE_POWER_CAPABILITY_CHANGED  RT_BIT(0) /* power capability changed */
#define RT_THERMAL_MSG_TABLE_CHANGED                    RT_BIT(0) /* Thermal table(s) changed */
#define RT_THERMAL_MSG_EVENT_KEEP_ALIVE                 RT_BIT(0) /* Request for user space handler to respond */

typedef rt_err_t (*rt_thermal_notifier_callback)(struct rt_thermal_notifier *notifier,
        rt_ubase_t msg);

struct rt_thermal_notifier
{
    rt_list_t list;

    struct rt_thermal_zone_device *zdev;
    rt_thermal_notifier_callback callback;
    void *priv;
};

rt_err_t rt_thermal_zone_device_register(struct rt_thermal_zone_device *);
rt_err_t rt_thermal_zone_device_unregister(struct rt_thermal_zone_device *);

rt_err_t rt_thermal_cooling_device_register(struct rt_thermal_cooling_device *);
rt_err_t rt_thermal_cooling_device_unregister(struct rt_thermal_cooling_device *);

rt_err_t rt_thermal_zone_notifier_register(struct rt_thermal_zone_device *,
        struct rt_thermal_notifier *notifier);
rt_err_t rt_thermal_zone_notifier_unregister(struct rt_thermal_zone_device *,
        struct rt_thermal_notifier *notifier);

void rt_thermal_zone_device_update(struct rt_thermal_zone_device *, rt_ubase_t msg);

rt_err_t rt_thermal_zone_get_trip(struct rt_thermal_zone_device *, int trip_id,
        struct rt_thermal_trip *out_trip);

#endif /* __THERMAL_H__ */
