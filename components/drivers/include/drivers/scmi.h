/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-26     GuEe-GUI     first version
 */

#ifndef __SCMI_H__
#define __SCMI_H__

#include <rtdef.h>
#include <drivers/misc.h>
#include <drivers/byteorder.h>

#define SCMI_PROTOCOL_ID_BASE       0x10
#define SCMI_PROTOCOL_ID_POWER      0x11
#define SCMI_PROTOCOL_ID_SYSTEM     0x12
#define SCMI_PROTOCOL_ID_PERF       0x13
#define SCMI_PROTOCOL_ID_CLOCK      0x14
#define SCMI_PROTOCOL_ID_SENSOR     0x15
#define SCMI_PROTOCOL_ID_RESET      0x16
#define SCMI_PROTOCOL_ID_VOLTAGE    0x17
#define SCMI_PROTOCOL_ID_POWERCAP   0x18

#define SCMI_SUCCESS                0       /* Success */
#define SCMI_ERR_SUPPORT            (-1)    /* Not supported */
#define SCMI_ERR_PARAMS             (-2)    /* Invalid Parameters */
#define SCMI_ERR_ACCESS             (-3)    /* Invalid access/permission denied */
#define SCMI_ERR_ENTRY              (-4)    /* Not found */
#define SCMI_ERR_RANGE              (-5)    /* Value out of range */
#define SCMI_ERR_BUSY               (-6)    /* Device busy */
#define SCMI_ERR_COMMS              (-7)    /* Communication Error */
#define SCMI_ERR_GENERIC            (-8)    /* Generic Error */
#define SCMI_ERR_HARDWARE           (-9)    /* Hardware Error */
#define SCMI_ERR_PROTOCOL           (-10)   /* Protocol Error */

#define SCMI_MAX_STR_SIZE           64
#define SCMI_SHORT_NAME_MAX_SIZE    16
#define SCMI_MAX_NUM_RATES          16

struct rt_scmi_device;

/*
 * struct rt_scmi_msg - Context of a SCMI message sent and the response received
 *
 * @protocol_id:    SCMI protocol ID
 * @message_id:     SCMI message ID for a defined protocol ID
 * @in_msg:         Pointer to the message payload sent by the driver
 * @in_msg_size:    Byte size of the message payload sent
 * @out_msg:        Pointer to buffer to store response message payload
 * @out_msg_size:   Byte size of the response buffer and response payload
 */
struct rt_scmi_msg
{
    struct rt_scmi_device *sdev;

    rt_uint32_t message_id;

    rt_uint8_t *in_msg;
    rt_size_t in_msg_size;

    rt_uint8_t *out_msg;
    rt_size_t out_msg_size;
};

/* Helper macro to match a message on input/output array references */
#define RT_SCMI_MSG_IN(MSG_ID, IN, OUT) \
(struct rt_scmi_msg) {                  \
    .message_id = MSG_ID,               \
    .in_msg = (rt_uint8_t *)&(IN),      \
    .in_msg_size = sizeof(IN),          \
    .out_msg = (rt_uint8_t *)&(OUT),    \
    .out_msg_size = sizeof(OUT),        \
}

enum scmi_common_message_id
{
    SCMI_COM_MSG_VERSION = 0x0,
    SCMI_COM_MSG_ATTRIBUTES = 0x1,
    SCMI_COM_MSG_MESSAGE_ATTRIBUTES = 0x2,
};

/*
 * SCMI Power Protocol
 */
enum scmi_power_protocol_cmd
{
    SCMI_POWER_DOMAIN_ATTRIBUTES = 0x3,
    SCMI_POWER_STATE_SET = 0x4,
    SCMI_POWER_STATE_GET = 0x5,
    SCMI_POWER_STATE_NOTIFY = 0x6,
    SCMI_POWER_DOMAIN_NAME_GET = 0x8,
};

struct scmi_power_attributes
{
    rt_le16_t num_domains;
    rt_le16_t reserved;
    rt_le32_t stats_addr_low;
    rt_le32_t stats_addr_high;
    rt_le32_t stats_size;
};

struct scmi_power_state_set_in
{
    rt_le32_t flags;
    rt_le32_t domain;
#define SCMI_POWER_STATE_TYPE_SHIFT         30
#define SCMI_POWER_STATE_ID_MASK            (RT_BIT(28) - 1)
#define SCMI_POWER_STATE_PARAM(type, id)    ((((type) & RT_BIT(0)) << SCMI_POWER_STATE_TYPE_SHIFT) | ((id) & SCMI_POWER_STATE_ID_MASK))
#define SCMI_POWER_STATE_GENERIC_ON         SCMI_POWER_STATE_PARAM(0, 0)
#define SCMI_POWER_STATE_GENERIC_OFF        SCMI_POWER_STATE_PARAM(1, 0)
    rt_le32_t state;
};

struct scmi_power_state_set_out
{
    rt_le32_t state;
};

/*
 * SCMI Clock Protocol
 */
enum scmi_clock_message_id
{
    SCMI_CLOCK_ATTRIBUTES = 0x3,
    SCMI_CLOCK_DESCRIBE_RATES = 0x4,
    SCMI_CLOCK_RATE_SET = 0x5,
    SCMI_CLOCK_RATE_GET = 0x6,
    SCMI_CLOCK_CONFIG_SET = 0x7,
};

/**
 * struct scmi_clk_state_in - Message payload for CLOCK_CONFIG_SET command
 * @clock_id:   SCMI clock ID
 * @attributes: Attributes of the targets clock state
 */
struct scmi_clk_state_in
{
    rt_le32_t clock_id;
    rt_le32_t attributes;
};

/**
 * struct scmi_clk_state_out - Response payload for CLOCK_CONFIG_SET command
 * @status: SCMI command status
 */
struct scmi_clk_state_out
{
    rt_le32_t status;
};

/**
 * struct scmi_clk_state_in - Message payload for CLOCK_RATE_GET command
 * @clock_id:   SCMI clock ID
 * @attributes: Attributes of the targets clock state
 */
struct scmi_clk_rate_get_in
{
    rt_le32_t clock_id;
};

/**
 * struct scmi_clk_rate_get_out - Response payload for CLOCK_RATE_GET command
 * @status:     SCMI command status
 * @rate_lsb:   32bit LSB of the clock rate in Hertz
 * @rate_msb:   32bit MSB of the clock rate in Hertz
 */
struct scmi_clk_rate_get_out
{
    rt_le32_t status;
    rt_le32_t rate_lsb;
    rt_le32_t rate_msb;
};

/**
 * struct scmi_clk_state_in - Message payload for CLOCK_RATE_SET command
 * @clock_id:   SCMI clock ID
 * @flags:      Flags for the clock rate set request
 * @rate_lsb:   32bit LSB of the clock rate in Hertz
 * @rate_msb:   32bit MSB of the clock rate in Hertz
 */
struct scmi_clk_rate_set_in
{
#define SCMI_CLK_RATE_ASYNC_NOTIFY  RT_BIT(0)
#define SCMI_CLK_RATE_ASYNC_NORESP  (RT_BIT(0) | RT_BIT(1))
#define SCMI_CLK_RATE_ROUND_DOWN    0
#define SCMI_CLK_RATE_ROUND_UP      RT_BIT(2)
#define SCMI_CLK_RATE_ROUND_CLOSEST RT_BIT(3)
    rt_le32_t flags;
    rt_le32_t clock_id;
    rt_le32_t rate_lsb;
    rt_le32_t rate_msb;
};

/**
 * struct scmi_clk_rate_set_out - Response payload for CLOCK_RATE_SET command
 * @status: SCMI command status
 */
struct scmi_clk_rate_set_out
{
    rt_le32_t status;
};

/*
 * SCMI Reset Domain Protocol
 */

enum scmi_reset_message_id
{
    SCMI_RESET_ATTRIBUTES = 0x3,
    SCMI_RESET_RESET = 0x4,
};

#define SCMI_ATTRIBUTES_FLAG_ASYNC   RT_BIT(31)
#define SCMI_ATTRIBUTES_FLAG_NOTIF   RT_BIT(30)

/**
 * struct scmi_attr_in - Payload for RESET_DOMAIN_ATTRIBUTES message
 * @domain_id:  SCMI reset domain ID
 */
struct scmi_attr_in
{
    rt_le32_t domain_id;
};

/**
 * struct scmi_attr_out - Payload for RESET_DOMAIN_ATTRIBUTES response
 * @attributes: Retrieved attributes of the reset domain
 * @latency:    Reset cycle max lantency
 * @name:       Reset domain name
 */
struct scmi_attr_out
{
    rt_le32_t attributes;
    rt_le32_t latency;
    rt_uint8_t name[SCMI_SHORT_NAME_MAX_SIZE];
};

/**
 * struct scmi_reset_in - Message payload for RESET command
 * @domain_id:      SCMI reset domain ID
 * @flags:          Flags for the reset request
 * @reset_state:    Reset target state
 */
struct scmi_reset_in
{
    rt_le32_t domain_id;
#define SCMI_RESET_FLAG_RESET        RT_BIT(0)
#define SCMI_RESET_FLAG_ASSERT       RT_BIT(1)
#define SCMI_RESET_FLAG_ASYNC        RT_BIT(2)
    rt_le32_t flags;
#define SCMI_ARCH_COLD_RESET        0
    rt_le32_t reset_state;
};

/**
 * struct scmi_reset_out - Response payload for RESET command
 * @status: SCMI command status
 */
struct scmi_reset_out
{
    rt_le32_t status;
};

/*
 * SCMI Voltage Domain Protocol
 */

enum scmi_voltage_domain_message_id
{
    SCMI_VOLTAGE_DOMAIN_ATTRIBUTES = 0x3,
    SCMI_VOLTAGE_DOMAIN_CONFIG_SET = 0x5,
    SCMI_VOLTAGE_DOMAIN_CONFIG_GET = 0x6,
    SCMI_VOLTAGE_DOMAIN_LEVEL_SET = 0x7,
    SCMI_VOLTAGE_DOMAIN_LEVEL_GET = 0x8,
};

#define SCMI_VOLTAGE_NAME_LEN       16

#define SCMI_VOLTAGE_CONFIG_MASK    RT_GENMASK(3, 0)
#define SCMI_VOLTAGE_CONFIG_OFF     0
#define SCMI_VOLTAGE_CONFIG_ON      0x7

/**
 * struct scmi_voltage_attr_in - Payload for VOLTAGE_DOMAIN_ATTRIBUTES message
 * @domain_id:  SCMI voltage domain ID
 */
struct scmi_voltage_attr_in
{
    rt_le32_t domain_id;
};

/**
 * struct scmi_voltage_attr_out - Payload for VOLTAGE_DOMAIN_ATTRIBUTES response
 * @attributes: Retrieved attributes of the voltage domain
 * @name:       Voltage domain name
 */
struct scmi_voltage_attr_out
{
    rt_le32_t attributes;
    char name[SCMI_VOLTAGE_NAME_LEN];
};

/**
 * struct scmi_voltage_config_set_in - Message payload for VOLTAGE_CONFIG_SET cmd
 * @domain_id:  SCMI voltage domain ID
 * @config:     Configuration data of the voltage domain
 */
struct scmi_voltage_config_set_in
{
    rt_le32_t domain_id;
    rt_le32_t config;
};

/**
 * struct scmi_voltage_config_set_out - Response for VOLTAGE_CONFIG_SET command
 * @status: SCMI command status
 */
struct scmi_voltage_config_set_out
{
    rt_le32_t status;
};

/**
 * struct scmi_voltage_config_get_in - Message payload for VOLTAGE_CONFIG_GET cmd
 * @domain_id:  SCMI voltage domain ID
 */
struct scmi_voltage_config_get_in
{
    rt_le32_t domain_id;
};

/**
 * struct scmi_voltage_config_get_out - Response for VOLTAGE_CONFIG_GET command
 * @status: SCMI command status
 * @config: Configuration data of the voltage domain
 */
struct scmi_voltage_config_get_out
{
    rt_le32_t status;
    rt_le32_t config;
};

/**
 * struct scmi_voltage_level_set_in - Message payload for VOLTAGE_LEVEL_SET cmd
 * @domain_id:      SCMI voltage domain ID
 * @flags:      Parameter flags for configuring target level
 * @voltage_level:  Target voltage level in microvolts (uV)
 */
struct scmi_voltage_level_set_in
{
    rt_le32_t domain_id;
    rt_le32_t flags;
    rt_le32_t voltage_level;
};

/**
 * struct scmi_voltage_level_set_out - Response for VOLTAGE_LEVEL_SET command
 * @status: SCMI    command status
 */
struct scmi_voltage_level_set_out
{
    rt_le32_t status;
};

/**
 * struct scmi_voltage_level_get_in - Message payload for VOLTAGE_LEVEL_GET cmd
 * @domain_id:      SCMI voltage domain ID
 */
struct scmi_voltage_level_get_in
{
    rt_le32_t domain_id;
};

/**
 * struct scmi_voltage_level_get_out - Response for VOLTAGE_LEVEL_GET command
 * @status:     SCMI command status
 * @voltage_level:  Voltage level in microvolts (uV)
 */
struct scmi_voltage_level_get_out
{
    rt_le32_t status;
    rt_le32_t voltage_level;
};

struct scmi_agent;

struct rt_scmi_device_id
{
    rt_uint8_t protocol_id;
    const char *name;
};

struct rt_scmi_device
{
    struct rt_device parent;

    const char *name;
    rt_uint8_t protocol_id;

    struct scmi_agent *agent;
};

struct rt_scmi_driver
{
    struct rt_driver parent;

    const char *name;
    const struct rt_scmi_device_id *ids;

    rt_err_t (*probe)(struct rt_scmi_device *sdev);
    rt_err_t (*remove)(struct rt_scmi_device *sdev);
    rt_err_t (*shutdown)(struct rt_scmi_device *sdev);
};

rt_err_t rt_scmi_driver_register(struct rt_scmi_driver *driver);
rt_err_t rt_scmi_device_register(struct rt_scmi_device *device);

#define RT_SCMI_DRIVER_EXPORT(driver)  RT_DRIVER_EXPORT(driver, scmi, BUILIN)

rt_err_t rt_scmi_process_msg(struct rt_scmi_device *sdev, struct rt_scmi_msg *msg);
const char *rt_scmi_strerror(rt_base_t err);

#endif /* __SCMI_H__ */
