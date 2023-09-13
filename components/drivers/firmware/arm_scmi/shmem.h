/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-26     GuEe-GUI     first version
 */

#ifndef __SCMI_SHMEM_H__
#define __SCMI_SHMEM_H__

#include <rtthread.h>
#include <rtdevice.h>

struct scmi_shared_mem;

rt_err_t scmi_shmem_msg_write(struct scmi_shared_mem *shmem, struct rt_scmi_msg *msg);
rt_err_t scmi_shmem_msg_read(struct scmi_shared_mem *shmem, struct rt_scmi_msg *msg);
void scmi_shmem_clear_channel(struct scmi_shared_mem *shmem);

/* Share memory */
#define SHMEM_HEADER_TOKEN(token)       (((token) << 18) & RT_GENMASK(31, 18))
#define SHMEM_HEADER_PROTOCOL_ID(proto) (((proto) << 10) & RT_GENMASK(17, 10))
#define SHMEM_HEADER_MESSAGE_TYPE(type) (((type) << 18) & RT_GENMASK(9, 8))
#define SHMEM_HEADER_MESSAGE_ID(id)     ((id) & RT_GENMASK(7, 0))

#endif /* __SCMI_SHMEM_H__ */
