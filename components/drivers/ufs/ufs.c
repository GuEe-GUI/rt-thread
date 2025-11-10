/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "rtdm.ufs"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

rt_inline rt_uint32_t ufs_ahit_timer_scale_us(rt_uint8_t ts)
{
    switch (ts)
    {
    case 0 /* 0b000 */: return 1;       /* 1 us */
    case 1 /* 0b001 */: return 10;      /* 10 us */
    case 2 /* 0b010 */: return 100;     /* 100 us */
    case 3 /* 0b011 */: return 1000;    /* 1 ms */
    case 4 /* 0b100 */: return 10000;   /* 10 ms */
    case 5 /* 0b101 */: return 100000;  /* 100 ms */
    default:
        return 0;
    }
}

static rt_err_t ufs_host_reset(struct rt_scsi_device *sdev)
{
    struct rt_ufs_host *ufs = rt_container_of(sdev->host, struct rt_ufs_host, parent);

    if (ufs->ops->reset)
    {
        return ufs->ops->reset(ufs);
    }

    return RT_EOK;
}

static rt_err_t ufs_host_transfer(struct rt_scsi_device *sdev,
        struct rt_scsi_cmd *cmd)
{
    rt_err_t err;
    struct rt_ufs_host *ufs = rt_container_of(sdev->host, struct rt_ufs_host, parent);

    switch (cmd->op.unknow.opcode)
    {
    case RT_SCSI_CMD_REQUEST_SENSE:
        break;

    case RT_SCSI_CMD_READ10:
        break;

    case RT_SCSI_CMD_READ16:
        break;

    case RT_SCSI_CMD_WRITE10:
        break;

    case RT_SCSI_CMD_WRITE16:
        break;

    case RT_SCSI_CMD_SYNCHRONIZE_CACHE10:
        break;

    case RT_SCSI_CMD_SYNCHRONIZE_CACHE16:
        break;

    case RT_SCSI_CMD_READ_CAPACITY10:
    {
        rt_size_t last_block, block_size;
        struct rt_scsi_read_capacity10_data *data = &cmd->data.read_capacity10;

        if (!err)
        {
            if (last_block > 0x100000000ULL)
            {
                last_block = 0xffffffff;
            }

            data->last_block = rt_cpu_to_be32(last_block);
            data->block_size = rt_cpu_to_be32(block_size);
        }
    }
        break;

    case RT_SCSI_CMD_READ_CAPACITY16:
    {
        rt_size_t last_block, block_size;
        struct rt_scsi_read_capacity16_data *data = &cmd->data.read_capacity16;

        if (!err)
        {
            data->last_block = rt_cpu_to_be64(last_block);
            data->block_size = rt_cpu_to_be32(block_size);
        }
    }
        break;

    case RT_SCSI_CMD_TEST_UNIT_READY:
        err = RT_EOK;
        break;

    case RT_SCSI_CMD_INQUIRY:
    {
        rt_uint32_t value;
        struct rt_scsi_inquiry_data *inquiry = &cmd->data.inquiry;

        inquiry->devtype = SCSI_DEVICE_TYPE_DIRECT;
        inquiry->rmb = 0;
        inquiry->length = 0;

        rt_sprintf(inquiry->vendor, "%x", HWREG32(ufs->regs + RT_UFS_REG_HCMID));

        value = HWREG32(ufs->regs + RT_UFS_REG_HCPID);
        rt_memcpy(inquiry->prodid, &value, sizeof(value));

        value = HWREG32(ufs->regs + RT_UFS_REG_VER);
        rt_memcpy(inquiry->prodrev, &value, sizeof(value));

        err = RT_EOK;
    }
        break;

    case RT_SCSI_CMD_READ12:
    case RT_SCSI_CMD_WRITE12:
    case RT_SCSI_CMD_WRITE_SAME10:
    case RT_SCSI_CMD_WRITE_SAME16:
    case RT_SCSI_CMD_MODE_SENSE:
    case RT_SCSI_CMD_MODE_SENSE10:
    case RT_SCSI_CMD_MODE_SELECT:
    case RT_SCSI_CMD_MODE_SELECT10:
        err = -RT_ENOSYS;
        break;

    default:
        err = -RT_EINVAL;
        break;
    }

    return err;
}

static struct rt_scsi_ops ufs_host_ops =
{
    .reset = ufs_host_reset,
    .transfer = ufs_host_transfer,
};

rt_err_t rt_ufs_host_register(struct rt_ufs_host *ufs)
{
    rt_err_t err;
    struct rt_scsi_host *scsi;

    if (!ufs || !ufs->ops)
    {
        return -RT_EINVAL;
    }

    ufs->cap = HWREG32(ufs->regs + RT_UFS_REG_CAP);

    scsi = &ufs->parent;
    scsi->ops = &ufs_host_ops;
    scsi->max_id = rt_max_t(rt_size_t, scsi->max_id, 1);
    scsi->max_lun = rt_max_t(rt_size_t, scsi->max_lun, 1);
    scsi->parallel_io = RT_TRUE;

    if ((err = rt_scsi_host_register(scsi)))
    {
        goto _fail;
    }

    return RT_EOK;

_fail:
    return err;
}

rt_err_t rt_ufs_host_unregister(struct rt_ufs_host *ufs)
{
    rt_scsi_host_unregister(&ufs->parent);

    return RT_EOK;
}
