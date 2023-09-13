/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-09-23     GuEe-GUI     first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "mailbox.rockchip"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define MAILBOX_A2B_INTEN   0x00
#define MAILBOX_A2B_STATUS  0x04
#define MAILBOX_A2B_CMD(x)  (0x08 + (x) * 8)
#define MAILBOX_A2B_DAT(x)  (0x0c + (x) * 8)

#define MAILBOX_B2A_INTEN   0x28
#define MAILBOX_B2A_STATUS  0x2c
#define MAILBOX_B2A_CMD(x)  (0x30 + (x) * 8)
#define MAILBOX_B2A_DAT(x)  (0x34 + (x) * 8)

struct rockchip_mbox_msg
{
    rt_uint32_t cmd;
    int rx_size;
};

struct rockchip_mbox_soc_data
{
    int num_chans;
};

struct rockchip_mbox_chan
{
    int idx;
    int irq;
    struct rockchip_mbox_msg *msg;
    struct rockchip_mbox *rk_mbox;
};

struct rockchip_mbox
{
    struct rt_mbox_controller parent;

    void *regs;
    struct rt_clk *pclk;
    struct rt_thread *irq_thread;

    rt_uint32_t buf_size;
    struct rockchip_mbox_chan chans[];
};

#define raw_to_rockchip_mbox(raw) rt_container_of(raw, struct rockchip_mbox, parent)

rt_inline rt_uint32_t rockchip_mbox_readl(struct rockchip_mbox *rk_mbox, int offset)
{
    return HWREG32(rk_mbox->regs + offset);
}

rt_inline void rockchip_mbox_writel(struct rockchip_mbox *rk_mbox, int offset,
        rt_uint32_t value)
{
    HWREG32(rk_mbox->regs + offset) = value;
}

static rt_err_t rockchip_mbox_request(struct rt_mbox_chan *chan)
{
    struct rockchip_mbox *rk_mbox = raw_to_rockchip_mbox(chan->ctrl);

    /* Enable all B2A interrupts */
    rockchip_mbox_writel(rk_mbox, MAILBOX_B2A_INTEN,
            (1 << rk_mbox->parent.num_chans) - 1);

    return RT_EOK;
}

static void rockchip_mbox_free(struct rt_mbox_chan *chan)
{
    int index;
    struct rockchip_mbox *rk_mbox = raw_to_rockchip_mbox(chan->ctrl);

    /* Disable all B2A interrupts */
    rockchip_mbox_writel(rk_mbox, MAILBOX_B2A_INTEN, 0);

    index = chan - rk_mbox->parent.chans;
    rk_mbox->chans[index].msg = RT_NULL;
}

static rt_err_t rockchip_mbox_send(struct rt_mbox_chan *chan, const void *data)
{
    int index;
    struct rockchip_mbox_msg *msg = (void *)data;
    struct rockchip_mbox *rk_mbox = raw_to_rockchip_mbox(chan->ctrl);

    if (msg->rx_size > rk_mbox->buf_size)
    {
        LOG_E("Transmit size over buf size(%d)", rk_mbox->buf_size);

        return -RT_EINVAL;
    }

    index = chan - rk_mbox->parent.chans;
    rk_mbox->chans[index].msg = msg;

    rockchip_mbox_writel(rk_mbox, MAILBOX_A2B_CMD(index), msg->cmd);
    rockchip_mbox_writel(rk_mbox, MAILBOX_A2B_DAT(index), msg->rx_size);

    return RT_EOK;
}

static const struct rt_mbox_controller_ops rk_mbox_ops =
{
    .request = rockchip_mbox_request,
    .free = rockchip_mbox_free,
    .send = rockchip_mbox_send,
};

static void rockchip_mbox_thread_isr(void *param)
{
    struct rockchip_mbox_msg *msg;
    struct rockchip_mbox *rk_mbox = param;

    while (RT_TRUE)
    {
        rt_thread_suspend(rk_mbox->irq_thread);
        rt_schedule();

        for (int idx = 0; idx < rk_mbox->parent.num_chans; ++idx)
        {
            msg = rk_mbox->chans[idx].msg;

            if (!msg)
            {
                LOG_E("Chan[%d]: B2A message is NULL", idx);

                break;
            }

            rt_mbox_recv(&rk_mbox->parent.chans[idx], msg);
            rk_mbox->chans[idx].msg = RT_NULL;
        }
    }
}

static void rockchip_mbox_isr(int irqno, void *param)
{
    rt_uint32_t status;
    struct rockchip_mbox *rk_mbox = param;

    status = rockchip_mbox_readl(rk_mbox, MAILBOX_B2A_STATUS);

    for (int idx = 0; idx < rk_mbox->parent.num_chans; ++idx)
    {
        if ((status & RT_BIT(idx)) && (irqno == rk_mbox->chans[idx].irq))
        {
            /* Clear mbox interrupt */
            rockchip_mbox_writel(rk_mbox, MAILBOX_B2A_STATUS, RT_BIT(idx));

            rt_thread_resume(rk_mbox->irq_thread);

            return;
        }
    }

    return;
}

static void rockchip_mbox_free_resource(struct rockchip_mbox *rk_mbox)
{
    if (rk_mbox->regs)
    {
        rt_iounmap(rk_mbox->regs);
    }

    if (!rt_is_err_or_null(rk_mbox->pclk))
    {
        rt_clk_disable_unprepare(rk_mbox->pclk);
        rt_clk_put(rk_mbox->pclk);
    }

    if (rk_mbox->irq_thread)
    {
        rt_thread_delete(rk_mbox->irq_thread);
    }

    rt_free(rk_mbox);
}

static rt_err_t rockchip_mbox_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    char chan_name[RT_NAME_MAX];
    rt_uint64_t io_addr, io_size;
    struct rockchip_mbox *rk_mbox;
    struct rockchip_mbox_chan *chan;
    struct rt_device *dev = &pdev->parent;
    const struct rockchip_mbox_soc_data *soc_data = pdev->id->data;

    rk_mbox = rt_calloc(1, sizeof(*rk_mbox) +
            soc_data->num_chans * sizeof(struct rockchip_mbox_chan));

    if (!rk_mbox)
    {
        return -RT_ENOMEM;
    }

    if ((err = rt_dm_dev_get_address(dev, 0, &io_addr, &io_size)))
    {
        goto _fail;
    }

    rk_mbox->regs = rt_ioremap((void *)io_addr, (size_t)io_size);

    if (!rk_mbox->regs)
    {
        err = -RT_EIO;
        goto _fail;
    }

    rk_mbox->pclk = rt_clk_get_by_name(dev, "pclk_mailbox");

    if (rt_is_err(rk_mbox->pclk))
    {
        err = rt_ptr_err(rk_mbox->pclk);

        goto _fail;
    }

    if ((err = rt_clk_prepare_enable(rk_mbox->pclk)))
    {
        goto _fail;
    }

    rk_mbox->irq_thread = rt_thread_create("rk_mbox", &rockchip_mbox_thread_isr,
            rk_mbox, SYSTEM_THREAD_STACK_SIZE, RT_THREAD_PRIORITY_MAX / 2, 10);

    if (!rk_mbox->irq_thread)
    {
        LOG_E("Create Mailbox IRQ thread fail");
        goto _fail;
    }

    rt_thread_startup(rk_mbox->irq_thread);

    chan = &rk_mbox->chans[0];

    for (int i = 0; i < soc_data->num_chans; ++i, ++chan)
    {
        int irq = rt_dm_dev_get_irq(dev, i);

        if (irq < 0)
        {
            err = irq;

            goto _fail;
        }

        rt_snprintf(chan_name, sizeof(chan_name), "rk_mbox-%d", i);

        rt_hw_interrupt_install(irq, rockchip_mbox_isr, rk_mbox, chan_name);
        rt_hw_interrupt_umask(irq);

        chan->idx = i;
        chan->irq = irq;
        chan->rk_mbox = rk_mbox;
    }

    rk_mbox->buf_size = io_size / (soc_data->num_chans * 2);

    dev->user_data = rk_mbox;

    rk_mbox->parent.dev = dev;
    rk_mbox->parent.num_chans = soc_data->num_chans;
    rk_mbox->parent.ops = &rk_mbox_ops;

    if ((err = rt_mbox_controller_register(&rk_mbox->parent)))
    {
        goto _fail;
    }

    return RT_EOK;

_fail:
    rockchip_mbox_free_resource(rk_mbox);

    return err;
}

static rt_err_t rockchip_mbox_remove(struct rt_platform_device *pdev)
{
    struct rockchip_mbox_chan *chan;
    struct rockchip_mbox *rk_mbox = pdev->parent.user_data;

    chan = &rk_mbox->chans[0];

    for (int i = 0; i < rk_mbox->parent.num_chans; ++i, ++chan)
    {
        rt_hw_interrupt_mask(chan->irq);
        rt_pic_detach_irq(chan->irq, rk_mbox);
    }

    rt_mbox_controller_unregister(&rk_mbox->parent);

    rockchip_mbox_free_resource(rk_mbox);

    return RT_EOK;
}

static const struct rockchip_mbox_soc_data rk3368_data =
{
    .num_chans = 4,
};

static const struct rt_ofw_node_id rockchip_mbox_ofw_ids[] =
{
    { .compatible = "rockchip,rk3368-mailbox", .data = &rk3368_data },
    { /* sentinel */ }
};

static struct rt_platform_driver rockchip_mbox_driver =
{
    .name = "mailbox-rockchip",
    .ids = rockchip_mbox_ofw_ids,

    .probe = rockchip_mbox_probe,
    .remove = rockchip_mbox_remove,
};

static int rockchip_mbox_drv_register(void)
{
    rt_platform_driver_register(&rockchip_mbox_driver);

    return 0;
}
INIT_FRAMEWORK_EXPORT(rockchip_mbox_drv_register);
