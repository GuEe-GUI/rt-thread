/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-21     GuEe-GUI     first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "soc.rockchip.hw-decompress"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <mmu.h>

#define DECOM_CTRL              0x0
#define DECOM_ENR               0x4
#define DECOM_RADDR             0x8
#define DECOM_WADDR             0xc
#define DECOM_UDDSL             0x10
#define DECOM_UDDSH             0x14
#define DECOM_TXTHR             0x18
#define DECOM_RXTHR             0x1c
#define DECOM_SLEN              0x20
#define DECOM_STAT              0x24
#define DECOM_ISR               0x28
#define DECOM_IEN               0x2c
#define DECOM_AXI_STAT          0x30
#define DECOM_TSIZEL            0x34
#define DECOM_TSIZEH            0x38
#define DECOM_MGNUM             0x3c
#define DECOM_FRAME             0x40
#define DECOM_DICTID            0x44
#define DECOM_CSL               0x48
#define DECOM_CSH               0x4c
#define DECOM_LMTSL             0x50
#define DECOM_LMTSH             0x54

#define LZ4_HEAD_CSUM_CHECK_EN  RT_BIT(1)
#define LZ4_BLOCK_CSUM_CHECK_EN RT_BIT(2)
#define LZ4_CONT_CSUM_CHECK_EN  RT_BIT(3)

#define DSOLIEN                 RT_BIT(19)
#define ZDICTEIEN               RT_BIT(18)
#define GCMEIEN                 RT_BIT(17)
#define GIDEIEN                 RT_BIT(16)
#define CCCEIEN                 RT_BIT(15)
#define BCCEIEN                 RT_BIT(14)
#define HCCEIEN                 RT_BIT(13)
#define CSEIEN                  RT_BIT(12)
#define DICTEIEN                RT_BIT(11)
#define VNEIEN                  RT_BIT(10)
#define WNEIEN                  RT_BIT(9)
#define RDCEIEN                 RT_BIT(8)
#define WRCEIEN                 RT_BIT(7)
#define DISEIEN                 RT_BIT(6)
#define LENEIEN                 RT_BIT(5)
#define LITEIEN                 RT_BIT(4)
#define SQMEIEN                 RT_BIT(3)
#define SLCIEN                  RT_BIT(2)
#define HDEIEN                  RT_BIT(1)
#define DSIEN                   RT_BIT(0)

#define DECOM_STOP              RT_BIT(0)
#define DECOM_COMPLETE          RT_BIT(0)
#define DECOM_GZIP_MODE         RT_BIT(4)
#define DECOM_ZLIB_MODE         RT_BIT(5)
#define DECOM_DEFLATE_MODE      RT_BIT(0)

#define DECOM_ENABLE            0x1
#define DECOM_DISABLE           0x0

#define DECOM_INT_MASK \
( \
    DSOLIEN | ZDICTEIEN | GCMEIEN | GIDEIEN | CCCEIEN | BCCEIEN | HCCEIEN | \
    CSEIEN | DICTEIEN | VNEIEN | WNEIEN | RDCEIEN | WRCEIEN | DISEIEN | \
    LENEIEN | LITEIEN | SQMEIEN | SLCIEN | HDEIEN | DSIEN \
)

enum rockchip_decompress_mod
{
    LZ4_MOD,
    GZIP_MOD,
    ZLIB_MOD,

    UNKNOW_MOD,
};

struct wait_event_queue
{
    rt_list_t list;

    const char *args;

    enum rockchip_decompress_mod mod;

    rt_ubase_t src;
    rt_ubase_t dst;
    rt_size_t dst_max_size;

    struct rt_thread *task;
    struct rt_completion done;
};

struct rockchip_decompress
{
    struct rt_device parent;

    int irq;
    void *regs;

    rt_ubase_t mem_start;
    rt_size_t mem_size;

    struct rt_clk_array *clk_arr;
    struct rt_reset_control *rstc;

    rt_list_t work_nodes;
    rt_list_t request_nodes;
    struct wait_event_queue *current;

    struct rt_mutex lock;
    struct rt_spinlock spinlock;
};

#define raw_to_rockchip_decompress(raw) rt_container_of(raw, struct rockchip_decompress, parent)

rt_inline rt_uint32_t rockchip_decompress_readl(struct rockchip_decompress *rk_decom, int offset)
{
    return HWREG32(rk_decom->regs + offset);
}

rt_inline void rockchip_decompress_writel(struct rockchip_decompress *rk_decom, int offset, rt_uint32_t value)
{
    HWREG32(rk_decom->regs + offset) = value;
}

static rt_err_t rockchip_decompress_submit(struct rockchip_decompress *rk_decom,
        struct wait_event_queue *weq)
{
    rt_uint32_t irq_status, decom_enr, mod_flags;

    rt_uint32_t mod = weq->mod;
    rt_ubase_t src = weq->src, dst = weq->dst;
    rt_size_t dst_max_size = weq->dst_max_size;

    if (rt_unlikely(mod >= UNKNOW_MOD))
    {
        return -RT_EINVAL;
    }

    LOG_D("Task: %s, Mode: %s, Src: %p, Dst: %p, MaxSize: %u",
            weq->task->parent.name, mod[((const char *const []) {
                [LZ4_MOD] = "LZ4",
                [GZIP_MOD] = "GZIP",
                [ZLIB_MOD] = "ZLIB",
            })], src, dst, dst_max_size);

    decom_enr = rockchip_decompress_readl(rk_decom, DECOM_ENR);
    if (decom_enr & 0x1)
    {
        LOG_E("Decompress busy");
        return -RT_EBUSY;
    }

    if (rk_decom->rstc)
    {
        rt_reset_control_assert(rk_decom->rstc);
        rt_hw_us_delay(10);
        rt_reset_control_deassert(rk_decom->rstc);
    }

    irq_status = rockchip_decompress_readl(rk_decom, DECOM_ISR);

    /* clear interrupts */
    if (irq_status)
    {
        rockchip_decompress_writel(rk_decom, DECOM_ISR, irq_status);
    }

    switch (mod)
    {
    case LZ4_MOD:
        mod_flags = LZ4_CONT_CSUM_CHECK_EN | LZ4_HEAD_CSUM_CHECK_EN |
                LZ4_BLOCK_CSUM_CHECK_EN | LZ4_MOD;
        break;

    case GZIP_MOD:
        mod_flags = DECOM_DEFLATE_MODE | DECOM_GZIP_MODE;
        break;

    case ZLIB_MOD:
        mod_flags = DECOM_DEFLATE_MODE | DECOM_ZLIB_MODE;
        break;
    }
    rockchip_decompress_writel(rk_decom, DECOM_CTRL, mod_flags);

    rockchip_decompress_writel(rk_decom, DECOM_RADDR, src);
    rockchip_decompress_writel(rk_decom, DECOM_WADDR, dst);

    rockchip_decompress_writel(rk_decom, DECOM_LMTSL, dst_max_size);
    rockchip_decompress_writel(rk_decom, DECOM_LMTSH, 0x0);

    rockchip_decompress_writel(rk_decom, DECOM_IEN, DECOM_INT_MASK);
    rockchip_decompress_writel(rk_decom, DECOM_ENR, DECOM_ENABLE);

    rk_decom->current = weq;

    return RT_EOK;
}

static rt_err_t rockchip_decompress_control(rt_device_t dev, int, void *args)
{
    rt_err_t err = RT_EOK;
    rt_bool_t can_submit;
    struct rt_thread *task;
    struct wait_event_queue *weq = RT_NULL, *weq_tmp, *weq_next;
    struct rockchip_decompress *rk_decom = raw_to_rockchip_decompress(dev);

    if (rt_unlikely(!args))
    {
        return -RT_EINVAL;
    }

    task = rt_thread_self();

    rt_mutex_take(&rk_decom->lock, RT_WAITING_FOREVER);

    rt_list_for_each_entry_safe(weq_tmp, weq_next, &rk_decom->request_nodes, list)
    {
        if (weq_tmp->task->stat == RT_THREAD_CLOSE)
        {
            /* ignore this task */
            LOG_D("Closed Task: %s", weq_tmp->task->parent.name);

            rt_list_remove(&weq_tmp->list);
            rt_free(weq_tmp);
            continue;
        }

        if (weq_tmp->task == task)
        {
            weq = weq_tmp;
            break;
        }
    }

    if (!weq)
    {
        weq = rt_malloc(sizeof(*weq));

        if (rt_unlikely(weq))
        {
            err = -RT_ENOMEM;
            goto _out_mutex;
        }

        rt_list_init(&weq->list);
        rt_list_insert_before(&rk_decom->request_nodes, &weq->list);

        weq->args = RT_NULL;
        weq->mod = UNKNOW_MOD;

        weq->src = ~0UL;
        weq->dst = ~0UL;
        weq->dst_max_size = ~0UL;

        weq->task = task;
        rt_completion_init(&weq->done);
    }

    if (!weq->args)
    {
        weq->args = args;
    }
    else
    {
        const char *cmd = weq->args;

        if (!rt_strcmp(cmd, "mode"))
        {
            if (rt_unlikely((rt_ubase_t)args >= UNKNOW_MOD))
            {
                err = -RT_ENOSYS;
            }
        }
        else if (!rt_strcmp(cmd, "src"))
        {
            void *src = rt_kmem_v2p(args);

            if (rt_unlikely(src == ARCH_MAP_FAILED))
            {
                err = -RT_EINVAL;
            }
            else
            {
                weq->src = (rt_ubase_t)src;
            }
        }
        else if (!rt_strcmp(cmd, "dst"))
        {
            void *dst = rt_kmem_v2p(args);

            if (rt_unlikely(dst == ARCH_MAP_FAILED))
            {
                err = -RT_EINVAL;
            }
            else
            {
                weq->dst = (rt_ubase_t)dst;
            }
        }
        else if (!rt_strcmp(cmd, "size"))
        {
            rt_size_t size = (rt_ubase_t)args;

            if (rt_unlikely(!size))
            {
                err = -RT_EINVAL;
            }
            else
            {
                weq->dst_max_size = size;
            }
        }

        /* ready to next args */
        weq->args = RT_NULL;
    }

    if (!err)
    {
        can_submit = weq->mod != UNKNOW_MOD &&
                weq->src != ~0UL &&
                weq->dst != ~0UL &&
                weq->dst_max_size != ~0UL;

        if (can_submit)
        {
            /* remove from request */
            rt_list_remove(&weq->list);
        }
    }

_out_mutex:
    rt_mutex_release(&rk_decom->lock);

    if (rt_unlikely(err))
    {
        return err;
    }

    if (can_submit)
    {
        rt_ubase_t level = rt_spin_lock_irqsave(&rk_decom->spinlock);

        /* submit work */
        rt_list_insert_before(&rk_decom->work_nodes, &weq->list);

        /* no working */
        if (!rk_decom->current)
        {
            rockchip_decompress_submit(rk_decom, weq);
        }

        rt_spin_unlock_irqrestore(&rk_decom->spinlock, level);

        err = rt_completion_wait(&weq->done, RT_WAITING_FOREVER);
        rt_free(weq);
    }

    return err;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops rockchip_decompress_ops =
{
    .control = rockchip_decompress_control,
};
#endif

static void rockchip_decompress_isr(int irqno, void *param)
{
    rt_uint32_t irq_status, decom_status;
    struct rockchip_decompress *rk_decom = param;

    irq_status = rockchip_decompress_readl(rk_decom, DECOM_ISR);
    /* clear interrupts */
    rockchip_decompress_writel(rk_decom, DECOM_ISR, irq_status);

    if (irq_status & DECOM_STOP)
    {
        rt_ubase_t level;
        struct wait_event_queue *weq = rk_decom->current, *next;

        decom_status = rockchip_decompress_readl(rk_decom, DECOM_STAT);

        if (!(decom_status & DECOM_COMPLETE))
        {
            LOG_E("Decompress failed: irq_status = 0x%x, decom_status = 0x%x",
                    irq_status, decom_status);

            weq->task->error = -RT_EIO;
        }

        level = rt_spin_lock_irqsave(&rk_decom->spinlock);

        /* finish prev work */
        rt_list_remove(&weq->list);

        /* next work */
        if (rt_list_isempty(&rk_decom->work_nodes))
        {
            rk_decom->current = RT_NULL;
        }
        else
        {
            next = rt_list_first_entry(&rk_decom->work_nodes, typeof(*next), list);

            rockchip_decompress_submit(rk_decom, next);
        }

        rt_spin_unlock_irqrestore(&rk_decom->spinlock, level);

        /* ask prev task done */
        rt_completion_done(&weq->done);
    }
}

static void rockchip_decompress_free(struct rockchip_decompress *rk_decom)
{
    if (rk_decom->regs)
    {
        rt_iounmap(rk_decom->regs);
    }

    if (!rt_is_err_or_null(rk_decom->clk_arr))
    {
        rt_clk_array_disable_unprepare(rk_decom->clk_arr);
        rt_clk_array_put(rk_decom->clk_arr);
    }

    if (!rt_is_err_or_null(rk_decom->rstc))
    {
        rt_reset_control_put(rk_decom->rstc);
    }

    rt_free(rk_decom);
}

static rt_err_t rockchip_decompress_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    rt_uint64_t addr, size;
    const char *dev_name;
    struct rt_device *dev = &pdev->parent;
    struct rockchip_decompress *rk_decom = rt_calloc(1, sizeof(*rk_decom));

    if (!rk_decom)
    {
        return -RT_ENOMEM;
    }

    if ((err = rt_dm_dev_get_address(dev, 0, &addr, &size)))
    {
        goto _fail;
    }

    rk_decom->regs = rt_dm_dev_iomap(dev, 0);

    if (!rk_decom->regs)
    {
        err = -RT_EIO;
        goto _fail;
    }

    rk_decom->clk_arr = rt_clk_get_array(dev);

    if (rt_is_err(rk_decom->clk_arr))
    {
        err = rt_ptr_err(rk_decom->clk_arr);
        goto _fail;
    }

    if ((err = rt_clk_array_prepare_enable(rk_decom->clk_arr)))
    {
        goto _fail;
    }

    rk_decom->rstc = rt_reset_control_get_by_name(dev, "dresetn");

    if (rt_is_err(rk_decom->rstc))
    {
        err = rt_ptr_err(rk_decom->rstc);
        goto _fail;
    }

    rk_decom->irq = rt_dm_dev_get_irq(dev, 0);

    if (rk_decom->irq < 0)
    {
        err = rk_decom->irq;
        goto _fail;
    }

    dev->user_data = rk_decom;

    rt_dm_dev_set_name_auto(&rk_decom->parent, "decom");
    dev_name = rt_dm_dev_get_name(&rk_decom->parent);

    rk_decom->parent.type = RT_Device_Class_Char;
#ifdef RT_USING_DEVICE_OPS
    rk_decom->parent.ops = &rockchip_decompress_ops;
#else
    rk_decom->parent.control = rockchip_decompress_control;
#endif

    if ((err = rt_device_register(&rk_decom->parent, dev_name, RT_DEVICE_FLAG_DEACTIVATE)))
    {
        goto _fail;
    }

    rt_list_init(&rk_decom->work_nodes);
    rt_list_init(&rk_decom->request_nodes);

    rt_mutex_init(&rk_decom->lock, dev_name, RT_IPC_FLAG_PRIO);
    rt_spin_lock_init(&rk_decom->spinlock);

    rt_hw_interrupt_install(rk_decom->irq, rockchip_decompress_isr, rk_decom, "rk-decompress");
    rt_hw_interrupt_umask(rk_decom->irq);

    return RT_EOK;

_fail:
    rockchip_decompress_free(rk_decom);

    return err;
}

static rt_err_t rockchip_decompress_remove(struct rt_platform_device *pdev)
{
    struct rockchip_decompress *rk_decom = pdev->parent.user_data;

    rt_hw_interrupt_mask(rk_decom->irq);
    rt_pic_detach_irq(rk_decom->irq, rk_decom);

    rt_device_unregister(&rk_decom->parent);

    rockchip_decompress_free(rk_decom);

    return RT_EOK;
}

static const struct rt_ofw_node_id rockchip_decompress_ofw_ids[] =
{
    { .compatible = "rockchip,hw-decompress" },
    { /* sentinel */ }
};

static struct rt_platform_driver rockchip_decompress_driver =
{
    .name = "rockchip-hwdecompress",
    .ids = rockchip_decompress_ofw_ids,

    .probe = rockchip_decompress_probe,
    .remove = rockchip_decompress_remove,
};
RT_PLATFORM_DRIVER_EXPORT(rockchip_decompress_driver);
