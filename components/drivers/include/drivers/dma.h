/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     the first version
 */

#ifndef __DMA_H__
#define __DMA_H__

#include <rtthread.h>

#include <mmu.h>
#include <mm_page.h>

struct rt_dma_chan;
struct rt_dma_controller_ops;

struct rt_dma_controller
{
    rt_list_t list;

    struct rt_device *dev;

    const struct rt_dma_controller_ops *ops;
};

enum rt_dma_transfer_direction
{
    RT_DMA_MEM_TO_MEM,
    RT_DMA_MEM_TO_DEV,
    RT_DMA_DEV_TO_MEM,
    RT_DMA_DEV_TO_DEV,
    RT_DMA_TRANS_NONE,
};

enum rt_dma_slave_buswidth
{
    RT_DMA_SLAVE_BUSWIDTH_UNDEFINED = 0,
    RT_DMA_SLAVE_BUSWIDTH_1_BYTE = 1,
    RT_DMA_SLAVE_BUSWIDTH_2_BYTES = 2,
    RT_DMA_SLAVE_BUSWIDTH_3_BYTES = 3,
    RT_DMA_SLAVE_BUSWIDTH_4_BYTES = 4,
    RT_DMA_SLAVE_BUSWIDTH_8_BYTES = 8,
    RT_DMA_SLAVE_BUSWIDTH_16_BYTES = 16,
    RT_DMA_SLAVE_BUSWIDTH_32_BYTES = 32,
    RT_DMA_SLAVE_BUSWIDTH_64_BYTES = 64,
    RT_DMA_SLAVE_BUSWIDTH_128_BYTES = 128,
};

struct rt_dma_slave_config
{
    enum rt_dma_transfer_direction direction;
    enum rt_dma_slave_buswidth src_addr_width;
    enum rt_dma_slave_buswidth dst_addr_width;

    rt_ubase_t src_addr;
    rt_ubase_t dst_addr;

    rt_uint32_t src_maxburst;
    rt_uint32_t dst_maxburst;
    rt_uint32_t src_port_window_size;
    rt_uint32_t dst_port_window_size;
};

#define RT_DMA_CHAN_F_NONE      RT_BIT(0)

struct rt_dma_controller_ops
{
    struct rt_dma_chan *(*request_channel)(struct rt_dma_controller *, struct rt_device *slave);
    rt_err_t (*release_channel)(struct rt_dma_chan *);

    rt_err_t (*start)(struct rt_dma_chan *);
    rt_err_t (*stop)(struct rt_dma_chan *);
    rt_err_t (*config)(struct rt_dma_chan *, struct rt_dma_slave_config *);

    rt_err_t (*prep_dma_memcpy)(struct rt_dma_chan *,
            rt_ubase_t dst, rt_ubase_t src, rt_size_t len, rt_ubase_t flags);

    rt_err_t (*prep_dma_cyclic)(struct rt_dma_chan *,
            rt_ubase_t buf_addr, rt_size_t buf_len, rt_size_t period_len,
            enum rt_dma_transfer_direction, rt_ubase_t flags);

    rt_err_t (*prep_dma_single)(struct rt_dma_chan *,
            rt_ubase_t buf_addr, rt_size_t buf_len,
            enum rt_dma_transfer_direction, rt_ubase_t flags);
};

struct rt_dma_chan
{
    struct rt_dma_controller *ctrl;
    struct rt_device *slave;

    rt_list_t list;
    void (*callback)(struct rt_dma_chan *);

    void *priv;
};

rt_err_t rt_dma_controller_register(struct rt_dma_controller *ctrl);
rt_err_t rt_dma_controller_unregister(struct rt_dma_controller *ctrl);

#define RT_DMA_F_LINEAR     RT_BIT(0)
#define RT_DMA_F_32BITS     RT_BIT(1)
#define RT_DMA_F_NOCACHE    RT_BIT(2)

#define RT_DMA_PAGE_SIZE    ARCH_PAGE_SIZE

void *rt_dma_alloc(struct rt_device *dev, rt_size_t size,
        rt_ubase_t *dma_handle, rt_ubase_t flags);

void rt_dma_free(struct rt_device *dev, rt_size_t size,
        void *cpu_addr, rt_ubase_t dma_handle, rt_ubase_t flags);

rt_inline void *rt_dma_alloc_coherent(struct rt_device *dev, rt_size_t size,
        rt_ubase_t *dma_handle)
{
    return rt_dma_alloc(dev, size, dma_handle,
            RT_DMA_F_NOCACHE | RT_DMA_F_LINEAR);
}

rt_inline void rt_dma_free_coherent(struct rt_device *dev, rt_size_t size,
        void *cpu_addr, rt_ubase_t dma_handle)
{
    rt_dma_free(dev, size, cpu_addr, dma_handle,
            RT_DMA_F_NOCACHE | RT_DMA_F_LINEAR);
}

void rt_dma_flush(struct rt_device *dev, void *dma_va, rt_size_t size);

rt_err_t rt_dma_pool_install(rt_region_t *region);
rt_err_t rt_dma_pool_extract(rt_region_t *region_list, rt_size_t list_len,
        rt_size_t cma_size, rt_size_t coherent_pool_size);

#endif /* __DMA_H__ */
