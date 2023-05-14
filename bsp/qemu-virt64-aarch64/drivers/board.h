/*
 * File      : board.h
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-07-06     Bernard    the first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <rtthread.h>
#include <virt.h>
#include <rtdef.h>


extern unsigned char __bss_start;
extern unsigned char __bss_end;

#define HEAP_BEGIN  (void *)&__bss_end

#ifdef RT_USING_SMART
#define HEAP_END    (rt_size_t)(KERNEL_VADDR_START + 64 * 1024 * 1024)
#define PAGE_START  HEAP_END + 1 * 1024 * 1024
#define PAGE_END    ((rt_size_t)KERNEL_VADDR_START + 128 * 1024 * 1024)
#else
#define HEAP_END    ((void *)HEAP_BEGIN + 64 * 1024 * 1024)
#define KERNEL_VADDR_START 0x40000000
#endif

#define UART_DR(base) __REG32(base + 0x00)
#define UART_FR(base) __REG32(base + 0x18)
#define UART_CR(base) __REG32(base + 0x30)
#define UART_IMSC(base) HWREG32(base + 0x38)

#define UARTFR_RXFE 0x10
#define UARTFR_TXFF 0x20

#define UARTIMSC_RXIM 0x10

#define UART_BASE_ADDR 0x09000000

#define IRQ_SPI_OFFSET 32
#define IRQ_PPI_OFFSET 16

void rt_hw_board_init(void);

int rt_hw_uart_init(void);

#endif
