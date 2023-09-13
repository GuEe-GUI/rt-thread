/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-02     Jesven       the first version
 */

#include <rtthread.h>
#include <backtrace.h>

#define BT_NESTING_MAX 100

static int unwind_frame(struct bt_frame *frame)
{
    unsigned long fp = frame->fp;

    if ((fp & 0x7)
#ifdef RT_USING_LWP
         || fp < KERNEL_VADDR_START
#endif
            )
    {
        return 1;
    }
    frame->fp = *(unsigned long *)fp;
    frame->pc = *(unsigned long *)(fp + 8);
    return 0;
}

static void walk_unwind(unsigned long pc, unsigned long fp)
{
    struct bt_frame frame;
    unsigned long lr = pc;
    int nesting = 0;

    frame.fp = fp;
    while (nesting < BT_NESTING_MAX)
    {
        rt_kprintf(" %p", (void *)lr);
        if (unwind_frame(&frame))
        {
            break;
        }
        lr = frame.pc;
        nesting++;
    }
}

void backtrace(unsigned long pc, unsigned long lr, unsigned long fp)
{
    rt_kprintf("please use: addr2line -e rtthread.elf -a -f %p", (void *)pc);
    walk_unwind(lr, fp);
    rt_kprintf("\n");
}

int rt_backtrace_skipn(int level)
{
    unsigned long lr;
    unsigned long fp = (unsigned long)__builtin_frame_address(0U);

    /* skip current frames */
    struct bt_frame frame;
    frame.fp = fp;

    /* skip n frames */
    do
    {
        if (unwind_frame(&frame))
            return -RT_ERROR;
        lr = frame.pc;

        /* INFO: level is signed integer */
    } while (level-- > 0);

    backtrace(0, lr, frame.fp);
    return 0;
}

int rt_backtrace(void)
{
    /* skip rt_backtrace itself */
    rt_backtrace_skipn(1);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(rt_backtrace, bt_test, backtrace test);