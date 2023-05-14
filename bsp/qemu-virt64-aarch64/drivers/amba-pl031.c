#include <rtthread.h>
#include <rtdevice.h>
#include <sys/time.h>
#include <board.h>

#include "drivers/amba_bus.h"

#define RTC_DR      0x00    /* data read register */
#define RTC_MR      0x04    /* match register */
#define RTC_LR      0x08    /* data load register */
#define RTC_CR      0x0c    /* control register */
#define RTC_IMSC    0x10    /* interrupt mask and set register */
#define RTC_RIS     0x14    /* raw interrupt status register */
#define RTC_MIS     0x18    /* masked interrupt status register */
#define RTC_ICR     0x1c    /* interrupt clear register */

#define RTC_CR_OPEN     1
#define RTC_CR_CLOSE    0

struct hw_rtc_device
{
    struct rt_device device;
    rt_ubase_t hw_base;
};

static struct hw_rtc_device rtc_device;

rt_inline rt_uint32_t pl031_read32(rt_ubase_t offset)
{
    return (*((volatile unsigned int *)(rtc_device.hw_base + offset)));
}

rt_inline void pl031_write32(rt_ubase_t offset, rt_uint32_t value)
{
    (*((volatile unsigned int *)(rtc_device.hw_base + offset))) = value;
}

static rt_err_t pl031_rtc_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t pl031_rtc_open(rt_device_t dev, rt_uint16_t oflag)
{
    pl031_write32(RTC_CR, RTC_CR_OPEN);
    return RT_EOK;
}

static rt_err_t pl031_rtc_close(rt_device_t dev)
{
    pl031_write32(RTC_CR, RTC_CR_CLOSE);
    return RT_EOK;
}

static rt_err_t pl031_rtc_control(rt_device_t dev, int cmd, void *args)
{

    RT_ASSERT(dev != RT_NULL);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:
        *(rt_uint32_t *)args = pl031_read32(RTC_DR);
        break;
    case RT_DEVICE_CTRL_RTC_SET_TIME:
        pl031_write32(RTC_LR, *(time_t *)args);
        break;
    default:
        return -RT_EINVAL;
    }
    return RT_EOK;
}

static rt_ssize_t pl031_rtc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    pl031_rtc_control(dev, RT_DEVICE_CTRL_RTC_GET_TIME, buffer);
    return size;
}

static rt_ssize_t pl031_rtc_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    pl031_rtc_control(dev, RT_DEVICE_CTRL_RTC_SET_TIME, (void *)buffer);
    return size;
}

static struct rt_device_ops pl031_ops =
{
    .init = pl031_rtc_init,
    .open = pl031_rtc_open,
    .close = pl031_rtc_close,
    .read = pl031_rtc_read,
    .write = pl031_rtc_write,
    .control = pl031_rtc_control
};

int pl031_probe(struct rt_device *dev)
{
    size_t reg_range = 0;
    void *reg_addr = RT_NULL;

    struct dtb_node *node = (struct dtb_node *)(dev->dtb_node);

    if (node)
    {
        reg_addr = (void *)dtb_node_get_addr_size(node, "reg", &reg_range);
        if ((reg_addr) && (reg_range != 0))
        {
            rtc_device.hw_base = (rt_base_t)rt_ioremap(reg_addr, reg_range);
        }

        rt_memset(&rtc_device, 0, sizeof(rtc_device));
        struct rt_device_ops *ops = ((struct rt_rtc_driver *)(dev->drv))->ops;

        rtc_device.device.type        = RT_Device_Class_RTC;
        rtc_device.device.rx_indicate = RT_NULL;
        rtc_device.device.tx_complete = RT_NULL;
        rtc_device.device.ops         = ops;
        rtc_device.device.user_data   = RT_NULL;

        /* register a rtc device */
        rt_device_register(&rtc_device.device, dev->drv->name, RT_DEVICE_FLAG_RDWR);
    }

    return 0;
}

struct rt_device_id pl031_ids[] =
    {
        {.compatible = "arm,pl031"},
        {/* sentinel */}};

struct rt_rtc_driver pl031_drv = {
    .parent = {
        .name = "pl031_rtc",
        .probe = pl031_probe,
        .ids = pl031_ids,
    },
    .ops = &pl031_ops,
};

AMBA_DRIVER_EXPORT(pl031_drv);

