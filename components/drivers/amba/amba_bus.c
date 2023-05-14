#include "drivers/amba_bus.h"
#include "drivers/core/bus.h"
#include "drivers/core/platform.h"

static struct rt_bus amba_bus =
{
    .name = "amba",
    .match = rt_platform_match,
};

/**
 *  @brief This function create a amba device
 *
 *  @param node the device node from dtb
 *
 *  @return a new amba device
 */
rt_amba_dev_t rt_amba_device_create(struct dtb_node *node)
{
    RT_ASSERT(node != RT_NULL);

    struct rt_amba_device *dev;
    dev = (rt_amba_dev_t)rt_malloc(sizeof(struct rt_amba_device));
    dev->dev.dtb_node = node;
    dev->name = node->name;

    return dev;
}

rt_err_t rt_amba_driver_register(void *drv)
{
    return rt_driver_register((struct rt_driver *)drv);
}

int amba_bus_init(void)
{
    rt_bus_register(&amba_bus);
    return 0;
}

INIT_BUS_EXPORT(amba_bus_init);
