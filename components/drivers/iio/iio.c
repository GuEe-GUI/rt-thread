/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-3-08      GuEe-GUI     the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#ifdef RT_USING_OFW
static void *ofw_iio_channel_get_by_index(struct rt_ofw_node *np, int index, int *out_channel)
{
    void *iio = RT_NULL;
    struct rt_ofw_cell_args iio_args;

    if (!rt_ofw_parse_phandle_cells(np, "io-channels", "#io-channel-cells", index, &iio_args))
    {
        iio = rt_ofw_data(iio_args.data);

        rt_ofw_node_put(iio_args.data);

        if (out_channel)
        {
            *out_channel = iio_args.args[0];
        }
    }

    return iio;
}
#else
static void *ofw_iio_channel_get_by_index(struct rt_ofw_node *np, int index, int *out_channel)
{
    return RT_NULL;
}
#endif /* RT_USING_OFW */

void *rt_iio_channel_get_by_index(struct rt_device *dev, int index, int *out_channel)
{
    void *iio = RT_NULL;

    if (!dev || index < 0)
    {
        return RT_NULL;
    }

    if (dev->ofw_node)
    {
        iio = ofw_iio_channel_get_by_index(dev->ofw_node, index, out_channel);
    }

    return iio;
}

void *rt_iio_channel_get_by_name(struct rt_device *dev, const char *name, int *out_channel)
{
    int index;

    if (!dev || !name)
    {
        return RT_NULL;
    }

    index = rt_dm_dev_prop_index_of_string(dev, "io-channel-names", name);

    return rt_iio_channel_get_by_index(dev, index, out_channel);
}
