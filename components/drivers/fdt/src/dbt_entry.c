#include <rtdef.h>
#include <drivers/core/rtdm.h>

#ifdef RT_USING_FDT
#include "dtb_node.h"
#endif

// TODO: FDT_START_ADDR should be configured by menuconfig, and define in rtconfig.h
#define FDT_START_ADDR 0x44000000

int dbt_init(void)
{
    void *fdt_start = (void *)FDT_START_ADDR;
    device_tree_setup(fdt_start);
    return 0;
}

INIT_EARLY_EXPORT(dbt_init);
