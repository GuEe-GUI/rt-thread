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

#define DBG_TAG "iommu.smmu-v3"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

struct arm_smmu_v3_stream_table_l1_desc
{
    rt_uint8_t span;
    rt_le64_t *l2_base;
    rt_uint64_t l2_base_phy;
};

struct arm_smmu_v3_stream_table
{
    rt_le64_t *strtab;
    rt_uint64_t strtab_phy;

    struct arm_smmu_v3_stream_table_l1_desc *l1_desc;
    rt_size_t num_l1_ents;

    rt_uint64_t strtab_base;
    rt_uint32_t strtab_base_cfg;
};

struct arm_smmuv3
{
    struct rt_iommu_device parent;

    union
    {
        void *base;
        void *page0;
    };
    void *page1;

#define ARM_SMMU_FEAT_2_LVL_STRTAB      RT_BIT(0)
#define ARM_SMMU_FEAT_2_LVL_CDTAB       RT_BIT(1)
#define ARM_SMMU_FEAT_TT_LE             RT_BIT(2)
#define ARM_SMMU_FEAT_TT_BE             RT_BIT(3)
#define ARM_SMMU_FEAT_PRI               RT_BIT(4)
#define ARM_SMMU_FEAT_ATS               RT_BIT(5)
#define ARM_SMMU_FEAT_SEV               RT_BIT(6)
#define ARM_SMMU_FEAT_MSI               RT_BIT(7)
#define ARM_SMMU_FEAT_COHERENCY         RT_BIT(8)
#define ARM_SMMU_FEAT_TRANS_S1          RT_BIT(9)
#define ARM_SMMU_FEAT_TRANS_S2          RT_BIT(10)
#define ARM_SMMU_FEAT_STALLS            RT_BIT(11)
#define ARM_SMMU_FEAT_HYP               RT_BIT(12)
#define ARM_SMMU_FEAT_STALL_FORCE       RT_BIT(13)
#define ARM_SMMU_FEAT_VAX               RT_BIT(14)
#define ARM_SMMU_FEAT_RANGE_INV         RT_BIT(15)
#define ARM_SMMU_FEAT_BTM               RT_BIT(16)
#define ARM_SMMU_FEAT_SVA               RT_BIT(17)
#define ARM_SMMU_FEAT_E2H               RT_BIT(18)
#define ARM_SMMU_FEAT_NESTING           RT_BIT(19)
#define ARM_SMMU_FEAT_ATTR_TYPES_OVR    RT_BIT(20)
    rt_uint32_t features;

#define ARM_SMMU_OPT_SKIP_PREFETCH      RT_BIT(0)
#define ARM_SMMU_OPT_PAGE0_REGS_ONLY    RT_BIT(1)
#define ARM_SMMU_OPT_MSIPOLL            RT_BIT(2)
#define ARM_SMMU_OPT_CMDQ_FORCE_SYNC    RT_BIT(2)
    rt_uint32_t options;

#define ARM_SMMU_EVENTQ_IRQ             0
#define ARM_SMMU_PRIQ_IRQ               1
#define ARM_SMMU_CMDQ_SYNC_IRQ          2
#define ARM_SMMU_GERROR_IRQ             3
    int irqs[4];

#define ARM_SMMU_MAX_ASIDS  (1 << 16)
    rt_uint32_t asid_bits;

    rt_uint32_t ssid_bits;
    rt_uint32_t sid_bits;

    struct arm_smmu_v3_stream_table stream_table;
};
#define raw_to_arm_smmuv3(raw) rt_container_of(raw, struct arm_smmuv3, parent)

static rt_err_t arm_smmuv3_attach(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain)
{

}

static rt_err_t arm_smmuv3_detach(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain)
{

}

static rt_err_t arm_smmuv3_map(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain,
        void *iopa, void *iova, rt_size_t size, mm_flag_t flags)
{

}

static rt_err_t arm_smmuv3_unmap(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain,
        void *iova, rt_size_t size)
{

}

static rt_err_t arm_smmuv3_iotlb_flush(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain)
{

}

static rt_err_t arm_smmuv3_iotlb_sync(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain,
        void *iova, rt_size_t size)
{

}

static int arm_smmuv3_ofw_parse(struct rt_iommu_device *iommu, struct rt_iommu_domain *domain,
        struct rt_ofw_cell_args *iommu_args)
{

}

static struct rt_iommu_ops arm_smmuv3_ops =
{
    .attach = arm_smmuv3_attach,
    .detach = arm_smmuv3_detach,
    .map = arm_smmuv3_map,
    .unmap = arm_smmuv3_unmap,
    .iotlb_flush = arm_smmuv3_iotlb_flush,
    .iotlb_sync = arm_smmuv3_iotlb_sync,
    .ofw_parse = arm_smmuv3_ofw_parse,
};

static rt_err_t arm_smmuv3_probe(struct rt_platform_device *pdev)
{
    rt_err_t err;
    struct rt_device *dev = &pdev->parent;
    struct arm_smmuv3 *smmu = rt_calloc(1, sizeof(*smmu));

    if (!smmu)
    {
        return -RT_ENOMEM;
    }

    smmu->parent.dev = dev;
    smmu->parent.ops = &arm_smmuv3_ops;
    smmu->parent.max_pasids = 1;

    if ((err = rt_iommu_device_register(&smmu->parent)))
    {
        goto _fail;
    }

    return RT_EOK;

_fail:
    rt_free(smmu);

    return err;
}

static const struct rt_ofw_node_id arm_smmuv3_ofw_ids[] =
{
    { .compatible = "arm,smmu-v3" },
    { /* sentinel */ }
};

static struct rt_platform_driver arm_smmuv3_driver =
{
    .name = "arm-smmu-v3",
    .ids = arm_smmuv3_ofw_ids,

    .probe = arm_smmuv3_probe,
};

static int arm_smmuv3_drv_register(void)
{
    rt_platform_driver_register(&arm_smmuv3_driver);

    return 0;
}
INIT_SUBSYS_EXPORT(arm_smmuv3_drv_register);

// -trace enable=smmu_add_mr \
// -trace enable=smmu_ptw_level \
// -trace enable=smmu_ptw_invalid_pte \
// -trace enable=smmu_ptw_page_pte \
// -trace enable=smmu_ptw_block_pte \
// -trace enable=smmu_get_pte \
// -trace enable=smmu_iotlb_inv_all \
// -trace enable=smmu_iotlb_inv_asid \
// -trace enable=smmu_iotlb_inv_vmid \
// -trace enable=smmu_iotlb_inv_iova \
// -trace enable=smmu_inv_notifiers_mr \
// -trace enable=smmu_iotlb_lookup_hit \
// -trace enable=smmu_iotlb_lookup_miss \
// -trace enable=smmu_iotlb_insert \
// -trace enable=smmuv3_read_mmio \
// -trace enable=smmuv3_trigger_irq \
// -trace enable=smmuv3_write_gerror \
// -trace enable=smmuv3_write_gerrorn \
// -trace enable=smmuv3_unhandled_cmd \
// -trace enable=smmuv3_cmdq_consume \
// -trace enable=smmuv3_cmdq_opcode \
// -trace enable=smmuv3_cmdq_consume_out \
// -trace enable=smmuv3_cmdq_consume_error \
// -trace enable=smmuv3_write_mmio \
// -trace enable=smmuv3_record_event \
// -trace enable=smmuv3_find_ste \
// -trace enable=smmuv3_find_ste_2lvl \
// -trace enable=smmuv3_get_ste \
// -trace enable=smmuv3_translate_disable \
// -trace enable=smmuv3_translate_bypass \
// -trace enable=smmuv3_translate_abort \
// -trace enable=smmuv3_translate_success \
// -trace enable=smmuv3_get_cd \
// -trace enable=smmuv3_decode_cd \
// -trace enable=smmuv3_decode_cd_tt \
// -trace enable=smmuv3_cmdq_cfgi_ste \
// -trace enable=smmuv3_cmdq_cfgi_ste_range \
// -trace enable=smmuv3_cmdq_cfgi_cd \
// -trace enable=smmuv3_config_cache_hit \
// -trace enable=smmuv3_config_cache_miss \
// -trace enable=smmuv3_range_inval \
// -trace enable=smmuv3_cmdq_tlbi_nh \
// -trace enable=smmuv3_cmdq_tlbi_nh_asid \
// -trace enable=smmuv3_cmdq_tlbi_s12_vmid \
// -trace enable=smmuv3_config_cache_inv \
// -trace enable=smmuv3_notify_flag_add \
// -trace enable=smmuv3_notify_flag_del \
// -trace enable=smmuv3_inv_notifiers_iova
