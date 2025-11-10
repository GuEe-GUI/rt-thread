/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     the first version
 */

#ifndef __UFS_H__
#define __UFS_H__

#include <rthw.h>
#include <rtthread.h>
#include <drivers/blk.h>
#include <drivers/misc.h>
#include <drivers/scsi.h>
#include <drivers/byteorder.h>

#define __BIT_FIELD(v, h, l)            ((v & RT_GENMASK(h, l)) >> l)

/* Host Capabilities */
#define RT_UFS_REG_CAP                  0x00 /* Host Controller Capabiities */
#define   RT_UFS_REG_CAP_NUTRS(v)       __BIT_FIELD(v, 4, 0) /* Number of UTP Transfer Request Slots */
#define   RT_UFS_REG_CAP_NORTT(v)       __BIT_FIELD(v, 15, 8) /* Number of outstanding READY TO TRANSFER requests supported */
#define   RT_UFS_REG_CAP_NUTMRS(v)      __BIT_FIELD(v, 18, 16) /* Number of UTP Task Management Request Slots */
#define   RT_UFS_REG_CAP_AUTOH8         RT_BIT(23) /* Auto-Hibernation Support */
#define   RT_UFS_REG_CAP_64AS           RT_BIT(24) /* 64-bit addressing supported */
#define   RT_UFS_REG_CAP_OODDS          RT_BIT(25) /* Out of order data delivery supported */
#define   RT_UFS_REG_CAP_UICDMETMS      RT_BIT(26) /* UIC DME_TEST_MODE command supported */
#define   RT_UFS_REG_CAP_UME            RT_BIT(27) /* Reserved for Unified Memory Extension */
#define RT_UFS_REG_VER                  0x08 /* UFS Version */
#define   RT_UFS_REG_VER_VS(v)          __BIT_FIELD(v, 03, 0) /* Version Suffix (VS) */
#define   RT_UFS_REG_VER_MNR(v)         __BIT_FIELD(v, 07, 4) /* Minor Version Number (MNR) */
#define   RT_UFS_REG_VER_MJR(v)         __BIT_FIELD(v, 15, 8) /* Major Version Number (MJR) */
#define RT_UFS_REG_HCPID                0x10 /* Host Controller Identification Descriptor – Product ID */
#define RT_UFS_REG_HCMID                0x14 /* Host Controller Identification Descriptor – Manufacturer ID */
#define   RT_UFS_REG_HCMID_MIC(v)       __BIT_FIELD(v, 7, 0) /* Manufacturer Identification Code */
#define   RT_UFS_REG_HCMID_BI(v)        __BIT_FIELD(v, 15, 8) /* Bank Index */
#define RT_UFS_REG_AHIT                 0x18 /* Auto-Hibernate Idle Timer */
#define   RT_UFS_REG_AHIT_AH8ITV(v)     __BIT_FIELD(v, 9, 0) /* Auto-Hibern8 Idle Timer Value */
#define   RT_UFS_REG_AHIT_TS(v)         __BIT_FIELD(v, 12, 10) /* Timer scale */
/* Operation and Runtime */
#define RT_UFS_REG_IS                   0x20  /* Interrupt Status */
#define   RT_UFS_REG_IS_UTRCS           RT_BIT(0)  /* UTP Transfer Request Completion Status */
#define   RT_UFS_REG_IS_UDEPRI          RT_BIT(1)  /* UIC DME_ENDPOINTRESET Indication */
#define   RT_UFS_REG_IS_UE              RT_BIT(2)  /* UIC Error */
#define   RT_UFS_REG_IS_UTMS            RT_BIT(3)  /* UIC Test Mode Status */
#define   RT_UFS_REG_IS_UPMS            RT_BIT(4)  /* UIC Power Mode Status */
#define   RT_UFS_REG_IS_UHXS            RT_BIT(5)  /* UIC Hibernate Exit Status */
#define   RT_UFS_REG_IS_UHES            RT_BIT(6)  /* UIC Hibernate Enter Status */
#define   RT_UFS_REG_IS_ULLS            RT_BIT(7)  /* UIC Link Lost Status */
#define   RT_UFS_REG_IS_ULSS            RT_BIT(8)  /* UIC Link Startup Status */
#define   RT_UFS_REG_IS_UTMRCS          RT_BIT(9)  /* UTP Task Management Request Completion Status */
#define   RT_UFS_REG_IS_UCCS            RT_BIT(10) /* UIC Command Completion Status */
#define   RT_UFS_REG_IS_DFES            RT_BIT(11) /* Device Fatal Error Status */
#define   RT_UFS_REG_IS_UTPES           RT_BIT(12) /* UTP Error Status */
#define   RT_UFS_REG_IS_HCFES           RT_BIT(16) /* Host Controller Fatal Error Status */
#define   RT_UFS_REG_IS_SBFES           RT_BIT(17) /* System Bus Fatal Error Status */
#define RT_UFS_REG_IE                   0x24  /* Interrupt Enable */
#define   RT_UFS_REG_IE_UTRCE           RT_BIT(0)  /* UTP Transfer Request Completion Enable */
#define   RT_UFS_REG_IE_UDEPRIE         RT_BIT(1)  /* UIC DME_ENDPOINTRESET */
#define   RT_UFS_REG_IE_UEE             RT_BIT(2)  /* UIC Error Enable */
#define   RT_UFS_REG_IE_UTMSE           RT_BIT(3)  /* UIC Test Mode Status Enable */
#define   RT_UFS_REG_IE_UPMSE           RT_BIT(4)  /* UIC Power Mode Status Enable */
#define   RT_UFS_REG_IE_UHXSE           RT_BIT(5)  /* UIC Hibernate Exit Status Enable */
#define   RT_UFS_REG_IE_UHESE           RT_BIT(6)  /* UIC Hibernate Enter Status Enable */
#define   RT_UFS_REG_IE_ULLSE           RT_BIT(7)  /* UIC Link Lost Status Enable */
#define   RT_UFS_REG_IE_ULSSE           RT_BIT(8)  /* UIC Link Startup Status Enable */
#define   RT_UFS_REG_IE_UTMRCE          RT_BIT(9)  /* UTP Task Management Request Completion Enable */
#define   RT_UFS_REG_IE_UCCE            RT_BIT(10) /* UIC COMMAND Completion Enable */
#define   RT_UFS_REG_IE_DFEE            RT_BIT(11) /* Device Fatal Error Enable */
#define   RT_UFS_REG_IE_UTPEE           RT_BIT(12) /* UTP Error Enable */
#define   RT_UFS_REG_IE_HCFEE           RT_BIT(16) /* Host Controller Fatal Error Enable */
#define   RT_UFS_REG_IE_SBFEE           RT_BIT(17) /* System Bus Fatal Error Enable */
#define RT_UFS_REG_HCS                  0x30  /* Host Controller Status */
#define   RT_UFS_REG_HCS_DP             RT_BIT(0) /* Device Present */
#define   RT_UFS_REG_HCS_UTRLRDY        RT_BIT(1) /* UTP Transfer Request List Ready */
#define   RT_UFS_REG_HCS_UTMRLRDY       RT_BIT(2) /* UTP Task Management Request List Ready */
#define   RT_UFS_REG_HCS_UCRDY          RT_BIT(3) /* UIC COMMAND Ready */
#define   RT_UFS_REG_HCS_UPMCRS(v)      __BIT_FIELD(v, 10, 8) /* UIC Power Mode Change Request Status */
#define   RT_UFS_REG_HCS_UTPEC(v)       __BIT_FIELD(v, 15, 12) /* UTP Error Code */
#define   RT_UFS_REG_HCS_TTAGUTPE(v)    __BIT_FIELD(v, 23, 16) /* Task Tag of UTP error */
#define   RT_UFS_REG_HCS_TLUNUTPE(v)    __BIT_FIELD(v, 31, 24) /* Target LUN of UTP error */
#define RT_UFS_REG_HCE                  0x34  /* Host Controller Enable */
#define RT_UFS_REG_UECPA                0x38  /* Host UIC Error Code PHY Adapter Layer */
#define   RT_UFS_REG_UECPA_EC(v)        __BIT_FIELD(v, 4, 0) /* UIC PHY Adapter Layer Error Code */
#define   RT_UFS_REG_UECPA_ERR          RT_BIT(31) /* UIC PHY AdapterA Layer Error */
#define RT_UFS_REG_UECDL                0x3c  /* Host UIC Error Code Data Link Layer */
#define   RT_UFS_REG_UECDL_EC(v)        __BIT_FIELD(v, 14, 0) /* UIC Data Link Layer Error Code */
#define   RT_UFS_REG_UECDL_ERR          RT_BIT(31) /* UIC Data Link Layer Error */
#define RT_UFS_REG_UECN                 0x40  /* Host UIC Error Code Network Layer */
#define   RT_UFS_REG_UECN_EC(v)         __BIT_FIELD(v, 2, 0) /* UIC Network Layer Error Code */
#define   RT_UFS_REG_UECN_ERR           RT_BIT(31) /* UIC Network Layer Error */
#define RT_UFS_REG_UECT                 0x44  /* Host UIC Error Code Transport Layer */
#define   RT_UFS_REG_UECT_EC(v)         __BIT_FIELD(v, 6, 0) /* UIC Transport Layer Error Code */
#define   RT_UFS_REG_UECT_ERR           RT_BIT(31) /* UIC Transport Layer Error */
#define RT_UFS_REG_UECDME               0x48  /* Host UIC Error Code DME */
#define   RT_UFS_REG_UECDME_EC(v)       __BIT_FIELD(v, 0, 0) /* UIC DME Error Code */
#define   RT_UFS_REG_UECDME_ERR         RT_BIT(31) /* UIC DME Error */
#define RT_UFS_REG_UTRIACR              0x4c  /* UTP Transfer Request Interrupt Aggregation Control Register */
#define   RT_UFS_REG_UTRIACR_IAEN       RT_BIT(31) /* Interrupt Aggregation Enable/Disable */
#define   RT_UFS_REG_UTRIACR_IAPWEN     RT_BIT(24) /* Interrupt aggregation parameter write enable */
#define   RT_UFS_REG_UTRIACR_IASB       RT_BIT(20) /* Interrupt aggregation status bit */
#define   RT_UFS_REG_UTRIACR_CTR        RT_BIT(16) /* Counter and Timer Reset */
#define   RT_UFS_REG_UTRIACR_IACTH(v)   __BIT_FIELD(v, 12, 8) /* Interrupt aggregation counter threshold */
#define   RT_UFS_REG_UTRIACR_IATOVAL(v) __BIT_FIELD(v, 7, 0) /* Interrupt aggregation counter threshold */
/* UTP Transfer */
#define RT_UFS_REG_UTRLBA               0x50 /* UTP Transfer Request List Base Address */
#define   RT_UFS_REG_UTRLBA_MASK(v)     __BIT_FIELD(v, 31, 10)
#define RT_UFS_REG_UTRLBAU              0x54 /* UTP Transfer Request List Base Address Upper 32-Bits */
#define   RT_UFS_REG_UTRLBAU_MASK(v)    __BIT_FIELD(v, 31, 0)
#define RT_UFS_REG_UTRLDBR              0x58 /* UTP Transfer Request List Door Bell Register */
#define RT_UFS_REG_UTRLCLR              0x5c /* UTP Transfer Request List CLear Register */
#define RT_UFS_REG_UTRLRSR              0x60 /* UTP Transfer Request Run-Stop Register */
/* UTP Task Managemeng */
#define RT_UFS_REG_UTMRLBA              0x70 /* UTP Task Management Request List Base Address */
#define   RT_UFS_REG_UTMRLBA_MASK(v)    __BIT_FIELD(v, 31, 10)
#define RT_UFS_REG_UTMRLBAU             0x74 /* UTP Task Management Request List Base Address Upper 32-Bits */
#define   RT_UFS_REG_UTMRLBAU_MASK(v)   __BIT_FIELD(v, 31, 0)
#define RT_UFS_REG_UTMRLDBR             0x78 /* UTP Task Management Request List Door Bell Register */
#define RT_UFS_REG_UTMRLCLR             0x7c /* UTP Task Management Request List CLear Register */
#define RT_UFS_REG_UTMRLRSR             0x80 /* UTP Task Management Run-Stop Register */
/* UIC Command */
#define RT_UFS_REG_UICCMD               0x90 /* UIC Command Register */
#define   RT_UFS_REG_UICCMD_CMDOP(v)    __BIT_FIELD(v, 7, 0) /* Command Opcode */
#define RT_UFS_REG_UCMDARG1             0x94 /* UIC Command Argument 1 */
#define RT_UFS_REG_UCMDARG2             0x98 /* UIC Command Argument 2 */
#define RT_UFS_REG_UCMDARG3             0x9c /* UIC Command Argument 3 */
/* UMA */
#define RT_UFS_REG_UMA_EXT              0xb0 /* Reserved for Unified Memory Extension */
/* Vendor Specific */
#define RT_UFS_REG_VS                   0xc0 /* Vendor Specific Registers */

/* UTP Error Code */
enum
{
    RT_UFS_UTPEC_ERR_INV_TYPE           = 1,
};

/* UIC Power Mode Change Request Status */
enum
{
    RT_UFS_UPMCRS_PWR_OK                = 0x0,
    RT_UFS_UPMCRS_PWR_LOCAL             = 0x1,
    RT_UFS_UPMCRS_PWR_REMOTE            = 0x2,
    RT_UFS_UPMCRS_PWR_BUSY              = 0x3,
    RT_UFS_UPMCRS_PWR_ERROR_CAP         = 0x4,
    RT_UFS_UPMCRS_PWR_FATAL_ERROR       = 0x5,
};

/* UIC PHY Adapter Layer Error Code */
enum
{
    RT_UFS_UECPA_EC_LANE0                           = RT_BIT(0),
    RT_UFS_UECPA_EC_LANE1                           = RT_BIT(1),
    RT_UFS_UECPA_EC_LANE2                           = RT_BIT(2),
    RT_UFS_UECPA_EC_LANE3                           = RT_BIT(3),
    RT_UFS_UECPA_EC_GENERIC                         = RT_BIT(4),
};

/* UIC Data Link Layer Error Code */
enum
{
    RT_UFS_UECDL_EC_NAC_RECEIVED                    = RT_BIT(0),
    RT_UFS_UECDL_EC_TCx_REPLAY_TIMER_EXPIRED        = RT_BIT(1),
    RT_UFS_UECDL_EC_AFCx_REQUEST_TIMER_EXPIRED      = RT_BIT(2),
    RT_UFS_UECDL_EC_FCx_PROTECTION_TIMER_EXPIRED    = RT_BIT(3),
    RT_UFS_UECDL_EC_CRC_ERROR                       = RT_BIT(4),
    RT_UFS_UECDL_EC_RX_BUFFER_OVERFLOW              = RT_BIT(5),
    RT_UFS_UECDL_EC_MAX_FRAME_LENGTH_EXCEEDED       = RT_BIT(6),
    RT_UFS_UECDL_EC_WRONG_SEQUENCE_NUMBER           = RT_BIT(7),
    RT_UFS_UECDL_EC_AFC_FRAME_SYNTAX_ERROR          = RT_BIT(8),
    RT_UFS_UECDL_EC_NAC_FRAME_SYNTAX_ERROR          = RT_BIT(9),
    RT_UFS_UECDL_EC_EOF_SYNTAX_ERROR                = RT_BIT(10),
    RT_UFS_UECDL_EC_FRAME_SYNTAX_ERROR              = RT_BIT(11),
    RT_UFS_UECDL_EC_BAD_CTRL_SYMBOL_TYPE            = RT_BIT(12),
    RT_UFS_UECDL_EC_PA_INIT_ERROR                   = RT_BIT(13),
    RT_UFS_UECDL_EC_PA_ERROR_IND_RECEIVED           = RT_BIT(14),
};

/* UIC Network Layer Error Code */
enum
{
    RT_UFS_UECN_EC_UNSUPPORTED_HEADER_TYPE          = RT_BIT(0),
    RT_UFS_UECN_EC_BAD_DEVICEID_ENC                 = RT_BIT(1),
    RT_UFS_UECN_EC_LHDR_TRAP_PACKET_DROPPING        = RT_BIT(2),
};

/* UIC Transport Layer Error Code */
enum
{
    RT_UFS_UECT_EC_UNSUPPORTED_HEADER_TYPE          = RT_BIT(0),
    RT_UFS_UECT_EC_UNKNOWN_CPORTID                  = RT_BIT(1),
    RT_UFS_UECT_EC_NO_CONNECTION_RX                 = RT_BIT(2),
    RT_UFS_UECT_EC_CONTROLLED_SEGMENT_DROPPING      = RT_BIT(3),
    RT_UFS_UECT_EC_BAD_TC                           = RT_BIT(4),
    RT_UFS_UECT_EC_E2E_CREDIT_OVERFLOW              = RT_BIT(5),
    RT_UFS_UECT_EC_SAFETY_VALVE_DROPPING            = RT_BIT(6),
};

/* UIC DME Error Code */
enum
{
    RT_UFS_UECDME_EC_GENERIC                        = RT_BIT(0),
};

/* UIC Command Opcode */
enum
{
    /* Configuration */
    RT_UFS_CMDOP_DME_GET                            = 0x1,
    RT_UFS_CMDOP_DME_SET                            = 0x2,
    RT_UFS_CMDOP_DME_PEER_GET                       = 0x3,
    RT_UFS_CMDOP_DME_PEER_SET                       = 0x4,
    /* Control */
    RT_UFS_CMDOP_DME_POWERON                        = 0x10,
    RT_UFS_CMDOP_DME_POWEROFF                       = 0x11,
    RT_UFS_CMDOP_DME_ENABLE                         = 0x12,
    RT_UFS_CMDOP_DME_RESET                          = 0x14,
    RT_UFS_CMDOP_DME_ENDPOINTRESET                  = 0x15,
    RT_UFS_CMDOP_DME_LINKSTARTUP                    = 0x16,
    RT_UFS_CMDOP_DME_HIBERNATE_ENTER                = 0x17,
    RT_UFS_CMDOP_DME_HIBERNATE_EXIT                 = 0x18,
    RT_UFS_CMDOP_DME_TEST_MODE                      = 0x1a,
};

/* UIC Config result code / Generic error code */
enum
{
    RT_UFS_CMDRES_SUCCESS                           = 0x0,
    RT_UFS_CMDRES_INVALID_MIB_ATTRIBUTE             = 0x1,
    RT_UFS_CMDRES_INVALID_MIB_ATTRIBUTE_VALUE       = 0x2,
    RT_UFS_CMDRES_READ_ONLY_MIB_ATTRIBUTE           = 0x3,
    RT_UFS_CMDRES_WRITE_ONLY_MIB_ATTRIBUTE          = 0x4,
    RT_UFS_CMDRES_BAD_INDEX                         = 0x5,
    RT_UFS_CMDRES_LOCKED_MIB_ATTRIBUTE              = 0x6,
    RT_UFS_CMDRES_BAD_TEST_FEATURE_INDEX            = 0x7,
    RT_UFS_CMDRES_PEER_COMMUNICATION_FAILURE        = 0x8,
    RT_UFS_CMDRES_BUSY                              = 0x9,
    RT_UFS_CMDRES_DME_FAILURE                       = 0xa,
    RT_UFS_CMDRES_MASK                              = 0xff,
};

struct rt_ufs_ops;

enum rt_ufs_notify_change_status
{
    RT_UFS_NOTIFY_CHANGE_STATUS_PRE,
    RT_UFS_NOTIFY_CHANGE_STATUS_POST,
};

struct rt_ufs_host
{
    struct rt_scsi_host parent;

    void *regs;
    int irq;

    rt_uint32_t cap;

    const struct rt_ufs_ops *ops;

    struct rt_completion done;
    struct rt_spinlock lock;
};

struct rt_ufs_ops
{
    rt_err_t (*init)(struct rt_ufs_host *ufs);
    rt_err_t (*exit)(struct rt_ufs_host *ufs);
    rt_err_t (*reset)(struct rt_ufs_host *ufs);
    rt_err_t (*link_startup_notify)(struct rt_ufs_host *ufs, enum rt_ufs_notify_change_status status);
};

rt_err_t rt_ufs_host_register(struct rt_ufs_host *ufs);
rt_err_t rt_ufs_host_unregister(struct rt_ufs_host *ufs);

#endif /* __UFS_H__ */
