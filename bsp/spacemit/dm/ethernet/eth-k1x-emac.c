/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-5-1       GuEe-GUI     first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "ethernet.k1x_emac"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <spacemit.h>
#include "ethernet_dm.h"

#define PHY_INTF_RGMII                                  RT_BIT(2)

/*
 * Only valid for rmii mode
 * 0: ref clock from external phy
 * 1: ref clock from soc
 */
#define REF_CLK_SEL                                     RT_BIT(3)

/*
 * EMAC function clock select
 * 0: 208M
 * 1: 312M
 */
#define FUNC_CLK_SEL                                    RT_BIT(4)

/* Only valid for rmii, invert tx clk */
#define RMII_TX_CLK_SEL                                 RT_BIT(6)

/* Only valid for rmii, invert rx clk */
#define RMII_RX_CLK_SEL                                 RT_BIT(7)

/*
 * Only valid for rgmiii
 * 0: tx clk from rx clk
 * 1: tx clk from soc
 */
#define RGMII_TX_CLK_SEL                                RT_BIT(8)

#define PHY_IRQ_EN                                      RT_BIT(12)
#define AXI_SINGLE_ID                                   RT_BIT(13)

#define RMII_TX_PHASE_OFFSET                            16
#define RMII_TX_PHASE_MASK                              RT_GENMASK(18, RMII_TX_PHASE_OFFSET)
#define RMII_RX_PHASE_OFFSET                            20
#define RMII_RX_PHASE_MASK                              RT_GENMASK(22, RMII_RX_PHASE_OFFSET)

#define RGMII_TX_PHASE_OFFSET                           24
#define RGMII_TX_PHASE_MASK                             RT_GENMASK(26, RGMII_TX_PHASE_OFFSET)
#define RGMII_RX_PHASE_OFFSET                           28
#define RGMII_RX_PHASE_MASK                             RT_GENMASK(30, RGMII_RX_PHASE_OFFSET)

#define EMAC_RX_DLINE_EN                                RT_BIT(0)
#define EMAC_RX_DLINE_STEP_OFFSET                       4
#define EMAC_RX_DLINE_STEP_MASK                         RT_GENMASK(5, EMAC_RX_DLINE_STEP_OFFSET)
#define EMAC_RX_DLINE_CODE_OFFSET                       8
#define EMAC_RX_DLINE_CODE_MASK                         RT_GENMASK(15, EMAC_RX_DLINE_CODE_OFFSET)

#define EMAC_TX_DLINE_EN                                RT_BIT(16)
#define EMAC_TX_DLINE_STEP_OFFSET                       20
#define EMAC_TX_DLINE_STEP_MASK                         RT_GENMASK(21, EMAC_TX_DLINE_STEP_OFFSET)
#define EMAC_TX_DLINE_CODE_OFFSET                       24
#define EMAC_TX_DLINE_CODE_MASK                         RT_GENMASK(31, EMAC_TX_DLINE_CODE_OFFSET)

/* DMA register set */
#define DMA_CONFIGURATION                               0x0000
#define DMA_CONTROL                                     0x0004
#define DMA_STATUS_IRQ                                  0x0008
#define DMA_INTERRUPT_ENABLE                            0x000c

#define DMA_TRANSMIT_AUTO_POLL_COUNTER                  0x0010
#define DMA_TRANSMIT_POLL_DEMAND                        0x0014
#define DMA_RECEIVE_POLL_DEMAND                         0x0018

#define DMA_TRANSMIT_BASE_ADDRESS                       0x001c
#define DMA_RECEIVE_BASE_ADDRESS                        0x0020
#define DMA_MISSED_FRAME_COUNTER                        0x0024
#define DMA_STOP_FLUSH_COUNTER                          0x0028

#define DMA_RECEIVE_IRQ_MITIGATION_CTRL                 0x002c

#define DMA_CURRENT_TRANSMIT_DESCRIPTOR_POINTER         0x0030
#define DMA_CURRENT_TRANSMIT_BUFFER_POINTER             0x0034
#define DMA_CURRENT_RECEIVE_DESCRIPTOR_POINTER          0x0038
#define DMA_CURRENT_RECEIVE_BUFFER_POINTER              0x003c

/* MAC Register set */
#define MAC_GLOBAL_CONTROL                              0x0100
#define MAC_TRANSMIT_CONTROL                            0x0104
#define MAC_RECEIVE_CONTROL                             0x0108
#define MAC_MAXIMUM_FRAME_SIZE                          0x010c
#define MAC_TRANSMIT_JABBER_SIZE                        0x0110
#define MAC_RECEIVE_JABBER_SIZE                         0x0114
#define MAC_ADDRESS_CONTROL                             0x0118
#define MAC_MDIO_CLK_DIV                                0x011c
#define MAC_ADDRESS1_HIGH                               0x0120
#define MAC_ADDRESS1_MED                                0x0124
#define MAC_ADDRESS1_LOW                                0x0128
#define MAC_ADDRESS2_HIGH                               0x012c
#define MAC_ADDRESS2_MED                                0x0130
#define MAC_ADDRESS2_LOW                                0x0134
#define MAC_ADDRESS3_HIGH                               0x0138
#define MAC_ADDRESS3_MED                                0x013c
#define MAC_ADDRESS3_LOW                                0x0140
#define MAC_ADDRESS4_HIGH                               0x0144
#define MAC_ADDRESS4_MED                                0x0148
#define MAC_ADDRESS4_LOW                                0x014c
#define MAC_MULTICAST_HASH_TABLE1                       0x0150
#define MAC_MULTICAST_HASH_TABLE2                       0x0154
#define MAC_MULTICAST_HASH_TABLE3                       0x0158
#define MAC_MULTICAST_HASH_TABLE4                       0x015c
#define MAC_FC_CONTROL                                  0x0160
#define MAC_FC_PAUSE_FRAME_GENERATE                     0x0164
#define MAC_FC_SOURCE_ADDRESS_HIGH                      0x0168
#define MAC_FC_SOURCE_ADDRESS_MED                       0x016c
#define MAC_FC_SOURCE_ADDRESS_LOW                       0x0170
#define MAC_FC_DESTINATION_ADDRESS_HIGH                 0x0174
#define MAC_FC_DESTINATION_ADDRESS_MED                  0x0178
#define MAC_FC_DESTINATION_ADDRESS_LOW                  0x017c
#define MAC_FC_PAUSE_TIME_VALUE                         0x0180
#define MAC_MDIO_CONTROL                                0x01a0
#define MAC_MDIO_DATA                                   0x01a4
#define MAC_RX_STATCTR_CONTROL                          0x01a8
#define MAC_RX_STATCTR_DATA_HIGH                        0x01ac
#define MAC_RX_STATCTR_DATA_LOW                         0x01b0
#define MAC_TX_STATCTR_CONTROL                          0x01b4
#define MAC_TX_STATCTR_DATA_HIGH                        0x01b8
#define MAC_TX_STATCTR_DATA_LOW                         0x01bc
#define MAC_TRANSMIT_FIFO_ALMOST_FULL                   0x01c0
#define MAC_TRANSMIT_PACKET_START_THRESHOLD             0x01c4
#define MAC_RECEIVE_PACKET_START_THRESHOLD              0x01c8
#define MAC_STATUS_IRQ                                  0x01e0
#define MAC_INTERRUPT_ENABLE                            0x01e4

/*
 * DMA_CONFIGURATION (0x0000) register bit info
 * 0-DMA controller in normal operation mode,
 * 1-DMA controller rstc to default state,
 * clearing all internal state information
 */
#define MREGBIT_SOFTWARE_RESET                          RT_BIT(0)
#define MREGBIT_BURST_1WORD                             RT_BIT(1)
#define MREGBIT_BURST_2WORD                             RT_BIT(2)
#define MREGBIT_BURST_4WORD                             RT_BIT(3)
#define MREGBIT_BURST_8WORD                             RT_BIT(4)
#define MREGBIT_BURST_16WORD                            RT_BIT(5)
#define MREGBIT_BURST_32WORD                            RT_BIT(6)
#define MREGBIT_BURST_64WORD                            RT_BIT(7)
#define MREGBIT_BURST_LENGTH                            RT_GENMASK(7, 1)
#define MREGBIT_DESCRIPTOR_SKIP_LENGTH                  RT_GENMASK(12, 8)
/* For Receive and Transmit DMA operate in Big-Endian mode for Descriptors. */
#define MREGBIT_DESCRIPTOR_BYTE_ORDERING                RT_BIT(13)
#define MREGBIT_BIG_LITLE_ENDIAN                        RT_BIT(14)
#define MREGBIT_TX_RX_ARBITRATION                       RT_BIT(15)
#define MREGBIT_WAIT_FOR_DONE                           RT_BIT(16)
#define MREGBIT_STRICT_BURST                            RT_BIT(17)
#define MREGBIT_DMA_64BIT_MODE                          RT_BIT(18)

/* DMA_CONTROL (0x0004) register bit info */
#define MREGBIT_START_STOP_TRANSMIT_DMA                 RT_BIT(0)
#define MREGBIT_START_STOP_RECEIVE_DMA                  RT_BIT(1)

/* DMA_STATUS_IRQ (0x0008) register bit info */
#define MREGBIT_TRANSMIT_TRANSFER_DONE_IRQ              RT_BIT(0)
#define MREGBIT_TRANSMIT_DES_UNAVAILABLE_IRQ            RT_BIT(1)
#define MREGBIT_TRANSMIT_DMA_STOPPED_IRQ                RT_BIT(2)
#define MREGBIT_RECEIVE_TRANSFER_DONE_IRQ               RT_BIT(4)
#define MREGBIT_RECEIVE_DES_UNAVAILABLE_IRQ             RT_BIT(5)
#define MREGBIT_RECEIVE_DMA_STOPPED_IRQ                 RT_BIT(6)
#define MREGBIT_RECEIVE_MISSED_FRAME_IRQ                RT_BIT(7)
#define MREGBIT_MAC_IRQ                                 RT_BIT(8)
#define MREGBIT_TRANSMIT_DMA_STATE                      RT_GENMASK(18, 16)
#define MREGBIT_RECEIVE_DMA_STATE                       RT_GENMASK(23, 20)

/* DMA_INTERRUPT_ENABLE ( 0x000C) register bit info */
#define MREGBIT_TRANSMIT_TRANSFER_DONE_INTR_ENABLE      RT_BIT(0)
#define MREGBIT_TRANSMIT_DES_UNAVAILABLE_INTR_ENABLE    RT_BIT(1)
#define MREGBIT_TRANSMIT_DMA_STOPPED_INTR_ENABLE        RT_BIT(2)
#define MREGBIT_RECEIVE_TRANSFER_DONE_INTR_ENABLE       RT_BIT(4)
#define MREGBIT_RECEIVE_DES_UNAVAILABLE_INTR_ENABLE     RT_BIT(5)
#define MREGBIT_RECEIVE_DMA_STOPPED_INTR_ENABLE         RT_BIT(6)
#define MREGBIT_RECEIVE_MISSED_FRAME_INTR_ENABLE        RT_BIT(7)
#define MREGBIT_MAC_INTR_ENABLE                         RT_BIT(8)

/* DMA RECEIVE IRQ MITIGATION CONTROL */
#define MREGBIT_RECEIVE_IRQ_FRAME_COUNTER_MSK           RT_GENMASK(7, 0)
#define MREGBIT_RECEIVE_IRQ_TIMEOUT_COUNTER_OFST        8
#define MREGBIT_RECEIVE_IRQ_TIMEOUT_COUNTER_MSK         RT_GENMASK(27, MREGBIT_RECEIVE_IRQ_TIMEOUT_COUNTER_OFST)
#define MRGEBIT_RECEIVE_IRQ_FRAME_COUNTER_MODE          RT_BIT(30)
#define MRGEBIT_RECEIVE_IRQ_MITIGATION_ENABLE           RT_BIT(31)

/* MAC_GLOBAL_CONTROL (0x0100) register bit info */
#define MREGBIT_SPEED                                   RT_GENMASK(1, 0)
#define MREGBIT_SPEED_10M                               0x0
#define MREGBIT_SPEED_100M                              RT_BIT(0)
#define MREGBIT_SPEED_1000M                             RT_BIT(1)
#define MREGBIT_FULL_DUPLEX_MODE                        RT_BIT(2)
#define MREGBIT_RESET_RX_STAT_COUNTERS                  RT_BIT(3)
#define MREGBIT_RESET_TX_STAT_COUNTERS                  RT_BIT(4)
#define MREGBIT_UNICAST_WAKEUP_MODE                     RT_BIT(8)
#define MREGBIT_MAGIC_PACKET_WAKEUP_MODE                RT_BIT(9)

/* MAC_TRANSMIT_CONTROL (0x0104) register bit info */
#define MREGBIT_TRANSMIT_ENABLE                         RT_BIT(0)
#define MREGBIT_INVERT_FCS                              RT_BIT(1)
#define MREGBIT_DISABLE_FCS_INSERT                      RT_BIT(2)
#define MREGBIT_TRANSMIT_AUTO_RETRY                     RT_BIT(3)
#define MREGBIT_IFG_LEN                                 RT_GENMASK(6, 4)
#define MREGBIT_PREAMBLE_LENGTH                         RT_GENMASK(9, 7)

/* MAC_RECEIVE_CONTROL (0x0108) register bit info */
#define MREGBIT_RECEIVE_ENABLE                          RT_BIT(0)
#define MREGBIT_DISABLE_FCS_CHECK                       RT_BIT(1)
#define MREGBIT_STRIP_FCS                               RT_BIT(2)
#define MREGBIT_STORE_FORWARD                           RT_BIT(3)
#define MREGBIT_STATUS_FIRST                            RT_BIT(4)
#define MREGBIT_PASS_BAD_FRAMES                         RT_BIT(5)
#define MREGBIT_ACOOUNT_VLAN                            RT_BIT(6)

/* MAC_MAXIMUM_FRAME_SIZE (0x010C) register bit info */
#define MREGBIT_MAX_FRAME_SIZE                          RT_GENMASK(13, 0)

/* MAC_TRANSMIT_JABBER_SIZE (0x0110) register bit info */
#define MREGBIT_TRANSMIT_JABBER_SIZE                    RT_GENMASK(15, 0)

/* MAC_RECEIVE_JABBER_SIZE (0x0114) register bit info */
#define MREGBIT_RECEIVE_JABBER_SIZE                     RT_GENMASK(15, 0)

/* MAC_ADDRESS_CONTROL   (0x0118) register bit info */
#define MREGBIT_MAC_ADDRESS1_ENABLE                     RT_BIT(0)
#define MREGBIT_MAC_ADDRESS2_ENABLE                     RT_BIT(1)
#define MREGBIT_MAC_ADDRESS3_ENABLE                     RT_BIT(2)
#define MREGBIT_MAC_ADDRESS4_ENABLE                     RT_BIT(3)
#define MREGBIT_INVERSE_MAC_ADDRESS1_ENABLE             RT_BIT(4)
#define MREGBIT_INVERSE_MAC_ADDRESS2_ENABLE             RT_BIT(5)
#define MREGBIT_INVERSE_MAC_ADDRESS3_ENABLE             RT_BIT(6)
#define MREGBIT_INVERSE_MAC_ADDRESS4_ENABLE             RT_BIT(7)
#define MREGBIT_PROMISCUOUS_MODE                        RT_BIT(8)

/* MAC_ADDRESSx_HIGH (0x0120) register bit info */
#define MREGBIT_MAC_ADDRESS1_01_BYTE                    RT_GENMASK(7, 0)
#define MREGBIT_MAC_ADDRESS1_02_BYTE                    RT_GENMASK(15, 8)
/* MAC_ADDRESSx_MED (0x0124) register bit info */
#define MREGBIT_MAC_ADDRESS1_03_BYTE                    RT_GENMASK(7, 0)
#define MREGBIT_MAC_ADDRESS1_04_BYTE                    RT_GENMASK(15, 8)
/* MAC_ADDRESSx_LOW (0x0128) register bit info */
#define MREGBIT_MAC_ADDRESS1_05_BYTE                    RT_GENMASK(7, 0)
#define MREGBIT_MAC_ADDRESS1_06_BYTE                    RT_GENMASK(15, 8)

/* MAC_FC_CONTROL (0x0160) register bit info */
#define MREGBIT_FC_DECODE_ENABLE                        RT_BIT(0)
#define MREGBIT_FC_GENERATION_ENABLE                    RT_BIT(1)
#define MREGBIT_AUTO_FC_GENERATION_ENABLE               RT_BIT(2)
#define MREGBIT_MULTICAST_MODE                          RT_BIT(3)
#define MREGBIT_BLOCK_PAUSE_FRAMES                      RT_BIT(4)

/* MAC_FC_PAUSE_FRAME_GENERATE (0x0164) register bit info */
#define MREGBIT_GENERATE_PAUSE_FRAME                    RT_BIT(0)

/* MAC_FC_SRC/DST_ADDRESS_HIGH (0x0168) register bit info */
#define MREGBIT_MAC_ADDRESS_01_BYTE                     RT_GENMASK(7, 0)
#define MREGBIT_MAC_ADDRESS_02_BYTE                     RT_GENMASK(15, 8)
/* MAC_FC_SRC/DST_ADDRESS_MED (0x016C) register bit info */
#define MREGBIT_MAC_ADDRESS_03_BYTE                     RT_GENMASK(7, 0)
#define MREGBIT_MAC_ADDRESS_04_BYTE                     RT_GENMASK(15, 8)
/* MAC_FC_SRC/DSTD_ADDRESS_LOW (0x0170) register bit info */
#define MREGBIT_MAC_ADDRESS_05_BYTE                     RT_GENMASK(7, 0)
#define MREGBIT_MAC_ADDRESS_06_BYTE                     RT_GENMASK(15, 8)

/* MAC_FC_PAUSE_TIME_VALUE (0x0180) register bit info */
#define MREGBIT_MAC_FC_PAUSE_TIME                       RT_GENMASK(15, 0)

/* MAC_MDIO_CONTROL (0x01A0) register bit info */
#define MREGBIT_PHY_ADDRESS                             RT_GENMASK(4, 0)
#define MREGBIT_REGISTER_ADDRESS                        RT_GENMASK(9, 5)
#define MREGBIT_MDIO_READ_WRITE                         RT_BIT(10)
#define MREGBIT_START_MDIO_TRANS                        RT_BIT(15)

/* MAC_MDIO_DATA (0x01A4) register bit info */
#define MREGBIT_MDIO_DATA                               RT_GENMASK(15, 0)

/* MAC_RX_STATCTR_CONTROL (0x01A8) register bit info */
#define MREGBIT_RX_COUNTER_NUMBER                       RT_GENMASK(4, 0)
#define MREGBIT_START_RX_COUNTER_READ                   RT_BIT(15)

/* MAC_RX_STATCTR_DATA_HIGH (0x01AC) register bit info */
#define MREGBIT_RX_STATCTR_DATA_HIGH                    RT_GENMASK(15, 0)
/* MAC_RX_STATCTR_DATA_LOW (0x01B0) register bit info */
#define MREGBIT_RX_STATCTR_DATA_LOW                     RT_GENMASK(15, 0)

/* MAC_TX_STATCTR_CONTROL (0x01B4) register bit info */
#define MREGBIT_TX_COUNTER_NUMBER                       RT_GENMASK(4, 0)
#define MREGBIT_START_TX_COUNTER_READ                   RT_BIT(15)

/* MAC_TX_STATCTR_DATA_HIGH (0x01B8) register bit info */
#define MREGBIT_TX_STATCTR_DATA_HIGH                    RT_GENMASK(15, 0)
/* MAC_TX_STATCTR_DATA_LOW (0x01BC) register bit info */
#define MREGBIT_TX_STATCTR_DATA_LOW                     RT_GENMASK(15, 0)

/* MAC_TRANSMIT_FIFO_ALMOST_FULL (0x01C0) register bit info */
#define MREGBIT_TX_FIFO_AF                              RT_GENMASK(13, 0)

/* MAC_TRANSMIT_PACKET_START_THRESHOLD (0x01C4) register bit info */
#define MREGBIT_TX_PACKET_START_THRESHOLD               RT_GENMASK(13, 0)

/* MAC_RECEIVE_PACKET_START_THRESHOLD (0x01C8) register bit info */
#define MREGBIT_RX_PACKET_START_THRESHOLD               RT_GENMASK(13, 0)

/* MAC_STATUS_IRQ  (0x01E0) register bit info */
#define MREGBIT_MAC_UNDERRUN_IRQ                        RT_BIT(0)
#define MREGBIT_MAC_JABBER_IRQ                          RT_BIT(1)

/* MAC_INTERRUPT_ENABLE (0x01E4) register bit info */
#define MREGBIT_MAC_UNDERRUN_INTERRUPT_ENABLE           RT_BIT(0)
#define MREGBIT_JABBER_INTERRUPT_ENABLE                 RT_BIT(1)

/* Receive Descriptors */
/* MAC_RECEIVE_DESCRIPTOR0 () register bit info */
#define MREGBIT_FRAME_LENGTH                            RT_GENMASK(13, 0)
#define MREGBIT_APPLICATION_STATUS                      RT_GENMASK(28, 14)
#define MREGBIT_LAST_DESCRIPTOR                         RT_BIT(29)
#define MREGBIT_FIRST_DESCRIPTOR                        RT_BIT(30)
#define MREGBIT_OWN_BIT                                 RT_BIT(31)

/* MAC_RECEIVE_DESCRIPTOR1 () register bit info */
#define MREGBIT_BUFFER1_SIZE                            RT_GENMASK(11, 0)
#define MREGBIT_BUFFER2_SIZE                            RT_GENMASK(23, 12)
#define MREGBIT_SECOND_ADDRESS_CHAINED                  RT_BIT(25)
#define MREGBIT_END_OF_RING                             RT_BIT(26)

/* MAC_RECEIVE_DESCRIPTOR2 () register bit info */
#define MREGBIT_BUFFER_ADDRESS1                         RT_GENMASK(31, 0)

/* MAC_RECEIVE_DESCRIPTOR3 () register bit info */
#define MREGBIT_BUFFER_ADDRESS1                         RT_GENMASK(31, 0)

/* Transmit Descriptors */
/* TD_TRANSMIT_DESCRIPTOR0 () register bit info */
#define MREGBIT_TX_PACKET_STATUS                        RT_GENMASK(29, 0)
#define MREGBIT_OWN_BIT                                 RT_BIT(31)

/* TD_TRANSMIT_DESCRIPTOR1 () register bit info */
#define MREGBIT_BUFFER1_SIZE                            RT_GENMASK(11, 0)
#define MREGBIT_BUFFER2_SIZE                            RT_GENMASK(23, 12)
#define MREGBIT_FORCE_EOP_ERROR                         RT_BIT(24)
#define MREGBIT_SECOND_ADDRESS_CHAINED                  RT_BIT(25)
#define MREGBIT_END_OF_RING                             RT_BIT(26)
#define MREGBIT_DISABLE_PADDING                         RT_BIT(27)
#define MREGBIT_ADD_CRC_DISABLE                         RT_BIT(28)
#define MREGBIT_FIRST_SEGMENT                           RT_BIT(29)
#define MREGBIT_LAST_SEGMENT                            RT_BIT(30)
#define MREGBIT_INTERRUPT_ON_COMPLETION                 RT_BIT(31)

/* TD_TRANSMIT_DESCRIPTOR2 () register bit info */
#define MREGBIT_BUFFER_ADDRESS1                         RT_GENMASK(31, 0)

/* TD_TRANSMIT_DESCRIPTOR3 () register bit info */
#define MREGBIT_BUFFER_ADDRESS1                         RT_GENMASK(31, 0)

/* RX frame status */
#define EMAC_RX_FRAME_ALIGN_ERR                         RT_BIT(0)
#define EMAC_RX_FRAME_RUNT                              RT_BIT(1)
#define EMAC_RX_FRAME_ETHERNET_TYPE                     RT_BIT(2)
#define EMAC_RX_FRAME_VLAN                              RT_BIT(3)
#define EMAC_RX_FRAME_MULTICAST                         RT_BIT(4)
#define EMAC_RX_FRAME_BROADCAST                         RT_BIT(5)
#define EMAC_RX_FRAME_CRC_ERR                           RT_BIT(6)
#define EMAC_RX_FRAME_MAX_LEN_ERR                       RT_BIT(7)
#define EMAC_RX_FRAME_JABBER_ERR                        RT_BIT(8)
#define EMAC_RX_FRAME_LENGTH_ERR                        RT_BIT(9)
#define EMAC_RX_FRAME_MAC_ADDR1_MATCH                   RT_BIT(10)
#define EMAC_RX_FRAME_MAC_ADDR2_MATCH                   RT_BIT(11)
#define EMAC_RX_FRAME_MAC_ADDR3_MATCH                   RT_BIT(12)
#define EMAC_RX_FRAME_MAC_ADDR4_MATCH                   RT_BIT(13)
#define EMAC_RX_FRAME_PAUSE_CTRL                        RT_BIT(14)

/* EMAC ptp 1588 register */
#define PTP_1588_CTRL                                   0x300
#define TX_TIMESTAMP_EN                                 RT_BIT(1)
#define RX_TIMESTAMP_EN                                 RT_BIT(2)
#define RX_PTP_PKT_TYPE_OFST                            3
#define RX_PTP_PKT_TYPE_MSK                             RT_GENMASK(5, RX_PTP_PKT_TYPE_OFST)

#define PTP_INRC_ATTR                                   0x304
#define INRC_VAL_MSK                                    RT_GENMASK(23, 0)
#define INCR_PERIOD_OFST                                24
#define INCR_PERIOD_MSK                                 RT_GENMASK(31, INCR_PERIOD_OFST)

#define PTP_ETH_TYPE                                    0x308
#define PTP_ETH_TYPE_MSK                                RT_GENMASK(15, 0)

#define PTP_MSG_ID                                      0x30c

#define PTP_UDP_PORT                                    0x310
#define PTP_UDP_PORT_MSK                                RT_GENMASK(15, 0)

/* Read current system time from controller */
#define SYS_TIME_GET_LOW                                0x320
#define SYS_TIME_GET_HI                                 0x324

#define SYS_TIME_ADJ_LOW                                0x328
#define SYS_TIME_LOW_MSK                                RT_GENMASK(31, 0)
#define SYS_TIME_ADJ_HI                                 0x32c
#define SYS_TIME_IS_NEG                                 RT_BIT(31)

#define TX_TIMESTAMP_LOW                                0x330
#define TX_TIMESTAMP_HI                                 0x334

#define RX_TIMESTAMP_LOW                                0x340
#define RX_TIMESTAMP_HI                                 0x344

#define RX_PTP_PKT_ATTR_LOW                             0x348
#define PTP_SEQ_ID_MSK                                  RT_GENMASK(15, 0)
#define PTP_SRC_ID_LOW_OFST                             16
#define PTP_SRC_ID_LOW_MSK                              RT_GENMASK(31, PTP_SRC_ID_LOW_OFST)

#define RX_PTP_PKT_ATTR_MID                             0x34c
#define PTP_SRC_ID_MID_MSK                              RT_GENMASK(31, 0)

#define RX_PTP_PKT_ATTR_HI                              0x350
#define PTP_SRC_ID_HI_MSK                               RT_GENMASK(31, 0)

#define PTP_1588_IRQ_STS                                0x360
#define PTP_1588_IRQ_EN                                 0x364
#define PTP_TX_TIMESTAMP                                RT_BIT(0)
#define PTP_RX_TIMESTAMP                                RT_BIT(1)

/* EMAC ptp register */
#define EMAC_DEFAULT_BUFSIZE                            1536
#define EMAC_RX_BUF_2K                                  2048
#define EMAC_RX_BUF_4K                                  4096

#define MAX_DATA_PWR_TX_DES                             11
#define MAX_DATA_LEN_TX_DES                             2048 //2048=1<<11

#define MAX_TX_STATS_NUM                                12
#define MAX_RX_STATS_NUM                                25

/* The sizes (in bytes) of a ethernet packet */
#define ETHERNET_HEADER_SIZE                            14
#define MAXIMUM_ETHERNET_FRAME_SIZE                     1518  //With FCS
#define MINIMUM_ETHERNET_FRAME_SIZE                     64  //With FCS
#define ETHERNET_FCS_SIZE                               4
#define MAXIMUM_ETHERNET_PACKET_SIZE                    (MAXIMUM_ETHERNET_FRAME_SIZE - ETHERNET_FCS_SIZE)

#define MINIMUM_ETHERNET_PACKET_SIZE                    (MINIMUM_ETHERNET_FRAME_SIZE - ETHERNET_FCS_SIZE)

#define CRC_LENGTH                                      ETHERNET_FCS_SIZE
#define MAX_JUMBO_FRAME_SIZE                            0x3F00

#define TX_STORE_FORWARD_MODE                           0x5EE

#define EMAC_TX_FRAMES                                  64
/* 40ms */
#define EMAC_TX_COAL_TIMEOUT                            40000

#define EMAC_RX_FRAMES                                  64

/*
 * AXI clk 312M, 1us = 312 cycle,
 * every packet almost take 120us when operate at 100Mbps
 * so we set 5 packet delay time which 600us as rx coal timeout
 */
#define EMAC_RX_COAL_TIMEOUT                            (600 * 312)

/* Only works for sizes that are powers of 2 */
#define EMAC_ROUNDUP(i, size)                           ((i) = (((i) + (size) - 1) & ~((size) - 1)))

/* Number of descriptors are required for len */
#define EMAC_TXD_COUNT(S, X)                            (((S) >> (X)) + 1)

#define CLK_PHASE_REVERT                                180

#define TXCLK_PHASE_DEFAULT                             0
#define RXCLK_PHASE_DEFAULT                             0

#define TX_PHASE                                        1
#define RX_PHASE                                        0

#define DEFAULT_TX_THRESHOLD                            192
#define DEFAULT_RX_THRESHOLD                            12
#define DEFAULT_TX_RING_NUM                             128
#define DEFAULT_RX_RING_NUM                             128
#define DEFAULT_DMA_BURST_LEN                           1
#define HASH_TABLE_SIZE                                 64

/* For ptp event message , udp port is 319 */
#define DEFAULT_UDP_PORT                                0x13f

/* PTP ethernet type */
#define DEFAULT_ETH_TYPE                                0x88f7

#define EMAC_SYSTIM_OVERFLOW_PERIOD                     (100 * 60 * 60 * 4)

#define INCVALUE_100MHZ                                 10
#define INCVALUE_SHIFT_100MHZ                           17
#define INCPERIOD_100MHZ                                1

typedef struct ifreq  st_ifreq, *pst_ifreq;

enum rx_frame_status
{
    frame_ok = 0,
    frame_discard,
    frame_max,
};

enum rx_ptp_type
{
    PTP_V2_L2_ONLY = 0x0,
    PTP_V1_L4_ONLY = 0x1,
    PTP_V2_L2_L4  = 0x2,
};

enum ptp_event_msg_id
{
    MSG_SYNC = 0x00,
    MSG_DELAY_REQ = 0x01,
    MSG_PDELAY_REQ = 0x02,
    MSG_PDELAY_RESP = 0x03,
    ALL_EVENTS = 0x03020100,
};

enum emac_state
{
    EMAC_DOWN,
    EMAC_RESET_REQUESTED,
    EMAC_RESETING,
    EMAC_TASK_SCHED,
    EMAC_STATE_MAX,
};

/* Receive Descriptor structure */
struct emac_rx_desc
{
    rt_uint32_t FramePacketLength:14;
    rt_uint32_t ApplicationStatus:15;
    rt_uint32_t LastDescriptor:1;
    rt_uint32_t FirstDescriptor:1;
    rt_uint32_t OWN:1;

    rt_uint32_t BufferSize1:12;
    rt_uint32_t BufferSize2:12;
    rt_uint32_t Reserved1:1;
    rt_uint32_t SecondAddressChained:1;
    rt_uint32_t EndRing:1;
    rt_uint32_t Reserved2:3;
    rt_uint32_t rx_timestamp:1;
    rt_uint32_t ptp_pkt:1;

    rt_uint32_t BufferAddr1;
    rt_uint32_t BufferAddr2;
};

/* Transmit Descriptor */
struct emac_tx_desc
{
    rt_uint32_t FramePacketStatus:30;
    rt_uint32_t tx_timestamp:1;
    rt_uint32_t OWN:1;

    rt_uint32_t BufferSize1:12;
    rt_uint32_t BufferSize2:12;
    rt_uint32_t ForceEOPError:1;
    rt_uint32_t SecondAddressChained:1;
    rt_uint32_t EndRing:1;
    rt_uint32_t DisablePadding:1;
    rt_uint32_t AddCRCDisable:1;
    rt_uint32_t FirstSegment:1;
    rt_uint32_t LastSegment:1;
    rt_uint32_t InterruptOnCompletion:1;

    rt_uint32_t BufferAddr1;
    rt_uint32_t BufferAddr2;
};

struct desc_buf
{
    u64 dma_addr;
    void *buff_addr;
    u16 dma_len;
    u8 map_as_page;
};

/* Descriptor buffer structure */
struct emac_tx_desc_buffer
{
    struct sk_buff *skb;
    struct desc_buf buf[2];
    u8 timestamped;
};

/* Descriptor buffer structure */
struct emac_desc_buffer
{
    struct sk_buff *skb;
    u64 dma_addr;
    void *buff_addr;
    unsigned long ulTimeStamp;
    u16 dma_len;
    u8 map_as_page;
    u8 timestamped;
};

/* Descriptor ring structure */
struct emac_desc_ring
{
    /* virtual memory address to the descriptor ring memory */
    void *desc_addr;
    /* physical address of the descriptor ring */
    dma_addr_t desc_dma_addr;
    /* length of descriptor ring in bytes */
    rt_uint32_t total_size;
    /* number of descriptors in the ring */
    rt_uint32_t total_cnt;
    /* next descriptor to associate a buffer with */
    rt_uint32_t head;
    /* next descriptor to check for DD status bit */
    rt_uint32_t tail;
    /* array of buffer information structs */
    union {
            struct emac_desc_buffer *desc_buf;
            struct emac_tx_desc_buffer *tx_desc_buf;
    };
};

struct emac_hw_stats
{
    rt_uint32_t tx_ok_pkts;
    rt_uint32_t tx_total_pkts;
    rt_uint32_t tx_ok_bytes;
    rt_uint32_t tx_err_pkts;
    rt_uint32_t tx_singleclsn_pkts;
    rt_uint32_t tx_multiclsn_pkts;
    rt_uint32_t tx_lateclsn_pkts;
    rt_uint32_t tx_excessclsn_pkts;
    rt_uint32_t tx_unicast_pkts;
    rt_uint32_t tx_multicast_pkts;
    rt_uint32_t tx_broadcast_pkts;
    rt_uint32_t tx_pause_pkts;
    rt_uint32_t rx_ok_pkts;
    rt_uint32_t rx_total_pkts;
    rt_uint32_t rx_crc_err_pkts;
    rt_uint32_t rx_align_err_pkts;
    rt_uint32_t rx_err_total_pkts;
    rt_uint32_t rx_ok_bytes;
    rt_uint32_t rx_total_bytes;
    rt_uint32_t rx_unicast_pkts;
    rt_uint32_t rx_multicast_pkts;
    rt_uint32_t rx_broadcast_pkts;
    rt_uint32_t rx_pause_pkts;
    rt_uint32_t rx_len_err_pkts;
    rt_uint32_t rx_len_undersize_pkts;
    rt_uint32_t rx_len_oversize_pkts;
    rt_uint32_t rx_len_fragment_pkts;
    rt_uint32_t rx_len_jabber_pkts;
    rt_uint32_t rx_64_pkts;
    rt_uint32_t rx_65_127_pkts;
    rt_uint32_t rx_128_255_pkts;
    rt_uint32_t rx_256_511_pkts;
    rt_uint32_t rx_512_1023_pkts;
    rt_uint32_t rx_1024_1518_pkts;
    rt_uint32_t rx_1519_plus_pkts;
    rt_uint32_t rx_drp_fifo_full_pkts;
    rt_uint32_t rx_truncate_fifo_full_pkts;

    spinlock_t stats_lock;
};

struct emac_priv;
struct emac_hw_ptp
{
    void (*config_hw_tstamping) (struct emac_priv *priv, rt_uint32_t enable, u8 rx_ptp_type, rt_uint32_t ptp_msg_id);
    rt_uint32_t (*config_systime_increment)(struct emac_priv *priv, rt_uint32_t ptp_clock, rt_uint32_t adj_clock);
    int (*init_systime) (struct emac_priv *priv, u64 set_ns);
    u64 (*get_phc_time)(struct emac_priv *priv);
    u64 (*get_tx_timestamp)(struct emac_priv *priv);
    u64 (*get_rx_timestamp)(struct emac_priv *priv);
};

struct emac_eth
{
    struct eth_device parent;

    int irq;
    void *iobase;
    rt_uint32_t apmu_base;

    struct rt_clk *mac_clk;
    struct rt_clk *phy_clk;
    struct rt_clk *ptp_clk;
    struct rt_reset_control *rstc;

    struct emac_desc_ring tx_ring, rx_ring;

    rt_uint32_t dma_buf_sz;
    rt_uint32_t wol;
    spinlock_t spStatsLock;
    struct work_struct tx_timeout_task;

    spinlock_t spTxLock;
    struct net_device *ndev;
    struct napi_struct napi;
    struct platform_device *pdev;

    void __iomem *iobase;


    int link;
    int duplex;
    int speed;
    phy_interface_t phy_interface;
    struct mii_bus *mii;
    struct phy_device *phy;
    struct emac_hw_stats *hw_stats;
    u8 tx_clk_phase;
    u8 rx_clk_phase;
    u8 clk_tuning_way;
    bool clk_tuning_enable;
    unsigned long state;
    rt_uint32_t tx_threshold;
    rt_uint32_t rx_threshold;
    rt_uint32_t tx_ring_num;
    rt_uint32_t rx_ring_num;
    rt_uint32_t dma_burst_len;
    rt_uint32_t ref_clk_frm_soc;
    void __iomem *ctrl_reg;
    void __iomem *dline_reg;
    s32 lpm_qos;
    rt_uint32_t tx_count_frames;
    rt_uint32_t tx_coal_frames;
    rt_uint32_t tx_coal_timeout;
    struct timer_list txtimer;
    struct ptp_clock *ptp_clock;
    struct ptp_clock_info ptp_clock_ops;
    spinlock_t ptp_lock;
    int ptp_support;
    rt_uint32_t ptp_clk_rate;
    int hwts_tx_en;
    int hwts_rx_en;
    struct emac_hw_ptp *hwptp;
    struct delayed_work systim_overflow_work;
    struct cyclecounter cc;
    struct timecounter tc;
};


rt_inline void emac_wr(struct emac_priv *priv, rt_uint32_t reg, rt_uint32_t val)
{
        writel(val, (priv->iobase + reg));
}

rt_inline int emac_rd(struct emac_priv *priv, rt_uint32_t reg)
{
        return readl(priv->iobase + reg);
}

enum clk_tuning_way {
        /* fpga clk tuning register */
        CLK_TUNING_BY_REG,
        /* zebu/evb rgmii delayline register */
        CLK_TUNING_BY_DLINE,
        /* evb rmii only revert tx/rx clock for clk tuning */
        CLK_TUNING_BY_CLK_REVERT,
        CLK_TUNING_MAX,
};

bool emac_is_rmii(struct emac_priv *priv)
{
        return priv->phy_interface == PHY_INTERFACE_MODE_RMII;
}

void emac_enable_axi_single_id_mode(struct emac_priv *priv, int en)
{
        rt_uint32_t val;

        val = readl(priv->ctrl_reg);
        if (en)
                val |= AXI_SINGLE_ID;
        else
                val &= ~AXI_SINGLE_ID;
        writel(val, priv->ctrl_reg);
}

void emac_phy_interface_config(struct emac_priv *priv)
{
        rt_uint32_t val;

        val = readl(priv->ctrl_reg);
        if (emac_is_rmii(priv)) {
                val &= ~PHY_INTF_RGMII;
                if (priv->ref_clk_frm_soc)
                        val |= REF_CLK_SEL;
                else
                        val &= ~REF_CLK_SEL;
        } else {
                val |= PHY_INTF_RGMII;
                if (priv->ref_clk_frm_soc)
                        val |= RGMII_TX_CLK_SEL;
        }
        writel(val, priv->ctrl_reg);
}

/* Name         emac_reset_hw
 * Arguments    priv : pointer to hardware data structure
 * Return       Status: 0 - Success;  non-zero - Fail
 * Description  TBDL
 */
int emac_reset_hw(struct emac_priv *priv)
{
    /* disable all the interrupts */
    emac_wr(priv, MAC_INTERRUPT_ENABLE, 0x0000);
    emac_wr(priv, DMA_INTERRUPT_ENABLE, 0x0000);

    /* disable transmit and receive units */
    emac_wr(priv, MAC_RECEIVE_CONTROL, 0x0000);
    emac_wr(priv, MAC_TRANSMIT_CONTROL, 0x0000);

    /* stop the DMA */
    emac_wr(priv, DMA_CONTROL, 0x0000);

    /* rstc mac, statistic counters */
    emac_wr(priv, MAC_GLOBAL_CONTROL, 0x0018);

    emac_wr(priv, MAC_GLOBAL_CONTROL, 0x0000);
    return 0;
}

/* Name         emac_init_hw
 * Arguments    pstHWData       : pointer to hardware data structure
 * Return       Status: 0 - Success;  non-zero - Fail
 * Description  TBDL
 * Assumes that the controller has previously been rstc
 * and is in apost-rstc uninitialized state.
 * Initializes the receive address registers,
 * multicast table, and VLAN filter table.
 * Calls routines to setup link
 * configuration and flow control settings.
 * Clears all on-chip counters. Leaves
 * the transmit and receive units disabled and uninitialized.
 */
int emac_init_hw(struct emac_priv *priv)
{
        rt_uint32_t val = 0;

        emac_enable_axi_single_id_mode(priv, 1);

        /* MAC Init
         * disable transmit and receive units
         */
        emac_wr(priv, MAC_RECEIVE_CONTROL, 0x0000);
        emac_wr(priv, MAC_TRANSMIT_CONTROL, 0x0000);

        /* enable mac address 1 filtering */
        emac_wr(priv, MAC_ADDRESS_CONTROL, MREGBIT_MAC_ADDRESS1_ENABLE);

        /* zero initialize the multicast hash table */
        emac_wr(priv, MAC_MULTICAST_HASH_TABLE1, 0x0);
        emac_wr(priv, MAC_MULTICAST_HASH_TABLE2, 0x0);
        emac_wr(priv, MAC_MULTICAST_HASH_TABLE3, 0x0);
        emac_wr(priv, MAC_MULTICAST_HASH_TABLE4, 0x0);

        emac_wr(priv, MAC_TRANSMIT_FIFO_ALMOST_FULL, 0x1f8);

        emac_wr(priv, MAC_TRANSMIT_PACKET_START_THRESHOLD, priv->tx_threshold);

        emac_wr(priv, MAC_RECEIVE_PACKET_START_THRESHOLD, priv->rx_threshold);

        /* set emac rx mitigation frame count */
        val = EMAC_RX_FRAMES & MREGBIT_RECEIVE_IRQ_FRAME_COUNTER_MSK;

        /* set emac rx mitigation timeout */
        val |= (EMAC_RX_COAL_TIMEOUT << MREGBIT_RECEIVE_IRQ_TIMEOUT_COUNTER_OFST) &
                MREGBIT_RECEIVE_IRQ_TIMEOUT_COUNTER_MSK;

        /* disable emac rx irq mitigation */
        val &= ~MRGEBIT_RECEIVE_IRQ_MITIGATION_ENABLE;

        emac_wr(priv, DMA_RECEIVE_IRQ_MITIGATION_CTRL, val);

        /* rstc dma */
        emac_wr(priv, DMA_CONTROL, 0x0000);

        emac_wr(priv, DMA_CONFIGURATION, 0x01);
        usleep_range(9000, 10000);
        emac_wr(priv, DMA_CONFIGURATION, 0x00);
        usleep_range(9000, 10000);

        val = 0;
        val |= MREGBIT_STRICT_BURST;
        val |= MREGBIT_DMA_64BIT_MODE;

        if (priv->dma_burst_len)
                val |= 1 << priv->dma_burst_len;
        else
                val |= MREGBIT_BURST_1WORD;

        emac_wr(priv, DMA_CONFIGURATION, val);

        /* if emac has ptp 1588 support, so enable PTP 1588 irq */
        if (priv->ptp_support)
                emac_wr(priv, PTP_1588_IRQ_EN, PTP_TX_TIMESTAMP|PTP_RX_TIMESTAMP);

        return 0;
}

int emac_set_mac_addr(struct emac_priv *priv, const unsigned char *addr)
{
        emac_wr(priv, MAC_ADDRESS1_HIGH, ((addr[1] << 8) | addr[0]));
        emac_wr(priv, MAC_ADDRESS1_MED, ((addr[3] << 8) | addr[2]));
        emac_wr(priv, MAC_ADDRESS1_LOW, ((addr[5] << 8) | addr[4]));

        return 0;
}

void emac_dma_start_transmit(struct emac_priv *priv)
{
        emac_wr(priv, DMA_TRANSMIT_POLL_DEMAND, 0xFF);
}

void emac_enable_interrupt(struct emac_priv *priv)
{
        rt_uint32_t val;
        val = emac_rd(priv, DMA_INTERRUPT_ENABLE);
        val |= MREGBIT_TRANSMIT_TRANSFER_DONE_INTR_ENABLE;
        val |= MREGBIT_RECEIVE_TRANSFER_DONE_INTR_ENABLE;
        emac_wr(priv, DMA_INTERRUPT_ENABLE, val);
}

void emac_disable_interrupt(struct emac_priv *priv)
{
        rt_uint32_t val;
        val = emac_rd(priv, DMA_INTERRUPT_ENABLE);
        val &= ~MREGBIT_TRANSMIT_TRANSFER_DONE_INTR_ENABLE;
        val &= ~MREGBIT_RECEIVE_TRANSFER_DONE_INTR_ENABLE;
        emac_wr(priv, DMA_INTERRUPT_ENABLE, val);
}

static inline rt_uint32_t emac_tx_avail(struct emac_priv *priv)
{
        struct emac_desc_ring *tx_ring = &priv->tx_ring;
        rt_uint32_t avail;

        if (tx_ring->tail > tx_ring->head)
                avail = tx_ring->tail - tx_ring->head - 1;
        else
                avail = tx_ring->total_cnt - tx_ring->head + tx_ring->tail - 1;

        return avail;
}

static void emac_tx_coal_timer_resched(struct emac_priv *priv)
{
        mod_timer(&priv->txtimer,
                  jiffies + usecs_to_jiffies(priv->tx_coal_timeout));
}

static void emac_tx_coal_timer(struct timer_list *t)
{
        struct emac_priv *priv = from_timer(priv, t, txtimer);

        if (likely(napi_schedule_prep(&priv->napi)))
                __napi_schedule(&priv->napi);
}

static int emac_tx_coal(struct emac_priv *priv, rt_uint32_t pkt_num)
{
        /* Manage tx mitigation */
        priv->tx_count_frames += pkt_num;
        if (likely(priv->tx_coal_frames > priv->tx_count_frames)) {
                emac_tx_coal_timer_resched(priv);
                return false;
        } else {
                priv->tx_count_frames = 0;
                return true;
        }
}

/* Name         emac_sw_init
 * Arguments    priv    : pointer to driver private data structure
 * Return       Status: 0 - Success;  non-zero - Fail
 * Description  Reads PCI space configuration information and
 *              initializes the variables with
 *              their default values
 */
static int emac_sw_init(struct emac_priv *priv)
{
        priv->dma_buf_sz = EMAC_DEFAULT_BUFSIZE;

        priv->tx_ring.total_cnt = priv->tx_ring_num;
        priv->rx_ring.total_cnt = priv->rx_ring_num;

        spin_lock_init(&priv->spStatsLock);
        spin_lock_init(&priv->spTxLock);
        spin_lock_init(&priv->hw_stats->stats_lock);

        INIT_WORK(&priv->tx_timeout_task, emac_tx_timeout_task);

        priv->tx_coal_frames = EMAC_TX_FRAMES;
        priv->tx_coal_timeout = EMAC_TX_COAL_TIMEOUT;

        timer_setup(&priv->txtimer, emac_tx_coal_timer, 0);

        return 0;
}

/* emac_get_tx_hwtstamp - get HW TX timestamps
 * @priv: driver private structure
 * @skb : the socket buffer
 * Description :
 * This function will read timestamp from the register & pass it to stack.
 * and also perform some sanity checks.
 */
static void emac_get_tx_hwtstamp(struct emac_priv *priv,
                                 struct sk_buff *skb)
{
        struct skb_shared_hwtstamps shhwtstamp;
        u64 ns;

        if (!priv->hwts_tx_en)
                return;

        /* exit if skb doesn't support hw tstamp */
        if (likely(!skb || !(skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS)))
                return;

        /* get the valid tstamp */
        ns = priv->hwptp->get_tx_timestamp(priv);

        memset(&shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
        shhwtstamp.hwtstamp = ns_to_ktime(ns);
        wmb();
        netdev_dbg(priv->ndev, "get valid TX hw timestamp %llu\n", ns);
        /* pass tstamp to stack */
        skb_tstamp_tx(skb, &shhwtstamp);

        return;
}

/* emac_get_rx_hwtstamp - get HW RX timestamps
 * @priv: driver private structure
 * @p : descriptor pointer
 * @skb : the socket buffer
 * Description :
 * This function will read received packet's timestamp from the descriptor
 * and pass it to stack. It also perform some sanity checks.
 */
static void emac_get_rx_hwtstamp(struct emac_priv *priv, struct emac_rx_desc *p,
                                 struct sk_buff *skb)
{
        struct skb_shared_hwtstamps *shhwtstamp = NULL;
        u64 ns;

        if (!priv->hwts_rx_en)
                return;

        /* Check if timestamp is available */
        if (p->ptp_pkt && p->rx_timestamp) {
                ns = priv->hwptp->get_rx_timestamp(priv);
                netdev_dbg(priv->ndev, "get valid RX hw timestamp %llu\n", ns);
                shhwtstamp = skb_hwtstamps(skb);
                memset(shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
                shhwtstamp->hwtstamp = ns_to_ktime(ns);
        } else  {
                netdev_dbg(priv->ndev, "cannot get RX hw timestamp\n");
        }
}

/**
 *  emac_hwtstamp_ioctl - control hardware timestamping.
 *  @dev: device pointer.
 *  @ifr: An IOCTL specific structure, that can contain a pointer to
 *  a proprietary structure used to pass information to the driver.
 *  Description:
 *  This function configures the MAC to enable/disable both outgoing(TX)
 *  and incoming(RX) packets time stamping based on user input.
 *  Return Value:
 *  0 on success and an appropriate -ve integer on failure.
 */
static int emac_hwtstamp_ioctl(struct net_device *dev, struct ifreq *ifr)
{
        struct emac_priv *priv = netdev_priv(dev);
        struct hwtstamp_config config;
        struct timespec64 now;
        u64 ns_ptp;
        rt_uint32_t ptp_event_msg_id = 0;
        rt_uint32_t rx_ptp_type = 0;

        if (!priv->ptp_support) {
                netdev_alert(priv->ndev, "No support for HW time stamping\n");
                priv->hwts_tx_en = 0;
                priv->hwts_rx_en = 0;

                return -EOPNOTSUPP;
        }

        if (copy_from_user(&config, ifr->ifr_data,
                           sizeof(struct hwtstamp_config)))
                return -EFAULT;

        netdev_dbg(priv->ndev, "%s config flags:0x%x, tx_type:0x%x, rx_filter:0x%x\n",
                   __func__, config.flags, config.tx_type, config.rx_filter);

        /* reserved for future extensions */
        if (config.flags)
                return -EINVAL;

        if (config.tx_type != HWTSTAMP_TX_OFF &&
            config.tx_type != HWTSTAMP_TX_ON)
                return -ERANGE;

        switch (config.rx_filter) {
        case HWTSTAMP_FILTER_NONE:
                /* time stamp no incoming packet at all */
                config.rx_filter = HWTSTAMP_FILTER_NONE;
                break;

        case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
                /* PTP v1, UDP, Sync packet */
                config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_SYNC;
                /* take time stamp for SYNC messages only */
                ptp_event_msg_id = MSG_SYNC;
                rx_ptp_type = PTP_V1_L4_ONLY;
                break;

        case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
                /* PTP v1, UDP, Delay_req packet */
                config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ;
                /* take time stamp for Delay_Req messages only */
                ptp_event_msg_id = MSG_DELAY_REQ;
                rx_ptp_type = PTP_V1_L4_ONLY;
                break;

        case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
                /* PTP v2, UDP, Sync packet */
                config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_SYNC;
                /* take time stamp for SYNC messages only */
                ptp_event_msg_id = MSG_SYNC;
                rx_ptp_type = PTP_V2_L2_L4;
                break;

        case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
                /* PTP v2, UDP, Delay_req packet */
                config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ;
                /* take time stamp for Delay_Req messages only */
                ptp_event_msg_id = MSG_DELAY_REQ;
                rx_ptp_type = PTP_V2_L2_L4;
                break;

        case HWTSTAMP_FILTER_PTP_V2_EVENT:
                /* PTP v2/802.AS1 any layer, any kind of event packet */
                config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
                ptp_event_msg_id = ALL_EVENTS;
                rx_ptp_type = PTP_V2_L2_L4;
                break;

        case HWTSTAMP_FILTER_PTP_V2_SYNC:
                /* PTP v2/802.AS1, any layer, Sync packet */
                config.rx_filter = HWTSTAMP_FILTER_PTP_V2_SYNC;
                /* take time stamp for SYNC messages only */
                ptp_event_msg_id = MSG_SYNC;
                rx_ptp_type = PTP_V2_L2_L4;
                break;

        case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
                /* PTP v2/802.AS1, any layer, Delay_req packet */
                config.rx_filter = HWTSTAMP_FILTER_PTP_V2_DELAY_REQ;
                /* take time stamp for Delay_Req messages only */
                ptp_event_msg_id = MSG_DELAY_REQ;
                rx_ptp_type = PTP_V2_L2_L4;
                break;
        default:
                return -ERANGE;
        }

        priv->hwts_rx_en = ((config.rx_filter == HWTSTAMP_FILTER_NONE) ? 0 : 1);
        priv->hwts_tx_en = config.tx_type == HWTSTAMP_TX_ON;

        if (!priv->hwts_tx_en && !priv->hwts_rx_en)
                priv->hwptp->config_hw_tstamping(priv, 0, 0, 0);
        else {

                priv->hwptp->config_hw_tstamping(priv, 1, rx_ptp_type, ptp_event_msg_id);

                /* initialize system time */
                ktime_get_real_ts64(&now);
                priv->hwptp->init_systime(priv, timespec64_to_ns(&now));

                /* program Increment reg */
                priv->hwptp->config_systime_increment(priv, priv->ptp_clk_rate, priv->ptp_clk_rate);

                ns_ptp = priv->hwptp->get_phc_time(priv);
                ktime_get_real_ts64(&now);
                /* check the diff between ptp timer and system time */
                if (abs(timespec64_to_ns(&now) - ns_ptp) > 5000)
                        priv->hwptp->init_systime(priv, timespec64_to_ns(&now));
        }
        return copy_to_user(ifr->ifr_data, &config,
                            sizeof(struct hwtstamp_config)) ? -EFAULT : 0;
}

/* Name         emac_ioctl
 * Arguments    pstNetdev : pointer to net_device structure
 *              pstIfReq : pointer to interface request structure used.
 *              u32Cmd : IOCTL command number
 * Return       Status: 0 - Success;  non-zero - Fail
 * Description  It is called by upper layer and
 *              handling various task IOCTL commands.
 */
static int emac_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
        int ret = -EOPNOTSUPP;

        if (!netif_running(ndev))
                return -EINVAL;

        switch (cmd) {
        case SIOCGMIIPHY:
        case SIOCGMIIREG:
        case SIOCSMIIREG:
                if (!ndev->phydev)
                        return -EINVAL;
                ret = phy_mii_ioctl(ndev->phydev, rq, cmd);
                break;
        case SIOCSHWTSTAMP:
                ret = emac_hwtstamp_ioctl(ndev, rq);
                break;
        default:
                break;
        }

        return ret;
}

/* Name         emac_interrupt_handler
 * Arguments    irq : irq number for which the interrupt is fired
 *              dev_id : pointer was passed to request_irq and same pointer is passed
 *              back to handler
 * Return       irqreturn_t : integer value
 * Description  Interrupt handler routine for interrupts from target for RX packets indication.
 */
static irqreturn_t emac_interrupt_handler(int irq, void *dev_id)
{
        struct net_device *ndev = (struct net_device *)dev_id;
        struct emac_priv *priv = netdev_priv(ndev);
        rt_uint32_t status;
        rt_uint32_t clr = 0;

        /* Check if emac is up */
        if (test_bit(EMAC_DOWN, &priv->state))
                return IRQ_HANDLED;

        /* read the status register for IRQ received */
        status = emac_rd(priv, DMA_STATUS_IRQ);

        if (status & MREGBIT_TRANSMIT_TRANSFER_DONE_IRQ) {
                emac_disable_interrupt(priv);
                clr |= MREGBIT_TRANSMIT_TRANSFER_DONE_IRQ;
                napi_schedule(&priv->napi);
        }

        if (status & MREGBIT_TRANSMIT_DES_UNAVAILABLE_IRQ)
                clr |= MREGBIT_TRANSMIT_DES_UNAVAILABLE_IRQ;

        if (status & MREGBIT_TRANSMIT_DMA_STOPPED_IRQ)
                clr |= MREGBIT_TRANSMIT_DMA_STOPPED_IRQ;

        if (status & MREGBIT_RECEIVE_TRANSFER_DONE_IRQ) {
                emac_disable_interrupt(priv);
                clr |= MREGBIT_RECEIVE_TRANSFER_DONE_IRQ;
                napi_schedule(&priv->napi);
        }

        if (status & MREGBIT_RECEIVE_DES_UNAVAILABLE_IRQ)
                clr |= MREGBIT_RECEIVE_DES_UNAVAILABLE_IRQ;

        if (status & MREGBIT_RECEIVE_DMA_STOPPED_IRQ)
                clr |= MREGBIT_RECEIVE_DMA_STOPPED_IRQ;

        if (status & MREGBIT_RECEIVE_MISSED_FRAME_IRQ)
                clr |= MREGBIT_RECEIVE_MISSED_FRAME_IRQ;

        emac_wr(priv, DMA_STATUS_IRQ, clr);

        if (priv->ptp_support) {
                status = emac_rd(priv, PTP_1588_IRQ_STS);
                if ((status & PTP_TX_TIMESTAMP) || (status & PTP_RX_TIMESTAMP))
                        napi_schedule(&priv->napi);

                emac_wr(priv, PTP_1588_IRQ_STS, status);
        }

        return IRQ_HANDLED;
}

/* Name         emac_configure_tx
 * Arguments    priv : pointer to driver private data structure
 * Return       none
 * Description  Configures the transmit unit of the device
 */
static void emac_configure_tx(struct emac_priv *priv)
{
        rt_uint32_t val;

        /* set the transmit base address */
        val = (rt_uint32_t)(priv->tx_ring.desc_dma_addr);

        emac_wr(priv, DMA_TRANSMIT_BASE_ADDRESS, val);

        /* Tx Inter Packet Gap value and enable the transmit */
        val = emac_rd(priv, MAC_TRANSMIT_CONTROL);
        val &= (~MREGBIT_IFG_LEN);
        val |= MREGBIT_TRANSMIT_ENABLE;
        val |= MREGBIT_TRANSMIT_AUTO_RETRY;
        emac_wr(priv, MAC_TRANSMIT_CONTROL, val);

        emac_wr(priv, DMA_TRANSMIT_AUTO_POLL_COUNTER, 0x00);

        /* start tx dma */
        val = emac_rd(priv, DMA_CONTROL);
        val |= MREGBIT_START_STOP_TRANSMIT_DMA;
        emac_wr(priv, DMA_CONTROL, val);
}

/* Name         emac_configure_rx
 * Arguments    priv : pointer to driver private data structure
 * Return       none
 * Description  Configures the receive unit of the device
 */
static void emac_configure_rx(struct emac_priv *priv)
{
        rt_uint32_t val;

        /* set the receive base address */
        val = (rt_uint32_t)(priv->rx_ring.desc_dma_addr);
        emac_wr(priv, DMA_RECEIVE_BASE_ADDRESS, val);

        /* enable the receive */
        val = emac_rd(priv, MAC_RECEIVE_CONTROL);
        val |= MREGBIT_RECEIVE_ENABLE;
        val |= MREGBIT_STORE_FORWARD;
        emac_wr(priv, MAC_RECEIVE_CONTROL, val);

        /* start rx dma */
        val = emac_rd(priv, DMA_CONTROL);
        val |= MREGBIT_START_STOP_RECEIVE_DMA;
        emac_wr(priv, DMA_CONTROL, val);
}

/* Name         emac_free_tx_buf
 * Arguments    priv : pointer to driver private data structure
 *              i: ring idx
 * Return       0 - Success;
 * Description  Freeing the TX buffer data.
 */
static int emac_free_tx_buf(struct emac_priv *priv, int i)
{
        struct emac_desc_ring *tx_ring;
        struct emac_tx_desc_buffer *tx_buf;
        struct desc_buf *buf;
        int j;

        tx_ring = &priv->tx_ring;
        tx_buf = &tx_ring->tx_desc_buf[i];

        for (j = 0; j < 2; j++) {
                buf = &tx_buf->buf[j];
                if (buf->dma_addr) {
                        if (buf->map_as_page)
                                dma_unmap_page(&priv->pdev->dev, buf->dma_addr,
                                               buf->dma_len, DMA_TO_DEVICE);
                        else
                                dma_unmap_single(&priv->pdev->dev, buf->dma_addr,
                                                 buf->dma_len, DMA_TO_DEVICE);

                        buf->dma_addr = 0;
                        buf->map_as_page = false;
                        buf->buff_addr = NULL;
                }
        }

        if (tx_buf->skb) {
                dev_kfree_skb_any(tx_buf->skb);
                tx_buf->skb = NULL;
        }
        return 0;
}

/* Name         emac_clean_tx_desc_ring
 * Arguments    priv : pointer to driver private data structure
 * Return       none
 * Description  Freeing the TX resources allocated earlier.
 */
static void emac_clean_tx_desc_ring(struct emac_priv *priv)
{
        struct emac_desc_ring *tx_ring = &priv->tx_ring;
        rt_uint32_t i;

        /* Free all the Tx ring sk_buffs */
        for (i = 0; i < tx_ring->total_cnt; i++)
                emac_free_tx_buf(priv, i);

        tx_ring->head = 0;
        tx_ring->tail = 0;
}

/* Name         emac_clean_rx_desc_ring
 * Arguments    priv : pointer to driver private data structure
 * Return       none
 * Description  Freeing the RX resources allocated earlier.
 */
static void emac_clean_rx_desc_ring(struct emac_priv *priv)
{
        struct emac_desc_ring *rx_ring;
        struct emac_desc_buffer *rx_buf;
        rt_uint32_t i;

        rx_ring = &priv->rx_ring;

        /* Free all the Rx ring sk_buffs */
        for (i = 0; i < rx_ring->total_cnt; i++) {
                rx_buf = &rx_ring->desc_buf[i];
                if (rx_buf->skb) {
                        dma_unmap_single(&priv->pdev->dev, rx_buf->dma_addr,
                                         rx_buf->dma_len, DMA_FROM_DEVICE);

                        dev_kfree_skb(rx_buf->skb);
                        rx_buf->skb = NULL;
                }
        }

        rx_ring->tail = 0;
        rx_ring->head = 0;
}

void emac_ptp_init(struct emac_priv *priv)
{
        int ret;

        if (priv->ptp_support) {
                ret = clk_prepare_enable(priv->ptp_clk);
                if (ret < 0) {
                        pr_warn("ptp clock failed to enable \n");
                        priv->ptp_clk = NULL;
                }
                emac_ptp_register(priv);
        }
}

void emac_ptp_deinit(struct emac_priv *priv)
{
        if (priv->ptp_support) {
                if (priv->ptp_clk)
                        clk_disable_unprepare(priv->ptp_clk);
                emac_ptp_unregister(priv);
        }
}

/* Name         emac_up
 * Arguments    priv : pointer to driver private data structure
 * Return       Status: 0 - Success;  non-zero - Fail
 * Description  This function is called from emac_open and
 *              performs the things when net interface is about to up.
 *              It configues the Tx and Rx unit of the device and
 *              registers interrupt handler.
 *              It also starts one watchdog timer to monitor
 *              the net interface link status.
 */
int emac_up(struct emac_priv *priv)
{
        struct net_device *ndev = priv->ndev;
        struct platform_device *pdev  = priv->pdev;
        int ret;
        rt_uint32_t val = 0;

        ret = emac_phy_connect(ndev);
        if (ret) {
                pr_err("%s  phy_connet failed\n", __func__);
                goto err;
        }
        /* init hardware */
        emac_init_hw(priv);

        emac_ptp_init(priv);

        emac_set_mac_addr(priv, ndev->dev_addr);
        /* configure transmit unit */
        emac_configure_tx(priv);
        /* configure rx unit */
        emac_configure_rx(priv);

        /* allocate buffers for receive descriptors */
        emac_alloc_rx_desc_buffers(priv);

        if (ndev->phydev)
                phy_start(ndev->phydev);

        /* allocates interrupt resources and
         * enables the interrupt line and IRQ handling
         */
        ret = request_irq(priv->irq, emac_interrupt_handler,
                          IRQF_SHARED, ndev->name, ndev);
        if (ret) {
                pr_err("request_irq failed\n");
                goto request_irq_failed;
        }

        /* enable mac interrupt */
        emac_wr(priv, MAC_INTERRUPT_ENABLE, 0x0000);

        val |= MREGBIT_TRANSMIT_TRANSFER_DONE_INTR_ENABLE;
        val |= MREGBIT_TRANSMIT_DMA_STOPPED_INTR_ENABLE;
        val |= MREGBIT_RECEIVE_TRANSFER_DONE_INTR_ENABLE;
        val |= MREGBIT_RECEIVE_DMA_STOPPED_INTR_ENABLE;
        val |= MREGBIT_RECEIVE_MISSED_FRAME_INTR_ENABLE;

        /* both rx tx */
        emac_wr(priv, DMA_INTERRUPT_ENABLE, val);

        napi_enable(&priv->napi);

        netif_start_queue(ndev);
        return 0;

request_irq_failed:
        emac_reset_hw(priv);
        if (ndev->phydev) {
                phy_stop(ndev->phydev);
                phy_disconnect(ndev->phydev);
        }
err:
        return ret;
}

/* Name         emac_down
 * Arguments    priv : pointer to driver private data structure
 * Return       Status: 0 - Success;  non-zero - Fail
 * Description  This function is called from emac_close and
 *              performs the things when net interface is about to down.
 *              It frees the irq, removes the various timers.
 *              It sets the net interface off and
 *              resets the hardware. Cleans the Tx and Rx
 *              ring descriptor.
 */
int emac_down(struct emac_priv *priv)
{
        struct net_device *ndev = priv->ndev;
        struct platform_device *pdev  = priv->pdev;

        netif_stop_queue(ndev);
        /* Stop and disconnect the PHY */
        if (ndev->phydev) {
                phy_stop(ndev->phydev);
                phy_disconnect(ndev->phydev);
        }

        priv->link = false;
        priv->duplex = DUPLEX_UNKNOWN;
        priv->speed = SPEED_UNKNOWN;

        napi_disable(&priv->napi);

        emac_wr(priv, MAC_INTERRUPT_ENABLE, 0x0000);
        emac_wr(priv, DMA_INTERRUPT_ENABLE, 0x0000);

        free_irq(priv->irq, ndev);

        emac_ptp_deinit(priv);

        emac_reset_hw(priv);
        netif_carrier_off(ndev);

        return 0;
}

/* Name         emac_alloc_tx_resources
 * Arguments    priv : pointer to driver private data structure
 * Return       Status: 0 - Success;  non-zero - Fail
 * Description  Allocates TX resources and getting virtual & physical address.
 */
int emac_alloc_tx_resources(struct emac_priv *priv)
{
        struct emac_desc_ring *tx_ring = &priv->tx_ring;
        struct platform_device *pdev  = priv->pdev;
        rt_uint32_t size;

        size = sizeof(struct emac_tx_desc_buffer) * tx_ring->total_cnt;

        /* allocate memory */
        tx_ring->tx_desc_buf = kzalloc(size, GFP_KERNEL);
        if (!tx_ring->tx_desc_buf) {
                pr_err("Memory allocation failed for the Transmit descriptor buffer\n");
                return -ENOMEM;
        }

        memset(tx_ring->tx_desc_buf, 0, size);

        tx_ring->total_size = tx_ring->total_cnt * sizeof(struct emac_tx_desc);

        EMAC_ROUNDUP(tx_ring->total_size, 1024);

        tx_ring->desc_addr = dma_alloc_coherent(&pdev->dev,
                                                        tx_ring->total_size,
                                                        &tx_ring->desc_dma_addr,
                                                        GFP_KERNEL);
        if (!tx_ring->desc_addr) {
                pr_err("Memory allocation failed for the Transmit descriptor ring\n");
                kfree(tx_ring->tx_desc_buf);
                return -ENOMEM;
        }

        memset(tx_ring->desc_addr, 0, tx_ring->total_size);

        tx_ring->head = 0;
        tx_ring->tail = 0;

        return 0;
}

/* Name         emac_alloc_rx_resources
 * Arguments    priv    : pointer to driver private data structure
 * Return       Status: 0 - Success;  non-zero - Fail
 * Description  Allocates RX resources and getting virtual & physical address.
 */
int emac_alloc_rx_resources(struct emac_priv *priv)
{
        struct emac_desc_ring *rx_ring = &priv->rx_ring;
        struct platform_device *pdev  = priv->pdev;
        rt_uint32_t buf_len;

        buf_len = sizeof(struct emac_desc_buffer) * rx_ring->total_cnt;

        rx_ring->desc_buf = kzalloc(buf_len, GFP_KERNEL);
        if (!rx_ring->desc_buf) {
                pr_err("Memory allocation failed for the Receive descriptor buffer\n");
                return -ENOMEM;
        }

        memset(rx_ring->desc_buf, 0, buf_len);

        /* round up to nearest 4K */
        rx_ring->total_size = rx_ring->total_cnt * sizeof(struct emac_rx_desc);

        EMAC_ROUNDUP(rx_ring->total_size, 1024);

        rx_ring->desc_addr = dma_alloc_coherent(&pdev->dev,
                                                        rx_ring->total_size,
                                                        &rx_ring->desc_dma_addr,
                                                        GFP_KERNEL);
        if (!rx_ring->desc_addr) {
                pr_err("Memory allocation failed for the Receive descriptor ring\n");
                kfree(rx_ring->desc_buf);
                return -ENOMEM;
        }

        memset(rx_ring->desc_addr, 0, rx_ring->total_size);

        rx_ring->head = 0;
        rx_ring->tail = 0;

        return 0;
}

/* Name         emac_free_tx_resources
 * Arguments    priv : pointer to driver private data structure
 * Return       none
 * Description  Frees the Tx resources allocated
 */
void emac_free_tx_resources(struct emac_priv *priv)
{
        emac_clean_tx_desc_ring(priv);
        kfree(priv->tx_ring.tx_desc_buf);
        priv->tx_ring.tx_desc_buf = NULL;
        dma_free_coherent(&priv->pdev->dev, priv->tx_ring.total_size,
                                priv->tx_ring.desc_addr,
                                priv->tx_ring.desc_dma_addr);
        priv->tx_ring.desc_addr = NULL;
}

/* Name         emac_free_rx_resources
 * Arguments    priv : pointer to driver private data structure
 * Return       none
 * Description  Frees the Rx resources allocated
 */
void emac_free_rx_resources(struct emac_priv *priv)
{
        emac_clean_rx_desc_ring(priv);
        kfree(priv->rx_ring.desc_buf);
        priv->rx_ring.desc_buf = NULL;
        dma_free_coherent(&priv->pdev->dev, priv->rx_ring.total_size,
                                priv->rx_ring.desc_addr,
                                priv->rx_ring.desc_dma_addr);
        priv->rx_ring.desc_addr = NULL;
}

/* Name         emac_open
 * Arguments    pstNetdev : pointer to net_device structure
 * Return       Status: 0 - Success;  non-zero - Fail
 * Description  This function is called when net interface is made up.
 *              Setting up Tx and Rx
 *              resources and making the interface up.
 */
static int emac_open(struct net_device *ndev)
{
        struct emac_priv *priv = netdev_priv(ndev);
        int ret;

        ret = emac_alloc_tx_resources(priv);
        if (ret) {
                pr_err("Error in setting up the Tx resources\n");
                goto emac_alloc_tx_resource_fail;
        }

        ret = emac_alloc_rx_resources(priv);
        if (ret) {
                pr_err("Error in setting up the Rx resources\n");
                goto emac_alloc_rx_resource_fail;
        }

        ret = emac_up(priv);
        if (ret) {
                pr_err("Error in making the net intrface up\n");
                goto emac_up_fail;
        }
        return 0;

emac_up_fail:
        emac_free_rx_resources(priv);
emac_alloc_rx_resource_fail:
        emac_free_tx_resources(priv);
emac_alloc_tx_resource_fail:
        return ret;
}

/* Name         emac_close
 * Arguments    pstNetdev : pointer to net_device structure
 * Return       Status: 0 - Success;  non-zero - Fail
 * Description  This function is called when net interface is made down.
 *              It calls the appropriate functions to
 *              free Tx and Rx resources.
 */
static int emac_close(struct net_device *ndev)
{
        struct emac_priv *priv = netdev_priv(ndev);

        emac_down(priv);
        emac_free_tx_resources(priv);
        emac_free_rx_resources(priv);

        return 0;
}

/* Name         emac_tx_clean_desc
 * Arguments    priv : pointer to driver private data structure
 * Return       1: Cleaned; 0:Failed
 * Description
 */
static int emac_tx_clean_desc(struct emac_priv *priv)
{
        struct emac_desc_ring *tx_ring;
        struct emac_tx_desc_buffer *tx_buf;
        struct emac_tx_desc *tx_desc;
        struct net_device *ndev = priv->ndev;
        rt_uint32_t i;

        netif_tx_lock(ndev);

        tx_ring = &priv->tx_ring;

        i = tx_ring->tail;

        while (i != tx_ring->head) {
                tx_desc = &((struct emac_tx_desc *)tx_ring->desc_addr)[i];

                /* if desc still own by dma, so we quit it */
                if (tx_desc->OWN)
                        break;

                tx_buf = &tx_ring->tx_desc_buf[i];

                if (tx_buf->timestamped && tx_buf->skb) {
                        emac_get_tx_hwtstamp(priv, tx_buf->skb);
                        tx_buf->timestamped = 0;
                }

                emac_free_tx_buf(priv, i);
                memset(tx_desc, 0, sizeof(struct emac_tx_desc));

                if (++i == tx_ring->total_cnt)
                        i = 0;
        }

        tx_ring->tail = i;

        if (unlikely(netif_queue_stopped(ndev) &&
                     emac_tx_avail(priv) > tx_ring->total_cnt / 4))
                netif_wake_queue(ndev);

        netif_tx_unlock(ndev);

        return 0;
}

static int emac_rx_frame_status(struct emac_priv *priv, struct emac_rx_desc *dsc)
{
        /* if last descritpor isn't set, so we drop it*/
        if (!dsc->LastDescriptor) {
                netdev_dbg(priv->ndev, "rx LD bit isn't set, drop it.\n");
                return frame_discard;
        }

        /*
         * A Frame that is less than 64-bytes (from DA thru the FCS field)
         * is considered as Runt Frame.
         * Most of the Runt Frames happen because of collisions.
         */
        if (dsc->ApplicationStatus & EMAC_RX_FRAME_RUNT) {
                netdev_dbg(priv->ndev, "rx frame less than 64.\n");
                return frame_discard;
        }

        /*
         * When the frame fails the CRC check,
         * the frame is assumed to have the CRC error
         */
        if (dsc->ApplicationStatus & EMAC_RX_FRAME_CRC_ERR) {
                netdev_dbg(priv->ndev, "rx frame crc error\n");
                return frame_discard;
        }

        /*
         * When the length of the frame exceeds
         * the Programmed Max Frame Length
         */
        if (dsc->ApplicationStatus & EMAC_RX_FRAME_MAX_LEN_ERR) {
                netdev_dbg(priv->ndev, "rx frame too long\n");
                return frame_discard;
        }

        /*
         * frame reception is truncated at that point and
         * frame is considered to have Jabber Error
         */
        if (dsc->ApplicationStatus & EMAC_RX_FRAME_JABBER_ERR) {
                netdev_dbg(priv->ndev, "rx frame has been truncated\n");
                return frame_discard;
        }

        /* this bit is only for 802.3 Type Frames */
        if (dsc->ApplicationStatus & EMAC_RX_FRAME_LENGTH_ERR) {
                netdev_dbg(priv->ndev, "rx frame length err for 802.3\n");
                return frame_discard;
        }

        if (dsc->FramePacketLength <= ETHERNET_FCS_SIZE ||
            dsc->FramePacketLength > priv->dma_buf_sz) {
                netdev_dbg(priv->ndev, "rx frame len too small or too long\n");
                return frame_discard;
        }
        return frame_ok;
}

/* Name         emac_rx_clean_desc
 * Arguments    priv : pointer to driver private data structure
 * Return       1: Cleaned; 0:Failed
 * Description
 */
static int emac_rx_clean_desc(struct emac_priv *priv, int budget)
{
        struct emac_desc_ring *rx_ring;
        struct emac_desc_buffer *rx_buf;
        struct net_device *ndev = priv->ndev;
        struct emac_rx_desc *rx_desc;
        struct sk_buff *skb = NULL;
        int status;
        rt_uint32_t receive_packet = 0;
        rt_uint32_t i;
        rt_uint32_t skb_len;

        rx_ring = &priv->rx_ring;

        i = rx_ring->tail;

        while (budget--) {
                /* get rx desc */
                rx_desc = &((struct emac_rx_desc *)rx_ring->desc_addr)[i];

                /* if rx_desc still owned by DMA, so we need to wait */
                if (rx_desc->OWN)
                        break;

                rx_buf = &rx_ring->desc_buf[i];

                if (!rx_buf->skb)
                        break;

                receive_packet++;

                dma_unmap_single(&priv->pdev->dev, rx_buf->dma_addr,
                                 rx_buf->dma_len, DMA_FROM_DEVICE);

                status = emac_rx_frame_status(priv, rx_desc);
                if (unlikely(status == frame_discard)) {
                        ndev->stats.rx_dropped++;
                        dev_kfree_skb_irq(rx_buf->skb);
                        rx_buf->skb = NULL;
                } else {
                        skb = rx_buf->skb;
                        skb_len = rx_desc->FramePacketLength - ETHERNET_FCS_SIZE;
                        skb_put(skb, skb_len);
                        skb->dev = ndev;
                        ndev->hard_header_len = ETH_HLEN;

                        emac_get_rx_hwtstamp(priv, rx_desc, skb);

                        skb->protocol = eth_type_trans(skb, ndev);

                        skb->ip_summed = CHECKSUM_NONE;

                        napi_gro_receive(&priv->napi, skb);

                        ndev->stats.rx_packets++;
                        ndev->stats.rx_bytes += skb_len;

                        memset(rx_desc, 0, sizeof(struct emac_rx_desc));
                        rx_buf->skb = NULL;
                }

                if (++i == rx_ring->total_cnt)
                        i = 0;
        }

        rx_ring->tail = i;

        emac_alloc_rx_desc_buffers(priv);

        return receive_packet;
}

/* Name         emac_alloc_rx_desc_buffers
 * Arguments    priv : pointer to driver private data structure
 * Return       1: Cleaned; 0:Failed
 * Description
 */
static void emac_alloc_rx_desc_buffers(struct emac_priv *priv)
{
        struct net_device *ndev = priv->ndev;
        struct emac_desc_ring *rx_ring = &priv->rx_ring;
        struct emac_desc_buffer *rx_buf;
        struct sk_buff *skb;
        struct emac_rx_desc *rx_desc;
        rt_uint32_t i;

        i = rx_ring->head;
        rx_buf = &rx_ring->desc_buf[i];

        while (!rx_buf->skb) {
                skb = netdev_alloc_skb_ip_align(ndev, priv->dma_buf_sz);
                if (!skb) {
                        pr_err("sk_buff allocation failed\n");
                        break;
                }

                skb->dev = ndev;

                rx_buf->skb = skb;
                rx_buf->dma_len = priv->dma_buf_sz;
                rx_buf->dma_addr = dma_map_single(&priv->pdev->dev,
                                                  skb->data,
                                                  priv->dma_buf_sz,
                                                  DMA_FROM_DEVICE);
                if (dma_mapping_error(&priv->pdev->dev, rx_buf->dma_addr)) {
                        netdev_err(ndev, "dma mapping_error\n");
                        goto dma_map_err;
                }

                rx_desc = &((struct emac_rx_desc *)rx_ring->desc_addr)[i];

                memset(rx_desc, 0, sizeof(struct emac_rx_desc));

                rx_desc->BufferAddr1 = rx_buf->dma_addr;
                rx_desc->BufferSize1 = rx_buf->dma_len;

                rx_desc->FirstDescriptor = 0;
                rx_desc->LastDescriptor = 0;
                if (++i == rx_ring->total_cnt) {
                        rx_desc->EndRing = 1;
                        i = 0;
                }
                dma_wmb();
                rx_desc->OWN = 1;

                rx_buf = &rx_ring->desc_buf[i];
        }
        rx_ring->head = i;
        return;
dma_map_err:
        dev_kfree_skb_any(skb);
        rx_buf->skb = NULL;
        return;
}

static int emac_rx_poll(struct napi_struct *napi, int budget)
{
        struct emac_priv *priv =
                                container_of(napi, struct emac_priv, napi);
        int work_done;

        emac_tx_clean_desc(priv);

        work_done = emac_rx_clean_desc(priv, budget);
        if (work_done < budget) {
                napi_complete(napi);
                emac_enable_interrupt(priv);
        }

        return work_done;
}

/* Name         emac_tx_mem_map
 * Arguments    priv : pointer to driver private data structure
 *              pstSkb : pointer to sk_buff structure passed by upper layer
 *              max_tx_len : max data len per descriptor
 *              frag_num : number of fragments in the packet
 * Return       number of descriptors needed for transmitting packet
 * Description
 */
static int emac_tx_mem_map(struct emac_priv *priv, struct sk_buff *skb,
                           rt_uint32_t max_tx_len, rt_uint32_t frag_num)
{
        struct emac_desc_ring *tx_ring;
        struct emac_tx_desc_buffer *tx_buf;
        struct emac_tx_desc *tx_desc;
        rt_uint32_t skb_linear_len = skb_headlen(skb);
        rt_uint32_t len, i, f, first, buf_idx = 0;
        phys_addr_t addr;
        u8 do_tx_timestamp = 0;

        tx_ring = &priv->tx_ring;

        i = tx_ring->head;
        first = i;

        skb_tx_timestamp(skb);

        if (unlikely((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
                                priv->hwts_tx_en)) {
                /* declare that device is doing timestamping */
                skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
                do_tx_timestamp = 1;
        }


        if (++i == tx_ring->total_cnt)
                i = 0;

        /* if the data is fragmented */
        for (f = 0; f < frag_num; f++) {
                const skb_frag_t *frag = &skb_shinfo(skb)->frags[f];

                len = skb_frag_size(frag);

                buf_idx = (f + 1) % 2;

                /* first frag fill into second buffer of first descriptor */
                if (f == 0) {
                        tx_buf = &tx_ring->tx_desc_buf[first];
                        tx_desc = &((struct emac_tx_desc *)tx_ring->desc_addr)[first];
                } else {
                        /* from second frags to more frags,
                         * we only get new descriptor when it frag num is odd.
                         */
                        if (!buf_idx) {
                                tx_buf = &tx_ring->tx_desc_buf[i];
                                tx_desc = &((struct emac_tx_desc *)tx_ring->desc_addr)[i];
                        }
                }
                tx_buf->buf[buf_idx].dma_len = len;

                addr = skb_frag_dma_map(&priv->pdev->dev, frag, 0,
                                       skb_frag_size(frag),
                                       DMA_TO_DEVICE);

                if (dma_mapping_error(&priv->pdev->dev, addr)) {
                        netdev_err(priv->ndev, "%s dma map page:%d error \n",
                                           __func__, f);
                        goto dma_map_err;
                }
                tx_buf->buf[buf_idx].dma_addr = addr;

                tx_buf->buf[buf_idx].map_as_page = true;

                if (do_tx_timestamp)
                        tx_buf->timestamped = 1;

                /*
                 * every desc has two buffer for packet
                 */

                if (buf_idx) {
                        tx_desc->BufferAddr2 = addr;
                        tx_desc->BufferSize2 = len;
                } else {
                        tx_desc->BufferAddr1 = addr;
                        tx_desc->BufferSize1 = len;

                        if (++i == tx_ring->total_cnt) {
                                tx_desc->EndRing = 1;
                                i = 0;
                        }
                }

                /*
                 * if frag num equal 1, we don't set tx_desc except buffer addr & size
                 */
                if (f > 0) {
                        if (f == (frag_num - 1)) {
                                tx_desc->LastSegment = 1;
                                tx_buf->skb = skb;
                                if (emac_tx_coal(priv, frag_num + 1))
                                        tx_desc->InterruptOnCompletion = 1;
                        }

                        tx_desc->OWN = 1;
                }
        }

        /* fill out first descriptor for skb linear data */
        tx_buf = &tx_ring->tx_desc_buf[first];

        tx_buf->buf[0].dma_len = skb_linear_len;

        addr = dma_map_single(&priv->pdev->dev, skb->data,
                              skb_linear_len, DMA_TO_DEVICE);
        if (dma_mapping_error(&priv->pdev->dev, addr)) {
                netdev_err(priv->ndev, "%s dma mapping_error\n", __func__);
                goto dma_map_err;
        }

        tx_buf->buf[0].dma_addr = addr;

        tx_buf->buf[0].buff_addr = skb->data;
        tx_buf->buf[0].map_as_page = false;

        /* fill tx descriptor */
        tx_desc = &((struct emac_tx_desc *)tx_ring->desc_addr)[first];
        tx_desc->BufferAddr1 = addr;
        tx_desc->BufferSize1 = skb_linear_len;
        tx_desc->FirstSegment = 1;

        /* if last desc for ring, need to end ring flag */
        if (first == (tx_ring->total_cnt - 1)) {
                tx_desc->EndRing = 1;
        }

        /*
         * if frag num more than 1, that means data need another desc
         * so current descriptor isn't last piece of packet data.
         */
        tx_desc->LastSegment = frag_num > 1 ? 0 : 1;
        if ((frag_num <= 1) && emac_tx_coal(priv, 1))
                tx_desc->InterruptOnCompletion = 1;

        if (do_tx_timestamp) {
                tx_desc->tx_timestamp = 1;
                tx_buf->timestamped = 1;
        }

        /* only last descriptor had skb pointer */
        if (tx_desc->LastSegment)
                tx_buf->skb = skb;

        tx_desc->OWN = 1;

        dma_wmb();

        emac_dma_start_transmit(priv);

        /* update tx ring head */
        tx_ring->head = i;

        return 0;
dma_map_err:
        dev_kfree_skb_any(skb);
        priv->ndev->stats.tx_dropped++;
        return 0;
}

/* Name         emac_start_xmit
 * Arguments    pstSkb : pointer to sk_buff structure passed by upper layer
 *              pstNetdev : pointer to net_device structure
 * Return       Status: 0 - Success;  non-zero - Fail
 * Description  This function is called by upper layer to
 *              handover the Tx packet to the driver
 *              for sending it to the device.
 *              Currently this is doing nothing but
 *              simply to simulate the tx packet handling.
 */
static int emac_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
        struct emac_priv *priv = netdev_priv(ndev);
        int nfrags = skb_shinfo(skb)->nr_frags;

        if (unlikely(emac_tx_avail(priv) < nfrags + 1)) {
                if (!netif_queue_stopped(ndev)) {
                        netif_stop_queue(ndev);
                        pr_err_ratelimited("tx ring full, stop tx queue\n");
                }
                return NETDEV_TX_BUSY;
        }

        emac_tx_mem_map(priv, skb, MAX_DATA_LEN_TX_DES, nfrags);

        ndev->stats.tx_packets++;
        ndev->stats.tx_bytes += skb->len;

        /* Make sure there is space in the ring for the next send. */
        if (unlikely(emac_tx_avail(priv) <= (MAX_SKB_FRAGS + 2)))
                netif_stop_queue(ndev);

        return NETDEV_TX_OK;
}

rt_uint32_t ReadTxStatCounters(struct emac_priv *priv, u8 cnt)
{
        rt_uint32_t val, tmp;

        val = 0x8000 | cnt;
        emac_wr(priv, MAC_TX_STATCTR_CONTROL, val);
        val = emac_rd(priv, MAC_TX_STATCTR_CONTROL);

        if (readl_poll_timeout_atomic(priv->iobase + MAC_TX_STATCTR_CONTROL,
                               val, !(val & 0x8000), 100, 10000)) {
                pr_err("%s timeout!!\n", __func__);
                return -EINVAL;
        }

        tmp = emac_rd(priv, MAC_TX_STATCTR_DATA_HIGH);
        val = tmp << 16;
        tmp = emac_rd(priv, MAC_TX_STATCTR_DATA_LOW);
        val |= tmp;

        return val;
}

rt_uint32_t ReadRxStatCounters(struct emac_priv *priv, u8 cnt)
{
        rt_uint32_t val, tmp;

        val = 0x8000 | cnt;
        emac_wr(priv, MAC_RX_STATCTR_CONTROL, val);
        val = emac_rd(priv, MAC_RX_STATCTR_CONTROL);

        if (readl_poll_timeout_atomic(priv->iobase + MAC_RX_STATCTR_CONTROL,
                               val, !(val & 0x8000), 100, 10000)) {
                pr_err("%s timeout!!\n", __func__);
                return -EINVAL;
        }

        tmp = emac_rd(priv, MAC_RX_STATCTR_DATA_HIGH);
        val = tmp << 16;
        tmp = emac_rd(priv, MAC_RX_STATCTR_DATA_LOW);
        val |= tmp;
        return val;
}

/* Name         emac_set_mac_address
 * Arguments    pstNetdev       : pointer to net_device structure
 *              addr : pointer to addr
 * Return       Status: 0 - Success;  non-zero - Fail
 * Description  It is called by upper layer to set the mac address.
 */
static int emac_set_mac_address(struct net_device *ndev, void *addr)
{
        struct emac_priv *priv = netdev_priv(ndev);
        int ret = 0;


        ret = eth_mac_addr(ndev, addr);
        if (ret)
                goto set_mac_error;

        /*
         * if nic not running, we just save addr
         * it will be set during device_open;
         * otherwise directly change hw mac setting.
         */

        if (netif_running(ndev))
                emac_set_mac_addr(priv, ndev->dev_addr);

set_mac_error:

        return ret;
}

void emac_mac_multicast_filter_clear(struct emac_priv *priv)
{
        emac_wr(priv, MAC_MULTICAST_HASH_TABLE1, 0x0);
        emac_wr(priv, MAC_MULTICAST_HASH_TABLE2, 0x0);
        emac_wr(priv, MAC_MULTICAST_HASH_TABLE3, 0x0);
        emac_wr(priv, MAC_MULTICAST_HASH_TABLE4, 0x0);
}

/* Configure Multicast and Promiscuous modes */
static void emac_rx_mode_set(struct net_device *ndev)
{
        struct emac_priv *priv = netdev_priv(ndev);
        struct netdev_hw_addr *ha;
        rt_uint32_t mc_filter[4] = {0};
        rt_uint32_t val;
        rt_uint32_t crc32, bit, reg, hash;

        val = emac_rd(priv, MAC_ADDRESS_CONTROL);

        val &= ~MREGBIT_PROMISCUOUS_MODE;

        if (ndev->flags & IFF_PROMISC) {
                /* enable promisc mode */
                val |= MREGBIT_PROMISCUOUS_MODE;
        } else if ((ndev->flags & IFF_ALLMULTI) ||
                        (netdev_mc_count(ndev) > HASH_TABLE_SIZE)) {
                /* Pass all multi */
                /* Set the 64 bits of the HASH tab. To be updated if taller
                 * hash table is used
                 */
                emac_wr(priv, MAC_MULTICAST_HASH_TABLE1, 0xffff);
                emac_wr(priv, MAC_MULTICAST_HASH_TABLE2, 0xffff);
                emac_wr(priv, MAC_MULTICAST_HASH_TABLE3, 0xffff);
                emac_wr(priv, MAC_MULTICAST_HASH_TABLE4, 0xffff);
        } else if (!netdev_mc_empty(ndev)) {
                emac_mac_multicast_filter_clear(priv);
                netdev_for_each_mc_addr(ha, ndev) {

                        /* Calculate the CRC of the MAC address */
                        crc32 = ether_crc(ETH_ALEN, ha->addr);

                        /*
                         * The HASH Table is an array of 4 16-bit registers. It is
                         * treated like an array of 64 bits (BitArray[hash_value]).
                         * Use the upper 6 bits of the above CRC as the hash value.
                         */
                        hash = (crc32 >> 26) & 0x3F;
                        reg = hash / 16;
                        bit = hash % 16;
                        mc_filter[reg] |= BIT(bit);
                        pr_debug("%s %pM crc32:0x%x hash:0x%x reg:%d bit:%d\n",
                                 __func__,ha->addr, crc32, hash, reg, bit);
                }
                emac_wr(priv, MAC_MULTICAST_HASH_TABLE1, mc_filter[0]);
                emac_wr(priv, MAC_MULTICAST_HASH_TABLE2, mc_filter[1]);
                emac_wr(priv, MAC_MULTICAST_HASH_TABLE3, mc_filter[2]);
                emac_wr(priv, MAC_MULTICAST_HASH_TABLE4, mc_filter[3]);
        }
        emac_wr(priv, MAC_ADDRESS_CONTROL, val);
        return;
}

/* Name         emac_change_mtu
 * Arguments    pstNetdev : pointer to net_device structure
 *              u32MTU  : maximum transmit unit value
 *              Return          Status: 0 - Success;  non-zero - Fail
 * Description  It is called by upper layer to set the MTU value.
 */
static int emac_change_mtu(struct net_device *ndev, int mtu)
{
        struct emac_priv *priv = netdev_priv(ndev);
        rt_uint32_t frame_len;

        if (netif_running(ndev)) {
                pr_err("must be stopped to change its MTU\n");
                return -EBUSY;
        }

        frame_len = mtu + ETHERNET_HEADER_SIZE + ETHERNET_FCS_SIZE;

        if (frame_len < MINIMUM_ETHERNET_FRAME_SIZE ||
                frame_len > EMAC_RX_BUF_4K) {
                        pr_err("Invalid MTU setting\n");
                        return -EINVAL;
        }

        if (frame_len <= EMAC_DEFAULT_BUFSIZE)
                priv->dma_buf_sz = EMAC_DEFAULT_BUFSIZE;
        else if (frame_len <= EMAC_RX_BUF_2K)
                priv->dma_buf_sz = EMAC_RX_BUF_2K;
        else
                priv->dma_buf_sz = EMAC_RX_BUF_4K;

        ndev->mtu = mtu;

        return 0;
}

static void emac_reset(struct emac_priv *priv)
{
        if (!test_and_clear_bit(EMAC_RESET_REQUESTED, &priv->state))
                return;
        if (test_bit(EMAC_DOWN, &priv->state))
                return;

        netdev_err(priv->ndev, "Reset controller.\n");

        rtnl_lock();
        netif_trans_update(priv->ndev);
        while (test_and_set_bit(EMAC_RESETING, &priv->state))
                usleep_range(1000, 2000);

        set_bit(EMAC_DOWN, &priv->state);
        dev_close(priv->ndev);
        dev_open(priv->ndev, NULL);
        clear_bit(EMAC_DOWN, &priv->state);
        clear_bit(EMAC_RESETING, &priv->state);
        rtnl_unlock();
}

static void emac_tx_timeout_task(struct work_struct *work)
{
        struct emac_priv *priv = container_of(work,
                                              struct emac_priv, tx_timeout_task);
        emac_reset(priv);
        clear_bit(EMAC_TASK_SCHED, &priv->state);
}

/* Name         emac_tx_timeout
 * Arguments    pstNetdev : pointer to net_device structure
 * Return       none
 * Description  It is called by upper layer
 *              for packet transmit timeout.
 */
static void emac_tx_timeout(struct net_device *ndev, unsigned int txqueue)
{
        struct emac_priv *priv = netdev_priv(ndev);

        netdev_info(ndev, "TX timeout\n");

        netif_carrier_off(priv->ndev);
        set_bit(EMAC_RESET_REQUESTED, &priv->state);

        if (!test_bit(EMAC_DOWN, &priv->state) &&
            !test_and_set_bit(EMAC_TASK_SCHED, &priv->state))
                schedule_work(&priv->tx_timeout_task);
}

static int clk_phase_rgmii_set(struct emac_priv *priv, bool is_tx)
{
        rt_uint32_t val;

        switch (priv->clk_tuning_way) {
        case CLK_TUNING_BY_REG:
                val = readl(priv->ctrl_reg);
                if (is_tx) {
                        val &= ~RGMII_TX_PHASE_MASK;
                        val |= (priv->tx_clk_phase & 0x7) << RGMII_TX_PHASE_OFFSET;
                } else {
                        val &= ~RGMII_RX_PHASE_MASK;
                        val |= (priv->rx_clk_phase & 0x7) << RGMII_RX_PHASE_OFFSET;
                }
                writel(val, priv->ctrl_reg);
                break;
        case CLK_TUNING_BY_DLINE:
                val = readl(priv->dline_reg);
                if (is_tx) {
                        val &= ~EMAC_TX_DLINE_CODE_MASK;
                        val |= priv->tx_clk_phase << EMAC_TX_DLINE_CODE_OFFSET;
                        val |= EMAC_TX_DLINE_EN;
                } else {
                        val &= ~EMAC_RX_DLINE_CODE_MASK;
                        val |= priv->rx_clk_phase << EMAC_RX_DLINE_CODE_OFFSET;
                        val |= EMAC_RX_DLINE_EN;
                }
                writel(val, priv->dline_reg);
                break;
        default:
                pr_err("wrong clk tuning way:%d !!\n", priv->clk_tuning_way);
                return -1;
        }
        pr_debug("%s tx phase:%d rx phase:%d\n",
                __func__, priv->tx_clk_phase, priv->rx_clk_phase);
        return 0;
}

static int clk_phase_rmii_set(struct emac_priv *priv, bool is_tx)
{
        rt_uint32_t val;

        switch (priv->clk_tuning_way) {
        case CLK_TUNING_BY_REG:
                val = readl(priv->ctrl_reg);
                if (is_tx) {
                        val &= ~RMII_TX_PHASE_MASK;
                        val |= (priv->tx_clk_phase & 0x7) << RMII_TX_PHASE_OFFSET;
                } else {
                        val &= ~RMII_RX_PHASE_MASK;
                        val |= (priv->rx_clk_phase & 0x7) << RMII_RX_PHASE_OFFSET;
                }
                writel(val, priv->ctrl_reg);
                break;
        case CLK_TUNING_BY_CLK_REVERT:
                val = readl(priv->ctrl_reg);
                if (is_tx) {
                        if (priv->tx_clk_phase == CLK_PHASE_REVERT)
                                val |= RMII_TX_CLK_SEL;
                        else
                                val &= ~RMII_TX_CLK_SEL;
                } else {
                        if (priv->rx_clk_phase == CLK_PHASE_REVERT)
                                val |= RMII_RX_CLK_SEL;
                        else
                                val &= ~RMII_RX_CLK_SEL;
                }
                writel(val, priv->ctrl_reg);
                break;
        default:
                pr_err("wrong clk tuning way:%d !!\n", priv->clk_tuning_way);
                return -1;
        }
        pr_debug("%s tx phase:%d rx phase:%d\n",
                __func__, priv->tx_clk_phase, priv->rx_clk_phase);
        return 0;
}

static int clk_phase_set(struct emac_priv *priv, bool is_tx)
{
        if (priv->clk_tuning_enable) {
                if (emac_is_rmii(priv)) {
                        clk_phase_rmii_set(priv, is_tx);
                } else {
                        clk_phase_rgmii_set(priv, is_tx);
                }
        }

        return 0;
}

static int emac_mii_reset(struct mii_bus *bus)
{
        struct emac_priv *priv = bus->priv;
        struct device *dev = &priv->pdev->dev;
        int rst_gpio, ldo_gpio;
        int active_state;
        rt_uint32_t delays[3] = {0};

        if (dev->of_node) {
                struct device_node *np = dev->of_node;

                if (!np)
                        return 0;

                ldo_gpio = of_get_named_gpio(np, "emac,ldo-gpio", 0);
                if (ldo_gpio >= 0) {
                        if (gpio_request(ldo_gpio, "mdio-ldo"))
                                return 0;

                        gpio_direction_output(ldo_gpio, 1);
                }

                rst_gpio = of_get_named_gpio(np, "emac,rstc-gpio", 0);
                if (rst_gpio < 0)
                        return 0;

                active_state = of_property_read_bool(np,
                                                     "emac,rstc-active-low");
                of_property_read_u32_array(np,
                                           "emac,rstc-delays-us", delays, 3);

                if (gpio_request(rst_gpio, "mdio-rstc"))
                        return 0;

                gpio_direction_output(rst_gpio,
                                      active_state ? 1 : 0);
                if (delays[0])
                        msleep(DIV_ROUND_UP(delays[0], 1000));

                gpio_set_value(rst_gpio, active_state ? 0 : 1);
                if (delays[1])
                        msleep(DIV_ROUND_UP(delays[1], 1000));

                gpio_set_value(rst_gpio, active_state ? 1 : 0);
                if (delays[2])
                        msleep(DIV_ROUND_UP(delays[2], 1000));
        }
        return 0;
}

static int emac_mii_read(struct mii_bus *bus, int phy_addr, int regnum)
{
        struct emac_priv *priv = bus->priv;
        rt_uint32_t cmd = 0;
        rt_uint32_t val;

        cmd |= phy_addr & 0x1F;
        cmd |= (regnum & 0x1F) << 5;
        cmd |= MREGBIT_START_MDIO_TRANS | MREGBIT_MDIO_READ_WRITE;

        emac_wr(priv, MAC_MDIO_DATA, 0x0);
        emac_wr(priv, MAC_MDIO_CONTROL, cmd);

        if (readl_poll_timeout(priv->iobase + MAC_MDIO_CONTROL,
                               val, !((val >> 15) & 0x1), 100, 10000))
                return -EBUSY;

        val = emac_rd(priv, MAC_MDIO_DATA);
        return val;
}

static int emac_mii_write(struct mii_bus *bus, int phy_addr, int regnum,
                            u16 value)
{
        struct emac_priv *priv = bus->priv;
        rt_uint32_t cmd = 0;
        rt_uint32_t val;

        emac_wr(priv, MAC_MDIO_DATA, value);

        cmd |= phy_addr & 0x1F;
        cmd |= (regnum & 0x1F) << 5;
        cmd |= MREGBIT_START_MDIO_TRANS;

        emac_wr(priv, MAC_MDIO_CONTROL, cmd);

        if (readl_poll_timeout(priv->iobase + MAC_MDIO_CONTROL,
                               val, !((val >> 15) & 0x1), 100, 10000))
                return -EBUSY;

        return 0;
}

static void emac_adjust_link(struct net_device *dev)
{
        struct phy_device *phydev = dev->phydev;
        struct emac_priv *priv = netdev_priv(dev);
        bool link_changed = false;
        rt_uint32_t ctrl;

        if (!phydev)
                return;

        if (phydev->link) {
                ctrl = emac_rd(priv, MAC_GLOBAL_CONTROL);

                /* Now we make sure that we can be in full duplex mode
                 * If not, we operate in half-duplex mode.
                 */
                if (phydev->duplex != priv->duplex) {
                        link_changed = true;

                        if (!phydev->duplex)
                                ctrl &= ~MREGBIT_FULL_DUPLEX_MODE;
                        else
                                ctrl |= MREGBIT_FULL_DUPLEX_MODE;
                        priv->duplex = phydev->duplex;
                }

                if (phydev->speed != priv->speed) {
                        link_changed = true;

                        ctrl &= ~MREGBIT_SPEED;

                        switch (phydev->speed) {
                        case SPEED_1000:
                                ctrl |= MREGBIT_SPEED_1000M;
                                break;
                        case SPEED_100:
                                ctrl |= MREGBIT_SPEED_100M;
                                break;
                        case SPEED_10:
                                ctrl |= MREGBIT_SPEED_10M;
                                break;
                        default:
                                pr_err("broken speed: %d\n", phydev->speed);
                                phydev->speed = SPEED_UNKNOWN;
                                break;
                        }
                        if (phydev->speed != SPEED_UNKNOWN) {
                                priv->speed = phydev->speed;
                        }
                }

                emac_wr(priv, MAC_GLOBAL_CONTROL, ctrl);

                if (!priv->link) {
                        priv->link = true;
                        link_changed = true;
                }
        } else if (priv->link) {
                priv->link = false;
                link_changed = true;
                priv->duplex = DUPLEX_UNKNOWN;
                priv->speed = SPEED_UNKNOWN;
        }

        if (link_changed)
                phy_print_status(phydev);
}

static int emac_phy_connect(struct net_device *dev)
{
        struct phy_device *phydev;
        struct device_node *np;
        struct emac_priv *priv = netdev_priv(dev);

        np = of_parse_phandle(priv->pdev->dev.of_node, "phy-handle", 0);
        if (!np && of_phy_is_fixed_link(priv->pdev->dev.of_node))
                np = of_node_get(priv->pdev->dev.of_node);
        if (!np)
                return -ENODEV;

        of_get_phy_mode(np, &priv->phy_interface);
        pr_info("priv phy_interface = %d\n", priv->phy_interface);

        emac_phy_interface_config(priv);

        phydev = of_phy_connect(dev, np,
                                &emac_adjust_link, 0, priv->phy_interface);
        if (IS_ERR_OR_NULL(phydev)) {
                pr_err("Could not attach to PHY\n");
                if (!phydev)
                        return -ENODEV;
                return PTR_ERR(phydev);
        }

        pr_info("%s:  %s: attached to PHY (UID 0x%x)"
                        " Link = %d\n", __func__,
                        dev->name, phydev->phy_id, phydev->link);

        /* Indicate that the MAC is responsible for PHY PM */
        phydev->mac_managed_pm = true;
        dev->phydev = phydev;

        clk_phase_set(priv, TX_PHASE);
        clk_phase_set(priv, RX_PHASE);
        return 0;
}

static int emac_mdio_init(struct emac_priv *priv)
{
        struct device_node *mii_np;
        struct device *dev = &priv->pdev->dev;
        int ret;

        mii_np = of_get_child_by_name(dev->of_node, "mdio-bus");
        if (!mii_np) {
                if (of_phy_is_fixed_link(dev->of_node)) {
                        if ((of_phy_register_fixed_link(dev->of_node) < 0)) {
                                return -ENODEV;
                        }
                        dev_dbg(dev, "find fixed link\n");
                        return 0;
                }

                dev_err(dev, "no %s child node found", "mdio-bus");
                return -ENODEV;
        }

        if (!of_device_is_available(mii_np)) {
                ret = -ENODEV;
                goto err_put_node;
        }

        priv->mii = devm_mdiobus_alloc(dev);
        if (!priv->mii) {
                ret = -ENOMEM;
                goto err_put_node;
        }
        priv->mii->priv = priv;
        priv->mii->name = "emac mii";
        priv->mii->rstc = emac_mii_reset;
        priv->mii->read = emac_mii_read;
        priv->mii->write = emac_mii_write;
        snprintf(priv->mii->id, MII_BUS_ID_SIZE, "%s",
                        priv->pdev->name);
        priv->mii->parent = dev;
        priv->mii->phy_mask = 0xffffffff;
        ret = of_mdiobus_register(priv->mii, mii_np);
        if (ret) {
                dev_err(dev, "Failed to register mdio bus.\n");
                goto err_put_node;
        }

        priv->phy = phy_find_first(priv->mii);
        if (!priv->phy) {
                dev_err(dev, "no PHY found\n");
                return -ENODEV;
        }

err_put_node:
        of_node_put(mii_np);
        return ret;
}

static int emac_mdio_deinit(struct emac_priv *priv)
{
        if (!priv->mii)
                return 0;

        mdiobus_unregister(priv->mii);
        return 0;
}

static void emac_stats_update(struct emac_priv *priv)
{
        struct emac_hw_stats *hwstats = priv->hw_stats;
        int i;
        rt_uint32_t *p;

        p = (rt_uint32_t *)(hwstats);

        for (i = 0; i < MAX_TX_STATS_NUM; i++)
                *(p + i) = ReadTxStatCounters(priv, i);

        p = (rt_uint32_t *)hwstats + MAX_TX_STATS_NUM;

        for (i = 0; i < MAX_RX_STATS_NUM; i++)
                *(p + i) = ReadRxStatCounters(priv, i);
}

static const struct net_device_ops emac_netdev_ops = {
        .ndo_open               = emac_open,
        .ndo_stop               = emac_close,
        .ndo_start_xmit         = emac_start_xmit,
        .ndo_set_mac_address    = emac_set_mac_address,
        .ndo_do_ioctl           = emac_ioctl,
        .ndo_eth_ioctl          = emac_ioctl,
        .ndo_change_mtu         = emac_change_mtu,
        .ndo_tx_timeout         = emac_tx_timeout,
        .ndo_set_rx_mode        = emac_rx_mode_set,
};

#ifdef CONFIG_PM_SLEEP
static int emac_resume(struct device *dev)
{
        struct emac_priv *priv = dev_get_drvdata(dev);
        struct net_device *ndev = priv->ndev;
        int ret;

        if (priv->ref_clk_frm_soc) {
                ret = clk_prepare_enable(priv->phy_clk);
                if (ret < 0) {
                        pr_err("failed to enable phy clock: %d\n", ret);
                        goto err;
                }
        }

        ret = clk_prepare_enable(priv->mac_clk);
        if (ret < 0) {
                pr_err("failed to enable mac clock: %d\n", ret);
                goto disable_phy_clk;
        }

        if (!netif_running(ndev)) {
                return 0;
        }

        emac_open(ndev);
        netif_device_attach(ndev);
        return 0;

disable_phy_clk:
        if (priv->ref_clk_frm_soc)
                clk_disable_unprepare(priv->phy_clk);
err:
        return ret;
}

static int emac_suspend(struct device *dev)
{
        struct emac_priv *priv = dev_get_drvdata(dev);
        struct net_device *ndev = priv->ndev;

        if (!ndev || !netif_running(ndev)) {
                clk_disable_unprepare(priv->mac_clk);
                if (priv->ref_clk_frm_soc)
                        clk_disable_unprepare(priv->phy_clk);
                return 0;
        }

        emac_close(ndev);
        clk_disable_unprepare(priv->mac_clk);
        if (priv->ref_clk_frm_soc)
                clk_disable_unprepare(priv->phy_clk);
        netif_device_detach(ndev);
        return 0;
}
#endif


void emac_hw_timestamp_config(struct emac_priv *priv, rt_uint32_t enable, u8 rx_ptp_type, rt_uint32_t ptp_msg_id)
{
        void __iomem *ioaddr = priv->iobase;
        rt_uint32_t val;

        if (enable) {
                /*
                 * enable tx/rx timestamp and config rx ptp type
                 */
                val = TX_TIMESTAMP_EN | RX_TIMESTAMP_EN;
                val |= (rx_ptp_type << RX_PTP_PKT_TYPE_OFST) & RX_PTP_PKT_TYPE_MSK;
                writel(val, ioaddr + PTP_1588_CTRL);

                /* config ptp message id */
                writel(ptp_msg_id, ioaddr + PTP_MSG_ID);

                /* config ptp ethernet type */
                writel(DEFAULT_ETH_TYPE, ioaddr + PTP_ETH_TYPE);

                /* config ptp udp port */
                writel(DEFAULT_UDP_PORT, ioaddr + PTP_UDP_PORT);

        } else
                writel(0, ioaddr + PTP_1588_CTRL);
}

rt_uint32_t emac_hw_config_systime_increment(struct emac_priv *priv, rt_uint32_t ptp_clock,
                                        rt_uint32_t adj_clock)
{
        void __iomem *ioaddr = priv->iobase;
        rt_uint32_t incr_val;
        rt_uint32_t incr_period;
        rt_uint32_t val;
        rt_uint32_t period = 0, def_period = 0;
        /*
         *  set system time counter resolution as ns
         *  if ptp clock is 50Mhz,  20ns per clock cycle,
         *  so increment value should be 20,
         *  increment period should be 1m
         */
        if (ptp_clock == adj_clock) {
                incr_val = INCVALUE_100MHZ << INCVALUE_SHIFT_100MHZ;
                incr_period = INCPERIOD_100MHZ;
        } else {
                def_period = div_u64(1000000000ULL, ptp_clock);
                period = div_u64(1000000000ULL, adj_clock);
                if (def_period == period)
                        return 0;

                incr_period = 1;
                incr_val = (def_period * def_period)/ period;
        }

        val = (incr_val | (incr_period << INCR_PERIOD_OFST));
        writel(val, ioaddr + PTP_INRC_ATTR);

        return 0;
}

u64 emac_hw_get_systime(struct emac_priv *priv)
{
        void __iomem *ioaddr = priv->iobase;
        u64 systimel, systimeh;
        u64 systim;

        /* update system time adjust low register */
        systimel = readl(ioaddr + SYS_TIME_GET_LOW);
        systimeh = readl(ioaddr + SYS_TIME_GET_HI);
        /* perform system time adjust */
        systim = (systimeh << 32) | systimel;

        return systim;
}

u64 emac_hw_get_phc_time(struct emac_priv *priv)
{
        unsigned long flags;
        u64 cycles, ns;

        spin_lock_irqsave(&priv->ptp_lock, flags);
        /* first read system time low register */
        cycles = emac_hw_get_systime(priv);
        ns = timecounter_cyc2time(&priv->tc, cycles);

        spin_unlock_irqrestore(&priv->ptp_lock, flags);

        return ns;
}

u64 emac_hw_get_tx_timestamp(struct emac_priv *priv)
{
        void __iomem *ioaddr = priv->iobase;
        unsigned long flags;
        u64 systimel, systimeh;
        u64 systim;
        u64 ns;

        /* first read system time low register */
        systimel = readl(ioaddr + TX_TIMESTAMP_LOW);
        systimeh = readl(ioaddr + TX_TIMESTAMP_HI);
        systim = (systimeh << 32) | systimel;

        spin_lock_irqsave(&priv->ptp_lock, flags);

        ns = timecounter_cyc2time(&priv->tc, systim);
        spin_unlock_irqrestore(&priv->ptp_lock, flags);
        return ns;
}

u64 emac_hw_get_rx_timestamp(struct emac_priv *priv)
{
        void __iomem *ioaddr = priv->iobase;
        unsigned long flags;
        u64 systimel, systimeh;
        u64 systim;
        u64 ns;

        /* first read system time low register */
        systimel = readl(ioaddr + RX_TIMESTAMP_LOW);
        systimeh = readl(ioaddr + RX_TIMESTAMP_HI);
        systim = (systimeh << 32) | systimel;

        spin_lock_irqsave(&priv->ptp_lock, flags);

        ns = timecounter_cyc2time(&priv->tc, systim);
        spin_unlock_irqrestore(&priv->ptp_lock, flags);
        return ns;
}

/**
 * emac_cyclecounter_read - read raw cycle counter (used by time counter)
 * @cc: cyclecounter structure
 **/
static u64 emac_cyclecounter_read(const struct cyclecounter *cc)
{
        struct emac_priv *priv = container_of(cc, struct emac_priv, cc);

        return emac_hw_get_systime(priv);
}
        /* according to spec , set system time upper register and adjust time */
int emac_hw_init_systime(struct emac_priv *priv, u64 set_ns)
{
        unsigned long flags;

        spin_lock_irqsave(&priv->ptp_lock, flags);

        timecounter_init(&priv->tc, &priv->cc, set_ns);

        spin_unlock_irqrestore(&priv->ptp_lock, flags);

        return 0;
}

struct emac_hw_ptp emac_hwptp = {
        .config_hw_tstamping = emac_hw_timestamp_config,
        .config_systime_increment = emac_hw_config_systime_increment,
        .init_systime = emac_hw_init_systime,
        .get_phc_time = emac_hw_get_phc_time,
        .get_tx_timestamp = emac_hw_get_tx_timestamp,
        .get_rx_timestamp = emac_hw_get_rx_timestamp,
};

/**
 * emac_adjust_freq
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ppb: desired period change in parts ber billion
 *
 * Description: this function will adjust the frequency of hardware clock.
 */
static int emac_adjust_freq(struct ptp_clock_info *ptp, s32 ppb)
{
        struct emac_priv *priv = to_emacpriv(ptp);
        void __iomem *ioaddr = priv->iobase;
        unsigned long flags;
        rt_uint32_t addend, incvalue;
        int neg_adj = 0;
        u64 adj;

        if ((ppb > ptp->max_adj) || (ppb <= -1000000000))
                return -EINVAL;
        if (ppb < 0) {
                neg_adj = 1;
                ppb = -ppb;
        }

        spin_lock_irqsave(&priv->ptp_lock, flags);

        incvalue = INCVALUE_100MHZ << INCVALUE_SHIFT_100MHZ;
        adj = incvalue;
        adj *= ppb;
        adj = div_u64(adj, 1000000000);
        /*
         * ppb = (Fnew - F0)/F0
         * diff = F0 * ppb
         */

        addend = neg_adj ? (incvalue - adj) : (incvalue + adj);
        pr_debug("emac_adjust_freq: new inc_val=%d ppb=%d\n", addend, ppb);
        addend = (addend | (INCPERIOD_100MHZ << INCR_PERIOD_OFST));
        writel(addend, ioaddr + PTP_INRC_ATTR);

        spin_unlock_irqrestore(&priv->ptp_lock, flags);

        return 0;
}

static int emac_adjust_fine(struct ptp_clock_info *ptp, long scaled_ppm)
{
        s32 ppb;

        ppb = scaled_ppm_to_ppb(scaled_ppm);
        return emac_adjust_freq(ptp, ppb);
}

/**
 * emac_adjust_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @delta: desired change in nanoseconds
 *
 * Description: this function will shift/adjust the hardware clock time.
 */
static int emac_adjust_time(struct ptp_clock_info *ptp, s64 delta)
{
        struct emac_priv *priv = to_emacpriv(ptp);
        unsigned long flags;

        spin_lock_irqsave(&priv->ptp_lock, flags);

        timecounter_adjtime(&priv->tc, delta);

        spin_unlock_irqrestore(&priv->ptp_lock, flags);

        return 0;
}

/**
 * emac_get_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: pointer to hold time/result
 *
 * Description: this function will read the current time from the
 * hardware clock and store it in @ts.
 */
static int emac_phc_get_time(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
        struct emac_priv *priv = to_emacpriv(ptp);
        unsigned long flags;
        u64 cycles, ns;

        spin_lock_irqsave(&priv->ptp_lock, flags);

        cycles = emac_hw_get_systime(priv);
        ns = timecounter_cyc2time(&priv->tc, cycles);

        spin_unlock_irqrestore(&priv->ptp_lock, flags);

        *ts = ns_to_timespec64(ns);

        return 0;
}

/**
 * emac_set_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: time value to set
 *
 * Description: this function will set the current time on the
 * hardware clock.
 */
static int emac_phc_set_time(struct ptp_clock_info *ptp,
                           const struct timespec64 *ts)
{
        struct emac_priv *priv = to_emacpriv(ptp);
        unsigned long flags;
        u64 ns;

        ns = timespec64_to_ns(ts);

        spin_lock_irqsave(&priv->ptp_lock, flags);

        timecounter_init(&priv->tc, &priv->cc, ns);

        spin_unlock_irqrestore(&priv->ptp_lock, flags);

        return 0;
}

static void emac_systim_overflow_work(struct work_struct *work)
{
        struct emac_priv *priv = container_of(work, struct emac_priv,
                                                     systim_overflow_work.work);
        struct timespec64 ts;
        u64 ns;
        ns = timecounter_read(&priv->tc);
        ts = ns_to_timespec64(ns);
        pr_debug("SYSTIM overflow check at %lld.%09lu\n",
              (long long) ts.tv_sec, ts.tv_nsec);
        schedule_delayed_work(&priv->systim_overflow_work,
                             EMAC_SYSTIM_OVERFLOW_PERIOD);
}
/* structure describing a PTP hardware clock */
static struct ptp_clock_info emac_ptp_clock_ops = {
        .owner = THIS_MODULE,
        .name = "emac_ptp_clock",
        .max_adj = 1000000000,
        .n_alarm = 0,
        .n_ext_ts = 0,
        .n_per_out = 0,
        .n_pins = 0,
        .pps = 0,
        .adjfine = emac_adjust_fine,
        .adjtime = emac_adjust_time,
        .gettime64 = emac_phc_get_time,
        .settime64 = emac_phc_set_time,
};

/**
 * emac_ptp_register
 * @priv: driver private structure
 * Description: this function will register the ptp clock driver
 * to kernel. It also does some house keeping work.
 */
void emac_ptp_register(struct emac_priv *priv)
{
        unsigned long flags;
        priv->cc.read = emac_cyclecounter_read;
        priv->cc.mask = CYCLECOUNTER_MASK(64);
        priv->cc.mult = 1;
        priv->cc.shift = INCVALUE_SHIFT_100MHZ;
        spin_lock_init(&priv->ptp_lock);
        priv->ptp_clock_ops = emac_ptp_clock_ops;

        INIT_DELAYED_WORK(&priv->systim_overflow_work,
                          emac_systim_overflow_work);
        schedule_delayed_work(&priv->systim_overflow_work,
                              EMAC_SYSTIM_OVERFLOW_PERIOD);
        priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_ops,
                                             NULL);
        if (IS_ERR(priv->ptp_clock)) {
                netdev_err(priv->ndev, "ptp_clock_register failed\n");
                priv->ptp_clock = NULL;
        } else if (priv->ptp_clock)
                netdev_info(priv->ndev, "registered PTP clock\n");
        else
                netdev_info(priv->ndev, "PTP_1588_CLOCK maybe not enabled\n");

        spin_lock_irqsave(&priv->ptp_lock, flags);
        timecounter_init(&priv->tc, &priv->cc,
                         ktime_to_ns(ktime_get_real()));
        spin_unlock_irqrestore(&priv->ptp_lock, flags);
        priv->hwptp = &emac_hwptp;
}

/**
 * emac_ptp_unregister
 * @priv: driver private structure
 * Description: this function will remove/unregister the ptp clock driver
 * from the kernel.
 */
void emac_ptp_unregister(struct emac_priv *priv)
{
    cancel_delayed_work_sync(&priv->systim_overflow_work);
    if (priv->ptp_clock) {
        ptp_clock_unregister(priv->ptp_clock);
        priv->ptp_clock = NULL;
        pr_debug("Removed PTP HW clock successfully on %s\n",
                 priv->ndev->name);
    }
    priv->hwptp = NULL;
}


static int emac_config_dt(struct platform_device *pdev, struct emac_priv *priv)
{


        return 0;
}

static rt_err_t emac_probe(struct rt_platform_device *pdev)
{
    struct rt_device *dev = &pdev->parent;
    struct emac_eth *eth = rt_calloc(1, sizeof(*eth));

    if (!eth)
    {
        return -RT_ENOMEM;
    }

        struct device_node *np = pdev->dev.of_node;
        struct resource *res;
        u8 mac_addr[ETH_ALEN] = {0};
        rt_uint32_t tx_phase, rx_phase;
        rt_uint32_t ctrl_reg;
        int ret;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    priv->iobase = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(priv->iobase)) {
        dev_err(&pdev->dev, "failed to io remap res reg 0\n");
        return -ENOMEM;
    }

    if (of_property_read_u32(np, "k1x,apmu-base-reg", &priv->apmu_base)) {
        priv->apmu_base = PMUA_BASE_REG;
    }

    priv->irq = irq_of_parse_and_map(np, 0);
    if (!priv->irq) {
        return -ENXIO;
    }

    if (of_property_read_u32(np, "ctrl-reg", &ctrl_reg)) {
        dev_err(&pdev->dev, "cannot find ctrl register in device tree\n");
        return -EINVAL;
    }

    priv->ctrl_reg = ioremap(priv->apmu_base + ctrl_reg, 4);

    if (of_property_read_u32(np, "tx-threshold",
                         &priv->tx_threshold)) {
        priv->tx_threshold = DEFAULT_TX_THRESHOLD;
        dev_dbg(&pdev->dev, "%s tx_threshold using default value:%d \n",
                __func__, priv->tx_threshold);
    }

    if (of_property_read_u32(np, "rx-threshold",
                         &priv->rx_threshold)) {
        priv->rx_threshold = DEFAULT_RX_THRESHOLD;
        dev_dbg(&pdev->dev, "%s rx_threshold using default value:%d \n",
                __func__, priv->rx_threshold);
    }

    if (of_property_read_u32(np, "tx-ring-num",
                         &priv->tx_ring_num)) {
        priv->tx_ring_num = DEFAULT_TX_RING_NUM;
        dev_dbg(&pdev->dev, "%s tx_ring_num using default value:%d \n",
                __func__, priv->tx_ring_num);
    }

    if (of_property_read_u32(np, "rx-ring-num",
                         &priv->rx_ring_num)) {
        priv->rx_ring_num = DEFAULT_RX_RING_NUM;
        dev_dbg(&pdev->dev, "%s rx_ring_num using default value:%d \n",
                __func__, priv->rx_ring_num);
    }

    if (of_property_read_u32(np, "dma-burst-len",
                         &priv->dma_burst_len)) {
        priv->dma_burst_len = DEFAULT_DMA_BURST_LEN;
        dev_dbg(&pdev->dev, "%s dma_burst_len using default value:%d \n",
                __func__, priv->dma_burst_len);
    } else {
        if (priv->dma_burst_len <= 0 || priv->dma_burst_len > 7) {
                dev_err(&pdev->dev, "%s burst len illegal, use default vallue:%d\n",
                        __func__, DEFAULT_DMA_BURST_LEN);
                priv->dma_burst_len = DEFAULT_DMA_BURST_LEN;
        }
    }

    if (of_property_read_bool(np, "ref-clock-from-phy")) {
        priv->ref_clk_frm_soc = 0;
        dev_dbg(&pdev->dev, "%s ref clock from external phy \n", __func__);
    } else
        priv->ref_clk_frm_soc = 1;


    ret = of_get_mac_address(np, mac_addr);
    if (ret) {
        if (ret == -EPROBE_DEFER)
                return ret;

        dev_info(&pdev->dev, "Using random mac address\n");
        eth_hw_addr_random(priv->ndev);
    } else {
        eth_hw_addr_set(priv->ndev, mac_addr);
    }

    dev_dbg(&pdev->dev, "%s tx-threshold:%d rx_therhold:%d tx_ring_num:%d rx_ring_num:%d dma-bur_len:%d\n",
        __func__, priv->tx_threshold, priv->rx_threshold, priv->tx_ring_num,
        priv->rx_ring_num, priv->dma_burst_len);

    priv->ptp_support = of_property_read_bool(np, "ptp-support");
    if (priv->ptp_support) {
        dev_dbg(&pdev->dev, "EMAC support IEEE1588 PTP Protocol\n");
        if (of_property_read_u32(np, "ptp-clk-rate",
                                &priv->ptp_clk_rate)) {
                priv->ptp_clk_rate = 20000000;
                dev_dbg(&pdev->dev, "%s ptp_clk rate using default value:%d may inaccurate!!1\n",
                        __func__, priv->ptp_clk_rate);
        }
    }
    priv->clk_tuning_enable = of_property_read_bool(np, "clk-tuning-enable");
    if (priv->clk_tuning_enable) {
        if (of_property_read_bool(np, "clk-tuning-by-reg"))
                priv->clk_tuning_way = CLK_TUNING_BY_REG;
        else if (of_property_read_bool(np, "clk-tuning-by-clk-revert"))
                priv->clk_tuning_way = CLK_TUNING_BY_CLK_REVERT;
        else if (of_property_read_bool(np, "clk-tuning-by-delayline")) {
                priv->clk_tuning_way = CLK_TUNING_BY_DLINE;
                if (of_property_read_u32(np, "dline-reg", &ctrl_reg)) {
                        dev_err(&pdev->dev, "cannot find delayline register in device tree\n");
                        return -EINVAL;
                }
                priv->dline_reg = ioremap(priv->apmu_base + ctrl_reg, 4);
        } else
                priv->clk_tuning_way = CLK_TUNING_BY_REG;

        if (of_property_read_u32(np, "tx-phase", &tx_phase))
                priv->tx_clk_phase = TXCLK_PHASE_DEFAULT;
        else
                priv->tx_clk_phase = tx_phase;

        if (of_property_read_u32(np, "rx-phase", &rx_phase))
                priv->rx_clk_phase = RXCLK_PHASE_DEFAULT;
        else
                priv->rx_clk_phase = rx_phase;
    }

    ndev->watchdog_timeo = 5 * HZ;
    ndev->base_addr = (unsigned long)priv->iobase;
    ndev->irq  = priv->irq;

    ndev->ethtool_ops = &emac_ethtool_ops;
    ndev->netdev_ops = &emac_netdev_ops;

    priv->mac_clk = devm_clk_get(&pdev->dev, "emac-clk");
    if (IS_ERR(priv->mac_clk)) {
            dev_err(&pdev->dev, "emac clock not found.\n");
            ret = PTR_ERR(priv->mac_clk);
            goto err_netdev;
    }

    ret = clk_prepare_enable(priv->mac_clk);
    if (ret < 0) {
            dev_err(&pdev->dev, "failed to enable emac clock: %d\n",
                    ret);
            goto err_netdev;
    }

    if (priv->ref_clk_frm_soc) {
            priv->phy_clk = devm_clk_get(&pdev->dev, "phy-clk");
            if (IS_ERR(priv->phy_clk)) {
                    dev_err(&pdev->dev, "phy clock not found.\n");
                    ret = PTR_ERR(priv->phy_clk);
                    goto mac_clk_disable;
            }

            ret = clk_prepare_enable(priv->phy_clk);
            if (ret < 0) {
                    dev_err(&pdev->dev, "failed to enable phy clock: %d\n",
                            ret);
                    goto mac_clk_disable;
            }
    }
    if (priv->ptp_support) {
            priv->ptp_clk = devm_clk_get(&pdev->dev, "ptp-clk");
            if (IS_ERR(priv->ptp_clk)) {
                    dev_err(&pdev->dev, "ptp clock not found.\n");
                    ret = PTR_ERR(priv->ptp_clk);
                    goto phy_clk_disable;
            }
    }

    priv->rstc = devm_reset_control_get_optional(&pdev->dev, NULL);
    if (IS_ERR(priv->rstc)) {
            dev_err(&pdev->dev, "Failed to get emac's resets\n");
            goto ptp_clk_disable;
    }

    reset_control_deassert(priv->rstc);

    emac_sw_init(priv);

    ret = emac_mdio_init(priv);
    if (ret) {
            dev_err(&pdev->dev, "failed to init mdio.\n");
            goto reset_assert;
    }

    SET_NETDEV_DEV(ndev, &pdev->dev);

    ret = register_netdev(ndev);
    if (ret) {
            pr_err("register_netdev failed\n");
            goto err_mdio_deinit;
    }
    dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));

    netif_napi_add(ndev, &priv->napi, emac_rx_poll);

    return 0;
err_mdio_deinit:
    emac_mdio_deinit(priv);
reset_assert:
    reset_control_assert(priv->rstc);
ptp_clk_disable:
    if (priv->ptp_support)
            clk_disable_unprepare(priv->ptp_clk);
phy_clk_disable:
    if (priv->ref_clk_frm_soc)
            clk_disable_unprepare(priv->phy_clk);
    del_timer_sync(&priv->txtimer);
mac_clk_disable:
    clk_disable_unprepare(priv->mac_clk);
err_netdev:
    free_netdev(ndev);
dev_info(&pdev->dev, "emac_probe failed ret = %d.\n", ret);
    return ret;
}

static rt_err_t emac_remove(struct rt_platform_device *pdev)
{
    struct rt_device *dev = &pdev->parent;
    struct emac_eth *eth = dev->user_data;

    unregister_netdev(priv->ndev);
    emac_reset_hw(priv);
    free_netdev(priv->ndev);
    emac_mdio_deinit(priv);
    rt_reset_control_assert(priv->rstc);
    rt_clk_disable_unprepare(priv->mac_clk);

    if (priv->ref_clk_frm_soc)
    {
        clk_disable_unprepare(priv->phy_clk);
    }

    return RT_EOK;
}

static const struct rt_ofw_node_id emac_ofw_ids[] =
{
    { .compatible = "spacemit,k1x-emac" },
    { /* sentinel */ }
};

static struct rt_platform_driver emac_driver =
{
    .name = "spacemit-pmic-adc",
    .ids = emac_ofw_ids,

    .probe = emac_probe,
    .remove = emac_remove,
};
RT_PLATFORM_DRIVER_EXPORT(emac_driver);
