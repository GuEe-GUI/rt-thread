/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     the first version
 */

#ifndef __MTD_CFI_H__
#define __MTD_CFI_H__

#define CFI_FLASH_8BIT                      0x01
#define CFI_FLASH_16BIT                     0x02
#define CFI_FLASH_BY8                       0x01
#define CFI_FLASH_BY16                      0x02
#define CFI_FLASH_BY32                      0x04
#define CFI_FLASH_X8                        0x00
#define CFI_FLASH_X16                       0x01
#define CFI_FLASH_X8X16                     0x02

/* commands */
#define CFI_FLASH_CMD_CFI                   0x98
#define CFI_FLASH_CMD_READ_ID               0x90
#define CFI_FLASH_CMD_RESET                 0xff
#define CFI_FLASH_CMD_BLOCK_ERASE           0x20
#define CFI_FLASH_CMD_ERASE_CONFIRM         0xd0
#define CFI_FLASH_CMD_WRITE                 0x40
#define CFI_FLASH_CMD_PROTECT               0x60
#define CFI_FLASH_CMD_PROTECT_SET           0x01
#define CFI_FLASH_CMD_PROTECT_CLEAR         0xd0
#define CFI_FLASH_CMD_CLEAR_STATUS          0x50
#define CFI_FLASH_CMD_WRITE_TO_BUFFER       0xe8
#define CFI_FLASH_CMD_WRITE_BUFFER_CONFIRM  0xd0

#define CFI_FLASH_STATUS_DONE               0x80
#define CFI_FLASH_STATUS_ESS                0x40
#define CFI_FLASH_STATUS_ECLBS              0x20
#define CFI_FLASH_STATUS_PSLBS              0x10
#define CFI_FLASH_STATUS_VPENS              0x08
#define CFI_FLASH_STATUS_PSS                0x04
#define CFI_FLASH_STATUS_DPS                0x02
#define CFI_FLASH_STATUS_R                  0x01
#define CFI_FLASH_STATUS_PROTECT            0x01

#define CFI_AMD_CMD_RESET                   0xf0
#define CFI_AMD_CMD_WRITE                   0xa0
#define CFI_AMD_CMD_ERASE_START             0x80
#define CFI_AMD_CMD_ERASE_SECTOR            0x30
#define CFI_AMD_CMD_ERASE_CHIP              0x10
#define CFI_AMD_CMD_UNLOCK_START            0xaa
#define CFI_AMD_CMD_UNLOCK_ACK              0x55

#define CFI_AMD_CMD_WRITE_TO_BUFFER         0x25
#define CFI_AMD_CMD_WRITE_BUFFER_CONFIRM    0x29

#define CFI_AMD_DQ7                         0x80
#define CFI_AMD_DQ5                         0x20
#define CFI_AMD_STATUS_TIMEOUT              0x20

#define CFI_AMD_STATUS_TOGGLE               0x40
#define CFI_AMD_STATUS_ERROR                0x20

#define CFI_AMD_ADDR_ERASE_START            0x555
#define CFI_AMD_ADDR_START                  0x555
#define CFI_AMD_ADDR_ACK                    0x2aa

#define CFI_FLASH_OFFSET_CFI                0x55
#define CFI_FLASH_OFFSET_CFI_RESP           0x10
#define CFI_FLASH_OFFSET_PRIMARY_VENDOR     0x13
#define CFI_FLASH_OFFSET_WTOUT              0x1f
#define CFI_FLASH_OFFSET_WBTOUT             0x20
#define CFI_FLASH_OFFSET_ETOUT              0x21
#define CFI_FLASH_OFFSET_CETOUT             0x22
#define CFI_FLASH_OFFSET_WMAX_TOUT          0x23
#define CFI_FLASH_OFFSET_WBMAX_TOUT         0x24
#define CFI_FLASH_OFFSET_EMAX_TOUT          0x25
#define CFI_FLASH_OFFSET_CEMAX_TOUT         0x26
#define CFI_FLASH_OFFSET_SIZE               0x27
#define CFI_FLASH_OFFSET_INTERFACE          0x28
#define CFI_FLASH_OFFSET_BUFFER_SIZE        0x2a
#define CFI_FLASH_OFFSET_NUM_ERASE_REGIONS  0x2c
#define CFI_FLASH_OFFSET_ERASE_REGIONS      0x2d
#define CFI_FLASH_OFFSET_PROTECT            0x02
#define CFI_FLASH_OFFSET_USER_PROTECTION    0x85
#define CFI_FLASH_OFFSET_INTEL_PROTECTION   0x81

#define CFI_FLASH_MAN                       0x01000000

#define CFI_CMDSET_NONE                     0
#define CFI_CMDSET_INTEL_EXTENDED           1
#define CFI_CMDSET_AMD_STANDARD             2
#define CFI_CMDSET_INTEL_STANDARD           3
#define CFI_CMDSET_AMD_EXTENDED             4
#define CFI_CMDSET_MITSU_STANDARD           256
#define CFI_CMDSET_MITSU_EXTENDED           257
#define CFI_CMDSET_SST                      258

#endif /* __MTD_CFI_H__ */
