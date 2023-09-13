/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-25     GuEe-GUI     the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "mtd.cfi"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <ioremap.h>

#include "mtd-cfi.h"

#define ERASE_POLL_LIMIT    0x60000000

struct cfi_flash_map
{
    struct rt_mtd_nor_device parent;

    struct rt_mutex rw_lock;

    void *base;

    rt_size_t sect_count;
    rt_ubase_t *sect;
    rt_ubase_t *protect;

    rt_uint16_t vendor;
    rt_uint16_t cmd_reset;
    rt_uint16_t interface;
    rt_uint32_t chipwidth;
    rt_uint32_t portwidth;
    rt_bool_t use_buffer_write;

    rt_size_t block_size;
    rt_uint16_t buffer_size;
    rt_uint32_t erase_blk_tout;
    rt_uint32_t erase_chip_tout;
    rt_uint32_t buffer_write_tout;
    rt_uint32_t write_tout;
    rt_size_t num_erase_regions;

    rt_uint64_t address;
    rt_uint64_t size;
};

#define raw_to_cfi_flash_map(raw)   rt_container_of(raw, struct cfi_flash_map, parent)

struct cfi_flash
{
    rt_uint32_t bank_width;

    rt_size_t maps_nr;
    struct cfi_flash_map *maps;
};

union cfiword
{
    rt_uint8_t c;
    rt_uint16_t w;
    rt_uint32_t l;
};

union cfiptr
{
    rt_uint8_t *cp;
    rt_uint16_t *wp;
    rt_uint32_t *lp;
};

static const rt_uint32_t offset_multiply[9][5] =
{
    /* chip width = 0 (dummy), 1 (8-bit), 2 (16-bit), 3 (dummy), 4(32-bit) */
    {0,  0, 0, 0, 0},   /* dummy */
    {0,  1, 0, 0, 0},   /* port width = 1 (8-bit) */
    {0,  4, 2, 0, 0},   /* port width = 2 (16-bit) */
    {0,  0, 0, 0, 0},   /* dummy */
    {0,  8, 4, 0, 4},   /* port width = 4 (32-bit) */
    {0,  0, 0, 0, 0},   /* 5, dummy */
    {0,  0, 0, 0, 0},   /* 6, dummy */
    {0,  0, 0, 0, 0},   /* 7, dummy */
    {0, 16, 8, 0, 8},   /* port width = 8 (64-bit) */
};

rt_used const char *const vendor_name[] =
{
    [CFI_CMDSET_INTEL_EXTENDED] = "Intel/Sharp extended",
    [CFI_CMDSET_AMD_STANDARD]   = "AMD/Fujitsu standard",
    [CFI_CMDSET_INTEL_STANDARD] = "Intel/Sharp standard",
    [CFI_CMDSET_AMD_EXTENDED]   = "AMD/Fujitsu extended",
    [CFI_CMDSET_MITSU_STANDARD] = "Mitsubishi standard",
    [CFI_CMDSET_MITSU_EXTENDED] = "Mitsubishi extendend",
};

static rt_uint8_t *cfi_flash_make_addr(struct cfi_flash_map *maps, rt_int32_t sect, rt_uint32_t offset)
{
    rt_ubase_t base = maps->base ? ((rt_ubase_t)maps->base + sect) : maps->sect[sect];

    return (rt_uint8_t *)(base + (offset * offset_multiply[maps->portwidth][maps->chipwidth]));
}

static rt_uint8_t cfi_flash_read_u8(struct cfi_flash_map *maps, rt_uint32_t offset)
{
    void *addr = cfi_flash_make_addr(maps, 0, offset);

    return HWREG8(addr);
}

static rt_uint16_t cfi_flash_read_u16(struct cfi_flash_map *maps, rt_int32_t sect, rt_uint32_t offset)
{
    void *addr = cfi_flash_make_addr(maps, sect, offset);

    return rt_cpu_to_le16(HWREG16(addr));
}

static rt_uint32_t cfi_flash_read_u32(struct cfi_flash_map *maps, rt_int32_t sect, rt_uint32_t offset)
{
    void *addr = cfi_flash_make_addr(maps, sect, offset);

    return rt_cpu_to_le32(HWREG32(addr));
}

static void cfi_flash_make_cmd(struct cfi_flash_map *maps, rt_uint8_t cmd, void *cmdbuf)
{
    rt_uint8_t *cp = (rt_uint8_t *)cmdbuf;

    if (maps->chipwidth < CFI_FLASH_BY32)
    {
        for (rt_int32_t i = 0; i < maps->portwidth; ++i)
        {
            *cp++ = ((i + 1) % maps->chipwidth) ? '\0' : cmd;
        }
    }
    else
    {
        rt_uint16_t cmd16 = cmd + cmd * 256;
        rt_uint16_t *ccp = (rt_uint16_t *)cmdbuf;

        for (rt_int32_t i = 0; i < maps->portwidth; i += 2)
        {
            *ccp++ = ((i + 2) % maps->chipwidth) ? '\0' : cmd16;
        }
    }

#ifndef ARCH_CPU_BIG_ENDIAN
    switch (maps->portwidth)
    {
    case CFI_FLASH_8BIT:
        break;

    case CFI_FLASH_16BIT:
        {
            rt_uint16_t stmpw = *(rt_uint16_t *)cmdbuf;

            *(rt_uint16_t *)cmdbuf = rt_be16_to_cpu(stmpw);
        }
        break;

    default:
        break;
    }
#endif
}

static void cfi_flash_write_cmd(struct cfi_flash_map *maps, rt_int32_t sect, rt_uint32_t offset, rt_uint8_t cmd)
{
    union cfiptr addr;
    union cfiword cword;

    addr.cp = cfi_flash_make_addr(maps, sect, offset);
    cfi_flash_make_cmd(maps, cmd, &cword);

    switch (maps->portwidth)
    {
    case CFI_FLASH_8BIT:
        *addr.cp = cword.c;
        break;

    case CFI_FLASH_16BIT:
        *addr.wp = cword.w;
        break;

    default:
        break;
    }
}

static rt_bool_t cfi_flash_isequ(struct cfi_flash_map *maps, union cfiptr cptr, rt_uint8_t value)
{
    switch (maps->portwidth)
    {
    case CFI_FLASH_8BIT:
        return cptr.cp[0] == value;

    case CFI_FLASH_16BIT:
        return cptr.wp[0] == value;

    default:
        return RT_FALSE;
    }
}

static rt_bool_t cfi_flash_isset(struct cfi_flash_map *maps, union cfiptr cptr, rt_uint8_t value)
{
    switch (maps->portwidth)
    {
    case CFI_FLASH_8BIT:
        return (cptr.cp[0] & value) == value;

    case CFI_FLASH_16BIT:
        return (cptr.wp[0] & value) == value;

    default:
        return RT_FALSE;
    }
}

static void cfi_flash_add_byte(struct cfi_flash_map *maps, union cfiword *cword, rt_uint8_t c)
{
    switch (maps->portwidth)
    {
    case CFI_FLASH_8BIT:
        cword->c = c;
        break;

    case CFI_FLASH_16BIT:
    #ifndef ARCH_CPU_BIG_ENDIAN
        {
            rt_uint16_t w = c;
            w <<= 8;
            cword->w = (cword->w >> 8) | w;
        }
    #else
        cword->w = (cword->w << 8) | c;
    #endif
        break;

    default:
        break;
    }
}

static rt_ubase_t cfi_find_sector(struct cfi_flash_map *maps, rt_ubase_t addr)
{
    rt_ubase_t sector;

    for (sector = maps->sect_count - 1; sector >= 0; --sector)
    {
        if (addr >= maps->sect[sector])
        {
            break;
        }
    }

    return sector;
}

static void cfi_flash_unlock_seq(struct cfi_flash_map *maps, rt_int32_t sect)
{
    cfi_flash_write_cmd(maps, sect, CFI_AMD_ADDR_START, CFI_AMD_CMD_UNLOCK_START);
    cfi_flash_write_cmd(maps, sect, CFI_AMD_ADDR_ACK, CFI_AMD_CMD_UNLOCK_ACK);
}

static rt_err_t cfi_flash_status_check(struct cfi_flash_map *maps, union cfiptr cptr,
        union cfiword *cword, rt_uint32_t tout, char *prompt)
{
    rt_int32_t retry;
    rt_int32_t ready = 0;
    rt_err_t err = RT_EOK;
    rt_uint16_t data, lastdata;

    retry = ERASE_POLL_LIMIT;

    lastdata = cptr.wp[0];

    while (!ready && retry-- >= 0)
    {
        data = *cptr.wp;

        /* test to see if bit6 is NOT toggling */
        if ((data & 0xffff) == (lastdata & 0xffff))
        {
            ready = 1;
        }
        else
        {
            if (data & 0x0020)
            {
                lastdata = *cptr.wp;
                data = *cptr.wp;

                /* test to see if bit6 is toggling */
                if ((data & 0xffff) != (lastdata & 0xffff))
                {
                    err = -RT_ERROR;
                }
            }
        }

        lastdata = data;
    }

    if (err || ready == 0)
    {
        LOG_E("error wait for flash ready, status = 0x%04x", data);

        err = -RT_ERROR;
    }

    return err;
}

static rt_err_t cfi_flash_full_status_check(struct cfi_flash_map *maps,
        union cfiptr cptr, union cfiword *cword, rt_uint32_t tout, char *prompt)
{
    rt_err_t err;

    err = cfi_flash_status_check(maps, cptr, cword, tout, prompt);

    switch (maps->vendor)
    {
    case CFI_CMDSET_INTEL_EXTENDED:
    case CFI_CMDSET_INTEL_STANDARD:
        if (err && !cfi_flash_isequ(maps, cptr, CFI_FLASH_STATUS_DONE))
        {
            err = -RT_EINVAL;

            LOG_E("flash %s error at address %8x", prompt, (rt_uint32_t)(rt_ubase_t)cptr.cp);

            if (cfi_flash_isset(maps, cptr, CFI_FLASH_STATUS_ECLBS | CFI_FLASH_STATUS_PSLBS))
            {
                LOG_E("command Sequence Error");
            }
            else if (cfi_flash_isset(maps, cptr, CFI_FLASH_STATUS_ECLBS))
            {
                LOG_E("block Erase Error");

                err = -RT_ERROR;
            }
            else if (cfi_flash_isset(maps, cptr, CFI_FLASH_STATUS_PSLBS))
            {
                LOG_E("locking Error");
            }

            if (cfi_flash_isset(maps, cptr, CFI_FLASH_STATUS_DPS))
            {
                LOG_E("block locked");

                err = -RT_EBUSY;
            }

            if (cfi_flash_isset(maps, cptr, CFI_FLASH_STATUS_VPENS))
            {
                LOG_E("vpp Low Error");
            }
        }
        break;

    case CFI_CMDSET_AMD_STANDARD:
    case CFI_CMDSET_AMD_EXTENDED:
        if (err)
        {
            LOG_E("error flash status");
        }
        break;

    default:
        break;
    }

    cfi_flash_write_cmd(maps, 0, 0, maps->cmd_reset);

    return err;
}

static rt_ssize_t cfi_flash_write_cfiword(struct cfi_flash_map *maps, rt_ubase_t dest, union cfiword *cword)
{
    rt_int32_t flag;
    union cfiptr cptr;

    cfi_flash_make_addr(maps, 0, 0);
    cptr.cp = (rt_uint8_t *)dest;

    /* check if Flash is (sufficiently) erased */
    switch (maps->portwidth)
    {
    case CFI_FLASH_8BIT:
        flag = (cptr.cp[0] & cword->c) == cword->c;
        break;

    case CFI_FLASH_16BIT:
        flag = (cptr.wp[0] & cword->w) == cword->w;
        break;

    default:
        return 2;
    }

    if (!flag)
    {
        return 2;
    }

    switch (maps->vendor)
    {
    case CFI_CMDSET_INTEL_EXTENDED:
    case CFI_CMDSET_INTEL_STANDARD:
        cfi_flash_write_cmd(maps, 0, 0, CFI_FLASH_CMD_CLEAR_STATUS);
        cfi_flash_write_cmd(maps, 0, 0, CFI_FLASH_CMD_WRITE);
        break;

    case CFI_CMDSET_AMD_EXTENDED:
    case CFI_CMDSET_AMD_STANDARD:
        cfi_flash_unlock_seq(maps, 0);
        cfi_flash_write_cmd(maps, 0, CFI_AMD_ADDR_START, CFI_AMD_CMD_WRITE);
        break;

    default:
        break;
    }

    switch (maps->portwidth)
    {
    case CFI_FLASH_8BIT:
        cptr.cp[0] = cword->c;
        break;

    case CFI_FLASH_16BIT:
        cptr.wp[0] = cword->w;
        break;

    default:
        break;
    }

    return cfi_flash_full_status_check(maps, cptr, cword, maps->write_tout, "write");
}

static rt_err_t cfi_flash_write_cfibuffer(struct cfi_flash_map *maps, rt_ubase_t dest, rt_uint8_t *cp, rt_size_t len)
{
    rt_err_t err;
    rt_int32_t sector, cnt;
    union cfiptr src, dst;
    union cfiword cword;

    switch (maps->vendor)
    {
    case CFI_CMDSET_INTEL_STANDARD:
    case CFI_CMDSET_INTEL_EXTENDED:
        cword.c = 0xff;
        src.cp = cp;
        dst.cp = (rt_uint8_t *)dest;
        sector = cfi_find_sector(maps, dest);

        cfi_flash_write_cmd(maps, sector, 0, CFI_FLASH_CMD_CLEAR_STATUS);
        cfi_flash_write_cmd(maps, sector, 0, CFI_FLASH_CMD_WRITE_TO_BUFFER);

        if (!(err = cfi_flash_status_check(maps, dst, &cword, maps->buffer_write_tout, "write to buffer")))
        {
            /* reduce the number of loops by the width of the port */
            switch (maps->portwidth)
            {
            case CFI_FLASH_8BIT:
                cnt = len;
                break;
            case CFI_FLASH_16BIT:
                cnt = len >> 1;
                break;

            default:
                return -RT_EINVAL;
            }

            if (maps->portwidth == CFI_FLASH_8BIT)
            {
                cfi_flash_write_cmd(maps, sector, 0, (rt_uint8_t)cnt - 1);
            }
            else
            {
                HWREG8(cfi_flash_make_addr(maps, sector, 0)) = (rt_uint8_t)cnt - 1;
            }

            while (cnt-- > 0)
            {
                switch (maps->portwidth)
                {
                case CFI_FLASH_8BIT:
                    *dst.cp++ = *src.cp++;
                    break;

                case CFI_FLASH_16BIT:
                    *dst.wp++ = *src.wp++;
                    break;

                default:
                    return -RT_EINVAL;
                    break;
                }
            }

            cfi_flash_write_cmd(maps, sector, 0, CFI_FLASH_CMD_WRITE_BUFFER_CONFIRM);

            cword.c = 0xff;
            dst.cp = (rt_uint8_t *)((rt_ubase_t)dst.cp - maps->portwidth);

            err = cfi_flash_full_status_check(maps, dst, &cword, maps->buffer_write_tout, "buffer write");
        }
        return err;

    case CFI_CMDSET_AMD_STANDARD:
    case CFI_CMDSET_AMD_EXTENDED:
        src.cp = cp;
        dst.cp = (rt_uint8_t *)dest;
        sector = cfi_find_sector(maps, dest);

        cfi_flash_unlock_seq(maps, 0);
        cfi_flash_write_cmd(maps, sector, 0, CFI_AMD_CMD_WRITE_TO_BUFFER);

        switch (maps->portwidth)
        {
        case CFI_FLASH_8BIT:
            cnt = len;
            cfi_flash_write_cmd(maps, sector, 0, (rt_uint8_t)cnt - 1);
            while (cnt-- > 0)
            {
                *dst.cp++ = *src.cp++;
            }
            break;

        case CFI_FLASH_16BIT:
            cnt = len >> 1;
            cfi_flash_write_cmd(maps, sector, 0, (rt_uint8_t)cnt - 1);
            while (cnt-- > 0)
            {
                *dst.wp++ = *src.wp++;
            }
            break;

        default:
            return -RT_EINVAL;
        }

        cfi_flash_write_cmd(maps, sector, 0, CFI_AMD_CMD_WRITE_BUFFER_CONFIRM);
        cword.c = 0xff;
        dst.cp -= maps->portwidth;

        return cfi_flash_full_status_check(maps, dst, &cword, maps->buffer_write_tout, "buffer write");

    default:
        LOG_E("unknown command set");

        return -RT_EINVAL;
    }
}

static void cfi_flash_reset(struct cfi_flash_map *maps)
{
    /*
     * We do not yet know what kind of commandset to use, so we issue
     * the reset command in both Intel and AMD variants, in the hope
     * that AMD flash roms ignore the Intel command.
     */
    cfi_flash_write_cmd(maps, 0, 0, CFI_AMD_CMD_RESET);
    rt_hw_us_delay(1);
    cfi_flash_write_cmd(maps, 0, 0, CFI_FLASH_CMD_RESET);
}

static rt_err_t cfi_flash_detect(struct cfi_flash *cfi, int map)
{
    union cfiptr cptr1, cptr2, cptr3;
    struct cfi_flash_map *maps = &cfi->maps[map];

    cfi_flash_reset(maps);

    for (maps->portwidth = CFI_FLASH_8BIT;
        maps->portwidth <= CFI_FLASH_16BIT;
        maps->portwidth <<= 1)
    {
        for (maps->chipwidth = CFI_FLASH_BY8;
            maps->chipwidth <= maps->portwidth;
            maps->chipwidth <<= 1)
        {
            cptr1.cp = cfi_flash_make_addr(maps, 0, CFI_FLASH_OFFSET_CFI_RESP);
            cptr2.cp = cfi_flash_make_addr(maps, 0, CFI_FLASH_OFFSET_CFI_RESP + 1);
            cptr3.cp = cfi_flash_make_addr(maps, 0, CFI_FLASH_OFFSET_CFI_RESP + 2);

            cfi_flash_write_cmd(maps, 0, CFI_FLASH_OFFSET_CFI, CFI_FLASH_CMD_CFI);

            if (cfi_flash_isequ(maps, cptr1, 'Q') &&
                cfi_flash_isequ(maps, cptr2, 'R') &&
                cfi_flash_isequ(maps, cptr3, 'Y'))
            {
                maps->interface = cfi_flash_read_u16(maps, 0, CFI_FLASH_OFFSET_INTERFACE);

                return RT_EOK;
            }
        }
    }

    return -RT_EINVAL;
}

static rt_err_t cfi_flash_read_id(struct rt_mtd_nor_device *dev)
{
    struct cfi_flash_map *maps = raw_to_cfi_flash_map(dev);

    return maps->vendor;
}

static rt_ssize_t cfi_flash_read(struct rt_mtd_nor_device *dev, rt_off_t offset, rt_uint8_t *data, rt_size_t length)
{
    struct cfi_flash_map *maps = raw_to_cfi_flash_map(dev);

    rt_mutex_take(&maps->rw_lock, RT_WAITING_FOREVER);

    cfi_flash_write_cmd(maps, offset / maps->block_size, offset % maps->block_size, maps->cmd_reset);

    rt_memcpy(data, ((const void *)(maps->base + offset)), length);

    rt_mutex_release(&maps->rw_lock);

    return length;
}

static rt_ssize_t cfi_flash_write(struct rt_mtd_nor_device *dev, rt_off_t offset,
        const rt_uint8_t *data, rt_size_t length)
{
    rt_err_t err;
    rt_ubase_t wp, cp;
    rt_int32_t byte_idx;
    union cfiword cword;
    rt_ubase_t i, align, buffered_size;
    struct cfi_flash_map *maps = raw_to_cfi_flash_map(dev);

    rt_mutex_take(&maps->rw_lock, RT_WAITING_FOREVER);

    /* get lower aligned address */
    offset += (rt_ubase_t)maps->base;
    wp = (offset & ~((rt_ubase_t)maps->portwidth - 1));

    for (byte_idx = 0; byte_idx < maps->portwidth; ++byte_idx)
    {
        if (*(rt_uint8_t *)wp != (rt_uint8_t)0xff)
        {
            length = -RT_ERROR;
            LOG_E("flash is not erased on the address %p, read %p", wp, *(rt_uint8_t *)wp);

            goto _out_lock;
        }
    }

    /* handle unaligned start */
    if ((align = offset - wp) != 0)
    {
        cword.l = 0;
        cp = wp;

        for (i = 0; i < align; ++i, ++cp)
        {
            cfi_flash_add_byte(maps, &cword, *(rt_uint8_t *)cp);
        }

        for (; (i < maps->portwidth) && (length > 0); i++)
        {
            cfi_flash_add_byte(maps, &cword, *data++);

            --length;
            ++cp;
        }

        for (; (length == 0) && (i < maps->portwidth); ++i, ++cp)
        {
            cfi_flash_add_byte(maps, &cword, *(rt_uint8_t *)cp);
        }

        if ((err = cfi_flash_write_cfiword(maps, wp, &cword)))
        {
            length = err;

            goto _out_lock;
        }

        wp = cp;
    }

    /* handle the aligned part */
    if (maps->use_buffer_write)
    {
        buffered_size = (maps->portwidth / maps->chipwidth);
        buffered_size *= maps->buffer_size;

        while (length >= maps->portwidth)
        {
            /* prohibit buffer write when buffer_size is 1 */
            if (maps->buffer_size == 1 || length == maps->portwidth)
            {
                cword.l = 0;

                for (i = 0; i < maps->portwidth; i++)
                {
                    cfi_flash_add_byte(maps, &cword, *data++);
                }

                if ((err = cfi_flash_write_cfiword(maps, wp, &cword)))
                {
                    length = err;

                    goto _out_lock;
                }

                wp += maps->portwidth;
                length -= maps->portwidth;

                continue;
            }

            /* write buffer until next buffered_size aligned boundary */
            i = buffered_size - (wp % buffered_size);

            if (i > length)
            {
                i = length;
            }

            if ((err = cfi_flash_write_cfibuffer(maps, wp, (rt_uint8_t *)data, i)))
            {
                length = err;

                goto _out_lock;
            }

            i -= i & (maps->portwidth - 1);
            wp += i;
            data += i;
            length -= i;
        }
    }
    else
    {
        while (length >= maps->portwidth)
        {
            cword.l = 0;

            for (i = 0; i < maps->portwidth; i++)
            {
                cfi_flash_add_byte(maps, &cword, *data++);
            }

            if ((err = cfi_flash_write_cfiword(maps, wp, &cword)))
            {
                length = err;

                goto _out_lock;
            }

            wp += maps->portwidth;
            length -= maps->portwidth;

            for (byte_idx = 0; byte_idx < maps->portwidth; byte_idx++)
            {
                if (*(rt_uint8_t *)wp != (rt_uint8_t)0xff)
                {
                    length = -RT_ERROR;
                    LOG_E("flash is not erased on the address %p, %p", wp, *(rt_uint8_t *)wp);

                    goto _out_lock;
                }
            }
        }
    }

    if (!length)
    {
        goto _out_lock;
    }

    /* handle unaligned tail bytes */
    cword.l = 0;

    for (i = 0, cp = wp; i < maps->portwidth && length > 0; ++i, ++cp)
    {
        cfi_flash_add_byte(maps, &cword, *data++);

        --length;
    }

    for (; i < maps->portwidth; ++i, ++cp)
    {
        cfi_flash_add_byte(maps, &cword, *(rt_uint8_t *)cp);
    }

    if ((err = cfi_flash_write_cfiword(maps, wp, &cword)))
    {
        length = err;
    }

_out_lock:
    rt_mutex_release(&maps->rw_lock);

    return length;
}

static rt_err_t cfi_flash_erase_block(struct rt_mtd_nor_device *dev, rt_off_t offset, rt_size_t length)
{
    union cfiptr cptr;
    union cfiword cword = { .c = 0xff };
    rt_int32_t sect;
    rt_int32_t prot = 0;
    rt_err_t err = RT_EOK;
    struct cfi_flash_map *maps = raw_to_cfi_flash_map(dev);
    rt_off_t sect_start = offset / maps->block_size, sect_end = (offset + length) / maps->block_size;

    rt_mutex_take(&maps->rw_lock, RT_WAITING_FOREVER);

    for (sect = sect_start; sect <= sect_end; ++sect)
    {
        if (maps->protect[sect])
        {
            ++prot;
        }
    }

    if (prot)
    {
        LOG_W("%d protected sectors will not be erased", prot);

        err = -RT_EIO;
        goto _out_lock;
    }

    if (sect_start == 0 && sect_end == (maps->sect_count - 1)
        && (maps->vendor == CFI_CMDSET_AMD_STANDARD || maps->vendor == CFI_CMDSET_AMD_EXTENDED))
    {
        /* Erase chip */
        sect = sect_start;

        cfi_flash_unlock_seq(maps, sect);
        cfi_flash_write_cmd(maps, sect, CFI_AMD_ADDR_ERASE_START, CFI_AMD_CMD_ERASE_START);
        cfi_flash_unlock_seq(maps, sect);
        cfi_flash_write_cmd(maps, sect, CFI_AMD_ADDR_ERASE_START, CFI_AMD_CMD_ERASE_CHIP);

        cptr.cp = cfi_flash_make_addr(maps, sect, 0);

        if (cfi_flash_full_status_check(maps, cptr, &cword, maps->erase_chip_tout, "chip erase"))
        {
            err = -RT_ERROR;
            goto _out_lock;
        }
    }
    else
    {
        for (sect = sect_start; sect <= sect_end; sect++)
        {
            /* not protected */
            if (maps->protect[sect] == 0)
            {
                switch (maps->vendor)
                {
                case CFI_CMDSET_INTEL_STANDARD:
                case CFI_CMDSET_INTEL_EXTENDED:
                    cfi_flash_write_cmd(maps, sect, 0, CFI_FLASH_CMD_CLEAR_STATUS);
                    cfi_flash_write_cmd(maps, sect, 0, CFI_FLASH_CMD_BLOCK_ERASE);
                    cfi_flash_write_cmd(maps, sect, 0, CFI_FLASH_CMD_ERASE_CONFIRM);
                    break;

                case CFI_CMDSET_AMD_STANDARD:
                case CFI_CMDSET_AMD_EXTENDED:
                    cfi_flash_unlock_seq(maps, sect);
                    cfi_flash_write_cmd(maps, sect, CFI_AMD_ADDR_ERASE_START, CFI_AMD_CMD_ERASE_START);
                    cfi_flash_unlock_seq(maps, sect);
                    cfi_flash_write_cmd(maps, sect, 0, CFI_AMD_CMD_ERASE_SECTOR);
                    break;

                default:
                    LOG_E("unkown flash vendor %d", maps->vendor);
                    break;
                }

                cptr.cp = cfi_flash_make_addr(maps, sect, 0);

                if (cfi_flash_full_status_check(maps, cptr, &cword, maps->erase_blk_tout, "sector erase"))
                {
                    err = -RT_ERROR;
                    goto _out_lock;
                }
            }
        }
    }

_out_lock:
    rt_mutex_release(&maps->rw_lock);

    return err;
}

const static struct rt_mtd_nor_driver_ops cfi_flash_ops =
{
    .read_id = cfi_flash_read_id,
    .read = cfi_flash_read,
    .write = cfi_flash_write,
    .erase_block = cfi_flash_erase_block,
};

static rt_err_t cfi_flash_probe(struct rt_platform_device *pdev)
{
    const char *name;
    rt_err_t err = RT_EOK;
    struct rt_device *dev = &pdev->parent;
    struct cfi_flash *cfi = rt_calloc(1, sizeof(*cfi));

    if (!cfi)
    {
        return -RT_ENOMEM;
    }

    cfi->maps_nr = rt_dm_dev_get_address_count(dev);

    if (cfi->maps_nr <= 0)
    {
        err = -RT_EEMPTY;

        goto _fail;
    }

    if ((err = rt_dm_dev_prop_read_u32(dev, "bank-width", &cfi->bank_width)))
    {
        goto _fail;
    }

    if (!(cfi->maps = rt_calloc(1, sizeof(*cfi->maps) * cfi->maps_nr)))
    {
        err = -RT_ENOMEM;

        goto _fail;
    }

    for (int i = 0; i < cfi->maps_nr; ++i)
    {
        struct cfi_flash_map *maps = &cfi->maps[i];
        rt_int32_t size_ratio, offset_etout, offset_wbtout, wtout;
        rt_uint64_t address, size;
        rt_size_t sect_count = 0;
        rt_ubase_t sector;
        /* map a page early first */
        void *early_base;

        if (rt_dm_dev_get_address(dev, i, &address, &size) < 0)
        {
            err = -RT_EIO;

            goto _fail;
        }

        maps->address = address;
        maps->size = size;

        early_base = rt_ioremap((void *)address, RT_MM_PAGE_SIZE);

        if (!early_base)
        {
            LOG_E("flash [%p, %p] %s", (rt_ubase_t)address, (rt_ubase_t)(address + size), "iomap fail");

            continue;
        }

        maps->base = early_base;

        if (cfi_flash_detect(cfi, i))
        {
            err = -RT_ERROR;
            LOG_D("flash [%p, %p] %s", (rt_ubase_t)address, (rt_ubase_t)(address + size), "not found");

            goto _fail;
        }

        maps->base = rt_ioremap((void *)address, size);

        if (!maps->base)
        {
            err = -RT_ERROR;
            LOG_E("flash [%p, %p] %s", (rt_ubase_t)address, (rt_ubase_t)(address + size), "iomap fail");

            goto _fail;
        }

        rt_iounmap(early_base);

        maps->num_erase_regions = cfi_flash_read_u8(maps, CFI_FLASH_OFFSET_NUM_ERASE_REGIONS);

        for (int seq = 0; seq < maps->num_erase_regions; ++seq)
        {
            rt_uint32_t erase_regions;

            erase_regions = cfi_flash_read_u32(maps, 0, CFI_FLASH_OFFSET_ERASE_REGIONS + seq * 4);
            sect_count += ((erase_regions >> 16) & 0xffff) + 1;
        }

        maps->sect = rt_malloc(sizeof(rt_ubase_t) * sect_count * 2);

        if (!maps->sect)
        {
            goto _fail;
        }

        maps->protect = &maps->sect[sect_count];
        maps->sect_count = sect_count;

        maps->buffer_size = (1 << cfi_flash_read_u16(maps, 0, CFI_FLASH_OFFSET_BUFFER_SIZE));

        offset_etout = 1 << cfi_flash_read_u8(maps, CFI_FLASH_OFFSET_ETOUT);
        maps->erase_blk_tout = (offset_etout * (1 << cfi_flash_read_u8(maps, CFI_FLASH_OFFSET_EMAX_TOUT)));
        maps->erase_chip_tout = maps->erase_blk_tout * maps->sect_count;

        offset_wbtout = 1 << cfi_flash_read_u8(maps, CFI_FLASH_OFFSET_WBTOUT);
        maps->buffer_write_tout = (offset_wbtout * (1 << cfi_flash_read_u8(maps, CFI_FLASH_OFFSET_WBMAX_TOUT)));

        wtout = 1 << cfi_flash_read_u8(maps, CFI_FLASH_OFFSET_WTOUT);
        maps->write_tout = (wtout * (1 << cfi_flash_read_u8(maps, CFI_FLASH_OFFSET_WMAX_TOUT))) / 1000;

        size_ratio = maps->portwidth / maps->chipwidth;

        if (maps->interface == CFI_FLASH_X8X16 && maps->chipwidth == CFI_FLASH_BY8)
        {
            size_ratio >>= 1;
        }

        maps->block_size = (1 << cfi_flash_read_u8(maps, CFI_FLASH_OFFSET_SIZE)) * size_ratio;

        maps->vendor = cfi_flash_read_u16(maps, 0, CFI_FLASH_OFFSET_PRIMARY_VENDOR);

        switch (maps->vendor)
        {
        case CFI_CMDSET_AMD_STANDARD:
        case CFI_CMDSET_AMD_EXTENDED:
            maps->use_buffer_write = RT_TRUE;
            maps->cmd_reset = CFI_AMD_CMD_RESET;
            break;

        case CFI_CMDSET_INTEL_STANDARD:
        case CFI_CMDSET_INTEL_EXTENDED:
            maps->use_buffer_write = RT_TRUE;
            maps->cmd_reset = CFI_FLASH_CMD_RESET;
            break;

        default:
            maps->cmd_reset = CFI_FLASH_CMD_RESET;
            break;
        }

        sector = (rt_ubase_t)maps->base;

        for (int seq = sect_count = 0; seq < maps->num_erase_regions; ++seq)
        {
            rt_uint32_t erase_regions, erase_region_size, erase_region_count;

            erase_regions = cfi_flash_read_u32(maps, 0, CFI_FLASH_OFFSET_ERASE_REGIONS + seq * 4);
            erase_region_size = (erase_regions & 0xffff) ? ((erase_regions & 0xffff) * 256) : 128;
            erase_region_count = ((erase_regions >> 16) & 0xffff) + 1;

            while (erase_region_count --> 0)
            {
                union cfiptr cptr;

                maps->sect[sect_count] = sector;
                sector += (erase_region_size * size_ratio);

                switch (maps->vendor)
                {
                case CFI_CMDSET_INTEL_EXTENDED:
                case CFI_CMDSET_INTEL_STANDARD:
                    cptr.cp = cfi_flash_make_addr(maps, sect_count, CFI_FLASH_OFFSET_PROTECT);
                    maps->protect[sect_count] = cfi_flash_isset(maps, cptr, CFI_FLASH_STATUS_PROTECT);
                    break;

                default:
                    /* default: not protected */
                    maps->protect[sect_count] = 0;
                    break;
                }

                ++sect_count;
            }
        }

        LOG_I("Flash partitions@%d:", i);
        LOG_I("  port width:    %d", (maps->portwidth << 3));
        LOG_I("  chip width:    %d", (maps->chipwidth << 3));
        LOG_I("  buffer size:   %d", maps->buffer_size);
        LOG_I("  sector count:  %d", maps->sect_count);
        LOG_I("  block size:    %d", maps->block_size);
        LOG_I("  mmio region:   [%p, %p]", (rt_ubase_t)maps->address, (rt_ubase_t)(maps->address + maps->size));
        LOG_I("  command set:   %s", maps->vendor < RT_ARRAY_SIZE(vendor_name) ? vendor_name[maps->vendor] : RT_NULL);

        maps->parent.ops = &cfi_flash_ops;
        maps->parent.block_start = 0;
        maps->parent.block_end = maps->sect_count;
        maps->parent.block_size = maps->block_size;

        if ((err = rt_dm_dev_set_name_auto(&maps->parent.parent, "nor")) < 0)
        {
            goto _fail;
        }

        name = rt_dm_dev_get_name(&maps->parent.parent);

        if ((err = rt_mutex_init(&maps->rw_lock, name, RT_IPC_FLAG_PRIO)))
        {
            goto _fail;
        }

        if ((err = rt_mtd_nor_register_device(name, &maps->parent)))
        {
            goto _fail;
        }
    }

    dev->user_data = cfi;

    return RT_EOK;

_fail:
    if (cfi->maps)
    {
        for (int i = 0; i < cfi->maps_nr; ++i)
        {
            struct cfi_flash_map *maps = &cfi->maps[i];

            if (maps->base)
            {
                rt_iounmap(maps->base);
            }

            if (maps->sect)
            {
                rt_free(maps->sect);
            }
        }

        rt_free(cfi->maps);
    }
    rt_free(cfi);

    return err;
}

static rt_err_t cfi_flash_remove(struct rt_platform_device *pdev)
{
    struct cfi_flash *cfi = pdev->parent.user_data;

    if (cfi->maps)
    {
        for (int i = 0; i < cfi->maps_nr; ++i)
        {
            struct cfi_flash_map *maps = &cfi->maps[i];

            if (maps->base)
            {
                rt_iounmap(maps->base);
            }

            if (maps->sect)
            {
                rt_free(maps->sect);
            }
        }

        rt_device_unregister(&cfi->maps->parent.parent);

        rt_free(cfi->maps);
    }

    rt_free(cfi);

    return RT_EOK;
}

static const struct rt_ofw_node_id cfi_flash_ofw_ids[] =
{
    { .compatible = "cfi-flash" },
    { /* sentinel */ }
};

static struct rt_platform_driver cfi_flash_driver =
{
    .name = "cfi-flash",
    .ids = cfi_flash_ofw_ids,

    .probe = cfi_flash_probe,
    .remove = cfi_flash_remove,
};
RT_PLATFORM_DRIVER_EXPORT(cfi_flash_driver);
