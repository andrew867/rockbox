/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * S5L8740 NAND controller (FMSS/FIL) -- raw PPN page I/O.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/
#ifndef __NAND_S5L8740_H__
#define __NAND_S5L8740_H__

#include <stdbool.h>
#include <stdint.h>

/*
 * Terminology, because three different "LBA"s are in play and mixing them up
 * is the single easiest way to get lost in this stack:
 *
 *   fmss_lba      the logical block number written in the 16-byte CS metadata
 *   disk_lba      the LBA of the exported FAT volume, what Rockbox asks for
 *   fat_base_lba  the fmss_lba that holds the FAT32 BPB, i.e. disk_lba 0
 *
 *   target_fmss_lba = fat_base_lba + disk_lba
 *
 * A "CS page" is one physical NAND page presented as four 4096-byte data
 * slots plus four 16-byte metadata slots, transferred as records of 4112
 * bytes each.
 */
#define NAND_FTL_SECTOR_SIZE        4096U
#define NAND_FTL_SECTORS_PER_LPN    4U
#define NAND_FTL_DEFAULT_CAPACITY   3856968U

#define NAND_MAX_CE                 2U
#define NAND_MAX_CAU                2U
#define NAND_PAGE_SIZE              16384U
#define NAND_META_SIZE              64U     /* 4 x 16-byte SFTL slots */
#define NAND_SLOTS_PER_PAGE         4U
#define NAND_SLOT_DATA              4096U
#define NAND_SLOT_META              16U
#define NAND_REC_BYTES              4112U   /* 4096 data + 16 meta */

/* PPN address packing, from the param page. */
#define NAND_PAGE_BITS              7
#define NAND_BLOCK_BITS             12
#define NAND_CAU_BITS               1
#define NAND_BLOCKS_PER_CAU         2088
#define NAND_PAGES_PER_BLOCK        128
#define NAND_BTOC_PAGE              127     /* last page of a block */
#define NAND_VBAS_PER_PAGE          4

/* CS metadata record types. */
#define NAND_META_TYPE_DATA         0x01u
#define NAND_META_TYPE_DATA2        0x02u
#define NAND_META_TYPE_BTOC         0x1cu
#define NAND_META_TYPE_SFTL_CXT     0x1fu
#define NAND_META_TYPE_VFL_CXT      0x20u

/* One decoded 16-byte CS metadata slot. */
struct nand_meta {
    uint8_t  type;
    uint8_t  flags;
    uint64_t weave;     /* 48-bit sequence number, little-endian at +2 */
    uint32_t lba;       /* fmss_lba, little-endian at +8 */
    bool     valid;
    bool     blank;
};

/* One CS physical page: four data slots and four metadata slots. */
struct nand_cs_page {
    uint8_t data[NAND_SLOTS_PER_PAGE][NAND_SLOT_DATA];
    uint8_t meta_raw[NAND_SLOTS_PER_PAGE][NAND_SLOT_META];
    struct nand_meta meta[NAND_SLOTS_PER_PAGE];
};

bool nand_hw_init(void);
bool nand_hw_present(void);
int  nand_hw_reset(void);

/*
 * Read one physical page as four data + four metadata records.
 * Always slot 0, span 4, rec 4112 -- the glass-proven CS ABI.
 */
int nand_cs_phys_read(uint8_t ce, uint8_t cau, uint16_t block, uint8_t page,
                      struct nand_cs_page *out);

/*
 * True only if the page is conclusively erased. Reads one record instead of
 * four, so it costs a quarter of a full page read -- but it returns a verdict
 * rather than data, and anything it cannot settle must be re-read in full.
 */
bool nand_cs_probe_empty(uint8_t ce, uint8_t cau, uint16_t block,
                         uint8_t page);

void nand_meta_decode(const uint8_t *m16, struct nand_meta *out);

static inline bool nand_meta_is_data(const struct nand_meta *m)
{
    if (!m || !m->valid || m->blank)
        return false;
    return m->type == NAND_META_TYPE_DATA ||
           m->type == NAND_META_TYPE_DATA2;
}

/*
 * Among four decoded records, the newest-weave slot claiming fmss_lba.
 * Returns 0..3, or -1 if none claims it.
 */
int nand_meta_pick_lba(const struct nand_cs_page *page, uint32_t fmss_lba);

/* Pack a physical address the way the sequencer wants it. */
static inline uint32_t nand_ppn_addr(unsigned cau, unsigned block,
                                     unsigned page, unsigned slc)
{
    return (uint32_t)page
         | ((uint32_t)block << NAND_PAGE_BITS)
         | ((uint32_t)cau << (NAND_PAGE_BITS + NAND_BLOCK_BITS))
         | ((uint32_t)slc << (NAND_PAGE_BITS + NAND_BLOCK_BITS + NAND_CAU_BITS));
}

#endif /* __NAND_S5L8740_H__ */
