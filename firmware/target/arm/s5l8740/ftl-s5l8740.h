/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Apple Whimory / SFTL flash translation layer for the iPod nano 7G.
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
#ifndef __FTL_S5L8740_H__
#define __FTL_S5L8740_H__

#include <stdbool.h>
#include <stdint.h>

/*
 * A superblock is one block index taken across every (ce, cau) bank. The FTL
 * writes sequentially within a superblock and stamps each page with a weave
 * sequence number, so "newest wins" is decided by weave, never by position.
 */
#define FTL_SB_KIND_UNKNOWN 0
#define FTL_SB_KIND_CLOSED  1   /* has a valid BTOC on its last page */
#define FTL_SB_KIND_OPEN    2   /* still being written; needs a page scan */
#define FTL_SB_KIND_FREE    3

/*
 * L2V is stored as coalesced ranges rather than one entry per LBA. A flat
 * table would be ~3.8M entries; the real mapping is overwhelmingly
 * contiguous, so ranges keep it small enough to live in DRAM.
 */
struct ftl_range {
    uint32_t start;     /* first fmss_lba */
    uint32_t len;       /* how many consecutive LBAs */
    uint32_t vba;       /* virtual block address of the first one */
    uint64_t weave;     /* which write won this range */
};

bool ftl_init(void);

/* Scan the device and rebuild L2V. Slow -- this walks every superblock. */
int ftl_recover(void);

bool ftl_ready(void);

/* Run the map rebuild on demand (debug menu), not during boot. */
int ftl_mount_now(void);

/* Resolve an fmss_lba to a physical slot. Returns 0 on success. */
int ftl_lookup(uint32_t fmss_lba, uint8_t *ce, uint8_t *cau, uint16_t *block,
               uint8_t *page, uint8_t *slot, uint64_t *weave);

/* Read one 4096-byte logical sector by fmss_lba. */
int ftl_read_fmss_lba(uint32_t fmss_lba, void *buf);

/* Read one 4096-byte sector of the exported FAT volume. */
int ftl_read_disk_lba(uint32_t disk_lba, void *buf);

/* fmss_lba of disk_lba 0 -- found by locating the FAT32 BPB. */
uint32_t ftl_fat_base_lba(void);
uint32_t ftl_disk_sectors(void);

/* Diagnostics for the debug screen. */
void ftl_get_cxt_stats(bool *loaded, unsigned *written, unsigned *empty,
                       unsigned *replayed);
const char *ftl_last_phase(void);

void ftl_get_stats(unsigned *ranges, unsigned *mapped, unsigned *sbs_closed,
                   unsigned *sbs_open);
const char *ftl_progress_phase(unsigned *cur, unsigned *total);

#endif /* __FTL_S5L8740_H__ */
