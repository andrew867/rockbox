/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Apple Whimory / SFTL flash translation layer for the iPod nano 7G.
 *
 * Ported from tools/linux-n31/drivers/ftl-s5l8740.c and
 * ftl-s5l8740-csmap.c.
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
#include <string.h>
#include "config.h"
#include "system.h"
#include "kernel.h"
#include "lcd.h"
#include "nand-s5l8740.h"
#include "ftl-s5l8740.h"

/*
 * ---------------------------------------------------------------------
 * How this works, because the shape is not obvious
 *
 * Whimory never overwrites in place. A logical sector is written to whatever
 * physical slot is next, stamped with a monotonically increasing 48-bit
 * "weave" sequence number, and the old copy is left behind until garbage
 * collection reclaims it. So at any moment several physical slots can claim
 * the same logical LBA, and the ONLY thing that distinguishes the live one is
 * that it has the highest weave. Position on the medium means nothing.
 *
 * Mounting therefore means reconstructing logical -> physical from scratch by
 * reading what the device wrote about itself.
 *
 * Blocks are grouped into superblocks: one block index taken across every
 * (ce, cau) bank. A superblock is written sequentially, and when it fills, a
 * block table of contents (BTOC) is written to its last page listing which
 * logical page each slot holds. Recovering a closed superblock is then one
 * page read instead of 128.
 *
 * Open superblocks -- the one or two still being written -- have no BTOC, so
 * they are walked page by page and the per-slot metadata is taken as the
 * authority. That metadata (type, weave, lba) is the best-proven part of this
 * whole stack, which is why it also validates anything the BTOC claims.
 *
 * The resulting map is stored as coalesced ranges, not one entry per LBA: a
 * flat table would be ~3.8M entries, while real mappings are overwhelmingly
 * contiguous runs.
 *
 * ---------------------------------------------------------------------
 * STATUS
 *
 * This is a READ-ONLY implementation, deliberately.
 *
 * The read path is what the Linux work proved out; writes are not attempted
 * at all. A bad write to the Whimory maps destroys the user data area with no
 * recovery short of a full restore, and there is no reason to risk that
 * before reads are trusted on this port.
 * ---------------------------------------------------------------------
 */

#define SB_COUNT            NAND_BLOCKS_PER_CAU
#define BANKS               (NAND_MAX_CE * NAND_MAX_CAU)

/* Data pages in a superblock: everything but the BTOC page. */
#define DATA_PAGES_PER_SB   (NAND_PAGES_PER_BLOCK - 1)

/*
 * Range table.
 *
 * 8192 was sized for the brute-force rebuild. The CXT snapshot seeds far more:
 * the Linux driver measures 239525 extents from it on this unit, which coalesce
 * down to a few thousand ranges -- but "a few thousand" is measured on one
 * volume, and the failure when it does not fit is the nastiest kind. The map
 * fills to exactly the ceiling and the rest of the device is simply absent:
 * reads past it return unmapped, FAT cannot fetch directory blocks, and the
 * mount comes up looking like a corrupt disk rather than a driver that stopped
 * writing down where things are. Linux hit precisely that at 200000 nodes and
 * lost 39512 extents to it.
 *
 * A range is 24 bytes, so this ceiling is about 780 KB of a 64 MB DRAM, and it
 * is a ceiling rather than an allocation -- entries exist only for extents that
 * exist. Headroom is close to free; silent truncation is not.
 */
#define MAX_RANGES          32768

/* LBA values that mean "nothing here" rather than a real mapping. */
#define LBA_BLANK           0xffffffffu
#define LBA_HOLE            0xfffffffeu
#define LBA_DELETED         0xfffffffdu
#define LBA_LIST            0xfffffffcu

static struct ftl_range ranges[MAX_RANGES];
static unsigned range_count;
static bool ranges_overflowed;

static bool ftl_is_ready;
static uint32_t fat_base_lba;
static uint32_t disk_sector_count;

static unsigned stat_sbs_closed;
static unsigned stat_sbs_open;
static unsigned stat_mapped;

static const char *prog_phase = "idle";
static unsigned prog_cur, prog_total;
static unsigned prog_found, prog_want;

/*
 * Open superblocks found during pass one.
 *
 * Pass one already visits every superblock and already decides which ones are
 * open -- that is the entire purpose of the sweep. It then threw the
 * coordinates away, and pass two re-read all 8352 BTOC pages to work out the
 * same answer a second time. Two identical full sweeps of the same NAND to
 * learn a fact the first one had in hand.
 *
 * Writing down four bytes per hit removes the second sweep completely. Open
 * superblocks are the ones currently being written, so there are normally a
 * handful; 512 is far above any plausible count and costs 2 KB.
 *
 * If it ever does overflow, pass two falls back to the old full sweep rather
 * than silently rebuilding an incomplete map. Slow and correct beats fast and
 * missing half the volume.
 */
#define OPEN_SB_MAX     512

static struct open_sb_ref {
    uint16_t block;
    uint8_t  ce;
    uint8_t  cau;
} open_sbs[OPEN_SB_MAX];

static unsigned open_sb_count;
static bool open_sb_overflow;

/* One page's worth of scratch, reused throughout. */
static struct nand_cs_page scratch;

static bool lba_is_special(uint32_t lba)
{
    return lba >= LBA_LIST;
}

/* ------------------------------------------------------------- ranges --- */

/*
 * Ranges are kept sorted by start LBA so lookup can binary search. Insertion
 * is O(n) on a memmove, which is fine: this runs once at mount and the table
 * is small enough that the copy is cheaper than maintaining a tree.
 */
static int range_find_index(uint32_t lba)
{
    int lo = 0, hi = (int)range_count - 1;

    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        const struct ftl_range *r = &ranges[mid];

        if (lba < r->start)
            hi = mid - 1;
        else if (lba >= r->start + r->len)
            lo = mid + 1;
        else
            return mid;
    }

    return -1;
}

/* Where a new range beginning at `start` belongs. */
static unsigned range_insert_pos(uint32_t start)
{
    unsigned lo = 0, hi = range_count;

    while (lo < hi) {
        unsigned mid = (lo + hi) / 2;

        if (ranges[mid].start < start)
            lo = mid + 1;
        else
            hi = mid;
    }

    return lo;
}

/*
 * Add one LBA -> VBA mapping.
 *
 * Newest weave wins. An older weave for an LBA already mapped is not an
 * error -- it is the stale copy that garbage collection has not reclaimed --
 * so it is dropped silently rather than treated as corruption.
 */
static void map_add(uint32_t lba, uint32_t vba, uint64_t weave)
{
    int idx;
    unsigned pos;

    if (lba_is_special(lba))
        return;

    idx = range_find_index(lba);
    if (idx >= 0) {
        struct ftl_range *r = &ranges[idx];

        if (weave <= r->weave)
            return;     /* stale write, correctly ignored */

        /*
         * A newer write to an LBA inside an existing range splits it. Rather
         * than doing that surgery, shrink the existing range to the part
         * before this LBA and insert the rest fresh -- simpler, and the
         * coalescing pass below stitches contiguous survivors back together.
         */
        if (lba == r->start) {
            r->start++;
            r->vba++;
            r->len--;
            if (!r->len) {
                memmove(r, r + 1,
                        (range_count - idx - 1) * sizeof(*r));
                range_count--;
            }
        } else if (lba == r->start + r->len - 1) {
            r->len--;
        } else {
            /* Interior hit: truncate here and let the tail be re-added. */
            r->len = lba - r->start;
        }
    }

    if (range_count >= MAX_RANGES) {
        ranges_overflowed = true;
        return;
    }

    /* Extend the preceding range when this continues it exactly. */
    pos = range_insert_pos(lba);
    if (pos > 0) {
        struct ftl_range *prev = &ranges[pos - 1];

        if (prev->start + prev->len == lba &&
            prev->vba + prev->len == vba &&
            prev->weave == weave) {
            prev->len++;
            stat_mapped++;
            return;
        }
    }

    memmove(&ranges[pos + 1], &ranges[pos],
            (range_count - pos) * sizeof(ranges[0]));

    ranges[pos].start = lba;
    ranges[pos].len = 1;
    ranges[pos].vba = vba;
    ranges[pos].weave = weave;
    range_count++;
    stat_mapped++;
}

/* Merge adjacent ranges that turned out to be contiguous in both spaces. */
static void ranges_coalesce(void)
{
    unsigned i, out = 0;

    if (!range_count)
        return;

    for (i = 1; i < range_count; i++) {
        struct ftl_range *a = &ranges[out];
        struct ftl_range *b = &ranges[i];

        if (a->start + a->len == b->start &&
            a->vba + a->len == b->vba) {
            a->len += b->len;
            if (b->weave > a->weave)
                a->weave = b->weave;
        } else {
            ranges[++out] = *b;
        }
    }

    range_count = out + 1;
}

/* -------------------------------------------------------- physical map -- */

/*
 * A VBA is a flat ordinal over every data slot on the device, so one number
 * carries bank, block, page and slot. Packing it this way keeps the range
 * table small -- ranges stay contiguous across page boundaries, which they
 * would not if each field were stored separately.
 */
static uint32_t phys_to_vba(unsigned ce, unsigned cau, unsigned block,
                            unsigned page, unsigned slot)
{
    uint32_t bank = ce * NAND_MAX_CAU + cau;
    uint32_t page_i = block * NAND_PAGES_PER_BLOCK + page;

    return ((bank * NAND_BLOCKS_PER_CAU * NAND_PAGES_PER_BLOCK + page_i)
            * NAND_SLOTS_PER_PAGE) + (slot & 3);
}

static void vba_to_phys(uint32_t vba, uint8_t *ce, uint8_t *cau,
                        uint16_t *block, uint8_t *page, uint8_t *slot)
{
    uint32_t slot_i = vba % NAND_SLOTS_PER_PAGE;
    uint32_t page_i = vba / NAND_SLOTS_PER_PAGE;
    uint32_t pages_per_bank = NAND_BLOCKS_PER_CAU * NAND_PAGES_PER_BLOCK;
    uint32_t bank = page_i / pages_per_bank;
    uint32_t rem = page_i % pages_per_bank;

    *slot = (uint8_t)slot_i;
    *page = (uint8_t)(rem % NAND_PAGES_PER_BLOCK);
    *block = (uint16_t)(rem / NAND_PAGES_PER_BLOCK);
    *cau = (uint8_t)(bank % NAND_MAX_CAU);
    *ce = (uint8_t)(bank / NAND_MAX_CAU);
}

/* ---------------------------------------------------------------- BTOC -- */

static uint32_t get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
         | ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

/*
 * A BTOC is an array of big-endian logical page numbers indexed by position
 * within the superblock. Two granularities occur: one entry per slot, or one
 * entry per page covering four slots. Which one is in use is inferred from
 * how many valid entries there are before the 0xFFFFFFFF terminator, exactly
 * as the Linux driver does -- a count that fits in the page count means page
 * granularity.
 */
static unsigned btoc_ingest(const uint8_t *btoc, unsigned len,
                            unsigned ce, unsigned cau, unsigned block,
                            uint64_t weave)
{
    unsigned i, n, valid = 0, added = 0;
    bool page_gran;

    n = len / 4;
    if (n > DATA_PAGES_PER_SB * NAND_SLOTS_PER_PAGE)
        n = DATA_PAGES_PER_SB * NAND_SLOTS_PER_PAGE;

    for (i = 0; i < n; i++) {
        if (get_be32(btoc + i * 4) == LBA_BLANK)
            break;
        valid++;
    }

    if (!valid)
        return 0;

    page_gran = valid <= DATA_PAGES_PER_SB;
    if (page_gran)
        n = valid;

    for (i = 0; i < n; i++) {
        uint32_t lpn = get_be32(btoc + i * 4);
        unsigned pg, slot;

        if (lpn == LBA_BLANK || lba_is_special(lpn))
            continue;

        if (page_gran) {
            /*
             * One entry per page: the entry names the first of four
             * consecutive logical pages, one per slot.
             */
            unsigned k;

            pg = i;
            for (k = 0; k < NAND_SLOTS_PER_PAGE; k++) {
                map_add(lpn * NAND_SLOTS_PER_PAGE + k,
                        phys_to_vba(ce, cau, block, pg, k), weave);
                added++;
            }
        } else {
            pg = i / NAND_SLOTS_PER_PAGE;
            slot = i % NAND_SLOTS_PER_PAGE;
            map_add(lpn, phys_to_vba(ce, cau, block, pg, slot), weave);
            added++;
        }
    }

    return added;
}

/* ------------------------------------------------------------- scanning -- */

static bool page_blank(const struct nand_cs_page *p)
{
    unsigned s;

    for (s = 0; s < NAND_SLOTS_PER_PAGE; s++) {
        if (!p->meta[s].blank)
            return false;
    }
    return true;
}

/*
 * Ingest one physical page using its own metadata as the authority.
 *
 * This is the path that does not depend on BTOC format at all, which is why
 * it is used both for open superblocks and to validate closed ones: type,
 * weave and lba in the 16-byte record are the best-established part of the
 * whole format.
 */
static unsigned ingest_page_meta(const struct nand_cs_page *p,
                                 unsigned ce, unsigned cau,
                                 unsigned block, unsigned page)
{
    unsigned s, added = 0;

    for (s = 0; s < NAND_SLOTS_PER_PAGE; s++) {
        const struct nand_meta *m = &p->meta[s];

        if (!nand_meta_is_data(m))
            continue;
        if (lba_is_special(m->lba))
            continue;

        map_add(m->lba, phys_to_vba(ce, cau, block, page, s), m->weave);
        added++;
    }

    return added;
}

/* Walk an open superblock until the first blank page. */
static void rebuild_open_sb(unsigned ce, unsigned cau, unsigned block)
{
    unsigned pg;

    for (pg = 0; pg < DATA_PAGES_PER_SB; pg++) {
        if (nand_cs_phys_read(ce, cau, block, pg, &scratch))
            break;

        /*
         * Sequential writing means the first blank page ends the live part
         * of the block -- there is nothing useful past it.
         */
        if (page_blank(&scratch))
            break;

        ingest_page_meta(&scratch, ce, cau, block, pg);
    }
}

/* ------------------------------------------------------------- recover -- */

/*
 * Find the FAT32 BPB among the mapped LBAs; its fmss_lba is disk_lba 0.
 *
 * The exported volume does not start at fmss_lba 0 -- the firmware area sits
 * in front of it -- and the offset is not a constant that can be hard-coded,
 * so it is discovered by looking for a real boot sector.
 */
static bool bpb_looks_valid(const uint8_t *sec)
{
    unsigned bytes_per_sec;

    /* Boot signature at the end of the first 512-byte sector. */
    if (sec[510] != 0x55 || sec[511] != 0xaa)
        return false;

    bytes_per_sec = sec[11] | (sec[12] << 8);
    if (bytes_per_sec != 512 && bytes_per_sec != 1024 &&
        bytes_per_sec != 2048 && bytes_per_sec != 4096)
        return false;

    /* Sectors per cluster must be a power of two, 1..128. */
    if (!sec[13] || (sec[13] & (sec[13] - 1)))
        return false;

    /* Reserved sector count is never zero on FAT. */
    if (!(sec[14] | (sec[15] << 8)))
        return false;

    return true;
}

static bool find_fat_base(void)
{
    static uint8_t sec[NAND_SLOT_DATA];
    unsigned i;

    /*
     * The BPB is at the start of the volume, so it is the first mapped LBA
     * whose contents look like one. Walking ranges rather than raw LBAs keeps
     * this to a handful of reads.
     */
    for (i = 0; i < range_count; i++) {
        uint32_t lba = ranges[i].start;

        if (ftl_read_fmss_lba(lba, sec))
            continue;

        if (bpb_looks_valid(sec)) {
            fat_base_lba = lba;

            /*
             * Everything mapped from here on is the volume. This is an
             * upper bound rather than the FAT's own total-sector field,
             * which is the honest thing to report when the map may be
             * incomplete.
             */
            {
                const struct ftl_range *last = &ranges[range_count - 1];

                disk_sector_count = (last->start + last->len) - fat_base_lba;
            }
            return true;
        }
    }

    return false;
}

/*
 * Mount progress, on screen.
 *
 * The scan is 8,352 superblock reads in pass one alone (2088 blocks x 4
 * banks), and a read that gets no answer costs CS_POLL_US -- 200 ms -- before
 * it gives up. If the NAND is not responding, that is 28 minutes per pass and
 * two passes to go, with the Rockbox logo sitting motionless the whole time.
 * It is not hung, but nothing on the device can tell you that, and "wait an
 * hour to find out" is not a debugging step.
 *
 * So the count goes on the glass. A moving number distinguishes slow from
 * stopped, which is the single most useful fact during a mount and the one
 * thing the beacons could never express -- they can say WHERE the boot is,
 * never whether it is still moving.
 *
 * Only the top band is repainted, and only every PROG_EVERY units. Full frames
 * are 103,680 PIO pixel writes on this target; repainting one per block would
 * make the display, not the NAND, the reason the mount was slow.
 */
#define PROG_EVERY      64
#define PROG_BAND_H     40

static void ftl_progress_paint(void)
{
    lcd_clear_display();
    lcd_putsf(0, 0, "FTL %s", prog_phase);
    lcd_putsf(0, 1, "%u / %u", prog_cur, prog_total);
    if (prog_want)
        lcd_putsf(0, 2, "open %u / %u", prog_found, prog_want);
    lcd_update_rect(0, 0, LCD_WIDTH, PROG_BAND_H);
}

/* ------------------------------------------------------------ SFTL CXT -- */

/*
 * The SFTL context is the FTL's own snapshot of the map, and reading it is
 * enormously cheaper than reconstructing what it already recorded.
 *
 * Layout, from the Linux driver:
 *
 *   A CXT superblock is identified by the meta of its FIRST slot (page 0,
 *   slot 0): type 0x1f, tag 1. Its weave orders it against the others; the
 *   newest base is the one to trust.
 *
 *   Inside, each slot is one record, tagged in meta byte 1. Tag 4 carries L2V
 *   extents, tag 255 ends the context. Every other tag is skipped.
 *
 *   An L2V record is a 4096-byte page of 8-byte pairs. The FIRST pair is a
 *   header: LBA to resume at, and a span that must equal 0xfffffff0 -- a
 *   marker, not a length. The rest are (vba, span) and the LBA advances by
 *   each span, so the records describe one continuously increasing run of the
 *   logical space. That is why records must be consecutive and why a gap is
 *   treated as corruption rather than skipped.
 *
 * Two things this must NOT do, both learned from the Linux driver:
 *
 *   VBAs that live inside a CXT superblock hold context records, not user
 *   data. They must never enter the map.
 *
 *   The snapshot is not the present. Superblocks written after it carry a
 *   higher weave and have to be replayed on top, or the mount comes up
 *   correct-but-stale and the newest files are missing. That replay is the
 *   whole reason the weave is tracked here.
 */
#define CXT_TAG_BASE        1
#define CXT_TAG_L2V         4
#define CXT_TAG_END         255
#define CXT_CONTIG_SPAN     0xfffffff0u

#define CXT_MAX_SB          32

/*
 * How far down from the top of the device to look for a context.
 *
 * The CXT lives in the high blocks -- around 1960 of 2088 on N31 -- so a
 * targeted hunt finds it in a few hundred reads instead of sweeping the whole
 * device to stumble across it. 256 blocks is roughly double the observed
 * offset, which is margin without being a second full scan.
 */
#define CXT_HUNT_BLOCKS     256

/* Every VBA on the device; anything at or above this is not a real address. */
#define VBA_LIMIT           ((uint32_t)(NAND_MAX_CE * NAND_MAX_CAU) * \
                             NAND_BLOCKS_PER_CAU * NAND_PAGES_PER_BLOCK * \
                             NAND_SLOTS_PER_PAGE)

struct cxt_sb_ref {
    uint16_t block;
    uint8_t  ce;
    uint8_t  cau;
};

static struct cxt_sb_ref cxt_sbs[CXT_MAX_SB];   /* all CXT superblocks */
static unsigned cxt_sb_count;

static struct cxt_sb_ref cxt_bases[CXT_MAX_SB]; /* those tagged BASE */
static uint64_t cxt_base_weaves[CXT_MAX_SB];
static unsigned cxt_base_count;

static bool cxt_loaded;
static uint64_t cxt_weave;          /* weave of the loaded snapshot */
static uint32_t cxt_next_lba;
static bool cxt_lba_valid;

static uint32_t rd32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/*
 * VBAs inside a CXT superblock are context records, not user data.
 * Linear scan: cxt_sb_count is a handful, and this runs once per extent.
 */
static bool vba_is_cxt(uint32_t vba)
{
    uint8_t ce, cau, page, slot;
    uint16_t block;
    unsigned i;

    vba_to_phys(vba, &ce, &cau, &block, &page, &slot);

    for (i = 0; i < cxt_sb_count; i++) {
        if (cxt_sbs[i].ce == ce && cxt_sbs[i].cau == cau &&
            cxt_sbs[i].block == block)
            return true;
    }
    return false;
}

/*
 * Append one CXT extent.
 *
 * Deliberately NOT map_add(). CXT records arrive in strictly increasing LBA
 * order -- the header's continuity check is what guarantees it -- and they are
 * loaded into an empty table before anything else. So each extent either
 * extends the last range or becomes the next one: O(1), no search, no memmove.
 *
 * map_add() does a sorted insert with a memmove per entry, which is right for
 * the scattered writes the BTOC replay produces and quadratic for a quarter of
 * a million ordered ones. Using it here would make loading the fast path
 * slower than the scan it replaces.
 */
static void cxt_append(uint32_t lba, uint32_t span, uint32_t vba, uint64_t weave)
{
    if (!span || lba_is_special(lba))
        return;

    if (range_count) {
        struct ftl_range *prev = &ranges[range_count - 1];

        if (prev->start + prev->len == lba &&
            prev->vba + prev->len == vba &&
            prev->weave == weave) {
            prev->len += span;
            stat_mapped += span;
            return;
        }
    }

    if (range_count >= MAX_RANGES) {
        ranges_overflowed = true;
        return;
    }

    ranges[range_count].start = lba;
    ranges[range_count].len = span;
    ranges[range_count].vba = vba;
    ranges[range_count].weave = weave;
    range_count++;
    stat_mapped += span;
}

/* One tag-4 record: a header pair followed by (vba, span) pairs. */
static int cxt_load_l2v(const uint8_t *d, uint64_t weave)
{
    uint32_t lba, span, vba;
    unsigned i, n = NAND_SLOT_DATA / 8;

    lba = rd32le(d);
    span = rd32le(d + 4);

    if (span == 0xffffffffu)
        return 0;               /* unwritten record, not an error */
    if (span != CXT_CONTIG_SPAN)
        return -1;              /* not the layout we understand */

    /*
     * The records describe one continuous run of the logical space. A break
     * means we have misparsed or the context is damaged, and continuing would
     * silently attach real VBAs to the wrong LBAs -- far worse than falling
     * back to the scan.
     */
    if (cxt_lba_valid && lba != cxt_next_lba)
        return -1;
    cxt_lba_valid = true;

    for (i = 1; i < n; i++) {
        vba  = rd32le(d + 8 * i);
        span = rd32le(d + 8 * i + 4);

        if (vba == 0xffffffffu || !span)
            break;              /* end of this record */

        if (vba < VBA_LIMIT && !vba_is_cxt(vba))
            cxt_append(lba, span, vba, weave);

        lba += span;
    }

    cxt_next_lba = lba;
    return 0;
}

/* Walk one CXT superblock in VBA order until the END tag. */
static int cxt_load_sb(unsigned ce, unsigned cau, unsigned block,
                       uint64_t weave)
{
    unsigned pg, s;

    cxt_lba_valid = false;
    cxt_next_lba = 0;

    for (pg = 0; pg < NAND_PAGES_PER_BLOCK; pg++) {
        if (nand_cs_phys_read(ce, cau, block, pg, &scratch))
            return -1;

        if ((pg % 8) == 0) {
            prog_cur = pg;
            ftl_progress_paint();
        }

        for (s = 0; s < NAND_SLOTS_PER_PAGE; s++) {
            const uint8_t *m = scratch.meta_raw[s];

            if (m[0] != NAND_META_TYPE_SFTL_CXT)
                continue;
            if (m[1] == CXT_TAG_END)
                return 0;
            if (m[1] != CXT_TAG_L2V)
                continue;

            if (cxt_load_l2v(scratch.data[s], weave))
                return -1;
        }
    }

    return 0;
}

/*
 * Find the context, newest first, and load it.
 *
 * Reads page 0 of each superblock in the top CXT_HUNT_BLOCKS blocks. That is
 * about a thousand reads against the 8352 a full sweep costs, and it is the
 * only place the context can be.
 */
static bool cxt_hunt_and_load(void)
{
    unsigned block, ce, cau, i, best;
    unsigned lo = (SB_COUNT > CXT_HUNT_BLOCKS) ? SB_COUNT - CXT_HUNT_BLOCKS : 0;

    cxt_sb_count = cxt_base_count = 0;
    cxt_loaded = false;
    cxt_weave = 0;

    prog_phase = "cxt find";
    prog_total = (SB_COUNT - lo) * BANKS;
    prog_cur = 0;
    prog_want = 0;

    for (block = SB_COUNT; block-- > lo; ) {
        for (ce = 0; ce < NAND_MAX_CE; ce++) {
            for (cau = 0; cau < NAND_MAX_CAU; cau++) {
                const uint8_t *m;

                prog_cur++;
                if ((prog_cur % PROG_EVERY) == 0)
                    ftl_progress_paint();

                if (nand_cs_phys_read(ce, cau, block, 0, &scratch))
                    continue;

                m = scratch.meta_raw[0];
                if (m[0] != NAND_META_TYPE_SFTL_CXT)
                    continue;

                if (cxt_sb_count < CXT_MAX_SB) {
                    cxt_sbs[cxt_sb_count].block = (uint16_t)block;
                    cxt_sbs[cxt_sb_count].ce    = (uint8_t)ce;
                    cxt_sbs[cxt_sb_count].cau   = (uint8_t)cau;
                    cxt_sb_count++;
                }

                if (m[1] == CXT_TAG_BASE && cxt_base_count < CXT_MAX_SB) {
                    cxt_bases[cxt_base_count].block = (uint16_t)block;
                    cxt_bases[cxt_base_count].ce    = (uint8_t)ce;
                    cxt_bases[cxt_base_count].cau   = (uint8_t)cau;
                    cxt_base_weaves[cxt_base_count] = scratch.meta[0].weave;
                    cxt_base_count++;
                }
            }
        }
    }

    /* Newest base wins; older ones are previous snapshots. */
    while (cxt_base_count) {
        best = 0;
        for (i = 1; i < cxt_base_count; i++) {
            if (cxt_base_weaves[i] > cxt_base_weaves[best])
                best = i;
        }

        prog_phase = "cxt load";
        prog_total = NAND_PAGES_PER_BLOCK;
        prog_cur = 0;
        ftl_progress_paint();

        range_count = 0;
        stat_mapped = 0;
        ranges_overflowed = false;

        if (cxt_load_sb(cxt_bases[best].ce, cxt_bases[best].cau,
                        cxt_bases[best].block, cxt_base_weaves[best]) == 0 &&
            range_count) {
            cxt_weave = cxt_base_weaves[best];
            cxt_loaded = true;
            return true;
        }

        /*
         * That base did not parse. Discard whatever it produced -- a partial
         * context is not a map -- and try the next newest.
         */
        range_count = 0;
        stat_mapped = 0;
        ranges_overflowed = false;

        cxt_bases[best] = cxt_bases[cxt_base_count - 1];
        cxt_base_weaves[best] = cxt_base_weaves[cxt_base_count - 1];
        cxt_base_count--;
    }

    return false;
}

/*
 * How many reads may fail back-to-back, from a cold start, before we accept
 * that the NAND is not talking to us.
 *
 * A device with data on it answers its very first BTOC read. Blank
 * superblocks return a page and are classified as free -- they do not fail.
 * So a long unbroken run of hard failures before ANY success does not mean
 * "an empty device", it means the controller is not responding, and every
 * further attempt buys another 200 ms of the same answer.
 *
 * 64 is generous enough to ride out a bad region at the start of the scan and
 * cheap enough to bail in about thirteen seconds instead of half an hour.
 * Once a single read has succeeded the budget is retired entirely: from that
 * point failures are ordinary bad blocks and must not abort a real mount.
 */
#define COLD_FAIL_LIMIT 64

int ftl_recover(void)
{
    unsigned ce, cau, block;
    unsigned scanned = 0;
    unsigned cold_fails = 0;
    bool any_read_ok = false;

    if (!nand_hw_present())
        return -1;

    range_count = 0;
    ranges_overflowed = false;
    stat_sbs_closed = stat_sbs_open = stat_mapped = 0;
    open_sb_count = 0;
    open_sb_overflow = false;
    ftl_is_ready = false;

    /*
     * Try the context first. When it loads, the map is already built and the
     * sweep below only has to find what was written AFTER the snapshot.
     */
    cxt_hunt_and_load();

    prog_phase = "scan";
    prog_total = SB_COUNT * BANKS;
    prog_cur = 0;

    /*
     * Pass one: classify every superblock from its last page.
     *
     * Reading only the BTOC page rather than every page is what makes mount
     * take seconds instead of quarter of an hour -- a full walk of this
     * device is over a million page reads.
     */
    for (block = 0; block < SB_COUNT; block++) {
        for (ce = 0; ce < NAND_MAX_CE; ce++) {
            for (cau = 0; cau < NAND_MAX_CAU; cau++) {
                unsigned s;
                bool is_btoc = false;
                uint64_t weave = 0;

                prog_cur = ++scanned;
                if ((scanned % PROG_EVERY) == 0)
                    ftl_progress_paint();

                if (nand_cs_phys_read(ce, cau, block, NAND_BTOC_PAGE,
                                      &scratch)) {
                    if (!any_read_ok && ++cold_fails >= COLD_FAIL_LIMIT) {
                        prog_phase = "no answer";
                        ftl_progress_paint();
                        return -1;
                    }
                    continue;
                }

                any_read_ok = true;

                if (page_blank(&scratch))
                    continue;   /* free superblock */

                for (s = 0; s < NAND_SLOTS_PER_PAGE; s++) {
                    if (scratch.meta[s].valid &&
                        scratch.meta[s].type == NAND_META_TYPE_BTOC) {
                        is_btoc = true;
                        weave = scratch.meta[s].weave;
                        break;
                    }
                }

                if (is_btoc) {
                    stat_sbs_closed++;

                    /*
                     * With a context loaded, superblocks older than the
                     * snapshot are already in the map -- replaying them costs
                     * a full BTOC parse each to produce entries map_add()
                     * would discard as stale anyway.
                     *
                     * Newer ones are exactly what the context cannot know
                     * about, so they still get replayed. That is what keeps
                     * the most recent writes present rather than mounting a
                     * correct-but-stale snapshot.
                     */
                    if (cxt_loaded && weave <= cxt_weave)
                        continue;

                    btoc_ingest(scratch.data[0], NAND_SLOT_DATA,
                                ce, cau, block, weave);
                } else {
                    /*
                     * Written but no BTOC: still open. The page-by-page
                     * rebuild is deferred to pass two so the cheap
                     * classification finishes first, but the COORDINATES are
                     * recorded here -- they are already in hand, and pass two
                     * used to re-read the whole device to recover them.
                     */
                    stat_sbs_open++;

                    if (open_sb_count < OPEN_SB_MAX) {
                        open_sbs[open_sb_count].block = (uint16_t)block;
                        open_sbs[open_sb_count].ce    = (uint8_t)ce;
                        open_sbs[open_sb_count].cau   = (uint8_t)cau;
                        open_sb_count++;
                    } else {
                        open_sb_overflow = true;
                    }
                }
            }
        }
    }

    /* Pass two: walk the open superblocks page by page. */
    prog_phase = "open";
    /*
     * Pass two counts the SWEEP, not the findings.
     *
     * This used to set prog_total = stat_sbs_open and advance prog_cur only
     * when an open superblock was found. That measures the wrong thing: the
     * work here is a second full 8352-block re-read of every BTOC page, and
     * open superblocks are rare, so the number sat still for minutes at a
     * time while the device was busy. On screen that is indistinguishable
     * from a hang -- which is exactly how it was reported, right after pass
     * one had finished cleanly at 8320/8352.
     *
     * A progress counter has one job: move when work is happening. The
     * open-superblock tally is still worth seeing, so it gets its own line
     * rather than being smuggled into the main one.
     */
    prog_cur = 0;
    prog_total = SB_COUNT * BANKS;
    prog_found = 0;
    prog_want = stat_sbs_open;

    prog_phase = "btoc";

    /*
     * Fast path: rebuild straight from the list pass one wrote down.
     *
     * This is the difference between one sweep of the NAND and two. The old
     * path re-read every BTOC page on the device to re-find superblocks that
     * had already been identified minutes earlier, which roughly doubled
     * mount time for no new information.
     *
     * The full sweep survives below purely for the overflow case.
     */
    if (stat_sbs_open && !open_sb_overflow) {
        unsigned i;

        prog_total = open_sb_count;
        prog_want = 0;

        for (i = 0; i < open_sb_count; i++) {
            prog_cur = i + 1;
            ftl_progress_paint();
            rebuild_open_sb(open_sbs[i].ce, open_sbs[i].cau,
                            open_sbs[i].block);
        }
    }
    else if (stat_sbs_open) {
        unsigned done = 0;
        unsigned swept = 0;

        for (block = 0; block < SB_COUNT && done < stat_sbs_open; block++) {
            for (ce = 0; ce < NAND_MAX_CE; ce++) {
                for (cau = 0; cau < NAND_MAX_CAU; cau++) {
                    unsigned s;
                    bool is_btoc = false;

                    prog_cur = ++swept;
                    if ((swept % PROG_EVERY) == 0)
                        ftl_progress_paint();

                    if (nand_cs_phys_read(ce, cau, block, NAND_BTOC_PAGE,
                                          &scratch))
                        continue;
                    if (page_blank(&scratch))
                        continue;

                    for (s = 0; s < NAND_SLOTS_PER_PAGE; s++) {
                        if (scratch.meta[s].valid &&
                            scratch.meta[s].type == NAND_META_TYPE_BTOC) {
                            is_btoc = true;
                            break;
                        }
                    }
                    if (is_btoc)
                        continue;

                    rebuild_open_sb(ce, cau, block);
                    prog_found = ++done;
                    ftl_progress_paint();
                }
            }
        }
    }

    prog_want = 0;
    prog_phase = "pack";
    ftl_progress_paint();
    ranges_coalesce();

    if (!range_count)
        return -1;

    ftl_is_ready = true;

    prog_phase = "bpb";
    ftl_progress_paint();
    if (!find_fat_base()) {
        /*
         * A map with no recognisable boot sector is not a mountable volume.
         * Report that rather than exporting an offset of zero and letting
         * the FAT layer read gibberish.
         */
        ftl_is_ready = false;
        prog_phase = "no-bpb";
        ftl_progress_paint();
        return -1;
    }

    prog_phase = "ready";
    ftl_progress_paint();
    return 0;
}

/* -------------------------------------------------------------- lookup -- */

int ftl_lookup(uint32_t fmss_lba, uint8_t *ce, uint8_t *cau, uint16_t *block,
               uint8_t *page, uint8_t *slot, uint64_t *weave)
{
    int idx = range_find_index(fmss_lba);
    const struct ftl_range *r;
    uint32_t vba;

    if (idx < 0)
        return -1;

    r = &ranges[idx];
    vba = r->vba + (fmss_lba - r->start);

    vba_to_phys(vba, ce, cau, block, page, slot);
    if (weave)
        *weave = r->weave;

    return 0;
}

int ftl_read_fmss_lba(uint32_t fmss_lba, void *buf)
{
    uint8_t ce, cau, page, slot;
    uint16_t block;
    int pick;

    if (!buf)
        return -1;
    if (ftl_lookup(fmss_lba, &ce, &cau, &block, &page, &slot, NULL))
        return -1;

    if (nand_cs_phys_read(ce, cau, block, page, &scratch))
        return -1;

    /*
     * Trust the metadata over the map. The mapped slot is where the LBA
     * should be, but re-checking which slot actually claims it -- and taking
     * the newest weave if more than one does -- is what stops a stale map
     * from silently returning the wrong sector's contents.
     */
    pick = nand_meta_pick_lba(&scratch, fmss_lba);
    if (pick < 0) {
        /* The map said this page; the page disagrees. Refuse rather than
         * hand back whatever happened to be in the expected slot. */
        return -1;
    }

    memcpy(buf, scratch.data[pick], NAND_SLOT_DATA);
    return 0;
}

int ftl_read_disk_lba(uint32_t disk_lba, void *buf)
{
    if (!ftl_is_ready)
        return -1;

    return ftl_read_fmss_lba(fat_base_lba + disk_lba, buf);
}

/* -------------------------------------------------------------- public -- */

bool ftl_init(void)
{
    if (!nand_hw_init())
        return false;

    range_count = 0;
    ftl_is_ready = false;
    return true;
}

bool ftl_ready(void)
{
    return ftl_is_ready;
}

uint32_t ftl_fat_base_lba(void)
{
    return fat_base_lba;
}

uint32_t ftl_disk_sectors(void)
{
    return disk_sector_count;
}

void ftl_get_stats(unsigned *rangesp, unsigned *mapped, unsigned *sbs_closed,
                   unsigned *sbs_open)
{
    if (rangesp)
        *rangesp = range_count;
    if (mapped)
        *mapped = stat_mapped;
    if (sbs_closed)
        *sbs_closed = stat_sbs_closed;
    if (sbs_open)
        *sbs_open = stat_sbs_open;
}

const char *ftl_progress_phase(unsigned *cur, unsigned *total)
{
    if (cur)
        *cur = prog_cur;
    if (total)
        *total = prog_total;

    /* Surface a dropped tail rather than reporting a clean map. */
    if (ranges_overflowed && !strcmp(prog_phase, "ready"))
        return "ready (range table full)";

    return prog_phase;
}
