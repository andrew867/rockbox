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
#include <stdio.h>
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
 * Raised again once the weave filter was fixed. That fix turns "replayed 2 of
 * 2308" into a real replay of every superblock the checkpoint does not cover,
 * and all of that has to land somewhere -- the previous ceiling was sized for
 * a run that was silently discarding the work.
 *
 * A range is 24 bytes, so this ceiling is about 3 MB of a 64 MB DRAM, and it
 * is a ceiling rather than an allocation -- entries exist only for extents
 * that exist. Deliberately far above any measured figure: the Linux driver
 * runs with a 500000-node ceiling on the same volume, and the point is to find
 * out whether we overflow rather than to quietly stop at a tidy number.
 * Headroom is close to free; silent truncation is not.
 */
#define MAX_RANGES          131072

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
static unsigned stat_probe_empty;
static unsigned stat_cxt_oob;       /* extents pointing outside the device */
static unsigned stat_cxt_self;      /* extents inside a context superblock */
static unsigned stat_cxt_hole;      /* extents the context marks as free space */
static unsigned stat_skipped;       /* closed SBs the checkpoint already covers */
static unsigned stat_btoc_fallback; /* closed SBs whose BTOC parsed to nothing */
static unsigned stat_btoc_form[2];  /* [0] plain record array, [1] 8-byte header */
static unsigned stat_sweep_readfail; /* page 0 unreadable; replayed anyway */
static unsigned stat_bpb_cands;     /* headers that looked like a BPB */
static bool     stat_bpb_fatsig;    /* the chosen one had a real FAT */

static const char *prog_phase = "idle";
static unsigned prog_cur, prog_total;
static unsigned prog_found, prog_want;

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
         * A newer write to an LBA inside an existing range splits that range
         * in two, and BOTH halves have to survive.
         *
         * This used to truncate at the split point with the comment "let the
         * tail be re-added". Nothing re-adds it. Every LBA after the split was
         * silently dropped from the map, and a dropped LBA is not an error --
         * reads of it simply fail, which surfaces much later as an unreadable
         * file or an invalid cluster chain and looks like disk corruption
         * rather than a range that was thrown away.
         *
         * Splitting properly is a few lines. The head keeps its start and vba;
         * the tail starts one LBA past the hit with its vba advanced to match,
         * and inherits the weave, because it is the same write.
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
            uint32_t head_len  = lba - r->start;
            uint32_t tail_len  = r->len - head_len - 1;
            uint32_t tail_lba  = lba + 1;
            uint32_t tail_vba  = r->vba + head_len + 1;
            uint64_t tail_weav = r->weave;

            r->len = head_len;

            if (tail_len) {
                if (range_count < MAX_RANGES) {
                    memmove(&ranges[idx + 2], &ranges[idx + 1],
                            (range_count - idx - 1) * sizeof(ranges[0]));
                    ranges[idx + 1].start = tail_lba;
                    ranges[idx + 1].len   = tail_len;
                    ranges[idx + 1].vba   = tail_vba;
                    ranges[idx + 1].weave = tail_weav;
                    range_count++;
                } else {
                    /* No room for the tail: say so rather than lose it
                     * quietly, which is the whole point of this fix. */
                    ranges_overflowed = true;
                }
            }
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
 * A VBA is a flat ordinal over every data slot on the device.
 *
 * The ORDER of the fields is not a free choice, and getting it wrong is what
 * produced a perfectly built map that FAT could not read: 579124 LBAs across
 * 1692 ranges, and no boot sector anywhere in it.
 *
 * This used to be bank-major -- every (ce, cau, block) triple got its own
 * superblock index, so the flat ordinal ran block-by-block through all of
 * plane 0, then all of plane 1, and so on. That is a perfectly reasonable
 * layout and it is not the one Apple uses.
 *
 * Apple treats a superblock as the SAME virtual block across every (ce, cau)
 * plane, and puts the plane index between the page and the slot:
 *
 *   vba = vblock * (pages_per_sb * planes * slots)
 *       + page   * (planes * slots)
 *       + plane  * slots
 *       + slot
 *
 * The CXT records its extents in that space. Under the old layout a CXT VBA
 * landed on an unrelated page, so every extent pointed somewhere real but
 * wrong -- which is exactly a full map with no recognisable BPB in it.
 *
 * Matching Apple's layout rather than translating on the way in is deliberate.
 * A translation would be correct but would shatter the map: consecutive CXT
 * VBAs stay contiguous only within one 4-slot group, because the next group
 * belongs to a different plane, so 579124 mapped LBAs would become ~145000
 * ranges instead of 1692. Adopting the layout keeps every run contiguous and
 * costs nothing -- this ordinal is ours to define, and only ever has to agree
 * with vba_to_phys() below.
 */
#define VBA_PER_PAGE    (NAND_MAX_CE * NAND_MAX_CAU * NAND_SLOTS_PER_PAGE)
#define VBA_PER_SB      (NAND_PAGES_PER_BLOCK * VBA_PER_PAGE)

static uint32_t phys_to_vba(unsigned ce, unsigned cau, unsigned block,
                            unsigned page, unsigned slot)
{
    uint32_t plane = ce * NAND_MAX_CAU + cau;

    return block * VBA_PER_SB
         + page  * VBA_PER_PAGE
         + plane * NAND_SLOTS_PER_PAGE
         + (slot & 3);
}

static void vba_to_phys(uint32_t vba, uint8_t *ce, uint8_t *cau,
                        uint16_t *block, uint8_t *page, uint8_t *slot)
{
    uint32_t vblock = vba / VBA_PER_SB;
    uint32_t rem    = vba % VBA_PER_SB;
    uint32_t plane  = (rem % VBA_PER_PAGE) / NAND_SLOTS_PER_PAGE;

    *block = (uint16_t)vblock;
    *page  = (uint8_t)(rem / VBA_PER_PAGE);
    *cau   = (uint8_t)(plane % NAND_MAX_CAU);
    *ce    = (uint8_t)(plane / NAND_MAX_CAU);
    *slot  = (uint8_t)(rem % NAND_SLOTS_PER_PAGE);
}

/* ---------------------------------------------------------------- BTOC -- */

static uint32_t get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
         | ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
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
/*
 * Defined with the CXT loader further down; the open-superblock fast-forward
 * below needs them and sits above it. Tentative definitions, so both refer to
 * the same object.
 */
static bool cxt_loaded;
static uint64_t cxt_weave;


/*
 * Find the first page in this superblock that the checkpoint does NOT cover.
 *
 * Open superblocks cannot be skipped wholesale -- that was the bug that threw
 * away the entire post-checkpoint history -- but that does not mean every page
 * in them has to be re-ingested. A block open across the checkpoint is mostly
 * pages the CXT already recorded; only the tail is new.
 *
 * Whimory appends, so weave increases monotonically down the block. That makes
 * "first page newer than the checkpoint" a boundary a binary search can find
 * in about seven reads instead of walking up to 128. Everything before it is
 * already in the map and re-reading it produces entries map_add() would
 * discard as stale.
 *
 * Blank means past the live end, so the search moves left: there is nothing
 * newer beyond it.
 *
 * This is what "fast-forward into the snapshot's future" actually means -- not
 * skipping blocks, which loses data, but skipping the prefix of each block
 * that the snapshot already accounts for.
 */
static unsigned open_sb_start(unsigned ce, unsigned cau, unsigned block)
{
    unsigned lo = 0, hi = DATA_PAGES_PER_SB;

    if (!cxt_loaded)
        return 0;

    while (lo < hi) {
        unsigned mid = lo + (hi - lo) / 2;
        struct nand_meta m;

        /*
         * Quarter-cost probe: the search needs only blank-ness and a weave,
         * both of which are in slot 0's meta. Reading all four records here
         * moved 16448 bytes to look at sixteen, seven times per superblock.
         */
        if (!nand_cs_probe_meta0(ce, cau, block, mid, &m)) {
            hi = mid;               /* unreadable: assume nothing newer past it */
            continue;
        }

        if (m.blank || m.weave > cxt_weave)
            hi = mid;
        else
            lo = mid + 1;
    }

    return lo;
}

static void rebuild_sb_pages(unsigned ce, unsigned cau, unsigned block,
                             unsigned limit)
{
    unsigned pg;

    if (limit > DATA_PAGES_PER_SB)
        limit = DATA_PAGES_PER_SB;

    for (pg = open_sb_start(ce, cau, block); pg < limit; pg++) {
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

static void rebuild_open_sb(unsigned ce, unsigned cau, unsigned block)
{
    rebuild_sb_pages(ce, cau, block, DATA_PAGES_PER_SB);
}

/* ------------------------------------------------------------------ BTOC */

/*
 * A BTOC page is an array of 16-byte big-endian records, built by s_btoc.c
 * sub_567E3C:
 *
 *   struct { u32 weaveSeqAdd; u32 aux; u32 lba; u32 span; }
 *
 * The decomp is worth reading for what it rules out. It always writes a real
 * span -- the caller asserts "0 != chunk" first, then it does
 * nextVbaOfs += span and asserts the VBA still matches
 * s_g_addr_to_vba(sb, nextVbaOfs). So a zero span was never written as a
 * record, and aux is not a length: at the call site it is
 * *(a1[8] + 12), a tag lifted from a stream descriptor.
 *
 * Records are consecutive in VBA space: record N covers vba_ofs ..
 * vba_ofs+span-1 within this block, with vba_ofs accumulating. That makes the
 * SUM OF SPANS the number of live VBAs in the superblock, which is the only
 * thing we actually want from the page.
 *
 * Two layouts occur. Some pages begin with the record array; 255 of 563 closed
 * superblocks on the Linux side instead carry an eight-byte header first, and
 * every one of those was being dropped -- 127 pages and about 2032 LBAs each,
 * which is what left directories unreadable on a volume whose FAT mounted
 * cleanly. Both offsets are tried.
 *
 * What is deliberately NOT taken from here is the mapping. The BTOC is a
 * physical-slot index; the per-slot meta is the authority, and the Linux
 * driver records why in one line -- "zeros/holes in BTOC were poisoning
 * L2V[0]". So this only decides HOW MANY PAGES TO READ, and every LBA still
 * comes from the meta of the page it was found in. A BTOC we misread costs
 * time, never correctness.
 */
#define BTOC_REC_SIZE           16
#define BTOC_HDR_SIZE           8
#define BTOC_VBAS_PER_SB        (NAND_PAGES_PER_BLOCK * NAND_SLOTS_PER_PAGE)
#define BTOC_DATA_VBAS_PER_SB   (DATA_PAGES_PER_SB * NAND_SLOTS_PER_PAGE)

/* Tokens that advance the VBA cursor without describing user data. */
static bool btoc_lba_is_token(uint32_t lba)
{
    return lba == LBA_BLANK || lba == LBA_HOLE ||
           lba == LBA_DELETED || lba == LBA_LIST;
}

/*
 * Sum the spans of a record array at `off`. Returns the live VBA count, or -1
 * if the bytes there do not read as records.
 */
static int btoc_scan(const uint8_t *p, unsigned len, unsigned off)
{
    unsigned i, vbas = 0, recs = 0;

    if (off + BTOC_REC_SIZE > len)
        return -1;

    /*
     * weaveSeqAdd is a delta from the block's base weave, so the first record
     * is always zero. It is the cheapest way to tell a record array from a
     * header, and it is what distinguishes the two layouts.
     */
    if (get_be32(p + off) != 0)
        return -1;

    for (i = off; i + BTOC_REC_SIZE <= len; i += BTOC_REC_SIZE) {
        uint32_t lba  = get_be32(p + i + 8);
        uint32_t span = get_be32(p + i + 12);

        if (!span || span > BTOC_DATA_VBAS_PER_SB)
            break;                      /* end of array, or not one */
        if (!btoc_lba_is_token(lba) && lba >= 0x01000000u)
            break;
        if (vbas + span > BTOC_VBAS_PER_SB)
            break;

        vbas += span;
        recs++;
    }

    return recs ? (int)vbas : -1;
}

/*
 * How many pages of this superblock hold data.
 *
 * Returns the page count, or -1 when the BTOC does not parse in either
 * layout -- including the shape that is a header followed by zeros where the
 * first record would be. Those cannot be assumed empty: the block may be full
 * and simply have no record array, in which case the only account of its
 * contents is the per-page meta.
 */
static int btoc_live_pages(const uint8_t *p, unsigned len, unsigned *form)
{
    int vbas;

    vbas = btoc_scan(p, len, 0);
    if (vbas >= 0) {
        *form = 0;
    } else {
        vbas = btoc_scan(p, len, BTOC_HDR_SIZE);
        if (vbas < 0)
            return -1;
        *form = 1;
    }

    return (int)(((unsigned)vbas + NAND_SLOTS_PER_PAGE - 1) /
                 NAND_SLOTS_PER_PAGE);
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

/* Defined with the CXT loader below; the BPB check needs it up here. */
static uint32_t rd32le(const uint8_t *p);

/*
 * Does the FAT itself start where this BPB says it does?
 *
 * This is the check that separates two BPBs that both look perfect.
 *
 * A stale copy of the volume header stays a well-formed BPB forever: sector 0
 * is found by content and is static, so it reads back valid no matter how out
 * of date it is. Reading sectors and confirming they read tells you nothing --
 * the Linux port scored two candidates 9/9 on exactly that test, picked the
 * newer one, and had the whole volume six sectors out. Sector 0 and the
 * FSInfo both validated; the FAT belonged to the other one, so vfat walked
 * into the middle of the table.
 *
 * The one thing a stale header cannot fake is the FAT it points at. Every
 * FAT32 opens with entry 0 = the media descriptor in the low byte and the
 * rest ones, and entry 1 = all ones, masked to the 28 bits FAT32 entries
 * actually carry. Point that at the wrong sector and it is not a FAT.
 */
static bool fat_first_sector_ok(uint32_t bpb_lba, const uint8_t *bpb)
{
    static uint8_t fat[NAND_SLOT_DATA];
    unsigned reserved = bpb[14] | (bpb[15] << 8);
    uint8_t media = bpb[21];
    uint32_t e0, e1;

    if (!reserved)
        return false;
    if (ftl_read_fmss_lba(bpb_lba + reserved, fat))
        return false;

    e0 = rd32le(fat) & 0x0fffffffu;
    e1 = rd32le(fat + 4) & 0x0fffffffu;

    return e0 == (0x0fffff00u | media) && e1 == 0x0fffffffu;
}

static bool find_fat_base(void)
{
    static uint8_t sec[NAND_SLOT_DATA];
    uint32_t fallback = 0;
    bool have_fallback = false;
    unsigned i;

    /*
     * Walk the ranges looking for a volume header, and prefer one whose FAT
     * is really there. Walking ranges rather than raw LBAs keeps this to a
     * handful of reads.
     */
    for (i = 0; i < range_count; i++) {
        uint32_t lba = ranges[i].start;

        if (ftl_read_fmss_lba(lba, sec))
            continue;
        if (!bpb_looks_valid(sec))
            continue;

        stat_bpb_cands++;

        if (!fat_first_sector_ok(lba, sec)) {
            /*
             * Looks like a BPB, does not point at a FAT. Almost certainly a
             * stale header. Remember it in case nothing better turns up --
             * mounting on a doubtful candidate is still better than refusing
             * to mount -- but keep looking.
             */
            if (!have_fallback) {
                fallback = lba;
                have_fallback = true;
            }
            continue;
        }

        fat_base_lba = lba;
        stat_bpb_fatsig = true;
        goto found;
    }

    if (!have_fallback)
        return false;

    fat_base_lba = fallback;
    stat_bpb_fatsig = false;

found:
    /*
     * Everything mapped from here on is the volume. This is an upper bound
     * rather than the FAT's own total-sector field, which is the honest thing
     * to report when the map may be incomplete.
     */
    {
        const struct ftl_range *last = &ranges[range_count - 1];

        disk_sector_count = (last->start + last->len) - fat_base_lba;
    }
    return true;
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
#define PROG_BAND_H     80

/*
 * A real bar, not just a count.
 *
 * The numbers alone answer "is it moving", which was the original question,
 * but they answer it badly: 4032/8352 requires reading two figures and doing
 * arithmetic to learn something a filled rectangle says at a glance. During a
 * mount that takes tens of seconds and passes through six phases with
 * different totals, at-a-glance is the whole point.
 *
 * Each phase drives the bar with its own total, so it fills and resets per
 * phase rather than pretending the mount is one uniform job. The phase name
 * above it says which one is running.
 */
#define PROG_BAR_X      8
#define PROG_BAR_Y      52
#define PROG_BAR_W      (LCD_WIDTH - 2 * PROG_BAR_X)
#define PROG_BAR_H      14

/*
 * Absolute pixel positions, not line indices.
 *
 * lcd_putsf() takes a LINE number and draws through the viewport's text
 * cursor, which advances -- so repainting in a loop walked the display
 * steadily downwards instead of overwriting itself. A progress indicator that
 * scrolls is not showing progress, it is producing a log.
 *
 * Drawing at fixed coordinates makes every repaint land on the previous one
 * by construction, with no dependence on cursor or viewport state.
 */
#define PROG_TEXT_X     4
#define PROG_LINE_H     12
#define PROG_LINE0_Y    4

/*
 * Repaint only when the bar would actually look different.
 *
 * Every repaint is a full 103,680-pixel frame -- see lcd_update_rect() -- so
 * painting once per PROG_EVERY units would add minutes to a mount to redraw
 * pixels that did not change. Quantising to PROG_STEPS caps it at a few dozen
 * frames per phase, which is smooth to look at and costs under a second.
 */
#define PROG_STEPS      40

static void ftl_progress_paint(void)
{
    static unsigned last_step = ~0u;
    static const char *last_phase;
    char buf[48];
    unsigned filled = 0;
    unsigned step;

    step = prog_total ? (prog_cur * PROG_STEPS / prog_total) : 0;
    if (step == last_step && prog_phase == last_phase)
        return;
    last_step = step;
    last_phase = prog_phase;

    /*
     * Clear only our band, and do it by drawing background rather than
     * lcd_clear_display(). Clearing the whole display each pass wipes
     * everything else on screen for a region we then never push.
     */
    lcd_set_drawmode(DRMODE_SOLID | DRMODE_INVERSEVID);
    lcd_fillrect(0, 0, LCD_WIDTH, PROG_BAND_H);
    lcd_set_drawmode(DRMODE_SOLID);

    snprintf(buf, sizeof(buf), "FTL %s", prog_phase);
    lcd_putsxy(PROG_TEXT_X, PROG_LINE0_Y, buf);

    snprintf(buf, sizeof(buf), "%u / %u", prog_cur, prog_total);
    lcd_putsxy(PROG_TEXT_X, PROG_LINE0_Y + PROG_LINE_H, buf);

    if (prog_want) {
        snprintf(buf, sizeof(buf), "open %u / %u", prog_found, prog_want);
        lcd_putsxy(PROG_TEXT_X, PROG_LINE0_Y + 2 * PROG_LINE_H, buf);
    }

    /*
     * Clamp rather than trust the ratio. prog_cur is advanced from several
     * loops and a phase that overshoots its estimate would otherwise draw
     * past the end of the frame.
     */
    if (prog_total) {
        unsigned cur = (prog_cur > prog_total) ? prog_total : prog_cur;

        filled = (unsigned)(((uint64_t)cur * (PROG_BAR_W - 2)) / prog_total);
    }

    lcd_drawrect(PROG_BAR_X, PROG_BAR_Y, PROG_BAR_W, PROG_BAR_H);
    if (filled)
        lcd_fillrect(PROG_BAR_X + 1, PROG_BAR_Y + 1, filled, PROG_BAR_H - 2);

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

/*
 * Free space, not a broken address.
 *
 * The context marks unmapped extents with this sentinel. It is far above any
 * real VBA, so a plain range check rejects it -- correctly, but it then counts
 * as an out-of-range drop, which reads like the map losing data it should
 * have kept. On this volume that is 609 records covering ~2.96M LBAs of a
 * 3.86M-sector device: a quarter-full disk, exactly as expected, and nothing
 * missing at all.
 *
 * Counting it separately is the whole point. An out-of-range drop is a bug
 * worth chasing; a hole is the disk saying it is empty there, and conflating
 * the two sends you looking for data that was never written.
 */
#define CXT_VBA_HOLE        0x007fffffu

#define CXT_MAX_SB          32

/*
 * Written superblocks found by the page-0 sweep.
 *
 * SB_COUNT * BANKS is 8352 entries at 16 bytes, so ~134 KB of a 64 MB DRAM.
 * That buys the whole restructure below: the sweep records what it saw
 * instead of throwing it away and re-reading the device to recover it.
 */
#define SB_LIST_MAX         (SB_COUNT * BANKS)

/* Every VBA on the device; anything at or above this is not a real address. */
#define VBA_LIMIT           ((uint32_t)NAND_BLOCKS_PER_CAU * VBA_PER_SB)

struct cxt_sb_ref {
    uint16_t block;
    uint8_t  ce;
    uint8_t  cau;
};

/* One written superblock, with the weave from its first slot. */
struct sb_ent {
    uint64_t weave;
    uint16_t block;
    uint8_t  ce;
    uint8_t  cau;
};

static struct sb_ent sb_list[SB_LIST_MAX];
static unsigned sb_list_count;

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

        /*
         * Count what is dropped. A skipped extent is an LBA range that maps
         * to nothing, and reads of it fail -- which surfaces as "invalid
         * cluster chain" or a failed directory read, i.e. it looks like a
         * corrupt disk rather than a specific hole the driver chose to leave.
         * Dropping is sometimes right (CXT VBAs are context records, not user
         * data), but it must never be invisible.
         */
        if (vba == CXT_VBA_HOLE)
            stat_cxt_hole++;
        else if (vba >= VBA_LIMIT)
            stat_cxt_oob++;
        else if (vba_is_cxt(vba))
            stat_cxt_self++;
        else
            cxt_append(lba, span, vba, weave);

        lba += span;
    }

    cxt_next_lba = lba;
    return 0;
}

/*
 * Walk a checkpoint as ONE superblock spanning all four planes.
 *
 * A checkpoint is not a per-plane object. The same virtual block on every
 * (ce, cau) plane is one superblock, and its records run in native VBA order
 * -- page 0 across all four planes, then page 1:
 *
 *   vblock 989  ofs 0..3   plane 0
 *               ofs 4..7   plane 1
 *               ofs 8..11  plane 2
 *               ofs 12..15 plane 3
 *               ofs 16..   plane 0, page 1
 *
 * This used to read one plane end to end -- every record of plane 0, then
 * every record of plane 1 -- which is the wrong order and only a quarter of
 * the data. The L2V parse cannot survive that: every record declares the LBA
 * it continues from and one whose declared start does not match the running
 * cursor is rejected, so out-of-order records are dropped or attached at the
 * wrong logical position.
 *
 * It also made a single checkpoint look like four candidates with four
 * weaves, which reads like the map being a generation stale when it is really
 * being assembled out of order.
 *
 * We already number VBAs the way the FTL does, so the walk is just an offset
 * loop through vba_to_phys(). One page read serves four consecutive offsets,
 * so this costs the same reads as before and gets all four planes.
 */
static int cxt_load_vblock(unsigned vblock, uint64_t weave)
{
    int last_ce = -1, last_cau = -1, last_page = -1;
    unsigned ofs;

    cxt_lba_valid = false;
    cxt_next_lba = 0;

    for (ofs = 0; ofs < VBA_PER_SB; ofs++) {
        uint32_t vba = (uint32_t)vblock * VBA_PER_SB + ofs;
        uint8_t ce, cau, page, slot;
        uint16_t blk;
        const uint8_t *m;

        vba_to_phys(vba, &ce, &cau, &blk, &page, &slot);

        if (ce != last_ce || cau != last_cau || page != last_page) {
            if (nand_cs_phys_read(ce, cau, blk, page, &scratch))
                return -1;

            last_ce = ce;
            last_cau = cau;
            last_page = page;

            prog_cur = ofs;
            ftl_progress_paint();
        }

        m = scratch.meta_raw[slot];

        if (m[0] != NAND_META_TYPE_SFTL_CXT)
            continue;
        if (m[1] == CXT_TAG_END)
            return 0;
        if (m[1] != CXT_TAG_L2V)
            continue;

        if (cxt_load_l2v(scratch.data[slot], weave))
            return -1;
    }

    return 0;
}

/*
 * How many reads may fail back-to-back, from a cold start, before we accept
 * that the NAND is not talking to us.
 *
 * A device with data on it answers its very first read. Blank superblocks
 * return a page and are classified as free -- they do not fail. So a long
 * unbroken run of hard failures before ANY success does not mean "an empty
 * device", it means the controller is not responding, and every further
 * attempt buys another timeout of the same answer.
 */
#define COLD_FAIL_LIMIT 64

/*
 * Sweep page 0 of every superblock.
 *
 * ONE read per superblock, and it answers three questions at once: is this
 * block free, is it a context, and -- if it is neither -- what is its weave.
 * The weave is the whole point, because it decides which superblocks the
 * context has already accounted for and which still need replaying.
 *
 * The previous version hunted for the context only in the top 256 blocks, on
 * the strength of a Linux comment putting it around block 1960. On this device
 * it usually is not there, and widening that hunt to the whole device would
 * simply have added a second full sweep on top of the classify sweep -- page 0
 * everywhere, then page 127 everywhere, ~16700 reads.
 *
 * So the two are merged. This sweep is the only unconditional pass over the
 * device. Page 127 is then read for the small set of superblocks the context
 * does not already cover, rather than for all 8352 of them.
 *
 * Free blocks cost exactly one read and are never looked at again -- the old
 * arrangement read their BTOC page too, to learn what page 0 had already said.
 */
static bool sb_sweep_page0(void)
{
    unsigned block, ce, cau, n = 0;
    unsigned cold_fails = 0;
    bool any_ok = false;

    cxt_sb_count = cxt_base_count = 0;
    sb_list_count = 0;
    stat_probe_empty = 0;
    stat_cxt_oob = stat_cxt_self = stat_cxt_hole = 0;
    stat_skipped = stat_btoc_fallback = 0;
    stat_btoc_form[0] = stat_btoc_form[1] = 0;
    stat_sweep_readfail = 0;
    stat_bpb_cands = 0;
    stat_bpb_fatsig = false;

    prog_phase = "scan";
    prog_total = SB_COUNT * BANKS;
    prog_cur = 0;
    prog_want = 0;

    for (block = 0; block < SB_COUNT; block++) {
        for (ce = 0; ce < NAND_MAX_CE; ce++) {
            for (cau = 0; cau < NAND_MAX_CAU; cau++) {
                const uint8_t *m;

                prog_cur = ++n;
                if ((n % PROG_EVERY) == 0)
                    ftl_progress_paint();

                /*
                 * Settle the common case at a quarter of the transfer. Most
                 * superblocks are free, and a free one has nothing further to
                 * say -- no weave, no context, no classification.
                 */
                if (nand_cs_probe_empty(ce, cau, block, 0)) {
                    any_ok = true;
                    stat_probe_empty++;
                    continue;
                }

                if (nand_cs_phys_read(ce, cau, block, 0, &scratch)) {
                    /*
                     * A device with data answers its first read. Blank
                     * superblocks return a page and are classified as free --
                     * they do not fail. So an unbroken run of hard failures
                     * before ANY success means the controller is not talking,
                     * and every further attempt buys another timeout of the
                     * same answer. 8352 of those is minutes of nothing.
                     *
                     * Retired permanently after the first success: from then
                     * on failures are ordinary bad blocks and must not abort a
                     * real mount.
                     */
                    if (!any_ok && ++cold_fails >= COLD_FAIL_LIMIT)
                        return false;

                    /*
                     * A block whose page 0 will not read is NOT an empty
                     * block, and dropping it here is the same mistake as
                     * treating an unparsed BTOC as empty: it silently removes
                     * up to 127 pages of data with nothing but a counter to
                     * show for it.
                     *
                     * Page 0 being unreadable says nothing about pages 1..126.
                     * So it goes on the list with a zero weave and the replay
                     * decides -- reading page 127 for a BTOC, and falling back
                     * to the per-page walk, which needs no recognisable
                     * structure at all. The cost is bounded the same way
                     * everything else here is: the walk stops at the first
                     * blank or unreadable page, so a genuinely dead block
                     * costs one more read.
                     */
                    if (sb_list_count < SB_LIST_MAX) {
                        sb_list[sb_list_count].block = (uint16_t)block;
                        sb_list[sb_list_count].ce    = (uint8_t)ce;
                        sb_list[sb_list_count].cau   = (uint8_t)cau;
                        sb_list[sb_list_count].weave = 0;
                        sb_list_count++;
                    }
                    stat_sweep_readfail++;
                    continue;
                }

                any_ok = true;
                m = scratch.meta_raw[0];

                if (m[0] == NAND_META_TYPE_SFTL_CXT) {
                    if (cxt_sb_count < CXT_MAX_SB) {
                        cxt_sbs[cxt_sb_count].block = (uint16_t)block;
                        cxt_sbs[cxt_sb_count].ce    = (uint8_t)ce;
                        cxt_sbs[cxt_sb_count].cau   = (uint8_t)cau;
                        cxt_sb_count++;
                    }
                    if (m[1] == CXT_TAG_BASE) {
                        /*
                         * One candidate per VIRTUAL BLOCK, not per plane.
                         * All four planes of a checkpoint carry the BASE tag,
                         * and counting them separately turns one checkpoint
                         * into four rival candidates -- three of which then
                         * look like older generations that contributed
                         * nothing. Keep the highest weave of the planes.
                         */
                        unsigned k;

                        for (k = 0; k < cxt_base_count; k++)
                            if (cxt_bases[k].block == (uint16_t)block)
                                break;

                        if (k < cxt_base_count) {
                            if (scratch.meta[0].weave > cxt_base_weaves[k])
                                cxt_base_weaves[k] = scratch.meta[0].weave;
                        } else if (cxt_base_count < CXT_MAX_SB) {
                            cxt_bases[k].block = (uint16_t)block;
                            cxt_bases[k].ce    = (uint8_t)ce;
                            cxt_bases[k].cau   = (uint8_t)cau;
                            cxt_base_weaves[k] = scratch.meta[0].weave;
                            cxt_base_count++;
                        }
                    }
                    continue;
                }

                if (page_blank(&scratch))
                    continue;           /* free superblock */

                if (sb_list_count < SB_LIST_MAX) {
                    sb_list[sb_list_count].block = (uint16_t)block;
                    sb_list[sb_list_count].ce    = (uint8_t)ce;
                    sb_list[sb_list_count].cau   = (uint8_t)cau;
                    sb_list[sb_list_count].weave = scratch.meta[0].weave;
                    sb_list_count++;
                }
            }
        }
    }

    return true;
}

/* Load the newest context that parses. */
static bool cxt_load_newest(void)
{
    unsigned i, best;

    cxt_loaded = false;
    cxt_weave = 0;

    while (cxt_base_count) {
        best = 0;
        for (i = 1; i < cxt_base_count; i++) {
            if (cxt_base_weaves[i] > cxt_base_weaves[best])
                best = i;
        }

        prog_phase = "cxt load";
        prog_total = VBA_PER_SB;
        prog_cur = 0;
        ftl_progress_paint();

        range_count = 0;
        stat_mapped = 0;
        ranges_overflowed = false;

        if (cxt_load_vblock(cxt_bases[best].block,
                            cxt_base_weaves[best]) == 0 && range_count) {
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


int ftl_recover(void)
{
    unsigned i;

    if (!nand_hw_present())
        return -1;

    range_count = 0;
    ranges_overflowed = false;
    stat_sbs_closed = stat_sbs_open = stat_mapped = 0;
    ftl_is_ready = false;

    /*
     * One pass over the device, then only what the context cannot account for.
     *
     * This used to be two full sweeps -- classify every superblock from its
     * BTOC page, then sweep the whole device AGAIN to re-find the open ones --
     * and the CXT hunt would have made it three. Page 0 carries everything
     * needed to decide what to do with a superblock, so it is read once and
     * the answers are kept.
     */
    if (!sb_sweep_page0()) {
        prog_phase = "no answer";
        ftl_progress_paint();
        return -1;
    }

    /*
     * The context is the FTL's own map. When it loads, most of the work below
     * simply does not happen.
     */
    cxt_load_newest();

    /*
     * Replay whatever the context does not already cover.
     *
     * With a context loaded this is only the superblocks written AFTER the
     * snapshot -- normally a handful, and exactly where the newest files live.
     * Older ones are skipped without even reading their BTOC page: the context
     * already has them, and map_add() would discard the results as stale.
     *
     * With no context it degrades to the old behaviour, reading the BTOC page
     * of every written superblock. Slower, but the same answer.
     */
    prog_phase = cxt_loaded ? "replay" : "btoc";
    prog_total = sb_list_count;
    prog_cur = 0;
    prog_found = 0;
    prog_want = 0;

    for (i = 0; i < sb_list_count; i++) {
        unsigned s;
        bool is_btoc = false;
        uint64_t weave = 0;

        prog_cur = i + 1;
        if ((prog_cur % PROG_EVERY) == 0)
            ftl_progress_paint();

        /*
         * The skip decision CANNOT be made from the weave recorded by the
         * page-0 sweep, and making it there is a bug that hides itself
         * perfectly.
         *
         * Page 0's weave is stamped when the superblock is OPENED -- it is the
         * oldest thing in the block. A superblock being appended to right up
         * until the moment power went away still has an ancient page 0. So
         * testing it against the checkpoint declares essentially every block
         * on the volume older than the checkpoint, and the replay skips all of
         * them.
         *
         * That is what "replayed 2 of 2308" was. It read like the weave filter
         * working beautifully; it was the filter discarding the entire post-
         * checkpoint history of the device. The resulting map is an exact copy
         * of the checkpoint with nothing after it -- which is precisely the FAT
         * and directory blocks written most recently, hence a volume that
         * builds a plausible map and then will not mount.
         *
         * So read page 127 first and decide from what is actually there.
         */
        if (nand_cs_phys_read(sb_list[i].ce, sb_list[i].cau,
                              sb_list[i].block, NAND_BTOC_PAGE, &scratch))
            continue;

        if (!page_blank(&scratch)) {
            for (s = 0; s < NAND_SLOTS_PER_PAGE; s++) {
                if (scratch.meta[s].valid &&
                    scratch.meta[s].type == NAND_META_TYPE_BTOC) {
                    is_btoc = true;
                    weave = scratch.meta[s].weave;
                    break;
                }
            }
        }

        if (is_btoc) {
            /*
             * Closed. The BTOC weave is written when the block is SEALED, so
             * nothing inside it is newer -- which makes it the only weave here
             * safe to skip on.
             */
            if (cxt_loaded && weave <= cxt_weave) {
                stat_skipped++;
                continue;
            }

            prog_found++;
            stat_sbs_closed++;

            /*
             * A BTOC that parses to nothing must NOT be treated as an empty
             * superblock.
             *
             * btoc_live_pages() understands both record layouts and returns -1
             * for anything else. The Linux side measured what that costs:
             * 255 of 563 closed superblocks read and then dropped, each one
             * 127 pages and roughly 2032 LBAs of directory and file data. The
             * symptom is a map that stops dead at a page boundary and resumes
             * in an unrelated block -- so reads fail, directories will not
             * load, and it presents as an invalid cluster chain rather than as
             * missing data.
             *
             * The fix is not another guess at the layout. Every page carries
             * its own meta, and rebuilding from that is the path open
             * superblocks already use and the one the whole recovery trusts
             * when there is no BTOC at all. It is slower for these blocks and
             * it cannot misread a format it does not recognise, because it
             * never looks at the BTOC.
             *
             * So the BTOC stays the fast path and the per-page walk is the
             * floor underneath it.
             */
            {
                unsigned form = 0;
                int pages = btoc_live_pages(scratch.data[0], NAND_SLOT_DATA,
                                            &form);

                if (pages < 0) {
                    /*
                     * Neither layout parsed. Read the whole block rather than
                     * assume it is empty -- an unparsed BTOC says nothing
                     * about how full the block is, and treating it as empty
                     * is what silently loses 127 pages of directory data.
                     */
                    stat_btoc_fallback++;
                    pages = DATA_PAGES_PER_SB;
                } else {
                    stat_btoc_form[form]++;
                }

                rebuild_sb_pages(sb_list[i].ce, sb_list[i].cau,
                                 sb_list[i].block, (unsigned)pages);
            }
        } else {
            /*
             * Open, and NEVER skipped regardless of the checkpoint.
             *
             * Open means still being written, which is exactly where
             * post-checkpoint data lives, and no single page in it bounds the
             * rest -- there is no weave that can honestly say "everything here
             * predates the snapshot".
             */
            prog_found++;
            stat_sbs_open++;
            rebuild_open_sb(sb_list[i].ce, sb_list[i].cau, sb_list[i].block);
        }
    }

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
         * Report that rather than exporting an offset of zero and letting the
         * FAT layer read gibberish.
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

/*
 * The facts that decide whether a mount failure is a context problem, a scan
 * problem or a FAT problem. Without these a failed mount is one word on a
 * screen and everything after it is guesswork.
 */
void ftl_get_cxt_stats(bool *loaded, unsigned *written, unsigned *empty,
                       unsigned *replayed, unsigned *dropped,
                       unsigned *skipped, bool *overflow,
                       unsigned *fallback, unsigned *form0,
                       unsigned *form1, unsigned *readfail,
                       unsigned *bpb_cands, bool *bpb_fatsig)
{
    if (loaded)
        *loaded = cxt_loaded;
    if (written)
        *written = sb_list_count;
    if (empty)
        *empty = stat_probe_empty;
    if (dropped)
        *dropped = stat_cxt_oob + stat_cxt_self;   /* holes are not drops */
    if (skipped)
        *skipped = stat_skipped;
    if (fallback)
        *fallback = stat_btoc_fallback;
    if (form0)
        *form0 = stat_btoc_form[0];
    if (form1)
        *form1 = stat_btoc_form[1];
    if (readfail)
        *readfail = stat_sweep_readfail;
    if (bpb_cands)
        *bpb_cands = stat_bpb_cands;
    if (bpb_fatsig)
        *bpb_fatsig = stat_bpb_fatsig;
    if (overflow)
        *overflow = ranges_overflowed;
    if (replayed)
        *replayed = prog_found;
}

const char *ftl_last_phase(void)
{
    return prog_phase;
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
