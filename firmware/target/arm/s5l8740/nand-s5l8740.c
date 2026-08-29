/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * S5L8740 NAND controller (FMSS/FIL) for the iPod nano 7G.
 *
 * Ported from tools/linux-n31/drivers/nand-s5l8740.c.
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
#include "nand-s5l8740.h"
#include "nand-s5l8740-seq.h"
#include "clocking-s5l8740.h"
#include "boot-beacon.h"

/*
 * The FMSS is a sequencer, not a register-poke NAND controller. Reads are
 * driven by uploading a microcode program plus a descriptor list and kicking
 * it; the sequencer then walks the NAND and DMAs data and spare into buffers
 * we point it at.
 *
 * That is why so little of this looks like a normal MTD driver, and why the
 * command sequence blob is carried verbatim -- it was extracted from OSOS
 * rather than reconstructed, and it is not something to hand-edit.
 */
#define FMCTRL0     0x00
#define FMCTRL1     0x04
#define FMCMD       0x08
#define FMADDR      0x0c
#define FMCE        0x14
#define FMCYCLES    0x2c
#define FMLEN       0x30
#define NANDSTAT    0x4c
#define FMDATA      0x80
#define FMSEQ       0xc00
#define FMSEQBASE   0xc04
#define FMSEQSTAT   0xc08
#define FMSEQIRQ    0xc0c
#define FMGEN0      0xd00
#define FMGEN1      0xd04
#define FMGEN2      0xd08
#define FMGEN3      0xd0c
#define FMGEN4      0xd10
#define FMGEN5      0xd14

#define FMR(off)    (*(REG32_PTR_T)(FMC_BASE + (off)))

#define DMA_STATUS_LEN      512
#define DMA_CMDLIST_LEN     256
#define DMA_SPARE_LEN       256
#define FMSS_SECTOR_LEN     4096
#define FMSS_PAGE_LEN       (16 * 1024)

/*
 * Values recovered from the stock read path (OSOS 4EDDDC / D39EC). None of
 * these are tunable guesses -- each one came out of a working capture.
 */
#define PAGE_CTRL0_OR       0x20011000  /* D04 timing template */
#define DMA_D14             7           /* address cycles - 1; >=7 selects v40 */
#define DMA_C6C             16
#define DMA_KICK            0xfff5      /* NOT 0x80000, which is a reset */

/*
 * Completion budget for one sequencer kick.
 *
 * Was 200000 (200 ms), which is roughly 2000x how long a NAND page read
 * actually takes and ten times what the Linux driver allows on the same
 * silicon -- it polls 2000 iterations of udelay(10), so 20 ms, and only
 * stretches to 200 ms on the no-IRQ fallback path.
 *
 * The number does not matter when reads succeed; it decides entirely how long
 * a FAILING scan takes to admit it. Pass one is 8352 superblock reads, so at
 * 200 ms apiece a device that never answers costs 28 minutes per pass, twice,
 * with a motionless logo on screen. At 20 ms it is under three minutes, and
 * the cold-failure budget in ftl_recover() cuts the common case to about a
 * second. Linux mounts this disk in 30 seconds; nothing here should be
 * allowed to take an hour to reach the same conclusion.
 */
#define CS_POLL_US          20000

/*
 * DMA buffers. These are accessed through the uncached DRAM alias, so the
 * sequencer sees what we wrote and we see what it wrote without any cache
 * maintenance in the middle.
 */
static uint8_t nand_seq[FMSS_SEQ_READ_LEN]  __attribute__((aligned(64)));
static uint8_t nand_data[FMSS_PAGE_LEN]     __attribute__((aligned(64)));
static uint8_t nand_spare[DMA_SPARE_LEN]    __attribute__((aligned(64)));
static uint8_t nand_stbuf[DMA_STATUS_LEN]   __attribute__((aligned(64)));
static uint32_t nand_cmdl[DMA_CMDLIST_LEN / 4] __attribute__((aligned(64)));

static bool nand_ready;
static unsigned pages_since_reset;

/*
 * S5L8740_UNCACHED_ADDR() casts through typeof(), which an array cannot be
 * cast to, so decay to a pointer before handing it over.
 */
#define UNCACHED(p) ((void *)((uintptr_t)(void *)(p) + 0x40000000u))
#define PHYS(p)     ((uint32_t)(uintptr_t)(p))

int nand_hw_reset(void)
{
    FMR(FMCTRL0) = 1;
    udelay(10);
    FMR(FMCTRL1) = 0xffffffff;
    udelay(10);
    pages_since_reset = 0;
    return 0;
}

bool nand_hw_present(void)
{
    return nand_ready;
}

bool nand_hw_init(void)
{
    beacon_mark(BEACON_CYAN);

    /*
     * Redundant in practice: clocking_init() writes 0 to every PWRCON bank at
     * system_init() time, exactly as the Linux driver's ungate-all does, so
     * this bit is already clear when we arrive. Kept because it documents
     * which gate owns the FMC and costs one store -- but it does mean a stall
     * on the FMCTRL writes below would NOT be an unclocked-block stall. That
     * removes the obvious suspect rather than confirming it.
     */
    clockgate_enable(CLKCON_PWRCON0, (1 << 4), true);

    nand_hw_reset();

    nand_ready = true;
    return true;
}

void nand_meta_decode(const uint8_t *m16, struct nand_meta *out)
{
    unsigned i;
    bool blank = true;

    if (!out)
        return;

    memset(out, 0, sizeof(*out));
    if (!m16)
        return;

    /*
     * Layout, proven on glass by the "lba mismatch" check:
     *   +0   type
     *   +1   flags
     *   +2   48-bit weave sequence, little-endian
     *   +8   fmss_lba, little-endian
     */
    out->type = m16[0];
    out->flags = m16[1];
    out->weave = (uint64_t)m16[2]
               | ((uint64_t)m16[3] << 8)
               | ((uint64_t)m16[4] << 16)
               | ((uint64_t)m16[5] << 24)
               | ((uint64_t)m16[6] << 32)
               | ((uint64_t)m16[7] << 40);
    out->lba = (uint32_t)m16[8]
             | ((uint32_t)m16[9] << 8)
             | ((uint32_t)m16[10] << 16)
             | ((uint32_t)m16[11] << 24);

    /*
     * Blank means erased or never written. Both 0x00 and 0xFF fills occur,
     * so a record is only blank if every byte is one of those two.
     */
    for (i = 0; i < NAND_SLOT_META; i++) {
        if (m16[i] != 0x00 && m16[i] != 0xff) {
            blank = false;
            break;
        }
    }

    out->blank = blank;
    out->valid = !blank;
}

int nand_meta_pick_lba(const struct nand_cs_page *page, uint32_t fmss_lba)
{
    int best = -1;
    uint64_t best_weave = 0;
    unsigned s;

    if (!page)
        return -1;

    /*
     * Several slots can claim the same LBA -- that is what an out-of-place
     * update looks like before garbage collection. The newest weave wins;
     * anything else hands back stale data that looks perfectly valid.
     */
    for (s = 0; s < NAND_SLOTS_PER_PAGE; s++) {
        const struct nand_meta *cur = &page->meta[s];

        if (!nand_meta_is_data(cur) || cur->lba != fmss_lba)
            continue;

        if (best < 0 || cur->weave > best_weave) {
            best = (int)s;
            best_weave = cur->weave;
        }
    }

    return best;
}

/* Clear stale completion/error and refuse to kick into a busy sequencer. */
static bool cs_preflight(void)
{
    uint32_t c0c = FMR(FMSEQIRQ);

    if (c0c & 0x0d) {
        FMR(FMSEQIRQ) = c0c & 0x0d;
        (void)FMR(FMSEQIRQ);
        udelay(10);
        c0c = FMR(FMSEQIRQ);
    }

    return (c0c & 0x0d) == 0;
}

static int cs_wait_complete(unsigned timeout_us)
{
    unsigned stop = USEC_TIMER + timeout_us;

    while (1) {
        uint32_t c0c = FMR(FMSEQIRQ);

        if (c0c & 0x0d)
            return ((c0c & 0x0d) == 1) ? 0 : -1;

        if (TIME_AFTER(USEC_TIMER, stop))
            return -1;
    }
}

/*
 * One page read as `span` records starting at `slot`.
 *
 * The descriptor list is the shape recovered from OSOS 4EDDDC:
 *
 *   desc0  [0] CE select, bit31 set because it is the last for this CE
 *          [2] column/length: (rec * span) | ((rec * slot) << 16)
 *          [3] the packed physical page address
 *   desc1  [0] CE select | 1
 *          [1] number of records
 *          [2] spare buffer address
 *          [3] data buffer address
 *   term   0x00010002
 */
static int fmss_dma_page_read(unsigned ce, uint32_t addr,
                              unsigned slot, unsigned span)
{
    uint32_t *cl = UNCACHED(nand_cmdl);
    uint32_t ce_bit = 1u << (16 + ce);
    uint32_t col_len;
    int ret;

    if (ce > 7 || span < 1 || span > 4 || slot > 3 || slot + span > 4)
        return -1;

    /*
     * Refresh the microcode and wipe every device-visible buffer before each
     * kick. The sequencer is stateful enough that reusing a dirty command
     * list produces reads that look plausible and are wrong.
     */
    memcpy(UNCACHED(nand_seq), fmss_seq_read_blob, FMSS_SEQ_READ_LEN);
    memset(UNCACHED(nand_data), 0, FMSS_PAGE_LEN);
    memset(UNCACHED(nand_spare), 0, DMA_SPARE_LEN);
    memset(UNCACHED(nand_stbuf), 0, DMA_STATUS_LEN);
    memset(cl, 0, DMA_CMDLIST_LEN);

    col_len = (NAND_REC_BYTES * span) | ((NAND_REC_BYTES * slot) << 16);

    cl[0] = ce_bit | 0x80000000u;
    cl[1] = 0;
    cl[2] = col_len;
    cl[3] = addr;
    cl[4] = ce_bit | 1u;
    cl[5] = span;
    cl[6] = PHYS(UNCACHED(nand_spare));
    cl[7] = PHYS(UNCACHED(nand_data));
    cl[8] = 0x00010002u;

    FMR(0xc6c) = DMA_C6C;

    if (!cs_preflight())
        return -1;

    FMR(FMGEN1) = PAGE_CTRL0_OR;                    /* timing template */
    FMR(FMGEN2) = PHYS(UNCACHED(nand_cmdl));        /* descriptor list */
    FMR(FMGEN3) = FMSS_SECTOR_LEN;
    FMR(FMGEN4) = PHYS(UNCACHED(nand_stbuf));       /* status */
    FMR(FMGEN5) = DMA_D14;                          /* address cycles - 1 */
    FMR(FMSEQBASE) = PHYS(UNCACHED(nand_seq));      /* microcode */

    /*
     * Do NOT poke +0x81C here. That is the ECC block from a different code
     * path, and writing it before a CS kick has correlated with SoC wedges.
     */

    /* Flush the posted MMIO programming before the kick. */
    (void)FMR(FMGEN2);
    (void)FMR(FMGEN3);
    (void)FMR(FMGEN4);
    (void)FMR(FMGEN5);
    (void)FMR(FMSEQBASE);

    FMR(FMSEQ) = DMA_KICK;
    (void)FMR(FMSEQ);

    ret = cs_wait_complete(CS_POLL_US);

    /* Acknowledge and settle regardless of outcome. */
    FMR(FMSEQIRQ) = 13;
    {
        unsigned i;

        for (i = 0; i < 10000; i++) {
            if ((FMR(FMSEQIRQ) & 0xd) == 0)
                break;
            udelay(1);
        }
    }
    FMR(FMCTRL0) = 1;

    return ret;
}

/*
 * Cheap "is this superblock free" probe.
 *
 * The classify sweep reads page 0 of all 8352 superblocks and, for the great
 * majority of them, learns one bit: empty. It was paying for a full four-record
 * page -- 4 x (4096 + 16) = 16448 bytes -- to obtain it. Across the sweep that
 * is roughly 130 MB of NAND transfer to answer a question worth a few hundred
 * kilobytes, and on a device where most blocks are free it dominates mount
 * time.
 *
 * The erase test only ever looks at slot 0: its meta, plus the head of its
 * data. fmss_dma_page_read() already took a (slot, span) pair and was simply
 * always called with (0, 4). A span of 1 moves 4112 bytes -- a quarter -- and
 * answers the common case outright.
 *
 * Two deliberate limits, because this is the storage path:
 *
 *   It returns a verdict, not data. Slots 1-3 are genuinely not read, so
 *   handing back a struct nand_cs_page would be handing back three-quarters
 *   stale buffer. Nothing can mistake a bool for a cheap general read.
 *
 *   "Not conclusively empty" is the only other answer it gives. A block that
 *   fails this probe is re-read in full, because classification needs every
 *   slot's meta -- a context superblock can be identified by ANY slot, so
 *   deciding that from slot 0 alone would silently misfile it. The saving is
 *   confined to blocks where one record is genuinely the whole answer.
 */
bool nand_cs_probe_empty(uint8_t ce, uint8_t cau, uint16_t block, uint8_t page)
{
    const uint8_t *d, *m;
    uint32_t addr;
    unsigned i;

    if (!nand_ready)
        return false;
    if (ce >= NAND_MAX_CE || cau >= NAND_MAX_CAU ||
        block >= NAND_BLOCKS_PER_CAU || page > NAND_BTOC_PAGE)
        return false;

    addr = nand_ppn_addr(cau, block, page, 0);

    if (fmss_dma_page_read(ce, addr, 0, 1)) {
        pages_since_reset++;
        return false;           /* read failed: let the full path decide */
    }
    pages_since_reset++;

    m = (const uint8_t *)UNCACHED(nand_spare);
    for (i = 0; i < NAND_SLOT_META; i++) {
        if (m[i] != 0x00 && m[i] != 0xff)
            return false;
    }

    /*
     * Head of the data only. An erased NAND page is uniform, so 64 bytes is
     * as conclusive as 4096 and costs nothing extra -- the transfer already
     * happened; this is just how much of it we bother to look at.
     */
    d = (const uint8_t *)UNCACHED(nand_data);
    for (i = 0; i < 64; i++) {
        if (d[i] != 0x00 && d[i] != 0xff)
            return false;
    }

    return true;
}

int nand_cs_phys_read(uint8_t ce, uint8_t cau, uint16_t block, uint8_t page,
                      struct nand_cs_page *out)
{
    uint32_t addr;
    unsigned s;
    int ret;

    if (!nand_ready || !out)
        return -1;
    if (ce >= NAND_MAX_CE || cau >= NAND_MAX_CAU ||
        block >= NAND_BLOCKS_PER_CAU || page > NAND_BTOC_PAGE)
        return -1;

    memset(out, 0, sizeof(*out));
    addr = nand_ppn_addr(cau, block, page, 0);

    ret = fmss_dma_page_read(ce, addr, 0, NAND_SLOTS_PER_PAGE);
    pages_since_reset++;

    if (ret)
        return ret;

    for (s = 0; s < NAND_SLOTS_PER_PAGE; s++) {
        const uint8_t *d = (const uint8_t *)UNCACHED(nand_data)
                         + s * NAND_SLOT_DATA;
        const uint8_t *m = (const uint8_t *)UNCACHED(nand_spare)
                         + s * NAND_SLOT_META;

        memcpy(out->data[s], d, NAND_SLOT_DATA);
        memcpy(out->meta_raw[s], m, NAND_SLOT_META);
        nand_meta_decode(out->meta_raw[s], &out->meta[s]);
    }

    return 0;
}
