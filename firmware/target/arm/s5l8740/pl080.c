/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * PL080 DMA controllers for Apple S5L8740 (iPod nano 7G).
 *
 * Ported from tools/linux-n31/drivers/dma-s5l8740-pl080.c.
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
#include "config.h"
#include "system.h"
#include "panic.h"
#include "pl080.h"

#define INT_STATUS      0x00
#define INT_TC_STATUS   0x04
#define INT_TC_CLEAR    0x08
#define INT_ERR_STATUS  0x0c
#define INT_ERR_CLEAR   0x10
#define RAW_TC          0x14
#define RAW_ERR         0x18
#define ENBLD_CHNS      0x1c
#define SOFT_BREQ       0x20
#define SOFT_SREQ       0x24
#define DMA_CONFIG      0x30
#define DMA_SYNC        0x34

#define Cx_SRC(i)       (0x100 + (i) * 0x20)
#define Cx_DST(i)       (0x104 + (i) * 0x20)
#define Cx_LLI(i)       (0x108 + (i) * 0x20)
#define Cx_CTL(i)       (0x10c + (i) * 0x20)
#define Cx_CFG(i)       (0x110 + (i) * 0x20)

#define CONFIG_EN       (1 << 0)

#define CTL_SBSIZE_SHIFT    12
#define CTL_DBSIZE_SHIFT    15
#define CTL_SWIDTH_SHIFT    18
#define CTL_DWIDTH_SHIFT    21
#define CTL_SRC_AI          (1 << 26)
#define CTL_DST_AI          (1 << 27)
#define CTL_PROT_PRIV       (1 << 28)
#define CTL_TC_IRQ          (1 << 31)

#define CFG_ENABLE          (1 << 0)
#define CFG_SRC_PERI_SHIFT  1
#define CFG_DST_PERI_SHIFT  6
#define CFG_FLOW_SHIFT      11
#define CFG_IE              (1 << 14)
#define CFG_ITC             (1 << 15)

/* Burst size 4 for both sides -- the value the stock IIS0 path uses. */
#define BURST_4             1

#define TERM_POLL_US        10
#define TERM_POLL_MAX       10

static const uint32_t dmac_base[PL080_NDMAC] = { DMAC0_BASE, DMAC1_BASE };

#define DMA_REG(d, off) (*(REG32_PTR_T)(dmac_base[d] + (off)))

/*
 * How many DMA errors a channel may raise before it is refused.
 *
 * Restarting a channel that errors on every attempt just resumes the storm,
 * and eight is well past the point where a transient becomes a pattern.
 */
#define CH_ERR_LIMIT    8

struct pl080_ch_state {
    pl080_callback  cb;
    void           *data;
    bool            allocated;
    uint8_t         errors;
    bool            stuck;
};

static struct pl080_ch_state ch_state[PL080_NDMAC][PL080_NCHANNELS];

static void pl080_irq(int dmac)
{
    uint32_t tc = DMA_REG(dmac, INT_TC_STATUS);
    uint32_t err = DMA_REG(dmac, INT_ERR_STATUS);
    int ch;

    if (err) {
        /*
         * DISABLE THE CHANNEL BEFORE CLEARING THE LATCH.
         *
         * Clearing an error while the channel is still enabled means whatever
         * raised it raises it again immediately, and the handler clears it
         * again. On a single core that is an interrupt storm, which from
         * outside is indistinguishable from a lockup -- and it explains a
         * failure that arrives after a few seconds of working playback rather
         * than at the first period.
         *
         * This used to panicf() instead, which avoids the storm by killing
         * the device outright. That is not better. A DMA error on an audio
         * channel is a recoverable audio fault, and turning it into a dead
         * machine loses every other thing the firmware could have told us --
         * the difference between a bug and a bug you cannot debug.
         *
         * So: stop the channel, clear the latch, count it, and after
         * CH_ERR_LIMIT refuse to start it again. Audio stops; the device
         * stays up and can say why.
         */
        for (ch = 0; ch < PL080_NCHANNELS; ch++) {
            if (!(err & (1 << ch)))
                continue;

            DMA_REG(dmac, Cx_CFG(ch)) = 0;

            if (ch_state[dmac][ch].errors < CH_ERR_LIMIT &&
                ++ch_state[dmac][ch].errors >= CH_ERR_LIMIT)
                ch_state[dmac][ch].stuck = true;
        }

        DMA_REG(dmac, INT_ERR_CLEAR) = err;
    }

    if (!tc)
        return;

    DMA_REG(dmac, INT_TC_CLEAR) = tc;

    for (ch = 0; ch < PL080_NCHANNELS; ch++) {
        if ((tc & (1 << ch)) && ch_state[dmac][ch].cb)
            ch_state[dmac][ch].cb(ch_state[dmac][ch].data);
    }
}

void INT_DMAC0(void)
{
    pl080_irq(0);
}

void INT_DMAC1(void)
{
    pl080_irq(1);
}

void pl080_init(void)
{
    int d, ch;

    for (d = 0; d < PL080_NDMAC; d++) {
        /* Stop every channel before enabling the controller. */
        for (ch = 0; ch < PL080_NCHANNELS; ch++) {
            DMA_REG(d, Cx_CFG(ch)) = 0;
            ch_state[d][ch].cb = NULL;
            ch_state[d][ch].data = NULL;
            ch_state[d][ch].allocated = false;
        }

        DMA_REG(d, INT_TC_CLEAR) = 0xff;
        DMA_REG(d, INT_ERR_CLEAR) = 0xff;
        DMA_REG(d, DMA_CONFIG) = CONFIG_EN;
    }

    VIC0INTENABLE = 1 << IRQ_DMAC0;
    VIC0INTENABLE = 1 << IRQ_DMAC1;
}

int pl080_alloc_channel(int dmac)
{
    int ch;
    int oldstatus = disable_irq_save();

    for (ch = 0; ch < PL080_NCHANNELS; ch++) {
        if (!ch_state[dmac][ch].allocated) {
            ch_state[dmac][ch].allocated = true;
            restore_irq(oldstatus);
            return ch;
        }
    }

    restore_irq(oldstatus);
    return -1;
}

void pl080_free_channel(int dmac, int channel)
{
    pl080_stop(dmac, channel);
    ch_state[dmac][channel].cb = NULL;
    ch_state[dmac][channel].data = NULL;
    ch_state[dmac][channel].allocated = false;
    ch_state[dmac][channel].errors = 0;
    ch_state[dmac][channel].stuck = false;
}

void pl080_set_callback(int dmac, int channel, pl080_callback cb, void *data)
{
    int oldstatus = disable_irq_save();

    ch_state[dmac][channel].cb = cb;
    ch_state[dmac][channel].data = data;

    restore_irq(oldstatus);
}

static uint32_t pl080_ctl(int width, int count, bool src_ai, bool dst_ai)
{
    return (count & 0xfff)
         | (BURST_4 << CTL_SBSIZE_SHIFT)
         | (BURST_4 << CTL_DBSIZE_SHIFT)
         | (width << CTL_SWIDTH_SHIFT)
         | (width << CTL_DWIDTH_SHIFT)
         | (src_ai ? CTL_SRC_AI : 0)
         | (dst_ai ? CTL_DST_AI : 0)
         | CTL_TC_IRQ;
    /*
     * No CTL_PROT_PRIV. RetailOS plays music with CTL = 0x84249000, and its
     * protection field is zero -- Prot=0, SI=1, width=16, SB=1, DB=1, both
     * AHB masters 0, TC_IRQ=1.
     *
     * The Linux port measured what setting it costs: with Prot=PRIV|BUFF the
     * control word came out 0xb5242000 and the IIS STATUS stuck in the 0x2A0
     * class instead of retail's 0x320. A protection attribute is not
     * something the peripheral should care about, and on this bus it plainly
     * does.
     */
}

void pl080_start_m2p(int dmac, int channel, const void *src, uint32_t dst_reg,
                     int peri, int width, int count)
{
    /*
     * A channel that has errored CH_ERR_LIMIT times is not restarted.
     * Handing it another descriptor only reproduces the fault, and the
     * handler would stop it again on the next interrupt.
     */
    if (ch_state[dmac][channel].stuck)
        return;

    DMA_REG(dmac, Cx_CFG(channel)) = 0;

    DMA_REG(dmac, Cx_SRC(channel)) = (uint32_t)src;
    DMA_REG(dmac, Cx_DST(channel)) = dst_reg;
    DMA_REG(dmac, Cx_LLI(channel)) = 0;
    DMA_REG(dmac, Cx_CTL(channel)) = pl080_ctl(width, count, true, false);

    DMA_REG(dmac, Cx_CFG(channel)) =
        CFG_ENABLE
      | (peri << CFG_DST_PERI_SHIFT)
      | (PL080_FLOW_M2P << CFG_FLOW_SHIFT)
      | CFG_IE | CFG_ITC;
}

void pl080_start_p2m(int dmac, int channel, uint32_t src_reg, void *dst,
                     int peri, int width, int count)
{
    /*
     * A channel that has errored CH_ERR_LIMIT times is not restarted.
     * Handing it another descriptor only reproduces the fault, and the
     * handler would stop it again on the next interrupt.
     */
    if (ch_state[dmac][channel].stuck)
        return;

    DMA_REG(dmac, Cx_CFG(channel)) = 0;

    DMA_REG(dmac, Cx_SRC(channel)) = src_reg;
    DMA_REG(dmac, Cx_DST(channel)) = (uint32_t)dst;
    DMA_REG(dmac, Cx_LLI(channel)) = 0;
    DMA_REG(dmac, Cx_CTL(channel)) = pl080_ctl(width, count, false, true);

    DMA_REG(dmac, Cx_CFG(channel)) =
        CFG_ENABLE
      | (peri << CFG_SRC_PERI_SHIFT)
      | (PL080_FLOW_P2M << CFG_FLOW_SHIFT)
      | CFG_IE | CFG_ITC;
}

void pl080_stop(int dmac, int channel)
{
    int i;

    /*
     * Clearing ENABLE outright would drop whatever is already in the
     * channel FIFO. Halt first, let the outstanding burst drain, then
     * disable -- the same order the Linux driver uses.
     */
    DMA_REG(dmac, Cx_CFG(channel)) |= (1 << 18);    /* HALT */

    for (i = 0; i < TERM_POLL_MAX; i++) {
        if (!(DMA_REG(dmac, ENBLD_CHNS) & (1 << channel)))
            break;
        udelay(TERM_POLL_US);
    }

    DMA_REG(dmac, Cx_CFG(channel)) &= ~CFG_ENABLE;
    DMA_REG(dmac, INT_TC_CLEAR) = 1 << channel;
    DMA_REG(dmac, INT_ERR_CLEAR) = 1 << channel;
}

bool pl080_channel_active(int dmac, int channel)
{
    return !!(DMA_REG(dmac, ENBLD_CHNS) & (1 << channel));
}
