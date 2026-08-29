/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * PCM playback (IIS0) for Apple S5L8740 (iPod nano 7G).
 *
 * Ported from tools/linux-n31/drivers/s5l8740-i2s.c, whose register values
 * come from the RetailOS music-playing oracle rather than from guesswork.
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
#include "audio.h"
#include "audiohw.h"
#include "pcm.h"
#include "pcm-internal.h"
#include "pcm_sink.h"
#include "pl080.h"
#include "clocking-s5l8740.h"
#include "gpio-s5l8740.h"
#include "iis-s5l8740.h"
#include "cs42l81.h"

/*
 * IIS0 @0x3CA00000 -> PL080 DMAC0 peripheral 10 -> CS42L81 on SPI0 -> jack.
 *
 * Every magic value below is the RetailOS oracle captured while stock
 * firmware was playing music. Two of them were got wrong during the Linux
 * bring-up and cost real debugging time, so they are called out:
 *
 *   TXCOM   0x6   -- 0xE was a failure mode
 *   STATUS  0x320 or 0x420 when healthy -- 0x82A0 meant wrong parent clock
 *   CLKDIV  0x110 (272) for 44.1 kHz, i.e. 12 MHz / 44100
 *
 * CLKDIV is at +0x40 (OSOS sub_4F716), NOT the +0x24 that the Rockbox
 * s5l8702 driver uses. Same peripheral family, different register.
 */

#define IIS0_DMAC       0
#define IIS0_TX_FIFO    (IIS0_BASE + I2STXFIFO)

/* Buffer chunking: the PL080 count field is 12 bits of transfers. */
#define MAX_XFER        0xffc

static int tx_channel = -1;

static const void *dma_start;
static size_t      dma_remaining;

/* Rate last programmed via sink_set_freq(), so sink_play() can re-apply it. */
static unsigned int cur_rate = 44100;

static void pcm_dma_arm(void);

/*
 * Rate table from RetailOS. clkdiv is the IIS0 divider; the codec rate code
 * is carried alongside because the CS42L81 driver will need the same table
 * when Phase 4's analog side lands.
 */
struct n31_rate {
    unsigned int rate;
    uint8_t      codec_code;
    uint16_t     clkdiv;
};

static const struct n31_rate n31_rates[] = {
    {  8000,  1, 1500 },
    { 11025,  2, 1088 },
    { 12000,  4, 1000 },
    { 16000,  5,  750 },
    { 22050,  6,  544 },
    { 24000,  8,  500 },
    { 32000,  9,  375 },
    { 44100, 10,  272 },
    { 48000, 12,  250 },
};

/*
 * One resolver, used by both the IIS divider and the codec rate code.
 *
 * They must never resolve independently. In the Linux tree the codec fell
 * back to 44.1 kHz for anything it did not recognise while the IIS side
 * refused the stream outright -- so an out-of-table rate left the codec
 * programmed for one rate and the clock divider set for another. Two halves
 * of one link configured differently is silence or noise, not an error.
 *
 * An exact match wins; otherwise the nearest supported rate by *relative*
 * distance, which keeps substitutions predictable (96000 -> 48000,
 * 5512 -> 8000) rather than collapsing everything onto the default.
 */
unsigned int n31_resolve_rate(unsigned int rate)
{
    unsigned i;
    unsigned best = 44100;
    unsigned long best_err = ~0UL;

    if (!rate)
        return 44100;

    for (i = 0; i < ARRAYLEN(n31_rates); i++) {
        unsigned r = n31_rates[i].rate;
        unsigned long err;

        if (r == rate)
            return rate;

        err = (r > rate) ? (r - rate) : (rate - r);
        err = (err * 100000UL) / r;     /* proportional, not absolute */

        if (err < best_err) {
            best_err = err;
            best = r;
        }
    }

    return best;
}

static uint16_t n31_clkdiv_for(unsigned int rate)
{
    unsigned i;

    rate = n31_resolve_rate(rate);

    for (i = 0; i < ARRAYLEN(n31_rates); i++) {
        if (n31_rates[i].rate == rate)
            return n31_rates[i].clkdiv;
    }
    return 272;     /* unreachable: the resolver only returns table rates */
}

uint8_t iis_codec_rate_code(unsigned int rate)
{
    unsigned i;

    /* Same resolver as the divider, so the two always agree. */
    rate = n31_resolve_rate(rate);

    for (i = 0; i < ARRAYLEN(n31_rates); i++) {
        if (n31_rates[i].rate == rate)
            return n31_rates[i].codec_code;
    }
    return 10;  /* 44.1 kHz */
}

/*
 * True when the codec must run its sample-rate converter rather than clocking
 * the DAC straight off the ASP. OSOS takes the SRC arm for every rate except
 * 48 kHz (rate code 12) -- sub_183138.
 */
bool iis_rate_uses_src(unsigned int rate)
{
    return iis_codec_rate_code(rate) != 12;
}

/*
 * IIS0 pads are GPIO 6, 7 and 20 at function 2 -- SEC pinmux words
 * 0x00061002, 0x00071002 and 0x02041002.
 */
static void iis0_pads(void)
{
    gpio_set_function(6, GPIO_FUNC_ALT2);
    gpio_set_function(7, GPIO_FUNC_ALT2);
    gpio_set_function(20, GPIO_FUNC_ALT2);
}

/* CLKCON+0x30 carries the audio parent selection as a whole dword. */
static void iis_audio_clock(bool playing)
{
    *(REG32_PTR_T)(CLKCON_BASE + CLKCON_AUDIO_OFF) =
        playing ? CLKCON_AUDIO_PLAY : CLKCON_AUDIO_IDLE;
}

/* OSOS sub_26DDDE: program the interface, leaving TX stopped. */
static void iis0_program(unsigned int rate)
{
    iis_audio_clock(true);

    /* sub_C09AC: enable the interface clock. */
    IIS_REG(IIS0_BASE, I2SCLKCON) = 1;

    iis0_pads();

    /* sub_BCB60 with 16-bit samples. */
    IIS_REG(IIS0_BASE, I2STXCON) = I2STXCON_N31_16;
    IIS_REG(IIS0_BASE, I2SRXCON) = I2SRXCON_N31;
    IIS_REG(IIS0_BASE, I2SRXCOM) &= ~4u;

    IIS_REG(IIS0_BASE, I2SCLKDIV) = n31_clkdiv_for(rate);
    IIS_REG(IIS0_BASE, I2SREG44) = I2SREG44_MUSIC;

    /* TXCOM stays 0 until the trigger; OSOS sub_B6620 does the same. */
    IIS_REG(IIS0_BASE, I2STXCOM) = I2STXCOM_STOP;
}

static void pcm_dma_complete(void *data)
{
    (void)data;

    if (dma_remaining) {
        pcm_dma_arm();
        return;
    }

    /* Ask the PCM layer for the next chunk. */
    if (pcm_play_dma_complete_callback(PCM_DMAST_OK, &dma_start,
                                       &dma_remaining)) {
        pcm_dma_arm();
        pcm_play_dma_status_callback(PCM_DMAST_STARTED);
    }
}

static void pcm_dma_arm(void)
{
    size_t chunk = dma_remaining;

    if (chunk > MAX_XFER * 4)
        chunk = MAX_XFER * 4;

    /*
     * The DMA engine reads through the uncached alias so it sees what the
     * codec layer just wrote without a cache clean on every buffer.
     */
    pl080_start_m2p(IIS0_DMAC, tx_channel,
                    S5L8740_UNCACHED_ADDR(dma_start),
                    IIS0_TX_FIFO,
                    PL080_PERI_IIS0_TX, PL080_WIDTH_32, chunk / 4);

    dma_start = (const char *)dma_start + chunk;
    dma_remaining -= chunk;
}

/*
 * OSOS sub_B6620: TXCOM |= 6 once the DMA channel is armed.
 *
 * Bit 3 has to be set as well before the DMA kick, or STATUS sticks at 0x24
 * and the jack stays silent -- found on glass, not in the disassembly.
 */
static void iis0_tx_kick(void)
{
    IIS_REG(IIS0_BASE, I2STXCOM) = I2STXCOM_PIO;
    IIS_REG(IIS0_BASE, I2STXCOM) = I2STXCOM_DMA;
}

static void iis0_tx_stop(void)
{
    IIS_REG(IIS0_BASE, I2STXCOM) = I2STXCOM_STOP;

    if (tx_channel >= 0)
        pl080_stop(IIS0_DMAC, tx_channel);

    IIS_REG(IIS0_BASE, I2SCLKCON) = 0;
    iis_audio_clock(false);
}

static void sink_init(void)
{
    if (tx_channel < 0) {
        tx_channel = pl080_alloc_channel(IIS0_DMAC);
        if (tx_channel >= 0)
            pl080_set_callback(IIS0_DMAC, tx_channel, pcm_dma_complete, NULL);
    }

    iis0_program(HW_SAMPR_DEFAULT);
}

static void sink_postinit(void)
{
    audiohw_postinit();
}

static void sink_set_freq(uint16_t freq)
{
    cur_rate = hw_freq_sampr[freq];
    IIS_REG(IIS0_BASE, I2SCLKDIV) = n31_clkdiv_for(cur_rate);
}

static void sink_lock(void)
{
    if (tx_channel >= 0)
        VIC0INTENCLEAR = 1 << IRQ_DMAC0;
}

static void sink_unlock(void)
{
    if (tx_channel >= 0)
        VIC0INTENABLE = 1 << IRQ_DMAC0;
}

static void sink_play(const void *addr, size_t size)
{
    if (tx_channel < 0)
        return;

    dma_start = addr;
    dma_remaining = size;

    iis0_program(cur_rate);
    pcm_dma_arm();
    iis0_tx_kick();

    /*
     * Only now are BCLK and LRCLK actually running, so this is the first
     * moment the codec can lock its serial port. Doing it at codec init --
     * against a dead clock -- completes every register write and achieves
     * nothing, which is the leading explanation for the Linux silence.
     */
#ifdef HAVE_CS42L81
    if (!cs42l81_asp_is_locked())
        cs42l81_asp_lock();
#endif
}

static void sink_stop(void)
{
    iis0_tx_stop();
    dma_remaining = 0;
}

struct pcm_sink builtin_pcm_sink = {
    .caps = {
        .samprs       = hw_freq_sampr,
        .num_samprs   = HW_NUM_FREQ,
        .default_freq = HW_FREQ_DEFAULT,
        .volume_type  = PCM_NATIVE_VOLUME_TYPE,
    },
    .ops = {
        .init     = sink_init,
        .postinit = sink_postinit,
        .set_freq = sink_set_freq,
        .lock     = sink_lock,
        .unlock   = sink_unlock,
        .play     = sink_play,
        .stop     = sink_stop,
    },
};

const void *pcm_play_dma_get_peak_buffer(int *count)
{
    *count = dma_remaining / 4;
    return dma_start;
}

size_t pcm_get_bytes_waiting(void)
{
    return dma_remaining;
}

#ifdef HAVE_PCM_DMA_ADDRESS
void *pcm_dma_addr(void *addr)
{
    return S5L8740_UNCACHED_ADDR(addr);
}
#endif

uint32_t iis0_status(void)
{
    return IIS_REG(IIS0_BASE, I2SSTATUS);
}
