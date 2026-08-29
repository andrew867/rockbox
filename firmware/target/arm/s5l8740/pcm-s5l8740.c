/***************************************************************************
 * PCM playback for Apple S5L8740 (iPod nano 7G) -- Phase 4 placeholder.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#include "config.h"
#include "system.h"
#include "audio.h"
#include "audiohw.h"
#include "pcm.h"
#include "pcm-internal.h"
#include "pcm_sink.h"

/*
 * The transport is IIS0 @0x3CA00000 -> PL080 DMAC0 peripheral 10 (destination
 * 0x3ca00010). RetailOS runs it with TXCOM 0x6 and CLKDIV 0x110 for 44.1 kHz,
 * and a healthy interface reads STATUS 0x320 or 0x420 -- the Linux port's
 * TXCOM 0xE and STATUS 0x82A0 were both wrong and cost real debugging time.
 *
 * None of that is wired up yet. Phase 4 is gated on the analog path working,
 * which is currently failing in the Linux tree at roughly -66 dBFS against
 * RetailOS's -16 dBFS. Running silent is the honest behaviour until then, and
 * it keeps the rest of the firmware alive so the display and UI work can
 * proceed.
 */

static int sink_locked = 0;

static void sink_init(void)
{
}

static void sink_postinit(void)
{
    audiohw_postinit();
}

static void sink_set_freq(uint16_t freq)
{
    (void)freq;
}

static void sink_lock(void)
{
    ++sink_locked;
}

static void sink_unlock(void)
{
    --sink_locked;
}

static void sink_play(const void *addr, size_t size)
{
    (void)addr;
    (void)size;
}

static void sink_stop(void)
{
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
    *count = 0;
    return NULL;
}

size_t pcm_get_bytes_waiting(void)
{
    return 0;
}
