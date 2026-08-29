/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
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
#ifndef __PL080_S5L8740_H__
#define __PL080_S5L8740_H__

#include <stdbool.h>
#include <stdint.h>

/*
 * Two PL080 controllers: DMAC0 @0x38200000 and DMAC1 @0x38700000, eight
 * channels each. Note these are NOT at 0x38400000 -- that is the DWC2 USB
 * controller, which an early version of the Linux map had confused with the
 * DMA engine.
 *
 * Peripheral request ids, from the RetailOS oracle:
 *   10  IIS0 TX   (destination 0x3ca00010)   -- confirmed
 *   11  IIS0 RX                              -- assumed
 *   13  IIS2 RX   (source 0x3d400038)        -- FM / BT PCM capture
 *
 * Peripheral 12 was tried and observed stuck Active on glass; 10 is the one
 * that walks. Rockbox's own s5l8702 IIS0 TX also uses 0xA.
 */
#define PL080_NDMAC         2
#define PL080_NCHANNELS     8

#define PL080_PERI_IIS0_TX  10
#define PL080_PERI_IIS0_RX  11
#define PL080_PERI_IIS2_RX  13

/* Flow control, PL080 CFG bits 13:11 */
#define PL080_FLOW_M2M      0
#define PL080_FLOW_M2P      1
#define PL080_FLOW_P2M      2

/* Transfer width, CTL bits 20:18 and 23:21 */
#define PL080_WIDTH_8       0
#define PL080_WIDTH_16      1
#define PL080_WIDTH_32      2

struct pl080_channel;

typedef void (*pl080_callback)(void *data);

void pl080_init(void);

/*
 * Claim a channel on the given controller, or -1 if none is free.
 * dmac is 0 or 1.
 */
int pl080_alloc_channel(int dmac);
void pl080_free_channel(int dmac, int channel);

/* Register a transfer-complete callback for a channel. */
void pl080_set_callback(int dmac, int channel, pl080_callback cb, void *data);

/*
 * Start a memory-to-peripheral transfer. src must be a physical (uncached)
 * address; count is in transfers, not bytes.
 */
void pl080_start_m2p(int dmac, int channel, const void *src, uint32_t dst_reg,
                     int peri, int width, int count);

/*
 * Start a peripheral-to-memory transfer.
 */
void pl080_start_p2m(int dmac, int channel, uint32_t src_reg, void *dst,
                     int peri, int width, int count);

void pl080_stop(int dmac, int channel);
bool pl080_channel_active(int dmac, int channel);

#endif /* __PL080_S5L8740_H__ */
