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
#ifndef __SPI_S5L8740_H__
#define __SPI_S5L8740_H__

#include <stdbool.h>
#include <stdint.h>

/*
 * Two SPI instances are wired on N31:
 *
 *   SPI0 @0x3C300000  CS42L81 / 338S1146 audio codec control
 *   SPI2 @0x3D200000  TI 343S0538 "Nimbus" touchscreen
 *
 * SPI1 and SPI3 exist in the SoC but are not routed to anything on this board.
 */
#define SPI_PORT_CODEC  0
#define SPI_PORT_TOUCH  2

void spi_init(void);

/* Bring one port up. Safe to call again; re-runs the engine setup. */
void spi_port_init(int port);

/*
 * Full-duplex PIO transfer. Either buffer may be NULL:
 *   tx == NULL  receive only (the engine self-clocks)
 *   rx == NULL  transmit only
 * Chip select is asserted for the whole call. Returns 0, or negative on
 * timeout.
 */
int spi_transfer(int port, const uint8_t *tx, uint8_t *rx, int len);

/*
 * As spi_transfer(), but able to leave chip select asserted when the frame
 * continues into the next call. Some protocols here frame on CS rather than
 * on length -- the Nimbus HBPP upload in particular -- and dropping CS
 * mid-frame silently splits it in two.
 */
int spi_transfer_cs(int port, const uint8_t *tx, uint8_t *rx, int len,
                    bool release_cs);

/* Diagnostics: which status encoding was latched, and the timeout counters. */
int  spi_get_status_family(int port);
void spi_get_timeouts(int port, unsigned *tx, unsigned *rx);

#endif /* __SPI_S5L8740_H__ */
