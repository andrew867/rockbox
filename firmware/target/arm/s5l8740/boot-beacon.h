/***************************************************************************
 * Early boot beacon for the iPod nano 7G.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#ifndef __BOOT_BEACON_H__
#define __BOOT_BEACON_H__

#include <stdint.h>

/*
 * Stage colours. The last colour on the panel is how far the boot got, which
 * on a device with no serial console is the only thing a hang can tell us.
 *
 *   U-Boot logo  never reached C -- crt0, MMU or the branch to main()
 *   RED          system_init() entered: crt0, MMU and DRAM are all good
 *   YELLOW       clocks and the microsecond timer are programmed
 *   GREEN        system_init() finished: GPIO, EIC, DMA and SPI all returned
 *   CYAN         kernel_init() returned -- the tick is actually firing
 *   BLUE         lcd_init() done, the real driver owns the panel
 *   MAGENTA      storage step passed (or was skipped)
 *
 * GREEN with no CYAN is the specific, expected failure worth calling out: it
 * means the tick timer never fired, and every sleep() in the firmware is an
 * infinite wait. Timer E's register offsets are assumed from the s5l8720
 * layout rather than confirmed on this SoC, so that is a live possibility.
 */
#define BEACON_RED      0xf800
#define BEACON_YELLOW   0xffe0
#define BEACON_GREEN    0x07e0
#define BEACON_CYAN     0x07ff
#define BEACON_BLUE     0x001f
#define BEACON_MAGENTA  0xf81f
#define BEACON_WHITE    0xffff
#define BEACON_BLACK    0x0000

/* Fill the panel. Brings the LCDIF up on first use. */
void beacon_fill(uint16_t colour);

/* Fill and hold long enough to be seen. */
void beacon_stage(uint16_t colour);

/* Top half / bottom half, for showing two facts at once. */
void beacon_split(uint16_t top, uint16_t bottom);

#endif /* __BOOT_BEACON_H__ */
