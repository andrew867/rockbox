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
 * Stage colours, as XRGB8888.
 *
 * The panel takes one 32-bit XRGB8888 word per pixel -- NOT the RGB565 this
 * port uses internally. That was found the hard way: the first beacons wrote
 * RGB565 constants and came out as entirely the wrong colours, because the
 * hardware read 0x0000ffff (RGB565 white) as R=00 G=ff B=ff and painted cyan.
 * Every colour here is therefore written out in full rather than reusing the
 * LCD_RGBPACK values.
 *
 * Every stage has its OWN colour. An earlier ladder reused RED for both the
 * end of crt0 and the start of system_init, which made those two states
 * indistinguishable at exactly the point the boot was stopping -- a beacon
 * that cannot tell two stages apart is not a beacon.
 *
 *   U-Boot logo  the bootloader never branched to us at all
 *   WHITE        crt0 entered, supervisor mode set          (asm, no C runtime)
 *   RED          memory_init() returned, MMU up             (asm)
 *   ORANGE       stacks built, about to branch to main()    (asm)
 *   YELLOW       system_init() entered -- first C code to paint
 *   GREEN        clocks and the microsecond timer programmed
 *   CYAN         system_init() finished: GPIO, EIC, DMA, SPI all returned
 *   BLUE         kernel_init() returned -- the tick is actually firing
 *   MAGENTA      i2c_init() and power_init() returned
 *   PURPLE       current_tick is ticking; lcd_init() is next
 *
 * Two gaps are worth naming in advance, because they are the ones most likely
 * to happen and are otherwise completely silent:
 *
 *   ORANGE, no YELLOW  C was entered but the first C beacon never painted.
 *                      The assembly beacons prove the LCDIF is running, so
 *                      this points at the C runtime -- bss, stack, literal
 *                      pools -- rather than at the display.
 *   CYAN, no BLUE      The tick timer never fired, so every sleep() in the
 *                      firmware is an infinite wait. Timer E's offsets are
 *                      assumed from the s5l8720 layout rather than confirmed
 *                      on this SoC, so this is a live possibility rather than
 *                      a theoretical one.
 */
#define BEACON_RED      0x00ff0000
#define BEACON_ORANGE   0x00ff8000
#define BEACON_YELLOW   0x00ffff00
#define BEACON_GREEN    0x0000ff00
#define BEACON_CYAN     0x0000ffff
#define BEACON_BLUE     0x000000ff
#define BEACON_MAGENTA  0x00ff00ff
#define BEACON_PURPLE   0x008000ff
#define BEACON_WHITE    0x00ffffff
#define BEACON_BLACK    0x00000000

/* Fill the panel. */
void beacon_fill(uint32_t colour);

/* Fill and hold long enough to be seen. */
void beacon_stage(uint32_t colour);

/* Top half / bottom half, for showing two facts at once. */
void beacon_split(uint32_t top, uint32_t bottom);

#endif /* __BOOT_BEACON_H__ */
