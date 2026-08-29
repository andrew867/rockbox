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
 *
 * Past that point the stages are SPLIT rather than solid -- a two-band screen,
 * stage colour on top and a result flag underneath. That is not decoration.
 * The solid ladder ran out of colours a human can tell apart at speed:
 * MAGENTA (0x00ff00ff) and PURPLE (0x008000ff) differ only in the red channel
 * and were reported from the glass as "magenta or purpleish, I cannot tell",
 * which is a beacon failing at its one job. A split reads as unmistakably
 * two-tone no matter which hues are involved.
 *
 *   WHITE / GREEN or RED   USEC_TIMER liveness -- see beacon_probe_usec()
 *   RED    / BLACK         lcdif_reset(): CLKCON gates dropped
 *   YELLOW / BLACK         lcdif_reset(): reset bit acked
 *   GREEN  / BLACK         lcdif_reset(): hold bit acked
 *   CYAN   / BLACK         lcdif_reset() returned
 *   WHITE  / RED           lcdif_program() returned
 *   WHITE  / GREEN         lcd_init_device() done; the real driver owns the
 *                          panel and the UI is next
 *
 * The reason for that much granularity inside one function: lcdif_reset() has
 * three timeout loops and a udelay(), and every one of them is built on
 * USEC_TIMER. If Timer E does not count, none of those timeouts can ever
 * expire and each loop is infinite. It is the densest concentration of that
 * risk in the whole boot path, so it gets the densest instrumentation.
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
#define BEACON_WHITE    0x00ffffff
#define BEACON_BLACK    0x00000000

/* Fill the panel. */
void beacon_fill(uint32_t colour);

/* Fill and hold long enough to be seen. */
void beacon_stage(uint32_t colour);

/* Top half / bottom half, for showing two facts at once. */
void beacon_split(uint32_t top, uint32_t bottom);

/*
 * Prove the microsecond timer actually counts.
 *
 * Timer E's register offsets on this SoC are assumed to match the s5l8720
 * layout rather than confirmed, and USEC_TIMER is the foundation under
 * udelay() and every driver timeout in the port. If it is stuck, those do not
 * fail -- they hang, silently and forever, which is indistinguishable from a
 * dozen other faults.
 *
 * Paints WHITE over GREEN if the timer moved, WHITE over RED if it did not.
 * Uses a bare spin rather than udelay() to measure the delay, because using
 * the thing under test to test itself would hang on exactly the case being
 * looked for.
 */
void beacon_probe_usec(void);

#endif /* __BOOT_BEACON_H__ */
