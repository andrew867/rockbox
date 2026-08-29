/***************************************************************************
 * Audio routing for the iPod nano 7G (N31) -- Phase 4 placeholder.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#include "config.h"
#include "audio.h"

/*
 * The playback chain is IIS0 @0x3CA00000 -> PL080 DMAC0 peri 10 -> CS42L81 on
 * SPI0 -> headphone jack.
 *
 * This comment used to add "with the D1830 sibling LDOs (regs 21-23 bit 4)
 * powering the analog side". That is wrong and acting on it locked the
 * Linux kernel on boot: the OSOS rail setter writes registers 20-23 as a
 * bare (code & 0x1F) at a 25 mV step, so bit 4 is the top bit of a voltage
 * field, not an enable. Setting it moved three rails by 400 mV each. What
 * gates the CS42L81 analog supply is still unknown.
 *
 * The digital transport is proven in Linux; the analog side is not -- a 1 kHz
 * tone measures about -66 dBFS at the jack against RetailOS's -16 dBFS, with
 * the PMIC's LDO registers reading 0x00 during playback. Phase 4 is blocked on
 * that being solved in the Linux tree first, since the fix is shared.
 */

void audio_input_mux(int source, unsigned flags)
{
    (void)source;
    (void)flags;
}

/*
 * Output routing. There is only one analog output on this device -- the
 * headphone jack -- so there is nothing to switch between; USB audio and
 * normal playback both end up at the same codec.
 *
 * TODO: Lightning audio out would be a second sink, but that needs the
 * Tristar mux map, which is still open RE.
 */
void audio_set_output_source(int source)
{
    (void)source;
}
