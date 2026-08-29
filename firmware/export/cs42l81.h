/***************************************************************************
 * Cirrus CS42L81 / Apple 338S1146 codec for the iPod nano 7G (N31).
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#ifndef _CS42L81_H
#define _CS42L81_H

#include <stdbool.h>
#include <stdint.h>
#include "audiohw.h"

/*
 * Volume is done in software: the RE corpus establishes that 0x0527 = 0x60 is
 * the stock playback level and 0xFF is silence, but not the step size or
 * usable range in between. Inventing a dB mapping would be a guess in the one
 * place a guess is audible.
 */
AUDIOHW_SETTING(VOLUME, "dB", 0, 1, -74, 0, 0)

/*
 * Bring the codec's serial port into lock.
 *
 * MUST be called only after IIS BCLK/LRCLK are running -- the codec will not
 * lock against a dead clock. This is the step the RE guide identifies as the
 * most likely cause of the Linux silence, so it is a separate entry point
 * rather than part of init.
 */
void cs42l81_asp_lock(void);
bool cs42l81_asp_is_locked(void);

struct cs42l81_route {
    uint8_t r0401;
    uint8_t r0403;
    uint8_t r0404;
    uint8_t r0500;
    uint8_t r0527;
    uint8_t r054f;
    uint8_t r0075;
    uint8_t r0220;
    uint8_t r002f;         /* bit 6: ASP locked, read live */
    uint8_t headset_type;  /* 0x000B & 3, telemetry only */
    bool    locked;
};

void cs42l81_get_route(struct cs42l81_route *r);

#endif /* _CS42L81_H */
