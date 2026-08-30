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
 * Real dB, from sub_D2C98.
 *
 * The first version of this driver did volume in software with an invented
 * range, on the grounds that the RE established 0x0527 = 0x60 for playback
 * and 0xFF for silence but not the steps in between. The whole-image
 * decompilation settles it: 0x0227 carries a signed dB code, 1 dB per step
 * above -50 dB and 2 dB per step below it, and sub_400330 gates the 2.5 V
 * analog rail on that value crossing -8 dB.
 *
 * So the range is not a guess any more, and it matters more than a volume
 * control usually does -- the rail transition is inside it.
 */
#define CS42L81_VOL_DB_MIN      (-76)
#define CS42L81_VOL_DB_MAX      12
#define CS42L81_VOL_DB_KNEE     (-50)
#define CS42L81_VOL_DB_DEFAULT  (-20)

AUDIOHW_SETTING(VOLUME, "dB", 0, 1,
                CS42L81_VOL_DB_MIN, CS42L81_VOL_DB_MAX,
                CS42L81_VOL_DB_DEFAULT)

/*
 * Play lifecycle, called by pcm-s5l8740.c.
 *
 * The order is not interchangeable and neither half works alone:
 *
 *   cs42l81_play_prepare(rate)   configure: rails, analog power-up, unfreeze,
 *                                rate, output path, graph
 *   cs42l81_pre_iis_start()      immediately BEFORE the IIS clocks start
 *   cs42l81_play_start()         arm the transport
 *   cs42l81_post_iis_start()     AFTER the clocks are running -- this is
 *                                where the unmute happens
 *   cs42l81_play_stop()          transport stop
 *
 * Unmuting before the interface runs is one of the ways to end up with a
 * codec whose every register reads back correctly and which makes no sound,
 * so post_iis is not an optional refinement.
 */
void cs42l81_play_prepare(unsigned rate);
void cs42l81_play_start(void);
void cs42l81_play_stop(void);
void cs42l81_pre_iis_start(void);
void cs42l81_post_iis_start(void);

/* Debug-menu telemetry. Reads live; changes nothing. */
struct cs42l81_route {
    uint8_t  r0401;
    uint8_t  r0403;
    uint8_t  r0404;
    uint8_t  r0500;
    uint8_t  r0527;         /* 0x60 unmuted, 0xFF muted */
    uint8_t  r054f;
    uint8_t  r0075;
    uint8_t  r0220;         /* bits 5,3: the idle/standby hold pair */
    uint8_t  r002f;         /* bit 7 analog ready; bit 6 is NOT a lock bit */
    uint8_t  r0227;         /* output gain, signed dB code */
    uint8_t  r0219;         /* 2v5 rail select, low three bits */
    uint8_t  rc96f;         /* 2v5 backpower latch: 0x1E up, 0x0E down */
    uint8_t  status_528;
    uint8_t  mode38;        /* shadow of MEMORY[0x892A038] */
    unsigned rate;
    bool     playing;
};

void cs42l81_get_route(struct cs42l81_route *r);

#endif /* _CS42L81_H */
