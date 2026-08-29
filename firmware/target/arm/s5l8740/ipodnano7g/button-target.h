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
#ifndef _BUTTON_TARGET_H_
#define _BUTTON_TARGET_H_

#include <stdbool.h>
#include "config.h"

/*
 * The N31 has five physical keys and a capacitive touchscreen, and the keys
 * come from two different controllers:
 *
 *   Vol+ / Vol-      SoC GPIO 40 / 41, read straight out of PDAT
 *   Home/Sleep/Play  Dialog D1830 PMIC over I2C1, packed into regs 7 and 8
 *
 * Touch is the TI 343S0538 "Nimbus" on SPI2 (touch-nano7g.c).
 */
#define BUTTON_VOL_UP       0x00000001
#define BUTTON_VOL_DOWN     0x00000002
#define BUTTON_HOME         0x00000004
#define BUTTON_POWER        0x00000008
#define BUTTON_PLAY         0x00000010

/*
 * Touchscreen 3x3 grid mode. Rockbox falls back to these when a screen is
 * not in absolute-pointer mode, so all nine have to exist even though the UI
 * is driven by absolute coordinates most of the time.
 */
#define BUTTON_TOPLEFT      0x00000020
#define BUTTON_TOPMIDDLE    0x00000040
#define BUTTON_TOPRIGHT     0x00000080
#define BUTTON_MIDLEFT      0x00000100
#define BUTTON_CENTER       0x00000200
#define BUTTON_MIDRIGHT     0x00000400
#define BUTTON_BOTTOMLEFT   0x00000800
#define BUTTON_BOTTOMMIDDLE 0x00001000
#define BUTTON_BOTTOMRIGHT  0x00002000

#define BUTTON_MAIN (BUTTON_VOL_UP    | BUTTON_VOL_DOWN    | BUTTON_HOME       | \
                     BUTTON_POWER     | BUTTON_PLAY        | BUTTON_TOPLEFT    | \
                     BUTTON_TOPMIDDLE | BUTTON_TOPRIGHT    | BUTTON_MIDLEFT    | \
                     BUTTON_CENTER    | BUTTON_MIDRIGHT    | BUTTON_BOTTOMLEFT | \
                     BUTTON_BOTTOMMIDDLE | BUTTON_BOTTOMRIGHT)

/* Software power-off is a long press on the Sleep key. */
#define POWEROFF_BUTTON     BUTTON_POWER
#define POWEROFF_COUNT      30

#endif /* _BUTTON_TARGET_H_ */
