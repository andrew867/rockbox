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
 * The N31 has five physical keys and no clickwheel. They come from two
 * different controllers:
 *
 *   Vol+ / Vol-      SoC GPIO 40 / 41, read straight out of PDAT
 *   Home/Sleep/Play  Dialog D1830 PMIC over I2C1, packed into regs 7 and 8
 *
 * Navigation proper is the capacitive touchscreen, which is Phase 6.
 */
#define BUTTON_VOL_UP       0x00000001
#define BUTTON_VOL_DOWN     0x00000002
#define BUTTON_HOME         0x00000004
#define BUTTON_POWER        0x00000008
#define BUTTON_PLAY         0x00000010

#define BUTTON_MAIN         (BUTTON_VOL_UP|BUTTON_VOL_DOWN|BUTTON_HOME \
                            |BUTTON_POWER|BUTTON_PLAY)

/* Software power-off is a long press on the Sleep key. */
#define POWEROFF_BUTTON     BUTTON_POWER
#define POWEROFF_COUNT      30

#endif /* _BUTTON_TARGET_H_ */
