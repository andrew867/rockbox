/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Buttons for the iPod nano 7G (N31).
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
#include "config.h"
#include "system.h"
#include "button.h"
#include "gpio-s5l8740.h"
#include "pmu-target.h"

/*
 * Volume keys are SoC pads 40 and 41, active low.
 *
 * They are read directly out of PDAT and are NEVER put through
 * gpio_direction_input(): that path issues mode 0xFFFE, which on these two
 * pads leaves them unreadable. This is the same reason gpio-keys-polled is
 * disabled in the N31 device tree -- Linux hit it first.
 *
 * Home, Sleep and Play come from the D1830 PMIC over I2C1, packed into
 * registers 7 and 8 (OSOS sub_26520):
 *
 *   Home  = reg 7 bit 4   (OSOS id 1)
 *   Sleep = reg 7 bit 5   (OSOS id 9)
 *   Play  = reg 8 bit 1   (OSOS id 6, side key between the volume keys)
 *
 * All active low: sub_4195D8(id, bit == 0).
 */

void button_init_device(void)
{
    /*
     * Nothing to configure. The volume pads are already inputs after the SEC
     * pinmux that U-Boot replays, and the PMIC keys are polled over I2C by
     * pmu_read_buttons().
     */
}

int button_read_device(void)
{
    int btn = 0;

    if (!gpio_get(GPIO_PAD_VOLUP))
        btn |= BUTTON_VOL_UP;
    if (!gpio_get(GPIO_PAD_VOLDOWN))
        btn |= BUTTON_VOL_DOWN;

    btn |= pmu_read_buttons();

    return btn;
}
