/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
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
#ifndef __GPIO_S5L8740_H__
#define __GPIO_S5L8740_H__

#include <stdint.h>
#include <stdbool.h>

/* Pad function nibbles as RetailOS uses them (sub_43D38C's mode argument). */
#define GPIO_FUNC_IN        0
#define GPIO_FUNC_OUT       1
#define GPIO_FUNC_ALT2      2
#define GPIO_FUNC_ALT3      3

void gpio_init(void);

/* Select the function nibble for a pad. */
void gpio_set_function(int pad, int func);

/* Configure as input/output and read/write the level. */
void gpio_direction_input(int pad);
void gpio_direction_output(int pad, bool level);
bool gpio_get(int pad);
void gpio_set(int pad, bool level);

/*
 * EIC: routes GPIO groups to VIC lines.
 *
 *   group 1 (pads 32-63)  -> VIC1 line 0
 *   group 2 (pads 64-95)  -> VIC0 line 31
 *
 * These were measured on the device by toggling each group's INTEN and
 * watching which VIC RAWINTR bit followed. An earlier guess put both on VIC0
 * lines 1 and 3; neither line exists for the EIC, so no GPIO interrupt was
 * ever delivered and buttons and the touch nIRQ latched pending forever.
 */
void eic_init(void);
void eic_enable_pad(int pad, bool enable);
void eic_ack_pad(int pad);

#endif /* __GPIO_S5L8740_H__ */
