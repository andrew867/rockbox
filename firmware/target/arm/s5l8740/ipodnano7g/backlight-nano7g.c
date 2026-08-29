/***************************************************************************
 * Backlight for the iPod nano 7G (N31).
 *
 * Ported from tools/linux-n31/drivers/backlight-s5l8740.c.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#include "config.h"
#include "system.h"
#include "backlight-target.h"
#include "backlight.h"

/*
 * A small MMIO block of its own at 0x3E000000 -- brightness is a single
 * register at +0x08 taking 0..0x3f.
 *
 * Brightness is NEVER driven through the LCDIF CON/PHTIME registers. That was
 * tried during the Linux bring-up and is wrong: those control the pixel
 * interface timing, not the backlight, and poking them corrupts the display.
 */
#define BL_CTRL     (*(REG32_PTR_T)(BACKLIGHT_BASE + 0x00))
#define BL_LEVEL    (*(REG32_PTR_T)(BACKLIGHT_BASE + 0x08))

static int backlight_level = DEFAULT_BRIGHTNESS_SETTING;

bool backlight_hw_init(void)
{
    /* U-Boot already wrote 62 here; take ownership at our own setting. */
    BL_LEVEL = backlight_level;
    return true;
}

void backlight_hw_brightness(int brightness)
{
    if (brightness < MIN_BRIGHTNESS_SETTING)
        brightness = MIN_BRIGHTNESS_SETTING;
    if (brightness > MAX_BRIGHTNESS_SETTING)
        brightness = MAX_BRIGHTNESS_SETTING;

    backlight_level = brightness;
    BL_LEVEL = brightness;
}

void backlight_hw_on(void)
{
    BL_LEVEL = backlight_level;
}

void backlight_hw_off(void)
{
    BL_LEVEL = 0;
}
