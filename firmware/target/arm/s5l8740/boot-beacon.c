/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Early boot beacon for the iPod nano 7G -- paint the panel before anything
 * that can block.
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
#include "boot-beacon.h"

/*
 * There is no serial console on this device in practice -- no DCSD cable, no
 * access to SEC UART3 -- so the panel is the only channel a boot has to say
 * anything at all. That makes the ORDER of Rockbox's own init actively
 * hostile to debugging: system_init() and kernel_init() both run before
 * lcd_init(), so anything that hangs in either leaves U-Boot's logo on the
 * screen and is indistinguishable from "the image never ran".
 *
 * This module exists to break that tie. It brings the LCDIF up far enough to
 * fill the screen, and does so with NO dependency on anything that could be
 * the thing that is broken:
 *
 *   - no interrupts        the VIC may not be wired correctly yet
 *   - no udelay()          USEC_TIMER is Timer E, whose offsets are ASSUMED
 *                          equal to the s5l8720 layout rather than confirmed;
 *                          if it does not count, every udelay() is an
 *                          infinite loop and so is every timeout built on it
 *   - no kernel, no tick   sleep() and yield() need a tick that may not fire
 *   - no allocations       nothing is initialised yet
 *
 * Every wait here is a bare bounded counter, which cannot hang no matter what
 * the timer is doing.
 *
 * Call beacon_stage() at each point of interest and the last colour on screen
 * says how far the boot got. That turns "nothing happened" into a specific
 * line number.
 */

#define N31_LCD_CON         (*(REG32_PTR_T)(LCDIF_BASE + 0x00))
#define N31_LCD_STATUS      (*(REG32_PTR_T)(LCDIF_BASE + 0x1c))
#define N31_LCD_PHTIME      (*(REG32_PTR_T)(LCDIF_BASE + 0x20))
#define N31_LCD_UNK2C       (*(REG32_PTR_T)(LCDIF_BASE + 0x2c))
#define N31_LCD_RESET       (*(REG32_PTR_T)(LCDIF_BASE + 0x30))
#define N31_LCD_WDATA       (*(REG32_PTR_T)(LCDIF_BASE + 0x40))
#define N31_LCD_UNK68       (*(REG32_PTR_T)(LCDIF_BASE + 0x68))
#define N31_LCD_UNK70       (*(REG32_PTR_T)(LCDIF_BASE + 0x70))
#define N31_LCD_SIZE        (*(REG32_PTR_T)(LCDIF_BASE + 0x74))
#define N31_LCD_UNK78       (*(REG32_PTR_T)(LCDIF_BASE + 0x78))
#define N31_LCD_UNK7C       (*(REG32_PTR_T)(LCDIF_BASE + 0x7c))
#define N31_LCD_UNK84       (*(REG32_PTR_T)(LCDIF_BASE + 0x84))
#define N31_LCD_UNKA4       (*(REG32_PTR_T)(LCDIF_BASE + 0xa4))

#define LCD_STATUS_BUSY     0x10
#define CON_HOLD            (1 << 10)
#define CON_RUN             (1 << 11)
#define CON_BASE            0x00100ab0
#define CON_MODE_MASK       0xc0000000
#define CON_FMT_MASK        0x00000007

/* Bare spin counts. Generous, but finite by construction. */
#define SPIN_SHORT          200000u
#define SPIN_PIXEL          20000u

static bool beacon_ready;

static void beacon_spin(unsigned n)
{
    while (n--)
        asm volatile ("" ::: "memory");
}

/*
 * Minimal LCDIF bring-up.
 *
 * Deliberately does NOT do the full reset dance from lcd-nano7g.c: that path
 * waits on status bits with real timeouts and pokes CLKCON, and any of it
 * could be the thing that is wrong. U-Boot has already left the interface
 * running and the panel lit, so the least-risk action is to reprogram
 * geometry and start pushing pixels.
 */
static void beacon_init(void)
{
    uint32_t con = N31_LCD_CON;
    uint32_t keep = con & (CON_MODE_MASK | CON_FMT_MASK);

    N31_LCD_UNK78 = 0x000a000a;
    N31_LCD_CON = keep | CON_BASE;
    N31_LCD_UNK2C = 1;
    N31_LCD_UNK68 = 0;
    N31_LCD_UNK70 = 0;
    N31_LCD_SIZE = LCD_HEIGHT | (LCD_WIDTH << 16);
    N31_LCD_PHTIME = 0;
    N31_LCD_UNK7C = 770;
    N31_LCD_UNK84 = 100;
    N31_LCD_UNKA4 = 1;

    N31_LCD_CON = (N31_LCD_CON & ~CON_HOLD) | CON_RUN;

    beacon_ready = true;
}

void beacon_fill(uint16_t colour)
{
    unsigned n = LCD_WIDTH * LCD_HEIGHT;

    if (!beacon_ready)
        beacon_init();

    while (n--) {
        unsigned spin = SPIN_PIXEL;

        /*
         * Bounded wait. If the interface never reports idle we push anyway
         * rather than hang -- a torn or partial screen still tells us the
         * code got here, which is the entire point.
         */
        while ((N31_LCD_STATUS & LCD_STATUS_BUSY) && spin)
            spin--;

        N31_LCD_WDATA = colour;
    }
}

/*
 * Paint a stage colour and leave it up long enough to be seen even if the
 * next stage is instant.
 */
void beacon_stage(uint16_t colour)
{
    beacon_fill(colour);
    beacon_spin(SPIN_SHORT);
}

/*
 * Split the screen into bands so several facts fit on one panel: the top band
 * is the stage reached, the bottom band a pass/fail flag for something the
 * caller cares about.
 */
void beacon_split(uint16_t top, uint16_t bottom)
{
    unsigned n = LCD_WIDTH * LCD_HEIGHT;
    unsigned half = n / 2;
    unsigned i;

    if (!beacon_ready)
        beacon_init();

    for (i = 0; i < n; i++) {
        unsigned spin = SPIN_PIXEL;

        while ((N31_LCD_STATUS & LCD_STATUS_BUSY) && spin)
            spin--;

        N31_LCD_WDATA = (i < half) ? top : bottom;
    }

    beacon_spin(SPIN_SHORT);
}
