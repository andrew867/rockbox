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
#include "kernel.h"
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

#define N31_LCD_STATUS      (*(REG32_PTR_T)(LCDIF_BASE + 0x1c))
#define N31_LCD_WDATA       (*(REG32_PTR_T)(LCDIF_BASE + 0x40))

#define LCD_STATUS_BUSY     0x10

/* Bare spin counts. Generous, but finite by construction. */
#define SPIN_SHORT          200000u
#define SPIN_PIXEL          20000u

static void beacon_spin(unsigned n)
{
    while (n--)
        asm volatile ("" ::: "memory");
}

/*
 * No LCDIF bring-up here. That is the design, not an omission.
 *
 * This module used to reprogram the interface first -- CON, SIZE, PHTIME and
 * a handful of undocumented registers -- on the assumption that something had
 * to be set up before pixels would land. The assembly beacons in crt0 then
 * proved otherwise: they paint the panel with nothing but two MMIO stores,
 * before the C runtime exists at all. U-Boot hands over with the interface
 * already running and the panel lit.
 *
 * Which turns that setup from harmless into actively harmful. Every register
 * it wrote was a chance to stop a working display, and if it did, the failure
 * mode is the cruellest one available: the screen freezes on the last colour
 * the assembly beacons painted, so the diagnostic becomes indistinguishable
 * from the bug it was added to find. A debugging aid must not be able to break
 * the thing it is reporting on.
 *
 * So: poll, store, nothing else -- the same two registers the assembly path
 * uses, for the same reason. They are the only two that are proven.
 */
void beacon_fill(uint32_t colour)
{
    unsigned n = LCD_WIDTH * LCD_HEIGHT;

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
void beacon_stage(uint32_t colour)
{
    beacon_fill(colour);
    beacon_spin(SPIN_SHORT);
}

/*
 * Split the screen into bands so several facts fit on one panel: the top band
 * is the stage reached, the bottom band a pass/fail flag for something the
 * caller cares about.
 */
void beacon_split(uint32_t top, uint32_t bottom)
{
    unsigned n = LCD_WIDTH * LCD_HEIGHT;
    unsigned half = n / 2;
    unsigned i;

    for (i = 0; i < n; i++) {
        unsigned spin = SPIN_PIXEL;

        while ((N31_LCD_STATUS & LCD_STATUS_BUSY) && spin)
            spin--;

        N31_LCD_WDATA = (i < half) ? top : bottom;
    }

    beacon_spin(SPIN_SHORT);
}

/*
 * USEC_TIMER liveness. See the comment on the declaration for why this is
 * worth a dedicated stage rather than being inferred from a hang.
 */
static uint32_t beacon_verdict = BEACON_BLACK;

void beacon_probe_usec(void)
{
    unsigned t0 = USEC_TIMER;

    beacon_spin(SPIN_SHORT);

    beacon_verdict = (USEC_TIMER != t0) ? BEACON_GREEN : BEACON_RED;
    beacon_split(BEACON_WHITE, beacon_verdict);
}

void beacon_mark(uint32_t top)
{
    beacon_split(top, beacon_verdict);
}

/*
 * Does the tick actually advance?
 *
 * sleep() blocks until current_tick moves, so if the tick timer is not firing
 * sleep() never returns -- and any diagnostic that calls it stops the boot at
 * the exact point it prints. That failure is indistinguishable from the code
 * it was measuring having hung, which makes it the worst possible bug to have
 * in an instrument.
 *
 * Uses a bare spin, not sleep(), for the same reason beacon_probe_usec() does:
 * using the thing under test to test itself hangs on precisely the case being
 * looked for.
 *
 * WHITE over GREEN if the tick moved, WHITE over RED if it did not.
 */
void beacon_probe_tick(void)
{
    long t0 = current_tick;

    beacon_spin(SPIN_SHORT * 16);

    if (current_tick != t0) {
        beacon_split(BEACON_WHITE, BEACON_GREEN);
        return;
    }

    /*
     * Dead. Walk the chain and report each link as its own colour, because
     * "the tick does not fire" has four independent causes and they need
     * completely different fixes:
     *
     *   RED    does Timer B count at all?          TBCNT moves
     *   YELLOW does it raise its interrupt line?   VIC0RAWINTR bit 7
     *   CYAN   is that line unmasked in the VIC?   VIC0INTENABLE bit 7
     *   WHITE  are IRQs enabled at the CPU?        CPSR I bit clear
     *
     * Green underneath means that link is good, red means it is where the
     * chain breaks. The first red band is the answer.
     *
     * Everything here is a bare read or a bounded spin -- nothing waits on
     * the thing being measured.
     */
    {
        uint32_t c0 = TBCNT;
        uint32_t cpsr;
        bool counts, raw = false, unmasked, irqs_on;
        unsigned i;

        beacon_spin(SPIN_SHORT * 4);
        counts = (TBCNT != c0);

        for (i = 0; i < 2000; i++) {
            if (VIC0RAWINTR & (1u << IRQ_TIMER)) {
                raw = true;
                break;
            }
            beacon_spin(256);
        }

        unmasked = (VIC0INTENABLE & (1u << IRQ_TIMER)) != 0;

        asm volatile ("mrs %0, cpsr" : "=r"(cpsr));
        irqs_on = !(cpsr & (1u << 7));      /* I bit clear = IRQs enabled */

        for (;;) {
            beacon_split(BEACON_RED,    counts   ? BEACON_GREEN : BEACON_RED);
            beacon_spin(SPIN_SHORT * 6);
            beacon_split(BEACON_YELLOW, raw      ? BEACON_GREEN : BEACON_RED);
            beacon_spin(SPIN_SHORT * 6);
            beacon_split(BEACON_CYAN,   unmasked ? BEACON_GREEN : BEACON_RED);
            beacon_spin(SPIN_SHORT * 6);
            beacon_split(BEACON_WHITE,  irqs_on  ? BEACON_GREEN : BEACON_RED);
            beacon_spin(SPIN_SHORT * 6);
        }
    }

}
