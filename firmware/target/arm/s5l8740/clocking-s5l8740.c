/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Clock gates for Apple S5L8740 (iPod nano 7G).
 *
 * Ported from tools/linux-n31/drivers/clk-s5l8702.c.
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
#include "clocking-s5l8740.h"

/*
 * Bring-up policy, carried over verbatim from the Linux driver: ungate
 * everything and leave it ungated.
 *
 * The N31 clock tree is only partly RE'd -- the PWRCON gate banks are known,
 * the PLL and divider fields are not -- so selective gating would be guessing.
 * Every peripheral this port touches is on one of these four banks, and the
 * power cost of leaving them running is not what is limiting this device
 * today.
 *
 * Three hard rules, each of which came from something breaking:
 *
 *   - Never remux the SYS PLL (+0x00 / +0x04). That kills live DRAM.
 *   - Never write CLKCON+0x50. That is the fatal/WDT latch (magic 0xA5).
 *   - Never poke the TIMER MMIO block from here. The equivalent poke in the
 *     Linux driver made the clock jump [1s -> 4380s] and hung USB, and was
 *     removed for exactly that reason.
 */

#define CLKCON(off)     (*(REG32_PTR_T)(CLKCON_BASE + (off)))

void clockgate_enable(int reg, uint32_t bits, bool enable)
{
    int oldstatus = disable_irq_save();

    /* PWRCON is set-to-disable: clearing a bit starts that clock. */
    if (enable)
        CLKCON(reg) &= ~bits;
    else
        CLKCON(reg) |= bits;

    restore_irq(oldstatus);
}

void clocking_init(void)
{
    /* Every AHB/APB gate on, whether or not a driver has claimed it. */
    CLKCON(CLKCON_PWRCON0) = 0;
    CLKCON(CLKCON_PWRCON1) = 0;
    CLKCON(CLKCON_PWRCON2) = 0;
    CLKCON(CLKCON_PWRCON4) = 0;

    /*
     * The CG16 enable bits (RetailOS sub_41CBD8) live in the high halves of
     * the divider registers. Clear only those bits -- the low halves are the
     * divider fields, which U-Boot has already programmed and which we have
     * no RE basis to recompute.
     */
    CLKCON(CLKCON_CG16_08) &= ~0x80008000u;
    CLKCON(CLKCON_CG16_0C) &= ~0x80008000u;
    CLKCON(CLKCON_CG16_10) &= ~0x00008000u;
    CLKCON(CLKCON_CG16_14) &= ~0x80008000u;
}
