/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Kernel tick for Apple S5L8740 (iPod nano 7G).
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

/*
 * Timer B drives the kernel tick at 10 kHz, matching the s5l8702 targets.
 *
 * Timer E free-runs as the microsecond reference (USEC_TIMER) and is started
 * by system_init() via timer_init() below.
 *
 * NOTE: the s5l8702 code pokes CLKCON before touching the timer. Do not copy
 * that here. On N31 the equivalent poke was found to make the clock jump
 * [1s -> 4380s] and hang USB, and was removed from clk-s5l8702.c for exactly
 * that reason.
 */

/* External clock is 24 MHz on N31 (DT: nclk). */
#define TIMER_ECLK      24000000

void INT_TIMER(void)
{
    /* clear interrupt -- write-1-to-clear via read/write back */
    TBCON = TBCON;

    call_tick_tasks();
}

void usec_timer_init(void)
{
    /*
     * Timer E: free-running 1 MHz microsecond counter. 24 MHz / 4 / 6 = 1 MHz.
     */
    TECMD = (1 << 1);       /* TE_CLR */
    TEPRE = 6 - 1;
    TECON = (1 << 6) |      /* select ECLK */
            (1 << 8);       /* TE_CS = ECLK / 4 */
    TEDATA0 = 0xffffffff;
    TECMD = (1 << 0);       /* TE_EN */
}

void tick_start(unsigned int interval_in_ms)
{
    int cycles = 10 * interval_in_ms;

    /* 10 kHz: 24 MHz / 16 / 150 */
    TBCMD = (1 << 1);   /* TB_CLR */
    TBPRE = 150 - 1;    /* prescaler */
    TBCON = (0 << 13) | /* TB_INT1_EN */
            (1 << 12) | /* TB_INT0_EN */
            (0 << 11) | /* TB_START */
            (2 << 8) |  /* TB_CS = ECLK / 16 */
            (1 << 6) |  /* select ECLK (24 MHz) */
            (0 << 4);   /* TB_MODE_SEL = interval mode */
    TBDATA0 = cycles;   /* interval period */
    TBCMD = (1 << 0);   /* TB_EN */

    VIC0INTENABLE = 1 << IRQ_TIMER;
}
