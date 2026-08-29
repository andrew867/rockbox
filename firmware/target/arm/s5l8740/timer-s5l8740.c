/***************************************************************************
 * User timer for Apple S5L8740 (iPod nano 7G).
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#include "config.h"
#include "system.h"
#include "timer.h"

/*
 * Timer F backs the general-purpose timer API, leaving Timer B for the kernel
 * tick and Timer E as the free-running microsecond reference.
 *
 * ECLK is 24 MHz on N31 (device tree "nclk"), not the 12 MHz the s5l8702
 * family header assumes -- see the TODO in s5l87xx.h. TIMER_FREQ is defined
 * per-SoC so callers computing cycle counts get the right number.
 */

void INT_TIMERF(void)
{
    /* clear interrupt */
    TFCON = TFCON;

    if (pfn_timer != NULL)
        pfn_timer();
}

bool timer_set(long cycles, bool start)
{
    int tf_en = TFCMD & (1 << 0);   /* save TF_EN status */

    TFCMD = (0 << 0);       /* TF_EN = disable */

    if (start) {
        if (pfn_unregister != NULL) {
            pfn_unregister();
            pfn_unregister = NULL;
        }
    }

    TFCON = (1 << 12) |     /* TF_INT0_EN */
            (4 << 8) |      /* TF_CS = ECLK / 1 */
            (1 << 6) |      /* select ECLK */
            (0 << 4);       /* TF_MODE_SEL = interval mode */
    TFPRE = 0;              /* no prescaler */
    TFDATA0 = cycles;

    /*
     * Writing TF_CLR initialises the timer: clears the counter, latches
     * TF_DATA0/1 into the internal buffers and resets the captured signal
     * state (s5l8700 DS).
     */
    TFCMD = (1 << 1) |      /* TF_CLR */
            (tf_en << 0);   /* restore previous TF_EN */

    return true;
}

bool timer_start(void)
{
    TFCMD = (1 << 0);       /* TF_EN = enable */
    return true;
}

void timer_stop(void)
{
    TFCMD = (0 << 0);       /* TF_EN = disable */
}
