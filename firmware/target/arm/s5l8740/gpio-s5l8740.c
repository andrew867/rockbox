/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * GPIO and external interrupt controller for Apple S5L8740 (iPod nano 7G).
 *
 * Ported from tools/linux-n31/drivers/gpio-s5l8740.c and
 * tools/linux-n31/drivers/irq-s5l8740-eic.c.
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
#include "gpio-s5l8740.h"

/*
 * The S5L8740 GPIO block is NOT the s5l8702 "group" model. Pads are arranged
 * in banks of eight with a 32-byte stride, and a pad is reconfigured by
 * writing a command word to a single latch register at +0x1e0 rather than by
 * poking the bank register directly:
 *
 *     GPIO_CMD = (bank << 16) | (pin << 8) | cmd
 *
 * This mirrors RetailOS sub_43D38C(gpio, mode, val):
 *
 *   mode == 1       output; cmd is 14 (low) or 15 (high). DIR is deliberately
 *                   NOT touched -- the stock code leaves it alone on this
 *                   path and so do we.
 *   mode == 0xFFFE  release; clear the DIR bit and issue cmd 0.
 *   otherwise       select function nibble `mode` and set the DIR bit.
 *
 * Pad 200 (0xC8) is a no-op in sub_43D38C and is skipped here for the same
 * reason.
 */
#define CMD_OUT_LOW     14
#define CMD_OUT_HIGH    15
#define MODE_RELEASE    0xFFFE

/* EIC, group g = 0..6, group = pad >> 5, bit = pad & 31. */
#define EIC_NGROUPS         7
#define EIC_INTLEVEL(g)     (*(REG32_PTR_T)(EIC_BASE + 0x80 + 4 * (g)))
#define EIC_INTSTAT(g)      (*(REG32_PTR_T)(EIC_BASE + 0xa0 + 4 * (g)))
#define EIC_INTEN(g)        (*(REG32_PTR_T)(EIC_BASE + 0xc0 + 4 * (g)))
#define EIC_INTTYPE(g)      (*(REG32_PTR_T)(EIC_BASE + 0xe0 + 4 * (g)))

static inline int pad_bank(int pad) { return pad >> 3; }
static inline int pad_bit(int pad)  { return pad & 7; }

static void gpio_cmd(int pad, unsigned mode, bool level)
{
    int bank = pad_bank(pad);
    int pin = pad_bit(pad);
    uint8_t cmd;

    if (pad == 200)
        return;

    if (mode == 1) {
        cmd = level ? CMD_OUT_HIGH : CMD_OUT_LOW;
    } else if (mode == MODE_RELEASE) {
        GPIO_PDIR(bank) &= ~(1 << pin);
        cmd = 0;
    } else {
        cmd = (uint8_t)mode;
        GPIO_PDIR(bank) |= (1 << pin);
    }

    GPIO_CMD = (bank << 16) | (pin << 8) | cmd;
}

void gpio_set_function(int pad, int func)
{
    gpio_cmd(pad, func, false);
}

void gpio_direction_input(int pad)
{
    /*
     * CAUTION: on the volume pads (40/41) this writes 0xFFFE and leaves them
     * unreadable. Those two are read straight out of PDAT by
     * button-nano7g.c and must never be put through this path -- the Linux
     * driver hit exactly this and it is why gpio-keys-polled is disabled in
     * the N31 device tree.
     */
    gpio_cmd(pad, MODE_RELEASE, false);
}

void gpio_direction_output(int pad, bool level)
{
    gpio_cmd(pad, 1, level);
}

bool gpio_get(int pad)
{
    return !!(GPIO_PDAT(pad_bank(pad)) & (1 << pad_bit(pad)));
}

void gpio_set(int pad, bool level)
{
    gpio_cmd(pad, 1, level);
}

void gpio_init(void)
{
    /*
     * Nothing to do. U-Boot has already replayed the SEC pinmux table
     * (sub_223C) by the time it hands over, which is the same reason the
     * Linux device tree carries apple,skip-sec-pinmux. Re-applying it here
     * would re-mux pads that later drivers have already claimed.
     *
     * If this port ever boots without U-Boot, the pinmux replay from
     * tools/linux-n31/drivers/gpio-s5l8740.c:s5l8740_pinmux_223C() plus
     * tools/linux-n31/drivers/pinmux_table.inc has to be brought over here.
     */
}

void eic_init(void)
{
    int g;

    /* Mask everything, then clear any latched status. SEC sub_223C. */
    for (g = 0; g < EIC_NGROUPS; g++) {
        EIC_INTEN(g) = 0;
        EIC_INTSTAT(g) = 0xffffffff;
        EIC_INTLEVEL(g) = 0;
        EIC_INTTYPE(g) = 0;
    }
}

void eic_enable_pad(int pad, bool enable)
{
    int g = pad >> 5;
    uint32_t bit = 1u << (pad & 31);

    if (g >= EIC_NGROUPS)
        return;

    /* Stock programs level-triggered, active low for every pad we use. */
    EIC_INTTYPE(g) &= ~bit;     /* 0 = level */
    EIC_INTLEVEL(g) |= bit;     /* 1 = active low */
    EIC_INTSTAT(g) = bit;       /* clear stale latch before unmasking */

    if (enable)
        EIC_INTEN(g) |= bit;
    else
        EIC_INTEN(g) &= ~bit;
}

void eic_ack_pad(int pad)
{
    int g = pad >> 5;

    if (g >= EIC_NGROUPS)
        return;

    EIC_INTSTAT(g) = 1u << (pad & 31);
}
