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
#ifndef __CLOCKING_S5L8740_H__
#define __CLOCKING_S5L8740_H__

#include <stdbool.h>
#include <stdint.h>

/*
 * CLKCON at 0x3C500000.
 *
 * The N31 clock tree is only partly reverse-engineered: the PWRCON gate banks
 * are known, the PLL and divider fields are not. So this is a gate driver, not
 * a rate driver, and there is no frequency scaling.
 */
#define CLKCON_SYS0     0x00
#define CLKCON_SYS1     0x04
#define CLKCON_CG16_08  0x08
#define CLKCON_CG16_0C  0x0c
#define CLKCON_CG16_10  0x10
#define CLKCON_CG16_14  0x14
#define CLKCON_PWRCON0  0x48
#define CLKCON_PWRCON1  0x4c
#define CLKCON_PWRCON2  0x58
#define CLKCON_PWRCON4  0x6c

/* PWRCON1 bits used by the SPI driver. */
#define PWRCON1_SPI0    (1 << 2)
#define PWRCON1_SPI2    (1 << 16)
#define PWRCON4_SPI0_2  (1 << 13)

void clocking_init(void);

/*
 * Clear (enable) or set (disable) a gate bit. PWRCON is set-to-disable:
 * a clear bit means the clock is running.
 */
void clockgate_enable(int reg, uint32_t bits, bool enable);

#endif /* __CLOCKING_S5L8740_H__ */
