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
#ifndef _I2C_S5L8740_H
#define _I2C_S5L8740_H

#include "config.h"

/* Bus numbers, matching the SoC's IIC0/IIC1. */
#define I2C_BUS_IIC0    0
#define I2C_BUS_IIC1    1

void i2c_init(void) INIT_ATTR;
void i2c_preinit(int bus);

int i2c_write(int bus, unsigned char slave, int address, int len, const unsigned char *data);
int i2c_read(int bus, unsigned char slave, int address, int len, unsigned char *data);

/* Unlocked variants, for callers that already hold the bus. */
int i2c_wr(int bus, unsigned char slave, int address, int len, const unsigned char *data);
int i2c_rd(int bus, unsigned char slave, int address, int len, unsigned char *data);

#endif /* _I2C_S5L8740_H */
