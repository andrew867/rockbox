/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * ST LIS331DLH accelerometer for the iPod nano 7G (N31).
 *
 * Ported from tools/linux-n31/drivers/lis3lv02d_i2c.c.
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
#include "i2c-s5l8740.h"
#include "accel-nano7g.h"

/*
 * On IIC1 at 7-bit 0x18 (wire 0x30 write / 0x31 read).
 *
 * Reading multiple registers needs the auto-increment bit (0x80) set in the
 * sub-address, which is standard for ST parts but easy to miss -- without it
 * every byte of a burst read comes back from the same register.
 */
#define ACCEL_BUS       I2C_BUS_IIC1
#define ACCEL_ADDR      I2C_ADDR_ACCEL

#define REG_WHO_AM_I    0x0f
#define REG_CTRL1       0x20
#define REG_CTRL2       0x21
#define REG_CTRL3       0x22
#define REG_CTRL4       0x23
#define REG_OUT_X_L     0x28

#define AUTOINC         0x80

/* LIS331DLH identifies as 0x32. */
#define WHO_AM_I_LIS331DLH  0x32

/*
 * CTRL1: normal power mode, 50 Hz output, all three axes enabled.
 * CTRL4: block data update, +/-2g, little endian.
 */
#define CTRL1_ON        0x27
#define CTRL4_BDU       0x80

static bool accel_present = false;

bool accel_init(void)
{
    unsigned char id;

    if (i2c_read(ACCEL_BUS, ACCEL_ADDR, REG_WHO_AM_I, 1, &id) < 0)
        return false;

    /*
     * A failed read used to be mistaken for a chip id: the Linux driver
     * stored the smbus s32 into a u8, so -ETIMEDOUT (-110) surfaced as
     * "unknown sensor type 0x92". Check the transfer result first, and only
     * then the value.
     */
    if (id != WHO_AM_I_LIS331DLH)
        return false;

    {
        unsigned char v;

        v = CTRL1_ON;
        if (i2c_write(ACCEL_BUS, ACCEL_ADDR, REG_CTRL1, 1, &v) < 0)
            return false;

        v = CTRL4_BDU;
        if (i2c_write(ACCEL_BUS, ACCEL_ADDR, REG_CTRL4, 1, &v) < 0)
            return false;
    }

    accel_present = true;
    return true;
}

bool accel_read(struct accel_reading *out)
{
    unsigned char buf[6];

    if (!accel_present)
        return false;

    if (i2c_read(ACCEL_BUS, ACCEL_ADDR, REG_OUT_X_L | AUTOINC,
                 sizeof(buf), buf) < 0)
        return false;

    /* 12-bit left-justified in a 16-bit little-endian pair. */
    out->x = (int16_t)(buf[0] | (buf[1] << 8)) >> 4;
    out->y = (int16_t)(buf[2] | (buf[3] << 8)) >> 4;
    out->z = (int16_t)(buf[4] | (buf[5] << 8)) >> 4;

    return true;
}

bool accel_available(void)
{
    return accel_present;
}
