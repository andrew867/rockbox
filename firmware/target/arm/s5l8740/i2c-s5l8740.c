/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * I2C master for Apple S5L8740 (iPod nano 7G).
 *
 * Ported from tools/linux-n31/drivers/i2c-s5l8702.c.
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
#include "i2c-s5l8740.h"

/*
 * Two buses, 0x300000 apart: IIC0 @0x3C600000 and IIC1 @0x3C900000.
 *
 * Everything this port cares about is on IIC1 -- the D1830 PMIC (0x73), the
 * LIS331DLH accelerometer (0x18) and the CBTL1609 Lightning mux (0x1a).
 *
 * Glass notes carried over from the Linux driver, both of which cost real
 * debugging time:
 *
 *  - IICCON bit 4 (IRQPEND) is the byte-done flag. The family VIC lines 21
 *    and 22 never fire on this SoC, so an interrupt-driven transfer waits
 *    forever. Poll IRQPEND instead.
 *
 *  - IIC1's pads (78/79) are already muxed by the bootloader and the bus is
 *    live at boot. IIC0's pads (4/5) are not, and IIC0 transmit has never
 *    been observed to complete -- do not put devices on bus 0 until that is
 *    resolved.
 */
#define IIC_STRIDE      0x300000
#define IIC_REG(bus, off) \
    (*(REG32_PTR_T)(IIC0_BASE + IIC_STRIDE * (bus) + (off)))

#define IICCON(bus)     IIC_REG(bus, 0x00)
#define IICSTAT(bus)    IIC_REG(bus, 0x04)
#define IICADD(bus)     IIC_REG(bus, 0x08)
#define IICDS(bus)      IIC_REG(bus, 0x0c)
#define IICUNK10(bus)   IIC_REG(bus, 0x10)

#define IICCON_ACK_GEN      (1 << 7)
#define IICCON_IRQPEND      (1 << 4)

#define IICSTAT_MODE_MTX    (3 << 6)    /* master transmit */
#define IICSTAT_MODE_MRX    (2 << 6)    /* master receive */
#define IICSTAT_BUSY_START  (1 << 5)
#define IICSTAT_TXRX_EN     (1 << 4)
#define IICSTAT_ACK_NAK     (1 << 0)

/* Generous, but finite: a wedged bus must not take the firmware with it. */
#define I2C_TIMEOUT_US      100000

static struct mutex i2c_mtx[2];

static void wait_rdy(int bus)
{
    unsigned stop = USEC_TIMER + I2C_TIMEOUT_US;

    while (IICUNK10(bus)) {
        if (TIME_AFTER(USEC_TIMER, stop))
            return;
    }
}

/* Wait for the current byte to complete. Returns false on timeout. */
static bool wait_byte(int bus)
{
    unsigned stop = USEC_TIMER + I2C_TIMEOUT_US;

    while (!(IICCON(bus) & IICCON_IRQPEND)) {
        if (TIME_AFTER(USEC_TIMER, stop))
            return false;
    }
    return true;
}

/* Clear IRQPEND, which is also what releases the bus for the next byte. */
static void next_byte(int bus)
{
    IICCON(bus) |= IICCON_IRQPEND;
}

static void i2c_stop(int bus)
{
    IICSTAT(bus) = IICSTAT_MODE_MTX | IICSTAT_TXRX_EN;
    next_byte(bus);
    wait_rdy(bus);
    IICSTAT(bus) = 0;
}

static bool i2c_start(int bus, unsigned char slave, bool read)
{
    wait_rdy(bus);

    IICCON(bus) = IICCON_ACK_GEN;
    IICADD(bus) = slave << 1;

    /* Address byte: 7-bit slave, LSB is the direction. */
    IICDS(bus) = (slave << 1) | (read ? 1 : 0);
    IICSTAT(bus) = (read ? IICSTAT_MODE_MRX : IICSTAT_MODE_MTX)
                 | IICSTAT_TXRX_EN | IICSTAT_BUSY_START;

    if (!wait_byte(bus))
        return false;

    /* NAK on the address means nothing answered. */
    return !(IICSTAT(bus) & IICSTAT_ACK_NAK);
}

void i2c_preinit(int bus)
{
    (void)bus;
    /*
     * The bootloader leaves IIC1 configured, so re-muxing here would be a
     * no-op at best. IIC0 would need pads 4/5 muxed; see the note above.
     */
}

void i2c_init(void)
{
    mutex_init(&i2c_mtx[0]);
    mutex_init(&i2c_mtx[1]);
}

int i2c_wr(int bus, unsigned char slave, int address,
           int len, const unsigned char *data)
{
    int i;
    int ret = 0;

    if (!i2c_start(bus, slave, false)) {
        ret = -1;
        goto out;
    }

    if (address >= 0) {
        next_byte(bus);
        IICDS(bus) = (unsigned char)address;
        if (!wait_byte(bus)) {
            ret = -2;
            goto out;
        }
    }

    for (i = 0; i < len; i++) {
        next_byte(bus);
        IICDS(bus) = data[i];
        if (!wait_byte(bus)) {
            ret = -3;
            goto out;
        }
    }

out:
    i2c_stop(bus);
    return ret;
}

int i2c_rd(int bus, unsigned char slave, int address,
           int len, unsigned char *data)
{
    int i;
    int ret = 0;

    /* Addressed read: write the register index, then repeated-start to read. */
    if (address >= 0) {
        if (!i2c_start(bus, slave, false)) {
            ret = -1;
            goto out;
        }
        next_byte(bus);
        IICDS(bus) = (unsigned char)address;
        if (!wait_byte(bus)) {
            ret = -2;
            goto out;
        }
    }

    if (!i2c_start(bus, slave, true)) {
        ret = -3;
        goto out;
    }

    for (i = 0; i < len; i++) {
        /* NAK the final byte so the slave releases the bus. */
        if (i == len - 1)
            IICCON(bus) &= ~IICCON_ACK_GEN;

        next_byte(bus);
        if (!wait_byte(bus)) {
            ret = -4;
            goto out;
        }
        data[i] = IICDS(bus);
    }

out:
    i2c_stop(bus);
    return ret;
}

int i2c_write(int bus, unsigned char slave, int address,
              int len, const unsigned char *data)
{
    int ret;

    mutex_lock(&i2c_mtx[bus]);
    ret = i2c_wr(bus, slave, address, len, data);
    mutex_unlock(&i2c_mtx[bus]);
    return ret;
}

int i2c_read(int bus, unsigned char slave, int address,
             int len, unsigned char *data)
{
    int ret;

    mutex_lock(&i2c_mtx[bus]);
    ret = i2c_rd(bus, slave, address, len, data);
    mutex_unlock(&i2c_mtx[bus]);
    return ret;
}
