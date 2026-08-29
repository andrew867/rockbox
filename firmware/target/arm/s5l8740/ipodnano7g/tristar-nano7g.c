/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * NXP CBTL1609A1 "Tristar" Lightning mux for the iPod nano 7G (N31).
 *
 * Ported from tools/linux-n31/drivers/apple-tristar-cbtl1609.c.
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
#include "tristar-nano7g.h"

/*
 * Detection only, and deliberately read-only.
 *
 * Routing on this part happens over IDBUS inside the chip, not through
 * host-written Dx mux registers -- RetailOS writes no mux map, and inventing
 * one risks leaving the connector in a state the stock firmware cannot
 * recover. So this reads identity and reports it; it never writes.
 *
 * The chip is on IIC1 (NOT IIC0, despite what the first device tree said) at
 * 7-bit 0x1a, which is 8-bit 0x34 write / 0x35 read.
 *
 * Reading all 0x35 back is the classic false positive here: that is the bus
 * echoing the 8-bit read address, not a register dump. A flat buffer of the
 * address byte means the transfer did not really happen, so treat it as
 * "not present" rather than as data.
 */
#define TRISTAR_BUS     I2C_BUS_IIC1
#define TRISTAR_ADDR    I2C_ADDR_TRISTAR

#define TRISTAR_REG_ID0     0x00
#define TRISTAR_REG_ID1     0x01

/* The 8-bit read address, which is what a dead bus echoes back. */
#define I2C_ADDR_ECHO       0x35

#define TS_ID_ACCX(id0)     (((id0) >> 6) & 3)
#define TS_ID_DX(id0)       (((id0) >> 4) & 3)

static bool tristar_present = false;

bool tristar_init(void)
{
    unsigned char id[2];

    if (i2c_read(TRISTAR_BUS, TRISTAR_ADDR, TRISTAR_REG_ID0,
                 sizeof(id), id) < 0)
        return false;

    /* Address echo, not a dump. */
    if (id[0] == I2C_ADDR_ECHO && id[1] == I2C_ADDR_ECHO)
        return false;

    tristar_present = true;
    return true;
}

bool tristar_read_id(struct tristar_id *out)
{
    unsigned char id[2];

    if (!tristar_present)
        return false;

    if (i2c_read(TRISTAR_BUS, TRISTAR_ADDR, TRISTAR_REG_ID0,
                 sizeof(id), id) < 0)
        return false;

    if (id[0] == I2C_ADDR_ECHO && id[1] == I2C_ADDR_ECHO)
        return false;

    out->id0  = id[0];
    out->id1  = id[1];
    out->accx = TS_ID_ACCX(id[0]);
    out->dx   = TS_ID_DX(id[0]);

    return true;
}

bool tristar_accessory_attached(void)
{
    struct tristar_id id;

    if (!tristar_read_id(&id))
        return false;

    /*
     * An idle connector reads as a flat value with no IDBUS accessory. Any
     * non-zero accessory/Dx field means something is on the other end.
     *
     * TODO: this distinguishes "something attached" from "nothing", which is
     * all the mux map supports today. Telling a charger from a dock from a
     * host needs the IDBUS decode, which is still open RE.
     */
    return (id.accx != 0) || (id.dx != 0);
}

bool tristar_available(void)
{
    return tristar_present;
}
