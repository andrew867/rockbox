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
#include <string.h>
#include "config.h"
#include "system.h"
#include "i2c-s5l8740.h"
#include "tristar-nano7g.h"

/*
 * Identification only, and deliberately read-only.
 *
 * "Mux map" is a misleading way to think about this part. Routing happens
 * over IDBUS *inside* the chip: the accessory presents an ID, and the
 * CBTL1609 programs its own ACCx/Dx switches from it per the THS7383 tables.
 * RetailOS on N31 was observed writing zero Dx/mux registers over I2C. There
 * is no host-side mux map to implement, and inventing one risks leaving the
 * connector in a state the stock firmware cannot recover from.
 *
 * What the host CAN do is read the ID and know what is plugged in. That is
 * what this does.
 *
 * The chip is on IIC1 (NOT IIC0, despite what the first device tree said) at
 * 7-bit 0x1a, which is 8-bit 0x34 write / 0x35 read.
 *
 * Reading all 0x35 back is the classic false positive: that is the bus
 * echoing the 8-bit read address, not a register dump. A flat buffer of the
 * address byte means the transfer did not really happen.
 */
#define TRISTAR_BUS     I2C_BUS_IIC1
#define TRISTAR_ADDR    I2C_ADDR_TRISTAR

#define TRISTAR_DUMP_LEN    0x40

/* The 8-bit read address, which is what a dead bus echoes back. */
#define I2C_ADDR_ECHO       0x35

#define TS_ID_ACCX(id0)     (((id0) >> 6) & 3)
#define TS_ID_DX(id0)       (((id0) >> 4) & 3)

/*
 * Known Lightning accessory signatures (nyansatan's ID table, HOSTID=1).
 * The six-byte signature appears at an unpredictable offset inside the dump,
 * so it is searched for rather than read from a fixed register.
 */
struct tristar_id_sig {
    unsigned char bytes[6];
    const char   *name;
    enum tristar_accessory kind;
};

static const struct tristar_id_sig tristar_known_ids[] = {
    { { 0x10, 0x0c, 0x00, 0x00, 0x00, 0x00 }, "usb-cable",           TRISTAR_ACC_USB },
    { { 0x04, 0xf1, 0x00, 0x00, 0x00, 0x00 }, "lightning-analog",    TRISTAR_ACC_ANALOG },
    { { 0x0b, 0xf0, 0x00, 0x00, 0x00, 0x00 }, "haywire-hdmi",        TRISTAR_ACC_VIDEO },
    { { 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }, "dcsd-or-uart-charge", TRISTAR_ACC_SERIAL },
    { { 0x20, 0x02, 0x00, 0x00, 0x00, 0x00 }, "kong-swd-idle",       TRISTAR_ACC_SERIAL },
    { { 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00 }, "kong-swd-astris",     TRISTAR_ACC_SERIAL },
    { { 0x20, 0x0e, 0x00, 0x00, 0x00, 0x00 }, "kanzi-swd-idle",      TRISTAR_ACC_SERIAL },
    { { 0xa0, 0x0c, 0x00, 0x00, 0x00, 0x00 }, "kanzi-swd-astris",    TRISTAR_ACC_SERIAL },
    { { 0x20, 0x00, 0x10, 0x00, 0x00, 0x00 }, "uart-charge",         TRISTAR_ACC_SERIAL },
};

static bool tristar_present;
static unsigned char last_dump[TRISTAR_DUMP_LEN];
static struct tristar_id cached;

/* Read the full register window. False if it did not happen for real. */
static bool tristar_dump(void)
{
    int i;
    bool flat = true;
    bool echo = true;

    if (i2c_read(TRISTAR_BUS, TRISTAR_ADDR, 0x00,
                 TRISTAR_DUMP_LEN, last_dump) < 0)
        return false;

    for (i = 0; i < TRISTAR_DUMP_LEN; i++) {
        if (last_dump[i] != I2C_ADDR_ECHO)
            echo = false;
        if (last_dump[i] != last_dump[0])
            flat = false;
    }

    /*
     * All-0x35 is the address echo. A flat dump of anything else is an idle
     * connector, which is a real reading -- just not an accessory.
     */
    if (echo)
        return false;

    cached.flat = flat;
    return true;
}

bool tristar_init(void)
{
    tristar_present = tristar_dump();
    return tristar_present;
}

bool tristar_read_id(struct tristar_id *out)
{
    unsigned s;
    int off;

    memset(&cached, 0, sizeof(cached));
    cached.kind = TRISTAR_ACC_NONE;
    cached.name = "unread";

    if (!tristar_present)
        return false;

    if (!tristar_dump()) {
        cached.name = "i2c-echo";
        return false;
    }

    if (cached.flat) {
        cached.name = "idle";
        cached.kind = TRISTAR_ACC_NONE;
        if (out)
            *out = cached;
        return true;
    }

    for (s = 0; s < ARRAYLEN(tristar_known_ids); s++) {
        const struct tristar_id_sig *sig = &tristar_known_ids[s];

        for (off = 0; off + 6 <= TRISTAR_DUMP_LEN; off++) {
            if (memcmp(last_dump + off, sig->bytes, 6))
                continue;

            cached.id0  = sig->bytes[0];
            cached.id1  = sig->bytes[1];
            cached.accx = TS_ID_ACCX(sig->bytes[0]);
            cached.dx   = TS_ID_DX(sig->bytes[0]);
            cached.name = sig->name;
            cached.kind = sig->kind;
            cached.valid = true;

            if (out)
                *out = cached;
            return true;
        }
    }

    /*
     * A non-flat dump with no matching signature is a real accessory we do
     * not have an entry for. Report it as unknown with the raw bytes rather
     * than guessing at what it might be.
     */
    cached.id0 = last_dump[0];
    cached.id1 = last_dump[1];
    cached.accx = TS_ID_ACCX(last_dump[0]);
    cached.dx = TS_ID_DX(last_dump[0]);
    cached.name = "unknown";
    cached.kind = TRISTAR_ACC_UNKNOWN;

    if (out)
        *out = cached;
    return true;
}

bool tristar_accessory_attached(void)
{
    struct tristar_id id;

    if (!tristar_read_id(&id))
        return false;

    return id.kind != TRISTAR_ACC_NONE;
}

const char *tristar_accessory_name(void)
{
    struct tristar_id id;

    if (!tristar_read_id(&id))
        return "unavailable";

    return id.name;
}

bool tristar_available(void)
{
    return tristar_present;
}

const unsigned char *tristar_last_dump(void)
{
    return last_dump;
}
