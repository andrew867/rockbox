/***************************************************************************
 * NXP CBTL1609A1 "Tristar" Lightning mux for the iPod nano 7G (N31).
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#ifndef __TRISTAR_NANO7G_H__
#define __TRISTAR_NANO7G_H__

#include <stdbool.h>
#include <stdint.h>

enum tristar_accessory {
    TRISTAR_ACC_NONE = 0,   /* idle connector */
    TRISTAR_ACC_USB,        /* plain USB cable */
    TRISTAR_ACC_ANALOG,     /* Lightning analog EarPods -- NOT the 3.5mm jack */
    TRISTAR_ACC_VIDEO,      /* Haywire HDMI adapter */
    TRISTAR_ACC_SERIAL,     /* DCSD / UART-charge / SWD debug bricks */
    TRISTAR_ACC_UNKNOWN,    /* real accessory, no signature match */
};

struct tristar_id {
    uint8_t id0;
    uint8_t id1;
    uint8_t accx;           /* id0 bits 7:6 */
    uint8_t dx;             /* id0 bits 5:4 */
    bool    valid;          /* matched a known signature */
    bool    flat;           /* dump was uniform -- idle connector */
    enum tristar_accessory kind;
    const char *name;
};

bool tristar_init(void);
bool tristar_read_id(struct tristar_id *out);
bool tristar_accessory_attached(void);
const char *tristar_accessory_name(void);
bool tristar_available(void);

/* Raw 0x40-byte register window, for the debug screens. */
const unsigned char *tristar_last_dump(void);

#endif /* __TRISTAR_NANO7G_H__ */
