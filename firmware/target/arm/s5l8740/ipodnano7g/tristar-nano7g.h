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

struct tristar_id {
    uint8_t id0;
    uint8_t id1;
    uint8_t accx;   /* id0 bits 7:6 */
    uint8_t dx;     /* id0 bits 5:4 */
};

/* Probe. False if the part does not answer, or the bus only echoes. */
bool tristar_init(void);

bool tristar_read_id(struct tristar_id *out);

/* True when an IDBUS accessory is present on the Lightning connector. */
bool tristar_accessory_attached(void);

bool tristar_available(void);

#endif /* __TRISTAR_NANO7G_H__ */
