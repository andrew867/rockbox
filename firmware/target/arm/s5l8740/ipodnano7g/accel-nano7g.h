/***************************************************************************
 * ST LIS331DLH accelerometer for the iPod nano 7G (N31).
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#ifndef __ACCEL_NANO7G_H__
#define __ACCEL_NANO7G_H__

#include <stdbool.h>
#include <stdint.h>

struct accel_reading {
    int16_t x;
    int16_t y;
    int16_t z;
};

/* Probe and configure. Returns false if the part does not answer. */
bool accel_init(void);

/* Signed 12-bit counts per axis. False if unavailable or the read failed. */
bool accel_read(struct accel_reading *out);

bool accel_available(void);

#endif /* __ACCEL_NANO7G_H__ */
