/***************************************************************************
 * TI 343S0538 "Nimbus" touchscreen for the iPod nano 7G (N31).
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#ifndef __TOUCH_NANO7G_H__
#define __TOUCH_NANO7G_H__

#include <stdbool.h>

/*
 * HBPP firmware upload results. "No file" is reported separately from a
 * protocol failure because the two mean very different things: one is a
 * packaging problem, the other is a bus or frame-format problem.
 */
#define TOUCH_HBPP_OK           0
#define TOUCH_HBPP_ERR_NOFILE   1
#define TOUCH_HBPP_ERR_UPLOAD   2
#define TOUCH_HBPP_ERR_EXEC     3
#define TOUCH_HBPP_ERR_BUS      4

/*
 * Upload the controller firmware and calibration from disk, then start it.
 * Requires a mounted filesystem, so it can only run after storage is up.
 */
int touch_hbpp_load(void);

/* Last HBPP result, for the debug screen. */
int touch_hbpp_status(void);

bool touch_init(void);
bool touch_available(void);

/*
 * Nonzero when the controller replied with a bootloader status word instead
 * of a valid runtime frame -- alive on the bus, application not running.
 */
uint16_t touch_bootloader_status(void);
unsigned touch_ping_failures(void);

/* True when per-device calibration was found in the IsyS handoff. */
bool touch_cal_loaded(void);

/*
 * Poll the controller. Returns BUTTON_TOUCHSCREEN with *x / *y filled when a
 * finger is down, or 0 otherwise.
 */
int touchscreen_read_device(int *x, int *y);

#endif /* __TOUCH_NANO7G_H__ */
