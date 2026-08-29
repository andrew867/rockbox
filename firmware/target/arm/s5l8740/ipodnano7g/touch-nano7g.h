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

bool touch_init(void);
bool touch_available(void);

/*
 * Poll the controller. Returns BUTTON_TOUCHSCREEN with *x / *y filled when a
 * finger is down, or 0 otherwise.
 */
int touchscreen_read_device(int *x, int *y);

#endif /* __TOUCH_NANO7G_H__ */
