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
#ifndef __PMU_TARGET_H__
#define __PMU_TARGET_H__

#include <stdbool.h>
#include <stdint.h>

/*
 * Dialog D1830 / Apple 338S1099 on IIC1 at 7-bit 0x73 (wire 0xE6/0xE7).
 * Ported from tools/linux-n31/drivers/gpio-d1830.c.
 */

void pmu_init(void);

/* Bluetooth rail: snapshot the boot state, then restore or drop it. */
void pmu_bt_rail_snapshot(void);
bool pmu_bt_rail_enable(bool on);

int  pmu_read(int reg);
int  pmu_write(int reg, unsigned char value);

/* Packed key state from regs 7 and 8, already mapped to BUTTON_* bits. */
int  pmu_read_buttons(void);

/* Battery voltage in millivolts, or -1 if not available. */
int  pmu_read_battery_voltage(void);

bool pmu_is_charging(void);
bool pmu_is_usb_present(void);

/* Enable the sibling LDOs the analog audio path needs (D1830 sub_23EC). */
void pmu_audio_rails(bool on);

void pmu_power_off(void);

#endif /* __PMU_TARGET_H__ */
