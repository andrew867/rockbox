/***************************************************************************
 * Power control for the iPod nano 7G (N31).
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#include "config.h"
#include "system.h"
#include "power.h"
#include "pmu-target.h"

void power_init(void)
{
    pmu_init();
}

void power_off(void)
{
    pmu_power_off();
}

unsigned int power_input_status(void)
{
    return pmu_is_usb_present() ? POWER_INPUT_USB_CHARGER : POWER_INPUT_NONE;
}

bool charging_state(void)
{
    return pmu_is_charging();
}

#ifdef HAVE_USB_CHARGING_ENABLE
/*
 * Called when the host grants a different current budget after enumeration.
 *
 * TODO: the D1830 charge-current register is not RE'd yet, so this records
 * the request without acting on it. Charging still works at whatever rate the
 * PMIC defaults to -- it simply is not being negotiated up to 500 mA.
 */
void usb_charging_maxcurrent_change(int maxcurrent)
{
    (void)maxcurrent;
}
#endif
