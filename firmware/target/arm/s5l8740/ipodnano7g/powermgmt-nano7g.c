/***************************************************************************
 * Battery curves for the iPod nano 7G (N31).
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#include "config.h"
#include "powermgmt.h"
#include "pmu-target.h"
#include "power.h"
#include "audiohw.h"
#include "adc-target.h"

/*
 * The low end is anchored to RetailOS sub_1E7C, which is real: it boots
 * normally at >= 3550 mV, goes to Low Power Boot between 3400 and 3549, and
 * below 3400 refuses to boot at all without external power.
 *
 * TODO: the middle of the discharge curve is the generic single-cell Li-ion
 * shape, not measured N31 data. It needs a real discharge run before the
 * battery percentage can be trusted.
 */
const unsigned short battery_level_disksafe = 3500;
const unsigned short battery_level_dangerous = 3400;
const unsigned short battery_level_shutoff = 3300;

/* voltages (millivolt) of 0%, 10%, ... 100% when charging disabled */
const unsigned short percent_to_volt_discharge[11] =
{
    3400, 3620, 3700, 3740, 3780, 3820, 3880, 3940, 4020, 4100, 4180
};

#if CONFIG_CHARGING
/* voltages (millivolt) of 0%, 10%, ... 100% when charging enabled */
const unsigned short percent_to_volt_charge[11] =
{
    3700, 3820, 3900, 3950, 3990, 4030, 4070, 4120, 4170, 4190, 4200
};
#endif /* CONFIG_CHARGING */

/* Returns battery voltage [millivolts] */
int _battery_voltage(void)
{
    return adc_read_battery_voltage();
}
