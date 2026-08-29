/***************************************************************************
 * ADC for the iPod nano 7G (N31).
 *
 * Thin shim onto the D1830 PMIC converter -- see pmu-nano7g.c, which carries
 * the RetailOS VBAT conversion formula.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#include "config.h"
#include "adc.h"
#include "adc-target.h"
#include "pmu-target.h"

unsigned int adc_read_battery_voltage(void)
{
    int mv = pmu_read_battery_voltage();

    /*
     * A failed I2C read must not read as a flat battery -- that would trip
     * the shutoff path. Report the nominal boot threshold instead.
     */
    return (mv < 0) ? 3700 : (unsigned int)mv;
}

unsigned short adc_read_millivolts(int channel)
{
    if (channel != ADC_BATTERY)
        return 0;
    return (unsigned short)adc_read_battery_voltage();
}

const char *adc_name(int channel)
{
    (void)channel;
    return "battery";
}

void adc_init(void)
{
}
