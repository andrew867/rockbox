/***************************************************************************
 * ADC channels for the iPod nano 7G (N31).
 *
 * There is no SoC ADC in play: everything measurable comes from the D1830
 * PMIC's own converter over I2C.
 ****************************************************************************/
#ifndef _ADC_TARGET_H_
#define _ADC_TARGET_H_

#include <stdbool.h>

enum {
    ADC_BATTERY = 0,
    NUM_ADC_CHANNELS
};

#define ADC_UNREG_POWER ADC_BATTERY /* For compatibility */

unsigned short adc_read_millivolts(int channel);
unsigned int adc_read_battery_voltage(void);
const char *adc_name(int channel);

#endif
