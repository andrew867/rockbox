/***************************************************************************
 * FM tuner inside the Broadcom BCM2078, driven over HCI.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#ifndef _BCM2078_TUNER_H_
#define _BCM2078_TUNER_H_

int bcm2078_tuner_set(int setting, int value);
int bcm2078_tuner_get(int setting);

#define tuner_set bcm2078_tuner_set
#define tuner_get bcm2078_tuner_get

#endif /* _BCM2078_TUNER_H_ */
