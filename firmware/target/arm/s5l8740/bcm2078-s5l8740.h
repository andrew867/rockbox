/***************************************************************************
 * BCM2078 HCI transport (Bluetooth + FM) for Apple S5L8740.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#ifndef __BCM2078_S5L8740_H__
#define __BCM2078_S5L8740_H__

#include <stdbool.h>
#include <stdint.h>

/* Power the controller and bring the HCI link up to 2.4 Mbaud. */
bool bcm2078_init(void);
void bcm2078_shutdown(void);
bool bcm2078_available(void);

/*
 * Issue an FM vendor command (HCI opcode 0xFC15) and wait for its Command
 * Complete. Returns the status byte, or negative on timeout.
 */
int bcm2078_fm_cmd(const uint8_t *payload, int len,
                   uint8_t *reply, int reply_len);

#endif /* __BCM2078_S5L8740_H__ */
