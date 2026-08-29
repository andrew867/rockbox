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
#ifndef __IIS_S5L8740_H__
#define __IIS_S5L8740_H__

#include <stdbool.h>
#include <stdint.h>

/*
 * Two I2S blocks are in use on N31:
 *
 *   IIS0 @0x3CA00000  playback to the CS42L81 codec (PL080 peri 10)
 *   IIS2 @0x3D400000  capture from the BCM2078 (PL080 peri 13) -- FM audio
 *                     and Bluetooth PCM
 *
 * IIS1 @0x3CD00000 exists but is unused on this board.
 */

#define I2SCLKCON   0x00
#define I2STXCON    0x04
#define I2STXCOM    0x08
#define I2STXFIFO   0x10
#define I2SRXCON    0x30
#define I2SRXCOM    0x34
#define I2SRXFIFO   0x38
#define I2SSTATUS   0x3c
/* OSOS sub_4F716 uses +0x40. NOT the +0x24 of the Rockbox s5l8702 driver. */
#define I2SCLKDIV   0x40
#define I2SREG44    0x44

#define IIS_REG(base, off)  (*(REG32_PTR_T)((base) + (off)))

/* IIS0, from the RetailOS music oracle. */
#define I2STXCON_N31_16     0x03100099u
#define I2SRXCON_N31        0x00001000u
#define I2SREG44_MUSIC      0x00010007u

#define I2STXCOM_STOP       0x0
#define I2STXCOM_DMA        0x6     /* stock value; 0xE was a failure mode */
#define I2STXCOM_PIO        0xc

/* IIS2 / FM capture. */
#define IIS2_TXCON_FM       0x0b000099u
#define IIS2_RXCON_FM       0x00001000u
#define IIS2_RXCOM_DMA      0x6u
#define IIS2_RXCOM_IDLE     0x2u
#define IIS2_CLKDIV_FM      0x96u
#define IIS2_REG44          0x00010007u

/* IIS2 PCM pads, claimed at function 2 when FM powers on (sub_15DD5C). */
#define IIS2_PAD_BCLK       97
#define IIS2_PAD_SYNC       98
#define IIS2_PAD_DATA       119

/* CLKCON+0x30 audio parent dword. */
#define CLKCON_AUDIO_OFF    0x30
#define CLKCON_AUDIO_PLAY   0x32190u
#define CLKCON_AUDIO_IDLE   0x1c20u

/* The divider is derived against this, per the RetailOS rate table. */
#define MCLK_ASSUME_HZ      12000000

/* CS42L81 rate code for a sample rate, shared with the codec driver. */
uint8_t iis_codec_rate_code(unsigned int rate);

/* IIS0 STATUS -- 0x320 / 0x420 when healthy, for the debug screens. */
uint32_t iis0_status(void);

/*
 * IIS2 capture, used by the FM radio path.
 *
 * The capture ring is filled by the PL080 in the background; iis2_capture_read
 * drains it. Returns the number of bytes copied.
 */
void iis2_capture_start(void);
void iis2_capture_stop(void);
bool iis2_capture_active(void);
uint32_t iis2_status(void);

#endif /* __IIS_S5L8740_H__ */
