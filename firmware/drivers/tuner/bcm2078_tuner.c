/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * FM tuner inside the Broadcom BCM2078, driven over HCI.
 *
 * Ported from tools/linux-n31/FM-FC15-COOKBOOK.md and the FM half of
 * tools/linux-n31/drivers/bcm2078-bt.c.
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
#include <stdbool.h>
#include <string.h>
#include "config.h"
#include "system.h"
#include "kernel.h"
#include "tuner.h"
#include "bcm2078_tuner.h"
#include "bcm2078-s5l8740.h"
#include "iis-s5l8740.h"

/*
 * This is not an I2C tuner chip. The FM receiver lives inside the Bluetooth
 * controller and is reached through HCI vendor opcode 0xFC15:
 *
 *     01 15 FC <plen> <payload>
 *
 *     payload write8:  <reg> 00 <val>
 *     payload write16: <reg> 00 <lo> <hi>
 *     payload read:    <reg> 01 <len>
 *
 * Audio does not come back over HCI either -- the controller streams PCM to
 * the SoC on IIS2, which is why tuning also starts the IIS2 capture.
 *
 * Every payload below is from the RetailOS BroadcomFM decompilation
 * (sub_4290C4 -> sub_FA9B6), not invented.
 */

#define FM_BAND_SPLIT_KHZ   87500

/* Tuning encodes frequency in kHz plus a fixed 1536 offset. */
#define FM_TUNE_OFFSET      1536

#define FM_DEFAULT_RSSI     33
#define FM_DEFAULT_NOISE    20

static bool fm_powered;
static bool fm_muted = true;
static int  fm_frequency;

static int fm_write8(uint8_t reg, uint8_t val)
{
    uint8_t p[3] = { reg, 0x00, val };

    return bcm2078_fm_cmd(p, sizeof(p), NULL, 0);
}

static int fm_write16(uint8_t reg, uint16_t val)
{
    uint8_t p[4] = { reg, 0x00, val & 0xff, val >> 8 };

    return bcm2078_fm_cmd(p, sizeof(p), NULL, 0);
}

static int fm_read(uint8_t reg, uint8_t *out, int len)
{
    uint8_t p[3] = { reg, 0x01, (uint8_t)len };

    return bcm2078_fm_cmd(p, sizeof(p), out, len);
}

/* DD136: the stock power-on writes three registers in this order. */
static void fm_power(bool on)
{
    if (on == fm_powered)
        return;

    if (on) {
        if (!bcm2078_available() && !bcm2078_init())
            return;

        fm_write8(0x00, 0x03);
        fm_write8(0x14, 0x0c);
        fm_write8(0x02, 0x02);

        /* DD334(0): route audio to the PCM interface. */
        {
            uint8_t route[4] = { 0x05, 0x00, 0x01, 0x00 };

            bcm2078_fm_cmd(route, sizeof(route), NULL, 0);
        }

        /* DD2FC: seek thresholds, RSSI 33 / noise 20. */
        {
            uint8_t thr[11] = {
                0xf9, 0x00, FM_DEFAULT_RSSI, 0x00, 0x00, 0x00,
                FM_DEFAULT_NOISE, 0x00, 0x00, 0x00, 0x00
            };

            bcm2078_fm_cmd(thr, sizeof(thr), NULL, 0);
        }

        /* The audio itself arrives on IIS2, not over HCI. */
        iis2_capture_start();
        fm_powered = true;
    } else {
        iis2_capture_stop();
        fm_write8(0x00, 0x00);      /* DD118 */
        fm_powered = false;
    }
}

static void fm_set_frequency(int freq_hz)
{
    int khz = freq_hz / 1000;
    uint16_t enc;

    if (!fm_powered)
        return;

    /* 56DA98: band select is a hard split at 87.5 MHz. */
    fm_write8(0x01, (khz >= FM_BAND_SPLIT_KHZ) ? 0x02 : 0x03);

    /* 56DB66: pre-tune. */
    fm_write16(0x10, 0x1203);

    enc = (uint16_t)((khz + FM_TUNE_OFFSET) & 0xffff);
    fm_write16(0x0a, enc);

    fm_frequency = freq_hz;
}

static void fm_set_mute(bool mute)
{
    if (!fm_powered)
        return;

    /* DD26C: 1 unmutes. */
    fm_write8(0x09, mute ? 0x00 : 0x01);
    fm_muted = mute;
}

/*
 * 56DB8E: flags 0x70 seeks down, 0xF0 seeks up. The seek is started and then
 * followed up by DD374's two-command sequence.
 */
static void fm_seek(int direction)
{
    if (!fm_powered)
        return;

    fm_write8(0x07, direction >= 0 ? 0xf0 : 0x70);
    fm_write8(0x08, FM_DEFAULT_RSSI);
    fm_write8(0xde, 0x01);

    fm_write8(0xfc, 0x00);
    fm_write8(0x09, 0x02);
}

int bcm2078_tuner_set(int setting, int value)
{
    switch (setting) {
    case RADIO_SLEEP:
        fm_power(value == 0);
        return 1;

    case RADIO_FREQUENCY:
        fm_set_frequency(value);
        return 1;

    case RADIO_MUTE:
        fm_set_mute(value != 0);
        return 1;

    case RADIO_SCAN_FREQUENCY:
        fm_set_frequency(value);
        fm_seek(1);
        return 1;

    case RADIO_FORCE_MONO:
        /*
         * TODO: the mono/stereo forcing register is not in the RE'd command
         * table. Accept and ignore rather than poking a guessed register.
         */
        return 1;

    default:
        return -1;
    }
}

int bcm2078_tuner_get(int setting)
{
    uint8_t status[4];

    switch (setting) {
    case RADIO_PRESENT:
        return bcm2078_available() ? 1 : 0;

    case RADIO_TUNED:
        if (!fm_powered)
            return 0;
        /* DD458: read the status register. */
        if (fm_read(0x4d, status, 1) < 0)
            return 0;
        return (status[0] & 0x01) ? 1 : 0;

    case RADIO_STEREO:
        if (!fm_powered)
            return 0;
        if (fm_read(0x4d, status, 1) < 0)
            return 0;
        return (status[0] & 0x02) ? 1 : 0;

    case RADIO_RSSI:
        if (!fm_powered)
            return 0;
        if (fm_read(0x4d, status, 1) < 0)
            return 0;
        return status[0];

    case RADIO_RSSI_MIN:
        return 0;

    case RADIO_RSSI_MAX:
        return 0xff;

    default:
        return -1;
    }
}
