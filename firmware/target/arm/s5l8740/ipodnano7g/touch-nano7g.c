/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * TI 343S0538 "Nimbus" touchscreen for the iPod nano 7G (N31).
 *
 * Ported from tools/linux-n31/drivers/apple-nimbus.c.
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
#include <string.h>
#include "config.h"
#include "system.h"
#include "kernel.h"
#include "button.h"
#include "touchscreen.h"
#include "spi-s5l8740.h"
#include "gpio-s5l8740.h"
#include "touch-nano7g.h"

/*
 * The controller sits on SPI2 with three control GPIOs: enable 14, reset 39
 * (active low) and an attention line on 38 (active low).
 *
 * Framing is fixed-size frames whose first byte is the magic 0xEA, with a
 * plain 16-bit additive checksum of the first 14 bytes stored little-endian
 * at the end of the frame. Reports come back as a payload whose first byte is
 * 'D' (0x44) -- the only report type this port needs.
 *
 * Rockbox wants a single absolute coordinate, so the multitouch slot handling
 * in the Linux driver collapses to "first finger with tip down wins".
 *
 * STATUS: the Linux driver's "first success" -- stable raw X/Y at one point --
 * has not been confirmed on hardware. This code follows the same sequence, so
 * if touch misbehaves, suspect the sequence rather than this transcription.
 */

#define NIMBUS_MAGIC        0xEA
#define NIMBUS_FRAME_LEN    16
#define NIMBUS_READ_MAX     512

#define NIMBUS_ABS_X_MAX    (LCD_WIDTH - 1)
#define NIMBUS_ABS_Y_MAX    (LCD_HEIGHT - 1)
#define NIMBUS_SCALE_X_DIV  0x0B1D
#define NIMBUS_SCALE_Y_DIV  0x1482
#define NIMBUS_COORD_BIAS   75

#define NIMBUS_SLOTS        8

static bool nimbus_up;
static int  last_x, last_y;
static bool last_down;

static uint8_t txbuf[NIMBUS_READ_MAX];
static uint8_t rxbuf[NIMBUS_READ_MAX];

static uint16_t nimbus_sum16(const uint8_t *buf, int len)
{
    uint16_t sum = 0;

    while (len-- > 0)
        sum += *buf++;
    return sum;
}

static void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = v & 0xff;
    p[1] = v >> 8;
}

static uint16_t get_le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

/*
 * The panel's raw range is wider than the pixel grid and its Y axis runs the
 * other way, so the mapping is a scale plus a flip, both from the stock
 * firmware's own constants.
 */
static void nimbus_map_coords(int16_t rawx, int16_t rawy, int *x, int *y)
{
    int xx = (NIMBUS_ABS_X_MAX * ((int)rawx + NIMBUS_COORD_BIAS))
           / NIMBUS_SCALE_X_DIV;
    int yy = (NIMBUS_ABS_Y_MAX * ((int)rawy + NIMBUS_COORD_BIAS))
           / NIMBUS_SCALE_Y_DIV;

    if (xx < 0)
        xx = 0;
    if (xx > NIMBUS_ABS_X_MAX)
        xx = NIMBUS_ABS_X_MAX;

    yy = NIMBUS_ABS_Y_MAX - yy;
    if (yy < 0)
        yy = 0;
    if (yy > NIMBUS_ABS_Y_MAX)
        yy = NIMBUS_ABS_Y_MAX;

    *x = xx;
    *y = yy;
}

/*
 * All transfers go through the SPI driver rather than poking SPI2 directly.
 *
 * The Linux driver originally drove the registers itself and did not treat a
 * receive timeout as a failure: it read RXDATA either way and returned
 * success, so when the ready bit never set, the FIFO's previous contents came
 * back as if they were a reply. That is where the repeating 0x4f81 came from
 * -- the part appeared to answer while telling us nothing.
 *
 * spi_transfer() now fails on timeout instead, so a dead bus reports as dead.
 *
 * The receive buffer is always supplied even for transmit-only frames: the
 * part clocks a reply out regardless, and leaving it undrained overruns the
 * RX FIFO partway through a long burst.
 */
static int nimbus_xfer(int len)
{
    return spi_transfer(SPI_PORT_TOUCH, txbuf, rxbuf, len);
}

/* A burst that keeps the part selected, for frames spanning several calls. */
static int nimbus_xfer_hold(int len) __attribute__((unused));
static int nimbus_xfer_hold(int len)
{
    return spi_transfer_cs(SPI_PORT_TOUCH, txbuf, rxbuf, len, false);
}

/* The 16-byte probe frame: magic, 01 01, checksum at the tail. */
static bool nimbus_probe_frame(void)
{
    memset(txbuf, 0, NIMBUS_FRAME_LEN);
    txbuf[0] = NIMBUS_MAGIC;
    txbuf[1] = 0x01;
    txbuf[2] = 0x01;
    put_le16(txbuf + 14, nimbus_sum16(txbuf, 14));

    if (nimbus_xfer(NIMBUS_FRAME_LEN) < 0)
        return false;

    return rxbuf[0] == NIMBUS_MAGIC;
}

/*
 * Report type 'D'. Layout: byte 0 is the type, byte 2 the offset to the first
 * record, byte 16 the record count and byte 17 the record stride. Each record
 * carries the tip state at +1 and a little-endian X/Y pair at +4 and +6.
 */
static void nimbus_parse_reports(const uint8_t *payload, int len)
{
    const uint8_t *rec;
    int off, count, stride, i;

    if (len < 18 || payload[0] != 0x44)
        return;

    off = payload[2];
    count = payload[16];
    stride = payload[17];

    if (!stride || off >= len)
        return;
    if (count > NIMBUS_SLOTS)
        count = NIMBUS_SLOTS;

    rec = payload + off;

    for (i = 0; i < count; i++) {
        int16_t rawx, rawy;

        if (rec + stride > payload + len)
            break;

        if (rec[1]) {
            rawx = (int16_t)get_le16(rec + 4);
            rawy = (int16_t)get_le16(rec + 6);
            nimbus_map_coords(rawx, rawy, &last_x, &last_y);
            last_down = true;
            return;             /* single touch: first finger down wins */
        }

        rec += stride;
    }

    last_down = false;
}

static void nimbus_read_reports(void)
{
    int len = NIMBUS_FRAME_LEN;

    memset(txbuf, 0, len);
    txbuf[0] = NIMBUS_MAGIC;
    txbuf[1] = 0x01;
    txbuf[2] = 0x01;
    /* Stock writes the checksum at TX + (len - 2), not at a fixed offset. */
    put_le16(txbuf + len - 2, nimbus_sum16(txbuf, 14));

    if (nimbus_xfer(len) < 0)
        return;

    if (rxbuf[0] != NIMBUS_MAGIC)
        return;

    if (rxbuf[2] && rxbuf[2] != 2) {
        int plen = rxbuf[2];

        if (plen >= 2 && (5 + (plen - 2)) <= len)
            nimbus_parse_reports(rxbuf + 5, plen - 2);
    } else if (len > 5 && rxbuf[5] == 0x44) {
        nimbus_parse_reports(rxbuf + 5, len - 5);
    }
}

bool touch_init(void)
{
    int tries;

    /* Enable high, then pulse reset low -- reset is active low. */
    gpio_direction_output(GPIO_PAD_NIMBUS_EN, true);
    sleep(HZ / 100);

    gpio_direction_output(GPIO_PAD_NIMBUS_RST, false);
    sleep(HZ / 100);
    gpio_direction_output(GPIO_PAD_NIMBUS_RST, true);
    sleep(HZ / 20);

    /* Attention line is an input; the controller pulls it low when ready. */
    gpio_set_function(GPIO_PAD_NIMBUS_IRQ, GPIO_FUNC_IN);

    /* Re-run the SPI2 engine setup, as the stock firmware does after enable. */
    spi_port_init(SPI_PORT_TOUCH);

    for (tries = 0; tries < 10; tries++) {
        if (nimbus_probe_frame()) {
            nimbus_up = true;
            return true;
        }
        sleep(HZ / 50);
    }

    /*
     * Ten clean failures, not ten stale-FIFO successes. spi_transfer() now
     * propagates receive timeouts, so reaching here means the part genuinely
     * did not answer rather than that we misread the status bits.
     */

    /*
     * TODO: firmware upload. The stock path pushes GrapeFirmware.bin (shipped
     * here as firmware/apple/grape-nimbus.bin) in 0x1FF0-byte chunks before
     * the controller reports properly, and follows it with a calibration
     * blob. That needs a filesystem, which arrives with the FTL in Phase 5.
     *
     * On a device where U-Boot or RetailOS has already loaded the controller
     * firmware, the probe above succeeds without it.
     */
    return false;
}

bool touch_available(void)
{
    return nimbus_up;
}

/*
 * Rockbox turns the digitiser off with the backlight to save power. Dropping
 * the enable line is the whole mechanism; the controller reloads its state
 * when it comes back, which is why touch_init() re-runs the probe.
 */
void touchscreen_enable_device(bool en)
{
    if (!nimbus_up)
        return;

    gpio_direction_output(GPIO_PAD_NIMBUS_EN, en);
}

int touchscreen_read_device(int *x, int *y)
{
    if (!nimbus_up)
        return 0;

    /* The attention line is active low: high means nothing new. */
    if (gpio_get(GPIO_PAD_NIMBUS_IRQ))
        return last_down ? BUTTON_TOUCHSCREEN : 0;

    nimbus_read_reports();

    if (!last_down)
        return 0;

    if (x)
        *x = last_x;
    if (y)
        *y = last_y;

    return BUTTON_TOUCHSCREEN;
}
