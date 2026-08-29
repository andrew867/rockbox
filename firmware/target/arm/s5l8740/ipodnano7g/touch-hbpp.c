/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Nimbus HBPP firmware upload for the iPod nano 7G touchscreen.
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
#include "file.h"
#include "spi-s5l8740.h"
#include "gpio-s5l8740.h"
#include "touch-nano7g.h"

/*
 * The touch controller has no firmware of its own in flash. Every cold boot,
 * the host uploads an ARM image over SPI2 using HBPP -- Apple's bootloader
 * protocol for this part -- then a calibration blob, then tells it to run.
 *
 * Until this port had storage there was nowhere to read those blobs from, so
 * touch only worked if a previous stage happened to have loaded the
 * controller already. With the FTL mounting the volume, they can come off
 * disk like everything else.
 *
 * ---------------------------------------------------------------------
 * Frame format
 *
 * HBPP fields are MIDDLE-ENDIAN, because the part is a 16-bit SPI slave and
 * every 32-bit value is carried as two byte-swapped halves: B1 B0 B3 B2. That
 * applies to the destination address, to the payload words, and to the
 * trailing checksum. Getting this wrong produces frames that look almost
 * right and are never acknowledged.
 *
 * Upload frame:
 *   18 E1 30 01                     opcode
 *   word_count BE u16               (len >> 10), (len >> 2)
 *   dest, swizzled B1 B0 B3 B2
 *   u16 BE byte-sum of the previous 6 bytes
 *   payload, u32s swizzled B1 B0 B3 B2
 *   u32 byte-sum of the swizzled payload, stored B1 B0 B3 B2
 *
 * Total SPI length is payload + 16. Maximum payload is 0x1FF0.
 * ---------------------------------------------------------------------
 */

#define HBPP_CHUNK_MAX      0x1ff0u
#define HBPP_HDR_LEN        16u

#define HBPP_ACK_CHUNK      0x4bc1u
#define HBPP_ENTER          0x19c1u     /* bootload cmd 6593 */

#define CAL_DEST            0x00400200u
#define CAL_LEN             0x200u

/* EXEC target, from OSOS 2D54C. */
#define EXEC_WORD0          0x00100018u
#define EXEC_WORD1          0x00000100u
#define EXEC_SETTLE_MS      40

#define FW_PATH             "/.rockbox/grape-nimbus.bin"
#define CAL_PATH            "/.rockbox/grape-nimbus-cal.bin"

/* One chunk plus its envelope. */
static uint8_t hbpp_tx[HBPP_CHUNK_MAX + HBPP_HDR_LEN];
static uint8_t hbpp_rx[HBPP_CHUNK_MAX + HBPP_HDR_LEN];
static uint8_t hbpp_src[HBPP_CHUNK_MAX];

static uint16_t sum16(const uint8_t *p, unsigned len)
{
    uint16_t s = 0;

    while (len--)
        s += *p++;
    return s;
}

static uint32_t sum32(const uint8_t *p, unsigned len)
{
    uint32_t s = 0;

    while (len--)
        s += *p++;
    return s;
}

/* B1 B0 B3 B2 over each 32-bit word. */
static void swizzle32(uint8_t *dst, const uint8_t *src, unsigned len)
{
    unsigned i;

    for (i = 0; i + 3 < len; i += 4) {
        dst[i]     = src[i + 1];
        dst[i + 1] = src[i];
        dst[i + 2] = src[i + 3];
        dst[i + 3] = src[i + 2];
    }
}

static unsigned build_upload_frame(uint8_t *buf, uint32_t dest,
                                   const uint8_t *src, unsigned len)
{
    uint16_t hdr_sum;
    uint32_t payload_sum;

    buf[0] = 0x18;
    buf[1] = 0xe1;
    buf[2] = 0x30;
    buf[3] = 0x01;

    /* Word count as big-endian u16, expressed straight from the length. */
    buf[4] = (uint8_t)((len >> 10) & 0xff);
    buf[5] = (uint8_t)((len >> 2) & 0xff);

    buf[6] = (uint8_t)((dest >> 8) & 0xff);
    buf[7] = (uint8_t)(dest & 0xff);
    buf[8] = (uint8_t)((dest >> 24) & 0xff);
    buf[9] = (uint8_t)((dest >> 16) & 0xff);

    hdr_sum = sum16(buf + 4, 6);
    buf[10] = (uint8_t)(hdr_sum >> 8);
    buf[11] = (uint8_t)hdr_sum;

    swizzle32(buf + 12, src, len);

    payload_sum = sum32(buf + 12, len);
    buf[12 + len]     = (uint8_t)((payload_sum >> 8) & 0xff);
    buf[12 + len + 1] = (uint8_t)(payload_sum & 0xff);
    buf[12 + len + 2] = (uint8_t)((payload_sum >> 24) & 0xff);
    buf[12 + len + 3] = (uint8_t)((payload_sum >> 16) & 0xff);

    return len + HBPP_HDR_LEN;
}

/*
 * Known-good prefixes, straight off the stock uploads. Checked rather than
 * assumed, because a swizzle or checksum mistake produces a frame that is
 * structurally plausible and simply never gets acknowledged -- which is a
 * miserable thing to debug from the ACK timeout alone.
 */
static bool prefix_is_expected(uint32_t dest, unsigned len, const uint8_t *tx)
{
    static const uint8_t fw0[12] = {
        0x18, 0xe1, 0x30, 0x01, 0x07, 0xfc, 0x00, 0x00,
        0x00, 0x00, 0x01, 0x03
    };
    static const uint8_t cal0[12] = {
        0x18, 0xe1, 0x30, 0x01, 0x00, 0x80, 0x02, 0x00,
        0x00, 0x40, 0x00, 0xc2
    };

    if (dest == 0 && len == HBPP_CHUNK_MAX)
        return memcmp(tx, fw0, sizeof(fw0)) == 0;

    if (dest == CAL_DEST && len == CAL_LEN)
        return memcmp(tx, cal0, sizeof(cal0)) == 0;

    return true;    /* no oracle for this shape; nothing to check against */
}

/* Status read: 1A A1, reply is a byte-swapped 16-bit word. */
static int hbpp_status(uint16_t *status)
{
    uint8_t tx[2] = { 0x1a, 0xa1 };
    uint8_t rx[2] = { 0 };

    if (spi_transfer(SPI_PORT_TOUCH, tx, rx, sizeof(tx)) < 0)
        return -1;

    if (status)
        *status = (uint16_t)((rx[0] << 8) | rx[1]);
    return 0;
}

static int hbpp_wait_ack(uint16_t expect, int retries)
{
    int i;

    for (i = 0; i < retries; i++) {
        uint16_t st = 0;

        if (hbpp_status(&st) == 0 && st == expect)
            return 0;
        sleep(HZ / 500 + 1);
    }

    return -1;
}

static int hbpp_send_chunk(uint32_t dest, const uint8_t *src, unsigned len)
{
    unsigned xfer;
    int try;

    xfer = build_upload_frame(hbpp_tx, dest, src, len);

    if (!prefix_is_expected(dest, len, hbpp_tx))
        return -2;      /* frame is malformed; do not put it on the wire */

    for (try = 0; try < 5; try++) {
        /*
         * The whole chunk is one CS-framed burst. HBPP frames on chip select,
         * so letting CS drop mid-chunk splits one frame into several and the
         * part simply stops acknowledging.
         */
        if (spi_transfer_cs(SPI_PORT_TOUCH, hbpp_tx, hbpp_rx, xfer, true) < 0)
            continue;

        if (hbpp_wait_ack(HBPP_ACK_CHUNK, 8) == 0)
            return 0;
    }

    return -1;
}

/* EXEC: 1D 53 plus two little-endian words and a checksum. */
static int hbpp_exec(uint32_t word0, uint32_t word1)
{
    uint8_t tx[12] = { 0x1d, 0x53 };
    uint8_t rx[12] = { 0 };
    uint16_t csum;

    tx[2] = (uint8_t)word0;
    tx[3] = (uint8_t)(word0 >> 8);
    tx[4] = (uint8_t)(word0 >> 16);
    tx[5] = (uint8_t)(word0 >> 24);
    tx[6] = (uint8_t)word1;
    tx[7] = (uint8_t)(word1 >> 8);
    tx[8] = (uint8_t)(word1 >> 16);
    tx[9] = (uint8_t)(word1 >> 24);

    csum = sum16(tx + 2, 8);
    tx[10] = (uint8_t)(csum >> 8);
    tx[11] = (uint8_t)csum;

    return spi_transfer(SPI_PORT_TOUCH, tx, rx, sizeof(tx));
}

/* Upload one file in chunks, starting at `dest`. */
static int hbpp_upload_file(const char *path, uint32_t dest, unsigned chunk_max)
{
    int fd;
    uint32_t at = dest;
    int total = 0;

    fd = open(path, O_RDONLY);
    if (fd < 0)
        return -1;

    while (1) {
        ssize_t got = read(fd, hbpp_src, chunk_max);

        if (got <= 0)
            break;

        /*
         * Chunks are padded to a whole number of 32-bit words: the swizzle
         * and the checksum both work in words, and a ragged tail would be
         * silently dropped by the swizzle loop.
         */
        while (got & 3)
            hbpp_src[got++] = 0;

        if (hbpp_send_chunk(at, hbpp_src, (unsigned)got)) {
            close(fd);
            return -1;
        }

        at += (uint32_t)got;
        total += (int)got;
    }

    close(fd);
    return total ? 0 : -1;
}

/*
 * Full bring-up: enter the bootloader, push the ARM image, push calibration,
 * then run.
 *
 * Returns 0 on success. A missing firmware file is reported distinctly from a
 * protocol failure, because the two mean very different things -- one is a
 * packaging problem, the other is a bus or format problem.
 */
int touch_hbpp_load(void)
{
    uint8_t enter[2];

    /* Bootload entry: opcode 0x19C1, big-endian on the wire. */
    enter[0] = (uint8_t)(HBPP_ENTER >> 8);
    enter[1] = (uint8_t)HBPP_ENTER;

    if (spi_transfer(SPI_PORT_TOUCH, enter, NULL, sizeof(enter)) < 0)
        return TOUCH_HBPP_ERR_BUS;

    sleep(HZ / 50);

    if (hbpp_upload_file(FW_PATH, 0, HBPP_CHUNK_MAX)) {
        /* Distinguish "no file" from "upload rejected". */
        int fd = open(FW_PATH, O_RDONLY);

        if (fd < 0)
            return TOUCH_HBPP_ERR_NOFILE;
        close(fd);
        return TOUCH_HBPP_ERR_UPLOAD;
    }

    /*
     * Calibration is per-device and lives beside the firmware. It is optional
     * here: without it the controller still runs, it is just not calibrated,
     * which is a far better outcome than refusing to bring touch up at all.
     */
    hbpp_upload_file(CAL_PATH, CAL_DEST, CAL_LEN);

    if (hbpp_exec(EXEC_WORD0, EXEC_WORD1) < 0)
        return TOUCH_HBPP_ERR_EXEC;

    /*
     * The part stops driving MISO from EXEC onwards and needs time to start
     * its application. 40 ms is the stock wait, not a guess.
     */
    sleep((HZ * EXEC_SETTLE_MS) / 1000 + 1);

    return TOUCH_HBPP_OK;
}
