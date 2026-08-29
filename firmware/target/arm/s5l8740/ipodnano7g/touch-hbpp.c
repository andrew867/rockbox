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

/*
 * Calibration is per-device and does NOT come from a file.
 *
 * It arrives through the A34 IsyS handoff: a 0x560-byte SysCfg object whose
 * magic is "IsyS", with the 512-byte calibration window at decimal offset
 * 350. U-Boot copies that object into reserved DRAM -- the same page the
 * Linux device tree reserves as n31-isys@9dff000 -- so it is readable here
 * without a device tree of our own.
 *
 * The RE is explicit that the alternatives are all wrong: there is no
 * grape-nimbus-cal.bin, no FTL IsyS scan, and it is NOT GrapeFirmware.bin
 * +350. Shipping a cal file was this driver's own invention.
 */
#define ISYS_ADDR           0x09DFF000u
#define ISYS_MAGIC          0x53797349u     /* "IsyS" */
#define ISYS_LEN            0x560u
#define ISYS_CAL_OFF        350u

static bool cal_loaded;

/* One chunk plus its envelope. */
static uint8_t hbpp_tx[HBPP_CHUNK_MAX + HBPP_HDR_LEN];
static uint8_t hbpp_rx[HBPP_CHUNK_MAX + HBPP_HDR_LEN];
static uint8_t hbpp_src[HBPP_CHUNK_MAX];

/*
 * Every 32-bit word byte-reversed (sub_273A0). Applied to the calibration
 * window before it is packetised -- which is a different transform from the
 * B1 B0 B3 B2 wire swizzle every DATA packet gets, and both are needed.
 */
static void bswap32_words(uint8_t *p, unsigned len)
{
    unsigned i;

    for (i = 0; i + 3 < len; i += 4) {
        uint8_t t0 = p[i], t1 = p[i + 1];

        p[i]     = p[i + 3];
        p[i + 1] = p[i + 2];
        p[i + 2] = t1;
        p[i + 3] = t0;
    }
}

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

/*
 * MT_SPI_Z2_WAKE (opcode 0xEE). A 16-byte frame whose reply is discarded.
 *
 * It is sent for its effect, not its answer: the part is asleep until it
 * sees this, and a download issued to a sleeping controller goes nowhere.
 */
static void hbpp_wake(void)
{
    uint8_t tx[16] = { 0 };
    uint8_t rx[16] = { 0 };

    tx[0] = 0xee;
    tx[14] = 0xee;

    spi_transfer(SPI_PORT_TOUCH, tx, rx, sizeof(tx));
}

/*
 * Post-download probe (sub_26494, reached from sub_20E94): a 16-byte 1A A1
 * frame padded with 18 E1 pairs. The stock code rejects the part only when
 * the transfer SUCCEEDS and returns two words it does not recognise -- a
 * failed transfer is not a verdict.
 *
 * Returns false only for that specific "answered with nonsense" case.
 */
static bool opcode_known(uint16_t w)
{
    return w == 0x18e1 || w == 0x1aa1 || w == 0x1f01 || w == 0x19c1 ||
           w == 0x4879 || w == 0x4bc1 || w == 0x4969 || w == 0x4ad1;
}

static bool hbpp_probe_26494(void)
{
    uint8_t tx[16];
    uint8_t rx[16] = { 0 };
    unsigned i;
    uint16_t w0, w1;

    tx[0] = 0x1a;
    tx[1] = 0xa1;
    for (i = 2; i < sizeof(tx); i += 2) {
        tx[i] = 0x18;
        tx[i + 1] = 0xe1;
    }

    if (spi_transfer(SPI_PORT_TOUCH, tx, rx, sizeof(tx)) < 0)
        return true;    /* transfer trouble is not a verdict on the part */

    w0 = (uint16_t)((rx[0] << 8) | rx[1]);
    w1 = (uint16_t)((rx[2] << 8) | rx[3]);

    return opcode_known(w0) && opcode_known(w1);
}

/*
 * Upload the per-device calibration window out of the IsyS object U-Boot
 * left in reserved DRAM. Optional: without it the controller still runs, it
 * is simply uncalibrated, and that beats refusing to bring touch up.
 */
static bool hbpp_upload_cal(void)
{
    const uint32_t *isys = (const uint32_t *)ISYS_ADDR;
    static uint8_t cal[CAL_LEN];

    if (isys[0] != ISYS_MAGIC)
        return false;   /* no handoff from the previous stage */

    memcpy(cal, (const uint8_t *)ISYS_ADDR + ISYS_CAL_OFF, CAL_LEN);
    bswap32_words(cal, CAL_LEN);

    return hbpp_send_chunk(CAL_DEST, cal, CAL_LEN) == 0;
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

    /* Wake the part before anything else; a sleeping controller ignores us. */
    hbpp_wake();
    sleep(HZ / 100);

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

    /* Per-device calibration, from the IsyS handoff. Optional. */
    cal_loaded = hbpp_upload_cal();

    /*
     * sub_20E94: probe after the download and before EXEC. The result is
     * checked but a bus problem is not treated as a verdict -- the stock code
     * only gives up when the part answers with words it does not recognise.
     */
    if (!hbpp_probe_26494())
        return TOUCH_HBPP_ERR_UPLOAD;

    if (hbpp_exec(EXEC_WORD0, EXEC_WORD1) < 0)
        return TOUCH_HBPP_ERR_EXEC;

    /*
     * The part stops driving MISO from EXEC onwards and needs time to start
     * its application. 40 ms is the stock wait, not a guess.
     */
    sleep((HZ * EXEC_SETTLE_MS) / 1000 + 1);

    return TOUCH_HBPP_OK;
}

bool touch_cal_loaded(void)
{
    return cal_loaded;
}
