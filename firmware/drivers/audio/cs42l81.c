/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Cirrus CS42L81 / Apple 338S1146 codec for the iPod nano 7G (N31).
 *
 * Built primarily from the RetailOS reverse engineering and live captures in
 * docs-internal/n7g-audio/, NOT from the Linux driver -- the Linux
 * implementation is still silent for reasons that are not understood, so
 * copying its structure would risk copying the fault.
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
#include "audio.h"
#include "audiohw.h"
#include "pcm_sampr.h"
#include "spi-s5l8740.h"
#include "iis-s5l8740.h"
#include "pmu-target.h"
#include "cs42l81.h"

/*
 * ---------------------------------------------------------------------
 * Why this is structured the way it is
 *
 * The RE guide's diagnosis of the Linux silence names four candidate causes.
 * Three of them are ORDERING problems rather than missing register writes,
 * so this driver is built around getting the order right:
 *
 *  1. The ASP lock handshake must run AFTER IIS BCLK/LRCLK are already
 *     running. The codec has a loss-of-signal state at 0x002F bit 6 and will
 *     not lock against a dead clock. A driver that does its whole init at
 *     probe time -- before any PCM has started -- can complete every write
 *     successfully and still never lock. That is why cs42l81_asp_lock() is a
 *     separate entry point called from the PCM start path, not from init.
 *
 *  2. A zero software volume silently undoes the unmute. audiohw_postinit()
 *     ends by applying volume, so an earlier 0x0527 = 0x60 gets overwritten
 *     with 0xFF if the volume state starts at zero. This driver initialises
 *     its volume to 0 dB and applies mute state explicitly and last.
 *
 *  3. The IIS divider parent is not frozen. That belongs to pcm-s5l8740.c;
 *     this driver takes the rate it is given and programs the codec's own
 *     serial control to match, so at least the two halves agree.
 *
 * The fourth (DMA vs PIO start) is a transport question and lives in the PCM
 * driver.
 *
 * The register sequences below are transcribed from the RetailOS oracle in
 * docs-internal/n7g-audio/N31-Audio-RetailOS-Linux-Driver-Guide-v1.md, which
 * is the highest-confidence source available: it is what the stock firmware
 * does while actually playing music.
 * ---------------------------------------------------------------------
 */

/* Registers, named only where the RE actually establishes a meaning. */
#define CS42_R0006          0x0006
#define CS42_R0007          0x0007
#define CS42_R0008          0x0008
#define CS42_R0009          0x0009
#define CS42_R000B          0x000b
#define CS42_R000E          0x000e  /* ASP clock control, bits 7:6 */
#define CS42_R000F          0x000f  /* ASP clock control, low nibble */
#define CS42_R002F          0x002f  /* bit 6: ASP locked */
#define CS42_R0073          0x0073
#define CS42_R0075          0x0075
#define CS42_R0079          0x0079
#define CS42_R010B          0x010b
#define CS42_R010C          0x010c
#define CS42_R0121          0x0121  /* SRC ratio pair */
#define CS42_R0122          0x0122
#define CS42_R0130          0x0130  /* SRC rate code */
#define CS42_R012F          0x012f
#define CS42_R0131          0x0131
#define CS42_R0219          0x0219
#define CS42_R0220          0x0220  /* bit 5: ASP enable; bit 6: sense */
#define CS42_R0227          0x0227
#define CS42_R0528          0x0528  /* mixer readback, 570620/5707D8 */
#define CS42_R0400          0x0400  /* mixer front controls */
#define CS42_R0401          0x0401  /* low 2 bits: output route/mute */
#define CS42_R0403          0x0403  /* playback tap selection */
#define CS42_R0404          0x0404
#define CS42_R0500          0x0500
#define CS42_R051E          0x051e
#define CS42_R0523          0x0523
#define CS42_R0527          0x0527  /* master output attenuation */
#define CS42_R0529          0x0529
#define CS42_R052A          0x052a
#define CS42_R0533          0x0533
#define CS42_R0534          0x0534
#define CS42_R054F          0x054f
#define CS42_R9901          0x9901
#define CS42_RC81F          0xc81f
#define CS42_RC85F          0xc85f
#define CS42_RC96F          0xc96f  /* power / backpower */

#define CS42_UNMUTE_ATTEN   0x60
#define CS42_MUTE_ATTEN     0xff

static bool cs42_up;
static bool cs42_muted = true;
static int  cs42_vol_l = 0, cs42_vol_r = 0;   /* in codec attenuation steps */
static bool cs42_asp_locked;
static unsigned int cs42_rate = 44100;
static uint8_t cs42_headset_type;   /* 0x000B & 3; telemetry only */

/* ------------------------------------------------------------------ SPI */

/*
 * Write frame is five bytes: 0x6C, register high, register low, 0x00, data.
 * The leading 0x6C is a fixed command byte, not an address.
 */
static void cs42_write(uint16_t reg, uint8_t val)
{
    uint8_t tx[5] = { 0x6c, (uint8_t)(reg >> 8), (uint8_t)reg, 0x00, val };

    spi_transfer(SPI_PORT_CODEC, tx, NULL, sizeof(tx));
}

/*
 * Read frame: 0x6D, register high, register low, 0x00, dummy. The returned
 * byte is the fifth of the reply.
 *
 * Reads matter here beyond convenience: the ASP lock state at 0x002F bit 6 is
 * only observable this way, so with reads the lock becomes a measurement
 * rather than an assumption.
 */
static uint8_t cs42_read(uint16_t reg)
{
    uint8_t tx[5] = { 0x6d, (uint8_t)(reg >> 8), (uint8_t)reg, 0x00, 0xff };
    uint8_t rx[5] = { 0 };

    if (spi_transfer(SPI_PORT_CODEC, tx, rx, sizeof(tx)) < 0)
        return 0;

    return rx[4];
}

/*
 * A write-through shadow is still kept, but only so the debug screen can show
 * the route without extra bus traffic. Read-modify-write goes to the device,
 * because the codec changes some of these bits by itself and an RMW built on
 * a stale shadow would quietly undo that.
 */
#define SHADOW_MAX  0x600
static uint8_t shadow[SHADOW_MAX];

static void cs42_set(uint16_t reg, uint8_t val)
{
    if (reg < SHADOW_MAX)
        shadow[reg] = val;
    cs42_write(reg, val);
}

static void cs42_rmw(uint16_t reg, uint8_t mask, uint8_t val)
{
    uint8_t cur = cs42_read(reg);

    cur = (uint8_t)((cur & ~mask) | (val & mask));
    cs42_set(reg, cur);
}

/* ------------------------------------------------------------ sequences */

/*
 * One-time bring-up. The 0x9901 A5/00 pair is an unlock-like operation and
 * the RE is explicit that it must NOT be re-run after mixer programming.
 */
static void cs42_init_once(void)
{
    /*
     * The stock sequence reads 0x0227 before touching anything, and the
     * result is never used. Kept anyway: a read is a bus transaction the
     * part sees, and dropping "useless" reads is a good way to skip a
     * handshake or leave a latch set. Cheap to keep, expensive to debug.
     */
    (void)cs42_read(CS42_R0227);

    cs42_set(CS42_RC96F, 0x0e);
    cs42_set(CS42_R9901, 0xa5);
    cs42_set(CS42_R9901, 0x00);
    cs42_set(CS42_RC81F, 0xff);
    cs42_set(CS42_RC85F, 0x0f);
}

/*
 * 2.5 V backpower transition. The 100 ms wait is from the stock sequence and
 * is not a guess -- the rail has to come up before 0xC96F is advanced.
 */
static void cs42_backpower(void)
{
    cs42_rmw(CS42_R0219, 0x07, 0x01);
    sleep(HZ / 10);
    cs42_set(CS42_RC96F, 0x1e);
    cs42_set(CS42_R0227, 0x40);
}

/*
 * Serial/clock control for a given sample rate (sub_183138).
 *
 * The 0x000E bits 7:6 -> 11, program, then -> 01 bracket is how the stock code
 * applies a clock change: the field is parked while the rate registers are
 * written, then committed.
 *
 * Almost none of this is constant across rates, which is easy to miss because
 * at 48 kHz the rate code is 12 = 0xC and the values come out looking like
 * the fixed 0x0C / 0xCC that the 48 kHz sequence documents:
 *
 *   0x000F low nibble = code
 *   0x012F            = code | (code << 4)
 *
 * And the codec takes one of two arms depending on whether its sample-rate
 * converter is needed:
 *
 *   code == 12 (48 kHz)   0x10B/0x10C = 8/9, 0x131 bit0 = 1   (direct)
 *   otherwise             0x121/0x122 = 8/9, 0x130 low = code,
 *                         0x131 bit0 = 0, 0x10B/0x10C = 4/0x33  (SRC)
 *
 * Taking the 48 kHz arm while the IIS divider runs 44.1 kHz is an ASP/SRC
 * mismatch, and it does not fail quietly -- it produces pulsed noise. The
 * Linux driver shipped that bug and it is worth not repeating: Rockbox's
 * default rate is 44.1 kHz, so the wrong arm would be the normal case.
 */
static void cs42_serial_setup(unsigned int rate)
{
    uint8_t code = iis_codec_rate_code(rate);

    cs42_rmw(CS42_R000E, 0xc0, 0xc0);   /* park */

    cs42_rmw(CS42_R000F, 0x0f, code);
    cs42_set(CS42_R012F, (uint8_t)(code | (code << 4)));

    if (code == 12) {
        /* 48 kHz: the DAC clocks straight off the ASP. */
        cs42_set(CS42_R010B, 0x08);
        cs42_set(CS42_R010C, 0x09);
        cs42_rmw(CS42_R0131, 0x01, 0x01);
    } else {
        /* Everything else runs through the sample-rate converter. */
        cs42_set(CS42_R0121, 0x08);
        cs42_set(CS42_R0122, 0x09);
        cs42_rmw(CS42_R0130, 0x0f, code);
        cs42_rmw(CS42_R0131, 0x01, 0x00);
        cs42_set(CS42_R010B, 0x04);
        cs42_set(CS42_R010C, 0x33);
    }

    cs42_rmw(CS42_R000E, 0xc0, 0x40);   /* commit */
    cs42_rmw(CS42_R0220, 0x20, 0x20);   /* ASP enable */

    cs42_rate = rate;
}

/*
 * The mixer/routing image from sub_5707D8.
 *
 * From 0x407 onward this is 22 three-byte records of (source, gain_hi,
 * gain_lo). Sources 0, 1, 0x0A and 0x0B carry gain 0x01E0 and everything else
 * is zero -- that is the working music graph, blasted as a fixed image.
 *
 * RE checkpoint 011 is worth reading before "improving" this: the dynamic
 * graph path that RetailOS can also take computes gain 0 for every source,
 * because the table it indexes has no ROM initialiser. Zero gain there is
 * RE-accurate rather than a bug, and the static image below is what music
 * actually uses.
 */
static const uint8_t cs42_mixer_image[] = {
    /* 0x0400 */ 0x04, 0x10, 0x00, 0x09, 0x08, 0x00, 0x00,
    /* 0x0407: 22 x (source, gain_hi, gain_lo) */
    0x00, 0x01, 0xe0,
    0x01, 0x01, 0xe0,
    0xfe, 0x00, 0xa0,
    0x02, 0x00, 0x00,
    0x03, 0x00, 0x00,
    0x04, 0x00, 0x00,
    0x05, 0x00, 0x00,
    0x06, 0x00, 0x00,
    0x07, 0x00, 0x00,
    0x08, 0x00, 0x00,
    0x09, 0x00, 0x00,
    0x0a, 0x01, 0xe0,
    0x0b, 0x01, 0xe0,
    0xff, 0x00, 0xa0,
    0x0c, 0x00, 0x00,
    0x0d, 0x00, 0x00,
    0x0e, 0x00, 0x00,
    0x0f, 0x00, 0x00,
    0x10, 0x00, 0x00,
    0x11, 0x00, 0x00,
    0x12, 0x00, 0x00,
    0x13, 0x00, 0x00,
};

static void cs42_mixer_setup(void)
{
    unsigned i;

    /* Pre-blast controls. */
    cs42_set(CS42_R0006, 0x24);
    cs42_set(CS42_R0529, 0x2c);
    cs42_set(CS42_R052A, 0x2c);
    cs42_set(CS42_R0533, 0x2c);
    cs42_set(CS42_R0534, 0x2c);

    for (i = 0; i < sizeof(cs42_mixer_image); i++)
        cs42_set((uint16_t)(0x0400 + i), cs42_mixer_image[i]);

    /*
     * Front controls are overridden after the blast. 0x403 = 2 / 0x404 = 1 is
     * the playback tap selection the stock code lands on.
     */
    cs42_set(CS42_R0400, 0x04);
    cs42_set(CS42_R0401, 0x12);
    cs42_set(0x0402, 0x00);
    cs42_set(CS42_R0403, 0x02);
    cs42_set(CS42_R0404, 0x01);
    cs42_set(0x0405, 0x00);
    cs42_set(0x0406, 0x00);

    /* Both 5707D8 and 570620 read 0x0528 back at this point. */
    (void)cs42_read(CS42_R0528);
}

/* Output path enable, after the mixer is programmed. */
static void cs42_output_enable(void)
{
    sleep(HZ / 10);

    cs42_set(CS42_R0500, 0x05);
    cs42_set(CS42_R0527, CS42_UNMUTE_ATTEN);
    cs42_rmw(CS42_R0075, 0x3f, 0x3c);
    cs42_rmw(CS42_R054F, 0xf0, 0x00);
    cs42_rmw(CS42_R0220, 0x28, 0x28);

    /* Output preparation. */
    cs42_rmw(CS42_R0007, 0x40, 0x00);
    cs42_rmw(CS42_R0006, 0x40, 0x00);
    cs42_rmw(CS42_R0220, 0x28, 0x28);
    cs42_rmw(CS42_R000F, 0x80, 0x80);
    cs42_rmw(CS42_R0075, 0x40, 0x40);
}

/*
 * Headset sense. The stock code classifies the plugged accessory here.
 *
 * Deliberately does not gate playback on the result: the RE guide is explicit
 * that an unexpected headset type must not block output, and forcing the known
 * headphone route is the right behaviour until sound works at all.
 */
static void cs42_headset_sense(void)
{
    uint8_t old220;
    int i;

    cs42_rmw(CS42_R0073, 0xc3, 0x00);
    cs42_rmw(CS42_R0073, 0xc0, 0xc0);
    cs42_rmw(CS42_R0079, 0x60, 0x00);

    /*
     * Temporary detection mode. The whole block was previously missing --
     * only the three RMWs above were done -- on the reasoning that we force
     * the headphone route anyway and do not need the answer.
     *
     * That reasoning is wrong. This is a sequence the codec runs, not a query
     * we opt into: it enters detection mode, waits for the comparator, reads
     * the result, and leaves detection mode again. Skipping it leaves 0x0220
     * bit 6 and 0x0009 in whatever state they happened to be in, which is not
     * the state the rest of the stock sequence assumes.
     *
     * The result is still deliberately not acted on -- an unexpected headset
     * type must never block playback -- but the sequence runs.
     */
    old220 = cs42_read(CS42_R0220);

    cs42_rmw(CS42_R0220, 0x40, 0x40);
    udelay(1000);

    cs42_rmw(CS42_R0009, 0xc0, 0xc0);

    for (i = 0; i < 3; i++) {
        udelay(1000);
        if (cs42_read(CS42_R002F) & 0x40)
            break;
    }

    /* headset type = 0x000B & 3; recorded, not enforced. */
    cs42_headset_type = cs42_read(CS42_R000B) & 3;

    cs42_rmw(CS42_R0009, 0xc0, 0x80);
    cs42_rmw(CS42_R0220, 0x40, old220 & 0x40);

    /* Two more reads the stock path ends on, results unused. */
    (void)cs42_read(CS42_R0008);
    (void)cs42_read(CS42_R0009);
}

/* -------------------------------------------------------------- public */

void audiohw_preinit(void)
{
    memset(shadow, 0, sizeof(shadow));

    spi_port_init(SPI_PORT_CODEC);

    /*
     * The analog rails are PMIC-side. During the Linux bring-up these read
     * back as 0x00 while a tone was supposedly playing, which is one of the
     * strongest clues about the silence -- so they go up first and
     * explicitly, before any codec register is touched.
     */
    pmu_audio_rails(true);
    sleep(HZ / 20);

    cs42_init_once();
    cs42_backpower();
    cs42_serial_setup(cs42_rate);
    cs42_mixer_setup();
    cs42_output_enable();
    cs42_headset_sense();

    cs42_up = true;
}

void audiohw_postinit(void)
{
    /*
     * Volume is applied LAST and from an explicitly non-zero default.
     *
     * The Linux path ends through the user-volume/mute code, so a volume
     * state that starts at zero rewrites 0x0527 from 0x60 to 0xFF and mutes
     * everything that was just carefully unmuted. Starting at 0 dB and
     * unmuting here makes that failure impossible rather than unlikely.
     */
    cs42_muted = false;
    audiohw_set_volume(0, 0);
}

void audiohw_close(void)
{
    if (!cs42_up)
        return;

    cs42_set(CS42_R0527, CS42_MUTE_ATTEN);
    cs42_rmw(CS42_R0401, 0x03, 0x01);
    pmu_audio_rails(false);
    cs42_up = false;
    cs42_asp_locked = false;
}

/*
 * ASP lock, to be called ONLY after IIS BCLK/LRCLK are running.
 *
 * This is the step the RE guide identifies as the most likely cause of the
 * Linux silence: the codec will not lock its serial port against a dead
 * clock, so running it at probe time completes every write and achieves
 * nothing.
 *
 * The lock state is genuinely observable at 0x002F bit 6, so this polls it
 * rather than assuming success. On failure the stock recovery is to pulse ASP
 * enable off and on and reprogram the clock registers, which is what the
 * retry loop does.
 */
void cs42l81_asp_lock(void)
{
    int attempt;

    if (!cs42_up)
        return;

    for (attempt = 0; attempt < 3; attempt++) {
        int poll;

        if (attempt > 0) {
            cs42_rmw(CS42_R0220, 0x20, 0x00);
            udelay(50);
            cs42_rmw(CS42_R0220, 0x20, 0x20);

            /* Reprogram for the rate actually in use, not a fixed 48 kHz. */
            cs42_serial_setup(cs42_rate);
        }

        for (poll = 0; poll < 3; poll++) {
            udelay(1000);
            if (cs42_read(CS42_R002F) & 0x40) {
                cs42_asp_locked = true;
                goto locked;
            }
        }
    }

    /*
     * Three full recovery attempts and the port never locked. Say so through
     * the flag rather than reporting success -- the debug screen surfaces it,
     * and silence with asp_locked=0 is a very different bug from silence with
     * asp_locked=1.
     */
    cs42_asp_locked = false;
    return;

locked:
    /* Re-assert the playback route; the pulse above can drop it. */
    cs42_set(CS42_R0527, cs42_muted ? CS42_MUTE_ATTEN : CS42_UNMUTE_ATTEN);
    cs42_rmw(CS42_R0401, 0x03, cs42_muted ? 0x01 : 0x02);
}

bool cs42l81_asp_is_locked(void)
{
    return cs42_asp_locked;
}

void audiohw_set_frequency(int fsel)
{
    unsigned int rate;

    if (fsel < 0 || fsel >= HW_NUM_FREQ)
        return;

    /*
     * The codec's rate code comes from the same resolver the IIS divider
     * uses, so the two halves of the link cannot end up configured for
     * different rates.
     */
    rate = hw_freq_sampr[fsel];

    if (!cs42_up) {
        cs42_rate = rate;
        return;
    }

    cs42_serial_setup(rate);

    /*
     * A rate change re-parks and re-commits the ASP, so whatever lock existed
     * is no longer trustworthy. Drop the flag and let the next play re-lock
     * against the new clock.
     */
    cs42_asp_locked = false;
}

void audiohw_set_volume(int vol_l, int vol_r)
{
    cs42_vol_l = vol_l;
    cs42_vol_r = vol_r;

    if (!cs42_up)
        return;

    /*
     * 0x0527 is a master attenuation: 0x60 is the stock playback level and
     * 0xFF is silence. The RE corpus does not establish the step size or the
     * usable range, so rather than invent a mapping this treats the control
     * as unmute/mute and leaves fine volume to the software mixer.
     *
     * HAVE_SW_VOLUME_CONTROL is enabled for the target for exactly this
     * reason.
     */
    cs42_set(CS42_R0527, cs42_muted ? CS42_MUTE_ATTEN : CS42_UNMUTE_ATTEN);
    cs42_rmw(CS42_R0401, 0x03, cs42_muted ? 0x01 : 0x02);
}

void audiohw_mute(bool mute)
{
    cs42_muted = mute;

    if (!cs42_up)
        return;

    cs42_set(CS42_R0527, mute ? CS42_MUTE_ATTEN : CS42_UNMUTE_ATTEN);
    cs42_rmw(CS42_R0401, 0x03, mute ? 0x01 : 0x02);

    if (mute) {
        /* Soft-ramp pulses, from the stock mute path. */
        cs42_rmw(CS42_R051E, 0x20, 0x20);
        cs42_rmw(CS42_R051E, 0x20, 0x00);
        cs42_rmw(CS42_R0523, 0x20, 0x20);
        cs42_rmw(CS42_R0523, 0x20, 0x00);
    }
}

/* Debug readback of the route registers the RE guide calls out. */
void cs42l81_get_route(struct cs42l81_route *r)
{
    if (!r)
        return;

    /* Read the device rather than the shadow: the point of this screen is
       to show what the codec actually thinks, not what we told it. */
    r->r0401 = cs42_read(CS42_R0401);
    r->r0403 = cs42_read(CS42_R0403);
    r->r0404 = cs42_read(CS42_R0404);
    r->r0500 = cs42_read(CS42_R0500);
    r->r0527 = cs42_read(CS42_R0527);
    r->r054f = cs42_read(CS42_R054F);
    r->r0075 = cs42_read(CS42_R0075);
    r->r0220 = cs42_read(CS42_R0220);
    r->r002f = cs42_read(CS42_R002F);
    r->headset_type = cs42_headset_type;
    r->locked = cs42_asp_locked;
}
