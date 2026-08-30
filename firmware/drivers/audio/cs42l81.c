/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Cirrus CS42L81 / Apple 338S1146 codec for the iPod nano 7G (N31).
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
 * Provenance, and why this was rewritten
 *
 * The first version of this driver was written from per-function RE extracts
 * and from the N31 bootloader's sequences, deliberately not from the Linux
 * driver, because Linux was silent at the time and copying its structure
 * risked copying the fault.
 *
 * Linux now produces sound, and it got there by decompiling the WHOLE image
 * rather than individual functions -- which turned up call sites the extracts
 * do not contain. Three things this driver believed turned out to be wrong,
 * and all three came from reading a function without its callers:
 *
 *  1. There is no ASP lock handshake. 0x002F is read exactly once in the
 *     entire image -- the readiness poll in sub_D3280(1), testing bit 7. Bit
 *     6, which this driver polled in a retry loop, is never examined
 *     anywhere. The old cs42l81_asp_lock() spent up to two seconds per
 *     playback start waiting on a bit that means nothing, re-running the rate
 *     programming between attempts.
 *
 *  2. sub_D34C0 is a three-way branch, not a sequence. This driver ran the
 *     183138 body and then the long arm's tail on top of it, every time. They
 *     are alternatives that program DIFFERENT registers for the same job --
 *     183138 uses 0x010B/0x010C where the long arm uses 0x0223/0x0224 -- so
 *     the rate went into a block the current mode does not read, and the
 *     tail's closing 0x0220 bit-5 drop released a hold 183138 had just taken.
 *
 *  3. The analog power-up this driver ran is the bootloader's (sub_1310), not
 *     the shipping firmware's (sub_D3280(1)). They differ in five registers
 *     and, more importantly, in the final state of 0x0007 bit 6: the
 *     bootloader leaves it clear, OSOS leaves it SET and clears it again in
 *     state 3. Running the bootloader's ending means state 3 clears a bit
 *     nothing set, and whatever it gates never makes its transition.
 *
 * Codec headset detection is gone entirely; this board does not use jack
 * detect, and the reads that were here came from a sweep for
 * read-and-discard commands -- the right instinct applied to a function that
 * is not on the play path.
 *
 * ---------------------------------------------------------------------
 * The order, which matters more than any individual register
 *
 *   prepare:  rails -> D3280(1) analog on -> D3280(3) unfreeze ->
 *             mailbox reads -> set rate -> output path enable -> play graph
 *   start:    unmute, re-arm the graph
 *   stop:     42D364(0); D3280(4) standby only if asked
 *
 *   pre_iis:  0x000F bit 7 low, BEFORE the IIS clocks start
 *   post_iis: ASP status, unmute, apply volume, AFTER they are running
 *
 * Two ordering facts are easy to get backwards and both produce silence:
 *
 *   set_rate must run BEFORE output_path_enable. sub_183138 deliberately ends
 *   muted at -90 dB with the 0x0220 bit-5 hold raised; sub_D2F64 clears
 *   0x0220 mask 0x28 as its second write, so output_path_enable is what
 *   releases that hold. Reversing them leaves the part muted.
 *
 *   The unmute belongs in post_iis, after the interface is actually running.
 *
 * D3280(4) is a power-DOWN sequence. It clears the analog enable state 1
 * polled for, drops the 2v5 rail, writes 0x0225 = 0 over state 1's 0x33, and
 * restores the NATIVE rate pair over whatever the SRC was given. It ran as
 * the last step of every prepare in the old driver, which is a good way to
 * configure a codec perfectly and then switch it off.
 * ---------------------------------------------------------------------
 */

/* ------------------------------------------------------------ registers -- */

#define CS42_R0006      0x0006      /* analog enable / freeze latch        */
#define CS42_R0007      0x0007      /* clock gate companion                */
#define CS42_R000D      0x000d
#define CS42_R000E      0x000e      /* serial-port park bracket            */
#define CS42_R000F      0x000f      /* ASP rate code, bit 7 = ASP enable   */
#define CS42_R0075      0x0075
#define CS42_R0074      0x0074
#define CS42_R007B      0x007b
#define CS42_R007C      0x007c
#define CS42_R002F      0x002f      /* bit 7: analog ready. Bit 6 unused.  */
#define CS42_R010B      0x010b      /* native DAC clock pair (183138)      */
#define CS42_R010C      0x010c
#define CS42_R011F      0x011f
#define CS42_R0120      0x0120
#define CS42_R0121      0x0121      /* SRC input pair                      */
#define CS42_R0122      0x0122
#define CS42_R012E      0x012e
#define CS42_R012F      0x012f      /* rate code, both nibbles             */
#define CS42_R0130      0x0130      /* SRC rate                            */
#define CS42_R0131      0x0131      /* bit 0: native (1) vs SRC (0)        */
#define CS42_R0201      0x0201
#define CS42_R0203      0x0203
#define CS42_R0204      0x0204
#define CS42_R0205      0x0205
#define CS42_R0206      0x0206
#define CS42_R0207      0x0207
#define CS42_R0219      0x0219      /* 2v5 rail select, low three bits     */
#define CS42_R0220      0x0220      /* bits 5,3: idle/standby hold pair    */
#define CS42_R0222      0x0222
#define CS42_R0223      0x0223      /* long-arm DAC clock pair             */
#define CS42_R0224      0x0224
#define CS42_R0225      0x0225      /* analog output, wide write           */
#define CS42_R0227      0x0227      /* output gain, wide write             */
#define CS42_R0229      0x0229      /* analog output, wide write           */
#define CS42_R0400      0x0400
#define CS42_R0401      0x0401      /* bit 0 stop, bit 1 play              */
#define CS42_R0500      0x0500
#define CS42_R051E      0x051e
#define CS42_R051F      0x051f
#define CS42_R0520      0x0520
#define CS42_R0523      0x0523
#define CS42_R0524      0x0524
#define CS42_R0525      0x0525      /* mailbox read FIFO port              */
#define CS42_R0527      0x0527      /* 0x60 unmuted, 0xFF muted            */
#define CS42_R0528      0x0528
#define CS42_R9901      0x9901      /* one-shot unlock, sub_D2EFC          */
#define CS42_RC81F      0xc81f
#define CS42_RC85F      0xc85f
#define CS42_RC96F      0xc96f      /* 2v5 backpower latch                 */

/* Gain codes. 0x40 is -90 dB (full mute), 0x41 is -76 dB. */
#define CS42_GAIN_MUTE      64
#define CS42_GAIN_MIN       65

/* Cap on how much of the mailbox FIFO to drain if the level looks wild. */
#define CS42_MBOX_DRAIN_MAX 32

/* Analog-ready poll: stock's own bound, 50 attempts a millisecond apart. */
#define CS42_READY_POLLS    50

/* --------------------------------------------------------------- state -- */

static bool     cs42_up;
static bool     cs42_muted = true;
static int      cs42_user_vol = CS42L81_VOL_DB_DEFAULT;
static unsigned cs42_rate;              /* rate the part is programmed for */
static unsigned cs42_prepared_rate;     /* 0 = not configured              */
static uint8_t  cs42_mode38;            /* shadows MEMORY[0x892A038]       */
static bool     cs42_unlock_done;
static bool     cs42_graph_latched;
static bool     cs42_playing;
static uint8_t  cs42_status_528;

/* ----------------------------------------------------------- SPI access -- */

/*
 * Two frame widths, and which registers need which is not a style choice.
 *
 * 0x0225, 0x0227 and 0x0229 take a six-byte frame with the register's low
 * byte OR'd with 0x80, a 0x01 count, and the value sent twice. Those three
 * are exactly the analog output registers -- which is not a coincidence,
 * because the analog power-up is what writes them.
 */
static void cs42_write(uint16_t reg, uint8_t val)
{
    uint8_t tx[6];

    switch (reg) {
    case CS42_R0225:
    case CS42_R0227:
    case CS42_R0229:
        tx[0] = 0x6c;
        tx[1] = (reg >> 8) & 0xff;
        tx[2] = (reg & 0xff) | 0x80;
        tx[3] = 0x01;
        tx[4] = val;
        tx[5] = val;
        spi_transfer(SPI_PORT_CODEC, tx, NULL, 6);
        return;
    default:
        tx[0] = 0x6c;
        tx[1] = (reg >> 8) & 0xff;
        tx[2] = reg & 0xff;
        tx[3] = 0x00;
        tx[4] = val;
        spi_transfer(SPI_PORT_CODEC, tx, NULL, 5);
        return;
    }
}

static uint8_t cs42_read(uint16_t reg)
{
    uint8_t tx[5], rx[5];

    tx[0] = 0x6d;
    tx[1] = (reg >> 8) & 0xff;
    tx[2] = reg & 0xff;
    tx[3] = 0x00;
    tx[4] = 0x00;

    memset(rx, 0, sizeof(rx));
    spi_transfer(SPI_PORT_CODEC, tx, rx, 5);
    return rx[4];
}

static void cs42_rmw(uint16_t reg, uint8_t mask, uint8_t val)
{
    uint8_t cur = cs42_read(reg);

    cs42_write(reg, (uint8_t)((cur & ~mask) | (val & mask)));
}

/* ---------------------------------------------------------- gain and dB -- */

/* sub_400330 sign-extends bit 6 only, then works with the signed result. */
static int cs42_gain_to_db(uint8_t raw)
{
    if (raw & 0x40)
        raw |= 0x80;
    return (int)(int8_t)raw;
}

/* sub_D2C98: dB to the signed code register 0x0227 carries. */
static int cs42_db_to_code(int db)
{
    if (db > CS42L81_VOL_DB_MAX)
        db = CS42L81_VOL_DB_MAX;
    if (db < CS42L81_VOL_DB_MIN)
        db = CS42L81_VOL_DB_MIN;
    if (db > CS42L81_VOL_DB_KNEE)
        return db;

    /*
     * Below the knee the register carries 2 dB per code, and sub_D2C98 steps
     * an odd figure down before halving rather than rounding.
     */
    if (db & 1)
        db--;
    return CS42L81_VOL_DB_KNEE + (db - CS42L81_VOL_DB_KNEE) / 2;
}

/*
 * Raise the 2.5 V analog backpower rail.
 *
 * This is the piece the first driver was missing entirely, and the symbols in
 * sub_400330 say what it is outright: gCS42L81_2v5Backpower_LastTimestamp,
 * sphwDACCS42L81.c. sub_400330 is not a volume setter -- it is a volume
 * setter wrapped around rail management, gated on the gain crossing -8 dB.
 *
 * Everything else can be right -- clocks present, DAC clocked, mixer powered,
 * unmuted -- and the jack stays silent if this rail was never raised. Writing
 * 0x0227 and nothing else, which is what the old driver did, leaves exactly
 * that state.
 *
 * The guard is stock's own: skip when 0xC96F already reads 0x1E and the low
 * three bits of 0x0219 already read 1.
 */
static void cs42_2v5_backpower_up(void)
{
    uint8_t r_c96f = cs42_read(CS42_RC96F);
    uint8_t r219 = cs42_read(CS42_R0219);

    if (r_c96f == 0x1e && (r219 & 0x07) == 0x01)
        return;

    cs42_write(CS42_RC96F, 0x0e);
    cs42_rmw(CS42_R0219, 0x07, 0x01);
    sleep(HZ / 20);             /* sub_345D58; a settle is all that fits */
    cs42_write(CS42_RC96F, 0x1e);
}

/*
 * The only place 0x0227 is written, so the rail threshold cannot be bypassed
 * by a caller that only wants a volume change.
 *
 * Going the other way stock starts a 601 ms timer and drops the rail when it
 * expires. Not implemented: dropping a rail we have only just learned to
 * raise buys nothing, and the idle current is not what is being optimised.
 */
static void cs42_set_output_gain(int code)
{
    int new_db = cs42_gain_to_db((uint8_t)(code & 0x7f));
    int old_db = cs42_gain_to_db(cs42_read(CS42_R0227));

    if (old_db < -8 && new_db >= -8)
        cs42_2v5_backpower_up();

    cs42_write(CS42_R0227, (uint8_t)(code & 0x7f));
}

static void cs42_apply_user_vol(void)
{
    if (cs42_muted)
        cs42_set_output_gain(CS42_GAIN_MUTE);
    else
        cs42_set_output_gain(cs42_db_to_code(cs42_user_vol));
}

/* --------------------------------------------------------- output path -- */

/*
 * sub_D2F64(271) -- the output-path "on" mode blast, reached via D2D2C(1).
 *
 * 0x000D is deliberately absent. sub_D2F64 writes it only when its v25 is
 * non-zero, and v25 is set solely in the (a1 & 0xC08) == 0 branch, which is
 * mode 6 and not mode 271. The old driver wrote 0x000D[1:0] = 0 in both, so
 * the "output on" path applied the "output off" path's value and held it
 * there for the whole of playback.
 *
 * 0x0204 is absent for a different reason: sub_42A5D6(516, v6, v32) with
 * v6 == 0 is a read-modify-write under an empty mask, i.e. nothing.
 */
static void cs42_apply_mode_271(void)
{
    cs42_rmw(CS42_R0006, 0x04, 0x00);
    cs42_rmw(CS42_R0220, 0x28, 0x00);
    cs42_mode38 = 0;                    /* v3 & 0x28, and v3 is 0 here */

    cs42_rmw(CS42_R0206, 0x3f, 0x3d);
    cs42_rmw(CS42_R0207, 0x3f, 0x3d);
    cs42_rmw(CS42_R0205, 0xff, 0x5a);
    cs42_rmw(CS42_R0206, 0xc0, 0x00);
    cs42_rmw(CS42_R0207, 0xc0, 0x00);
    cs42_rmw(CS42_R0203, 0xc0, 0x00);
    cs42_rmw(CS42_R000E, 0x40, 0x40);
    cs42_rmw(CS42_R011F, 0x3f, 0x1c);
    cs42_rmw(CS42_R0120, 0x3f, 0x1c);
    cs42_rmw(CS42_R012E, 0xff, 0xaa);
    cs42_rmw(CS42_R000E, 0x40, 0x00);
    sleep(HZ * 105 / 1000);
}

/*
 * sub_D2F64(6) -- the "off" companion.
 *
 * 0x0006 bit 2 is SET here and CLEARED in mode 271. sub_D2F64 computes it as
 * v31, which becomes 4 exactly when (a1 & 0x180) == 0 -- true for 6, false
 * for 271. The old driver drove it the same way in both.
 */
static void cs42_apply_mode_6(void)
{
    cs42_rmw(CS42_R0006, 0x04, 0x04);
    cs42_rmw(CS42_R0220, 0x28, 0x00);
    cs42_mode38 = 0;

    cs42_rmw(CS42_R000D, 0x03, 0x00);
    cs42_rmw(CS42_R0206, 0x3f, 0x34);
    cs42_rmw(CS42_R0207, 0x3f, 0x34);
    cs42_rmw(CS42_R0204, 0x03, 0x00);
    cs42_rmw(CS42_R0206, 0xc0, 0x00);
    cs42_rmw(CS42_R0207, 0xc0, 0x00);
    cs42_rmw(CS42_R0203, 0xc0, 0xc0);
    sleep(HZ * 60 / 1000);
}

/* sub_D2D2C(1). Also what releases the 0x0220 hold sub_183138 raised. */
static void cs42_output_path_enable(void)
{
    cs42_rmw(CS42_R0206, 0x3f, 0x08);
    cs42_rmw(CS42_R0207, 0x3f, 0x08);
    cs42_apply_mode_271();
}

/* ------------------------------------------------------- D3280 states -- */

/*
 * sub_D3280(1) -- the analog power-up the shipping firmware runs.
 *
 * Against the bootloader's sub_1310, which is what this driver used to run:
 *
 *              bootloader sub_1310      OSOS sub_D3280(1)
 *   0x0227     rmw 0x7f = 0x40          wr = 0x40   (wide frame)
 *   0x0225     rmw 0xff = 0x19          wr = 0x33   (wide frame)
 *   0x0226     rmw 0xff = 0x19          not written
 *   0x0228     rmw 0x7f = 0x40          not written
 *   0x0229     not written              wr = 0x40   (wide frame)
 *   0x0075     not touched              rmw 0x80 = 0x00
 *   0x0220     rmw 0x78 = 0x50          rmw 0x78 = 0x78
 *   0x0006 b0  set                      set
 *   0x002F     poll bit 7               poll bit 7
 *   0x0006 b6  set                      set
 *   0x0007 b6  CLEARED                  SET
 *
 * The last row is the one that matters. State 3 clears 0x0007 bit 6; running
 * the bootloader's ending means state 3 clears a bit nothing ever set.
 *
 * Bit 7 of 0x002F is the analog block reporting itself ready, and everything
 * after is sequenced against it. Programming a codec that has not finished
 * powering explains a register file that reads back perfectly and drives
 * nothing. The poll is bounded -- stock's is a bare do/while with no escape,
 * which is fine in an RTOS that owns the machine and is not fine here.
 */
static void cs42_d3280_state1_analog_on(void)
{
    unsigned i;
    uint8_t v2f = 0;

    cs42_write(CS42_R0227, 0x40);
    cs42_write(CS42_R0225, 0x33);
    cs42_write(CS42_R0229, 0x40);
    cs42_rmw(CS42_R0075, 0x80, 0x00);
    cs42_rmw(CS42_R0220, 0x78, 0x78);

    /*
     * A one-shot start, not a level to hold. There are six writes to 0x0006
     * in the whole image and nothing re-sets bit 0 after the graph table, so
     * anything that "restores" it is putting back a bit stock leaves clear.
     */
    cs42_rmw(CS42_R0006, 0x01, 0x01);

    for (i = 0; i < CS42_READY_POLLS; i++) {
        udelay(1000);
        v2f = cs42_read(CS42_R002F);
        if (v2f & 0x80)
            break;
    }

    cs42_rmw(CS42_R0006, 0x40, 0x40);
    cs42_rmw(CS42_R0007, 0x40, 0x40);
}

/*
 * sub_D3280(3) -- restores the codec clock and releases the freeze latch
 * state 1 takes (0x0006 bit 6, 0x0007 bit 6, 0x0075 bit 7).
 *
 * States 1 and 3 are a matched pair around the clock gate and must run in
 * that order. Running 3 first releases a latch nothing took, and 1 then takes
 * it with no one left to release it.
 *
 * The 0x007B/0x007C reads in the middle are stock's and are kept; nothing
 * consults them or gates on them.
 */
static void cs42_d3280_state3_unfreeze(void)
{
    uint8_t r74;

    /*
     * sub_D2EFC, guarded so it runs exactly once. Whatever 0x9901 gates,
     * stock wants it opened here and not again.
     */
    if (!cs42_unlock_done) {
        cs42_write(CS42_R9901, 0xa5);
        cs42_write(CS42_R9901, 0x00);
        cs42_unlock_done = true;
    }

    cs42_rmw(CS42_R0007, 0x40, 0x00);
    cs42_rmw(CS42_R0006, 0x40, 0x00);

    cs42_mode38 = 0x28;

    /* State 1 already put mask 0x78 to 0x78, which subsumes this. Stock
     * writes it anyway. */
    cs42_rmw(CS42_R0220, 0x28, 0x28);
    cs42_rmw(CS42_R000F, 0x80, 0x80);
    cs42_rmw(CS42_R0075, 0x40, 0x40);

    r74 = cs42_read(CS42_R0074);
    cs42_write(CS42_R0074, (uint8_t)((r74 & 0xe7) | 0x08));
    (void)cs42_read(CS42_R007B);
    (void)cs42_read(CS42_R007C);
    cs42_write(CS42_R0074, r74);

    cs42_rmw(CS42_R0075, 0x40, 0x00);
    cs42_rmw(CS42_R0075, 0x80, 0x80);

    (void)cs42_read(CS42_R000F);
    (void)cs42_read(CS42_R002F);
}

/*
 * sub_D3280(4) -- standby. A power-DOWN sequence, and the stop path is the
 * only place it belongs.
 *
 * It clears the analog enable state 1 polled for, drops the 2v5 rail to 0x0E,
 * writes 0x0225 = 0 over state 1's 0x33, and puts back 0x0223/0x0224 = 08/09
 * -- the NATIVE rate pair, clobbering whatever the SRC was just given. The
 * old driver ran this as the last step of every prepare.
 */
static void cs42_d3280_state4_standby(void)
{
    cs42_rmw(CS42_R0007, 0x40, 0x00);
    cs42_rmw(CS42_R0219, 0x78, 0x78);
    cs42_write(CS42_R0229, 0x40);
    cs42_rmw(CS42_R0006, 0x01, 0x00);
    cs42_rmw(CS42_R0201, 0xe0, 0x40);
    cs42_write(CS42_RC81F, 0xff);
    cs42_write(CS42_RC85F, 0x0f);
    cs42_write(CS42_RC96F, 0x0e);
    cs42_write(CS42_R0223, 0x08);
    cs42_write(CS42_R0224, 0x09);
    cs42_write(CS42_R0225, 0x00);
    cs42_write(CS42_R0229, 0x40);
    cs42_set_output_gain(CS42_GAIN_MUTE);
    cs42_set_output_gain(CS42_GAIN_MIN);
    cs42_write(CS42_R0229, 0x41);
    cs42_rmw(CS42_R000E, 0xc0, 0x40);
}

/* ------------------------------------------------------------- the rate -- */

/*
 * sub_D34C0 is a three-way branch on MEMORY[0x892A038], modelled as
 * cs42_mode38:
 *
 *   & 0x08 and & 0x20  -> short: 0x000F/0x012F only
 *   & 0x08 and not     -> long:  0x0121/0x0122/0x0130/0x0131 + 0x0222/3/4
 *   otherwise          -> sub_183138(code, 1)
 *
 * They are alternatives. 183138 programs 0x010B/0x010C where the long arm
 * programs 0x0223/0x0224 -- different blocks for the same job.
 */

/* Short arm: rate code into 0x000F/0x012F, bracketed, and nothing else. */
static void cs42_d34c0_short(uint8_t code)
{
    cs42_rmw(CS42_R000E, 0xc0, 0xc0);
    cs42_rmw(CS42_R000F, 0x0f, code);
    cs42_write(CS42_R012F, (uint8_t)(code | (code << 4)));
    cs42_rmw(CS42_R000E, 0xc0, 0x40);
}

/*
 * Long arm: mute, raise the hold, program inside a 0x000E bracket, drop the
 * hold and restore the gain.
 *
 * Stock's SRC test is a pair of config-flag lookups crossed with the rate.
 * With no access to that config the native case is code 12 exactly, as
 * everywhere else here.
 */
static void cs42_d34c0_long(uint8_t code)
{
    bool src = (code != 12);

    cs42_set_output_gain(CS42_GAIN_MUTE);
    cs42_rmw(CS42_R0220, 0x20, 0x20);
    udelay(1000);

    cs42_rmw(CS42_R000E, 0xc0, 0xc0);
    cs42_rmw(CS42_R000F, 0x0f, code);
    cs42_write(CS42_R012F, (uint8_t)(code | (code << 4)));

    if (src) {
        cs42_write(CS42_R0121, 0x08);
        cs42_write(CS42_R0122, 0x09);
        cs42_rmw(CS42_R0130, 0x0f, code);
        cs42_rmw(CS42_R0131, 0x01, 0x00);
        cs42_write(CS42_R0223, 0x04);
        cs42_write(CS42_R0224, 0x33);
    } else {
        cs42_rmw(CS42_R0131, 0x01, 0x01);
        cs42_write(CS42_R0223, 0x08);
        cs42_write(CS42_R0224, 0x09);
    }

    cs42_write(CS42_R0222, src ? 12 : code);

    cs42_rmw(CS42_R000E, 0xc0, 0x40);
    cs42_rmw(CS42_R0220, 0x20, 0x00);
    udelay(1000);

    /* Stock restores its cached volume here; ours comes back at the end of
     * prepare via cs42_apply_user_vol(). */
}

/*
 * sub_183138(code, 1).
 *
 * Ends muted at -90 dB with the 0x0220 bit-5 hold raised, and that is not a
 * transcription slip: sub_D2F64 clears 0x0220 mask 0x28 as its second write,
 * so output_path_enable is what releases it. This is exactly why prepare has
 * to run set_rate before output_path_enable.
 *
 * code 12 (48 kHz) runs native on 0x010B/0x010C; everything else goes through
 * the SRC, which needs 0x0121/0x0122/0x0130 as well. Always taking the 48 kHz
 * arm while the interface ran 44.1 is an ASP/SRC mismatch, and pulsed noise
 * with it.
 */
static void cs42_183138_set_rate(uint8_t code)
{
    cs42_rmw(CS42_R000E, 0xc0, 0xc0);
    cs42_rmw(CS42_R000F, 0x0f, code);
    cs42_write(CS42_R012F, (uint8_t)(code | (code << 4)));

    if (code == 12) {
        cs42_write(CS42_R010B, 0x08);
        cs42_write(CS42_R010C, 0x09);
        cs42_rmw(CS42_R0131, 0x01, 0x01);
    } else {
        cs42_write(CS42_R0121, 0x08);
        cs42_write(CS42_R0122, 0x09);
        cs42_rmw(CS42_R0130, 0x0f, code);
        cs42_rmw(CS42_R0131, 0x01, 0x00);
        cs42_write(CS42_R010B, 0x04);
        cs42_write(CS42_R010C, 0x33);
    }

    cs42_rmw(CS42_R000E, 0xc0, 0x40);

    /* Both of these were missing from the old driver. See above. */
    cs42_set_output_gain(CS42_GAIN_MUTE);
    cs42_rmw(CS42_R0220, 0x20, 0x20);
}

/*
 * Prepare takes the 183138 branch directly and does not consult mode38 to get
 * there, which is worth justifying because it looks like ignoring the
 * dispatch.
 *
 * mode38 shadows 0x0220 bits 5 and 3. D3280(3) sets both and writes 0x28 to
 * the shadow in the same breath; sub_D2F64 clears both and writes v3 & 0x28,
 * which is zero for mode 271 and mode 6 alike. Those two bits read as an
 * idle/standby pair -- cleared whenever a route exists at all.
 *
 * At the point prepare programs the rate, D3280(3) has just set them, so a
 * faithful dispatch would take the SHORT arm -- which writes 0x000F and
 * 0x012F and nothing else. That cannot be right at 44.1 kHz, because the SRC
 * lives in 0x0121/0x0122/0x0130/0x0131 and the short arm never touches it.
 *
 * The reading that fits: D34C0 is the steady-state entry point (a track
 * change, route already up) while initial route setup calls sub_183138
 * directly -- consistent with 183138 having its own a2 argument and its own
 * route gate.
 */
static void cs42_set_rate(unsigned rate)
{
    uint8_t code = iis_codec_rate_code(rate);

    cs42_183138_set_rate(code);
    cs42_rate = rate;
}

/* ------------------------------------------------------ mailbox / status -- */

/*
 * sub_15A50C then sub_1326D2: latch, sample both levels, release, then the
 * read-and-discard acknowledge pair, then drain the read FIFO by the level
 * the part reported.
 */
static void cs42_mailbox_reads(void)
{
    uint8_t lvl_520;
    unsigned i, drain;

    cs42_rmw(CS42_R051E, 0x01, 0x01);
    (void)cs42_read(CS42_R051F);
    lvl_520 = cs42_read(CS42_R0520);
    cs42_rmw(CS42_R051E, 0x01, 0x00);

    (void)cs42_read(CS42_R0520);
    (void)cs42_read(CS42_R0524);

    drain = lvl_520;
    if (drain > CS42_MBOX_DRAIN_MAX)
        drain = CS42_MBOX_DRAIN_MAX;
    for (i = 0; i < drain; i++)
        (void)cs42_read(CS42_R0525);
}

/*
 * Read the serial-port status and the clock registers. No gate, no retry.
 *
 * This replaces a five-attempt lock handshake on 0x002F bit 6. The
 * whole-image decompilation reads 0x002F exactly once -- the readiness poll
 * in sub_D3280(1), testing bit 7 -- and never examines bit 6 anywhere. There
 * is no ASP lock handshake in this part.
 */
static void cs42_asp_status(void)
{
    (void)cs42_read(CS42_R002F);
    (void)cs42_read(CS42_R000F);
    (void)cs42_read(CS42_R012F);
}

/* ------------------------------------------------------------ the graph -- */

struct cs42_regval {
    uint16_t reg;
    uint8_t  val;
};

/*
 * sub_5707D8, literal. Verified position-for-position against the image
 * across all 80 writes -- not a loop reconstructed from a pattern.
 */
static const struct cs42_regval cs42_static_5707d8[] = {
    { 0x0006, 0x24 },
    { 0x0529, 0x2c }, { 0x052a, 0x2c }, { 0x0533, 0x2c }, { 0x0534, 0x2c },

    { 0x0400, 0x04 }, { 0x0401, 0x10 }, { 0x0402, 0x00 }, { 0x0403, 0x09 },
    { 0x0404, 0x08 }, { 0x0405, 0x00 }, { 0x0406, 0x00 },

    { 0x0407, 0x00 }, { 0x0408, 0x01 }, { 0x0409, 0xe0 },
    { 0x040a, 0x01 }, { 0x040b, 0x01 }, { 0x040c, 0xe0 },
    { 0x040d, 0xfe }, { 0x040e, 0x00 }, { 0x040f, 0xa0 },

    { 0x0410, 0x02 }, { 0x0411, 0x00 }, { 0x0412, 0x00 },
    { 0x0413, 0x03 }, { 0x0414, 0x00 }, { 0x0415, 0x00 },
    { 0x0416, 0x04 }, { 0x0417, 0x00 }, { 0x0418, 0x00 },
    { 0x0419, 0x05 }, { 0x041a, 0x00 }, { 0x041b, 0x00 },
    { 0x041c, 0x06 }, { 0x041d, 0x00 }, { 0x041e, 0x00 },
    { 0x041f, 0x07 }, { 0x0420, 0x00 }, { 0x0421, 0x00 },
    { 0x0422, 0x08 }, { 0x0423, 0x00 }, { 0x0424, 0x00 },
    { 0x0425, 0x09 }, { 0x0426, 0x00 }, { 0x0427, 0x00 },

    { 0x0428, 0x0a }, { 0x0429, 0x01 }, { 0x042a, 0xe0 },
    { 0x042b, 0x0b }, { 0x042c, 0x01 }, { 0x042d, 0xe0 },
    { 0x042e, 0xff }, { 0x042f, 0x00 }, { 0x0430, 0xa0 },

    { 0x0431, 0x0c }, { 0x0432, 0x00 }, { 0x0433, 0x00 },
    { 0x0434, 0x0d }, { 0x0435, 0x00 }, { 0x0436, 0x00 },
    { 0x0437, 0x0e }, { 0x0438, 0x00 }, { 0x0439, 0x00 },
    { 0x043a, 0x0f }, { 0x043b, 0x00 }, { 0x043c, 0x00 },
    { 0x043d, 0x10 }, { 0x043e, 0x00 }, { 0x043f, 0x00 },
    { 0x0440, 0x11 }, { 0x0441, 0x00 }, { 0x0442, 0x00 },
    { 0x0443, 0x12 }, { 0x0444, 0x00 }, { 0x0445, 0x00 },
    { 0x0446, 0x13 }, { 0x0447, 0x00 }, { 0x0448, 0x00 },

    { 0x0400, 0x04 },
    { 0x0401, 0x12 },
};

/*
 * The graph is built at PLAY, after the power-up.
 *
 * sub_5707D8 is reached only from sub_570620, which is reached from
 * sub_42D364(1) -- the play trigger. It is not part of bring-up.
 */
static void cs42_build_play_graph(void)
{
    unsigned i;

    for (i = 0; i < ARRAYLEN(cs42_static_5707d8); i++)
        cs42_write(cs42_static_5707d8[i].reg, cs42_static_5707d8[i].val);

    sleep(HZ / 10);                     /* sub_43E006(100) */
    cs42_write(CS42_R0500, 0x05);
    cs42_status_528 = cs42_read(CS42_R0528);

    cs42_graph_latched = true;
}

/* --------------------------------------------------- play transport -- */

/* sub_F141C: 0x60 unmuted, 0xFF muted. */
static void cs42_f141c(bool on)
{
    cs42_write(CS42_R0527, on ? 0x60 : 0xff);
}

/* sub_F1444 -- meter soft-ramp pulse. */
static void cs42_f1444(void)
{
    cs42_rmw(CS42_R051E, 0x20, 0x20);
    cs42_rmw(CS42_R051E, 0x20, 0x00);
    cs42_rmw(CS42_R0523, 0x20, 0x20);
    cs42_rmw(CS42_R0523, 0x20, 0x00);
}

/*
 * sub_42D364(0): F141C(0); 0x401 bit0 = 1; bit1 = 0; F1444.
 *
 * Bits 0 and 1 of 0x0401 are always driven in separate masked writes and
 * never as a pair -- a mask of 0x03 would clear bit 0 as a side effect of
 * setting bit 1.
 */
static void cs42_42d364_stop(void)
{
    cs42_f141c(false);
    cs42_rmw(CS42_R0401, 0x01, 0x01);
    cs42_rmw(CS42_R0401, 0x02, 0x00);
    cs42_f1444();
    cs42_playing = false;
}

/* ------------------------------------------------------------- prepare -- */

/*
 * The order here is the whole point. See the note at the top of the file.
 *
 * Idempotent by rate: something opens the PCM repeatedly during a normal
 * boot, and re-running the power-up each time is both slow and a way to
 * un-configure a part that was already right.
 */
static void cs42_codec_prepare(unsigned rate)
{
    if (cs42_prepared_rate == rate)
        return;

    /* Sibling LDOs 21-23. Without these the analog side is untrimmed. */
    pmu_audio_rails(true);

    cs42_d3280_state1_analog_on();
    cs42_d3280_state3_unfreeze();
    cs42_mailbox_reads();

    /* Before output_path_enable: 183138 leaves the 0x0220 hold raised and
     * output_path_enable is what releases it. */
    cs42_set_rate(rate);
    cs42_output_path_enable();

    cs42_build_play_graph();

    cs42_apply_user_vol();
    cs42_prepared_rate = rate;
}

/* ---------------------------------------------------- public interface -- */

void audiohw_preinit(void)
{
    spi_port_init(SPI_PORT_CODEC);
    cs42_up = true;
    cs42_muted = true;
    cs42_prepared_rate = 0;
    cs42_graph_latched = false;
    cs42_mode38 = 0;
}

void audiohw_postinit(void)
{
    if (!cs42_up)
        return;

    /*
     * Deliberately does not configure anything.
     *
     * Configuration belongs to the play path -- the graph is built at PLAY in
     * stock, and the analog power-up is sequenced against clocks that are not
     * running yet at init time. The old driver did its whole init here, which
     * is how every write could succeed against a part that never made a
     * sound.
     */
}

void audiohw_close(void)
{
    if (!cs42_up)
        return;

    cs42_42d364_stop();
    cs42_d3280_state4_standby();
    cs42_prepared_rate = 0;
    cs42_graph_latched = false;
    cs42_up = false;
}

void audiohw_set_frequency(int fsel)
{
    unsigned rate;

    if (!cs42_up)
        return;
    if (fsel < 0 || fsel >= HW_NUM_FREQ)
        return;

    rate = hw_freq_sampr[fsel];
    if (rate == cs42_prepared_rate)
        return;

    /* A rate change is a reconfiguration, not a tweak. */
    cs42_prepared_rate = 0;
    cs42_codec_prepare(rate);
}

void audiohw_set_volume(int vol_l, int vol_r)
{
    /* One gain register: the louder side wins, as elsewhere in Rockbox. */
    int db = (vol_l > vol_r) ? vol_l : vol_r;

    cs42_user_vol = db / 10;            /* Rockbox passes tenths of a dB */
    if (cs42_up)
        cs42_apply_user_vol();
}

void audiohw_mute(bool mute)
{
    cs42_muted = mute;
    if (!cs42_up)
        return;

    cs42_f141c(!mute);
    cs42_apply_user_vol();
}

/*
 * Called by the PCM driver immediately BEFORE the IIS clocks start.
 *
 * Deliberately does nothing to 0x000F.
 *
 * An earlier version dropped bit 7 here, described as "so the interface is
 * not running into a codec that is still being configured". That is the
 * of_asp_slave experiment from the Linux driver, and it is off by default
 * there -- in the configuration that actually makes sound, bit 7 stays SET
 * from D3280(3), which raises it on purpose.
 *
 * Clearing it drops the ASP enable immediately before the clocks arrive,
 * which is a good way to hand a running interface to a serial port that is
 * switched off.
 *
 * The master/slave contract needs nothing from this end either. The codec is
 * SND_SOC_DAIFMT_CBS_CFS -- bit-clock slave and frame slave -- and the Linux
 * codec driver has no set_fmt at all, because slave is the part's default
 * rather than something programmed. BCLK and LRCLK are SoC-driven: IIS0 is
 * the master, and I2SCLKDIV at +0x40 is what divides MCLK down to the frame
 * rate. See iis0_program() in pcm-s5l8740.c.
 *
 * The hook is kept because the ordering it marks is real -- configuration
 * before clocks, unmute after -- and because a future part of the sequence
 * may need to land here.
 */
void cs42l81_pre_iis_start(void)
{
    if (!cs42_up)
        return;
}

/*
 * Called by the PCM driver AFTER the IIS clocks are running.
 *
 * The unmute lives here rather than in prepare, and that is the point: an
 * unmute issued before the interface is running is one of the ways to get
 * a perfectly configured, completely silent codec.
 */
void cs42l81_post_iis_start(void)
{
    if (!cs42_up)
        return;

    cs42_asp_status();
    cs42_f141c(!cs42_muted);
    cs42_rmw(CS42_R0401, 0x02, 0x02);
    cs42_apply_user_vol();
    cs42_playing = true;
}

void cs42l81_play_prepare(unsigned rate)
{
    if (!cs42_up)
        return;
    if (cs42_playing)
        cs42_42d364_stop();
    cs42_codec_prepare(rate);
}

void cs42l81_play_start(void)
{
    if (!cs42_up)
        return;

    cs42_f141c(true);

    if (!cs42_graph_latched)
        cs42_build_play_graph();
    else
        cs42_rmw(CS42_R0401, 0x02, 0x02);

    cs42_playing = true;
}

void cs42l81_play_stop(void)
{
    if (!cs42_up)
        return;
    cs42_42d364_stop();
}

void cs42l81_get_route(struct cs42l81_route *r)
{
    if (!r)
        return;

    memset(r, 0, sizeof(*r));
    if (!cs42_up)
        return;

    r->r0401 = cs42_read(CS42_R0401);
    r->r0403 = cs42_read(0x0403);
    r->r0404 = cs42_read(0x0404);
    r->r0500 = cs42_read(CS42_R0500);
    r->r0527 = cs42_read(CS42_R0527);
    r->r054f = cs42_read(0x054f);
    r->r0075 = cs42_read(CS42_R0075);
    r->r0220 = cs42_read(CS42_R0220);
    r->r002f = cs42_read(CS42_R002F);
    r->r0227 = cs42_read(CS42_R0227);
    r->r0219 = cs42_read(CS42_R0219);
    r->rc96f = cs42_read(CS42_RC96F);
    r->status_528 = cs42_status_528;
    r->mode38 = cs42_mode38;
    r->rate = cs42_rate;
    r->playing = cs42_playing;
}
