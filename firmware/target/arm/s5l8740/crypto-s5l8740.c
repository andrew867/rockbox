/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * SHA1, AES and PRNG engines for Apple S5L8740 (iPod nano 7G).
 *
 * Ported from tools/linux-n31/drivers/s5l8702-sha1.c, s5l8702-aes.c and
 * s5l8702-prng.c.
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
#include "crypto-s5l8740.h"
#include "clocking-s5l8740.h"

/*
 * These three blocks are what the FTL and any image verification will need.
 *
 * NOTE the register maps here are NOT the ones in Rockbox's existing
 * crypto-s5l8702.c. That driver describes a different AES block (AESCONTROL
 * at +0x00, AESGO at +0x04, AESTYPE at +0x6c); the engine on this SoC is the
 * descriptor-driven one the Linux N31 driver targets, with POWER at +0x00 and
 * COMMAND at +0x04. They are related parts with genuinely different
 * programming models, so this is a separate driver rather than a widened
 * guard.
 */

/* ------------------------------------------------------------------ SHA1 */

#define SHA1_CONF       0x00
#define SHA1_SWRESET    0x04
#define SHA1_ENDIAN     0x10
#define SHA1_RESULT     0x20
#define SHA1_DATA       0x40
#define SHA1_MASTER     0x80

#define SHA1_CONF_BUSY  (1 << 0)
#define SHA1_CONF_GO    (1 << 1)
#define SHA1_CONF_CONT  (1 << 3)

#define SHA1_BLOCK_BYTES    64
#define SHA1_BLOCK_WORDS    (SHA1_BLOCK_BYTES / 4)
#define SHA1_DIGEST_WORDS   5

#define SHA1_TIMEOUT_US     100000

#define SHA1R(off)  (*(REG32_PTR_T)(SHA1_BASE + (off)))

static int sha1_wait_idle(void)
{
    unsigned stop = USEC_TIMER + SHA1_TIMEOUT_US;

    while (SHA1R(SHA1_CONF) & SHA1_CONF_BUSY) {
        if (TIME_AFTER(USEC_TIMER, stop))
            return -1;
    }
    return 0;
}

static void sha1_hw_reset(void)
{
    SHA1R(SHA1_SWRESET) = 1;
    SHA1R(SHA1_SWRESET) = 0;

    SHA1R(SHA1_CONF) = 0;
    SHA1R(SHA1_MASTER) = 0;
    SHA1R(SHA1_ENDIAN) = 0;
}

static int sha1_feed_block(const uint32_t *block, bool first)
{
    unsigned i;

    for (i = 0; i < SHA1_BLOCK_WORDS; i++)
        SHA1R(SHA1_DATA + i * 4) = block[i];

    /*
     * CONT tells the engine to chain onto the running state rather than
     * restarting from the initial digest constants, so it must be clear for
     * the first block of a message and set for every one after it.
     */
    SHA1R(SHA1_CONF) = SHA1_CONF_GO | (first ? 0 : SHA1_CONF_CONT);

    return sha1_wait_idle();
}

int s5l8740_sha1(const void *data, uint32_t size, void *hash)
{
    const uint8_t *p = data;
    uint32_t remaining = size;
    uint32_t block[SHA1_BLOCK_WORDS];
    uint32_t *out = hash;
    uint64_t bitlen = (uint64_t)size * 8;
    bool first = true;
    unsigned i;

    sha1_hw_reset();

    while (remaining >= SHA1_BLOCK_BYTES) {
        memcpy(block, p, SHA1_BLOCK_BYTES);
        if (sha1_feed_block(block, first))
            return -1;
        first = false;
        p += SHA1_BLOCK_BYTES;
        remaining -= SHA1_BLOCK_BYTES;
    }

    /*
     * Padding is done in software: 0x80, then zeroes, then the message
     * length in bits as a big-endian 64-bit value. If the length does not
     * fit in this block, it spills into one more.
     */
    memset(block, 0, sizeof(block));
    memcpy(block, p, remaining);
    ((uint8_t *)block)[remaining] = 0x80;

    if (remaining >= SHA1_BLOCK_BYTES - 8) {
        if (sha1_feed_block(block, first))
            return -1;
        first = false;
        memset(block, 0, sizeof(block));
    }

    {
        uint8_t *b = (uint8_t *)block;

        for (i = 0; i < 8; i++)
            b[SHA1_BLOCK_BYTES - 1 - i] = (uint8_t)(bitlen >> (8 * i));
    }

    if (sha1_feed_block(block, first))
        return -1;

    for (i = 0; i < SHA1_DIGEST_WORDS; i++)
        out[i] = SHA1R(SHA1_RESULT + i * 4);

    return 0;
}

/* ------------------------------------------------------------------ PRNG */

#define PRNG_CONF       0x00
#define PRNG_DATA       0x04
#define PRNG_SEED       0x08

#define PRNG_CONF_FIFO_CNT  0x7
#define PRNG_FIFO_WORDS     5
#define PRNG_TIMEOUT_US     100000

#define PRNGR(off)  (*(REG32_PTR_T)(PRNG_BASE + (off)))

void s5l8740_prng_seed(uint32_t seed)
{
    PRNGR(PRNG_SEED) = seed;
}

int s5l8740_prng_read(uint32_t *out, unsigned count)
{
    unsigned i;

    for (i = 0; i < count; i++) {
        unsigned stop = USEC_TIMER + PRNG_TIMEOUT_US;

        /* Wait for the FIFO to hold at least one word. */
        while ((PRNGR(PRNG_CONF) & PRNG_CONF_FIFO_CNT) == 0) {
            if (TIME_AFTER(USEC_TIMER, stop))
                return -1;
        }

        out[i] = PRNGR(PRNG_DATA);
    }

    return 0;
}

/* ------------------------------------------------------------------- AES */

#define AES_POWER           0x00
#define AES_COMMAND         0x04
#define AES_SWRST           0x08
#define AES_IRQ             0x0c
#define AES_IRQ_MASK        0x10
#define AES_CFG             0x14
#define AES_TBUF_START      0x20
#define AES_TBUF_SIZE       0x24
#define AES_SBUF_START      0x28
#define AES_SBUF_SIZE       0x2c
#define AES_CRYPT_START     0x30
#define AES_CRYPT_SIZE      0x34
#define AES_KEY_BASE        0x4c    /* KEY_MX..KEY_L, 8 words */
#define AES_CIPHERKEY_SEL   0x6c
#define AES_ENDIAN          0x70
#define AES_IV_BASE         0x74    /* IV_1..IV_4 */
#define AES_COMPLIMENT      0x88
#define AES_UNK8C           0x8c

#define AES_CMD_STOP        0
#define AES_CMD_START       1
#define AES_CMD_ABORT       2

#define AES_IRQ_ALL         0xf
#define AES_IRQ_XFR_DONE    (1 << 0)
#define AES_IRQ_ILLEGAL_OP  (1 << 3)

#define AES_CFG_KEYSIZE_SHIFT   4

#define AES_TIMEOUT_US      500000

#define AESR(off)   (*(REG32_PTR_T)(AES_BASE + (off)))

static void aes_reset(void)
{
    AESR(AES_SWRST) = 1;
    AESR(AES_SWRST) = 0;
}

static void aes_clear_state(void)
{
    unsigned i;

    for (i = 0; i < 8; i++)
        AESR(AES_KEY_BASE + i * 4) = 0;
    for (i = 0; i < 4; i++)
        AESR(AES_IV_BASE + i * 4) = 0;

    AESR(AES_IRQ) = AES_IRQ_ALL;
}

static int aes_write_key(const void *key, unsigned keylen)
{
    const uint32_t *k = key;
    unsigned words = keylen / 4;
    unsigned i;

    if (keylen != 16 && keylen != 24 && keylen != 32)
        return -1;

    /*
     * The key registers run KEY_MX..KEY_L, most significant first, so a
     * short key is written into the high words and the rest left zero.
     */
    for (i = 0; i < words; i++)
        AESR(AES_KEY_BASE + i * 4) = k[i];

    return 0;
}

int s5l8740_aes(enum s5l8740_aes_dir dir, enum s5l8740_aes_key keytype,
                const void *key, unsigned keylen,
                const void *iv, void *data, uint32_t size)
{
    uint32_t cfg;
    uint32_t irq;
    unsigned stop;
    int hw_key_type = keytype;

    if (size == 0 || (size & 0xf))
        return -1;      /* AES works in whole 16-byte blocks */

    clockgate_enable(CLKCON_PWRCON0, (1 << 7), true);

    aes_reset();
    AESR(AES_POWER) = 1;
    AESR(AES_IRQ_MASK) = 0;
    aes_clear_state();

    /*
     * An all-zero user key is not the same thing as "no key": the engine has
     * a dedicated zero-key type, and the stock firmware selects it rather
     * than loading zeroes into the key registers.
     */
    if (keytype == S5L8740_AES_KEY_USER && key) {
        const uint8_t *k = key;
        unsigned i;
        bool all_zero = true;

        for (i = 0; i < keylen; i++) {
            if (k[i]) {
                all_zero = false;
                break;
            }
        }
        if (all_zero)
            hw_key_type = S5L8740_AES_KEY_ZERO;
    }

    AESR(AES_CIPHERKEY_SEL) = hw_key_type;

    /* The engine wants the one's complement of the key selector echoed back. */
    AESR(AES_COMPLIMENT) = ~AESR(AES_CIPHERKEY_SEL);

    if (hw_key_type == S5L8740_AES_KEY_USER) {
        if (aes_write_key(key, keylen))
            return -1;
    }

    AESR(AES_UNK8C) = 0;

    if (iv) {
        const uint32_t *v = iv;
        unsigned i;

        for (i = 0; i < 4; i++)
            AESR(AES_IV_BASE + i * 4) = v[i];
    }

    /*
     * CFG encoding.
     *
     * For the fused GID/UID keys, N31 RetailOS (sub_422FFA) writes exactly
     * (encrypt ? 1 : 0) | 0xE -- so 0x0F encrypt, 0x0E decrypt. That is CBC
     * plus bits 2:1, with no software key-size field.
     *
     * Rockbox's s5l8702 hwkeyaes() uses 0x09 / 0x08 instead. Do NOT
     * transplant that encoding here: the SoCs are related but the fused CFG
     * encoding differs, and this one is what was observed on N31.
     */
    if (hw_key_type == S5L8740_AES_KEY_GID ||
        hw_key_type == S5L8740_AES_KEY_UID) {
        cfg = (dir == S5L8740_AES_ENCRYPT ? 1 : 0) | 0xe;
    } else {
        unsigned keysize;

        switch (keylen) {
        case 24: keysize = 1; break;
        case 32: keysize = 2; break;
        default: keysize = 0; break;    /* 128-bit */
        }

        cfg = (keysize << AES_CFG_KEYSIZE_SHIFT)
            | (dir == S5L8740_AES_ENCRYPT ? 1 : 0);
    }
    AESR(AES_CFG) = cfg;

    AESR(AES_ENDIAN) = 0;

    /*
     * In-place: the engine reads and writes the same buffer. Addresses go
     * through the uncached alias so the engine sees current data without a
     * cache clean, and so its writes are visible afterwards.
     */
    {
        uint32_t phys = (uint32_t)S5L8740_UNCACHED_ADDR(data);

        AESR(AES_CRYPT_START) = phys;
        AESR(AES_CRYPT_SIZE) = size;
        AESR(AES_TBUF_START) = phys;
        AESR(AES_TBUF_SIZE) = size;
        AESR(AES_SBUF_START) = phys;
        AESR(AES_SBUF_SIZE) = size;
    }

    AESR(AES_COMMAND) = AES_CMD_START;

    stop = USEC_TIMER + AES_TIMEOUT_US;
    do {
        irq = AESR(AES_IRQ);
        if (TIME_AFTER(USEC_TIMER, stop)) {
            AESR(AES_COMMAND) = AES_CMD_ABORT;
            AESR(AES_IRQ) = AES_IRQ_ALL;
            return -1;
        }
    } while (!(irq & AES_IRQ_ALL));

    AESR(AES_IRQ) = AES_IRQ_ALL;
    AESR(AES_COMMAND) = AES_CMD_STOP;
    AESR(AES_POWER) = 0;

    return (irq & AES_IRQ_ILLEGAL_OP) ? -1 : 0;
}
