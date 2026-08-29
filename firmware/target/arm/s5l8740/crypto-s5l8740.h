/***************************************************************************
 * SHA1, AES and PRNG for Apple S5L8740 (iPod nano 7G).
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#ifndef __CRYPTO_S5L8740_H__
#define __CRYPTO_S5L8740_H__

#include <stdbool.h>
#include <stdint.h>

/* Hardware SHA1 over a flat buffer. hash receives 20 bytes. */
int s5l8740_sha1(const void *data, uint32_t size, void *hash);

void s5l8740_prng_seed(uint32_t seed);
int  s5l8740_prng_read(uint32_t *out, unsigned count);

enum s5l8740_aes_dir {
    S5L8740_AES_DECRYPT = 0,
    S5L8740_AES_ENCRYPT = 1,
};

/*
 * Key sources. GID and UID are fused into the SoC and never leave it -- they
 * are what Apple's image formats are keyed on.
 */
enum s5l8740_aes_key {
    S5L8740_AES_KEY_USER = 0,
    S5L8740_AES_KEY_GID  = 1,
    S5L8740_AES_KEY_UID  = 2,
    S5L8740_AES_KEY_ZERO = 3,
};

/*
 * In-place AES over a whole number of 16-byte blocks. iv may be NULL for ECB.
 * Returns 0, or negative on timeout or illegal operation.
 */
int s5l8740_aes(enum s5l8740_aes_dir dir, enum s5l8740_aes_key keytype,
                const void *key, unsigned keylen,
                const void *iv, void *data, uint32_t size);

#endif /* __CRYPTO_S5L8740_H__ */
