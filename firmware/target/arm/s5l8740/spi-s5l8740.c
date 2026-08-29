/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * SPI master for Apple S5L8740 (iPod nano 7G).
 *
 * Ported from tools/linux-n31/drivers/spi-s5l8702.c, which reconstructed the
 * transfer path from RetailOS sub_4043D0 and the engine setup from sub_11B70.
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
#include "config.h"
#include "system.h"
#include "kernel.h"
#include "spi-s5l8740.h"
#include "clocking-s5l8740.h"
#include "gpio-s5l8740.h"

#define SPICTRL     0x00
#define SPISETUP    0x04
#define SPISTATUS   0x08
#define SPIPIN      0x0c
#define SPITXDATA   0x10
#define SPIRXDATA   0x20
#define SPICLKDIV   0x30
#define SPIRXLIMIT  0x34
#define SPIUNK38    0x38
#define SPIUNK3C    0x3c
#define SPIUNK40    0x40
#define SPIUNK44    0x44
/* sub_4043D0 writes 1 here after every TX word. */
#define SPIUNK4C    0x4c

#define SPISTATUS_TXBUSY    0x7c0
#define SPISTATUS_RXRDY     0xf800
#define SPISTATUS_KICK      0x400000

#define SPISETUP_RXMODE     (1 << 0)
/* mode 0x1A from sub_11B70 -> 0x402E | 0x10. NOT 0x403C, a different mode. */
#define SPISETUP_11B70      0x403e

#define SPICTRL_RESET_FIFO  0xc
#define SPICTRL_ENABLE      0x1

#define SPIPIN_CS           (1 << 1)

#define SPI_WAIT_GUARD      500000

static uint32_t spi_base(int port)
{
    return (port == SPI_PORT_TOUCH) ? SPI2_BASE : SPI0_BASE;
}

#define SPI_REG(port, off)  (*(REG32_PTR_T)(spi_base(port) + (off)))

static struct mutex spi_mtx[3];

/*
 * Both waits accept the Rockbox Classic status encodings as a fallback. The
 * two families report TX-empty and RX-ready in different bits, and the N31
 * engine has been observed using each depending on which setup path ran.
 */
static int spi_wait_clear(int port, uint32_t mask)
{
    unsigned guard = SPI_WAIT_GUARD;

    while (guard--) {
        if ((SPI_REG(port, SPISTATUS) & mask) == 0)
            return 0;
    }

    if (mask == SPISTATUS_TXBUSY) {
        guard = SPI_WAIT_GUARD / 4;
        while (guard--) {
            if ((SPI_REG(port, SPISTATUS) & 0x1f0) == 0)
                return 0;
        }
    }
    return -1;
}

static int spi_wait_set(int port, uint32_t mask)
{
    unsigned guard = SPI_WAIT_GUARD;

    while (guard--) {
        if (SPI_REG(port, SPISTATUS) & mask)
            return 0;
    }

    if (mask == SPISTATUS_RXRDY) {
        guard = SPI_WAIT_GUARD / 4;
        while (guard--) {
            if (SPI_REG(port, SPISTATUS) & 0x3e00)
                return 0;
        }
    }
    return -1;
}

/*
 * After the sub_11B70 setup, STATUS bit 6 (0x40) stays set and the 0x7C0 mask
 * never reaches zero. Treat that exact residue as "not busy" rather than
 * timing out on every single transfer.
 */
static int spi_wait_tx_idle(int port)
{
    int ret = spi_wait_clear(port, SPISTATUS_TXBUSY);

    if (ret && (SPI_REG(port, SPISTATUS) & SPISTATUS_TXBUSY) == 0x40)
        ret = 0;
    return ret;
}

static void spi_cs(int port, bool assert)
{
    uint32_t pin = SPI_REG(port, SPIPIN);

    if (assert)
        pin &= ~SPIPIN_CS;
    else
        pin |= SPIPIN_CS;
    SPI_REG(port, SPIPIN) = pin;
}

/*
 * Pad muxing.
 *
 * SPI0 is OSOS sub_743A4: pad 0 to function 4, pads 1-3 to function 2.
 * SPI2 is sub_20690(1): pad 0x57 to function 5, pads 0x58-0x5A to function 3,
 * with 0x57's PUNC bit cleared first (sub_23CD0).
 */
static void spi0_pinmux(void)
{
    gpio_set_function(0, 4);
    gpio_set_function(1, GPIO_FUNC_ALT2);
    gpio_set_function(2, GPIO_FUNC_ALT2);
    gpio_set_function(3, GPIO_FUNC_ALT2);
}

static void spi2_pinmux(void)
{
    GPIO_PUNK10(0x57 >> 3) &= ~(1 << (0x57 & 7));

    gpio_set_function(0x57, 5);
    gpio_set_function(0x58, GPIO_FUNC_ALT3);
    gpio_set_function(0x59, GPIO_FUNC_ALT3);
    gpio_set_function(0x5A, GPIO_FUNC_ALT3);
}

/*
 * Engine setup, RetailOS sub_11B70(port, 0x1A, 0x2EE0, a4).
 *
 * The +0x38 / +0x3c pair affects receive behaviour -- leaving them at reset
 * made touch ping RX return junk -- so both are programmed even though the
 * derivation is only partly understood.
 *
 * For SPI2 the stock oracle captured with touch working has +0x3c = 0x18c.
 * The formula 3 * 24 * (a4 + 1) gives 0x90 for a4 = 1, and solving for 0x18c
 * needs a4 = 4.5, so the derivation does not reproduce the hardware. The
 * stock value wins.
 */
static void spi_engine_init(int port, unsigned a4, uint32_t u3c)
{
    const unsigned clk_kunit = 24;

    SPI_REG(port, SPIUNK44) = 10;
    SPI_REG(port, SPIUNK38) = clk_kunit * a4;
    SPI_REG(port, SPIUNK40) = 255;
    SPI_REG(port, SPIUNK3C) = u3c;
    SPI_REG(port, SPICLKDIV) = 2;
}

void spi_port_init(int port)
{
    if (port == SPI_PORT_CODEC) {
        /* PWRCON1 SPI0 plus the companion gate in PWRCON4. */
        clockgate_enable(CLKCON_PWRCON1, PWRCON1_SPI0, true);
        clockgate_enable(CLKCON_PWRCON4, PWRCON4_SPI0_2, true);

        spi0_pinmux();

        SPI_REG(port, SPISTATUS) = 0xf;
        SPI_REG(port, SPICTRL) |= SPICTRL_RESET_FIFO;
        spi_engine_init(port, 8, 3 * 24 * (8 + 1));
        SPI_REG(port, SPIPIN) = 0x6;    /* idle: CS deasserted */
        SPI_REG(port, SPISETUP) = SPISETUP_11B70;
        SPI_REG(port, SPICTRL) |= SPICTRL_RESET_FIFO;
        SPI_REG(port, SPICTRL) = SPICTRL_ENABLE;
    } else {
        /* Bit 15 goes with SPI2 in PWRCON1. */
        clockgate_enable(CLKCON_PWRCON1, PWRCON1_SPI2 | (1 << 15), true);

        spi2_pinmux();

        /* OSOS writes neither SPIPIN nor STATUS on this path. */
        spi_engine_init(port, 1, 0x18c);
        SPI_REG(port, SPISETUP) = SPISETUP_11B70;
        SPI_REG(port, SPICTRL) = SPICTRL_ENABLE;
    }
}

void spi_init(void)
{
    mutex_init(&spi_mtx[SPI_PORT_CODEC]);
    mutex_init(&spi_mtx[SPI_PORT_TOUCH]);

    spi_port_init(SPI_PORT_CODEC);
    spi_port_init(SPI_PORT_TOUCH);
}

/* RetailOS sub_4043D0. */
static int spi_transfer_locked(int port, const uint8_t *tx, uint8_t *rx, int len)
{
    int i;
    int ret;

    spi_cs(port, true);

    SPI_REG(port, SPICTRL) |= SPICTRL_RESET_FIFO;

    ret = spi_wait_tx_idle(port);
    if (ret)
        goto out;
    ret = spi_wait_clear(port, SPISTATUS_RXRDY);
    if (ret)
        goto out;

    /*
     * TX present  -> clear RXMODE, then kick with STATUS bit 22.
     * RX only     -> set RXMODE, then STATUS bit 0 to self-clock.
     */
    if (tx) {
        SPI_REG(port, SPISETUP) &= ~SPISETUP_RXMODE;
        SPI_REG(port, SPISTATUS) |= SPISTATUS_KICK;
    } else {
        SPI_REG(port, SPISETUP) |= SPISETUP_RXMODE;
        SPI_REG(port, SPISTATUS) |= 1;
    }

    for (i = 0; i < len; i++) {
        /* RXLIMIT is only armed when the caller actually wants data back. */
        SPI_REG(port, SPIRXLIMIT) = rx ? 1 : 0;

        ret = spi_wait_tx_idle(port);
        if (ret)
            break;

        if (tx) {
            SPI_REG(port, SPITXDATA) = tx[i];
            SPI_REG(port, SPIUNK4C) = 1;
        }

        if (rx) {
            ret = spi_wait_set(port, SPISTATUS_RXRDY);
            if (ret)
                break;
            rx[i] = (uint8_t)SPI_REG(port, SPIRXDATA);
        }
    }

    if (!ret)
        ret = spi_wait_tx_idle(port);

    /* Epilogue: drop both the kick and RXMODE. */
    SPI_REG(port, SPISETUP) &= ~0x400001u;

out:
    spi_cs(port, false);
    return ret;
}

int spi_transfer(int port, const uint8_t *tx, uint8_t *rx, int len)
{
    int ret;

    if (port != SPI_PORT_CODEC && port != SPI_PORT_TOUCH)
        return -1;

    mutex_lock(&spi_mtx[port]);
    ret = spi_transfer_locked(port, tx, rx, len);
    mutex_unlock(&spi_mtx[port]);
    return ret;
}
