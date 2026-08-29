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

/*
 * Status encodings.
 *
 * This engine has been seen reporting completion in either of two ways,
 * depending on which init path ran:
 *
 *   ROS      TX busy 0x7C0 -> 0,   RX ready 0xF800 nonzero
 *   CLASSIC  TX level 0x1F0 -> 0,  RX level 0x3E00 nonzero
 *
 * This driver originally waited on the ROS masks and then, on timeout,
 * accepted the CLASSIC masks as a fallback. That is a correctness hazard
 * rather than merely a slow path: the masks OVERLAP -- 0x7C0 and 0x1F0 share
 * bits 6-8, 0xF800 and 0x3E00 share bits 11-13 -- so the wrong family's test
 * can read as satisfied while the transfer is still in flight. RXDATA is then
 * sampled early and returns the FIFO's previous contents. That does not look
 * like noise, because it is not noise: it is the same stale word every time,
 * which is exactly the repeating 0x4f81 pattern seen on SPI2.
 *
 * So the encoding is a per-instance property. While unlatched, both families
 * are polled in ONE loop and whichever genuinely satisfies first is latched.
 * After that only the latched family is consulted, and a timeout is a real
 * timeout rather than a cue to try the other interpretation.
 */
#define SPISTATUS_TXBUSY_ROS    0x7c0
#define SPISTATUS_RXRDY_ROS     0xf800
#define SPISTATUS_TXLVL_CLASSIC 0x1f0
#define SPISTATUS_RXLVL_CLASSIC 0x3e00
/* Stays set after the sub_11B70 setup; idle for ROS purposes. */
#define SPISTATUS_TXBUSY_RESIDUE 0x40
#define SPISTATUS_TXFULL        0x100

#define SPISTATUS_KICK          0x400000

#define SPISETUP_RXMODE     (1 << 0)
/* mode 0x1A from sub_11B70 -> 0x402E | 0x10. NOT 0x403C, a different mode. */
#define SPISETUP_11B70      0x403e

#define SPICTRL_RESET_FIFO  0xc
#define SPICTRL_ENABLE      0x1

#define SPIPIN_CS           (1 << 1)

#define SPI_WAIT_GUARD      500000

enum spi_family {
    SPI_FAM_AUTO = 0,
    SPI_FAM_ROS,
    SPI_FAM_CLASSIC,
};

struct spi_port_state {
    enum spi_family fam;
    bool            cs_held;
    unsigned        tx_timeouts;
    unsigned        rx_timeouts;
};

/* Indexed by port number; only 0 and 2 are wired on this board. */
static struct spi_port_state spi_state[3];
static struct mutex spi_mtx[3];

static uint32_t spi_base(int port)
{
    return (port == SPI_PORT_TOUCH) ? SPI2_BASE : SPI0_BASE;
}

#define SPI_REG(port, off)  (*(REG32_PTR_T)(spi_base(port) + (off)))

static void spi_latch_fam(int port, enum spi_family f)
{
    spi_state[port].fam = f;
}

/*
 * Wait for the transmit side to go idle.
 *
 * Only the latched family is consulted. While still AUTO both are polled in
 * the same loop, so the first genuine completion decides rather than one
 * family getting a full guard interval of head start.
 */
static int spi_wait_tx_idle(int port)
{
    unsigned guard = SPI_WAIT_GUARD;
    enum spi_family fam = spi_state[port].fam;

    while (guard--) {
        uint32_t st = SPI_REG(port, SPISTATUS);

        if (fam != SPI_FAM_CLASSIC) {
            uint32_t b = st & SPISTATUS_TXBUSY_ROS;

            if (b == 0 || b == SPISTATUS_TXBUSY_RESIDUE) {
                spi_latch_fam(port, SPI_FAM_ROS);
                return 0;
            }
        }
        if (fam != SPI_FAM_ROS && (st & SPISTATUS_TXLVL_CLASSIC) == 0) {
            spi_latch_fam(port, SPI_FAM_CLASSIC);
            return 0;
        }
    }

    spi_state[port].tx_timeouts++;
    return -1;
}

/* Wait for received data to actually be available. Same latching rule. */
static int spi_wait_rx_ready(int port)
{
    unsigned guard = SPI_WAIT_GUARD;
    enum spi_family fam = spi_state[port].fam;

    while (guard--) {
        uint32_t st = SPI_REG(port, SPISTATUS);

        if (fam != SPI_FAM_CLASSIC && (st & SPISTATUS_RXRDY_ROS)) {
            spi_latch_fam(port, SPI_FAM_ROS);
            return 0;
        }
        if (fam != SPI_FAM_ROS && (st & SPISTATUS_RXLVL_CLASSIC)) {
            spi_latch_fam(port, SPI_FAM_CLASSIC);
            return 0;
        }
    }

    /*
     * A receive timeout is an error and propagates. Reading RXDATA anyway
     * would hand the caller the FIFO's previous contents as if it were a
     * reply -- the exact bug this driver used to have.
     */
    spi_state[port].rx_timeouts++;
    return -1;
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
    /*
     * Re-measure the status encoding after any re-init: which family the
     * engine reports in depends on the init path that ran, so a latch from
     * before a re-setup is not necessarily still true.
     */
    spi_state[port].fam = SPI_FAM_AUTO;
    spi_state[port].cs_held = false;

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
static int spi_transfer_locked(int port, const uint8_t *tx, uint8_t *rx,
                               int len, bool assert_cs, bool deassert_cs)
{
    int i;
    int ret;

    if (assert_cs)
        spi_cs(port, true);
    spi_state[port].cs_held = true;

    SPI_REG(port, SPICTRL) |= SPICTRL_RESET_FIFO;

    ret = spi_wait_tx_idle(port);
    if (ret)
        goto out;

    /*
     * Drain anything the FIFO is still holding before the real transfer, so
     * a stale word cannot be mistaken for this transfer's first reply.
     */
    while (SPI_REG(port, SPISTATUS) &
           (SPISTATUS_RXRDY_ROS | SPISTATUS_RXLVL_CLASSIC))
        (void)SPI_REG(port, SPIRXDATA);

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
        /*
         * RXLIMIT is armed for every word, not only when the caller wants
         * data. The part clocks a reply out regardless, and leaving it
         * undrained overruns the RX FIFO partway through a long burst --
         * which silently loses everything after the header.
         */
        SPI_REG(port, SPIRXLIMIT) = 1;

        ret = spi_wait_tx_idle(port);
        if (ret)
            break;

        /* TXDATA is byte wide; a wider write only puts the low byte out. */
        SPI_REG(port, SPITXDATA) = tx ? tx[i] : 0xff;
        if (tx)
            SPI_REG(port, SPIUNK4C) = 1;

        ret = spi_wait_rx_ready(port);
        if (ret)
            break;

        {
            uint8_t b = (uint8_t)SPI_REG(port, SPIRXDATA);

            if (rx)
                rx[i] = b;
        }
    }

    if (!ret)
        ret = spi_wait_tx_idle(port);

    /* Epilogue: drop both the kick and RXMODE. */
    SPI_REG(port, SPISETUP) &= ~0x400001u;

out:
    /*
     * Always drop CS on error. Leaving it asserted after a timeout would
     * strand the bus for every later transfer.
     */
    if (deassert_cs || ret) {
        spi_cs(port, false);
        spi_state[port].cs_held = false;
    }

    return ret;
}

int spi_transfer(int port, const uint8_t *tx, uint8_t *rx, int len)
{
    return spi_transfer_cs(port, tx, rx, len, true);
}

/*
 * Chip-select-aware variant.
 *
 * Some protocols on this bus frame on chip select rather than on length --
 * the Nimbus HBPP upload is the one that matters -- so a caller has to be
 * able to hold CS down across several calls. Dropping CS between them
 * silently splits one frame into several, and the part simply stops
 * acknowledging.
 */
int spi_transfer_cs(int port, const uint8_t *tx, uint8_t *rx, int len,
                    bool release_cs)
{
    int ret;

    if (port != SPI_PORT_CODEC && port != SPI_PORT_TOUCH)
        return -1;

    mutex_lock(&spi_mtx[port]);
    ret = spi_transfer_locked(port, tx, rx, len,
                              !spi_state[port].cs_held, release_cs);
    mutex_unlock(&spi_mtx[port]);
    return ret;
}

int spi_get_status_family(int port)
{
    return (int)spi_state[port].fam;
}

void spi_get_timeouts(int port, unsigned *tx, unsigned *rx)
{
    if (tx)
        *tx = spi_state[port].tx_timeouts;
    if (rx)
        *rx = spi_state[port].rx_timeouts;
}
