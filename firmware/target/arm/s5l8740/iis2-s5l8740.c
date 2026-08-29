/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * IIS2 capture (BCM2078 PCM / FM audio) for Apple S5L8740.
 *
 * Ported from the IIS2 half of tools/linux-n31/drivers/s5l8740-i2s.c.
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
#include "pl080.h"
#include "gpio-s5l8740.h"
#include "iis-s5l8740.h"

/*
 * FM audio does not come out of the tuner over I2C -- the BCM2078 hands the
 * SoC a digital PCM stream on IIS2, and the SoC plays it back through the
 * normal IIS0 path. So "listening to the radio" is a capture on this block
 * feeding a playback on the other.
 *
 * PL080 peripheral 13, source is the IIS2 RX FIFO at +0x38.
 *
 * IMPORTANT: GPIO 97, 98 and 119 are the IIS2 PCM pads, claimed at function 2
 * when FM powers on and released when it powers off (RetailOS sub_15DD5C).
 * They were once listed as Bluetooth shutdown / device-wake / host-wake, and
 * handing them to the BT driver made it drive this capture bus as GPIOs.
 * They belong here, not to hci.
 */

#define IIS2_DMAC       0
#define IIS2_RX_FIFO    (IIS2_BASE + I2SRXFIFO)

/*
 * Two ping-pong buffers. Sized for ~10 ms at 48 kHz stereo 16-bit, which is
 * short enough to keep latency sane and long enough that the completion
 * interrupt is not hot.
 */
#define IIS2_BUF_WORDS  512

static uint32_t iis2_buf[2][IIS2_BUF_WORDS] __attribute__((aligned(32)));
static int      iis2_cur;
static int      rx_channel = -1;
static bool     capturing;

static void iis2_arm(int buf)
{
    pl080_start_p2m(IIS2_DMAC, rx_channel,
                    IIS2_RX_FIFO,
                    S5L8740_UNCACHED_ADDR(&iis2_buf[buf][0]),
                    PL080_PERI_IIS2_RX, PL080_WIDTH_32, IIS2_BUF_WORDS);
}

static void iis2_dma_complete(void *data)
{
    (void)data;

    if (!capturing)
        return;

    /*
     * Flip to the other buffer immediately so the BCM2078 never stalls, then
     * the filled one is available to whatever consumes it.
     *
     * TODO: hand the completed buffer to the playback path. Today FM audio
     * routing is done inside the controller (the "audio route" HCI command),
     * so this capture is not yet the path to the speaker -- it exists for
     * recording and for the digital-loopback route once Phase 4 lands.
     */
    iis2_cur ^= 1;
    iis2_arm(iis2_cur);
}

static void iis2_pads(bool claim)
{
    if (claim) {
        gpio_set_function(IIS2_PAD_BCLK, GPIO_FUNC_ALT2);
        gpio_set_function(IIS2_PAD_SYNC, GPIO_FUNC_ALT2);
        gpio_set_function(IIS2_PAD_DATA, GPIO_FUNC_ALT2);
    } else {
        gpio_set_function(IIS2_PAD_BCLK, 0);
        gpio_set_function(IIS2_PAD_SYNC, 0);
        gpio_set_function(IIS2_PAD_DATA, 0);
    }
}

void iis2_capture_start(void)
{
    if (capturing)
        return;

    if (rx_channel < 0) {
        rx_channel = pl080_alloc_channel(IIS2_DMAC);
        if (rx_channel < 0)
            return;
        pl080_set_callback(IIS2_DMAC, rx_channel, iis2_dma_complete, NULL);
    }

    iis2_pads(true);

    IIS_REG(IIS2_BASE, I2SCLKCON) = 1;
    IIS_REG(IIS2_BASE, I2STXCON) = IIS2_TXCON_FM;
    IIS_REG(IIS2_BASE, I2SRXCON) = IIS2_RXCON_FM;
    IIS_REG(IIS2_BASE, I2SCLKDIV) = IIS2_CLKDIV_FM;
    IIS_REG(IIS2_BASE, I2SREG44) = IIS2_REG44;

    iis2_cur = 0;
    capturing = true;
    iis2_arm(iis2_cur);

    IIS_REG(IIS2_BASE, I2SRXCOM) = IIS2_RXCOM_DMA;
}

void iis2_capture_stop(void)
{
    if (!capturing)
        return;

    capturing = false;

    IIS_REG(IIS2_BASE, I2SRXCOM) = IIS2_RXCOM_IDLE;

    if (rx_channel >= 0)
        pl080_stop(IIS2_DMAC, rx_channel);

    IIS_REG(IIS2_BASE, I2SCLKCON) = 0;

    /* Give the pads back, exactly as sub_15DD5C does on FM power-off. */
    iis2_pads(false);
}

bool iis2_capture_active(void)
{
    return capturing;
}

uint32_t iis2_status(void)
{
    return IIS_REG(IIS2_BASE, I2SSTATUS);
}
