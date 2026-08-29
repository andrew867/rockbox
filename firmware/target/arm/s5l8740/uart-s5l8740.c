/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * SEC UART console for Apple S5L8740 (iPod nano 7G).
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
#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include "cpu.h"
#include "system.h"
#include "serial.h"

/*
 * UART3 @0x3DD00000 is the SEC console -- the same port IpodSec prints its
 * early boot messages on, and the one the Linux port uses as stdout. It is
 * the debug lifeline for this whole bring-up, so it is deliberately kept
 * dead simple: polled TX, no interrupts, no ring buffer, nothing that can
 * itself fail while something else is being debugged.
 *
 * Registers are the Samsung s5l/s3c layout that firmware/drivers uses for
 * the other S5L parts.
 */
#define ULCON       (*(REG32_PTR_T)(UART3_BASE + 0x00))
#define UCON        (*(REG32_PTR_T)(UART3_BASE + 0x04))
#define UFCON       (*(REG32_PTR_T)(UART3_BASE + 0x08))
#define UMCON       (*(REG32_PTR_T)(UART3_BASE + 0x0c))
#define UTRSTAT     (*(REG32_PTR_T)(UART3_BASE + 0x10))
#define UERSTAT     (*(REG32_PTR_T)(UART3_BASE + 0x14))
#define UFSTAT      (*(REG32_PTR_T)(UART3_BASE + 0x18))
#define UMSTAT      (*(REG32_PTR_T)(UART3_BASE + 0x1c))
#define UTXH        (*(REG32_PTR_T)(UART3_BASE + 0x20))
#define URXH        (*(REG32_PTR_T)(UART3_BASE + 0x24))
#define UBRDIV      (*(REG32_PTR_T)(UART3_BASE + 0x28))
#define UDIVSLOT    (*(REG32_PTR_T)(UART3_BASE + 0x2c))

#define UTRSTAT_TX_EMPTY    (1 << 2)
#define UTRSTAT_TXBUF_EMPTY (1 << 1)
#define UTRSTAT_RX_READY    (1 << 0)

/* UART reference clock is the 24 MHz external clock (DT: nclk). */
#define UART_CLK        24000000
#define UART_BAUD       115200

/*
 * Long enough that a wedged UART cannot hang the whole firmware, short enough
 * that it is invisible when the port is healthy.
 */
#define TX_SPIN_LIMIT   100000

void uart_init(void)
{
    /*
     * U-Boot has already configured and used this port, so the pinmux and
     * clocking are live. Reprogramming the frame format and divider is still
     * worth doing: it makes the port independent of whatever the previous
     * stage left behind, and costs nothing.
     */
    ULCON = 0x03;               /* 8N1 */
    UCON  = 0x05;               /* TX/RX polling mode, no interrupts */
    UFCON = 0x00;               /* FIFOs off -- simplest possible TX path */
    UMCON = 0x00;               /* no flow control */

    UBRDIV = (UART_CLK / (UART_BAUD * 16)) - 1;
    UDIVSLOT = 0;
}

int tx_rdy(void)
{
    return (UTRSTAT & UTRSTAT_TXBUF_EMPTY) ? 1 : 0;
}

void tx_writec(unsigned char c)
{
    int spin = TX_SPIN_LIMIT;

    while (!(UTRSTAT & UTRSTAT_TXBUF_EMPTY)) {
        if (--spin <= 0)
            return;             /* drop the byte rather than hang */
    }

    UTXH = c;
}

void serial_setup(void)
{
    uart_init();
}

void serial_bitrate(int rate)
{
    if (rate <= 0)
        return;

    UBRDIV = (UART_CLK / (rate * 16)) - 1;
    UDIVSLOT = 0;
}

int remote_control_rx(void)
{
    return 0;
}
