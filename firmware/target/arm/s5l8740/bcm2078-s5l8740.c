/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * BCM2078 HCI transport over UART1 for Apple S5L8740 (iPod nano 7G).
 *
 * Ported from tools/linux-n31/drivers/bcm2078-bt.c.
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
#include "gpio-s5l8740.h"
#include "bcm2078-s5l8740.h"

/*
 * The BCM2078KUBG is a combined Bluetooth + FM part on UART1 @0x3DB00000,
 * speaking H4 HCI. Rockbox has no Bluetooth stack, so this is not a BT
 * driver: it is the transport plus exactly enough HCI to drive the FM tuner,
 * which is the part of the chip this device actually needs.
 *
 * Power-up is a single GPIO. sphwBluetooth_Init (sub_570054) does
 *
 *     sub_43D38C(0x46u, 1, 1);
 *
 * i.e. pad 70 as an output driven high, before any HCI traffic. Without it
 * the controller never answers and HCI_Reset times out.
 *
 * The controller boots at 115200 and is immediately moved to 2.4 Mbaud by
 * vendor command 0xFC18 with payload 00 00 00 9F 24 00 (0x00249F00). Leaving
 * it at 115200 runs the link eight times slower than stock.
 *
 * NOTE: GPIO 97/98/119 are NOT Bluetooth control lines. They are the IIS2 PCM
 * pads and belong to iis2-s5l8740.c. Listing them here made the Linux driver
 * drive the capture bus as GPIOs.
 */

#define ULCON       (*(REG32_PTR_T)(UART1_BASE + 0x00))
#define UCON        (*(REG32_PTR_T)(UART1_BASE + 0x04))
#define UFCON       (*(REG32_PTR_T)(UART1_BASE + 0x08))
#define UMCON       (*(REG32_PTR_T)(UART1_BASE + 0x0c))
#define UTRSTAT     (*(REG32_PTR_T)(UART1_BASE + 0x10))
#define UFSTAT      (*(REG32_PTR_T)(UART1_BASE + 0x18))
#define UTXH        (*(REG32_PTR_T)(UART1_BASE + 0x20))
#define URXH        (*(REG32_PTR_T)(UART1_BASE + 0x24))
#define UBRDIV      (*(REG32_PTR_T)(UART1_BASE + 0x28))
#define UDIVSLOT    (*(REG32_PTR_T)(UART1_BASE + 0x2c))

#define UTRSTAT_TXBUF_EMPTY (1 << 1)
#define UTRSTAT_RX_READY    (1 << 0)

#define UART_CLK        24000000
#define BAUD_BOOT       115200
#define BAUD_FAST       2400000

/* H4 packet types */
#define H4_CMD          0x01
#define H4_ACL          0x02
#define H4_EVT          0x04

#define HCI_EV_CMD_COMPLETE 0x0e

#define HCI_OP_RESET        0x0c03
#define HCI_OP_BAUD         0xfc18
#define HCI_OP_FM           0xfc15

#define RX_TIMEOUT_MS   1000

static bool bt_up;

static void uart_set_baud(unsigned baud)
{
    UBRDIV = (UART_CLK / (baud * 16)) - 1;
    UDIVSLOT = 0;
}

static void uart_setup(unsigned baud)
{
    ULCON = 0x03;       /* 8N1 */
    UCON  = 0x05;       /* polled */
    UFCON = 0x00;
    UMCON = 0x00;
    uart_set_baud(baud);
}

static void uart_putc(uint8_t c)
{
    unsigned spin = 100000;

    while (!(UTRSTAT & UTRSTAT_TXBUF_EMPTY)) {
        if (--spin == 0)
            return;
    }
    UTXH = c;
}

static int uart_getc_timeout(unsigned timeout_us)
{
    unsigned stop = USEC_TIMER + timeout_us;

    while (!(UTRSTAT & UTRSTAT_RX_READY)) {
        if (TIME_AFTER(USEC_TIMER, stop))
            return -1;
    }
    return (int)(URXH & 0xff);
}

static void hci_send_cmd(uint16_t opcode, const uint8_t *param, int plen)
{
    int i;

    uart_putc(H4_CMD);
    uart_putc(opcode & 0xff);
    uart_putc(opcode >> 8);
    uart_putc(plen);

    for (i = 0; i < plen; i++)
        uart_putc(param[i]);
}

/*
 * Wait for a Command Complete for the given opcode and return its status
 * byte, or negative on timeout. Events for other opcodes are drained rather
 * than treated as errors -- the controller emits unsolicited events and
 * mistaking one for a reply desynchronises the stream.
 */
static int hci_wait_complete(uint16_t opcode, uint8_t *ret_param, int ret_len)
{
    unsigned deadline = USEC_TIMER + RX_TIMEOUT_MS * 1000;

    while (!TIME_AFTER(USEC_TIMER, deadline)) {
        int c = uart_getc_timeout(RX_TIMEOUT_MS * 1000);
        int evt, plen, i;
        uint16_t op;

        if (c < 0)
            return -1;
        if (c != H4_EVT)
            continue;

        evt = uart_getc_timeout(RX_TIMEOUT_MS * 1000);
        plen = uart_getc_timeout(RX_TIMEOUT_MS * 1000);
        if (evt < 0 || plen < 0)
            return -1;

        if (evt != HCI_EV_CMD_COMPLETE || plen < 3) {
            while (plen-- > 0)
                uart_getc_timeout(RX_TIMEOUT_MS * 1000);
            continue;
        }

        (void)uart_getc_timeout(RX_TIMEOUT_MS * 1000);   /* ncmd */
        op = uart_getc_timeout(RX_TIMEOUT_MS * 1000);
        op |= uart_getc_timeout(RX_TIMEOUT_MS * 1000) << 8;
        plen -= 3;

        for (i = 0; i < plen; i++) {
            int b = uart_getc_timeout(RX_TIMEOUT_MS * 1000);

            if (b < 0)
                return -1;
            if (ret_param && i < ret_len)
                ret_param[i] = (uint8_t)b;
        }

        if (op == opcode)
            return (plen > 0 && ret_param) ? ret_param[0] : 0;
    }

    return -1;
}

bool bcm2078_init(void)
{
    static const uint8_t baud_param[] = { 0x00, 0x00, 0x00, 0x9f, 0x24, 0x00 };
    uint8_t status[8];

    if (bt_up)
        return true;

    /* sub_570054: pad 70 high powers the controller. */
    gpio_direction_output(GPIO_PAD_BT_POWER, true);
    sleep(HZ / 10);

    uart_setup(BAUD_BOOT);

    hci_send_cmd(HCI_OP_RESET, NULL, 0);
    if (hci_wait_complete(HCI_OP_RESET, status, sizeof(status)) < 0)
        return false;

    /* Vendor baud change, then follow it on our side. */
    hci_send_cmd(HCI_OP_BAUD, baud_param, sizeof(baud_param));
    if (hci_wait_complete(HCI_OP_BAUD, status, sizeof(status)) < 0)
        return false;

    uart_set_baud(BAUD_FAST);
    sleep(HZ / 50);

    /*
     * TODO: patchram. The stock path uploads brcm/BCM2076B1.hcd before the
     * controller is fully functional. The blob ships in
     * firmware/BCM2076B1.hcd; loading it needs a filesystem, which this port
     * does not have until the FTL lands (Phase 5). FM has been observed to
     * respond without it, so this is not a hard blocker for the tuner.
     */

    bt_up = true;
    return true;
}

void bcm2078_shutdown(void)
{
    if (!bt_up)
        return;

    gpio_direction_output(GPIO_PAD_BT_POWER, false);
    bt_up = false;
}

bool bcm2078_available(void)
{
    return bt_up;
}

int bcm2078_fm_cmd(const uint8_t *payload, int len, uint8_t *reply, int reply_len)
{
    if (!bt_up)
        return -1;

    hci_send_cmd(HCI_OP_FM, payload, len);
    return hci_wait_complete(HCI_OP_FM, reply, reply_len);
}
