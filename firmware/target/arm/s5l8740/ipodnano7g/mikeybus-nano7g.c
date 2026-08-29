/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Apple MikeyBus (headset identity and inline remote) for the iPod nano 7G.
 *
 * Ported from tools/linux-n31/drivers/apple-mikeybus.c and
 * docs-internal/n7g-mikeybus/N31-MIKEYBUS-FOR-DUMMIES-v3.md.
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
#include "button.h"
#include "gpio-s5l8740.h"
#include "mikeybus-nano7g.h"

/*
 * MikeyBus is the sideband on the 3.5 mm jack that tells the device which
 * headset is plugged in and carries the inline remote's buttons. It is a
 * packet protocol on UART2, not a resistor divider on a GPIO.
 *
 * RetailOS models it with two tasks -- CMikeyBusUartReadTask and
 * CMikeyBusUartResistorTask -- over two channels:
 *
 *   channel 3   headset model / "resistor" sample   (command 3 / 0x8D)
 *   channel 4   remote byte stream                  (command 9 / 0x71)
 *
 * WHAT IS PROVEN (decompilation, and matching the Linux driver):
 *
 *   - the packet layout and the four lower packet types
 *   - the 0xAA -> emit an extra 0x01 stream rule
 *   - the headset model byte table, including which values mean "nothing
 *     plugged in"
 *   - the two command frames above
 *
 * WHAT IS NOT PROVEN, and is flagged at each use below:
 *
 *   - the UART baud rate and frame format. The device tree says 115200 and
 *     that is what this uses, but the RE notes are explicit that no baud has
 *     been promoted from the firmware.
 *   - the raw byte codes for individual remote buttons. RetailOS's event
 *     names are known (HandleMikeyCenter, HandleMikeyVolumeUp,
 *     HandleMikeyVolumeDown, HandleMikeyAllUp) but the wire encoding that
 *     produces them is not.
 *
 * NEVER do resistor detection on GPIO 66/67. Those are the UART2 TX/RX pads
 * and nothing else; sampling them as a model-detect divider is a mistake this
 * project has already made once.
 */

#define ULCON       (*(REG32_PTR_T)(UART2_BASE + 0x00))
#define UCON        (*(REG32_PTR_T)(UART2_BASE + 0x04))
#define UFCON       (*(REG32_PTR_T)(UART2_BASE + 0x08))
#define UMCON       (*(REG32_PTR_T)(UART2_BASE + 0x0c))
#define UTRSTAT     (*(REG32_PTR_T)(UART2_BASE + 0x10))
#define UTXH        (*(REG32_PTR_T)(UART2_BASE + 0x20))
#define URXH        (*(REG32_PTR_T)(UART2_BASE + 0x24))
#define UBRDIV      (*(REG32_PTR_T)(UART2_BASE + 0x28))
#define UDIVSLOT    (*(REG32_PTR_T)(UART2_BASE + 0x2c))

#define UTRSTAT_TXBUF_EMPTY (1 << 1)
#define UTRSTAT_RX_READY    (1 << 0)

#define UART_CLK        24000000
/*
 * ASSUMPTION, not RE: the device tree carries current-speed = 115200 and the
 * Linux driver defaults there, but no baud has been recovered from the stock
 * firmware. If the headset never identifies, this is the first thing to
 * doubt -- the packet layer below is well grounded, the wire rate is not.
 */
#define MIKEY_BAUD      115200

/* Lower packet types (sub_500ECC). */
#define PKT_RX_BYTES    0x70
#define PKT_IGNORED_74  0x74
#define PKT_STATUS_76   0x76
#define PKT_STATUS_8A   0x8a

#define MIKEY_CH_RESISTOR   3
#define MIKEY_CH_READ       4

/* Model sample values that mean "nothing is plugged in". */
#define SAMPLE_OPEN_CIRCUIT 0x0b
#define SAMPLE_DEFAULT      0x64
/* sub_410DB0: sample 15 with a zero modifier is remapped to 100. */
#define SAMPLE_REMAP_FROM   0x0f
#define SAMPLE_REMAP_TO     0x64

#define RX_RING_SIZE    256
#define PKT_MAX         64

struct mikey_state {
    bool     up;
    bool     plugged;
    uint8_t  model;
    uint8_t  model_sample;

    /* Assembled remote byte stream (post 0xAA rule). */
    uint8_t  stream[RX_RING_SIZE];
    unsigned stream_head;

    /* Packet reassembly. */
    uint8_t  pkt[PKT_MAX];
    unsigned pkt_len;

    /* Latest decoded remote button bitmap. */
    uint8_t  remote_bits;

    unsigned rx_bytes;
    unsigned packets;
};

static struct mikey_state mikey;

const char *mikeybus_model_name(uint8_t model)
{
    switch (model) {
    case 0x01: return "A18";
    case 0x02: return "B18";
    case 0x03: return "A62";
    case 0x04: return "B15";
    case 0x05: return "A36";
    case 0x06: return "Apple noise occluding";
    case 0x07: return "mfg noise occluding";
    case 0x08: return "mfg noise occluding w/ mic";
    case 0x09: return "mfg std";
    case 0x0a: return "mfg std w/ mic";
    case 0x0b: return "open circuit";
    case 0x0d: return "B60f";
    case 0x0e: return "B60g";
    case 0x0f: return "B149";
    case 0x10: return "B187";
    case 0x64: return "default/open/unknown";
    default:   return "inscrutable";
    }
}

static bool sample_is_plugged(uint8_t sample)
{
    /* Both of these mean "no headset", not "an unusual headset". */
    return sample != SAMPLE_OPEN_CIRCUIT && sample != SAMPLE_DEFAULT;
}

/* ------------------------------------------------------------------ UART */

static void uart_setup(void)
{
    /*
     * GPIO 66/67 at function 2 is the UART2 pad mux and NOTHING else. They
     * are not DIN lines and must never be sampled as a resistor divider.
     */
    gpio_set_function(GPIO_PAD_MIKEY_A, GPIO_FUNC_ALT2);
    gpio_set_function(GPIO_PAD_MIKEY_B, GPIO_FUNC_ALT2);

    ULCON = 0x03;       /* 8N1 -- also an assumption; see the baud note */
    UCON  = 0x05;       /* polled */
    UFCON = 0x00;
    UMCON = 0x00;

    UBRDIV = (UART_CLK / (MIKEY_BAUD * 16)) - 1;
    UDIVSLOT = 0;
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

static int uart_getc(void)
{
    if (!(UTRSTAT & UTRSTAT_RX_READY))
        return -1;
    return (int)(URXH & 0xff);
}

/* ------------------------------------------------------------- stream ---- */

static void stream_put(uint8_t b)
{
    mikey.stream[mikey.stream_head] = b;
    mikey.stream_head = (mikey.stream_head + 1) % RX_RING_SIZE;
}

/*
 * sub_2542F0: every received byte goes to the stream, and a 0xAA is followed
 * by an extra 0x01. That is an escape rule, so 0xAA cannot be treated as an
 * ordinary data byte by anything downstream.
 */
static void rx_byte(uint8_t b)
{
    stream_put(b);
    if (b == 0xaa)
        stream_put(0x01);

    mikey.rx_bytes++;
}

/*
 * Remote button decode.
 *
 * UNPROVEN. RetailOS's event vocabulary is known from its function names --
 * HandleMikeyCenter, HandleMikeyVolumeUp, HandleMikeyVolumeDown and
 * HandleMikeyAllUp -- and the existence of an "all up" event says the wire
 * carries a bitmap of currently-held buttons rather than discrete press and
 * release codes. Everything past that is inference:
 *
 *   bit 0  centre / play-pause
 *   bit 1  volume up
 *   bit 2  volume down
 *   zero   all released
 *
 * That is the ordering the event names are listed in and the conventional
 * Apple three-button remote layout, but it has not been confirmed against a
 * real remote. If the buttons come out permuted, this function is the only
 * thing that needs changing -- and mikeybus_raw_stream() exists so the true
 * codes can be read off the device in one sitting.
 */
#define REMOTE_CENTER   (1 << 0)
#define REMOTE_VOL_UP   (1 << 1)
#define REMOTE_VOL_DOWN (1 << 2)

static void decode_remote(uint8_t b)
{
    mikey.remote_bits = b & (REMOTE_CENTER | REMOTE_VOL_UP | REMOTE_VOL_DOWN);
}

/* ------------------------------------------------------------- packets --- */

/*
 * Lower packet layout (sub_500ECC):
 *   pkt[0]  total length
 *   pkt[1]  type
 *   pkt[2]  channel
 *   pkt[3+] payload
 */
static void handle_packet(const uint8_t *pkt, unsigned len)
{
    if (len < 2)
        return;

    mikey.packets++;

    switch (pkt[1]) {
    case PKT_RX_BYTES: {
        unsigned count, i;

        if (len < 3 || pkt[0] < 3 || pkt[0] > len)
            return;

        count = pkt[0] - 3;
        for (i = 0; i < count; i++)
            rx_byte(pkt[3 + i]);

        /* The remote's state is the most recent payload byte. */
        if (count)
            decode_remote(pkt[3 + count - 1]);
        break;
    }

    case PKT_IGNORED_74:
        /* Deliberately ignored; the stock lower handler does the same. */
        break;

    case PKT_STATUS_76:
    case PKT_STATUS_8A:
        if (len < 4)
            return;
        /*
         * Presence/status. Bit 0x10 and bit 0x20 move a shadow byte in the
         * stock code; what that shadow drives is not established, so this
         * only uses it as a hint that something changed.
         */
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------- commands -- */

/*
 * RetailOS command object, as recovered:
 *   cmd[3] = 0xFF
 *   cmd[6] = class
 *   cmd[7] = opcode
 *   cmd[8] = channel
 *
 * Whether the lower transport takes these bytes verbatim is NOT established
 * -- the Linux driver keeps this path behind a flag for exactly that reason.
 * It is sent here because a passive listen never makes the headset identify
 * itself, but a failure to answer should be read as "framing unconfirmed"
 * rather than "no headset".
 */
static void send_command(uint8_t cls, uint8_t opcode, uint8_t channel)
{
    uint8_t cmd[9] = { 0, 0, 0, 0xff, 0, 0, cls, opcode, channel };
    unsigned i;

    for (i = 0; i < sizeof(cmd); i++)
        uart_putc(cmd[i]);
}

static void request_model(void)
{
    send_command(3, 0x8d, MIKEY_CH_RESISTOR);
}

static void open_read_channel(void)
{
    send_command(9, 0x71, MIKEY_CH_READ);
}

/* ------------------------------------------------------------- public ---- */

bool mikeybus_init(void)
{
    memset(&mikey, 0, sizeof(mikey));

    uart_setup();

    open_read_channel();
    request_model();

    mikey.up = true;
    mikey.model_sample = SAMPLE_DEFAULT;
    mikey.model = SAMPLE_DEFAULT;

    return true;
}

void mikeybus_poll(void)
{
    int c;

    if (!mikey.up)
        return;

    while ((c = uart_getc()) >= 0) {
        if (mikey.pkt_len < PKT_MAX)
            mikey.pkt[mikey.pkt_len++] = (uint8_t)c;

        /*
         * pkt[0] is the total length, so a packet is complete once that many
         * bytes have arrived. A length byte of 0 would never complete, so
         * treat it as a desync and restart.
         */
        if (mikey.pkt_len >= 1 && mikey.pkt[0] == 0) {
            mikey.pkt_len = 0;
            continue;
        }

        if (mikey.pkt_len >= 1 && mikey.pkt_len >= mikey.pkt[0]) {
            handle_packet(mikey.pkt, mikey.pkt_len);
            mikey.pkt_len = 0;
        }

        if (mikey.pkt_len >= PKT_MAX)
            mikey.pkt_len = 0;
    }
}

void mikeybus_set_model_sample(uint8_t sample, uint8_t modifier)
{
    /* sub_410DB0: sample 15 with a zero modifier is remapped to 100. */
    if (sample == SAMPLE_REMAP_FROM && modifier == 0)
        sample = SAMPLE_REMAP_TO;

    mikey.model_sample = sample;
    mikey.model = sample;
    mikey.plugged = sample_is_plugged(sample);
}

bool mikeybus_jack_present(void)
{
    return mikey.plugged;
}

uint8_t mikeybus_model(void)
{
    return mikey.model;
}

int mikeybus_read_buttons(void)
{
    int btn = 0;

    if (!mikey.up || !mikey.plugged)
        return 0;

    if (mikey.remote_bits & REMOTE_CENTER)
        btn |= BUTTON_PLAY;
    if (mikey.remote_bits & REMOTE_VOL_UP)
        btn |= BUTTON_VOL_UP;
    if (mikey.remote_bits & REMOTE_VOL_DOWN)
        btn |= BUTTON_VOL_DOWN;

    return btn;
}

const uint8_t *mikeybus_raw_stream(unsigned *head)
{
    if (head)
        *head = mikey.stream_head;
    return mikey.stream;
}

void mikeybus_get_counters(unsigned *rx_bytes, unsigned *packets)
{
    if (rx_bytes)
        *rx_bytes = mikey.rx_bytes;
    if (packets)
        *packets = mikey.packets;
}
