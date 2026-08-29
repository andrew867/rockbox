/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * LCDIF driver for the iPod nano 7G (N31).
 *
 * Ported from tools/linux-n31/s5l8740.c (TinyDRM), which reconstructed the
 * sequence from the stock firmware.
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
#include "lcd.h"
#include "lcd-target.h"
#include "boot-beacon.h"

/*
 * ---------------------------------------------------------------------
 * PERFORMANCE WARNING
 *
 * This is the single-pixel programmed-I/O path: every pixel is one status
 * poll plus one 32-bit store to N31_LCD_WDATA. A full 240x432 frame is 103,680
 * of those pairs.
 *
 * That is deliberate for now -- it is the only path proven to put pixels on
 * the panel, and a DMA/accelerated replacement is being developed separately.
 * lcd_update_rect() is written so that only the dirty rectangle is pushed,
 * which is what makes the UI tolerable until the fast path lands.
 *
 * When the DMA driver arrives, lcd_update_rect() is the only function that
 * should need to change.
 * ---------------------------------------------------------------------
 */

#define N31_LCD_CON         (*(REG32_PTR_T)(LCDIF_BASE + 0x00))
#define N31_LCD_WCMD        (*(REG32_PTR_T)(LCDIF_BASE + 0x04))
#define N31_LCD_RCMD        (*(REG32_PTR_T)(LCDIF_BASE + 0x0c))
#define N31_LCD_RDATA       (*(REG32_PTR_T)(LCDIF_BASE + 0x10))
#define N31_LCD_DBUFF       (*(REG32_PTR_T)(LCDIF_BASE + 0x14))
#define N31_LCD_INTCON      (*(REG32_PTR_T)(LCDIF_BASE + 0x18))
#define N31_LCD_STATUS      (*(REG32_PTR_T)(LCDIF_BASE + 0x1c))
#define N31_LCD_PHTIME      (*(REG32_PTR_T)(LCDIF_BASE + 0x20))
#define N31_LCD_UNK2C       (*(REG32_PTR_T)(LCDIF_BASE + 0x2c))
#define N31_LCD_RESET       (*(REG32_PTR_T)(LCDIF_BASE + 0x30))
#define N31_LCD_WDATA       (*(REG32_PTR_T)(LCDIF_BASE + 0x40))
#define N31_LCD_UNK68       (*(REG32_PTR_T)(LCDIF_BASE + 0x68))
#define N31_LCD_UNK70       (*(REG32_PTR_T)(LCDIF_BASE + 0x70))
#define N31_LCD_SIZE        (*(REG32_PTR_T)(LCDIF_BASE + 0x74))
#define N31_LCD_UNK78       (*(REG32_PTR_T)(LCDIF_BASE + 0x78))
#define N31_LCD_UNK7C       (*(REG32_PTR_T)(LCDIF_BASE + 0x7c))
#define N31_LCD_UNK84       (*(REG32_PTR_T)(LCDIF_BASE + 0x84))
#define N31_LCD_UNKA4       (*(REG32_PTR_T)(LCDIF_BASE + 0xa4))

#define N31_LCD_STATUS_BUSY         0x10
#define N31_LCD_STATUS_RESETTING    0x1000

#define CON_HOLD        (1 << 10)
#define CON_RUN         (1 << 11)
#define CON_BASE        0x00100ab0
#define CON_MODE_MASK   0xc0000000
#define CON_FMT_MASK    0x00000007

/* CLKCON gates that have to be cycled across an LCDIF reset. */
#define CLKCON_08       (*(REG32_PTR_T)(CLKCON_BASE + 0x08))
#define CLKCON_18       (*(REG32_PTR_T)(CLKCON_BASE + 0x18))
#define CLKCON_08_MASK  0x7fff7fff
#define CLKCON_18_MASK  0xffff3fff

#define PIXEL_TIMEOUT_US    100000
#define RESET_TIMEOUT_US    500000

static bool lcd_on = true;

/*
 * Iteration backstop for every wait in this file.
 *
 * lcdif_reset() gates the LCDIF clocks OFF and then waits for the block to
 * update STATUS and to clear its reset bit. A block with no clock may do
 * neither, which means the timeout is not the unlikely path here -- it is the
 * expected one. Every such wait therefore escapes solely via USEC_TIMER, and
 * if that timer does not count, every one of them is infinite.
 *
 * That is what a red bottom band on the boot beacon would be reporting, and
 * it is too much load-bearing weight to rest on an assumed register offset.
 */
#define WAIT_GUARD      50000000u

/* Wait for a status bit to clear; returns false on timeout. */
static bool wait_status_clear(uint32_t mask, unsigned timeout_us)
{
    unsigned stop = USEC_TIMER + timeout_us;
    unsigned guard = WAIT_GUARD;

    while (N31_LCD_STATUS & mask) {
        if (TIME_AFTER(USEC_TIMER, stop))
            return false;
        if (--guard == 0)
            return false;
    }
    return true;
}

/*
 * Reprogram the interface for our mode. Leaves it stopped.
 *
 * Note N31_LCD_SIZE packs height in the low half and width in the high half.
 */
static void lcdif_program(void)
{
    uint32_t con = N31_LCD_CON;
    uint32_t keep = con & (CON_MODE_MASK | CON_FMT_MASK);

    N31_LCD_UNK78 = 0x000a000a;
    N31_LCD_CON = keep | CON_BASE;
    N31_LCD_UNK2C = 1;
    N31_LCD_UNK68 = 0;
    N31_LCD_UNK70 = 0;
    N31_LCD_SIZE = LCD_HEIGHT | (LCD_WIDTH << 16);
    N31_LCD_PHTIME = 0;
    N31_LCD_UNK7C = 770;
    N31_LCD_UNK84 = 100;
    N31_LCD_UNKA4 = 1;
}

static void lcdif_reset(void)
{
    uint32_t clk08, clk18;
    unsigned stop, guard;

    /* An LCDIF reset needs two CLKCON gates cycled around it. */
    clk08 = CLKCON_08;
    clk18 = CLKCON_18;
    CLKCON_08 = clk08 & CLKCON_08_MASK;
    CLKCON_18 = clk18 & CLKCON_18_MASK;

    beacon_mark(BEACON_RED);

    N31_LCD_CON &= ~CON_HOLD;

    wait_status_clear(N31_LCD_STATUS_RESETTING, RESET_TIMEOUT_US);

    beacon_mark(BEACON_ORANGE);

    N31_LCD_RESET = 1;
    stop = USEC_TIMER + RESET_TIMEOUT_US;
    guard = WAIT_GUARD;
    while (N31_LCD_RESET) {
        if (TIME_AFTER(USEC_TIMER, stop))
            break;              /* reset did not ack -- carry on anyway */
        if (--guard == 0)
            break;
    }

    beacon_mark(BEACON_YELLOW);

    /*
     * Poke the hold bit until the interface admits it is held. The stock code
     * loops on this for up to half a second, so we do too.
     */
    stop = USEC_TIMER + RESET_TIMEOUT_US;
    guard = WAIT_GUARD / 1000u;         /* udelay(150) dominates this one */
    while (!(N31_LCD_CON & CON_HOLD)) {
        N31_LCD_CON |= CON_HOLD;
        if (TIME_AFTER(USEC_TIMER, stop))
            break;
        if (--guard == 0)
            break;
        udelay(150);
    }

    beacon_mark(BEACON_GREEN);

    N31_LCD_CON &= ~CON_HOLD;

    CLKCON_08 = clk08;
    CLKCON_18 = clk18;

    beacon_mark(BEACON_CYAN);
}

static void lcdif_run(void)
{
    uint32_t con = N31_LCD_CON;

    if ((con & (CON_MODE_MASK | CON_FMT_MASK)) !=
        (CON_BASE & (CON_MODE_MASK | CON_FMT_MASK))) {
        wait_status_clear(N31_LCD_STATUS_RESETTING, RESET_TIMEOUT_US);
        N31_LCD_CON = (con & 0x3ffffff8) | CON_BASE;
    }
    N31_LCD_CON |= CON_RUN;
}

void lcd_init_device(void)
{
    /*
     * Adopt the running interface. Deliberately touch nothing.
     *
     * This used to call lcdif_reset() + lcdif_program() + lcdif_run(), and it
     * hung on the first one, every time. The reason is worth writing down,
     * because the fix is "do less" and that always looks like giving up:
     *
     * We do not need any of it. U-Boot hands over with the LCDIF clocked,
     * running and scanning out its logo, and the assembly beacons in crt0
     * prove pixels land with zero setup -- two MMIO stores, before the C
     * runtime exists. The reset sequence existed to reach a state we are
     * already in.
     *
     * And it cannot be done safely from here. lcdif_reset() gates the LCDIF
     * clocks off and then reads the block's registers; an MMIO read from a
     * block with no clock stalls the bus, which is a hang inside a single
     * load instruction that no loop guard or timeout can break out of. That
     * is exactly what the beacon reported: red top (gates dropped) with a
     * GREEN bottom band confirming USEC_TIMER was counting fine the whole
     * time. The timeouts were healthy; the code never got back to them.
     *
     * Linux runs this same sequence successfully, which is why it was ported
     * faithfully -- but it resets an idle LCDIF that DRM has already shut
     * down, not one mid-scanout. The sequence is correct; performing it on a
     * live interface is not.
     *
     * lcdif_run() is skipped for a second, independent reason: it rewrites
     * CON when the MODE/FMT bits do not match CON_BASE, and CON_BASE asks for
     * zero in both. The format currently in CON is the one the panel is
     * demonstrably working with -- XRGB8888, inherited from the bootloader
     * and never programmed by Linux either. Overwriting it would trade a
     * known-good display for an assumption.
     *
     * The full path stays for lcd_power(), where the block really has been
     * stopped and a reset is both safe and necessary.
     *
     * TODO: the display rail is a PMIC control (see lcd_manage_rail in the
     * Linux driver). Until the D1830 driver lands in Phase 3 we rely on
     * whatever the previous stage left enabled.
     */
    lcd_on = true;

    /*
     * The real driver owns the panel from here.
     *
     * Deliberately NOT solid BLUE, which is what this used to be: main.c
     * already paints solid BLUE when kernel_init() returns, so the two most
     * widely separated points in the boot were reporting the same thing. The
     * split is unambiguous against every solid stage before it.
     */
    beacon_split(BEACON_WHITE, BEACON_GREEN);
}

/*
 * The panel wants XRGB8888; Rockbox draws in RGB565.
 *
 * LCD_WDATA takes one 32-bit XRGB8888 word per pixel. It does NOT take the
 * RGB565 this port uses as its framebuffer format, and it does not convert:
 * storing a raw fb_data here makes the hardware read the two 565 halves as
 * green and blue and paint something unrelated to what was drawn. The early
 * boot beacons hit exactly that -- an RGB565 white came out cyan -- which is
 * what surfaced this.
 *
 * Note the LCDIF format field is never programmed, here or in the Linux
 * driver: both preserve whatever the bootloader left in CON. So the format is
 * an inherited fact about the handoff rather than a choice either driver gets
 * to make, and matching it is not optional.
 *
 * The low bits are replicated rather than zero-filled so that full-scale
 * inputs reach full-scale outputs -- 0x1f must become 0xff, not 0xf8, or
 * whites come out dingy and the whole range is compressed.
 */
static inline uint32_t n31_rgb565_to_xrgb8888(unsigned p)
{
    unsigned r = (p >> 11) & 0x1f;
    unsigned g = (p >> 5)  & 0x3f;
    unsigned b =  p        & 0x1f;

    r = (r << 3) | (r >> 2);
    g = (g << 2) | (g >> 4);
    b = (b << 3) | (b >> 2);

    return (r << 16) | (g << 8) | b;
}

void lcd_update_rect(int x, int y, int width, int height)
{
    const fb_data *fb = FBADDR(0, 0);
    int px, py;

    if (!lcd_on)
        return;

    if (x < 0) { width += x; x = 0; }
    if (y < 0) { height += y; y = 0; }
    if (x + width > LCD_WIDTH)
        width = LCD_WIDTH - x;
    if (y + height > LCD_HEIGHT)
        height = LCD_HEIGHT - y;
    if (width <= 0 || height <= 0)
        return;

    for (py = y; py < y + height; py++) {
        const fb_data *row = fb + py * LCD_WIDTH;

        for (px = x; px < x + width; px++) {
            unsigned spin = PIXEL_TIMEOUT_US;

            /*
             * One poll and one store per pixel. See the warning at the top
             * of this file -- this is the slow path by construction.
             */
            while (N31_LCD_STATUS & N31_LCD_STATUS_BUSY) {
                if (--spin == 0)
                    return;     /* interface wedged; give up on this frame */
            }

            N31_LCD_WDATA = n31_rgb565_to_xrgb8888(row[px]);
        }
    }
}

void lcd_update(void)
{
    lcd_update_rect(0, 0, LCD_WIDTH, LCD_HEIGHT);
}

void lcd_power(bool on)
{
    if (on == lcd_on)
        return;

    if (on) {
        lcdif_reset();
        lcdif_program();
        lcdif_run();
    } else {
        N31_LCD_CON &= ~CON_RUN;
    }
    lcd_on = on;
}

#ifdef HAVE_LCD_ENABLE
void lcd_enable(bool on)
{
    lcd_power(on);
    if (on)
        lcd_update();
}

bool lcd_active(void)
{
    return lcd_on;
}
#endif

#ifdef HAVE_LCD_SHUTDOWN
void lcd_shutdown(void)
{
    lcd_power(false);
}
#endif
