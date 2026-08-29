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

/* Wait for a status bit to clear; returns false on timeout. */
static bool wait_status_clear(uint32_t mask, unsigned timeout_us)
{
    unsigned stop = USEC_TIMER + timeout_us;

    while (N31_LCD_STATUS & mask) {
        if (TIME_AFTER(USEC_TIMER, stop))
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
    unsigned stop;

    /* An LCDIF reset needs two CLKCON gates cycled around it. */
    clk08 = CLKCON_08;
    clk18 = CLKCON_18;
    CLKCON_08 = clk08 & CLKCON_08_MASK;
    CLKCON_18 = clk18 & CLKCON_18_MASK;

    N31_LCD_CON &= ~CON_HOLD;

    wait_status_clear(N31_LCD_STATUS_RESETTING, RESET_TIMEOUT_US);

    N31_LCD_RESET = 1;
    stop = USEC_TIMER + RESET_TIMEOUT_US;
    while (N31_LCD_RESET) {
        if (TIME_AFTER(USEC_TIMER, stop))
            break;              /* reset did not ack -- carry on anyway */
    }

    /*
     * Poke the hold bit until the interface admits it is held. The stock code
     * loops on this for up to half a second, so we do too.
     */
    stop = USEC_TIMER + RESET_TIMEOUT_US;
    while (!(N31_LCD_CON & CON_HOLD)) {
        N31_LCD_CON |= CON_HOLD;
        if (TIME_AFTER(USEC_TIMER, stop))
            break;
        udelay(150);
    }

    N31_LCD_CON &= ~CON_HOLD;

    CLKCON_08 = clk08;
    CLKCON_18 = clk18;
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
     * U-Boot has already brought the panel up and is showing something, so
     * the rail is live and the LCDIF is running. Reprogramming it for our
     * geometry is enough; a full off/on cycle would blank the screen for no
     * benefit during boot.
     *
     * TODO: the display rail is a PMIC control (see lcd_manage_rail in the
     * Linux driver). Until the D1830 driver lands in Phase 3 we rely on
     * whatever the previous stage left enabled, which is why lcd_enable()
     * below only stops and starts the interface.
     */
    lcdif_reset();
    lcdif_program();
    lcdif_run();
    lcd_on = true;
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

            N31_LCD_WDATA = row[px];
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
