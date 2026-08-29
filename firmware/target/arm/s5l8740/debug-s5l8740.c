/***************************************************************************
 * Debug menu entries for Apple S5L8740 (iPod nano 7G).
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#include <stdbool.h>
#include "config.h"
#include "system.h"
#include "lcd.h"
#include "font.h"
#include "action.h"
#include "button.h"
#include "gpio-s5l8740.h"
#include "pmu-target.h"
#include "mikeybus-nano7g.h"
#include "touch-nano7g.h"
#include "tristar-nano7g.h"
#include "spi-s5l8740.h"
#include "iis-s5l8740.h"
#include "ftl-s5l8740.h"
#include "audiohw.h"

/*
 * Deliberately read-only. During bring-up the debug screens are one of the few
 * ways to see hardware state on the device itself, and a stray write here --
 * to a GPIO pad or a PMIC register -- is exactly the kind of thing that ends a
 * debugging session early.
 */
bool dbg_hw_info(void)
{
    int line;

    lcd_setfont(FONT_SYSFIXED);

    while (1) {
        line = 0;
        lcd_clear_display();

        lcd_putsf(0, line++, "CPU: S5L8740 (Cortex-A5)");
        lcd_putsf(0, line++, "DRAM: %d MB @ %08x", MEMORYSIZE, DRAM_ORIG);
        line++;

        lcd_putsf(0, line++, "VIC0 IRQ:  %08lx", (unsigned long)VIC0IRQSTATUS);
        lcd_putsf(0, line++, "VIC1 IRQ:  %08lx", (unsigned long)VIC1IRQSTATUS);
        line++;

        lcd_putsf(0, line++, "Vol+ pad %d: %d", GPIO_PAD_VOLUP,
                  gpio_get(GPIO_PAD_VOLUP));
        lcd_putsf(0, line++, "Vol- pad %d: %d", GPIO_PAD_VOLDOWN,
                  gpio_get(GPIO_PAD_VOLDOWN));
        line++;

        lcd_putsf(0, line++, "PMIC r5:  %02x", pmu_read(0x05));
        lcd_putsf(0, line++, "PMIC r7:  %02x", pmu_read(0x07));
        lcd_putsf(0, line++, "PMIC r8:  %02x", pmu_read(0x08));
        lcd_putsf(0, line++, "VBAT:     %d mV", pmu_read_battery_voltage());

        lcd_update();

        if (action_userabort(HZ / 5))
            break;
    }

    lcd_setfont(FONT_UI);
    return false;
}

/*
 * MikeyBus raw stream.
 *
 * This screen exists for one specific job: the inline remote's button codes
 * are inference rather than RE, and this shows the actual bytes arriving so
 * they can be read off a device with a remote plugged in. Press each button
 * in turn and the encoding falls out in a couple of minutes.
 */
bool dbg_mikeybus(void)
{
    lcd_setfont(FONT_SYSFIXED);

    while (1) {
        const uint8_t *stream;
        unsigned head, rx, pkts;
        int line = 0;
        int i;

        mikeybus_poll();

        lcd_clear_display();
        lcd_putsf(0, line++, "MikeyBus (UART2)");

        mikeybus_get_counters(&rx, &pkts);
        lcd_putsf(0, line++, "rx=%u packets=%u", rx, pkts);
        lcd_putsf(0, line++, "jack=%d model=%02x",
                  mikeybus_jack_present(), mikeybus_model());
        lcd_putsf(0, line++, "%s", mikeybus_model_name(mikeybus_model()));
        line++;

        lcd_putsf(0, line++, "last 32 stream bytes:");
        stream = mikeybus_raw_stream(&head);
        for (i = 0; i < 4; i++) {
            int j;
            char buf[32];
            char *p = buf;

            for (j = 0; j < 8; j++) {
                unsigned idx = (head + 256 - 32 + i * 8 + j) % 256;

                p += snprintf(p, 4, "%02x ", stream[idx]);
            }
            lcd_putsf(0, line++, "%s", buf);
        }

        line++;
        lcd_putsf(0, line++, "Lightning: %s", tristar_accessory_name());
        line++;

        /*
         * Touch state, worded so the two failure modes cannot be confused:
         * a bootloader status word means the part is alive but its runtime
         * never started, which is a firmware/EXEC problem, not wiring.
         */
        {
            uint16_t bl = touch_bootloader_status();

            lcd_putsf(0, line++, "Touch up=%d hbpp=%d",
                      touch_available(), touch_hbpp_status());
            if (bl)
                lcd_putsf(0, line++, "  BOOTLOADER %04x (fw not run)", bl);
            else
                lcd_putsf(0, line++, "  ping fails=%u", touch_ping_failures());
        }
        lcd_putsf(0, line++, "SPI0 fam=%d SPI2 fam=%d",
                  spi_get_status_family(SPI_PORT_CODEC),
                  spi_get_status_family(SPI_PORT_TOUCH));

        lcd_update();

        if (action_userabort(HZ / 5))
            break;
    }

    lcd_setfont(FONT_UI);
    return false;
}

/*
 * Audio route and FTL state.
 *
 * The route registers are the ones the RE guide names as the readback set for
 * diagnosing silence, and IIS0 STATUS is the transport's own health: 0x320 or
 * 0x420 means the interface is running, 0x82A0 meant the wrong parent clock
 * during the Linux bring-up.
 */
bool dbg_audio(void)
{
    lcd_setfont(FONT_SYSFIXED);

    while (1) {
        int line = 0;
        unsigned rng, mapped, closed, open;
        unsigned cur, total;
        const char *phase;
#ifdef HAVE_CS42L81
        struct cs42l81_route rt;

        cs42l81_get_route(&rt);
#endif
        lcd_clear_display();
        lcd_putsf(0, line++, "IIS0 STATUS %08lx", (unsigned long)iis0_status());
        lcd_putsf(0, line++, "(0x320/0x420 = running)");
        line++;

#ifdef HAVE_CS42L81
        lcd_putsf(0, line++, "CS42 lock=%d 002f=%02x", rt.locked, rt.r002f);
        lcd_putsf(0, line++, "0401=%02x 0403=%02x 0404=%02x",
                  rt.r0401, rt.r0403, rt.r0404);
        lcd_putsf(0, line++, "0500=%02x 0527=%02x 054f=%02x",
                  rt.r0500, rt.r0527, rt.r054f);
        lcd_putsf(0, line++, "0075=%02x 0220=%02x", rt.r0075, rt.r0220);
        lcd_putsf(0, line++, "(0527: 60=play ff=mute)");
#endif
        line++;

        phase = ftl_progress_phase(&cur, &total);
        ftl_get_stats(&rng, &mapped, &closed, &open);
        lcd_putsf(0, line++, "FTL %s %u/%u", phase, cur, total);
        lcd_putsf(0, line++, "ranges=%u mapped=%u", rng, mapped);
        lcd_putsf(0, line++, "sb closed=%u open=%u", closed, open);
        lcd_putsf(0, line++, "fat_base=%u", ftl_fat_base_lba());

        lcd_update();

        if (action_userabort(HZ / 5))
            break;
    }

    lcd_setfont(FONT_UI);
    return false;
}

bool dbg_ports(void)
{
    int line;

    lcd_setfont(FONT_SYSFIXED);

    while (1) {
        int bank;

        line = 0;
        lcd_clear_display();
        lcd_putsf(0, line++, "GPIO banks (PCON / PDAT)");

        /* Only the first eight banks fit on screen; pads 0..63. */
        for (bank = 0; bank < 8; bank++) {
            lcd_putsf(0, line++, "%2d: %08lx %08lx", bank,
                      (unsigned long)GPIO_PCON(bank),
                      (unsigned long)GPIO_PDAT(bank));
        }

        lcd_update();

        if (action_userabort(HZ / 5))
            break;
    }

    lcd_setfont(FONT_UI);
    return false;
}
