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
