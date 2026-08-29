/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Dialog D1830 PMIC for the iPod nano 7G (N31).
 *
 * Ported from tools/linux-n31/drivers/gpio-d1830.c.
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
#include "button.h"
#include "i2c-s5l8740.h"
#include "pmu-target.h"

/*
 * The PMIC lives on IIC1 at 7-bit 0x73 (wire 0xE6 write / 0xE7 read). It owns
 * the power rails, the battery ADC, the poweroff cut, and three of the five
 * physical keys.
 */
#define PMU_BUS     I2C_BUS_IIC1
#define PMU_ADDR    I2C_ADDR_PMIC

/* Button state is packed into registers 7 and 8 (OSOS sub_26520). */
#define D1830_REG_BTN0          0x07
#define D1830_BTN0_HOME         (1 << 4)    /* OSOS id 1 */
#define D1830_BTN0_SLEEP        (1 << 5)    /* OSOS id 9 */
#define D1830_REG_BTN1          0x08
#define D1830_BTN1_PLAY         (1 << 1)    /* OSOS id 6, side key */

#define D1830_REG_STATUS0       0x05
#define D1830_STATUS_VBUS       (1 << 6)

#define D1830_REG_POWEROFF      0x0d
#define D1830_POWEROFF_BIT      (1 << 0)
#define D1830_REG_RESTART       0x49    /* 1 = come back after the cut */
#define D1830_REG_CLR_ON_CUT    0x6e

/* Battery ADC */
#define D1830_REG_ADC_CFG       0x30
#define D1830_REG_ADC_LOW       0x31
#define D1830_REG_ADC_HIGH      0x32
#define D1830_ADC_CH_VBAT       3
#define D1830_ADC_START         0x10
#define D1830_ADC_SAMPLES       5

/*
 * Audio rails: SEC sub_23EC brings up sibling LDOs in registers 21-23, bit 4.
 * Phase 4 needs these; nothing before it does.
 */
#define D1830_REG_LDO21         0x15
#define D1830_REG_LDO22         0x16
#define D1830_REG_LDO23         0x17
#define D1830_LDO_ENABLE        (1 << 4)

int pmu_read(int reg)
{
    unsigned char v;

    if (i2c_read(PMU_BUS, PMU_ADDR, reg, 1, &v) < 0)
        return -1;
    return v;
}

int pmu_write(int reg, unsigned char value)
{
    return i2c_write(PMU_BUS, PMU_ADDR, reg, 1, &value);
}

void pmu_init(void)
{
    /*
     * Deliberately does nothing but establish that the chip answers.
     *
     * The Linux driver's probe is GPIO/VBAT read-only for a reason: writing
     * register 13 turns the device off, and replaying the stock hibernate
     * cookie during bring-up strands the pod. Reads only until Phase 3 has
     * proven the rest on glass.
     */
    (void)pmu_read(D1830_REG_STATUS0);
}

int pmu_read_buttons(void)
{
    int btn = 0;
    int r;

    /*
     * Keys are active low: sub_4195D8(id, bit == 0).
     *
     * A failed read returns -1, which must not be mistaken for "every key
     * pressed" -- hence the explicit >= 0 guards.
     */
    r = pmu_read(D1830_REG_BTN0);
    if (r >= 0) {
        if (!(r & D1830_BTN0_HOME))
            btn |= BUTTON_HOME;
        if (!(r & D1830_BTN0_SLEEP))
            btn |= BUTTON_POWER;
    }

    r = pmu_read(D1830_REG_BTN1);
    if (r >= 0) {
        if (!(r & D1830_BTN1_PLAY))
            btn |= BUTTON_PLAY;
    }

    return btn;
}

/*
 * RetailOS VBAT conversion, taken from the firmware's own formula rather than
 * a full-scale guess:
 *
 *     mv = (62 * raw + 207000) / 100
 *
 * It reproduces the stock reference points exactly.
 */
static int d1830_adc_to_mv(int raw)
{
    return (62 * raw + 207000) / 100;
}

int pmu_read_battery_voltage(void)
{
    int lo, hi, raw, i;
    int cfg;

    cfg = pmu_read(D1830_REG_ADC_CFG);
    if (cfg < 0)
        return -1;

    if (pmu_write(D1830_REG_ADC_CFG,
                  (cfg & ~0x0f) | D1830_ADC_CH_VBAT | D1830_ADC_START) < 0)
        return -1;

    /* Conversion is not instant; the stock code polls the start bit. */
    for (i = 0; i < D1830_ADC_SAMPLES; i++) {
        cfg = pmu_read(D1830_REG_ADC_CFG);
        if (cfg >= 0 && !(cfg & D1830_ADC_START))
            break;
        udelay(1000);
    }

    lo = pmu_read(D1830_REG_ADC_LOW);
    hi = pmu_read(D1830_REG_ADC_HIGH);
    if (lo < 0 || hi < 0)
        return -1;

    raw = (hi << 8) | lo;
    return d1830_adc_to_mv(raw);
}

bool pmu_is_usb_present(void)
{
    int r = pmu_read(D1830_REG_STATUS0);

    return (r >= 0) && (r & D1830_STATUS_VBUS);
}

bool pmu_is_charging(void)
{
    /*
     * TODO: the charger state machine in gpio-d1830.c distinguishes charging
     * from merely powered. Until that is ported, VBUS presence is the honest
     * answer to "is external power attached".
     */
    return pmu_is_usb_present();
}

void pmu_audio_rails(bool on)
{
    static const unsigned char regs[] = {
        D1830_REG_LDO21, D1830_REG_LDO22, D1830_REG_LDO23
    };
    unsigned i;

    for (i = 0; i < sizeof(regs); i++) {
        int v = pmu_read(regs[i]);

        if (v < 0)
            continue;
        pmu_write(regs[i], on ? (v | D1830_LDO_ENABLE)
                              : (v & ~D1830_LDO_ENABLE));
    }
}

void pmu_power_off(void)
{
    int v;

    /*
     * Order matters and comes straight from sub_128C: clear-on-cut first,
     * then say whether to come back, and only then pull the plug.
     */
    pmu_write(D1830_REG_CLR_ON_CUT, 0);
    pmu_write(D1830_REG_RESTART, 0);

    v = pmu_read(D1830_REG_POWEROFF);
    if (v >= 0)
        pmu_write(D1830_REG_POWEROFF, v | D1830_POWEROFF_BIT);

    while (1);
}
