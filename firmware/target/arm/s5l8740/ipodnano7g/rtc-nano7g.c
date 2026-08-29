/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Real-time clock inside the D1830 PMIC, for the iPod nano 7G (N31).
 *
 * Ported from the RTC half of tools/linux-n31/drivers/gpio-d1830.c.
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
#include "rtc.h"
#include "pmu-target.h"

/*
 * There is no separate RTC chip on this board. The D1830 keeps a plain
 * little-endian 32-bit seconds counter in registers 124..127, read and
 * written a byte at a time least significant first (RetailOS sub_16517E).
 *
 * Reads and writes are self-consistent, so timekeeping survives a reboot.
 * What is NOT established is the epoch the stock firmware uses -- nothing in
 * the RE says whether this counts from 1970, from 2001 like other Apple
 * formats, or from an arbitrary factory zero. So this treats it as a plain
 * Unix time_t: setting the clock in Rockbox and reading it back is
 * self-consistent, but the value will not necessarily agree with what
 * RetailOS would display until the epoch is confirmed.
 */
#define D1830_RTC_REG_BASE      124
#define D1830_RTC_REG_COUNT     4

/* Days in each month, non-leap. */
static const unsigned char days_in_month[12] = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

static bool is_leap(int year)
{
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

static uint32_t d1830_rtc_read_secs(void)
{
    uint32_t secs = 0;
    int i;

    for (i = 0; i < D1830_RTC_REG_COUNT; i++) {
        int v = pmu_read(D1830_RTC_REG_BASE + i);

        if (v < 0)
            return 0;
        secs |= ((uint32_t)v & 0xff) << (8 * i);
    }

    return secs;
}

static void d1830_rtc_write_secs(uint32_t secs)
{
    int i;

    for (i = 0; i < D1830_RTC_REG_COUNT; i++)
        pmu_write(D1830_RTC_REG_BASE + i, (secs >> (8 * i)) & 0xff);
}

void rtc_init(void)
{
}

int rtc_read_datetime(struct tm *tm)
{
    uint32_t secs = d1830_rtc_read_secs();
    uint32_t days = secs / 86400;
    uint32_t rem = secs % 86400;
    int year = 1970;
    int month = 0;

    tm->tm_hour = rem / 3600;
    tm->tm_min = (rem % 3600) / 60;
    tm->tm_sec = rem % 60;

    /* 1 Jan 1970 was a Thursday. */
    tm->tm_wday = (int)((days + 4) % 7);

    while (1) {
        uint32_t ydays = is_leap(year) ? 366 : 365;

        if (days < ydays)
            break;
        days -= ydays;
        year++;
    }

    tm->tm_year = year - 1900;
    tm->tm_yday = (int)days;

    while (1) {
        uint32_t mdays = days_in_month[month];

        if (month == 1 && is_leap(year))
            mdays++;
        if (days < mdays)
            break;
        days -= mdays;
        month++;
    }

    tm->tm_mon = month;
    tm->tm_mday = (int)days + 1;

    return 1;
}

int rtc_write_datetime(const struct tm *tm)
{
    int year = tm->tm_year + 1900;
    uint32_t days = 0;
    int y, m;

    for (y = 1970; y < year; y++)
        days += is_leap(y) ? 366 : 365;

    for (m = 0; m < tm->tm_mon; m++) {
        days += days_in_month[m];
        if (m == 1 && is_leap(year))
            days++;
    }

    days += tm->tm_mday - 1;

    d1830_rtc_write_secs(days * 86400
                       + tm->tm_hour * 3600
                       + tm->tm_min * 60
                       + tm->tm_sec);

    return 1;
}
