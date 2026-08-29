/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
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
#ifndef SYSTEM_TARGET_H
#define SYSTEM_TARGET_H

#include "system-arm.h"
#include "mmu-arm.h"
#include "cpucache-arm.h"

/*
 * TODO: the N31 CLKCON tree is only partly reverse-engineered. clk-s5l8702.c
 * does an ungate-all rather than real rate control, so there is no frequency
 * scaling here yet and these are nominal.
 */
#define CPUFREQ_SLEEP   32768
#define CPUFREQ_MAX     400000000
#define CPUFREQ_DEFAULT 400000000
#define CPUFREQ_NORMAL  400000000

#define STORAGE_WANTS_ALIGN

/*
 * Uncached alias of DRAM. set_page_tables() maps the same physical memory a
 * second time at +0x40000000 with caching disabled, which is how drivers get
 * a coherent view of DMA buffers without cache maintenance on every access.
 */
#define S5L8740_UNCACHED_ADDR(a) ((typeof(a)) ((uintptr_t)(a) + 0x40000000))
#define S5L8740_PHYSICAL_ADDR(a) ((typeof(a)) ((uintptr_t)(a) & ~0x40000000))

#define inl(a)    (*(volatile unsigned long *) (a))
#define outl(a,b) (*(volatile unsigned long *) (b) = (a))
#define inb(a)    (*(volatile unsigned char *) (a))
#define outb(a,b) (*(volatile unsigned char *) (b) = (a))
#define inw(a)    (*(volatile unsigned short*) (a))
#define outw(a,b) (*(volatile unsigned short*) (b) = (a))

/*
 * system-arm-classic.c's exception handler passes the faulting stack pointer
 * to rb_backtrace(). Nothing in the tree defines __get_sp() for this profile,
 * so provide it here -- reading SP directly is correct because UIE() runs on
 * the same stack as the fault it is reporting.
 */
static inline int __get_sp(void)
{
    register int sp asm("sp");
    return sp;
}

static inline void udelay(unsigned usecs)
{
    unsigned stop = USEC_TIMER + usecs;
    while (TIME_BEFORE(USEC_TIMER, stop));
}

#endif /* SYSTEM_TARGET_H */
