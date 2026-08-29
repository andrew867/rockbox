/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * System init and interrupt dispatch for Apple S5L8740 (iPod nano 7G).
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
#include "panic.h"
#include "gpio-s5l8740.h"
#include "boot-beacon.h"
#include "clocking-s5l8740.h"
#include "spi-s5l8740.h"
#include "pl080.h"

void usec_timer_init(void);

/*
 * Interrupt vector table. Two PL192 VICs give 64 lines; the EIC funnels GPIO
 * groups into two of them (see eic_init() in gpio-s5l8740.c).
 */
#define DEFAULT_INTERRUPT(name) \
    extern __attribute__((weak,alias("UIRQ"))) void name(void)

static void UIRQ(void);

DEFAULT_INTERRUPT(INT_TIMER);
DEFAULT_INTERRUPT(INT_DMAC0);
DEFAULT_INTERRUPT(INT_DMAC1);
DEFAULT_INTERRUPT(INT_USB_FUNC);
DEFAULT_INTERRUPT(INT_IIC0);
DEFAULT_INTERRUPT(INT_IIC1);
DEFAULT_INTERRUPT(INT_UART0);
DEFAULT_INTERRUPT(INT_UART1);
DEFAULT_INTERRUPT(INT_UART2);
DEFAULT_INTERRUPT(INT_UART3);
DEFAULT_INTERRUPT(INT_EIC_GROUP1);
DEFAULT_INTERRUPT(INT_EIC_GROUP2);

static void (* const irqvector[])(void) =
{
    [0 ... 63]          = UIRQ,
    [IRQ_TIMER]         = INT_TIMER,
    [IRQ_DMAC0]         = INT_DMAC0,
    [IRQ_DMAC1]         = INT_DMAC1,
    [IRQ_USB_OTG]       = INT_USB_FUNC,
    [IRQ_IIC0]          = INT_IIC0,
    [IRQ_IIC1]          = INT_IIC1,
    [IRQ_UART0]         = INT_UART0,
    [IRQ_UART1]         = INT_UART1,
    [IRQ_UART2]         = INT_UART2,
    [IRQ_UART3]         = INT_UART3,
    [IRQ_EIC_GROUP2]    = INT_EIC_GROUP2,
    [IRQ_EIC_GROUP1]    = INT_EIC_GROUP1,
};

static int current_irq = 0;

static void UIRQ(void)
{
    panicf("Unhandled IRQ %d!", current_irq);
}

void irq_handler(void) __attribute__((interrupt ("IRQ")));
void irq_handler(void)
{
    (void)VIC0ADDRESS;
    (void)VIC1ADDRESS;
    uint32_t irqs0 = VIC0IRQSTATUS;
    uint32_t irqs1 = VIC1IRQSTATUS;
    for (current_irq = 0; irqs0; current_irq++, irqs0 >>= 1)
        if (irqs0 & 1)
            irqvector[current_irq]();
    for (current_irq = 32; irqs1; current_irq++, irqs1 >>= 1)
        if (irqs1 & 1)
            irqvector[current_irq]();
    VIC0ADDRESS = NULL;
    VIC1ADDRESS = NULL;
}

void fiq_handler(void) __attribute__((interrupt ("FIQ"), naked, \
                                      weak, alias("fiq_dummy")));
void fiq_dummy(void)
{
    asm volatile (
        "subs   pc, lr, #4   \r\n"
    );
}

/*
 * Page tables.
 *
 * DRAM is mapped 1:1 and cached, the whole I/O window is mapped uncached,
 * and DRAM gets a second uncached alias at +0x40000000 for DMA buffers.
 *
 * Nothing is mapped at address 0: ARMv7-A puts the exception vectors wherever
 * VBAR points, so the "map IRAM to 0" entry the s5l8702 targets need has no
 * equivalent here.
 */
static void set_page_tables(void)
{
    /* DRAM, cached. 64 MB starting at 0x08000000. */
    map_section(DRAM_ORIG, DRAM_ORIG, MEMORYSIZE, CACHE_ALL);

    /*
     * I/O. 0x38000000..0x3fffffff covers every block in the N31 map, from
     * SHA1 at 0x38000000 up to the Lightning/TVOut transport at 0x3E600000.
     */
    map_section(IO_BASE, IO_BASE, 0x80, CACHE_NONE);

    /*
     * SEC IRAM, uncached and 1:1.
     *
     * We do not use it -- this port runs entirely from DRAM -- but the DFU
     * ROM and the earlier boot stages do, and their setup is not something we
     * can decompile. Leaving the window unmapped means any pointer they left
     * behind faults the moment the MMU comes on, which would look like the
     * firmware dying instantly for no visible reason.
     *
     * Mapping a region we never touch costs two page-table entries. Not
     * mapping it costs a boot that fails in a way nothing can report.
     */
    map_section(IRAM_ORIG, IRAM_ORIG, 2, CACHE_NONE);

    /*
     * The low 128 MB, uncached and 1:1, for the same reason: whatever the
     * bootloader parked below DRAM stays reachable rather than becoming a
     * fault. Cheap insurance while nothing about this boot is proven.
     */
    map_section(0x00000000, 0x00000000, 128, CACHE_NONE);

    /* Uncached alias of DRAM for DMA. */
    map_section(DRAM_ORIG, (uintptr_t)S5L8740_UNCACHED_ADDR(DRAM_ORIG),
                MEMORYSIZE, CACHE_NONE);
}

void memory_init(void)
{
    /*
     * The MMU can be left off entirely during bring-up.
     *
     * With it off, DRAM and every peripheral are reachable 1:1 and nothing
     * the earlier boot stages did can be lost to a missing mapping -- at the
     * cost of running uncached, which is slow but correct. The ARMv7-A MMU
     * path has never executed on this silicon, so being able to take it out
     * of the picture in one define is worth having while the first boot is
     * still being chased.
     */
#ifdef N31_NO_MMU
    (void)set_page_tables;
#else
    ttb_init();
    set_page_tables();
    enable_mmu();
#endif
}

void system_init(void)
{
    /*
     * FIRST THING, before anything that could block.
     *
     * Reaching here proves crt0 ran, the ARMv7-A MMU came up, DRAM works and
     * we are executing C -- none of which had ever been demonstrated on
     * silicon. With no serial console, painting the panel is the only way to
     * say so, and it has to happen before system_init() and kernel_init(),
     * both of which run ahead of Rockbox's own lcd_init().
     */
    beacon_stage(BEACON_YELLOW);

    /* Both VICs were disarmed in crt0; set them up properly now. */
    VIC0INTENCLEAR = ~0;
    VIC1INTENCLEAR = ~0;
    VIC0INTSELECT = 0;      /* everything is IRQ, nothing is FIQ */
    VIC1INTSELECT = 0;
    VIC0ADDRESS = NULL;
    VIC1ADDRESS = NULL;

    /*
     * Clocks first: every block below needs its gate open, and the policy is
     * ungate-all rather than per-driver gating while the clock tree is only
     * partly RE'd.
     */
    clocking_init();
    usec_timer_init();

    /* Clocks and the microsecond timer are programmed. */
    beacon_stage(BEACON_GREEN);

    gpio_init();
    eic_init();
    pl080_init();
    spi_init();

    /* Everything system_init() owns returned without hanging. */
    beacon_stage(BEACON_CYAN);
}

void system_reboot(void)
{
    /*
     * TODO: the watchdog at 0x3C800000 reboots the SoC (syscon-reboot writes
     * 0x100000), but the Linux DT keeps that node disabled pending proof that
     * poweroff works, and an unproven WDT poke here would strand the device
     * out of DFU. Spin instead until it is verified on glass.
     */
    disable_interrupt(IRQ_FIQ_STATUS);
    while (1);
}

void system_exception_wait(void)
{
    disable_interrupt(IRQ_FIQ_STATUS);
    while (1);
}

int system_memory_guard(int newmode)
{
    (void)newmode;
    return 0;
}

#ifdef HAVE_ADJUSTABLE_CPU_FREQ
void set_cpu_frequency(long frequency)
{
    /* TODO: needs the CLKCON PLL/divider map, which is not RE'd yet. */
    cpu_frequency = frequency;
}
#endif
