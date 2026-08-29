/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Startup for Apple S5L8740 (iPod nano 7G / N31), Cortex-A5.
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
#include "cpu.h"

/*
 * Entered from U-Boot (bootm of an mImage wrapping this binary) with the
 * image loaded at DRAM_ORIG. Unlike the s5l8702 targets we do not relocate
 * anything into IRAM: the SEC IRAM is 128 KB and shared with the DFU ROM and
 * U-Boot, and we have 64 MB of DRAM to work with.
 *
 * That in turn means the exception vectors stay in DRAM rather than being
 * copied to address 0. ARMv7-A gives us VBAR for exactly this, so nothing
 * has to be mapped at 0 and the "do not execute from 0x0" dance in the
 * s5l8702 crt0 has no equivalent here.
 */

    .section .intvect,"ax",%progbits
    .global    start
    .global    _newstart
    /* Exception vectors -- installed via VBAR, not copied to 0. */
start:
    b _newstart
    ldr pc, =undef_instr_handler
    ldr pc, =software_int_handler
    ldr pc, =prefetch_abort_handler
    ldr pc, =data_abort_handler
    ldr pc, =reserved_handler
    ldr pc, =irq_handler
    ldr pc, =fiq_handler
    .ltorg

_newstart:
    msr     cpsr_c, #0xd3           /* supervisor mode, IRQ/FIQ off */

    /*
     * U-Boot hands over with the MMU and caches on and its own page tables
     * live. Tear that down before we touch anything, cleaning first so that
     * U-Boot's dirty lines reach DRAM rather than being dropped.
     */
    mrc     p15, 0, r0, c1, c0, 0
    tst     r0, #1                  /* MMU currently enabled? */
    beq     1f
    bl      commit_discard_dcache
1:
    mrc     p15, 0, r0, c1, c0, 0
    bic     r0, r0, #(1 << 12)      /* I: instruction cache off */
    bic     r0, r0, #(1 << 2)       /* C: data cache off */
    bic     r0, r0, #1              /* M: MMU off */
    mcr     p15, 0, r0, c1, c0, 0
    dsb
    isb

    mov     r0, #0
    mcr     p15, 0, r0, c8, c7, 0   /* invalidate entire TLB */
    mcr     p15, 0, r0, c7, c5, 0   /* invalidate entire ICache */
    mcr     p15, 0, r0, c7, c5, 6   /* invalidate branch predictor */
    dsb
    isb

    /* Point VBAR at our vector table, which lives wherever we were loaded. */
    ldr     r0, =_intvectstart
    mcr     p15, 0, r0, c12, c0, 0  /* VBAR */
    isb

    /*
     * Reset both PL192 VICs before enabling interrupts anywhere. U-Boot
     * leaves its own handlers installed, and on N31 the bootloader's vectors
     * for lines 16-31 in particular have to be replaced or the EIC-routed
     * GPIO interrupts land in U-Boot's stubs.
     */
    ldr     r1, =VIC_BASE
    add     r2, r1, #VIC_OFFSET
    mvn     r4, #0
    str     r4, [r1, #0x14]         /* VIC0INTENCLEAR = ~0 */
    str     r4, [r2, #0x14]         /* VIC1INTENCLEAR = ~0 */
    str     r4, [r1, #0xf00]        /* VIC0ADDRESS */
    str     r4, [r2, #0xf00]        /* VIC1ADDRESS */

    /* Initialise bss section to zero */
    ldr     r2, =_edata
    ldr     r3, =_end
    mov     r4, #0
1:
    cmp     r3, r2
    strhi   r4, [r2], #4
    bhi     1b

    /*
     * Set up the MMU and turn caches on. Everything above ran uncached; from
     * here the port runs with DRAM cached and the I/O window strongly
     * ordered. See mmu-armv7a.S and system-s5l8740.c:set_page_tables().
     */
    bl      memory_init

    /* Set up stack for IRQ mode */
    msr     cpsr_c, #0xd2
    ldr     sp, =_irqstackend

    /* Set up stack for FIQ mode */
    msr     cpsr_c, #0xd1
    ldr     sp, =_fiqstackend

    /* Let svc, abort and undefined modes use the irq stack */
    msr     cpsr_c, #0xd3
    ldr     sp, =_irqstackend
    msr     cpsr_c, #0xd7
    ldr     sp, =_irqstackend
    msr     cpsr_c, #0xdb
    ldr     sp, =_irqstackend

    /* Switch to sys mode */
    msr     cpsr_c, #0xdf

    /* Set up the main stack and munge it with 0xdeadbeef */
    ldr     sp, =stackend
    ldr     r2, =stackbegin
    ldr     r3, =0xdeadbeef
1:
    cmp     sp, r2
    strhi   r3, [r2], #4
    bhi     1b

    b       main
    .ltorg
