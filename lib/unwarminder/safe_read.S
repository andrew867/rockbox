/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Copyright (C) 2012 by Amaury Pouly
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

.syntax unified

.data
was_aborted:
    .word   0

.section    .text.safe_read8
.type       safe_read8, %function
.global     safe_read8
@ bool safe_read8(uint8_t *addr, uint8_t *value)
safe_read8:
    @ was_aborted = 0
    ldr     r2, =was_aborted
    mov     r3, #0
    str     r3, [r2]
    @ r0=*addr
safe_read8_faulty_addr:
    ldrb    r0, [r0]
    @ if(was_aborted)
    ldr     r2, [r2]
    cmp     r2, #1
    @   return false;
    moveq   r0, #0
    bxeq    lr
    @ if(value != NULL)
    cmp     r1, #0
    @   *value = r0
    strbne  r0, [r1]
    @ return true;
    mov     r0, #1
    bx      lr
.size       safe_read8, . - safe_read8

.section    .text.safe_read16
.type       safe_read16, %function
.global     safe_read16
@ bool safe_read16(uint16_t *addr, uint16_t *value)
safe_read16:
    @ was_aborted = 0
    ldr     r2, =was_aborted
    mov     r3, #0
    str     r3, [r2]
    @ r0=*addr
safe_read16_faulty_addr:
    ldrh    r0, [r0]
    @ if(was_aborted)
    ldr     r2, [r2]
    cmp     r2, #1
    @   return false;
    moveq   r0, #0
    bxeq    lr
    @ if(value != NULL)
    cmp     r1, #0
    @   *value = r0
    strhne  r0, [r1]
    @ return true;
    mov     r0, #1
    bx      lr
.size       safe_read16, . - safe_read16

.section    .text.safe_read32
.type       safe_read32, %function
.global     safe_read32
@ bool safe_read32(uint32_t *addr, uint32_t *value)
safe_read32:
    @ was_aborted = 0
    ldr     r2, =was_aborted
    mov     r3, #0
    str     r3, [r2]
    @ r0=*addr
safe_read32_faulty_addr:
    ldr     r0, [r0]
    @ if(was_aborted)
    ldr     r2, [r2]
    cmp     r2, #1
    @   return false;
    moveq   r0, #0
    bxeq    lr
    @ if(value != NULL)
    cmp     r1, #0
    @   *value = r0
    strne   r0, [r1]
    @ return true;
    mov     r0, #1
    bx      lr
.size       safe_read32, . - safe_read32

/*
 * The fixup handler is needed by every ARM profile that can take a data
 * abort, not just the classic one.
 *
 * safe_read8/16/32 fault ON PURPOSE -- that is the whole mechanism. The
 * handler recognises the faulting PC as one of the three probe sites, sets
 * was_aborted, and resumes at the following instruction so the probe can
 * return false. Without it the probe is not "safe" at all: it is an
 * unhandled abort.
 *
 * That matters most in the one place safe_read is actually used -- the stack
 * unwinder, which runs from UIE while reporting an earlier crash. A probe
 * that panics instead of returning false replaces the original fault's
 * address and PC with the unwinder's own, so the crash report describes the
 * crash reporter. On the nano7g that surfaced as a data abort on 0x64616f6c,
 * which is the ASCII bytes "load" being walked as a pointer: a perfectly
 * ordinary unwinder probe of a stack slot that is not one, reported as if it
 * were the bug.
 *
 * ARMv7-A takes data aborts with exactly the same LR semantics as ARMv5/v6
 * (LR_abt = faulting instruction + 8, resume via subs pc, lr, #4), so the
 * code below is correct as written for the application profile. Only the
 * gate needed widening.
 */
#if (CONFIG_PLATFORM & PLATFORM_NATIVE) && \
    (defined(CPU_ARM_CLASSIC) || defined(CPU_ARM_APPLICATION))
.section    .text.data_abort_handler
.type       data_abort_handler, %function
.global     data_abort_handler
data_abort_handler:
    @ store minimal amount of registers
    stmfd   sp!, {r0-r1}
    @ compute faulty address
    sub     r0, lr, #8
    @ compare to safe_read8
    ldr     r1, =safe_read8_faulty_addr
    cmp     r0, r1
    beq     1f
    @ compare to safe_read16
    ldr     r1, =safe_read16_faulty_addr
    cmp     r0, r1
    beq     1f
    @ compare to safe_read32
    ldr     r1, =safe_read32_faulty_addr
    cmp     r0, r1
    beq     1f
    @ otherwise just normally to UIE
    mov     r1, #2
    b       UIE
1:
    @ set was_aborted
    ldr     r1, =was_aborted
    mov     r0, #1
    str     r0, [r1]
    @ restore registers
    ldmfd   sp!, {r0-r1}
    @ restore mode and jump back to the *next* instruction
    subs    pc, lr, #4
.size       data_abort_handler, . - data_abort_handler
#endif
