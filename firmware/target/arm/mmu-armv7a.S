/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * ARMv7-A (Cortex-A) MMU and cache maintenance.
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

/*
 * ARMv7-A differs from the ARMv6 code in firmware/target/arm/mmu-armv6.S in
 * three ways that matter:
 *
 *  1. There is no "clean entire data cache" CP15 operation. ARM11 had
 *     c7,c10,0 and c7,c14,0; ARMv7-A deleted both. Whole-cache maintenance
 *     must walk every set and way of every data cache level, discovered at
 *     runtime through CLIDR/CCSIDR. That is what the set/way helper below
 *     does, and it is why these routines are much longer than their ARMv6
 *     counterparts.
 *
 *  2. There are no MCRR "range" operations (c6/c12/c14 with two registers).
 *     Those were an ARM11/XScale extension. Ranges are walked one cache
 *     line at a time by MVA, with the line size read from CTR rather than
 *     assumed to be 32 bytes.
 *
 *  3. Barriers are real instructions (DSB/DMB/ISB), not CP15 writes.
 *
 * The exception model is otherwise the classic ARM one, so this pairs with
 * system-arm-classic.c rather than the Cortex-M code.
 */

#include "config.h"
#include "cpu.h"

/****************************************************************************
 * MMU setup
 *
 * Short-descriptor (VMSAv7) 1 MB sections. The TTB is 16 KB of 4-byte
 * entries, one per megabyte of the 4 GB virtual space.
 ***************************************************************************/

/*
 * void ttb_init(void);
 */
    .section    .text.ttb_init, "ax", %progbits
    .align      2
    .global     ttb_init
    .type       ttb_init, %function
ttb_init:
    ldr     r0, =TTB_BASE_ADDR
    /*
     * TTBCR.N = 0: TTBR0 covers the whole address space and is 16 KB
     * aligned. TTBR1 is unused.
     */
    mov     r1, #0
    mcr     p15, 0, r1, c2, c0, 2   @ TTBCR = 0

    /*
     * TTBR0 low bits describe how the table walk itself is cached.
     * 0x6b = IRGN[0]=1 (bit 6 with IRGN[1]=bit 0 clear -> write-back
     * write-allocate inner), S=0, RGN=0b01 (outer write-back write-allocate),
     * NOS=0. Keeping the walk cacheable matters: with it clear every TLB
     * miss becomes an uncached DRAM read.
     */
    orr     r1, r0, #0x6b
    mcr     p15, 0, r1, c2, c0, 0   @ TTBR0

    mvn     r1, #0
    mcr     p15, 0, r1, c3, c0, 0   @ all domains = manager
    dsb
    isb
    bx      lr
    .size   ttb_init, .-ttb_init

/*
 * void map_section(unsigned int pa, unsigned int va, int mb, int flags);
 *
 * flags carries C in bit 3 and B in bit 2, matching mmu-arm.h's CACHE_ALL
 * (0x0C), BUFFERED (0x04) and CACHE_NONE (0), so callers written against the
 * ARMv6 code work unchanged.
 *
 * Descriptor layout (VMSAv7 section, SCTLR.AFE == 0):
 *   [1:0]   0b10   section
 *   [2]     B
 *   [3]     C
 *   [4]     XN     left 0 -- Rockbox executes from DRAM
 *   [8:5]   domain 0
 *   [11:10] AP[1:0] = 0b11 with AP[2]=0 -> full access
 *   [14:12] TEX = 0b000
 *   [16]    S      left 0 -- single core, no coherent masters
 *
 * Note this deliberately does NOT set bit 4 the way mmu-armv6.S does. On
 * VMSAv6/v7 bit 4 of a section descriptor is XN, not SBO; setting it makes
 * the region non-executable, which would fault the moment we branch into it.
 */
    .section    .text.map_section, "ax", %progbits
    .align      2
    .global     map_section
    .type       map_section, %function
map_section:
    @ pa &= ~0xfffff
    mov     r0, r0, lsr #20
    mov     r0, r0, lsl #20

    orr     r0, r0, r3              @ C/B from flags
    orr     r0, r0, #(3 << 10)      @ AP[1:0] = 0b11, full access
    orr     r0, r0, #2              @ section descriptor

    @ entry = TTB_BASE + (va >> 20) * 4
    mov     r1, r1, lsr #20
    ldr     r3, =TTB_BASE_ADDR
    add     r1, r3, r1, lsl #2

    cmp     r2, #0
    bxle    lr
    mov     r3, #0
1:
    str     r0, [r1], #4
    add     r0, r0, #0x100000
    add     r3, r3, #1
    cmp     r2, r3
    bne     1b

    dsb
    bx      lr
    .size   map_section, .-map_section

/*
 * void enable_mmu(void);
 *
 * Caches are invalidated (not cleaned) first: whatever is in them at this
 * point predates the page tables and must not be written back over them.
 */
    .section    .text.enable_mmu, "ax", %progbits
    .align      2
    .global     enable_mmu
    .type       enable_mmu, %function
enable_mmu:
    stmfd   sp!, {r4-r11, lr}

    mov     r0, #0
    mcr     p15, 0, r0, c8, c7, 0   @ invalidate entire TLB
    mcr     p15, 0, r0, c7, c5, 0   @ invalidate entire ICache
    mcr     p15, 0, r0, c7, c5, 6   @ invalidate branch predictor
    dsb
    isb

    bl      __v7_dcache_invalidate_all

    mrc     p15, 0, r0, c1, c0, 0   @ SCTLR
    bic     r0, r0, #(1 << 13)      @ V = 0: vectors come from VBAR
    bic     r0, r0, #(1 << 29)      @ AFE = 0: classic AP[2:0] permissions
    bic     r0, r0, #(1 << 28)      @ TRE = 0: no remap of TEX/C/B
    orr     r0, r0, #(1 << 11)      @ Z: branch prediction
    orr     r0, r0, #(1 << 12)      @ I: instruction cache
    orr     r0, r0, #(1 << 2)       @ C: data cache
    orr     r0, r0, #1              @ M: MMU
    mcr     p15, 0, r0, c1, c0, 0
    dsb
    isb

    ldmfd   sp!, {r4-r11, pc}
    .size   enable_mmu, .-enable_mmu
    .ltorg

/****************************************************************************
 * Cache maintenance
 ***************************************************************************/

/*
 * Data cache line size, in bytes, into the given register.
 * CTR.DminLine (bits 19:16) is log2 of the line length in *words*.
 */
    .macro  dcache_line_size reg, tmp
    mrc     p15, 0, \tmp, c0, c0, 1     @ CTR
    lsr     \tmp, \tmp, #16
    and     \tmp, \tmp, #0xf
    mov     \reg, #4
    lsl     \reg, \reg, \tmp
    .endm

/*
 * Walk every set and way of every data/unified cache level and apply one
 * maintenance op.
 *
 * r0 selects the operation:  0 = invalidate (DCISW)
 *                            1 = clean (DCCSW)
 *                            2 = clean and invalidate (DCCISW)
 *
 * Clobbers r0-r11. This is the standard CLIDR/CCSIDR walk; the level loop
 * runs outermost-last because cleaning must push data from L1 towards
 * memory in order.
 */
    .section    .text.__v7_dcache_op_all, "ax", %progbits
    .align      2
    .type       __v7_dcache_op_all, %function
__v7_dcache_op_all:
    mov     r10, r0                 @ r10 = op selector
    dsb
    mrc     p15, 1, r0, c0, c0, 1   @ CLIDR
    mov     r3, r0, lsr #23
    ands    r3, r3, #(7 << 1)       @ r3 = LoC * 2
    beq     5f                      @ nothing to do

    mov     r9, #0                  @ r9 = current level * 2
1:  @ level loop
    add     r2, r9, r9, lsr #1      @ r2 = level * 3
    mov     r1, r0, lsr r2
    and     r1, r1, #7              @ cache type at this level
    cmp     r1, #2
    blt     4f                      @ no data/unified cache here

    mcr     p15, 2, r9, c0, c0, 0   @ CSSELR = level, data cache
    isb
    mrc     p15, 1, r1, c0, c0, 0   @ CCSIDR
    and     r2, r1, #7
    add     r2, r2, #4              @ r2 = log2(line size in bytes)
    ldr     r6, =0x3ff
    ands    r6, r6, r1, lsr #3      @ r6 = max way index
    clz     r5, r6                  @ r5 = way field shift
    ldr     r7, =0x7fff
    ands    r7, r7, r1, lsr #13     @ r7 = max set index

2:  @ way loop
    mov     r8, r7                  @ r8 = working set index
3:  @ set loop
    orr     r11, r9, r6, lsl r5     @ level | way
    orr     r11, r11, r8, lsl r2    @ | set

    cmp     r10, #0
    mcreq   p15, 0, r11, c7, c6, 2  @ DCISW
    cmp     r10, #1
    mcreq   p15, 0, r11, c7, c10, 2 @ DCCSW
    cmp     r10, #2
    mcreq   p15, 0, r11, c7, c14, 2 @ DCCISW

    subs    r8, r8, #1
    bge     3b
    subs    r6, r6, #1
    bge     2b
4:
    add     r9, r9, #2
    cmp     r3, r9
    bgt     1b
5:
    mov     r10, #0
    mcr     p15, 2, r10, c0, c0, 0  @ CSSELR = 0
    dsb
    isb
    bx      lr
    .size   __v7_dcache_op_all, .-__v7_dcache_op_all

    .section    .text.__v7_dcache_invalidate_all, "ax", %progbits
    .align      2
    .global     __v7_dcache_invalidate_all
    .type       __v7_dcache_invalidate_all, %function
__v7_dcache_invalidate_all:
    stmfd   sp!, {r4-r11, lr}
    mov     r0, #0
    bl      __v7_dcache_op_all
    ldmfd   sp!, {r4-r11, pc}
    .size   __v7_dcache_invalidate_all, .-__v7_dcache_invalidate_all

/*
 * void commit_dcache(void);
 */
    .section    .text.commit_dcache, "ax", %progbits
    .align      2
    .global     commit_dcache
    .type       commit_dcache, %function
commit_dcache:
    stmfd   sp!, {r4-r11, lr}
    mov     r0, #1
    bl      __v7_dcache_op_all
    ldmfd   sp!, {r4-r11, pc}
    .size   commit_dcache, .-commit_dcache

/*
 * void commit_discard_dcache(void);
 */
    .section    .icode.commit_discard_dcache, "ax", %progbits
    .align      2
    .global     commit_discard_dcache
    .type       commit_discard_dcache, %function
commit_discard_dcache:
    stmfd   sp!, {r4-r11, lr}
    mov     r0, #2
    bl      __v7_dcache_op_all
    ldmfd   sp!, {r4-r11, pc}
    .size   commit_discard_dcache, .-commit_discard_dcache

/*
 * void commit_discard_idcache(void);
 */
    .section    .icode.commit_discard_idcache, "ax", %progbits
    .align      2
    .global     commit_discard_idcache
    .type       commit_discard_idcache, %function
commit_discard_idcache:
    stmfd   sp!, {r4-r11, lr}
    mov     r0, #2
    bl      __v7_dcache_op_all
    mov     r0, #0
    mcr     p15, 0, r0, c7, c5, 0   @ invalidate entire ICache
    mcr     p15, 0, r0, c7, c5, 6   @ invalidate branch predictor
    dsb
    isb
    ldmfd   sp!, {r4-r11, pc}
    .size   commit_discard_idcache, .-commit_discard_idcache

/*
 * void commit_dcache_range(const void *base, unsigned int size);
 *
 * Clean (write back) without invalidating. Used before handing a buffer to
 * a DMA engine that will read it.
 */
    .section    .text.commit_dcache_range, "ax", %progbits
    .align      2
    .global     commit_dcache_range
    .type       commit_dcache_range, %function
commit_dcache_range:
    cmp     r1, #0
    bxeq    lr
    dcache_line_size r2, r3
    add     r1, r0, r1              @ end
    sub     r3, r2, #1
    bic     r0, r0, r3              @ align start down
1:
    mcr     p15, 0, r0, c7, c10, 1  @ DCCMVAC
    add     r0, r0, r2
    cmp     r0, r1
    blo     1b
    dsb
    bx      lr
    .size   commit_dcache_range, .-commit_dcache_range

/*
 * void discard_dcache_range(const void *base, unsigned int size);
 *
 * Invalidate without writing back, so a DMA engine's writes are visible.
 *
 * Unaligned edges are a data-loss hazard: the first and last lines may hold
 * bytes outside the range that are still dirty, and plain invalidation would
 * drop them. Those two lines are cleaned-and-invalidated instead, which is
 * the same compromise the ARMv6 version makes.
 */
    .section    .text.discard_dcache_range, "ax", %progbits
    .align      2
    .global     discard_dcache_range
    .type       discard_dcache_range, %function
discard_dcache_range:
    cmp     r1, #0
    bxeq    lr
    dcache_line_size r2, r3
    add     r1, r0, r1              @ end
    sub     r3, r2, #1

    tst     r0, r3                  @ start not line aligned?
    bic     r0, r0, r3
    mcrne   p15, 0, r0, c7, c14, 1  @ DCCIMVAC on the partial first line
    addne   r0, r0, r2

    tst     r1, r3                  @ end not line aligned?
    bic     r1, r1, r3
    mcrne   p15, 0, r1, c7, c14, 1  @ DCCIMVAC on the partial last line

    cmp     r0, r1
    bhs     2f
1:
    mcr     p15, 0, r0, c7, c6, 1   @ DCIMVAC
    add     r0, r0, r2
    cmp     r0, r1
    blo     1b
2:
    dsb
    bx      lr
    .size   discard_dcache_range, .-discard_dcache_range

/*
 * void commit_discard_dcache_range(const void *base, unsigned int size);
 */
    .section    .text.commit_discard_dcache_range, "ax", %progbits
    .align      2
    .global     commit_discard_dcache_range
    .type       commit_discard_dcache_range, %function
commit_discard_dcache_range:
    cmp     r1, #0
    bxeq    lr
    dcache_line_size r2, r3
    add     r1, r0, r1
    sub     r3, r2, #1
    bic     r0, r0, r3
1:
    mcr     p15, 0, r0, c7, c14, 1  @ DCCIMVAC
    add     r0, r0, r2
    cmp     r0, r1
    blo     1b
    dsb
    bx      lr
    .size   commit_discard_dcache_range, .-commit_discard_dcache_range
