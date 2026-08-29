#define __ASSEMBLER__
#include "config.h"
#include "cpu.h"

ENTRY(start)

OUTPUT_FORMAT(elf32-littlearm)
OUTPUT_ARCH(arm)
STARTUP(target/arm/s5l8740/crt0.o)

#define PLUGINSIZE PLUGIN_BUFFER_SIZE
#define CODECSIZE CODEC_SIZE

#define DRAMORIG 0x08000000

/*
 * Everything lives in DRAM on this target. The s5l8702 ports carve .icode,
 * .iram and the stacks out of the SoC's internal SRAM and map it at address
 * 0; the N31's SEC IRAM is only 128 KB and is shared with the DFU ROM and
 * U-Boot, so we leave it alone and spend DRAM instead -- there is 64 MB of it
 * and the LCDIF, not memory latency, is this port's bottleneck.
 *
 * The TTB is 16 KB at the very top of DRAM (see TTB_BASE_ADDR), so the
 * audio buffer must stop short of it.
 */
#define DRAMSIZE (DRAM_SIZE - PLUGINSIZE - CODECSIZE - TTB_SIZE)

/* End of the audio buffer, where the codec buffer starts */
#define ENDAUDIOADDR  (DRAMORIG + DRAMSIZE)
#define CODECORIG (ENDAUDIOADDR)

/* Where the codec buffer ends, and the plugin buffer starts */
#define ENDADDR (ENDAUDIOADDR + CODECSIZE)

MEMORY
{
    DRAM : ORIGIN = DRAMORIG, LENGTH = DRAMSIZE
}

SECTIONS
{
    loadaddress = DRAMORIG;

    /*
     * The vector table is the first thing in the image and stays where it is
     * loaded; crt0 points VBAR at it rather than copying it to address 0.
     */
    .intvect :
    {
        _intvectstart = . ;
        *(.intvect)
        _intvectend = . ;
    } > DRAM

    .text :
    {
        _loadaddress = .;
        _textstart = .;
        *(.init.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        . = ALIGN(0x4);
    } > DRAM

    .rodata :
    {
        *(.rodata*)
        . = ALIGN(0x4);
    } > DRAM

    .data :
    {
        *(.data*)
        . = ALIGN(0x4);
    } > DRAM

    /DISCARD/ :
    {
        *(.eh_frame)
    }

    /*
     * No physically separate IRAM here, but the .icode/.irodata/.idata and
     * .ibss section names are used all over the codebase, so they still need
     * somewhere to land. Keeping the _iramstart/_iramcopy symbols equal makes
     * the copy loop a no-op for any code that still runs one.
     */
    .iram :
    {
        _iramstart = .;
        *(.icode*)
        *(.irodata*)
        *(.idata*)
        . = ALIGN(0x4);
        _iramend = .;
    } > DRAM
    _iramcopy = _iramstart ;

    .ibss (NOLOAD) :
    {
        _iedata = .;
        *(.qharray)
        *(.ibss*)
        . = ALIGN(0x4);
        _iend = .;
    } > DRAM

    .stack (NOLOAD) :
    {
       . = ALIGN(8);
       *(.stack)
       stackbegin = .;
       _stackbegin = .;
       . += 0x2000;
       stackend = .;
       _stackend = .;
       _irqstackbegin = .;
       . += 0x800;
       _irqstackend = .;
       _fiqstackbegin = .;
       . += 0x800;
       _fiqstackend = .;
    } > DRAM

    .bss (NOLOAD) :
    {
       _edata = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(0x4);
       _end = .;
    } > DRAM

    .audiobuf (NOLOAD) :
    {
        . = ALIGN(4);
        _audiobuffer = .;
        audiobuffer = .;
    } > DRAM

    .audiobufend ENDAUDIOADDR (NOLOAD) :
    {
        audiobufend = .;
        _audiobufend = .;
    } > DRAM

    .codec CODECORIG (NOLOAD) :
    {
        codecbuf = .;
        _codecbuf = .;
    } > DRAM

    .plugin ENDADDR (NOLOAD) :
    {
        _pluginbuf = .;
        pluginbuf = .;
    }
}
