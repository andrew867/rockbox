# iPod nano 7G (N31) — driver status

A snapshot of what exists on each side of this port and what has actually run
on hardware. Gathered from the tree, not from memory: 33 driver files under
`firmware/target/arm/s5l8740/`, all compiled (34 `SOURCES` entries), against
26 Linux modules in `tools/linux-n31/drivers/` and the ~28 MMIO blocks in
`firmware/export/s5l87xx.h`.

The "on glass" column is the important one. Written and compiled says nothing;
several of these have never executed a single instruction on the device.

## Legend

| | |
| --- | --- |
| ✅ | proven on hardware |
| ⚠️ | ran at least once, not exercised properly |
| ❌ | written, never run on the device |

## By subsystem

| MMIO | Block | Linux module | Rockbox driver | On glass |
| --- | --- | --- | --- | --- |
| — | CPU / MMU / cache | kernel | `crt0.S`, `mmu-armv7a.S` | ✅ |
| `3C700000` | Timer E | kernel | `timer-s5l8740.c` | ✅ confirmed counting |
| `38E00000`/`38E01000` | PL192 VIC ×2 | `irq-vic.c` | `system-s5l8740.c` | ✅ |
| `39700000` | EIC (GPIO→VIC) | `irq-s5l8740-eic.c` | `gpio-s5l8740.c` | ✅ |
| `3C500000` | CLKCON | `clk-s5l8702.c` | `clocking-s5l8740.c` | ✅ ungate-all matches |
| `38300000` | LCDIF | `s5l8740.c` (TinyDRM) | `lcd-nano7g.c` | ✅ logo renders |
| `38A00000` | FMC / FMSS NAND | `nand-s5l8740.c` | `nand-s5l8740.c` | ✅ reads |
| — | Whimory FTL | `ftl-s5l8740.c` + csmap/vecmap | `ftl-s5l8740.c` | ✅ mounts, CXT loads |
| `3C600000`/`3C900000` | I²C ×2 | `i2c-s5l8702.c` | `i2c-s5l8740.c` | ✅ |
| `pmic@73` | D1830 PMIC | `gpio-d1830.c` | `pmu-nano7g.c` | ✅ VBAT, buttons |
| `38400000` | DWC2 USB | `phy-s5l8702-usb2.c` | `usb-s5l8740.c` | ⚠️ enumerated once |
| `38200000`/`38700000` | PL080 DMA ×2 | `dma-s5l8740-pl080.c` | `pl080.c` | ⚠️ |
| `3CA00000` | IIS0 | `s5l8740-i2s.c` | `pcm-s5l8740.c` | ❌ |
| `3C300000` | SPI0 → CS42L81 | `cs42l81-spi.c` | `spi-s5l8740.c`, `cs42l81.c` | ❌ |
| `3D200000` | SPI2 → Nimbus touch | `apple-nimbus.c` | `touch-nano7g.c`, `touch-hbpp.c` | ❌ |
| `3DB00000` | UART1 → BCM2078 BT | `bcm2078-bt.c` | `bcm2078-s5l8740.c` | ❌ |
| `3D400000` | IIS2 (FM capture) | in `nano7-audio.c` | `iis2-s5l8740.c` | ❌ |
| — | FM tuner | via BT vendor HCI | `bcm2078_tuner.c` | ❌ wired, untested |
| `i2c@18` | LIS331DLH accelerometer | `lis3lv02d_i2c.c` | `accel-nano7g.c` | ❌ |
| `3DC00000` | UART2 MikeyBus | `apple-mikeybus.c` | `mikeybus-nano7g.c` | ❌ |
| `lightning@1a` | Tristar CBTL1609 | `apple-tristar-cbtl1609.c` | `tristar-nano7g.c` | ❌ |
| `3E000000` | Backlight | `backlight-s5l8740.c` | `backlight-nano7g.c` | ❌ |
| `38000000` / `38C00000` / `3C100000` | SHA1 / AES / PRNG | `s5l8702-{sha1,aes,prng}.c` | `crypto-s5l8740.c` | ❌ |
| `3CC00000` | UART0 | — | `uart-s5l8740.c` | ❌ |
| `3DD00000` | UART3 (debug console) | `serial@3dd00000` | — | ❌ no cable exists |

## Asymmetries worth knowing

**Rockbox carries drivers Linux has no counterpart for** — RTC, ADC, buttons,
battery/powermgmt, `boot-beacon.c`, `debug-s5l8740.c`. These are obligations of
being the OS. Linux gets the equivalents from its own subsystems, so their
absence there is not a gap.

**Linux carries one we do not need** — `n31-early-bringup.c`. U-Boot does that
job before handing over to us.

**`DMAC1` at `0x38700000` is in our MMIO map and has no DT node.** Either the
Linux side only ever used DMAC0, or the second controller is simply
undeclared there. Worth resolving before trusting the second channel set for
audio, because "it works on Linux" is not evidence about a block Linux never
touched.

**There is no NAND node in the DTS at all.** The Linux FTL is bound another
way. Both sides have working drivers so nothing is broken by it, but it does
mean the device tree is not a complete inventory of the hardware and should
not be read as one.

**TVOUT (`39100000`–`39300000`) has no driver on either side**, and the
watchdog appears only as `syscon-reboot`.

## Where the port actually is

Working:

- Boots from U-Boot through crt0, MMU and caches
- Display, framebuffer and text
- NAND reads and the Whimory FTL, including the SFTL checkpoint fast path

Blocked:

- `disk_mount_all()` — this is the FAT layer, not storage. The map builds and
  reports ready; something above it does not finish mounting.

Untouched by hardware:

- The entire audio path. Codec, IIS0 and the DMA that feeds it are written and
  have never run. This is the largest block of unexercised code in the port and
  the most likely source of the next several surprises.
- Touch, Bluetooth, FM, accelerometer, MikeyBus, Tristar, backlight, crypto.

Nine files carry TODO or stub markers, concentrated in power management and
the LCD DMA path.

## A note on reading this table

Every ❌ is code that compiles, links, and looks reasonable. That was also true
of the LCD reset sequence that hung the boot, the weave filter that silently
discarded the entire post-checkpoint history, and the VBA layout that built a
complete map of the wrong places. Each of those was correct-looking, ported
faithfully from a working Linux driver, and wrong on this device for a reason
that only appeared when it ran.

Treat the ❌ column as a list of things not yet known to work, rather than a
list of things expected to work.
