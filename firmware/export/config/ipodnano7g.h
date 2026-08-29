/*
 * This config file is for the Apple iPod nano 7th Generation (N31, S5L8740).
 *
 * Hardware facts here come from the N31 reverse-engineering work in
 * tools/linux-n31 -- primarily s5l8740-n31.dts, which is the authoritative
 * MMIO/GPIO map, and docs-internal/N31-MAJOR-SYSTEMS-STATUS.md for what has
 * actually been proven on the device as opposed to merely coded.
 *
 * Anything still unproven is marked TODO rather than guessed at.
 */

#define IPOD_ARCH 1

/* For Rolo and boot loader */
#define MODEL_NUMBER 126

#define MODEL_NAME   "Apple iPod nano 7G"

/*
 * Hardware sample rates. IIS0 CLKDIV 0x110 is the RetailOS setting for
 * 44.1 kHz music; the other rates are the usual dividers off the same clock
 * and are unverified until Phase 4 exercises them.
 */
#define HW_SAMPR_CAPS   (SAMPR_CAP_44 | SAMPR_CAP_22 | SAMPR_CAP_11 \
                       | SAMPR_CAP_48 | SAMPR_CAP_24 | SAMPR_CAP_12 \
                       | SAMPR_CAP_32 | SAMPR_CAP_16 | SAMPR_CAP_8)

/* define this if you have a bitmap LCD display */
#define HAVE_LCD_BITMAP

/* define this if you have a colour LCD */
#define HAVE_LCD_COLOR

/* define this if you want album art for this target */
#define HAVE_ALBUMART

/* define this to enable bitmap scaling */
#define HAVE_BMP_SCALING

/* define this to enable JPEG decoding */
#define HAVE_JPEG

/* define this if you have access to the quickscreen */
#define HAVE_QUICKSCREEN

/* define this if you would like tagcache to build on this target */
#define HAVE_TAGCACHE

/*
 * The N31 has no clickwheel: navigation is the capacitive touchscreen plus
 * five physical keys (Vol+/-, Home, Sleep, Play).
 */
#define CONFIG_KEYPAD IPOD_NANO7G_PAD

/* LCD dimensions -- LCDIF @0x38300000, confirmed on glass */
#define LCD_WIDTH  240
#define LCD_HEIGHT 432
/* sqrt(432^2 + 240^2) / 2.5" = 197.7 */
#define LCD_DPI 198
/*
 * Software USB detach: drop the data connection on a HOME press and return to
 * the UI with the cable still in and charging. See usb_soft_detach().
 */
#define HAVE_USB_SOFT_DETACH

/*
 * Storage may legitimately be absent while the FTL is being brought up, and a
 * USB cable event must not panic the firmware because of it.
 */
#define HAVE_OPTIONAL_STORAGE

/*
 * Debug console over USB.
 *
 * There is no serial port on this device in any practical sense -- no DCSD
 * cable, no access to SEC UART3 -- which is why the whole bring-up was done
 * with coloured beacons. USB CDC is the only text channel available.
 *
 * Rockbox already has the pieces: usb_serial.c is a CDC class driver, and
 * logf.c pushes every logf() line straight to usb_serial_send(). Both are
 * simply switched off on every iPod target. Turning them on costs one USB
 * interface and gives running commentary instead of a colour.
 */
#define ROCKBOX_HAS_LOGF
#define USB_ENABLE_SERIAL

/*
 * Serial-only USB, for when the UI has to stay usable.
 *
 * With mass storage enabled, plugging a cable does what it does on every
 * Rockbox target: storage is handed to the host and the firmware switches to
 * the USB screen. That is correct behaviour and it is not what you want while
 * watching the device do something.
 *
 * The takeover comes specifically from the mass-storage driver requesting
 * exclusive storage -- nothing else asks for it. So a build with storage
 * declined keeps the UI live, keeps the disk ours, and still enumerates the
 * serial interface. The cable becomes a console rather than a mode switch.
 *
 * Off by default: losing disk access over USB is too high a price for the
 * ordinary case. Turn it on for a debugging build.
 */
/* #define N31_USB_SERIAL_ONLY */

/*
 * USB monitoring is back on.
 *
 * It was off while the FTL was being brought up, because a cable event takes
 * storage from the firmware and hands it to the host -- and that is also
 * exactly where a failed mount surfaced, so plugging in a cable to power the
 * device replaced the FTL diagnostics with a USB teardown.
 *
 * The volume mounts now, so the condition has been met. HAVE_OPTIONAL_STORAGE
 * stays: it costs nothing and means a future storage regression reports
 * itself instead of panicking on the next cable event.
 */

#define LCD_DEPTH  16
#define LCD_PIXELFORMAT RGB565

#define CONFIG_LCD LCD_S5L8740

/*
 * The panel takes no command sequence of its own -- the LCDIF is the whole
 * story -- so a real off/on cycle is possible. See lcd-nano7g.c.
 */
#define HAVE_LCD_SHUTDOWN
#define HAVE_LCD_ENABLE

/* Define this to have CPU boosted while scrolling in the UI */
#define HAVE_GUI_BOOST

#define AB_REPEAT_ENABLE

/* Define this if you do software codec */
#define CONFIG_CODEC SWCODEC

/*
 * RTC: a plain little-endian 32-bit seconds counter in D1830 registers
 * 124..127 (RetailOS sub_16517E). There is no separate RTC chip.
 *
 * The epoch the stock firmware uses is NOT established -- the driver treats
 * the counter as Unix time, which is self-consistent but will not
 * necessarily agree with what RetailOS displays.
 */
#define CONFIG_RTC RTC_D1830

/*
 * Audio codec: Cirrus CS42L81 / Apple 338S1146 on SPI0 @0x3C300000.
 *
 * Built from the RetailOS RE and live captures rather than ported from the
 * Linux driver, which is still silent for reasons that are not understood --
 * copying its structure would risk copying the fault.
 *
 * Software volume: the RE establishes that 0x0527 = 0x60 is the stock
 * playback level and 0xFF is silence, but not the step size in between, and
 * inventing a dB mapping would be a guess in the one place a guess is
 * audible.
 */
#define HAVE_CS42L81
/*
 * With no real codec there is no hardware volume control, so volume is
 * applied in software. This also becomes the fallback once cs42l81 lands
 * if its analog gain path turns out to be unusable.
 */
#define HAVE_SW_VOLUME_CONTROL

/* Define this for LCD backlight available -- MMIO @0x3E000000 */
#define HAVE_BACKLIGHT
#define HAVE_BACKLIGHT_BRIGHTNESS

/*
 * FM radio. Not a tuner chip on a bus: the receiver lives inside the BCM2078
 * Bluetooth controller and is driven by HCI vendor opcode 0xFC15 over UART1.
 * Audio comes back as digital PCM on IIS2, not over HCI.
 */
#define CONFIG_TUNER BCM2078_TUNER
#define HAVE_RADIO_REGION
#define HAVE_FMRADIO_IN

/*
 * FM audio arrives as digital PCM on IIS2 rather than as an analog line-in,
 * so the radio counts as a recordable input source even though there is no
 * ADC in the path.
 */
#define INPUT_SRC_CAPS (SRC_CAP_FMRADIO)

/*
 * Capacitive touchscreen (TI 343S0538 "Nimbus") on SPI2. This is the device's
 * real navigation surface; the five physical keys supplement it.
 */
#define HAVE_TOUCHSCREEN
#define HAVE_BUTTON_DATA

/* Define this if you have a software controlled poweroff */
#define HAVE_SW_POWEROFF

/* The number of bytes reserved for loadable codecs */
#define CODEC_SIZE 0x100000

/* The number of bytes reserved for loadable plugins */
#define PLUGIN_BUFFER_SIZE 0x80000

/*
 * TODO: real capacity for the N31 pack. The DT simple-battery node carries
 * 200 mAh / 740 mWh design values; these are placeholders until the D1830
 * fuel gauge scale is confirmed.
 */
#define BATTERY_CAPACITY_DEFAULT 220
#define BATTERY_CAPACITY_MIN     150
#define BATTERY_CAPACITY_MAX     300
#define BATTERY_CAPACITY_INC      10

#define CONFIG_BATTERY_MEASURE VOLTAGE_MEASURE

/* Hardware controlled charging with monitoring */
#define CONFIG_CHARGING CHARGING_MONITOR

/* define current usage levels -- TODO: measure */
#define CURRENT_NORMAL     17
#define CURRENT_BACKLIGHT  23

/* define this if the unit can be powered or charged via USB */
#define HAVE_USB_POWER

/* define this if the hardware can be powered off while charging */
#define HAVE_POWEROFF_WHILE_CHARGING

/* The exact type of CPU */
#define CONFIG_CPU S5L8740

/*
 * TODO: confirm. The N31 CLKCON tree is only partly RE'd (clk-s5l8702.c does
 * an ungate-all rather than real rate control), so no frequency scaling yet.
 */
#define CPU_FREQ 400000000

/* I2C interface -- IIC0 @0x3C600000, IIC1 @0x3C900000 */
#define CONFIG_I2C I2C_S5L8740

/*
 * Storage: Toshiba NAND behind FMC/FMSS @0x38A00000 with an Apple Whimory
 * derived FTL. Phase 5. Until then the target links against a stub and has no
 * storage, which is why the bootloader path is DFU/U-Boot rather than disk.
 */
#define CONFIG_STORAGE STORAGE_NAND
#define CONFIG_NAND 0
#define HAVE_FLASH_STORAGE
#define HAVE_STORAGE_FLUSH

/* Whimory BPB reports 4096-byte sectors, not the 2048 the nano4g assumes. */
#define SECTOR_SIZE 4096

/* Offset ( in the firmware file's header ) to the file CRC */
#define FIRMWARE_OFFSET_FILE_CRC 0

/* Offset ( in the firmware file's header ) to the real data */
#define FIRMWARE_OFFSET_FILE_DATA 8

#define BOOTFILE_EXT "ipod"
#define BOOTFILE "rockbox." BOOTFILE_EXT
#define BOOTDIR "/.rockbox"

/** Port-specific settings **/

/*
 * Backlight MMIO takes 0..0x3f at +0x08; U-Boot already writes 62.
 */
#define MIN_BRIGHTNESS_SETTING      1
#define MAX_BRIGHTNESS_SETTING      0x3f
#define DEFAULT_BRIGHTNESS_SETTING  0x3e

/*
 * USB: DWC2 OTG @0x38400000 behind the PHY @0x3C400000.
 *
 * DESIGNWARE gives this target bulk, interrupt and isochronous transfers, so
 * the USB stack enables storage, HID, charging-only and -- because of the
 * isochronous support -- USB audio. With an Apple vendor id it also picks up
 * the iAP class.
 *
 * Mass storage is only useful once the FTL lands (Phase 5); the other modes
 * work without storage.
 */
#define CONFIG_USBOTG USBOTG_DESIGNWARE
#define HAVE_USBSTACK
#define USB_VENDOR_ID 0x05AC
#define USB_PRODUCT_ID 0x1266
#define USB_DEVBSS_ATTR __attribute__((aligned(32)))
#define USB_READ_BUFFER_SIZE (1024*24)
#define USB_DW_CLOCK 0
#define USB_DW_TURNAROUND 5
#define HAVE_USB_CHARGING_ENABLE
#define HAVE_USB_HID_MOUSE

/* Serial: SEC UART3 @0x3DD00000 is the debug console for the whole port. */
#define HAVE_SERIAL

/*
 * No PP-style hardware click, no piezo on N31.
 */
