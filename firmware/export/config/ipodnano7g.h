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
 * The N31 has no clickwheel: navigation is a capacitive touchscreen (TI
 * 343S0538 "Nimbus" on SPI2) plus five physical keys. The touch driver is
 * Phase 6 work and is not wired up yet, so for now the port is driven
 * entirely by the physical keys and HAVE_TOUCHSCREEN stays off.
 */
#define CONFIG_KEYPAD IPOD_NANO7G_PAD

/* LCD dimensions -- LCDIF @0x38300000, confirmed on glass */
#define LCD_WIDTH  240
#define LCD_HEIGHT 432
/* sqrt(432^2 + 240^2) / 2.5" = 197.7 */
#define LCD_DPI 198
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
 * TODO: the D1830 PMIC has an RTC but the register map is not RE'd yet.
 */
#define CONFIG_RTC 0

/*
 * Audio codec: Cirrus CS42L81 / Apple 338S1146 on SPI0 @0x3C300000.
 *
 * The control path is coded in Linux (cs42l81-spi.c) but analog output does
 * not work there yet -- a 1 kHz tone measures about -66 dBFS at the jack
 * against RetailOS's -16 dBFS. Until that is solved, the port declares the
 * dummy codec so the audio stack builds and runs silently rather than
 * pretending to a driver that cannot make sound. Phase 4 swaps this for
 * HAVE_CS42L81 and a real firmware/drivers/audio/cs42l81.c.
 */
#define HAVE_DUMMY_CODEC

/* Define this for LCD backlight available -- MMIO @0x3E000000 */
#define HAVE_BACKLIGHT
#define HAVE_BACKLIGHT_BRIGHTNESS

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
 * USB: DWC2 OTG @0x38400000 behind the PHY @0x3C400000. Phase 6 -- the
 * controller is left alone for now so that the DFU/U-Boot load path is not
 * disturbed mid-boot.
 */
/* #define CONFIG_USBOTG USBOTG_DESIGNWARE */

/* Serial: SEC UART3 @0x3DD00000 is the debug console for the whole port. */
#define HAVE_SERIAL

/*
 * No PP-style hardware click, no piezo on N31.
 */
