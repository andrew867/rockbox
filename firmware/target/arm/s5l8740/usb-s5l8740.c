/***************************************************************************
 * USB for Apple S5L8740 (iPod nano 7G) -- Phase 6 placeholder.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#include "config.h"
#include "system.h"
#include "usb.h"
#include "pmu-target.h"

/*
 * The controller is a DWC2 at 0x38400000 behind the PHY at 0x3C400000, and
 * Rockbox already has a DesignWare driver -- so this is mostly a wiring job
 * once the port is otherwise healthy.
 *
 * It is deliberately left alone for now. The device is loaded over DFU
 * through U-Boot, and taking the USB controller away from the bootloader
 * mid-boot during bring-up would remove the recovery path we are relying on.
 *
 * Cable presence still works, because that comes from the PMIC over I2C
 * rather than from the USB block itself.
 */

int usb_detect(void)
{
    return pmu_is_usb_present() ? USB_INSERTED : USB_EXTRACTED;
}

void usb_init_device(void)
{
}

void usb_enable(bool on)
{
    (void)on;
}

void usb_attach(void)
{
}
