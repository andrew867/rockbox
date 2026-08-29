/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * USB OTG (DWC2) and PHY for Apple S5L8740 (iPod nano 7G).
 *
 * Ported from tools/linux-n31/drivers/phy-s5l8702-usb2.c plus the dwc2
 * parameters measured on glass.
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
#include "usb.h"
#include "usb-designware.h"
#include "clocking-s5l8740.h"
#include "pmu-target.h"

/* Rockbox has udelay() but no mdelay(). These are init-time only. */
#define mdelay(ms)  udelay((ms) * 1000)

/* USB2 PHY @0x3C400000 */
#define OTGPHY_PWR      (*(REG32_PTR_T)(USB_PHY_BASE + 0x000))
#define OTGPHY_CLK      (*(REG32_PTR_T)(USB_PHY_BASE + 0x004))
#define OTGPHY_RSTCON   (*(REG32_PTR_T)(USB_PHY_BASE + 0x008))
#define OTGPHY_MODE     (*(REG32_PTR_T)(USB_PHY_BASE + 0x01c))

/* DWC2 registers we touch directly, before the generic driver takes over. */
#define OTG_PCGCCTL     (*(REG32_PTR_T)(USB_OTG_BASE + 0xe00))
#define OTG_DCTL        (*(REG32_PTR_T)(USB_OTG_BASE + 0x804))
#define DCTL_SFTDISCON  (1 << 1)

/*
 * FIFO layout.
 *
 * Glass reading of GHWCFG3 on this SoC: DFIFO depth 2080 words, dedicated
 * FIFOs, 6 IN endpoints. RetailOS sub_1B543A sizes the non-periodic TX FIFO
 * at 32 words and caps the first IN endpoint at 512 bytes.
 *
 * Isochronous support is what gives this target USB audio, so the periodic
 * TX FIFO is sized generously rather than minimally.
 */
const struct usb_dw_config usb_dw_config =
{
    .phytype = DWC_PHYTYPE_UTMI_16,

    /* 2080 words available */
    .rx_fifosz   = 0x400,
    .nptx_fifosz = 0x20,    /* RetailOS NP = 32 words */
    .ptx_fifosz  = 0x300,   /* dedicated IN FIFOs, incl. isochronous */

#ifdef USB_DW_ARCH_SLAVE
    .disable_double_buffering = false,
#else
    .ahb_burst_len = HBSTLEN_INCR8,
#ifndef USB_DW_SHARED_FIFO
    .ahb_threshold = 8,
#endif
#endif
};

void usb_dw_target_enable_clocks(void)
{
    /*
     * U-Boot's DFU gadget leaves D+ pulled up and the core possibly in
     * suspend. Drop the pull-up and clear suspend BEFORE resetting the PHY,
     * or the host sees a device that never re-enumerates -- this was the
     * failure mode when the Linux port first took the controller over from
     * the bootloader.
     */
    OTG_DCTL |= DCTL_SFTDISCON;
    OTG_PCGCCTL = 0;
    mdelay(10);

    OTGPHY_PWR = 0;
    mdelay(10);
    OTGPHY_RSTCON = 1;
    mdelay(10);
    OTGPHY_RSTCON = 0;
    mdelay(10);
    OTGPHY_MODE = 6;
    OTGPHY_CLK = 1;

    /* U-Boot waits ~400 ms for PLL lock; so do we. */
    mdelay(400);

    OTG_DCTL &= ~DCTL_SFTDISCON;
}

void usb_dw_target_disable_clocks(void)
{
    OTGPHY_PWR = 0xff;
    mdelay(10);
    OTGPHY_RSTCON = 0xff;
    mdelay(10);
    OTGPHY_MODE = 4;
}

void usb_dw_target_enable_irq(void)
{
    VIC0INTENABLE = 1 << IRQ_USB_OTG;
}

void usb_dw_target_disable_irq(void)
{
    VIC0INTENCLEAR = 1 << IRQ_USB_OTG;
}

void usb_dw_target_clear_irq(void)
{
}

/*
 * The interrupt body lives in the DesignWare driver as INT_USB_FUNC(); the
 * vector table in system-s5l8740.c points IRQ_USB_OTG straight at it.
 */

/*
 * Cable presence comes from the PMIC over I2C rather than from the USB block,
 * so it works whether or not the controller is powered.
 */
int usb_detect(void)
{
    return pmu_is_usb_present() ? USB_INSERTED : USB_EXTRACTED;
}

void usb_init_device(void)
{
    usb_dw_target_disable_irq();
}

void usb_enable(bool on)
{
    if (on)
        usb_core_init();
    else
        usb_core_exit();
}

/* usb_attach() belongs to the DesignWare driver on this controller. */
