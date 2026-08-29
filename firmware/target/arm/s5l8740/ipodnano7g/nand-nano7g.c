/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Storage layer for the iPod nano 7G (N31): Whimory FTL over the FMSS NAND.
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
#include <string.h>
#include "config.h"
#include "system.h"
#include "kernel.h"
#include "storage.h"
#include "nand-s5l8740.h"
#include "ftl-s5l8740.h"
#include "boot-beacon.h"

/*
 * Rockbox asks for 4096-byte sectors (SECTOR_SIZE in the board config), which
 * is exactly one Whimory logical page, so this layer is a thin shim onto
 * ftl_read_disk_lba().
 *
 * READ ONLY. nand_write_sectors() returns an error and always will until the
 * read path has been trusted on hardware for a while: a bad write to the
 * Whimory maps destroys the user data area, and the only recovery is a full
 * restore from iTunes.
 */

static long last_disk_activity = -1;
static bool storage_ready;

int nand_init(void)
{
    if (!ftl_init())
        return 0;   /* no storage, but the firmware still runs */

    /*
     * The FTL scan runs at boot again.
     *
     * It was deferred for a specific reason, and that reason has expired.
     * Rebuilding the map is ~16,700 page reads on a NAND path that had never
     * run on hardware, and a hang in the middle of it was indistinguishable
     * from dead firmware on a device with no serial console. The condition
     * attached to the deferral was that everything else had to be seen
     * working first.
     *
     * It has been: the panel, the logo and Rockbox's own USB screen all
     * render, and the assembly and C beacons both paint reliably. A hang in
     * here is now a visible event with an address rather than a black screen,
     * which is exactly what was missing before.
     *
     * Leaving it deferred has its own cost, and it is no longer the smaller
     * one. With no map there is no partition, so disk_mount_all() fails, and
     * usb_slave_mode(false) panics with "mount: 0" the moment the host
     * releases storage back to us -- which is what happens on every USB
     * unplug. The firmware cannot survive a cable event without storage.
     *
     * Bracketed with beacons so a stall names its own phase.
     */
    beacon_mark(BEACON_MAGENTA);

    if (ftl_recover() == 0) {
        storage_ready = true;
        beacon_mark(BEACON_CYAN);
    } else {
        /*
         * Not fatal here. Reporting a clean "no storage" lets the firmware
         * finish booting and say so on a working screen, which is far more
         * useful than refusing to start.
         */
        beacon_mark(BEACON_RED);
    }

    last_disk_activity = current_tick;
    return 0;
}

/*
 * Run the FTL scan on demand. Separate from nand_init() so a hang here is
 * something the user chose to trigger, in front of a working UI, rather than
 * a silent stall three seconds into boot.
 */
int ftl_mount_now(void)
{
    int ret = ftl_recover();

    storage_ready = (ret == 0);
    return ret;
}

void nand_spindown(int seconds)
{
    (void)seconds;
}

void nand_spin(void)
{
    last_disk_activity = current_tick;
}

#ifdef HAVE_STORAGE_FLUSH
int nand_flush(void)
{
    return 0;       /* nothing is buffered; this driver never writes */
}
#endif

void nand_sleepnow(void)
{
}

void nand_enable(bool on)
{
    (void)on;
}

long nand_last_disk_activity(void)
{
    return last_disk_activity;
}

int nand_event(long id, intptr_t data)
{
    (void)id;
    (void)data;
    return 0;
}

void nand_get_info(IF_MD(int drive,) struct storage_info *info)
{
    IF_MD((void)drive;)

    info->sector_size = NAND_FTL_SECTOR_SIZE;
    info->num_sectors = storage_ready ? ftl_disk_sectors() : 0;
    info->vendor = "Apple";
    info->product = "iPod nano 7G NAND";
    info->revision = "1.00";
}

int nand_read_sectors(IF_MD(int drive,) unsigned long start, int count,
                      void *buf)
{
    uint8_t *out = buf;
    int i;

    IF_MD((void)drive;)

    if (!storage_ready)
        return -1;

    last_disk_activity = current_tick;

    for (i = 0; i < count; i++) {
        if (ftl_read_disk_lba((uint32_t)(start + i),
                              out + (size_t)i * NAND_FTL_SECTOR_SIZE))
            return -1;
    }

    return 0;
}

int nand_write_sectors(IF_MD(int drive,) unsigned long start, int count,
                       const void *buf)
{
    IF_MD((void)drive;)
    (void)start; (void)count; (void)buf;

    /*
     * Deliberately unimplemented. Writing Whimory correctly means allocating
     * from the free superblock list, stamping a new weave, updating the BTOC
     * and eventually running garbage collection -- and getting any of that
     * wrong corrupts the map for data that is already on the device.
     *
     * Rockbox is perfectly usable read-only: it plays music and keeps its
     * settings in memory. That trade is worth making until the read path has
     * a lot of hours on it.
     */
    return -1;
}

bool nand_present(IF_MD_NONVOID(int drive))
{
    IF_MD((void)drive;)
    return storage_ready;
}
