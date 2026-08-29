/***************************************************************************
 * NAND storage for the iPod nano 7G (N31) -- Phase 5 placeholder.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 ****************************************************************************/
#include "config.h"
#include "system.h"
#include "storage.h"

static long last_disk_activity = -1;

/*
 * Not implemented yet, and deliberately so.
 *
 * The hardware is a Toshiba NAND behind the FMC/FMSS at 0x38A00000 with an
 * Apple Whimory-derived FTL on top. Both halves exist in the Linux tree
 * (tools/linux-n31/drivers/nand-s5l8740.c, ftl-s5l8740.c) and page-level
 * reads work on glass, but the L2V map is not yet proven end to end -- the
 * PASS test is LBA0 resolving to a real boot sector that mounts read-only.
 *
 * Until that lands, this target has no storage and is loaded over DFU/U-Boot.
 * Returning 0 from nand_init() rather than an error keeps the rest of the
 * firmware booting so the display, buttons and audio work can proceed.
 *
 * NOTE for whoever implements this: land the read path first and leave
 * writes unimplemented. A bad write to the Whimory maps destroys the user
 * data area, and there is no recovery short of a restore.
 */

int nand_event(long id, intptr_t data)
{
    (void)id;
    (void)data;
    return 0;
}

long nand_last_disk_activity(void)
{
    return last_disk_activity;
}

int nand_init(void)
{
    return 0;
}

void nand_spindown(int seconds)
{
    (void)seconds;
}

void nand_spin(void)
{
}

#ifdef HAVE_STORAGE_FLUSH
int nand_flush(void)
{
    return 0;
}
#endif

void nand_sleepnow(void)
{
}

void nand_enable(bool on)
{
    (void)on;
}

void nand_get_info(IF_MD(int drive,) struct storage_info *info)
{
    IF_MD((void)drive;)
    info->sector_size = SECTOR_SIZE;
    info->num_sectors = 0;
    info->vendor = "Apple";
    info->product = "iPod nano 7G NAND";
    info->revision = "0.00";
}

int nand_read_sectors(IF_MD(int drive,) unsigned long start, int count,
                      void *buf)
{
    IF_MD((void)drive;)
    (void)start; (void)count; (void)buf;
    return -1;
}

int nand_write_sectors(IF_MD(int drive,) unsigned long start, int count,
                       const void *buf)
{
    IF_MD((void)drive;)
    (void)start; (void)count; (void)buf;
    return -1;
}

bool nand_present(IF_MD_NONVOID(int drive))
{
    IF_MD((void)drive;)
    return false;
}
