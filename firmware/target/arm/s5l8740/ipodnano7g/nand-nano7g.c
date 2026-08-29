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
#include <stdio.h>
#include "lcd.h"
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

/* Cleared until the volume is mounted; gates the boot-time read counter. */
bool mount_done;

/*
 * One-line trace for the mount path.
 *
 * disk_mount_all() stops before issuing a single read, so the read counter --
 * which only exists inside nand_read_sectors() -- can never fire. The failure
 * is in the disk layer above storage, and nothing there says anything.
 *
 * No sleep: these are called in sequence and a full frame is already ~20ms,
 * which is long enough to see the sequence stop and short enough not to
 * change the timing of what is being measured.
 */
void n31_trace(const char *tag)
{
    /*
     * Prove entry before touching the display. If ORANGE (before the call)
     * appears and this CYAN does not, the hang is the call itself; if both
     * appear and no text follows, the LCD path is what is stuck.
     */
    beacon_stage(BEACON_CYAN);

    lcd_clear_display();
    lcd_putsxy(4, 4,  "N7G disk");
    lcd_putsxy(4, 20, tag);
    lcd_update();

    /*
     * Hold. Without this the nine markers flash past in under 200ms, which
     * looks identical to none of them painting at all -- and "nothing
     * appeared" was read as "the function is never entered" when it may
     * simply have been too fast to see.
     */
    sleep(HZ / 4);
}

/* Points at the trace counter while the mount is running; NULL after. */
static unsigned *mount_fails;

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
    beacon_mark(BEACON_WHITE);

    storage_ready = (ftl_recover() == 0);

    /*
     * Report the mount, pass or fail, and hold it long enough to read.
     *
     * A failed mount used to be a single word on the progress screen, and the
     * next thing that happened was a panic on the following USB event -- by
     * which point every number that would have explained it was gone. These
     * five lines separate the cases that look identical from the outside:
     *
     *   no context, and a written-superblock count near zero
     *       the sweep found nothing; a NAND or classification problem
     *   no context, but plenty of written superblocks
     *       the context failed to parse and the scan fallback ran
     *   context loaded, ranges near zero
     *       the context parsed but produced no map
     *   ranges present, phase "no-bpb"
     *       the map is built and FAT does not recognise it -- an L2V or
     *       address-translation problem, not a storage one
     */
    /*
     * Report only a FAILED mount, and only then hold the boot.
     *
     * A working mount has nothing to say that is worth two seconds of every
     * startup -- the progress bar already showed the work happening, and
     * stopping afterwards to display "it worked" is the firmware admiring
     * itself. A failure is different: these numbers are the entire account of
     * what went wrong, and without them the next symptom is a panic several
     * seconds later with none of this on screen.
     */
    if (!storage_ready) {
        unsigned ranges = 0, mapped = 0, closed = 0, open = 0;
        unsigned written = 0, empty = 0, replayed = 0, dropped = 0;
        unsigned skipped = 0, fallback = 0, form0 = 0, form1 = 0, rdfail = 0;
        bool cxt = false, overflow = false;

        ftl_get_stats(&ranges, &mapped, &closed, &open);
        ftl_get_cxt_stats(&cxt, &written, &empty, &replayed, &dropped,
                          &skipped, &overflow, &fallback, &form0, &form1, &rdfail);

        lcd_clear_display();
        lcd_putsf(0, 0, "FTL %s", ftl_last_phase());
        lcd_putsf(0, 1, "cxt %s  wr %u", cxt ? "yes" : "NO", written);
        lcd_putsf(0, 2, "empty %u  rpl %u", empty, replayed);
        lcd_putsf(0, 3, "rng %u  map %u", ranges, mapped);
        lcd_putsf(0, 4, "cl %u  op %u  drop %u", closed, open, dropped);
        lcd_putsf(0, 5, "skip %u  fb %u %s", skipped, fallback,
                  overflow ? "OVERFLOW" : "");
        lcd_putsf(0, 6, "btoc %u/%u  rdf %u", form0, form1, rdfail);
        lcd_update();

        sleep(5 * HZ);
    }

    beacon_mark(storage_ready ? BEACON_BLUE : BEACON_RED);

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

    /*
     * Live read counter during the mount.
     *
     * disk_mount_all() stops with no output and no way to tell a slow mount
     * from a wedged one -- the two look identical from outside and need
     * completely different work. A counter that keeps climbing says FAT is
     * grinding through reads; one that freezes names the read it froze on.
     *
     * Every 2048 reads, because a repaint is a full 103,680-pixel frame.
     * Temporary, and it comes out with the stage markers.
     */
    /*
     * Live read trace during the mount.
     *
     * The first attempt printed every 2048 reads and never printed at all,
     * which was itself the finding: the mount wedges in fewer reads than
     * that. A mount only needs a few hundred, so the interval was coarser
     * than the thing it was measuring.
     *
     * Every one of the first 64 reads is shown with its sector, then every
     * 256. That distinguishes the three cases this has been stuck between:
     * a sector that never returns (the number stops on it), FAT looping over
     * a small set (the same sector repeats), and a slow but progressing mount
     * (the number climbs).
     *
     * Failures are counted separately -- nand_read_sectors() returns -1 for
     * an unmapped LBA, and a caller retrying that forever looks exactly like
     * a hang from out here.
     */
    if (!mount_done) {
        static unsigned reads, fails;
        char buf[40];

        reads++;

        if (reads <= 64 || (reads % 256) == 0) {
            lcd_clear_display();
            lcd_putsxy(4, 4,  "N7G stage");
            lcd_putsxy(4, 20, "disk_mount_all");
            snprintf(buf, sizeof(buf), "rd %u  n %d", reads, count);
            lcd_putsxy(4, 36, buf);
            snprintf(buf, sizeof(buf), "lba %lu", (unsigned long)start);
            lcd_putsxy(4, 52, buf);
            snprintf(buf, sizeof(buf), "fail %u", fails);
            lcd_putsxy(4, 68, buf);
            lcd_update();
        }

        mount_fails = &fails;
    }

    last_disk_activity = current_tick;

    for (i = 0; i < count; i++) {
        if (ftl_read_disk_lba((uint32_t)(start + i),
                              out + (size_t)i * NAND_FTL_SECTOR_SIZE)) {
            if (mount_fails)
                (*mount_fails)++;
            return -1;
        }
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
