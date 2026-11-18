/*
 * parallel-simlink.c - Externer Teilnehmer am IEEE-488-Bus ueber simlink.
 *
 * Ein anderer Emulator (z. B. picoem mit der petSD2-Firmware) haengt ueber
 * gemeinsamen Speicher am IEEE-Bus. Seine Leitungen erscheinen in VICE unter
 * dem Teilnehmerbit PARALLEL_EXT; die Leitungen von PET und Laufwerken werden
 * bei jeder Aenderung mit Zeitstempel veroeffentlicht.
 *
 * Massstab ist der echte PET: REN ist dort fest auf Masse (dauernd aktiv),
 * IFC haengt am Reset, SRQ geht auf PIA2 CB1.
 *
 * Ressource "IEEESimlink" (Name des Busses, leer = aus),
 * "IEEESimlinkStep" (Abgleich spaetestens alle n Takte).
 *
 * This file is part of VICE, the Versatile Commodore Emulator.
 * See README for copyright notice.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */

#include "vice.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "alarm.h"
#include "cmdline.h"
#include "drive.h"
#include "lib.h"
#include "log.h"
#include "machine.h"
#include "maincpu.h"
#include "parallel.h"
#include "resources.h"
#include "types.h"
#include "util.h"

#if !defined(__MSDOS__)
#define HAVE_SIMLINK 1
#include "simlink.h"
#endif

void (*parallel_ext_srq_hook)(int active, CLOCK clk) = NULL;

static char *simlink_name = NULL;
static int simlink_step = 64;

#ifdef HAVE_SIMLINK

static log_t sl_log = LOG_DEFAULT;
static simlink_t *sl = NULL;
static int sl_slot = -1;
static alarm_t *sl_alarm = NULL;
static uint64_t ps_per_cycle = 1000000;
static int started = 0;
static int ifc_active = 0;
static CLOCK ifc_end = 0;
static uint8_t ext_srq = 0;

static uint64_t last_ps = 0;
static int64_t base_ps = 0;     /* simlink-Zeit = base + clk * ps_per_cycle */

/* Zeitstempel duerfen nicht rueckwaerts laufen (Alarm-Offset, Catch-up) */
static uint64_t clk2ps(CLOCK c)
{
    uint64_t t = (uint64_t)((int64_t)((uint64_t)c * ps_per_cycle) + base_ps);
    if (t < last_ps) {
        t = last_ps;
    }
    last_ps = t;
    return t;
}

/* Leitungen von PET und internen Laufwerken (ohne den externen Teilnehmer) */
static uint8_t vice_pull(void)
{
    uint8_t p = SL_IEEE_REN;            /* echter PET: REN fest auf Masse */
    if (parallel_atn & ~PARALLEL_EXT) {
        p |= SL_IEEE_ATN;
    }
    if (parallel_dav & ~PARALLEL_EXT) {
        p |= SL_IEEE_DAV;
    }
    if (parallel_nrfd & ~PARALLEL_EXT) {
        p |= SL_IEEE_NRFD;
    }
    if (parallel_ndac & ~PARALLEL_EXT) {
        p |= SL_IEEE_NDAC;
    }
    if (parallel_eoi & ~PARALLEL_EXT) {
        p |= SL_IEEE_EOI;
    }
    if (ifc_active) {
        p |= SL_IEEE_IFC;
    }
    return p;
}

static void sl_alarm_handler(CLOCK offset, void *data);

/* beim Programmende abmelden, damit kein toter Teilnehmer im Bus bleibt */
static void sl_atexit(void)
{
    if (sl) {
        simlink_detach(sl, sl_slot);
        simlink_close(sl, 0);
        sl = NULL;
    }
}
static void parallel_ext_sync(CLOCK clk);

static int stopping = 0;

static void sl_stop(void)
{
    if (sl) {
        simlink_detach(sl, sl_slot);
        simlink_close(sl, 0);
        sl = NULL;
        sl_slot = -1;
        /* Leitungen des externen Teilnehmers freigeben (ohne neu zu verbinden) */
        stopping = 1;
        parallel_clr_atn((uint8_t)~PARALLEL_EXT);
        parallel_clr_dav((uint8_t)~PARALLEL_EXT);
        parallel_clr_nrfd((uint8_t)~PARALLEL_EXT);
        parallel_clr_ndac((uint8_t)~PARALLEL_EXT);
        parallel_clr_eoi((uint8_t)~PARALLEL_EXT);
        parallel_ext_set_bus(0xff);
        stopping = 0;
        if (ext_srq && parallel_ext_srq_hook) {
            parallel_ext_srq_hook(0, maincpu_clk);
        }
        ext_srq = 0;
    }
    if (sl_alarm) {
        alarm_unset(sl_alarm);
    }
    drive_ext_sync_remove(parallel_ext_sync);
    started = 0;
}

static void sl_start(void)
{
    long cps;

    if (started || simlink_name == NULL || *simlink_name == 0) {
        return;
    }
    if (maincpu_alarm_context == NULL) {
        return;     /* zu frueh, spaeter erneut */
    }
    started = 1;
    if (sl_log == LOG_DEFAULT) {
        sl_log = log_open("Simlink");
    }
    sl = simlink_open(simlink_name, SIMLINK_BUS_IEEE, 0, 0xff);
    if (!sl) {
        sl = simlink_open(simlink_name, SIMLINK_BUS_IEEE, 1, 0xff);
    }
    if (!sl) {
        log_error(sl_log, "Cannot open simlink bus '%s'.", simlink_name);
        return;
    }
    sl_slot = simlink_attach(sl, SL_TYPE_VICE, 1);
    if (sl_slot < 0) {
        log_error(sl_log, "No free participant slot on bus '%s'.", simlink_name);
        simlink_close(sl, 0);
        sl = NULL;
        return;
    }
    {
        static int atexit_done = 0;
        if (!atexit_done) {
            atexit(sl_atexit);
            atexit_done = 1;
        }
    }
    cps = machine_get_cycles_per_second();
    ps_per_cycle = cps > 0 ? (uint64_t)(1000000000000ULL / (uint64_t)cps) : 1000000;
    if (!sl_alarm) {
        sl_alarm = alarm_new(maincpu_alarm_context, "Simlink", sl_alarm_handler, NULL);
    }
    /* Zeitbasis an den Bus anlehnen (Spaeteinsteiger beginnt beim langsamsten) */
    base_ps = (int64_t)simlink_start_time(sl, sl_slot) - (int64_t)((uint64_t)maincpu_clk * ps_per_cycle);
    last_ps = 0;
    alarm_set(sl_alarm, maincpu_clk + (CLOCK)simlink_step);
    drive_ext_sync_add(parallel_ext_sync);
    simlink_set(sl, sl_slot, clk2ps(maincpu_clk), vice_pull(),
                (uint8_t)~parallel_bus_vice());
    log_message(sl_log, "Attached to IEEE bus '%s' as participant %d (%lld ps/cycle).",
                simlink_name, sl_slot, (long long)ps_per_cycle);
}

/* Bei jeder Leitungsaenderung von PET oder Laufwerk aufgerufen. */
void parallel_ext_notify(void)
{
    if (stopping) {
        return;
    }
    if (!started) {
        sl_start();
    }
    if (!sl) {
        return;
    }
    simlink_set(sl, sl_slot, clk2ps(maincpu_clk), vice_pull(), (uint8_t)~parallel_bus_vice());
}

/* Haken vor jedem Nachziehen der Laufwerke - VICE ruft das vor jedem Lesen
   der IEEE-Leitungen durch den Rechner auf (drive_catch_up_hook): holt den
   Stand der externen Teilnehmer zum Zeitpunkt clk und speist ihn unter
   PARALLEL_EXT ein. */
static void parallel_ext_sync(CLOCK clk)
{
    uint8_t l, d, srq;

    if (!started) {
        sl_start();
    }
    if (!sl) {
        return;
    }
    l = simlink_read_others(sl, sl_slot, clk2ps(clk), &d);

    if (l & SL_IEEE_ATN) {
        parallel_set_atn(PARALLEL_EXT);
    } else {
        parallel_clr_atn((uint8_t)~PARALLEL_EXT);
    }
    if (l & SL_IEEE_DAV) {
        parallel_set_dav(PARALLEL_EXT);
    } else {
        parallel_clr_dav((uint8_t)~PARALLEL_EXT);
    }
    if (l & SL_IEEE_NRFD) {
        parallel_set_nrfd(PARALLEL_EXT);
    } else {
        parallel_clr_nrfd((uint8_t)~PARALLEL_EXT);
    }
    if (l & SL_IEEE_NDAC) {
        parallel_set_ndac(PARALLEL_EXT);
    } else {
        parallel_clr_ndac((uint8_t)~PARALLEL_EXT);
    }
    if (l & SL_IEEE_EOI) {
        parallel_set_eoi(PARALLEL_EXT);
    } else {
        parallel_clr_eoi((uint8_t)~PARALLEL_EXT);
    }
    parallel_ext_set_bus((uint8_t)~d);

    srq = (l & SL_IEEE_SRQ) ? 1 : 0;
    if (srq != ext_srq) {
        ext_srq = srq;
        if (parallel_ext_srq_hook) {
            parallel_ext_srq_hook(srq, clk);
        }
    }
}

/* Reset des Rechners: IFC kurz aktiv (am echten PET haengt IFC am Reset). */
void parallel_ext_reset(void)
{
    if (!started) {
        sl_start();
    }
    if (!sl) {
        return;
    }
    ifc_active = 1;
    ifc_end = maincpu_clk + 100;
    parallel_ext_notify();
}

static void sl_alarm_handler(CLOCK offset, void *data)
{
    CLOCK now = maincpu_clk - offset;

    if (ifc_active && now >= ifc_end) {
        ifc_active = 0;
        parallel_ext_notify();
    }
    parallel_ext_sync(now);
    if (sl_alarm && sl) {
        alarm_set(sl_alarm, now + (CLOCK)simlink_step);
    }
}

#else  /* !HAVE_SIMLINK */

void parallel_ext_notify(void)
{
}


void parallel_ext_reset(void)
{
}

static void sl_stop(void)
{
}

#endif

/* ------------------------------------------------------------------------- */

static int set_simlink_name(const char *val, void *param)
{
    if (util_string_set(&simlink_name, val ? val : "")) {
        return 0;
    }
    sl_stop();
    return 0;
}

static int set_simlink_step(int val, void *param)
{
    simlink_step = val < 1 ? 1 : val;
    return 0;
}

static const resource_string_t resources_string[] = {
    { "IEEESimlink", "", RES_EVENT_NO, NULL,
      &simlink_name, set_simlink_name, NULL },
    RESOURCE_STRING_LIST_END
};

static const resource_int_t resources_int[] = {
    { "IEEESimlinkStep", 64, RES_EVENT_NO, NULL,
      &simlink_step, set_simlink_step, NULL },
    RESOURCE_INT_LIST_END
};

int parallel_simlink_resources_init(void)
{
    if (resources_register_string(resources_string) < 0) {
        return -1;
    }
    return resources_register_int(resources_int);
}

static const cmdline_option_t cmdline_options[] = {
    { "-ieeesimlink", SET_RESOURCE, CMDLINE_ATTRIB_NEED_ARGS,
      NULL, NULL, "IEEESimlink", NULL,
      "<name>", "Attach an external participant to the IEEE-488 bus via simlink shared memory (empty: off)" },
    { "-ieeesimlinkstep", SET_RESOURCE, CMDLINE_ATTRIB_NEED_ARGS,
      NULL, NULL, "IEEESimlinkStep", NULL,
      "<cycles>", "Synchronise with the simlink bus at least every <cycles> CPU cycles" },
    CMDLINE_LIST_END
};

int parallel_simlink_cmdline_options_init(void)
{
    return cmdline_register_options(cmdline_options);
}
