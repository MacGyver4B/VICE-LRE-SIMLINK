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
static int started = 0;
static int stopping = 0;
static int ifc_active = 0;
static CLOCK ifc_end = 0;
static uint8_t ext_srq = 0;

/* Zeit wie beim IEC-Teilnehmer: simlink-Zeit = base + Takt * 1e12 / hz */
static uint64_t hz = 1000000;
static int64_t base_ps = 0;
static uint64_t done_ps = 0;
static int start_pending = 0;   /* Startzustand der anderen noch uebernehmen */        /* bis hierher abgeglichen */
static int in_sync = 0;
static uint8_t pub_pull = 0, pub_data = 0;
static uint64_t pub_ps = 0;
static int ext_valid = 0;
static uint8_t ext_pull = 0, ext_data = 0;

/* Zusammenhang Haupt-/Laufwerkstakt je Laufwerk (vor jedem Nachziehen) */
static drive_sync_snap_t snap[NUM_DISK_UNITS];

#define PS_PER_S 1000000000000ULL

static uint64_t m2ps(CLOCK c)
{
    unsigned __int128 x = (unsigned __int128)c * PS_PER_S / hz;
    return (uint64_t)((int64_t)(uint64_t)x + base_ps);
}

/* letzter Takt mit Zeit <= t */
static CLOCK ps2m_floor(uint64_t t)
{
    unsigned __int128 x;
    CLOCK c;

    if ((int64_t)t <= base_ps) {
        return 0;
    }
    x = (unsigned __int128)(uint64_t)((int64_t)t - base_ps) * hz / PS_PER_S;
    c = (CLOCK)x;
    while (m2ps(c + 1) <= t) {
        c++;
    }
    while (c > 0 && m2ps(c) > t) {
        c--;
    }
    return c;
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

static void publish(uint64_t t)
{
    uint8_t p = vice_pull();
    uint8_t d = (uint8_t)~parallel_bus_vice();

    if (p == pub_pull && d == pub_data) {
        return;
    }
    if (t < pub_ps) {
        t = pub_ps;
    }
    if (t < done_ps) {
        t = done_ps;
    }
    pub_ps = t;
    pub_pull = p;
    pub_data = d;
    simlink_set(sl, sl_slot, t, p, d);
}

/* Zeitstempel fuer eine Aenderung durch 'who' (PARALLEL_*-Bits) */
static uint64_t stamp(uint8_t who)
{
    unsigned int dnr;

    if (in_sync) {
        for (dnr = 0; dnr < NUM_DISK_UNITS; dnr++) {
            if (who & (PARALLEL_DRV0 << dnr)) {
                /* Laufwerkstakt -> Haupttakt, unabhaengig von der Schrittweite */
                return m2ps(drive_sync_main_clock(&snap[dnr], *diskunit_context[dnr]->clk_ptr));
            }
        }
        return done_ps;
    }
    return m2ps(maincpu_clk);
}

static void sl_alarm_handler(CLOCK offset, void *data);
static void parallel_ext_sync(CLOCK clk);
static void sl_start(void);
static void sl_stop(void);

/* beim Programmende abmelden, damit kein toter Teilnehmer im Bus bleibt */
static void sl_atexit(void)
{
    if (sl) {
        simlink_detach(sl, sl_slot);
        simlink_close(sl, 0);
        sl = NULL;
    }
}

/* Vor einer Leitungsaenderung durch den PET (CPU oder Geraeteemulation):
   den Bus bis jetzt abgleichen, solange der alte Zustand noch gilt. Sonst
   veroeffentlicht der Abgleich in parallel_ext_notify() den neuen Zustand
   schon mit den Zeitstempeln der dabei nachgeholten Aenderungen des externen
   Teilnehmers, also bis zu simlink_step Takte zu frueh. */
void parallel_ext_presync(uint8_t who)
{
    if (stopping || !sl || in_sync || !(who & (PARALLEL_CPU | PARALLEL_EMU))) {
        return;
    }
    parallel_ext_sync(maincpu_clk);
}

/* Bei jeder Leitungsaenderung durch PET, Laufwerk oder Geraeteemulation */
void parallel_ext_notify(uint8_t who)
{
    /* Verbunden wird nur an sicheren Stellen (Reset, Einstellung), nie hier:
       der Aufruf kann aus der Laufwerks-CPU kommen. */
    if (stopping || !sl || !(who & (uint8_t)~PARALLEL_EXT)) {
        return;
    }
    if (!in_sync && (who & (PARALLEL_CPU | PARALLEL_EMU))) {
        /* Aenderung durch den PET: Bus bis jetzt abgleichen (meist schon in
           parallel_ext_presync() geschehen, dann ohne Wirkung) */
        parallel_ext_sync(maincpu_clk);
        if (!sl) {
            return;
        }
    }
    publish(stamp(who));
}

/* Laufwerke bis Takt c nachziehen (Haken ist waehrenddessen gesperrt) */
static void run_drives(CLOCK c)
{
    unsigned int dnr;

    for (dnr = 0; dnr < NUM_DISK_UNITS; dnr++) {
        diskunit_context_t *unit = diskunit_context[dnr];
        if (unit && unit->enable) {
            drive_sync_snapshot(unit, &snap[dnr]);
        }
    }
    drive_cpu_execute_all(c);
}

/* Stand des externen Teilnehmers zum Zeitpunkt t (Takt c) einspeisen */
static void apply_ext(uint64_t t, CLOCK c)
{
    uint8_t d, l = simlink_peek_others(sl, sl_slot, t, &d);
    uint8_t srq;

    if (ext_valid && l == ext_pull && d == ext_data) {
        return;
    }
    ext_valid = 1;
    ext_pull = l;
    ext_data = d;

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
            parallel_ext_srq_hook(srq, c);
        }
    }
    /* Laufwerke reagieren in VICE auf ATN sofort (Quittung) */
    publish(t);
}

/* Haken vor jedem Nachziehen der Laufwerke - VICE ruft das vor jedem Lesen
   der IEEE-Leitungen durch den Rechner auf (drive_catch_up_hook). Abgleich
   ohne Vorlauf wie beim IEC-Teilnehmer: jede Aenderung des externen
   Teilnehmers genau zu ihrer Zeit einspeisen, Laufwerke bis dahin nachziehen. */
static void parallel_ext_sync(CLOCK clk)
{
    uint64_t target;

    if (in_sync || !sl) {
        return;
    }
    target = m2ps(clk);
    if (target <= done_ps) {
        return;
    }
    in_sync = 1;
    while (done_ps < target) {
        int err = 0;
        uint64_t m = simlink_peers_progress(sl, sl_slot, &err);
        uint64_t upto, tc;

        if (err) {
            log_error(sl_log, "Peer reported an error, detaching.");
            in_sync = 0;
            sl_stop();
            return;
        }
        /* Fortschritt m: alles vor m ist endgueltig */
        if (m <= done_ps + 1) {
            m = simlink_wait_peers(sl, sl_slot, done_ps + 1);
        }
        upto = (m - 1 < target) ? m - 1 : target;
        /* Ein Eintrag genau zur Startzeit faellt sonst durch, weil
           simlink_next_change erst danach zaehlt. Hier ist m > done_ps,
           der Zustand bei done_ps also endgueltig. */
        if (start_pending) {
            start_pending = 0;
            apply_ext(done_ps, ps2m_floor(done_ps));
        }

        while (simlink_next_change(sl, sl_slot, done_ps, upto, &tc)) {
            CLOCK c = ps2m_floor(tc);
            run_drives(c);
            apply_ext(tc, c);
            done_ps = tc;
            simlink_progress(sl, sl_slot, done_ps + 1);
        }
        if (upto == target) {
            run_drives(clk);
        } else {
            run_drives(ps2m_floor(upto));
        }
        done_ps = upto;
        simlink_progress(sl, sl_slot, done_ps + 1);
    }
    in_sync = 0;
}

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
    drive_ext_exact = 0;
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
    /* Laufwerke auf den aktuellen Stand bringen, bevor die Zeit zaehlt */
    drive_cpu_execute_all(maincpu_clk);

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
    hz = cps > 0 ? (uint64_t)cps : 1000000;
    base_ps = 0;
    base_ps = (int64_t)simlink_start_time(sl, sl_slot) - (int64_t)m2ps(maincpu_clk);
    done_ps = m2ps(maincpu_clk);
    pub_ps = done_ps;
    pub_pull = 0;
    pub_data = 0;
    ext_valid = 0;
    publish(done_ps);
    simlink_progress(sl, sl_slot, done_ps);
    /* Zustand der anderen zur Startzeit beim ersten Abgleich uebernehmen */
    start_pending = 1;

    if (!sl_alarm) {
        sl_alarm = alarm_new(maincpu_alarm_context, "Simlink", sl_alarm_handler, NULL);
    }
    alarm_set(sl_alarm, maincpu_clk + (CLOCK)simlink_step);
    drive_ext_exact = 1;
    drive_ext_sync_add(parallel_ext_sync);
    log_message(sl_log, "Attached to IEEE bus '%s' as participant %d.",
                simlink_name, sl_slot);
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
    /* Der Reset setzt maincpu_clk auf 6 zurueck (cpu_reset). Die simlink-Zeit
       darf nicht rueckwaerts laufen, sonst wartet das Laufwerk, bis VICE
       die alte Zeit wieder erreicht hat (so lange, wie der PET schon lief).
       Daher die Basis so verschieben, dass es bei done_ps weitergeht, und
       den Abgleich-Alarm auf die neue Taktzeit legen. */
    if (m2ps(maincpu_clk) < done_ps) {
        base_ps += (int64_t)(done_ps - m2ps(maincpu_clk));
        if (sl_alarm) {
            alarm_set(sl_alarm, maincpu_clk + (CLOCK)simlink_step);
        }
    }
    parallel_ext_sync(maincpu_clk);
    ifc_active = 1;
    ifc_end = maincpu_clk + 100;
    publish(m2ps(maincpu_clk));
}

static void sl_alarm_handler(CLOCK offset, void *data)
{
    CLOCK now = maincpu_clk - offset;

    parallel_ext_sync(now);
    if (ifc_active && now >= ifc_end && sl) {
        ifc_active = 0;
        publish(m2ps(now));
    }
    if (sl_alarm && sl) {
        alarm_set(sl_alarm, now + (CLOCK)simlink_step);
    }
}

#else  /* !HAVE_SIMLINK */

void parallel_ext_presync(uint8_t who)
{
}

void parallel_ext_notify(uint8_t who)
{
}


void parallel_ext_reset(void)
{
}

static void sl_stop(void)
{
}

static void sl_start(void)
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
    sl_start();     /* sofort, wenn die Emulation schon laeuft (sonst beim Reset) */
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
