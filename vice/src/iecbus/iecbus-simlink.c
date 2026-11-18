/*
 * iecbus-simlink.c - Externer Rechner am IEC-Bus ueber simlink.
 *
 * Ein anderer Emulator (z. B. picoem mit der petSD2-Firmware) uebernimmt die
 * Rechnerseite des IEC-Busses; die Laufwerke von VICE haengen an ihm. Der
 * eigentliche Rechner der Maschine ist dann vom Bus getrennt (xdrive).
 *
 * Zeitabgleich (konservativ, ohne Vorlauf): Laufwerke werden nur bis zu der
 * Zeit nachgezogen, bis zu der der Stand aller anderen Teilnehmer bekannt ist.
 * Jede Aenderung des externen Rechners wird genau zu ihrer Zeit eingespeist
 * (Laufwerke bis dahin nachziehen, dann Leitungen setzen). Die Ausgaben der
 * Laufwerke werden mit dem Laufwerkstakt gestempelt.
 *
 * Ressourcen: "IECSimlink" (Busname, leer = aus), "IECSimlinkStep" (Abgleich
 * spaetestens alle n Takte des Rechners).
 *
 * xdrive: heisst das Programm "xdrive" (eigenes Programm aus denselben
 * Bibliotheken wie x64sc, vgl. den Link x64 -> x64sc), gelten eigene
 * Voreinstellungen: Bus "xdrive" verbunden, eigene Einstellungsdatei.
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
#include <stdint.h>
#include <string.h>

#include "alarm.h"
#include "archdep.h"
#include "cmdline.h"
#include "drive.h"
#include "drivetypes.h"
#include "iecbus.h"
#include "lib.h"
#include "log.h"
#include "machine.h"
#include "maincpu.h"
#include "resources.h"
#include "types.h"
#include "util.h"

#include "simlink.h"

int iecbus_ext_active = 0;
const iecbus_ext_machine_t *iecbus_ext_machine = NULL;

static char *simlink_name = NULL;
static int simlink_step = 64;

static log_t sl_log = LOG_DEFAULT;
static simlink_t *sl = NULL;
static int sl_slot = -1;
static alarm_t *sl_alarm = NULL;

static uint64_t hz = 1000000;       /* Takt des Rechners */
static int64_t base_ps = 0;         /* simlink-Zeit = base + Takt * 1e12 / hz */
static uint64_t done_ps = 0;
static int start_pending = 0;   /* Startzustand der anderen noch uebernehmen */        /* bis hierher abgeglichen (= Fortschritt) */
static int in_sync = 0;
static uint8_t pub_pull = 0;        /* zuletzt veroeffentlicht */
static uint64_t pub_ps = 0;
static int ext_valid = 0;
static uint8_t ext_pull = 0;        /* zuletzt eingespeist */

/* Zusammenhang Haupt-/Laufwerkstakt je Laufwerk (vor jedem Nachziehen) */
static drive_sync_snap_t snap[NUM_DISK_UNITS];

#define PS_PER_S 1000000000000ULL

static uint64_t m2ps(CLOCK c)
{
    unsigned __int128 x = (unsigned __int128)c * PS_PER_S / hz;
    return (uint64_t)((int64_t)(uint64_t)x + base_ps);
}

/* letzter Takt des Rechners mit Zeit <= t */
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

/* Leitungen der Laufwerke (und IEC-Geraete) ohne den externen Rechner */
static uint8_t vice_pull(void)
{
    uint8_t lv = 0xff, p = 0;
    unsigned int unit;

    for (unit = 4; unit < 8 + NUM_DISK_UNITS; unit++) {
        lv &= iecbus.drv_bus[unit];
    }
    if (!(lv & 0x80)) {
        p |= SL_IEC_DATA;
    }
    if (!(lv & 0x40)) {
        p |= SL_IEC_CLK;
    }
    return p;
}

static void publish(uint64_t t)
{
    uint8_t p = vice_pull();

    if (p == pub_pull) {
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
    simlink_set(sl, sl_slot, t, p, 0);
}

/* Aufruf aus iec_drive_write(): Laufwerk dnr hat seine Ausgaenge geaendert */
void iecbus_ext_drive_changed(unsigned int dnr)
{
    uint64_t t = done_ps;

    if (!sl) {
        return;
    }
    if (in_sync && dnr < NUM_DISK_UNITS) {
        /* Laufwerkstakt -> Haupttakt, unabhaengig von der Schrittweite */
        t = m2ps(drive_sync_main_clock(&snap[dnr], *diskunit_context[dnr]->clk_ptr));
    }
    publish(t);
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

static uint8_t to_ext(uint8_t l)
{
    uint8_t p = 0;
    if (l & SL_IEC_ATN) {
        p |= IECBUS_EXT_ATN;
    }
    if (l & SL_IEC_CLK) {
        p |= IECBUS_EXT_CLK;
    }
    if (l & SL_IEC_DATA) {
        p |= IECBUS_EXT_DATA;
    }
    return p;
}

/* Stand des externen Rechners zum Zeitpunkt t (Takt c) einspeisen */
static void apply_ext(uint64_t t, CLOCK c)
{
    uint8_t l = simlink_peek_others(sl, sl_slot, t, NULL) & (SL_IEC_ATN | SL_IEC_CLK | SL_IEC_DATA);

    if (ext_valid && l == ext_pull) {
        return;
    }
    ext_valid = 1;
    ext_pull = l;
    iecbus_ext_machine->host_write(to_ext(l), c);
    /* ATN-Quittung der Laufwerke reagiert ohne Verzoegerung */
    publish(t);
}

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

/* Haken in drive.c: vor jedem Nachziehen der Laufwerke bis clk */
static void ext_sync(CLOCK clk)
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
        /* Fortschritt m heisst: alle Eintraege vor m sind endgueltig; ein
           Eintrag genau bei m kann noch kommen. Daher nur bis m - 1. */
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

        /* Aenderungen des externen Rechners der Reihe nach einspeisen */
        while (simlink_next_change(sl, sl_slot, done_ps, upto, &tc)) {
            CLOCK c = ps2m_floor(tc);
            run_drives(c);
            apply_ext(tc, c);
            done_ps = tc;
            /* alles bis einschliesslich tc ist veroeffentlicht */
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

static void sl_alarm_handler(CLOCK offset, void *data)
{
    CLOCK now = maincpu_clk - offset;

    ext_sync(now);
    if (sl_alarm && sl) {
        alarm_set(sl_alarm, now + (CLOCK)simlink_step);
    }
}

static void sl_stop(void)
{
    if (sl) {
        simlink_detach(sl, sl_slot);
        simlink_close(sl, 0);
        sl = NULL;
        sl_slot = -1;
        log_message(sl_log, "Detached from IEC bus.");
        /* Leitungen freigeben, eigenen Rechner wieder anschliessen */
        if (iecbus_ext_machine) {
            iecbus_ext_machine->host_write(0, maincpu_clk);
            iecbus_ext_machine->disconnect(0);
        }
    }
    if (sl_alarm) {
        alarm_unset(sl_alarm);
    }
    drive_ext_sync_remove(ext_sync);
    drive_ext_exact = 0;
    drive_ext_iec_changed = NULL;
    iecbus_ext_active = 0;
}

static void sl_start(void)
{
    long cps;

    if (sl || simlink_name == NULL || *simlink_name == 0) {
        return;
    }
    if (maincpu_alarm_context == NULL || iecbus_ext_machine == NULL) {
        return;
    }
    if (sl_log == LOG_DEFAULT) {
        sl_log = log_open("Simlink");
    }
    sl = simlink_open(simlink_name, SIMLINK_BUS_IEC, 0, 0xff);
    if (!sl) {
        sl = simlink_open(simlink_name, SIMLINK_BUS_IEC, 1, 0xff);
    }
    if (!sl) {
        log_error(sl_log, "Cannot open simlink IEC bus '%s'.", simlink_name);
        return;
    }
    /* Laufwerke auf den aktuellen Stand bringen, bevor die Zeit zaehlt */
    drive_cpu_execute_all(maincpu_clk);

    sl_slot = simlink_attach(sl, SL_TYPE_VICE, 1);
    if (sl_slot < 0) {
        log_error(sl_log, "No free participant slot on IEC bus '%s'.", simlink_name);
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
    ext_valid = 0;

    /* Rechner der Maschine vom Bus trennen; Bus zunaechst frei */
    iecbus_ext_machine->disconnect(1);
    iecbus_ext_active = 1;
    iecbus_ext_machine->host_write(0, maincpu_clk);
    publish(done_ps);
    simlink_progress(sl, sl_slot, done_ps);
    /* Zustand der anderen zur Startzeit beim ersten Abgleich uebernehmen */
    start_pending = 1;

    drive_ext_iec_changed = iecbus_ext_drive_changed;
    drive_ext_exact = 1;
    drive_ext_sync_add(ext_sync);
    if (!sl_alarm) {
        sl_alarm = alarm_new(maincpu_alarm_context, "IECSimlink", sl_alarm_handler, NULL);
    }
    alarm_set(sl_alarm, maincpu_clk + (CLOCK)simlink_step);
    log_message(sl_log, "Attached to IEC bus '%s' as participant %d (machine disconnected from IEC).",
                simlink_name, sl_slot);
}

/* Reset der Maschine: Verbindung (wieder) herstellen */
void iecbus_ext_reset(void)
{
    if (!sl) {
        sl_start();
    } else {
        /* Rechnerseite bleibt beim externen Teilnehmer */
        ext_valid = 0;
    }
}

/* ------------------------------------------------------------------------- */

static int set_simlink_name(const char *val, void *param)
{
    if (util_string_set(&simlink_name, val ? val : "")) {
        return 0;
    }
    sl_stop();
    sl_start();
    return 0;
}

static int set_simlink_step(int val, void *param)
{
    simlink_step = val < 1 ? 1 : val;
    return 0;
}

static const resource_string_t resources_string[] = {
    { "IECSimlink", "", RES_EVENT_NO, NULL,
      &simlink_name, set_simlink_name, NULL },
    RESOURCE_STRING_LIST_END
};

/* xdrive: dasselbe mit verbundenem Bus als Voreinstellung */
static const resource_string_t resources_string_xdrive[] = {
    { "IECSimlink", "xdrive", RES_EVENT_NO, NULL,
      &simlink_name, set_simlink_name, NULL },
    RESOURCE_STRING_LIST_END
};

#ifdef _WIN32
#define XDRIVE_CONFIG_NAME "xdrive.ini"
#else
#define XDRIVE_CONFIG_NAME "xdriverc"
#endif

/* laeuft dieses Programm als xdrive? */
int iecbus_simlink_is_xdrive(void)
{
    const char *n = archdep_program_name();
    return n != NULL && strcmp(n, "xdrive") == 0;
}

static const resource_int_t resources_int[] = {
    { "IECSimlinkStep", 64, RES_EVENT_NO, NULL,
      &simlink_step, set_simlink_step, NULL },
    RESOURCE_INT_LIST_END
};

int iecbus_simlink_resources_init(void)
{
    if (iecbus_simlink_is_xdrive()) {
        /* eigene Einstellungsdatei, sofern nicht per -config vorgegeben */
        if (vice_config_file == NULL) {
            vice_config_file = util_join_paths(archdep_user_config_path(),
                                               XDRIVE_CONFIG_NAME, NULL);
        }
        if (resources_register_string(resources_string_xdrive) < 0) {
            return -1;
        }
        return resources_register_int(resources_int);
    }
    if (resources_register_string(resources_string) < 0) {
        return -1;
    }
    return resources_register_int(resources_int);
}

void iecbus_simlink_resources_shutdown(void)
{
    /* beim Beenden nur abmelden, die Emulation nicht mehr anfassen */
    if (sl) {
        simlink_detach(sl, sl_slot);
        simlink_close(sl, 0);
        sl = NULL;
    }
    drive_ext_iec_changed = NULL;
    iecbus_ext_active = 0;
    lib_free(simlink_name);
    simlink_name = NULL;
}

static const cmdline_option_t cmdline_options[] = {
    { "-iecsimlink", SET_RESOURCE, CMDLINE_ATTRIB_NEED_ARGS,
      NULL, NULL, "IECSimlink", NULL,
      "<name>", "Let an external computer drive the IEC bus via simlink shared memory; the emulated computer is disconnected from IEC (empty: off)" },
    { "-iecsimlinkstep", SET_RESOURCE, CMDLINE_ATTRIB_NEED_ARGS,
      NULL, NULL, "IECSimlinkStep", NULL,
      "<cycles>", "Synchronise with the simlink IEC bus at least every <cycles> CPU cycles" },
    CMDLINE_LIST_END
};

int iecbus_simlink_cmdline_options_init(void)
{
    return cmdline_register_options(cmdline_options);
}
