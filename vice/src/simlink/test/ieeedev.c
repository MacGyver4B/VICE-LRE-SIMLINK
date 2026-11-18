/*
 * ieeedev - einfaches IEEE-488-Testgeraet am simlink-Bus.
 *
 * Verhaelt sich wie ein CBM-Laufwerk auf Protokollebene (ohne DOS-Emulation):
 * nimmt Dateien per OPEN/Daten/CLOSE entgegen (SAVE, PRINT#) und liefert sie
 * wieder aus (LOAD, GET#). Dateien liegen im Arbeitsverzeichnis.
 *
 *   ieeedev [-u unit] [-s schritt_ns] [-v] busname
 *
 * Laeuft mit eigener Zeitbasis (Standard 1 us pro Schritt) und wartet beim
 * Lesen auf die anderen Teilnehmer - genau wie ein Emulator.
 */
#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include <ctype.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "simlink.h"

static simlink_t *sl;
static int slot;
static uint64_t t_ps = 0;
static uint64_t step_ps = 1000000;   /* 1 us */
static int verbose = 0;
static int unit = 8;
/* Testoptionen fuer das Zeitverhalten des PET */
static unsigned hold_us = 0;        /* -H: NRFD vor jedem Byte unter ATN so lange halten */
static unsigned first_us = 0;       /* -D: als Sprecher vor dem ersten Byte warten */
static unsigned gap_us = 0;         /* -G: als Sprecher vor jedem weiteren Byte warten */
static unsigned hold_data_us = 0;   /* -N: NRFD vor jedem Datenbyte (ohne ATN) halten */
static unsigned ack_us = 0;         /* -A: NDAC nach DAV so lange halten (Quittung verzoegern) */
static int talk_started = 0;
static volatile sig_atomic_t stop = 0;

static uint8_t my_pull = 0, my_data = 0;
static uint8_t bus = 0, bus_data = 0;

static void on_sig(int s)
{
    (void)s;
    stop = 1;
}

/* einen Schritt weiter: eigene Leitungen setzen, Bus lesen */
static void tick(void)
{
    t_ps += step_ps;
    {
        static uint8_t ob = 0xff, od = 0;
        bus = simlink_read(sl, slot, t_ps, &bus_data);
        if (verbose > 1 && (bus != ob || bus_data != od)) {
            fprintf(stderr, "%12.3f us  bus %02X data %02X  (eigen %02X)\n",
                    t_ps / 1e6, bus, bus_data, my_pull);
            ob = bus;
            od = bus_data;
        }
    }
    if (stop) {
        simlink_detach(sl, slot);
        simlink_close(sl, 0);
        exit(1);
    }
}

static void drive(uint8_t pull, uint8_t data)
{
    if (pull != my_pull || data != my_data) {
        my_pull = pull;
        my_data = data;
        simlink_set(sl, slot, t_ps, pull, data);
    }
}

static void pull_on(uint8_t m)
{
    drive(my_pull | m, my_data);
}

static void pull_off(uint8_t m)
{
    drive(my_pull & (uint8_t)~m, my_data);
}

/* statt jede Schrittzeit nachzusehen: auf das naechste Umschalten einer der
   Leitungen in mask warten (Vorausschau: wir reagieren fruehestens einen
   Schritt danach), dann lesen */
static void tick_change(uint8_t mask)
{
    uint64_t tc;
    simlink_wait_change(sl, slot, t_ps, mask, step_ps, 0, &tc);
    simlink_wait_end(sl, slot, tc + step_ps);
    t_ps = tc;
    tick();
}

#define LOW(m)  (bus & (m))

/* ---- Dateien ---------------------------------------------------------- */

static char names[16][64];
static FILE *files[16];
static int writing[16];
static unsigned char rbuf[16];
static int rhave[16];

static void sanitize(const char *in, int n, char *out)
{
    int i, j = 0;
    for (i = 0; i < n && j < 60; i++) {
        unsigned char c = (unsigned char)in[i];
        if (c == ',' ) {
            break;          /* ",S,W" usw. abschneiden */
        }
        if (c == ':' ) {
            j = 0;          /* "0:NAME" */
            continue;
        }
        out[j++] = (isalnum(c) || c == '.' || c == '-' || c == '_') ? (char)tolower(c) : '_';
    }
    if (j == 0) {
        out[j++] = '_';
    }
    out[j] = 0;
}

static void do_open(int sa, const char *name, int n)
{
    char fn[64];
    int w;
    sanitize(name, n, fn);
    w = (sa == 1) || (strstr(name, ",W") != NULL) || (strstr(name, ",w") != NULL);
    if (files[sa]) {
        fclose(files[sa]);
    }
    strcpy(names[sa], fn);
    files[sa] = fopen(fn, w ? "wb" : "rb");
    writing[sa] = w;
    rhave[sa] = 0;
    fprintf(stderr, "ieeedev: OPEN %d \"%.*s\" -> %s (%s)%s\n", sa, n, name, fn,
            w ? "schreiben" : "lesen", files[sa] ? "" : " FEHLT");
}

static void do_close(int sa)
{
    if (files[sa]) {
        long sz = ftell(files[sa]);
        fclose(files[sa]);
        files[sa] = NULL;
        fprintf(stderr, "ieeedev: CLOSE %d (%s, %ld Bytes)\n", sa, names[sa], sz);
    }
}

/* ---- Handshake -------------------------------------------------------- */

/* ein Byte empfangen; liefert -1 wenn ATN-Wechsel abbricht */
static void pause_us(unsigned n)
{
    if (n) {
        t_ps += (uint64_t)(n - 1) * step_ps;
        tick();
    }
}

static int listen_byte(int under_atn, int *eoi)
{
    int b;
    if (under_atn ? hold_us : hold_data_us) {
        /* nicht bereit: NRFD und NDAC halten (der PET wartet ohne Frist) */
        drive((uint8_t)(my_pull | SL_IEEE_NDAC | SL_IEEE_NRFD), 0);
        pause_us(under_atn ? hold_us : hold_data_us);
    }
    /* bereit: NRFD frei, NDAC gezogen */
    drive((uint8_t)((my_pull | SL_IEEE_NDAC) & ~SL_IEEE_NRFD & ~SL_IEEE_DAV & ~SL_IEEE_EOI), 0);
    for (int first = 1;; first = 0) {
        if (first) {
            tick();
        } else {
            tick_change(SL_IEEE_ATN | SL_IEEE_DAV);
        }
        if (!!LOW(SL_IEEE_ATN) != under_atn) {
            return -1;
        }
        if (LOW(SL_IEEE_DAV)) {
            break;
        }
    }
    pull_on(SL_IEEE_NRFD);
    if (!under_atn && ack_us) {
        pause_us(ack_us);          /* Quittung (NDAC frei) verzoegern */
    }
    b = bus_data;
    *eoi = LOW(SL_IEEE_EOI) ? 1 : 0;
    pull_off(SL_IEEE_NDAC);
    for (int first = 1;; first = 0) {
        if (first) {
            tick();
        } else {
            tick_change(SL_IEEE_ATN | SL_IEEE_DAV);
        }
        if (!LOW(SL_IEEE_DAV)) {
            break;
        }
        if (!!LOW(SL_IEEE_ATN) != under_atn && !under_atn) {
            break;
        }
    }
    pull_on(SL_IEEE_NDAC);
    return b;
}

/* ein Byte senden; liefert -1 bei ATN */
static int talk_byte(int b, int eoi)
{
    pause_us(talk_started ? gap_us : first_us);
    talk_started = 1;
    /* warten, bis alle Hoerer bereit (NRFD frei) */
    for (int first = 1;; first = 0) {
        if (first) {
            tick();
        } else {
            tick_change(SL_IEEE_ATN | SL_IEEE_NRFD);
        }
        if (LOW(SL_IEEE_ATN)) {
            return -1;
        }
        if (!LOW(SL_IEEE_NRFD)) {
            break;
        }
    }
    drive((uint8_t)(my_pull | (eoi ? SL_IEEE_EOI : 0)), (uint8_t)b);
    tick();
    pull_on(SL_IEEE_DAV);
    for (int first = 1;; first = 0) {
        if (first) {
            tick();
        } else {
            tick_change(SL_IEEE_ATN | SL_IEEE_NDAC);
        }
        if (LOW(SL_IEEE_ATN)) {
            drive((uint8_t)(my_pull & ~(SL_IEEE_DAV | SL_IEEE_EOI)), 0);
            return -1;
        }
        if (!LOW(SL_IEEE_NDAC)) {
            break;
        }
    }
    drive((uint8_t)(my_pull & ~(SL_IEEE_DAV | SL_IEEE_EOI)), 0);
    return 0;
}

int main(int argc, char **argv)
{
    int c;
    enum { IDLE, LISTENER, TALKER } role = IDLE;
    int sa = 0, opening = -1;
    char namebuf[256];
    int namelen = 0;
    unsigned long nbytes = 0;

    while ((c = getopt(argc, argv, "u:s:vH:D:G:N:A:")) != -1) {
        switch (c) {
        case 'u': unit = atoi(optarg); break;
        case 's': step_ps = (uint64_t)atoll(optarg) * 1000; break;
        case 'v': verbose++; break;
        case 'H': hold_us = (unsigned)atoi(optarg); break;
        case 'D': first_us = (unsigned)atoi(optarg); break;
        case 'G': gap_us = (unsigned)atoi(optarg); break;
        case 'N': hold_data_us = (unsigned)atoi(optarg); break;
        case 'A': ack_us = (unsigned)atoi(optarg); break;
        default:
            fprintf(stderr, "usage: ieeedev [-u unit] [-s step_ns] [-H us] [-D us] [-G us] [-v] bus\n");
            return 2;
        }
    }
    if (optind >= argc) {
        fprintf(stderr, "usage: ieeedev [-u unit] [-s step_ns] [-H us] [-D us] [-G us] [-v] bus\n");
        return 2;
    }
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    sl = simlink_open(argv[optind], SIMLINK_BUS_IEEE, 0, 0xff);
    if (!sl) {
        sl = simlink_open(argv[optind], SIMLINK_BUS_IEEE, 1, 0xff);
    }
    if (!sl) {
        perror("simlink_open");
        return 1;
    }
    slot = simlink_attach(sl, SL_TYPE_TOOL, 0);
    if (slot < 0) {
        fprintf(stderr, "kein freier Platz\n");
        return 1;
    }
    fprintf(stderr, "ieeedev: Geraet %d an Bus '%s', Teilnehmer %d\n", unit, argv[optind], slot);

    /* erst loslaufen, wenn ein Rechner am Bus haengt (sonst liefe die
       eigene Zeit ungebremst davon) */
    while (simlink_others(sl, slot) == 0 && !stop) {
        usleep(10000);
    }
    t_ps = 0;
    for (;;) {
        int eoi = 0, b;
        tick();
        if (LOW(SL_IEEE_ATN)) {
            /* Befehlsphase: Bytes unter ATN empfangen */
            b = listen_byte(1, &eoi);
            if (b < 0) {
                continue;
            }
            if (verbose) {
                fprintf(stderr, "ieeedev: ATN $%02X\n", b);
            }
            if (b == 0x20 + unit) {
                role = LISTENER;
            } else if (b == 0x3f) {
                if (role == LISTENER && opening >= 0) {
                    do_open(opening, namebuf, namelen);
                    opening = -1;
                }
                if (role == LISTENER) {
                    role = IDLE;
                }
            } else if (b == 0x40 + unit) {
                role = TALKER;
            } else if (b == 0x5f) {
                if (role == TALKER) {
                    role = IDLE;
                }
            } else if (b >= 0x20 && b < 0x5f) {
                /* anderes Geraet adressiert */
                if ((b & 0xe0) == 0x20 || (b & 0xe0) == 0x40) {
                    if (role == TALKER && (b & 0xe0) == 0x40) {
                        role = IDLE;
                    }
                }
            } else if (role != IDLE && (b & 0xf0) == 0x60) {
                sa = b & 0x0f;
            } else if (role != IDLE && (b & 0xf0) == 0xf0) {
                sa = b & 0x0f;
                opening = sa;
                namelen = 0;
            } else if (role != IDLE && (b & 0xf0) == 0xe0) {
                do_close(b & 0x0f);
                role = (role == LISTENER) ? LISTENER : role;
            }
            continue;
        }
        if (role == LISTENER) {
            b = listen_byte(0, &eoi);
            if (b < 0) {
                continue;
            }
            if (opening >= 0) {
                if (namelen < (int)sizeof(namebuf) - 1) {
                    namebuf[namelen++] = (char)b;
                }
            } else if (files[sa] && writing[sa]) {
                fputc(b, files[sa]);
                nbytes++;
            }
            continue;
        }
        if (role != TALKER) {
            talk_started = 0;
        }
        if (role == TALKER) {
            /* Umschaltung: Rechner gibt ATN frei und wird Hoerer */
            drive((uint8_t)(my_pull & ~(SL_IEEE_NDAC | SL_IEEE_NRFD)), 0);
            if (!files[sa] || writing[sa]) {
                /* nichts zu liefern: wie Laufwerk "file not found" -
                   ein Byte mit EOI (der Rechner prueft den Status) */
                if (talk_byte(0x0d, 1) == 0) {
                    role = IDLE;
                }
                continue;
            }
            if (!rhave[sa]) {
                int ch = fgetc(files[sa]);
                if (ch == EOF) {
                    role = IDLE;
                    continue;
                }
                rbuf[sa] = (unsigned char)ch;
                rhave[sa] = 1;
            }
            {
                int nx = fgetc(files[sa]);
                int last = (nx == EOF);
                if (talk_byte(rbuf[sa], last) < 0) {
                    if (nx != EOF) {
                        ungetc(nx, files[sa]);
                    }
                    continue;       /* ATN: Byte bleibt gepuffert */
                }
                nbytes++;
                if (last) {
                    rhave[sa] = 0;
                    role = IDLE;
                    fprintf(stderr, "ieeedev: Datei %s ausgeliefert (gesamt %lu Bytes)\n",
                            names[sa], nbytes);
                } else {
                    rbuf[sa] = (unsigned char)nx;
                }
            }
            continue;
        }
        /* unbeteiligt: alle Leitungen frei, bis ATN kommt */
        drive(0, 0);
        if (!LOW(SL_IEEE_ATN)) {
            tick_change(SL_IEEE_ATN);
        }
    }
}
