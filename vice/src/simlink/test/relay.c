/*
 * relay - Relais zwischen IEEE-488 (PET) und IEC (serielle Floppy), Testprogramm.
 *
 * Haengt als Geraet am IEEE-Bus (z. B. xpet) und als Rechner am IEC-Bus
 * (z. B. xdrive mit 1541) und uebersetzt auf Protokollebene. Kein eigenes
 * DOS: Befehle, Dateinamen und Daten werden weitergereicht.
 *
 *   relay [-u ieee_unit] [-d iec_unit] [-w s] [-v] ieeebus iecbus
 *
 * Zeitverhalten wie im Konzept (pet-ieee-timeouts.md):
 * - PET sendet (OPEN, Befehle, SAVE): Bytes sofort annehmen, NRFD halten,
 *   solange die IEC-Seite arbeitet.
 * - PET empfaengt (LOAD, GET#, Fehlerkanal): Vor dem Annehmen des Sekundaer-
 *   befehls nach TALK (NRFD gehalten, Datenleitungen schon gueltig) den Kanal
 *   ueber IEC holen; danach sofort liefern. Keine Daten (Datei fehlt): nichts
 *   liefern, der PET meldet nach 65 ms ?FILE NOT FOUND.
 *
 * Vereinfachung dieses Tests: ein Lesekanal wird beim ersten TALK ganz geholt
 * (bis 64 KB), Schreibdaten werden bis UNLISTEN gesammelt.
 *
 * Zeit: eine gemeinsame Uhr; je Bus ein fester Versatz (Startzeit). Waehrend
 * das Relais auf einem Bus arbeitet, steht die Zeit des anderen Busses - dessen
 * Emulator wartet so lange (wie ein Geraet, das NRFD haelt).
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

#define STEP 1000000ULL             /* 1 us */
#define BUFMAX 65536

static volatile sig_atomic_t stop = 0;
static int verbose = 0;
static uint64_t t = 0;              /* gemeinsame Zeit des Relais */
static int started = 0;

typedef struct {
    simlink_t *sl;
    int slot;
    uint64_t off;
    uint8_t pull, data;             /* eigene Leitungen */
    uint8_t in, in_data;            /* zuletzt gelesener Bus */
} sbus_t;

static sbus_t I, C;                 /* IEEE, IEC */

static void on_sig(int s)
{
    (void)s;
    stop = 1;
}

static void quit(int code)
{
    if (I.sl) {
        simlink_detach(I.sl, I.slot);
        simlink_close(I.sl, 0);
    }
    if (C.sl) {
        simlink_detach(C.sl, C.slot);
        simlink_close(C.sl, 0);
    }
    exit(code);
}

static void rd(sbus_t *b)
{
    uint8_t o = b->in, od = b->in_data;
    b->in = simlink_read(b->sl, b->slot, t + b->off, &b->in_data);
    if (started && simlink_others(b->sl, b->slot) == 0) {
        fprintf(stderr, "relay: Gegenseite am %s-Bus beendet\n", b == &I ? "IEEE" : "IEC");
        quit(0);
    }
    if (verbose > 2 && (o != b->in || od != b->in_data)) {
        fprintf(stderr, "relay %10.3f ms %s bus %02x data %02x (eigen %02x)\n", t / 1e9,
                b == &I ? "IEEE" : "IEC ", b->in, b->in_data, b->pull);
    }
    if (stop) {
        quit(1);
    }
}

static void tick(sbus_t *b)
{
    t += STEP;
    rd(b);
}

static void wait_us(sbus_t *b, unsigned n)
{
    if (n) {
        t += (uint64_t)(n - 1) * STEP;
        tick(b);
    }
}

static void set(sbus_t *b, uint8_t pull, uint8_t data)
{
    if (pull != b->pull || data != b->data) {
        b->pull = pull;
        b->data = data;
        simlink_set(b->sl, b->slot, t + b->off, pull, data);
    }
}

/* auf das naechste Umschalten einer Leitung in mask warten (Vorausschau),
   hoechstens timeout_us (0 = ohne Frist); dann lesen */
static void wait_change(sbus_t *b, uint8_t mask, unsigned timeout_us);

/* Am IEEE-Bus nie unbegrenzt warten: in Schritten, und die IEC-Seite jeweils
   mitziehen - sonst stuende dort die Zeit, und das Laufwerk koennte z. B.
   nach CLOSE den Verzeichniseintrag nicht schreiben. */
#define IDLE_STEP_US 1000

static void wait_change_raw(sbus_t *b, uint8_t mask, unsigned timeout_us);

static void wait_change(sbus_t *b, uint8_t mask, unsigned timeout_us)
{
    if (b == &I && (timeout_us == 0 || timeout_us > IDLE_STEP_US)) {
        wait_change_raw(b, mask, IDLE_STEP_US);
        rd(&C);
        return;
    }
    wait_change_raw(b, mask, timeout_us);
}

static void wait_change_raw(sbus_t *b, uint8_t mask, unsigned timeout_us)
{
    uint64_t tc, dl = timeout_us ? t + b->off + (uint64_t)timeout_us * STEP : 0;
    int ch = simlink_wait_change(b->sl, b->slot, t + b->off, mask, STEP, dl, &tc);
    uint64_t tn = (ch ? tc + STEP : dl) - b->off;
    if (verbose > 1) {
        int e = 0; uint64_t pp = simlink_peers_progress(b->sl, b->slot, &e);
        fprintf(stderr, "relay: %s wait mask %02x ch=%d t=%.3f -> %.3f ms peers=%llu err=%d dl=%llu\n", b == &I ? "IEEE" : "IEC", mask, ch, t / 1e9, tn / 1e9, (unsigned long long)pp, e, (unsigned long long)dl);
    }
    if (tn < t) {
        tn = t;
    }
    simlink_wait_end(b->sl, b->slot, tn + b->off);
    t = tn;
    rd(b);
}

/* ======================= IEC: Rechnerseite =========================== */

#define ATN_C  SL_IEC_ATN
#define CLK_C  SL_IEC_CLK
#define DATA_C SL_IEC_DATA

static int iec_unit = 8;
static int iec_err = 0;

static void c_on(uint8_t m)  { set(&C, C.pull | m, 0); }
static void c_off(uint8_t m) { set(&C, (uint8_t)(C.pull & ~m), 0); }

/* warten, bis (bus & m) == want; 0 ok, -1 Frist */
static int c_wait(uint8_t m, uint8_t want, unsigned timeout_us)
{
    uint64_t dl = timeout_us ? t + (uint64_t)timeout_us * STEP : 0;
    tick(&C);
    for (;;) {
        if ((C.in & m) == want) {
            return 0;
        }
        if (dl && t >= dl) {
            return -1;
        }
        wait_change(&C, m, dl ? (unsigned)((dl - t) / STEP) : 0);
    }
}

static int c_send(uint8_t b, int eoi)
{
    int i;
    c_off(CLK_C);
    if (c_wait(DATA_C, 0, 0) < 0) {
        return -1;
    }
    if (eoi) {
        if (c_wait(DATA_C, DATA_C, 1000) < 0 || c_wait(DATA_C, 0, 1000) < 0) {
            return -1;
        }
    }
    wait_us(&C, 20);
    c_on(CLK_C);
    wait_us(&C, 80);
    for (i = 0; i < 8; i++) {
        if (b & (1 << i)) {
            c_off(DATA_C);
        } else {
            c_on(DATA_C);
        }
        wait_us(&C, 30);
        c_off(CLK_C);
        wait_us(&C, 60);
        c_on(CLK_C);
        c_off(DATA_C);
        wait_us(&C, 60);
    }
    return c_wait(DATA_C, DATA_C, 1000);
}

static int c_atn(uint8_t b1, int b2)
{
    c_on(ATN_C | CLK_C);
    c_off(DATA_C);
    if (c_wait(DATA_C, DATA_C, 2000) < 0) {
        c_off(ATN_C | CLK_C);
        return -1;                  /* Geraet nicht vorhanden */
    }
    wait_us(&C, 100);
    if (c_send(b1, 0) < 0 || (b2 >= 0 && c_send((uint8_t)b2, 0) < 0)) {
        c_off(ATN_C | CLK_C);
        return -1;
    }
    return 0;
}

static void c_atn_end(void)
{
    wait_us(&C, 20);
    c_off(ATN_C);
    wait_us(&C, 100);
}

static int c_listen(int sec)
{
    if (c_atn((uint8_t)(0x20 + iec_unit), sec) < 0) {
        return -1;
    }
    c_atn_end();
    return 0;
}

static void c_unlisten(void)
{
    if (c_atn(0x3f, -1) == 0) {
        c_atn_end();
    }
    c_off(CLK_C | DATA_C);
}

static int c_recv(int *eoi)
{
    int i, b = 0;
    *eoi = 0;
    if (c_wait(CLK_C, 0, 0) < 0) {
        return -1;
    }
    c_off(DATA_C);
    if (c_wait(CLK_C, CLK_C, 200) < 0) {
        *eoi = 1;
        c_on(DATA_C);
        wait_us(&C, 60);
        c_off(DATA_C);
        if (c_wait(CLK_C, CLK_C, 64000) < 0) {
            return -1;              /* Sprecher schweigt (z. B. Datei fehlt) */
        }
    }
    for (i = 0; i < 8; i++) {
        if (c_wait(CLK_C, 0, 2000) < 0) {
            return -1;
        }
        if (!(C.in & DATA_C)) {
            b |= 1 << i;
        }
        if (c_wait(CLK_C, CLK_C, 2000) < 0) {
            return -1;
        }
    }
    c_on(DATA_C);
    return b;
}

/* Kanal sa ganz lesen (TALK ... EOI ... UNTALK); liefert Anzahl */
static size_t c_read_channel(int sa, unsigned char *buf, size_t max)
{
    size_t n = 0;
    int eoi = 0, b;

    if (c_atn((uint8_t)(0x40 + iec_unit), 0x60 | sa) < 0) {
        iec_err = 1;
        return 0;
    }
    /* Umschalten: Rechner wird Hoerer */
    c_on(DATA_C);
    c_off(ATN_C);
    wait_us(&C, 20);
    c_off(CLK_C);
    if (c_wait(CLK_C, CLK_C, 2000) < 0) {
        iec_err = 1;
    } else {
        while (!eoi) {
            b = c_recv(&eoi);
            if (b < 0) {
                break;
            }
            if (n < max) {
                buf[n++] = (unsigned char)b;
            }
        }
    }
    if (c_atn(0x5f, -1) == 0) {
        c_atn_end();
    }
    c_off(CLK_C | DATA_C);
    return n;
}

static void c_write(int sec, const unsigned char *s, size_t n)
{
    size_t i;
    if (verbose > 1) {
        fprintf(stderr, "relay: IEC LISTEN %d $%02X, %lu Bytes\n", iec_unit, sec, (unsigned long)n);
    }
    if (c_listen(sec) < 0) {
        iec_err = 1;
        return;
    }
    for (i = 0; i < n; i++) {
        if (c_send(s[i], i + 1 == n) < 0) {
            fprintf(stderr, "relay: IEC Senden abgebrochen bei Byte %lu\n", (unsigned long)i);
            iec_err = 1;
            break;
        }
    }
    c_unlisten();
}

/* ======================= IEEE: Geraeteseite ========================== */

#define LOW(m) (I.in & (m))

static int ieee_unit = 8;

static void i_drive(uint8_t pull, uint8_t data) { set(&I, pull, data); }
static void i_on(uint8_t m)  { i_drive(I.pull | m, I.data); }
static void i_off(uint8_t m) { i_drive((uint8_t)(I.pull & ~m), I.data); }

/* Kanaele */
static unsigned char *cbuf[16];
static size_t clen[16], cpos[16];
static int cvalid[16];
static unsigned char wbuf[BUFMAX];
static size_t wlen = 0;
static unsigned char name[256];
static size_t namelen = 0;
static int opening = -1, talk_pending = 0;

static void log_msg(const char *fmt, const char *a, long v)
{
    fprintf(stderr, "relay %10.3f ms: ", (double)t / 1e9);
    fprintf(stderr, fmt, a, v);
    fputc('\n', stderr);
}

/* Lesekanal bereitstellen (vor dem Annehmen des TALK-Sekundaerbefehls) */
static void prefetch(int sa)
{
    if (cvalid[sa] && sa != 15) {
        return;
    }
    if (!cbuf[sa]) {
        cbuf[sa] = malloc(BUFMAX);
    }
    iec_err = 0;
    clen[sa] = c_read_channel(sa, cbuf[sa], BUFMAX);
    cpos[sa] = 0;
    cvalid[sa] = 1;
    log_msg("Kanal %s geholt: %ld Bytes", sa == 15 ? "15" : (sa == 0 ? "0 (LOAD)" : "n"), (long)clen[sa]);
}

/* ein Byte empfangen; -1 bei ATN-Wechsel */
static int i_listen_byte(int under_atn, int *eoi)
{
    int b;

    if (under_atn && talk_pending) {
        /* Sekundaerbefehl nach TALK: NRFD halten, Datenleitungen sind schon
           gueltig (das PET-Kernal legt das Byte vor dem Warten auf NRFD an) */
        i_drive((uint8_t)(I.pull | SL_IEEE_NDAC | SL_IEEE_NRFD), 0);
        wait_us(&I, 300);
        b = I.in_data;
        if ((b & 0xf0) == 0x60) {
            prefetch(b & 0x0f);
            tick(&I);       /* IEEE-Zeit nachziehen */
        }
        talk_pending = 0;
    }
    i_drive((uint8_t)((I.pull | SL_IEEE_NDAC) & ~(SL_IEEE_NRFD | SL_IEEE_DAV | SL_IEEE_EOI)), 0);
    tick(&I);
    for (;;) {
        if (!!LOW(SL_IEEE_ATN) != under_atn) {
            return -1;
        }
        if (LOW(SL_IEEE_DAV)) {
            break;
        }
        wait_change(&I, SL_IEEE_ATN | SL_IEEE_DAV, 0);
    }
    i_on(SL_IEEE_NRFD);
    b = I.in_data;
    *eoi = LOW(SL_IEEE_EOI) ? 1 : 0;
    i_off(SL_IEEE_NDAC);
    tick(&I);
    while (LOW(SL_IEEE_DAV)) {
        wait_change(&I, SL_IEEE_ATN | SL_IEEE_DAV, 0);
    }
    i_on(SL_IEEE_NDAC);
    return b;
}

/* ein Byte senden; -1 bei ATN */
static int i_talk_byte(int b, int eoi)
{
    tick(&I);
    while (LOW(SL_IEEE_NRFD)) {
        if (LOW(SL_IEEE_ATN)) {
            return -1;
        }
        wait_change(&I, SL_IEEE_ATN | SL_IEEE_NRFD, 0);
    }
    if (LOW(SL_IEEE_ATN)) {
        return -1;
    }
    i_drive((uint8_t)(I.pull | (eoi ? SL_IEEE_EOI : 0)), (uint8_t)b);
    tick(&I);
    i_on(SL_IEEE_DAV);
    tick(&I);
    while (LOW(SL_IEEE_NDAC)) {
        if (LOW(SL_IEEE_ATN)) {
            i_drive((uint8_t)(I.pull & ~(SL_IEEE_DAV | SL_IEEE_EOI)), 0);
            return -1;
        }
        wait_change(&I, SL_IEEE_ATN | SL_IEEE_NDAC, 0);
    }
    i_drive((uint8_t)(I.pull & ~(SL_IEEE_DAV | SL_IEEE_EOI)), 0);
    return 0;
}

static void finish_listen(int sa)
{
    if (opening >= 0) {
        log_msg("OPEN %s (Kanal %ld)", "", opening);
        c_write(0xf0 | opening, name, namelen);
        cvalid[opening] = 0;
        opening = -1;
    } else if (wlen) {
        iec_err = 0;
        c_write(0x60 | sa, wbuf, wlen);
        log_msg("Daten an Kanal %s weitergereicht: %ld Bytes", iec_err ? "(FEHLER)" : "", (long)wlen);
        wlen = 0;
        cvalid[15] = 0;
    }
}

int main(int argc, char **argv)
{
    int c;
    double wait_s = 3.0;
    enum { IDLE, LISTENER, TALKER } role = IDLE;
    int sa = 0;

    while ((c = getopt(argc, argv, "u:d:w:v")) != -1) {
        switch (c) {
        case 'u': ieee_unit = atoi(optarg); break;
        case 'd': iec_unit = atoi(optarg); break;
        case 'w': wait_s = atof(optarg); break;
        case 'v': verbose++; break;
        default: goto usage;
        }
    }
    if (optind + 2 != argc) {
usage:
        fprintf(stderr, "usage: relay [-u ieee_unit] [-d iec_unit] [-w s] [-v] ieeebus iecbus\n");
        return 2;
    }
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    I.sl = simlink_open(argv[optind], SIMLINK_BUS_IEEE, 0, 0xff);
    if (!I.sl) {
        I.sl = simlink_open(argv[optind], SIMLINK_BUS_IEEE, 1, 0xff);
    }
    C.sl = simlink_open(argv[optind + 1], SIMLINK_BUS_IEC, 0, 0xff);
    if (!C.sl) {
        C.sl = simlink_open(argv[optind + 1], SIMLINK_BUS_IEC, 1, 0xff);
    }
    if (!I.sl || !C.sl) {
        perror("simlink_open");
        return 1;
    }
    I.slot = simlink_attach(I.sl, SL_TYPE_TOOL, 0);
    C.slot = simlink_attach(C.sl, SL_TYPE_TOOL, 0);
    if (I.slot < 0 || C.slot < 0) {
        fprintf(stderr, "kein freier Platz\n");
        return 1;
    }
    fprintf(stderr, "relay: IEEE '%s' Geraet %d  <->  IEC '%s' Laufwerk %d\n",
            argv[optind], ieee_unit, argv[optind + 1], iec_unit);
    while ((simlink_others(I.sl, I.slot) == 0 || simlink_others(C.sl, C.slot) == 0) && !stop) {
        usleep(10000);
    }
    started = 1;
    I.off = simlink_start_time(I.sl, I.slot);
    C.off = simlink_start_time(C.sl, C.slot);
    t = 0;              /* VICE uebernimmt den Zustand zur Startzeit */
    /* Laufwerk hochfahren lassen (IEC-Zeit vor). Am IEEE-Bus derweil NRFD
       und NDAC halten: das Relais schaut dort so lange nicht hin, und der PET
       wartet auf NRFD ohne Zeitgrenze. */
    i_drive(SL_IEEE_NRFD | SL_IEEE_NDAC, 0);
    if (verbose > 1) {
        fprintf(stderr, "relay: start I.off=%llu C.off=%llu peersI=%llu\n", (unsigned long long)I.off,
                (unsigned long long)C.off, (unsigned long long)simlink_peers_progress(I.sl, I.slot, NULL));
    }
    wait_us(&C, (unsigned)(wait_s * 1e6));
    if (verbose > 1) {
        fprintf(stderr, "relay: nach Anlauf peersI=%llu others=%d\n",
                (unsigned long long)simlink_peers_progress(I.sl, I.slot, NULL), simlink_others(I.sl, I.slot));
    }

    for (;;) {
        int eoi = 0, b;
        tick(&I);
        if (LOW(SL_IEEE_ATN)) {
            b = i_listen_byte(1, &eoi);
            if (b < 0) {
                continue;
            }
            if (verbose) {
                fprintf(stderr, "relay: ATN $%02X\n", b);
            }
            if (b == 0x20 + ieee_unit) {
                role = LISTENER;
            } else if (b == 0x3f) {
                if (role == LISTENER) {
                    finish_listen(sa);
                    role = IDLE;
                }
            } else if (b == 0x40 + ieee_unit) {
                role = TALKER;
                talk_pending = 1;
            } else if (b == 0x5f) {
                if (role == TALKER) {
                    role = IDLE;
                }
            } else if ((b & 0xe0) == 0x40 && role == TALKER) {
                role = IDLE;        /* anderes Geraet spricht */
            } else if (role != IDLE && (b & 0xf0) == 0x60) {
                sa = b & 0x0f;
                wlen = 0;
            } else if (role != IDLE && (b & 0xf0) == 0xf0) {
                sa = b & 0x0f;
                opening = sa;
                namelen = 0;
            } else if (role != IDLE && (b & 0xf0) == 0xe0) {
                sa = b & 0x0f;
                log_msg("CLOSE%s %ld", "", sa);
                c_listen(0xe0 | sa);
                c_unlisten();
                cvalid[sa] = 0;
                cvalid[15] = 0;
            }
            continue;
        }
        if (role == LISTENER) {
            b = i_listen_byte(0, &eoi);
            if (b < 0) {
                continue;
            }
            if (opening >= 0) {
                if (namelen < sizeof(name)) {
                    name[namelen++] = (unsigned char)b;
                }
            } else if (wlen < BUFMAX) {
                wbuf[wlen++] = (unsigned char)b;
            }
            continue;
        }
        if (role == TALKER) {
            i_drive((uint8_t)(I.pull & ~(SL_IEEE_NDAC | SL_IEEE_NRFD)), 0);
            if (!cvalid[sa] || cpos[sa] >= clen[sa]) {
                /* nichts zu liefern: schweigen, der PET meldet den Fehler */
                if (!LOW(SL_IEEE_ATN)) {
                    wait_change(&I, SL_IEEE_ATN, 0);
                }
                continue;
            }
            if (i_talk_byte(cbuf[sa][cpos[sa]], cpos[sa] + 1 == clen[sa]) == 0) {
                cpos[sa]++;
                if (cpos[sa] >= clen[sa]) {
                    role = IDLE;
                    if (sa == 15) {
                        cvalid[15] = 0;
                    }
                }
            }
            continue;
        }
        i_drive(0, 0);
        if (!LOW(SL_IEEE_ATN)) {
            wait_change(&I, SL_IEEE_ATN, 0);
        }
    }
}
