/*
 * iechost - einfacher Rechner am simlink-IEC-Bus (Testprogramm).
 *
 * Spricht das serielle Commodore-Protokoll auf Leitungsebene (ATN, CLK, DATA)
 * wie das KERNAL eines C64/VC20, mit eigener Zeitbasis (1 us pro Schritt).
 *
 *   iechost [-u unit] [-w sekunden] [-v] bus befehl...
 *
 *   load  NAME DATEI     Datei NAME vom Laufwerk lesen und in DATEI speichern
 *   save  DATEI NAME     DATEI auf das Laufwerk schreiben (als PRG NAME)
 *   status               Fehlerkanal lesen
 *   cmd   TEXT           Befehl auf Kanal 15 senden (z. B. "N:TEST,01")
 *   dir   DATEI          Verzeichnis ("$") lesen und als PRG speichern
 *
 * Mehrere Befehle hintereinander sind moeglich. -w: Wartezeit (emulierte
 * Sekunden) nach dem Start, damit das Laufwerk hochfahren kann (Standard 2).
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
static const uint64_t step_ps = 1000000;   /* 1 us */
static int verbose = 0;
static int unit = 8;
static volatile sig_atomic_t stop = 0;

static uint8_t my_pull = 0;
static uint8_t bus = 0;

#define ATN  SL_IEC_ATN
#define CLK  SL_IEC_CLK
#define DATA SL_IEC_DATA

static void on_sig(int s)
{
    (void)s;
    stop = 1;
}

static void tick(void)
{
    static uint8_t ob = 0xff;
    t_ps += step_ps;
    bus = simlink_read(sl, slot, t_ps, NULL);
    if (verbose > 1 && bus != ob) {
        fprintf(stderr, "%12.3f us  bus %s%s%s  (eigen %s%s%s)\n", t_ps / 1e6,
                (bus & ATN) ? "ATN " : "    ", (bus & CLK) ? "CLK " : "    ",
                (bus & DATA) ? "DATA" : "    ", (my_pull & ATN) ? "ATN " : "    ",
                (my_pull & CLK) ? "CLK " : "    ", (my_pull & DATA) ? "DATA" : "    ");
        ob = bus;
    }
    if (stop) {
        simlink_detach(sl, slot);
        simlink_close(sl, 0);
        exit(1);
    }
}

/* n us nichts tun: Fortschritt gleich um n us vorsetzen (Vorausschau),
   erst am Ende den Bus lesen */
static void wait_us(unsigned n)
{
    if (n == 0) {
        return;
    }
    t_ps += (uint64_t)(n - 1) * step_ps;
    tick();
}

static void lines(uint8_t pull)
{
    if (pull != my_pull) {
        my_pull = pull;
        simlink_set(sl, slot, t_ps, pull, 0);
    }
}

static void pull_on(uint8_t m)
{
    lines(my_pull | m);
}

static void pull_off(uint8_t m)
{
    lines((uint8_t)(my_pull & ~m));
}

/* wartet, bis (bus & m) == want; 0 = ok, -1 = Zeitueberschreitung.
   Statt jede Mikrosekunde nachzusehen, meldet der Rechner dem Bus: "ich
   reagiere fruehestens eine Mikrosekunde nach der naechsten Aenderung" -
   die anderen koennen bis dahin ohne Warten weiterrechnen. */
static int wait_for(uint8_t m, uint8_t want, unsigned timeout_us)
{
    uint64_t deadline = timeout_us ? t_ps + (uint64_t)timeout_us * step_ps : 0;

    tick();
    for (;;) {
        uint64_t tc, t_new;
        int changed;

        if ((bus & m) == want) {
            return 0;
        }
        if (deadline && t_ps >= deadline) {
            return -1;
        }
        changed = simlink_wait_change(sl, slot, t_ps, m, step_ps, deadline, &tc);
        t_new = changed ? tc + step_ps : deadline;
        if (t_new < t_ps) {
            t_new = t_ps;
        }
        simlink_wait_end(sl, slot, t_new);
        t_ps = t_new - step_ps;
        tick();
    }
}

static void fail(const char *what)
{
    fprintf(stderr, "iechost: Fehler: %s (bei %.3f ms)\n", what, t_ps / 1e9);
    simlink_detach(sl, slot);
    simlink_close(sl, 0);
    exit(1);
}

/* ---- Senden (Rechner ist Sprecher) ------------------------------------- */

static void send_byte(uint8_t b, int eoi)
{
    int i;

    /* bereit zum Senden: CLK frei */
    pull_off(CLK);
    /* warten, bis alle Hoerer bereit sind (DATA frei) */
    if (wait_for(DATA, 0, 0) < 0) {
        fail("Hoerer nicht bereit");
    }
    if (eoi) {
        /* EOI: nicht reagieren; Hoerer quittiert mit DATA-Puls */
        if (wait_for(DATA, DATA, 1000) < 0) {
            fail("keine EOI-Quittung");
        }
        if (wait_for(DATA, 0, 1000) < 0) {
            fail("EOI-Quittung haengt");
        }
    }
    /* Zeiten grosszuegig: die 1541 fragt CLK in Schleifen von rund 50 us ab
       (mit ATN-Pruefung); kuerzere Phasen verpasst sie */
    wait_us(20);
    pull_on(CLK);
    wait_us(80);
    for (i = 0; i < 8; i++) {
        if (b & (1 << i)) {
            pull_off(DATA);         /* 1 = DATA frei */
        } else {
            pull_on(DATA);
        }
        wait_us(30);
        pull_off(CLK);              /* Bit gueltig */
        wait_us(60);
        pull_on(CLK);
        pull_off(DATA);
        wait_us(60);
    }
    /* Quittung: Hoerer zieht DATA innerhalb 1 ms */
    if (wait_for(DATA, DATA, 1000) < 0) {
        fail("Byte nicht quittiert");
    }
}

static void atn_begin(void)
{
    pull_on(ATN | CLK);
    pull_off(DATA);
    /* Geraete melden sich mit DATA */
    if (wait_for(DATA, DATA, 2000) < 0) {
        fail("Geraet nicht vorhanden");
    }
    wait_us(100);
}

static void atn_end(void)
{
    wait_us(20);
    pull_off(ATN);
    wait_us(100);
}

/* ---- Empfangen (Rechner ist Hoerer) ------------------------------------ */

static int recv_byte(int *eoi)
{
    int i, b = 0;

    *eoi = 0;
    /* Sprecher bereit: CLK frei */
    if (wait_for(CLK, 0, 0) < 0) {
        fail("Sprecher haengt");
    }
    pull_off(DATA);                 /* bereit fuer Daten */
    if (wait_for(CLK, CLK, 200) < 0) {
        /* 200 us ohne Reaktion: EOI, quittieren */
        *eoi = 1;
        pull_on(DATA);
        wait_us(60);
        pull_off(DATA);
        if (wait_for(CLK, CLK, 0) < 0) {
            fail("Sprecher nach EOI");
        }
    }
    for (i = 0; i < 8; i++) {
        if (wait_for(CLK, 0, 2000) < 0) {
            fail("Bit-Takt fehlt");
        }
        if (!(bus & DATA)) {
            b |= 1 << i;
        }
        if (wait_for(CLK, CLK, 2000) < 0) {
            fail("Bit-Takt haengt");
        }
    }
    pull_on(DATA);                  /* Byte quittieren */
    return b;
}

/* ---- Befehle ----------------------------------------------------------- */

static void listen(int sa_cmd)
{
    atn_begin();
    send_byte((uint8_t)(0x20 + unit), 0);
    send_byte((uint8_t)sa_cmd, 0);
    atn_end();
}

static void unlisten(void)
{
    atn_begin();
    send_byte(0x3f, 0);
    atn_end();
    pull_off(CLK | DATA);
}

static void talk(int sa)
{
    atn_begin();
    send_byte((uint8_t)(0x40 + unit), 0);
    send_byte((uint8_t)(0x60 | sa), 0);
    /* Umschalten: Rechner wird Hoerer */
    pull_on(DATA);
    pull_off(ATN);
    wait_us(20);
    pull_off(CLK);
    if (wait_for(CLK, CLK, 2000) < 0) {
        fail("Laufwerk uebernimmt nicht (Umschaltung)");
    }
}

static void untalk(void)
{
    atn_begin();
    send_byte(0x5f, 0);
    atn_end();
    pull_off(CLK | DATA);
}

static void send_string(const unsigned char *s, size_t n)
{
    size_t i;
    for (i = 0; i < n; i++) {
        send_byte(s[i], i + 1 == n);
    }
}

static void open_file(int sa, const char *name)
{
    listen(0xf0 | sa);
    send_string((const unsigned char *)name, strlen(name));
    unlisten();
}

static void close_file(int sa)
{
    listen(0xe0 | sa);
    unlisten();
}

static size_t read_channel(int sa, unsigned char *buf, size_t max)
{
    size_t n = 0;
    int eoi = 0;
    talk(sa);
    while (!eoi) {
        int b = recv_byte(&eoi);
        if (n < max) {
            buf[n++] = (unsigned char)b;
        }
    }
    untalk();
    return n;
}

static void upper(char *s)
{
    for (; *s; s++) {
        *s = (char)toupper((unsigned char)*s);
    }
}

static void cmd_status(void)
{
    unsigned char buf[256];
    size_t n = read_channel(15, buf, sizeof(buf) - 1), i;
    for (i = 0; i < n; i++) {
        if (buf[i] == 13) {
            buf[i] = 0;
        }
    }
    buf[n] = 0;
    printf("Status: %s\n", buf);
}

static void cmd_load(const char *name, const char *out)
{
    static unsigned char buf[1 << 20];
    char nm[64];
    size_t n;
    FILE *f;

    snprintf(nm, sizeof(nm), "%s", name);
    upper(nm);
    open_file(0, nm);
    n = read_channel(0, buf, sizeof(buf));
    close_file(0);
    f = fopen(out, "wb");
    if (!f) {
        perror(out);
        exit(1);
    }
    fwrite(buf, 1, n, f);
    fclose(f);
    printf("load \"%s\": %zu Bytes -> %s (%.3f s emuliert)\n", nm, n, out, t_ps / 1e12);
}

static void cmd_save(const char *in, const char *name)
{
    static unsigned char buf[1 << 20];
    char nm[64];
    size_t n;
    FILE *f = fopen(in, "rb");

    if (!f) {
        perror(in);
        exit(1);
    }
    n = fread(buf, 1, sizeof(buf), f);
    fclose(f);
    snprintf(nm, sizeof(nm), "%s,P,W", name);
    upper(nm);
    open_file(1, nm);
    listen(0x61);
    send_string(buf, n);
    unlisten();
    close_file(1);
    printf("save \"%s\": %zu Bytes (%.3f s emuliert)\n", name, n, t_ps / 1e12);
}

static void cmd_cmd(const char *text)
{
    char c[128];
    snprintf(c, sizeof(c), "%s", text);
    upper(c);
    listen(0x6f);
    send_string((const unsigned char *)c, strlen(c));
    unlisten();
    printf("cmd \"%s\"\n", c);
}

int main(int argc, char **argv)
{
    int c;
    double wait_s = 2.0;

    while ((c = getopt(argc, argv, "u:w:v")) != -1) {
        switch (c) {
        case 'u': unit = atoi(optarg); break;
        case 'w': wait_s = atof(optarg); break;
        case 'v': verbose++; break;
        default:
            goto usage;
        }
    }
    if (optind + 1 >= argc) {
usage:
        fprintf(stderr, "usage: iechost [-u unit] [-w s] [-v] bus "
                "{load NAME FILE | save FILE NAME | status | cmd TEXT | dir FILE}...\n");
        return 2;
    }
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    sl = simlink_open(argv[optind], SIMLINK_BUS_IEC, 0, 0xff);
    if (!sl) {
        sl = simlink_open(argv[optind], SIMLINK_BUS_IEC, 1, 0xff);
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
    fprintf(stderr, "iechost: Rechner an IEC-Bus '%s' (Teilnehmer %d), Laufwerk %d\n",
            argv[optind], slot, unit);
    /* erst loslaufen, wenn Laufwerke am Bus haengen */
    while (simlink_others(sl, slot) == 0 && !stop) {
        usleep(10000);
    }
    t_ps = simlink_start_time(sl, slot);
    lines(0);
    wait_us((unsigned)(wait_s * 1e6));

    for (c = optind + 1; c < argc; c++) {
        if (!strcmp(argv[c], "status")) {
            cmd_status();
        } else if (!strcmp(argv[c], "load") && c + 2 < argc) {
            cmd_load(argv[c + 1], argv[c + 2]);
            c += 2;
        } else if (!strcmp(argv[c], "dir") && c + 1 < argc) {
            cmd_load("$", argv[c + 1]);
            c += 1;
        } else if (!strcmp(argv[c], "save") && c + 2 < argc) {
            cmd_save(argv[c + 1], argv[c + 2]);
            c += 2;
        } else if (!strcmp(argv[c], "cmd") && c + 1 < argc) {
            cmd_cmd(argv[c + 1]);
            c += 1;
        } else {
            fprintf(stderr, "unbekannter Befehl: %s\n", argv[c]);
            break;
        }
    }
    simlink_detach(sl, slot);
    simlink_close(sl, 0);
    return 0;
}
