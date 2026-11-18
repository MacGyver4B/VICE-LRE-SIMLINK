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
 * Daten laufen in Portionen bis 254 Bytes: Lesen per TALK ... UNTALK, die
 * naechste Portion erst, wenn die vorige ausgeliefert ist; Schreiben per
 * LISTEN ... UNLISTEN, das letzte Byte mit EOI. Mit -v steht jede Portion im
 * Log, mit -v -v zusaetzlich jedes Byte unter ATN.
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
static uint64_t t_back = 0;         /* Zeit auf der Laufwerksseite (Holen/Schreiben) */

typedef struct {
    simlink_t *sl;
    int slot;
    uint64_t off;
    uint8_t pull, data;             /* eigene Leitungen */
    uint8_t in, in_data;            /* zuletzt gelesener Bus */
} sbus_t;

static sbus_t I, C;                 /* IEEE, IEC */
static sbus_t *FR = &I, *BK = &C;   /* Rechnerseite (vorn), Laufwerksseite (hinten) */
static int reverse = 0;
static int whole_writes = 0;       /* -W: Schreibdaten erst bei UNLISTEN weiterreichen */             /* -r: Rechner am IEC-Bus, Laufwerke am IEEE-Bus */

static void on_sig(int s)
{
    (void)s;
    stop = 1;
}

static void quit(int code)
{
    if (verbose && t > 0) {
        fprintf(stderr, "relay: Zeit gesamt %.3f s, davon Laufwerksseite %.3f s, Rechnerseite %.3f s\n",
                (double)t / 1e12, (double)t_back / 1e12, (double)(t - t_back) / 1e12);
    }
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

/* Auf der Rechnerseite nie unbegrenzt warten: in Schritten, und die
   Laufwerksseite jeweils mitziehen - sonst stuende dort die Zeit, und das
   Laufwerk koennte z. B. nach CLOSE den Verzeichniseintrag nicht schreiben. */
#define IDLE_STEP_US 1000

static void wait_change_raw(sbus_t *b, uint8_t mask, unsigned timeout_us);

static void wait_change(sbus_t *b, uint8_t mask, unsigned timeout_us)
{
    if (b == FR && (timeout_us == 0 || timeout_us > IDLE_STEP_US)) {
        wait_change_raw(b, mask, IDLE_STEP_US);
        rd(BK);
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
/* Kanal sa lesen: TALK, bis EOI oder max Bytes, UNTALK. *got_eoi = 1, wenn
   das letzte Byte EOI trug. Ohne EOI hoert der Rechner nach max Bytes per
   UNTALK auf; beim naechsten TALK liefert die Floppy an derselben Stelle
   weiter. */
static size_t c_read_channel(int sa, unsigned char *buf, size_t max, int *got_eoi)
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
        while (!eoi && n < max) {
            b = c_recv(&eoi);
            if (b < 0) {
                break;
            }
            buf[n++] = (unsigned char)b;
        }
        /* Quittung des letzten Bytes (DATA low) stehen lassen, bis der
           Sprecher sie sicher gesehen hat; sonst sendet er es beim naechsten
           TALK noch einmal */
        wait_us(&C, 100);
    }
    if (c_atn(0x5f, -1) == 0) {
        c_atn_end();
    }
    c_off(CLK_C | DATA_C);
    *got_eoi = eoi;
    return n;
}

/* LISTEN sec, n Bytes, UNLISTEN; EOI auf dem letzten Byte nur mit eoi_last.
   Ohne EOI schreibt die Floppy beim naechsten LISTEN auf denselben Kanal
   einfach weiter. */
static void c_write(int sec, const unsigned char *s, size_t n, int eoi_last)
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
        if (c_send(s[i], eoi_last && i + 1 == n) < 0) {
            fprintf(stderr, "relay: IEC Senden abgebrochen bei Byte %lu\n", (unsigned long)i);
            iec_err = 1;
            break;
        }
    }
    c_unlisten();
}

/* ============== Gegenrichtung: IEC-Geraeteseite (-r) ================= */

static int d_unit = 8;              /* eigene Nummer am IEC-Bus */

static void d_drive(uint8_t pull) { set(&C, pull, 0); }
static void d_on(uint8_t m)  { d_drive(C.pull | m); }
static void d_off(uint8_t m) { d_drive((uint8_t)(C.pull & ~m)); }

/* warten, bis (Bus & m) == want. atn >= 0: abbrechen, sobald ATN nicht
   mehr diesem Zustand entspricht (1 = gezogen). 0 ok, -1 ATN, -2 Frist */
static int d_wait(uint8_t m, uint8_t want, int atn, unsigned timeout_us)
{
    uint64_t dl = timeout_us ? t + (uint64_t)timeout_us * STEP : 0;
    tick(&C);
    for (;;) {
        if (atn >= 0 && !!(C.in & ATN_C) != atn) {
            return -1;
        }
        if ((C.in & m) == want) {
            return 0;
        }
        if (dl && t >= dl) {
            return -2;
        }
        wait_change(&C, (uint8_t)(m | ATN_C), dl ? (unsigned)((dl - t) / STEP) : 0);
    }
}

/* ein Byte als Hoerer; under_atn: unter ATN. -1 bei ATN-Wechsel */
static int d_recv(int under_atn, int *eoi)
{
    int i, b = 0, r;

    *eoi = 0;
    /* Erst muss der Sprecher CLK gezogen haben: beim Beginn von ATN zieht
       der C64 ATN und erst einige us spaeter CLK - wer vorher nur auf
       "CLK frei" schaut, gibt DATA zu frueh frei, und der C64 meldet
       "device not present". Nur kurz warten: kam das Relais spaet dazu (z. B.
       nach dem Anlauf), ist CLK schon wieder frei und bleibt es. */
    if (d_wait(CLK_C, CLK_C, under_atn, 100) == -1) {
        return -1;
    }
    /* Sprecher bereit: er gibt CLK frei */
    if (d_wait(CLK_C, 0, under_atn, 0) < 0) {
        return -1;
    }
    d_off(DATA_C);                  /* bereit fuer Daten */
    r = d_wait(CLK_C, CLK_C, under_atn, 200);
    if (r == -1) {
        return -1;
    }
    if (r == -2) {
        /* EOI: Sprecher wartet; 60 us DATA ziehen als Quittung */
        *eoi = 1;
        d_on(DATA_C);
        wait_us(&C, 60);
        d_off(DATA_C);
        if (d_wait(CLK_C, CLK_C, under_atn, 0) < 0) {
            return -1;
        }
    }
    for (i = 0; i < 8; i++) {
        if (d_wait(CLK_C, 0, under_atn, 0) < 0) {          /* Bit gueltig */
            return -1;
        }
        if (!(C.in & DATA_C)) {
            b |= 1 << i;
        }
        if (d_wait(CLK_C, CLK_C, under_atn, 0) < 0) {
            return -1;
        }
    }
    d_on(DATA_C);                   /* Quittung */
    return b;
}

/* ein Byte als Sprecher; -1 bei ATN, -2 keine Quittung */
static int d_send(int b, int eoi)
{
    int i;

    d_off(CLK_C);                   /* bereit zum Senden */
    if (d_wait(DATA_C, 0, 0, 0) < 0) {          /* Hoerer bereit */
        return -1;
    }
    if (eoi) {
        if (d_wait(DATA_C, DATA_C, 0, 0) < 0 || d_wait(DATA_C, 0, 0, 0) < 0) {
            return -1;
        }
    }
    d_on(CLK_C);
    /* Start (CLK low) lang genug halten: der C64 liest nach dem Freigeben
       von DATA bis zu rund 80 us nicht (Kernal ACPTR, dazu Badlines); sieht
       er den Start nicht, haelt er das erste Bit dafuer und zaehlt nur 7 */
    wait_us(&C, 150);
    for (i = 0; i < 8; i++) {
        if (b & (1 << i)) {
            d_off(DATA_C);
        } else {
            d_on(DATA_C);
        }
        /* Beide Phasen laenger als ACPTR braucht, um einen Wechsel sicher zu
           sehen: Doppel-Lesen ($DD00 zweimal, gleich), dazu eine Badline
           (bis 47 Takte) - sonst haelt der C64 das naechste Bit fuer das
           alte und zaehlt eins zu wenig */
        wait_us(&C, 40);
        d_off(CLK_C);
        wait_us(&C, 80);
        d_on(CLK_C);
        d_off(DATA_C);
        wait_us(&C, 60);
    }
    if (d_wait(DATA_C, DATA_C, 0, 1000) < 0) {
        return -2;
    }
    return 0;
}

/* =========== Gegenrichtung: IEEE-Controllerseite (-r) ================= */

static int k_unit = 8;              /* Laufwerk am IEEE-Bus */

#define KLOW(m) (I.in & (m))

static void k_drive(uint8_t pull, uint8_t data) { set(&I, pull, data); }

/* warten, bis (Bus & m) == want; 0 ok, -1 Frist */
static int k_wait(uint8_t m, uint8_t want, unsigned timeout_us)
{
    uint64_t dl = timeout_us ? t + (uint64_t)timeout_us * STEP : 0;
    tick(&I);
    for (;;) {
        if ((I.in & m) == want) {
            return 0;
        }
        if (dl && t >= dl) {
            return -1;
        }
        wait_change(&I, m, dl ? (unsigned)((dl - t) / STEP) : 0);
    }
}

/* ein Byte als Sprecher (Controller unter ATN oder Daten) */
static int k_send(uint8_t b, int eoi)
{
    uint8_t base = (uint8_t)(I.pull & SL_IEEE_ATN);

    k_drive(base, 0);
    if (k_wait(SL_IEEE_NRFD, 0, 0) < 0) {        /* alle Hoerer bereit */
        return -1;
    }
    k_drive((uint8_t)(base | (eoi ? SL_IEEE_EOI : 0)), b);
    wait_us(&I, 2);
    k_drive((uint8_t)(base | SL_IEEE_DAV | (eoi ? SL_IEEE_EOI : 0)), b);
    if (k_wait(SL_IEEE_NDAC, 0, 65000) < 0) {    /* alle haben angenommen */
        k_drive(base, 0);
        return -1;
    }
    k_drive(base, 0);
    return 0;
}

/* ATN, Adressbyte, Sekundaerbyte (b2 < 0: keins). -1: Geraet fehlt */
static int k_atn(uint8_t b1, int b2)
{
    k_drive(SL_IEEE_ATN, 0);
    wait_us(&I, 100);
    if (!KLOW(SL_IEEE_NDAC) && !KLOW(SL_IEEE_NRFD)) {
        k_drive(0, 0);
        return -1;                  /* niemand quittiert ATN */
    }
    if (k_send(b1, 0) < 0 || (b2 >= 0 && k_send((uint8_t)b2, 0) < 0)) {
        k_drive(0, 0);
        return -1;
    }
    return 0;
}

static void k_atn_end(void)
{
    wait_us(&I, 2);
    k_drive(0, 0);
    wait_us(&I, 20);
}

/* Offenes LISTEN am IEEE-Bus (-1: keins). Schreibdaten in Portionen gehen in
   EINER Sitzung (LISTEN ... UNLISTEN) wie beim PET: ein neues LISTEN je
   Portion kam bei der 8250 genau dann, wenn sie den vollen Block schrieb,
   und sie gab danach NRFD nicht mehr frei. */
static int k_listening = -1;

static void k_unlisten(void)
{
    if (k_listening >= 0) {
        if (k_atn(0x3f, -1) == 0) {
            k_atn_end();
        }
        k_listening = -1;
    }
}

static void k_write(int sec, const unsigned char *s, size_t n, int eoi_last)
{
    size_t i;
    if (verbose > 1) {
        fprintf(stderr, "relay: IEEE LISTEN %d $%02X, %lu Bytes%s\n", k_unit, sec, (unsigned long)n,
                k_listening == sec ? " (weiter)" : "");
    }
    if (k_listening != sec) {
        k_unlisten();
        if (k_atn((uint8_t)(0x20 + k_unit), sec) < 0) {
            iec_err = 1;
            return;
        }
        k_atn_end();
        k_listening = sec;
    }
    for (i = 0; i < n; i++) {
        if (k_send(s[i], eoi_last && i + 1 == n) < 0) {
            fprintf(stderr, "relay: IEEE Senden abgebrochen bei Byte %lu\n", (unsigned long)i);
            iec_err = 1;
            break;
        }
    }
    if (eoi_last || iec_err) {
        k_unlisten();
    }
}

/* Kanal sa lesen: TALK, bis EOI oder max Bytes, UNTALK */
static size_t k_read(int sa, unsigned char *buf, size_t max, int *got_eoi)
{
    size_t n = 0;
    int eoi = 0;

    *got_eoi = 0;
    k_unlisten();
    if (k_atn((uint8_t)(0x40 + k_unit), 0x60 | sa) < 0) {
        iec_err = 1;
        return 0;
    }
    /* Umschalten: Controller wird Hoerer */
    k_drive(SL_IEEE_ATN | SL_IEEE_NRFD | SL_IEEE_NDAC, 0);
    wait_us(&I, 2);
    k_drive(SL_IEEE_NRFD | SL_IEEE_NDAC, 0);
    while (!eoi && n < max) {
        k_drive(SL_IEEE_NDAC, 0);                   /* bereit */
        /* erstes Byte: das Laufwerk darf suchen; danach wie der PET */
        if (k_wait(SL_IEEE_DAV, SL_IEEE_DAV, n == 0 ? 500000 : 65000) < 0) {
            break;                                  /* Sprecher schweigt */
        }
        k_drive(SL_IEEE_NRFD | SL_IEEE_NDAC, 0);
        buf[n++] = I.in_data;
        eoi = KLOW(SL_IEEE_EOI) ? 1 : 0;
        k_drive(SL_IEEE_NRFD, 0);                   /* angenommen */
        if (k_wait(SL_IEEE_DAV, 0, 65000) < 0) {
            break;
        }
        k_drive(SL_IEEE_NRFD | SL_IEEE_NDAC, 0);
    }
    /* UNTALK */
    k_drive(SL_IEEE_ATN, 0);
    wait_us(&I, 100);
    k_send(0x5f, 0);
    k_atn_end();
    *got_eoi = eoi;
    return n;
}

static void k_close(int sa)
{
    k_unlisten();
    if (k_atn((uint8_t)(0x20 + k_unit), 0xe0 | sa) == 0) {
        k_send(0x3f, 0);
        k_atn_end();
    }
}

/* ---- Laufwerksseite, je nach Richtung -------------------------------- */

static void be_write_raw(int sec, const unsigned char *s, size_t n, int eoi_last);

static size_t be_read(int sa, unsigned char *buf, size_t max, int *eoi)
{
    return reverse ? k_read(sa, buf, max, eoi) : c_read_channel(sa, buf, max, eoi);
}

static void be_write(int sec, const unsigned char *s, size_t n, int eoi_last)
{
    uint64_t t0 = t;
    be_write_raw(sec, s, n, eoi_last);
    t_back += t - t0;
}

static void be_write_raw(int sec, const unsigned char *s, size_t n, int eoi_last)
{
    if (reverse) {
        k_write(sec, s, n, eoi_last);
    } else {
        c_write(sec, s, n, eoi_last);
    }
}

static void be_close(int sa)
{
    if (reverse) {
        k_close(sa);
    } else {
        c_listen(0xe0 | sa);
        c_unlisten();
    }
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
static int ceof[16];                /* Dateiende von der IEC-Seite gesehen */
static unsigned long ctotal[16];    /* bisher von der IEC-Seite geholt */
static unsigned char wbuf[BUFMAX];
static size_t wlen = 0;
static unsigned long wtotal = 0;    /* schon weitergereichte Schreibdaten */
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
#define CHUNK 254

static void prefetch(int sa)
{
    int eoi = 0;
    const char *what = sa == 15 ? "15" : (sa == 0 ? "0 (LOAD)" : "n");

    /* Fehlerkanal: neu holen, wenn verbraucht oder nach einem Befehl
       (cvalid[15] = 0) - GET# liest mit eigenem TALK je Byte, z. B. die
       Antwort auf M-R. Datenkanal: in Portionen bis CHUNK Bytes, die naechste
       erst, wenn die vorige ausgeliefert ist; danach Dateiende. */
    if (cvalid[sa] && cpos[sa] < clen[sa]) {
        return;
    }
    if (sa != 15 && cvalid[sa] && ceof[sa]) {
        return;
    }
    if (!cbuf[sa]) {
        cbuf[sa] = malloc(BUFMAX);
    }
    if (!cvalid[sa]) {
        ctotal[sa] = 0;
    }
    iec_err = 0;
    {
        uint64_t t0 = t;
        clen[sa] = be_read(sa, cbuf[sa], sa == 15 ? BUFMAX : CHUNK, &eoi);
        t_back += t - t0;
    }
    cpos[sa] = 0;
    cvalid[sa] = 1;
    ctotal[sa] += clen[sa];
    /* Fehlerkanal immer ganz; Datenkanal: Ende bei EOI, bei weniger als einer
       Portion (Abbruch) oder wenn gar nichts kam (Datei fehlt) */
    ceof[sa] = (sa == 15 || eoi || clen[sa] < CHUNK);
    if (ceof[sa]) {
        fprintf(stderr, "relay %10.3f ms: Kanal %s geholt: %lu Bytes%s\n", (double)t / 1e9,
                what, ctotal[sa], (sa == 15 || eoi || ctotal[sa] == 0) ? "" : " (ohne EOI abgebrochen)");
    } else if (verbose) {
        fprintf(stderr, "relay %10.3f ms: Kanal %s: %lu Bytes bisher\n", (double)t / 1e9,
                what, ctotal[sa]);
    }
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
        be_write(0xf0 | opening, name, namelen, 1);
        cvalid[opening] = 0;
        if (namelen) {
            cvalid[15] = 0;     /* OPEN mit Namen setzt den Fehlerkanal neu */
        }
        opening = -1;
    } else if (wlen) {
        iec_err = 0;
        be_write(0x60 | sa, wbuf, wlen, 1);
        fprintf(stderr, "relay %10.3f ms: Daten an Kanal %d weitergereicht: %lu Bytes%s\n",
                (double)t / 1e9, sa, wtotal + (unsigned long)wlen, iec_err ? " (FEHLER)" : "");
        wlen = 0;
        wtotal = 0;
        if (sa == 15) {
            cvalid[15] = 0;     /* Befehl: Fehlerkanal neu holen */
        }
    }
}

/* Gegenrichtung: Hauptschleife als Geraet am IEC-Bus */
static void run_reverse(void)
{
    enum { IDLE, LISTENER, TALKER } role = IDLE;
    int sa = 0, turned = 0;

    for (;;) {
        int eoi = 0, b;
        tick(&C);
        if (C.in & ATN_C) {
            /* ATN: sofort DATA ziehen (Quittung), CLK frei */
            if (!(C.pull & DATA_C) || (C.pull & CLK_C)) {
                d_drive(DATA_C);
            }
            b = d_recv(1, &eoi);
            if (b < 0) {
                continue;
            }
            if (verbose > 1) {
                fprintf(stderr, "relay: IEC ATN $%02X\n", b);
            }
            if (b == 0x20 + d_unit) {
                role = LISTENER;
            } else if (b == 0x3f) {
                if (role == LISTENER) {
                    finish_listen(sa);
                    role = IDLE;
                }
            } else if (b == 0x40 + d_unit) {
                role = TALKER;
                turned = 0;
            } else if (b == 0x5f) {
                if (role == TALKER) {
                    role = IDLE;
                }
            } else if ((b & 0xe0) == 0x20 || (b & 0xe0) == 0x40) {
                if (role == TALKER && (b & 0xe0) == 0x40) {
                    role = IDLE;        /* anderes Geraet spricht */
                }
            } else if (role != IDLE && (b & 0xf0) == 0x60) {
                sa = b & 0x0f;
                wlen = 0;
                wtotal = 0;
            } else if (role != IDLE && (b & 0xf0) == 0xf0) {
                sa = b & 0x0f;
                opening = sa;
                namelen = 0;
            } else if (role != IDLE && (b & 0xf0) == 0xe0) {
                sa = b & 0x0f;
                log_msg("CLOSE%s %ld", "", sa);
                be_close(sa);
                cvalid[sa] = 0;
            }
            continue;
        }
        if (role == LISTENER) {
            /* DATA bleibt gezogen, bis der Sprecher bereit ist */
            b = d_recv(0, &eoi);
            if (b < 0) {
                continue;
            }
            if (opening >= 0) {
                if (namelen < sizeof(name)) {
                    name[namelen++] = (unsigned char)b;
                }
            } else if (wlen < BUFMAX) {
                wbuf[wlen++] = (unsigned char)b;
                if (sa != 15 && wlen == CHUNK + 1 && !whole_writes) {
                    be_write(0x60 | sa, wbuf, CHUNK, 0);
                    wtotal += CHUNK;
                    wbuf[0] = wbuf[CHUNK];
                    wlen = 1;
                    if (verbose) {
                        fprintf(stderr, "relay %10.3f ms: Kanal %d: %lu Bytes geschrieben bisher (IEEE-Seite bei %.3f ms)\n",
                                (double)t / 1e9, sa, wtotal,
                                (double)(simlink_peers_progress(I.sl, I.slot, NULL) - I.off) / 1e9);
                    }
                    tick(&C);
                }
            }
            continue;
        }
        if (role == TALKER) {
            if (!turned) {
                /* Umschalten: Rechner zieht DATA und gibt CLK frei; dann
                   uebernimmt das Geraet CLK (Sprecher, noch nicht bereit) */
                d_drive(DATA_C);
                if (d_wait(CLK_C, 0, 0, 0) < 0) {
                    continue;
                }
                d_drive(CLK_C);
                turned = 1;
                wait_us(&C, 80);
                /* Daten holen, solange CLK gehalten ist (der Rechner wartet
                   darauf ohne Frist) */
                prefetch(sa);
                tick(&C);
            }
            if (cvalid[sa] && cpos[sa] >= clen[sa] && !ceof[sa]) {
                prefetch(sa);           /* naechste Portion, CLK gehalten */
                tick(&C);
            }
            if (!cvalid[sa] || cpos[sa] >= clen[sa]) {
                /* nichts zu liefern: CLK frei und schweigen; der Rechner
                   meldet den Fehler (Zeitueberschreitung) */
                d_drive(0);
                if (!(C.in & ATN_C)) {
                    wait_change(&C, ATN_C, 0);
                }
                continue;
            }
            b = d_send(cbuf[sa][cpos[sa]], cpos[sa] + 1 == clen[sa] && ceof[sa]);
            if (b == 0) {
                cpos[sa]++;
                if (cpos[sa] >= clen[sa] && ceof[sa]) {
                    d_drive(0);
                    role = IDLE;
                }
            } else if (b == -2) {
                fprintf(stderr, "relay %10.3f ms: IEC: Byte %lu von Kanal %d nicht quittiert\n",
                        (double)t / 1e9, (unsigned long)cpos[sa], sa);
                d_drive(0);             /* Rechner hat nicht quittiert */
                role = IDLE;
            }
            continue;
        }
        d_drive(0);
        if (!(C.in & ATN_C)) {
            wait_change(&C, ATN_C, 0);
        }
    }
}

int main(int argc, char **argv)
{
    int c;
    double wait_s = 3.0;
    enum { IDLE, LISTENER, TALKER } role = IDLE;
    int sa = 0;

    while ((c = getopt(argc, argv, "u:d:w:vrW")) != -1) {
        switch (c) {
        case 'u': ieee_unit = d_unit = atoi(optarg); break;
        case 'd': iec_unit = k_unit = atoi(optarg); break;
        case 'r': reverse = 1; break;
        case 'W': whole_writes = 1; break;
        case 'w': wait_s = atof(optarg); break;
        case 'v': verbose++; break;
        default: goto usage;
        }
    }
    if (optind + 2 != argc) {
usage:
        fprintf(stderr, "usage: relay [-r] [-u geraet] [-d laufwerk] [-w s] [-v] ieeebus iecbus\n"
                        "  ohne -r: Rechner (PET) am IEEE-Bus, Laufwerk am IEC-Bus\n"
                        "  -r:      Rechner (C64) am IEC-Bus, Laufwerk am IEEE-Bus\n"
                        "  -u: eigene Nummer am Rechnerbus, -d: Nummer des Laufwerks\n");
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
    if (reverse) {
        FR = &C;
        BK = &I;
        fprintf(stderr, "relay: IEC '%s' Geraet %d  <->  IEEE '%s' Laufwerk %d\n",
                argv[optind + 1], d_unit, argv[optind], k_unit);
    } else {
        fprintf(stderr, "relay: IEEE '%s' Geraet %d  <->  IEC '%s' Laufwerk %d\n",
                argv[optind], ieee_unit, argv[optind + 1], iec_unit);
    }
    {
        int waited = 0;
        while ((simlink_others(I.sl, I.slot) == 0 || simlink_others(C.sl, C.slot) == 0) && !stop) {
            usleep(10000);
            if (++waited > 3000) {
                fprintf(stderr, "relay: nach 30 s keine Gegenseite an beiden Bussen, Ende\n");
                quit(1);
            }
        }
    }
    started = 1;
    I.off = simlink_start_time(I.sl, I.slot);
    C.off = simlink_start_time(C.sl, C.slot);
    t = 0;              /* VICE uebernimmt den Zustand zur Startzeit */
    if (reverse) {
        /* Laufwerk hochfahren lassen (IEEE-Zeit vor); der C64 wartet so
           lange, weil seine Zeit steht */
        t = STEP;
        d_drive(DATA_C);    /* derweil DATA halten: der C64 wartet ohne Frist */
        wait_us(&I, (unsigned)(wait_s * 1e6));
        run_reverse();
    }
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
            if (verbose > 1) {
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
                wtotal = 0;
            } else if (role != IDLE && (b & 0xf0) == 0xf0) {
                sa = b & 0x0f;
                opening = sa;
                namelen = 0;
            } else if (role != IDLE && (b & 0xf0) == 0xe0) {
                sa = b & 0x0f;
                log_msg("CLOSE%s %ld", "", sa);
                be_close(sa);
                cvalid[sa] = 0;
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
                /* Schreibdaten (nicht Befehle) in Portionen weiterreichen,
                   dabei NRFD halten. Ein Byte bleibt zurueck, damit das
                   letzte bei UNLISTEN mit EOI rausgeht. */
                if (sa != 15 && wlen == CHUNK + 1 && !whole_writes) {
                    be_write(0x60 | sa, wbuf, CHUNK, 0);
                    wtotal += CHUNK;
                    wbuf[0] = wbuf[CHUNK];
                    wlen = 1;
                    if (verbose) {
                        fprintf(stderr, "relay %10.3f ms: Kanal %d: %lu Bytes geschrieben bisher (IEEE-Seite bei %.3f ms)\n",
                                (double)t / 1e9, sa, wtotal,
                                (double)(simlink_peers_progress(I.sl, I.slot, NULL) - I.off) / 1e9);
                    }
                    tick(&I);
                }
            }
            continue;
        }
        if (role == TALKER) {
            i_drive((uint8_t)(I.pull & ~(SL_IEEE_NDAC | SL_IEEE_NRFD)), 0);
            if (cvalid[sa] && cpos[sa] >= clen[sa] && !ceof[sa]) {
                /* naechste Portion holen; der PET wartet derweil auf DAV
                   (im Test steht seine Zeit, solange das Relais am IEC-Bus
                   arbeitet) */
                prefetch(sa);
                tick(&I);
            }
            if (!cvalid[sa] || cpos[sa] >= clen[sa]) {
                /* nichts zu liefern: schweigen, der PET meldet den Fehler */
                if (!LOW(SL_IEEE_ATN)) {
                    wait_change(&I, SL_IEEE_ATN, 0);
                }
                continue;
            }
            if (i_talk_byte(cbuf[sa][cpos[sa]], cpos[sa] + 1 == clen[sa] && ceof[sa]) == 0) {
                cpos[sa]++;
                if (cpos[sa] >= clen[sa] && ceof[sa]) {
                    role = IDLE;
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
