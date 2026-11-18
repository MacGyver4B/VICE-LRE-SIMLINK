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

static int c_fast;
static void c_fast_send(uint8_t b);
static int c_atn(uint8_t b1, int b2)
{
    if (c_fast) {
        c_fast_send(0xff);       /* schneller Rechner (C128 $E367) */
        wait_us(&C, 40);
    }
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

/* Kanaele (Lesepuffer je Sekundaeradresse) */
static unsigned char *cbuf[16];
static size_t clen[16], cpos[16];
static int cvalid[16];

/* ================= AR6-Schnelllader (Rechnerseite, -a) =================
   Nachbildung der Host-Seite des Action Replay 6 (bank2.s, V6.0 PAL) fuer
   die 1541: Laufwerkscode hochladen, Spuren mit Befehl $E0 lesen, Name und
   Sektorkette im Relais verfolgen. Der Laufwerkscode wird zur Laufzeit aus
   dem CRT-Abbild gelesen (-a datei.crt), er liegt nicht im Quelltext.
   Zeiten in Laufwerkstakten (1 us), gemessen ab dem eigenen Schreibzugriff
   bzw. ab der beobachteten Flanke. Leitung gesetzt = low. */

#define US(n) ((uint64_t)(n) * STEP)

static unsigned char *ar_rom = NULL;    /* Bank 2, $8000-$9FFF */
static unsigned ar_gap = 30;    /* Pause nach DATA low; Laufwerk braucht bis T0+72 */
static int ar_pending = 0;              /* Kanal 0 wird per AR6 geladen */
static int ar_loaded = 0;               /* Laufwerkscode laeuft */
static unsigned char ar_name[256];
static int ar_save = 0;                 /* Kanal 1 wird per AR6 gespeichert */
static unsigned char ar_sbuf[BUFMAX];
static size_t ar_slen = 0;
static size_t ar_namelen = 0;
static unsigned char ar_trk[36][21][256];
static int ar_have[36][21];

static int ar_load_crt(const char *fn)
{
    FILE *f = fopen(fn, "rb");
    unsigned char h[64], ch[16];
    if (!f || fread(h, 1, 64, f) != 64 || memcmp(h, "C64 CARTRIDGE", 13)) {
        fprintf(stderr, "relay: %s ist kein CRT-Abbild\n", fn);
        return -1;
    }
    fseek(f, (long)((h[0x10] << 24) | (h[0x11] << 16) | (h[0x12] << 8) | h[0x13]), SEEK_SET);
    while (fread(ch, 1, 16, f) == 16) {
        long ln = (ch[4] << 24) | (ch[5] << 16) | (ch[6] << 8) | ch[7];
        int bank = (ch[0x0a] << 8) | ch[0x0b];
        if (bank == 2 && ln >= 16 + 8192) {
            ar_rom = malloc(8192);
            if (fread(ar_rom, 1, 8192, f) != 8192) {
                break;
            }
            fclose(f);
            return 0;
        }
        fseek(f, ln - 16, SEEK_CUR);
    }
    fclose(f);
    fprintf(stderr, "relay: Bank 2 fehlt in %s\n", fn);
    return -1;
}

#define AR(a) (ar_rom + ((a) - 0x8000))

/* bis zum Zeitpunkt tp vorruecken und lesen */
static void ar_at(uint64_t tp)
{
    if (tp > t) {
        t = tp;
    }
    rd(&C);
}

/* warten, bis (C.in & m) == want; liefert die Zeit der Flanke (bzw. jetzt),
   0 bei Frist */
static uint64_t ar_until(uint8_t m, uint8_t want, unsigned timeout_us)
{
    uint64_t dl = t + US(timeout_us), tc;
    rd(&C);
    while ((C.in & m) != want) {
        int ch = simlink_wait_change(C.sl, C.slot, t + C.off, m, 100000, dl + C.off, &tc);
        uint64_t tn = ch ? tc - C.off + 100000 : dl;
        if (tn < t) {
            tn = t;
        }
        simlink_wait_end(C.sl, C.slot, tn + C.off);
        t = tn;
        rd(&C);
        if (!ch) {
            return 0;
        }
        if ((C.in & m) == want) {
            return tc - C.off;
        }
    }
    return t;
}

/* Laufwerk beschaeftigt (CLK low) abwarten, dann bereit (CLK high) */
static int ar_ready(unsigned timeout_us)
{
    rd(&C);
    if (!(C.in & CLK_C)) {
        ar_until(CLK_C, CLK_C, 50000);   /* erst low (beschaeftigt) */
    }
    return ar_until(CLK_C, 0, timeout_us) ? 0 : -1;
}

/* ein Halbbyte als zwei Leitungspaare: erst (CLK=b3, DATA=b1), dann (b2, b0) */
static uint8_t ar_pair(int n, int first)
{
    uint8_t p = 0;
    if (first) {
        if (n & 8) p |= CLK_C;
        if (n & 2) p |= DATA_C;
    } else {
        if (n & 4) p |= CLK_C;
        if (n & 1) p |= DATA_C;
    }
    return p;
}

/* Rechner -> Laufwerk, 1 Byte (L824E): Freigabe T0, Paare bei +8/+18/+34/+44
   wie beim C64, DATA low bei +57. Das Laufwerk fragt alle 7 Takte ab; die
   Freigabe muss daher laenger als 7 us stehen, sonst kann es ein Byte
   uebersehen (bei +7 gingen einzelne Bytes verloren). */
static void ar_put(uint8_t b)
{
    uint64_t t0 = t + US(2);
    ar_at(t0);
    set(&C, 0, 0);
    ar_at(t0 + US(8));  set(&C, ar_pair(b >> 4, 1), 0);
    ar_at(t0 + US(18)); set(&C, ar_pair(b >> 4, 0), 0);
    ar_at(t0 + US(34)); set(&C, ar_pair(b & 15, 1), 0);
    ar_at(t0 + US(44)); set(&C, ar_pair(b & 15, 0), 0);
    ar_at(t0 + US(57)); set(&C, DATA_C, 0);
}

static int ar_send(uint8_t b)
{
    if (ar_ready(2000000) < 0) {
        return -1;
    }
    ar_put(b);
    return 0;
}

/* Abtastwert: Bit 1 = DATA high, Bit 0 = CLK high */
static int ar_sample(uint64_t tp)
{
    ar_at(tp);
    return (C.in & DATA_C ? 0 : 2) | (C.in & CLK_C ? 0 : 1);
}

/* Laufwerk -> Rechner, 1 Byte mit Handshake (L8188). Das Laufwerk gibt
   seine Leitungen frei (W) und fragt dann alle 7 Takte, ob auch der Rechner
   frei ist. Anders als der C64 geben wir DATA schon frei, solange das
   Laufwerk noch beschaeftigt ist (CLK low): dann erkennt es die Freigabe beim
   ersten Abfragen (W+4), das erste Paar kommt bei W+10, die weiteren im
   Abstand 8. Abgetastet wird in der Mitte (W+14+8k). So haengt nichts vom
   Abfragetakt oder von der Latenz der Emulation ab. */
static unsigned ar_fallback = 0;
static int ar_first = 0;        /* naechstes Byte ist das erste nach dem Start */
static unsigned ar_p0 = 14;     /* erste Abtastung nach W: 1541 +14, 1581 +10 */

static int ar_get(unsigned timeout_us)
{
    uint64_t w;
    int i, b = 0;
    rd(&C);
    if (!(C.in & CLK_C) && !ar_until(CLK_C, CLK_C, ar_p0 == 10 && !ar_first ? 100 : 50000)) {
        /* Laufwerk war schon bereit (CLK frei, wartet auf uns): ohne
           eigene Flanke. Freigabe jetzt (T0); das Laufwerk fragt alle 3,5 us
           (1581) bzw. 7 us (1541) ab, erstes Paar 3-6,5 bzw. 6-13 us
           spaeter. Rueckweg, falls wir die Belegt-Flanke verpasst haben. */
        uint64_t t0 = t + US(1);
        unsigned p = ar_p0 == 10 ? 9 : 14;
        ar_fallback++;
        ar_at(t0);
        set(&C, 0, 0);
        for (i = 0; i < 4; i++) {
            b |= ar_sample(t0 + US(p + 8 * i)) << (2 * i);
        }
        ar_at(t0 + US(p + 26));
        set(&C, DATA_C, 0);
        return b;
    }
    set(&C, 0, 0);
    ar_first = 0;
    if (!(w = ar_until(CLK_C, 0, timeout_us))) {
        set(&C, DATA_C, 0);
        return -1;
    }
    for (i = 0; i < 4; i++) {
        b |= ar_sample(w + US(ar_p0 + 8 * i)) << (2 * i);
    }
    ar_at(w + US(ar_p0 + 26));
    set(&C, DATA_C, 0);
    return b;
}

/* Block 256 Byte ohne Handshake (L8408), danach Pruefbyte; 0 ok */
static int ar_block(unsigned char *buf)
{
    uint64_t r;
    int n, k, x = 0, c;
    ar_at(t + US(15));
    set(&C, 0, 0);                              /* DATA frei: Laufwerk beginnt */
    if (!ar_until(CLK_C, 0, 5000) || !ar_until(CLK_C, CLK_C, 100) ||
        !ar_until(CLK_C, 0, 100) || !ar_until(CLK_C, CLK_C, 100) ||
        !(r = ar_until(CLK_C, 0, 100))) {
        return -1;                              /* kein Sync-Muster */
    }
    for (n = 0; n < 64; n++) {
        for (k = 0; k < 4; k++) {
            uint64_t t0 = r + US(179 * n + 12 + 40 * k);
            int b = ar_sample(t0 + US(4)) | ar_sample(t0 + US(14)) << 2 |
                    ar_sample(t0 + US(24)) << 4 | ar_sample(t0 + US(34)) << 6;
            buf[n * 4 + k] = (unsigned char)b;
            x ^= b;
        }
    }
    c = ar_get(2000);
    if (c < 0) {
        return -1;
    }
    return (x ^ c) ? -2 : 0;
}

static int ar_sectors(int tr)
{
    return tr < 18 ? 21 : tr < 25 ? 19 : tr < 31 ? 18 : 17;
}

/* ganze Spur lesen ($E0 Spur 0); Anzahl gelesener Sektoren oder <0 */
static int ar_track(int tr)
{
    unsigned char buf[256];
    int s, cnt, got = 0, e;
    if (tr < 1 || tr > 35) {
        return -1;
    }
    if (ar_send(0xe0) < 0 || ar_send((uint8_t)tr) < 0 || ar_send(0) < 0) {
        fprintf(stderr, "relay: AR6 Spur %d: Befehl nicht angenommen\n", tr);
        return -1;
    }
    cnt = ar_get(3000000);
    if (cnt < 0 || cnt >= 0x16) {
        fprintf(stderr, "relay: AR6 Spur %d: Fehler $%02X\n", tr, cnt & 0xff);
        return -1;
    }
    for (;;) {
        s = ar_get(500000);
        if (s < 0) {
            fprintf(stderr, "relay: AR6 Spur %d: keine Sektornummer\n", tr);
            return -1;
        }
        if (s >= 0x15) {
            break;
        }
        e = ar_block(buf);
        if (e == 0) {
            memcpy(ar_trk[tr][s], buf, 256);
            ar_have[tr][s] = 1;
            got++;
        } else if (e == -1) {
            fprintf(stderr, "relay: AR6 Spur %d Sektor %d: kein Block\n", tr, s);
            return -1;
        } else {
            fprintf(stderr, "relay: AR6 Spur %d Sektor %d: Pruefsumme falsch\n", tr, s);
        }
    }
    if (verbose) {
        fprintf(stderr, "relay %10.3f ms: AR6 Spur %d: %d Sektoren (gemeldet %d)\n",
                (double)t / 1e9, tr, got, cnt);
    }
    return got;
}

static unsigned char *ar_sector(int tr, int s)
{
    int tries;
    if (tr < 1 || tr > 35 || s < 0 || s >= ar_sectors(tr)) {
        return NULL;
    }
    for (tries = 0; !ar_have[tr][s] && tries < 3; tries++) {
        if (ar_track(tr) < 0) {
            return NULL;
        }
    }
    return ar_have[tr][s] ? ar_trk[tr][s] : NULL;
}

/* M-W in Stuecken zu 35 Bytes (L892B) */
static void ar_mw(int addr, const unsigned char *s, int n)
{
    unsigned char cmd[64];
    while (n > 0) {
        int k = n < 35 ? n : 35;
        cmd[0] = 'M'; cmd[1] = '-'; cmd[2] = 'W';
        cmd[3] = (unsigned char)addr; cmd[4] = (unsigned char)(addr >> 8); cmd[5] = (unsigned char)k;
        memcpy(cmd + 6, s, (size_t)k);
        c_write(0x6f, cmd, (size_t)(6 + k), 1);
        addr += k; s += k; n -= k;
    }
}

/* Laufwerkscode hochladen (L88FE/L8962): Bootstrap nach $0150, M-E mit
   Seitenzahl und Startadresse (Tabelle L9EBA), dann Seite 0 ab $93CD nach
   $0300 und weitere Seiten ab src nach $0400 ... schnell.
   LOAD: src $94CC, 2 Seiten, Start $0331; SAVE: src $8E65, 2 Seiten, $044F */
static int ar_upload_x(int src2, int pages, int start)
{
    unsigned char me[] = { 'M', '-', 'E', 0x50, 0x01, 0, 0, 0, 0x2c, 0xa0 };
    int p, i;
    me[5] = (unsigned char)pages;
    me[6] = (unsigned char)start;
    me[7] = (unsigned char)(start >> 8);
    iec_err = 0;
    ar_mw(0x0150, AR(0x8a2e), 70);
    c_write(0x6f, me, sizeof me, 1);
    if (iec_err) {
        return -1;
    }
    set(&C, DATA_C, 0);                         /* Ruhezustand: DATA low */
    for (p = 0; p <= pages; p++) {
        const unsigned char *src = p == 0 ? AR(0x93cd) : AR(src2 + 256 * (p - 1));
        if (ar_ready(200000) < 0) {
            fprintf(stderr, "relay: AR6 Upload: Laufwerk meldet sich nicht (Seite %d)\n", p);
            set(&C, 0, 0);
            return -1;
        }
        for (i = 0; i < 256; i++) {
            ar_put(src[i]);
            ar_at(t + US(ar_gap));              /* Laufwerk zurueck in die Warteschleife */
        }
    }
    ar_loaded = 1;
    return 0;
}

static int ar_upload(void)
{
    return ar_upload_x(0x94cc, 2, 0x0331);
}

/* zurueck ins DOS: Byte unter $80 als Fehlercode (0 = ok, $62 = Datei fehlt) */
static void ar_exit(uint8_t code)
{
    if (ar_loaded) {
        ar_send(code);
        ar_until(CLK_C, CLK_C, 2000);
        set(&C, 0, 0);
        ar_at(t + US(2000));
        ar_loaded = 0;
    }
}

/* Name wie das DOS vergleichen: ? passt auf alles, * beendet */
static int ar_match(const unsigned char *pat, size_t pn, const unsigned char *e)
{
    size_t i;
    for (i = 0; i < 16; i++) {
        if (i < pn && pat[i] == '*') {
            return 1;
        }
        if (i >= pn) {
            return e[i] == 0xa0;
        }
        if (pat[i] != '?' && pat[i] != e[i]) {
            return 0;
        }
    }
    return pn <= 16;
}

/* Laufwerkstyp per M-R $FFFE (wie AR6 L89D0): Low-Byte des IRQ-Vektors,
   $67 = 1541/1570/1571, $03 = 1581; sonst 0 (Standardprotokoll). Bei jedem
   Zugriff neu, damit ein getauschtes oder spaeter eingeschaltetes Laufwerk
   richtig erkannt wird. */
static int ar_detect(void)
{
    static const unsigned char mr[] = { 'M', '-', 'R', 0xfe, 0xff, 1 };
    unsigned char b[8];
    int eoi = 0;
    size_t n;
    iec_err = 0;
    set(&C, 0, 0);
    c_write(0x6f, mr, sizeof mr, 1);
    if (iec_err) {
        return 0;
    }
    n = c_read_channel(15, b, 1, &eoi);
    cvalid[15] = 0;
    if (n < 1) {
        return 0;
    }
    if (verbose) {
        fprintf(stderr, "relay: AR6 Erkennung: $FFFE = $%02X\n", b[0]);
    }
    if (b[0] == 0x03) {
        return 1581;
    }
    if (b[0] != 0x67) {
        return 0;
    }
    /* 1541 oder 1570/1571: $FF79 ist $AA bzw. $E6 */
    {
        static const unsigned char mr2[] = { 'M', '-', 'R', 0x79, 0xff, 1 };
        c_write(0x6f, mr2, sizeof mr2, 1);
        n = c_read_channel(15, b, 1, &eoi);
        cvalid[15] = 0;
        if (verbose) {
            fprintf(stderr, "relay: AR6 Erkennung: $FF79 = $%02X\n", n ? b[0] : 0);
        }
        return n && b[0] == 0xe6 ? 1571 : 1541;
    }
}

/* 1581 (Laufwerkscode $99B0 -> $0500, M-E $0500): Datei normal oeffnen und
   die Ladeadresse lesen, damit das DOS den ersten Block im Puffer hat; dann
   je Block Anzahl + Daten (ganzer Block ab Byte 2), Anzahl 0 = Ende. Das
   Laufwerk kehrt selbst ins DOS zurueck. Zeitbezug wie bei der 1541, die
   Paare kommen aber W+6/+14/+22/+30 (2 MHz, gleichmaessig 8 us). */
static long ar_load1581(const unsigned char *nm, size_t nn, unsigned char *out, size_t max)
{
    static const unsigned char me[] = { 'M', '-', 'E', 0x00, 0x05 };
    unsigned char hd[4];
    int eoi = 0, c, k, i;
    long n = 0;
    uint64_t t0 = t;

    iec_err = 0;
    c_write(0xf0, nm, nn, 1);
    if (c_read_channel(0, hd, 2, &eoi) < 2 || eoi) {
        c_listen(0xe0);
        c_unlisten();
        return -1;                      /* fehlt oder winzig: normal */
    }
    ar_mw(0x0500, AR(0x99b0), 315);
    c_write(0x6f, me, sizeof me, 1);
    if (iec_err) {
        c_listen(0xe0);
        c_unlisten();
        return -1;
    }
    set(&C, DATA_C, 0);
    ar_p0 = 10;
    ar_first = 1;
    ar_fallback = 0;
    for (;;) {
        c = ar_get(3000000);
        if (c < 0) {
            fprintf(stderr, "relay: AR6 1581: keine Blocklaenge\n");
            n = -1;
            break;
        }
        if (c == 0) {
            break;
        }
        for (i = 0; i < c; i++) {
            k = ar_get(200000);
            if (k < 0 || (size_t)n >= max) {
                fprintf(stderr, "relay: AR6 1581: Abbruch bei Byte %ld\n", n);
                n = -1;
                break;
            }
            out[n++] = (unsigned char)k;
        }
        if (n < 0) {
            break;
        }
    }
    ar_p0 = 14;
    set(&C, 0, 0);
    ar_at(t + US(2000));
    c_listen(0xe0);
        c_unlisten();
    if (n >= 0) {
        fprintf(stderr, "relay %10.3f ms: AR6 1581: %ld Bytes in %.3f s (%.0f B/s), %u ohne Flanke\n", (double)t / 1e9,
                n, (double)(t - t0) / 1e12, n / ((double)(t - t0) / 1e12), ar_fallback);
    }
    return n;
}

/* ============ Fast Serial / Burst (1571 im 1571-Modus, 1581, -B) ==========
   VICE bildet Fast Serial byteweise nach; ueber simlink schaltet jedes Byte
   SRQ um und steht in den Daten. Der Rechner kuendigt sich vor jedem ATN mit
   einem Byte $FF an (C128-Kernal $E362), daran erkennt das Laufwerk einen
   schnellen Rechner. Burst-Fastload (U0 + $1F + Name, C128-Kernal $F3EE):
   vor jedem Byte schaltet der Rechner CLK um, das Laufwerk antwortet mit
   einem Byte. Je Block ein Status: 0/1 = voller Block (254 Bytes, im ersten
   Block Ladeadresse + 252), $1F = letzter Block, dann Anzahl, 2 = Datei
   fehlt, sonst Fehler. */

static int c_fast = 0;              /* vor ATN Fast-Byte senden */
static int burst = 0;               /* -B: 1571 per Burst statt AR6 (1541-Modus) */
static int ar81 = 0;                /* -8: 1581 per AR6 statt Burst (Versuch) */
static uint8_t fs_seen = 0;

static void c_fast_send(uint8_t b)
{
    set(&C, (uint8_t)(C.pull ^ SL_IEC_SRQ), b);
}

static int fs_get(unsigned timeout_us)
{
    uint8_t d = 0, s;
    uint64_t dl = t + US(timeout_us);
    s = simlink_read_others(C.sl, C.slot, t + C.off, &d) & SL_IEC_SRQ;
    while (s == fs_seen) {
        if (t >= dl) {
            return -1;
        }
        wait_change_raw(&C, SL_IEC_SRQ, (unsigned)((dl - t) / STEP) + 1);
        s = simlink_read_others(C.sl, C.slot, t + C.off, &d) & SL_IEC_SRQ;
    }
    fs_seen = s;
    return d;
}

static void fs_toggle(void)
{
    if (C.pull & CLK_C) {
        c_off(CLK_C);
    } else {
        c_on(CLK_C);
    }
}

/* n Bytes je mit CLK-Umschalten davor holen (wie $F4C5: lesen, umschalten) */
static int fs_bytes(unsigned char *out, long *n, size_t max, int cnt)
{
    int i, b;
    for (i = 0; i < cnt; i++) {
        b = fs_get(200000);
        if (b < 0 || (size_t)*n >= max) {
            return -1;
        }
        out[(*n)++] = (unsigned char)b;
        fs_toggle();
    }
    return 0;
}

static long fs_load(int ty, const unsigned char *nm, size_t nn, unsigned char *out, size_t max)
{
    unsigned char cmd[300];
    long n = 0;
    int st, c;
    uint64_t t0 = t;
    uint8_t d;

    if (ty == 1571) {
        c_write(0x6f, (const unsigned char *)"U0>M1", 5, 1);
        wait_us(&C, 20000);
    }
    c_fast = 1;
    cmd[0] = 'U'; cmd[1] = '0'; cmd[2] = 0x1f;
    memcpy(cmd + 3, nm, nn);
    iec_err = 0;
    c_write(0x6f, cmd, nn + 3, 1);
    c_fast = 0;     /* nur dieses ATN; sonst antwortet das Laufwerk auch auf TALK schnell */
    if (iec_err) {
        return -1;
    }
    fs_seen = simlink_read_others(C.sl, C.slot, t + C.off, &d) & SL_IEC_SRQ;
    c_off(CLK_C | DATA_C);
    fs_toggle();
    st = fs_get(3000000);
    if (verbose) {
        fprintf(stderr, "relay: Burst: erster Status $%02X\n", st & 0xff);
    }
    if (st == 2) {
        c_off(CLK_C);
        return 0;
    }
    if (st < 0 || (st > 2 && st != 0x1f)) {
        c_off(CLK_C);
        return -1;
    }
    c = -1;
    if (st == 0x1f) {
        fs_toggle();
        c = fs_get(200000);
    }
    fs_toggle();
    if (fs_bytes(out, &n, max, 1) < 0 || fs_bytes(out, &n, max, 1) < 0) {
        c_off(CLK_C);
        return -1;
    }
    if (c >= 0) {
        /* einziger Block: die Anzahl zaehlt ohne Ladeadresse (pruefen) */
        if (fs_bytes(out, &n, max, c) < 0) {
            c_off(CLK_C);
            return -1;
        }
    } else {
        if (fs_bytes(out, &n, max, 252) < 0) {
            c_off(CLK_C);
            return -1;
        }
        for (;;) {
            st = fs_get(3000000);
            if (st < 0) {
                c_off(CLK_C);
                return -1;
            }
            if (st < 2) {
                fs_toggle();
                if (fs_bytes(out, &n, max, 254) < 0) {
                    c_off(CLK_C);
                    return -1;
                }
                continue;
            }
            if (st != 0x1f) {
                fprintf(stderr, "relay: Burst: Status $%02X\n", st);
                c_off(CLK_C);
                return -1;
            }
            fs_toggle();
            c = fs_get(200000);
            fs_toggle();
            if (c < 0 || fs_bytes(out, &n, max, c) < 0) {
                c_off(CLK_C);
                return -1;
            }
            break;
        }
    }
    c_off(CLK_C);
    ar_at(t + US(1000));
    fprintf(stderr, "relay %10.3f ms: Burst %d: %ld Bytes in %.3f s (%.0f B/s)\n", (double)t / 1e9,
            ty, n, (double)(t - t0) / 1e12, n / ((double)(t - t0) / 1e12));
    return n;
}

/* Datei per AR6 laden; Laenge, 0 wenn nicht gefunden, -1 bei Fehler */
static long ar_loadfile(const unsigned char *nm, size_t nn, unsigned char *out, size_t max)
{
    const unsigned char *sec, *p = nm;
    int tr = 18, s = 1, hops = 0, i, ft = -1, fs = 0;
    long n = 0;
    uint64_t t0 = t;

    if (nn >= 2 && p[1] == ':') {
        p += 2; nn -= 2;
    } else if (nn >= 1 && p[0] == ':') {
        p++; nn--;
    }
    for (i = 0; i < (int)nn; i++) {
        if (p[i] == ',') {
            nn = (size_t)i;
        }
    }
    memset(ar_have, 0, sizeof ar_have);
    if (ar_upload() < 0) {
        return -1;
    }
    /* Verzeichnis */
    while (tr && ft < 0 && hops++ < 20) {
        if (!(sec = ar_sector(tr, s))) {
            ar_exit(0);
            return -1;
        }
        for (i = 0; i < 8; i++) {
            const unsigned char *e = sec + 32 * i;
            if ((e[2] & 0x87) == 0x82 && ar_match(p, nn, e + 5)) {
                ft = e[3]; fs = e[4];
                break;
            }
        }
        tr = sec[0]; s = sec[1];
    }
    if (ft < 0) {
        ar_exit(0x62);
        return 0;
    }
    /* Kette */
    tr = ft; s = fs; hops = 0;
    while (tr && hops++ < 800) {
        int k;
        if (!(sec = ar_sector(tr, s))) {
            ar_exit(0);
            return -1;
        }
        k = sec[0] ? 254 : (sec[1] >= 1 ? sec[1] - 1 : 0);
        if ((size_t)(n + k) > max) {
            ar_exit(0);
            return -1;
        }
        memcpy(out + n, sec + 2, (size_t)k);
        n += k;
        tr = sec[0]; s = sec[1];
    }
    ar_exit(0);
    fprintf(stderr, "relay %10.3f ms: AR6: %ld Bytes in %.3f s (%.0f B/s)\n", (double)t / 1e9,
            n, (double)(t - t0) / 1e12, n / ((double)(t - t0) / 1e12));
    return n;
}

/* Fehlerkanal holen und fuer den PET bereitlegen */
static void ar_errchan(void)
{
    int eoi = 0;
    if (!cbuf[15]) {
        cbuf[15] = malloc(BUFMAX);
    }
    set(&C, 0, 0);
    clen[15] = c_read_channel(15, cbuf[15], BUFMAX, &eoi);
    cpos[15] = 0;
    cvalid[15] = 1;
}

/* Datei per AR6 speichern (L8D31-L8DC3, $BF = $FE): Datei normal oeffnen
   (Sekundaeradresse 1) und zwei Bytes schreiben, damit das DOS Eintrag und
   ersten Block anlegt; dann Laufwerkscode, Blockzahl, je Block 256 Bytes:
   [verbleibende Bloecke, Endmarke, 254 Bytes Datei]. Das Laufwerk schreibt
   die Kette selbst und schliesst die Datei. 0 ok, 1 Fehler gemeldet
   (Fehlerkanal liegt bereit), -1 AR6 nicht moeglich (normal speichern) */
static int ar_savefile(const unsigned char *nm, size_t nn, const unsigned char *d, size_t len)
{
    unsigned char cmd[260], blk[256];
    size_t n, i, r;
    int k;
    uint64_t t0 = t;

    if (len < 2 || ar_detect() != 1541) {
        return -1;
    }
    n = (len + 253) / 254;
    if (n > 255) {
        return -1;
    }
    if (nn && nm[0] == '@') {               /* ersetzen: erst loeschen (L8D36) */
        cmd[0] = 'S';
        memcpy(cmd + 1, nm + 1, nn - 1);
        c_write(0x6f, cmd, nn, 1);
        nm++;
        nn--;
    }
    iec_err = 0;
    c_write(0xf1, nm, nn, 1);
    c_write(0x61, d, 2, 1);
    ar_errchan();
    if (iec_err || clen[15] < 2 || cbuf[15][0] != '0' || cbuf[15][1] != '0') {
        fprintf(stderr, "relay: AR6 SAVE: Oeffnen gescheitert: %.*s\n",
                (int)(clen[15] > 2 ? clen[15] - 1 : 0), cbuf[15]);
        return 1;
    }
    if (ar_upload_x(0x8e65, 2, 0x044f) < 0) {
        return -1;
    }
    ar_send((uint8_t)n);
    ar_at(t + US(1300));                    /* L8D90: CLK frei = Diskette voll */
    if (!(C.in & CLK_C)) {
        ar_loaded = 0;
        set(&C, 0, 0);
        ar_at(t + US(5000));
        ar_errchan();
        fprintf(stderr, "relay: AR6 SAVE: Laufwerk lehnt ab: %.*s\n",
                (int)(clen[15] > 2 ? clen[15] - 1 : 0), cbuf[15]);
        return 1;
    }
    r = len - 254 * (n - 1);
    for (i = 0; i < n; i++) {
        size_t m = i + 1 < n ? 254 : r;
        memset(blk, 0, sizeof blk);
        blk[0] = (unsigned char)(n - 1 - i);
        blk[1] = (unsigned char)(r + 2);
        memcpy(blk + 2, d + 254 * i, m);
        if (ar_ready(3000000) < 0) {
            fprintf(stderr, "relay: AR6 SAVE: Laufwerk nicht bereit fuer Block %lu\n", (unsigned long)i);
            ar_loaded = 0;
            set(&C, 0, 0);
            ar_errchan();
            return 1;
        }
        for (k = 0; k < 256; k++) {
            ar_put(blk[k]);
            ar_at(t + US(ar_gap));
        }
        if (verbose > 1) {
            fprintf(stderr, "relay %10.3f ms: AR6 SAVE Block %lu\n", (double)t / 1e9, (unsigned long)i);
        }
    }
    /* Laufwerk schreibt den Rest, BAM und Verzeichnis, dann zurueck ins DOS */
    ar_until(CLK_C, 0, 3000000);
    ar_at(t + US(1000));
    set(&C, 0, 0);
    ar_loaded = 0;
    ar_at(t + US(1000));
    fprintf(stderr, "relay %10.3f ms: AR6 SAVE: %lu Bytes in %.3f s (%.0f B/s)\n", (double)t / 1e9,
            (unsigned long)len, (double)(t - t0) / 1e12, len / ((double)(t - t0) / 1e12));
    cvalid[15] = 0;
    return 0;
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
    if (sa == 0 && ar_pending) {
        uint64_t t0 = t;
        int ty = ar_detect();
        long n = -1;
        if ((burst && ty == 1571) || (ty == 1581 && !ar81)) {
            n = fs_load(ty, ar_name, ar_namelen, cbuf[0], BUFMAX);
        } else if (ty == 1541 || ty == 1571) {
            if (ty == 1571) {
                c_write(0x6f, (const unsigned char *)"U0>M0", 5, 1);   /* 1541-Modus */
            }
            n = ar_loadfile(ar_name, ar_namelen, cbuf[0], BUFMAX);
        } else if (ty == 1581) {
            n = ar_load1581(ar_name, ar_namelen, cbuf[0], BUFMAX);
        }
        ar_pending = 0;
        t_back += t - t0;
        if (n >= 0) {
            clen[0] = (size_t)n;
            cpos[0] = 0;
            cvalid[0] = 1;
            ceof[0] = 1;
            ctotal[0] = (unsigned long)n;
            cvalid[15] = 0;
            if (getenv("AR_DUMP") && n > 0) {
                FILE *df = fopen(getenv("AR_DUMP"), "wb");
                if (df) {
                    fwrite(cbuf[0], 1, (size_t)n, df);
                    fclose(df);
                }
            }
            fprintf(stderr, "relay %10.3f ms: Kanal 0 (LOAD) per AR6 geholt: %ld Bytes\n",
                    (double)t / 1e9, n);
            return;
        }
        fprintf(stderr, "relay: AR6 gescheitert, lade normal\n");
        set(&C, 0, 0);
        be_write(0xf0, ar_name, ar_namelen, 1);
        iec_err = 0;
    }
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
    if (opening == 0 && ar_rom && !reverse && namelen && name[0] != '$') {
        log_msg("OPEN %s (Kanal %ld) per AR6", "", 0);
        memcpy(ar_name, name, namelen);
        ar_namelen = namelen;
        ar_pending = 1;
        cvalid[0] = 0;
        cvalid[15] = 0;
        opening = -1;
    } else if (opening == 1 && ar_rom && !reverse && namelen) {
        log_msg("OPEN %s (Kanal %ld) per AR6", "", 1);
        memcpy(ar_name, name, namelen);
        ar_namelen = namelen;
        ar_save = 1;
        ar_slen = 0;
        opening = -1;
    } else if (opening >= 0) {
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

    while ((c = getopt(argc, argv, "u:d:w:vrWa:g:B8")) != -1) {
        switch (c) {
        case 'u': ieee_unit = d_unit = atoi(optarg); break;
        case 'd': iec_unit = k_unit = atoi(optarg); break;
        case 'r': reverse = 1; break;
        case 'W': whole_writes = 1; break;
        case 'a': if (ar_load_crt(optarg) < 0) return 1; break;
        case 'g': ar_gap = (unsigned)atoi(optarg); break;
        case 'B': burst = 1; break;
        case '8': ar81 = 1; break;
        case 'w': wait_s = atof(optarg); break;
        case 'v': verbose++; break;
        default: goto usage;
        }
    }
    if (optind + 2 != argc) {
usage:
        fprintf(stderr, "usage: relay [-r] [-a ar6.crt] [-u geraet] [-d laufwerk] [-w s] [-v] ieeebus iecbus\n"
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
                if (sa == 1 && ar_save) {
                    uint64_t t0 = t;
                    int e;
                    ar_save = 0;
                    if (getenv("AR_DUMP")) {
                        char fn[1024];
                        FILE *df;
                        snprintf(fn, sizeof fn, "%s.save", getenv("AR_DUMP"));
                        df = fopen(fn, "wb");
                        if (df) {
                            fwrite(ar_sbuf, 1, ar_slen, df);
                            fclose(df);
                        }
                    }
                    e = ar_savefile(ar_name, ar_namelen, ar_sbuf, ar_slen);
                    if (e < 0) {
                        fprintf(stderr, "relay: AR6 SAVE nicht moeglich, speichere normal\n");
                        set(&C, 0, 0);
                        c_write(0xf1, ar_name, ar_namelen, 1);
                        c_write(0x61, ar_sbuf, ar_slen, 1);
                        be_close(1);
                    }
                    t_back += t - t0;
                    cvalid[15] = e > 0;
                    tick(&I);
                } else {
                    be_close(sa);
                }
                cvalid[sa] = 0;
            }
            continue;
        }
        if (role == LISTENER) {
            b = i_listen_byte(0, &eoi);
            if (b < 0) {
                continue;
            }
            if (opening < 0 && ar_save && sa == 1) {
                if (ar_slen < BUFMAX) {
                    ar_sbuf[ar_slen++] = (unsigned char)b;
                }
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
