#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
/*
 * sltest - zwei Prozesse spielen einen IEEE-488-Dreidraht-Handshake ueber simlink.
 *
 *   sltest talker   <bus> <MHz> <bytes>   legt den Bus an, sendet
 *   sltest listener <bus> <MHz> <bytes>             empfaengt und prueft
 *
 * Jeder Prozess hat seinen eigenen Takt und liest den Bus in jedem Takt
 * (schlimmster Fall). Pro Byte meldet der Talker zusaetzlich einen Impuls
 * (PSTB), um die Impulszaehler zu pruefen.
 */
#include "simlink.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static uint8_t pattern(unsigned i) { return (uint8_t)(i * 7 + 3); }

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static int talker(const char *name, double mhz, unsigned n)
{
    simlink_t *sl = simlink_open(name, SIMLINK_BUS_IEEE, 1, 0xff);
    int slot, state = 0;
    uint64_t t = 0, step = (uint64_t)(1e6 / mhz);
    unsigned i = 0;
    double t0;

    if (!sl) { perror("open"); return 1; }
    slot = simlink_attach(sl, SL_TYPE_TOOL, 1);
    /* auf den Listener warten */
    while (!(simlink_read(sl, slot, 0, NULL) & SL_IEEE_NDAC)) usleep(1000);
    t0 = now_s();
    while (i < n) {
        uint8_t l = simlink_read(sl, slot, t, NULL);
        switch (state) {
        case 0: /* warten bis NRFD high (alle bereit), Daten anlegen */
            if (!(l & SL_IEEE_NRFD)) { simlink_set(sl, slot, t, 0, pattern(i)); state = 1; }
            break;
        case 1: /* Daten stehen einen Takt, dann DAV */
            simlink_set(sl, slot, t, SL_IEEE_DAV, pattern(i)); state = 2;
            break;
        case 2: /* warten bis NDAC high (alle haben uebernommen) */
            if (!(l & SL_IEEE_NDAC)) {
                simlink_set(sl, slot, t, 0, 0);
                simlink_pulse(sl, slot, t, SL_PULSE_PSTB);
                i++; state = 0;
            }
            break;
        }
        t += step;
        if (simlink_progress(sl, slot, t) < 0) { fprintf(stderr, "Fehler gemeldet\n"); return 1; }
    }
    {
        double dt = now_s() - t0;
        printf("talker:   %u Bytes, emuliert %.6f ms, Laufzeit %.3f s, Faktor %.2f, Wartefaelle %llu\n",
               n, t / 1e9, dt, dt / (t / 1e12), (unsigned long long)simlink_waits(sl));
    }
    /* Listener noch zu Ende laufen lassen */
    sleep(1);
    simlink_detach(sl, slot);
    simlink_close(sl, 1);
    return 0;
}

static int listener(const char *name, double mhz, unsigned n)
{
    simlink_t *sl = NULL;
    int slot, state = 0;
    uint64_t t, step = (uint64_t)(1e6 / mhz), cycles = 0;
    unsigned i = 0, errors = 0, pulses = 0;

    while (!(sl = simlink_open(name, SIMLINK_BUS_IEEE, 0, 0))) usleep(1000);
    slot = simlink_attach(sl, SL_TYPE_TOOL, 0);
    t = 0;
    simlink_set(sl, slot, t, SL_IEEE_NRFD | SL_IEEE_NDAC, 0);
    simlink_progress(sl, slot, t);
    while (i < n) {
        uint8_t d, l = simlink_read(sl, slot, t, &d);
        switch (state) {
        case 0: /* bereit: NRFD loslassen, NDAC halten */
            simlink_set(sl, slot, t, SL_IEEE_NDAC, 0); state = 1;
            break;
        case 1: /* warten auf DAV low */
            if (l & SL_IEEE_DAV) {
                if (d != pattern(i)) errors++;
                i++;
                simlink_set(sl, slot, t, SL_IEEE_NRFD, 0); /* uebernommen: NDAC los, NRFD halten */
                state = 2;
            }
            break;
        case 2: /* warten bis DAV wieder high, dann NDAC halten */
            if (!(l & SL_IEEE_DAV)) { simlink_set(sl, slot, t, SL_IEEE_NRFD | SL_IEEE_NDAC, 0); state = 0; }
            break;
        }
        pulses += simlink_pulses(sl, slot, SL_PULSE_PSTB);
        t += step; cycles++;
        if (simlink_progress(sl, slot, t) < 0) { fprintf(stderr, "Fehler gemeldet\n"); return 1; }
        if (cycles > 100000000ull) { fprintf(stderr, "Zeitueberschreitung\n"); return 1; }
    }
    /* letzte Impulse abholen */
    for (int k = 0; k < 64; k++) { t += step; simlink_read(sl, slot, t, NULL); pulses += simlink_pulses(sl, slot, SL_PULSE_PSTB); }
    printf("listener: %u Bytes, Fehler %u, Impulse %u, emuliert %.6f ms, Wartefaelle %llu\n",
           i, errors, pulses, t / 1e9, (unsigned long long)simlink_waits(sl));
    simlink_detach(sl, slot);
    simlink_close(sl, 0);
    return errors != 0 || pulses != n;
}


/* Ringpuffer-Test: der Pulser erzeugt in jedem Takt einen Impuls und wechselt
   die Daten; der Zaehler liest nur alle 'every' Takte. Alle Impulse muessen
   ankommen, auch wenn der Ring (64 Eintraege) dazwischen mehrfach umlaeuft. */
static int pulser(const char *name, unsigned n)
{
    simlink_t *sl = simlink_open(name, SIMLINK_BUS_IEC, 1, 0xff);
    int slot = simlink_attach(sl, SL_TYPE_TOOL, 1);
    uint64_t t = 0;
    while (!(simlink_read(sl, slot, 0, NULL) & SL_IEC_ATN)) usleep(1000);
    for (unsigned i = 0; i < n; i++) {
        simlink_set(sl, slot, t, SL_IEC_CLK, (uint8_t)i);
        simlink_pulse(sl, slot, t, SL_PULSE_SRQ);
        t += 1000000;
        simlink_progress(sl, slot, t);
    }
    sleep(1);
    simlink_detach(sl, slot);
    simlink_close(sl, 1);
    return 0;
}

static int counter(const char *name, unsigned n, unsigned every)
{
    simlink_t *sl = NULL;
    int slot;
    uint64_t t = 0;
    unsigned got = 0, wrong = 0;
    while (!(sl = simlink_open(name, SIMLINK_BUS_IEC, 0, 0))) usleep(1000);
    slot = simlink_attach(sl, SL_TYPE_TOOL, 0);
    simlink_set(sl, slot, 0, SL_IEC_ATN, 0);
    while (got < n) {
        uint8_t d;
        t += (uint64_t)every * 1000000;
        simlink_read(sl, slot, t, &d);
        got += simlink_pulses(sl, slot, SL_PULSE_SRQ);
        /* vor t hat der Pulser t/1us Impulse gesendet, die Daten sind (t/1us - 1) */
        if (got != (t / 1000000 < n ? t / 1000000 : n)) wrong++;
        if (got < n && d != (uint8_t)(t / 1000000 - 1)) wrong++;
    }
    printf("counter:  %u Impulse, Lesen alle %u Takte, Abweichungen %u, Wartefaelle %llu\n",
           got, every, wrong, (unsigned long long)simlink_waits(sl));
    simlink_detach(sl, slot);
    simlink_close(sl, 0);
    return wrong != 0;
}

int main(int argc, char **argv)
{
    if (argc >= 4 && !strcmp(argv[1], "pulser"))
        return pulser(argv[2], (unsigned)atoi(argv[3]));
    if (argc >= 5 && !strcmp(argv[1], "counter"))
        return counter(argv[2], (unsigned)atoi(argv[3]), (unsigned)atoi(argv[4]));
    if (argc >= 5 && !strcmp(argv[1], "talker"))
        return talker(argv[2], atof(argv[3]), (unsigned)atoi(argv[4]));
    if (argc >= 5 && !strcmp(argv[1], "listener"))
        return listener(argv[2], atof(argv[3]), (unsigned)atoi(argv[4]));
    fprintf(stderr, "sltest talker <bus> <MHz> <bytes> | listener <bus> <MHz> <bytes>\n");
    return 2;
}
