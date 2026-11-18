/*
 * buslog - passiver Mitschreiber am simlink-Bus (Diagnose).
 *
 * Haengt als Teilnehmer am Bus, zieht nie eine Leitung und schreibt jeden
 * Wechsel mit emulierter Zeit auf. Am Ende eine Zusammenfassung: Anzahl
 * Wechsel je Leitung und kuerzeste Low- bzw. High-Phase.
 *
 *   buslog [-i] [-o datei] bus
 *     -i   IEEE-Bus (Standard: IEC)
 *     -o   Wechsel als Text in diese Datei (Zeit in us, Leitungen, Daten)
 *
 * IEC mit x64sc: x64sc ... -iecsimlink bus -iecsimlinkdevice
 * (der C64 bleibt am Bus, buslog haengt daneben wie ein Geraet ohne Funktion)
 */
#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "simlink.h"

static volatile sig_atomic_t stop = 0;
static void on_sig(int s) { (void)s; stop = 1; }

int main(int argc, char **argv)
{
    int c, ieee = 0, slot, i;
    const char *out = NULL;
    FILE *f = NULL;
    simlink_t *sl;
    uint64_t t = 1000000, tc, last[8] = { 0 }, first_t = 0, last_t = 0;
    uint64_t minlow[8], minhigh[8];
    unsigned long n[8] = { 0 }, total = 0;
    uint8_t prev = 0, d, pd = 0, mask;
    const char *names_iec[8] = { "ATN", "CLK", "DATA", "SRQ", "b4", "b5", "b6", "RESET" };
    const char *names_ieee[8] = { "ATN", "DAV", "NRFD", "NDAC", "EOI", "SRQ", "IFC", "REN" };

    while ((c = getopt(argc, argv, "io:")) != -1) {
        switch (c) {
        case 'i': ieee = 1; break;
        case 'o': out = optarg; break;
        default: goto usage;
        }
    }
    if (optind >= argc) {
usage:
        fprintf(stderr, "usage: buslog [-i] [-o datei] bus\n");
        return 2;
    }
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    for (i = 0; i < 8; i++) {
        minlow[i] = minhigh[i] = UINT64_MAX;
    }
    sl = simlink_open(argv[optind], ieee ? SIMLINK_BUS_IEEE : SIMLINK_BUS_IEC, 0, 0xff);
    if (!sl) {
        sl = simlink_open(argv[optind], ieee ? SIMLINK_BUS_IEEE : SIMLINK_BUS_IEC, 1, 0xff);
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
    if (out) {
        f = fopen(out, "w");
        if (!f) {
            perror(out);
            return 1;
        }
        fprintf(f, "# Zeit_us Leitungen(hex, 1 = gezogen) Daten(hex)\n");
    }
    mask = ieee ? 0xff : (SL_IEC_ATN | SL_IEC_CLK | SL_IEC_DATA);
    for (i = 0; i < 3000 && !simlink_others(sl, slot) && !stop; i++) {
        usleep(10000);
    }
    /* spaet angehaengt: ab der eigenen Startzeit (= Stand der anderen) */
    t = simlink_start_time(sl, slot) + 1;
    while (!stop && simlink_others(sl, slot)) {
        uint8_t l;
        /* bis zur naechsten Aenderung vorausschauend warten; nie eigene Leitungen */
        if (!simlink_wait_change(sl, slot, t, mask, 1000000, 0, &tc)) {
            int e = 0;
            uint64_t pp = simlink_peers_progress(sl, slot, &e);
            fprintf(stderr, "buslog: Warten endete ohne Aenderung bei %.3f s (andere %d, Fortschritt %.3f s, Fehler %d)\n",
                    t / 1e12, simlink_others(sl, slot), pp / 1e12, e);
            simlink_wait_end(sl, slot, t);
            if (simlink_others(sl, slot)) {
                continue;
            }
            break;
        }
        t = tc + 1;
        simlink_wait_end(sl, slot, t);
        l = simlink_read_others(sl, slot, t, &d) & mask;
        if (l == prev && (!ieee || d == pd)) {
            continue;
        }
        if (!first_t) {
            first_t = tc;
        }
        last_t = tc;
        total++;
        for (i = 0; i < 8; i++) {
            uint8_t m = (uint8_t)(1u << i);
            if ((l ^ prev) & m) {
                if (last[i]) {
                    uint64_t w = tc - last[i];
                    if (prev & m) {
                        if (w < minlow[i]) {
                            minlow[i] = w;
                        }
                    } else if (w < minhigh[i]) {
                        minhigh[i] = w;
                    }
                }
                last[i] = tc;
                n[i]++;
            }
        }
        if (f) {
            fprintf(f, "%.3f %02x %02x\n", tc / 1e6, l, d);
        }
        prev = l;
        pd = d;
    }
    if (f) {
        fclose(f);
    }
    printf("buslog: %lu Wechsel von %.3f s bis %.3f s\n", total, first_t / 1e12, last_t / 1e12);
    for (i = 0; i < 8; i++) {
        if (!n[i]) {
            continue;
        }
        printf("  %-5s %8lu Wechsel, kuerzeste Low-Phase %8.3f us, kuerzeste High-Phase %8.3f us\n",
               ieee ? names_ieee[i] : names_iec[i], n[i],
               minlow[i] == UINT64_MAX ? 0.0 : minlow[i] / 1e6,
               minhigh[i] == UINT64_MAX ? 0.0 : minhigh[i] / 1e6);
    }
    simlink_detach(sl, slot);
    simlink_close(sl, 0);
    return 0;
}
