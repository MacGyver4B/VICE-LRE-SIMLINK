# simlink

Kopplung von Emulatoren auf Leitungsebene für IEEE-488 und den seriellen IEC-Bus (mit optionalem Parallelkabel). Gedacht für xpet, `xdrive` (VICE-Laufwerke ohne Rechner), picoem (RP2350) und AVR-Simulatoren.

Stand: Version 1, 2026-11-18. Nur Linux/POSIX (Shared Memory über `shm_open`); Windows folgt.

## Prinzip

- Ein Shared-Memory-Block pro Bus, bis zu 8 Teilnehmer. Jeder schreibt nur seinen eigenen Bereich.
- Jeder Teilnehmer schreibt seine Leitungsänderungen mit Zeitstempel (Pikosekunden) in einen Ringpuffer (64 Einträge) und meldet laufend seinen Fortschritt.
- **Gewartet wird nur beim Lesen:** `simlink_read(t)` wartet, bis alle anderen `t` erreicht haben, und liefert deren Zustand *vor* `t`. Zwischen zwei Buszugriffen rechnet jeder frei – wie VICE seine Laufwerke nur beim Buszugriff nachholt.
- Dadurch ist jeder Lauf **exakt und reproduzierbar**, unabhängig davon, wie das Betriebssystem die Prozesse plant. Ein einstellbarer „Vorsprung“ ist nicht nötig.
- Eine Änderung zum Zeitpunkt `t` sieht ein anderer Teilnehmer beim ersten Lesen nach `t`.
- Impulse (PC2, CA2→FLAG, SRQ-Takt) werden gezählt, nicht als Pegel übertragen; kein Impuls geht verloren, auch wenn selten gelesen wird.
- Leitungen aktiv low, Open Collector: Bit = 1 heißt „zieht auf low“; Buspegel = ODER über alle Teilnehmer.

## Bitbelegung `pull`

| Bit | IEEE-488 | IEC |
| --- | --- | --- |
| 0 | ATN | ATN |
| 1 | DAV | CLK |
| 2 | NRFD | DATA |
| 3 | NDAC | SRQ (Burst-Takt) |
| 4 | EOI | RESET |
| 5 | SRQ | PSTB (Parallelkabel PC2 → CB1) |
| 6 | IFC | PACK (Parallelkabel CA2 → FLAG2) |
| 7 | REN | – |

`data`: IEEE DIO1–8; IEC Datenbyte des Parallelkabels.

## API (Kurzfassung)

```c
simlink_t *sl = simlink_open("petsd2", SIMLINK_BUS_IEEE, create, line_mask);
int slot = simlink_attach(sl, SL_TYPE_VICE, clockmaster);
simlink_set(sl, slot, t_ps, pull, data);     /* bei Änderung */
simlink_pulse(sl, slot, t_ps, SL_PULSE_PSTB);/* Impuls */
simlink_progress(sl, slot, t_ps);            /* Fortschritt, wartet nicht */
uint8_t lines = simlink_read(sl, slot, t_ps, &data); /* wartet bis alle t erreicht haben */
unsigned n = simlink_pulses(sl, slot, SL_PULSE_PSTB);
```

uint64_t t0 = simlink_start_time(sl, slot);  /* Zeitbasis fuer Spaeteinsteiger */
int n = simlink_others(sl, slot);              /* andere aktive Teilnehmer */
uint8_t ext = simlink_read_others(sl, slot, t_ps, &data); /* nur fremde Leitungen */
```

Zeitstempel und Fortschritt laufen nur vorwaerts. Ein Teilnehmer, der spaeter
dazukommt, beginnt bei der Zeit des langsamsten aktiven Teilnehmers
(`simlink_start_time`) und rechnet seine eigene Zeit darauf um.

## In xpet

```
xpet -ieeesimlink <bus> [-ieeesimlinkstep <takte>]
```

- Ressourcen `IEEESimlink` (Busname, leer = aus) und `IEEESimlinkStep`
  (Abgleich spaetestens alle n Takte, Standard 64).
- Der externe Teilnehmer erscheint im IEEE-Code unter dem Teilnehmerbit
  `PARALLEL_EXT` (0x40) neben PET und Laufwerken (`src/parallel/parallel-simlink.c`).
- PET-Seite wie am echten PET: REN dauernd aktiv, IFC waehrend des Resets,
  SRQ des externen Teilnehmers auf PIA2 CB1.
- Gelesen wird vor jedem Zugriff des PET auf die IEEE-Leitungen
  (petvia.c, petpia1.c, petpia2.c); veroeffentlicht wird bei jeder Aenderung
  durch PET oder Laufwerk.
- Fuer die extern bediente Geraeteadresse die eingebaute Geraeteemulation
  abschalten (`+busdevice<n>`, kein Laufwerk auf dieser Adresse).
- Nur POSIX (Linux, macOS); unter Windows ist die Funktion vorerst leer.
- Grenze v1: Laufwerke mit True-Drive-Emulation in VICE werden mit dem
  Takt des PET gestempelt (Laufwerke laufen in VICE nachgezogen), nicht
  mit ihrem eigenen.

## Test

Bibliothek allein (zwei Prozesse):

```
cc -O2 -std=c11 -I. -o sltest test/sltest.c simlink.c -lpthread
test/run.sh
```

Testgeraet am IEEE-Bus (Protokollebene, Dateien im Arbeitsverzeichnis):

```
cc -O2 -std=c11 -I. -o ieeedev test/ieeedev.c simlink.c -lpthread
./ieeedev [-u 8] [-s schritt_ns] [-v] pet1 &
xpet -drive8type 0 +busdevice8 -ieeesimlink pet1
```

- IEEE-Handshake zwischen zwei Prozessen (Talker 1 MHz, Listener 2 MHz), 20 000 Bytes, fünf Läufe: keine Fehler, alle Impulse, emulierte Zeit in jedem Lauf identisch (79,999 ms), etwa 1,7-mal schneller als Echtzeit bei Lesen in jedem Takt.
- Ringpuffer: Impulse in jedem Takt, Leser liest nur alle 1/50/200 Takte – alle 5000 Impulse und alle Datenwerte kommen richtig an.

- xpet (headless, BASIC 4, 4032) mit `ieeedev` (Schritt 0,5 µs):
  `SAVE`/`LOAD` ueber den simlink-Bus, 5013 Bytes hin und zurueck
  bytegleich; zwei Laeufe ergeben eine identische Busspur (69 730 Wechsel).
- Gemischt: 2031 mit True-Drive-Emulation auf 8 und `ieeedev` auf 9
  gleichzeitig: Kopieren 8 → 9 → 8 fehlerfrei.
- Ohne `-ieeesimlink` unveraendert: Dateisystemgeraet und 2031 (TDE)
  laden und speichern wie vorher.
