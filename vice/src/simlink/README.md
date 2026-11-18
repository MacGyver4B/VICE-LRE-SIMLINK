# simlink

Kopplung von Emulatoren auf Leitungsebene für IEEE-488 und den seriellen IEC-Bus (mit optionalem Parallelkabel). Gedacht für xpet, `xdrive` (VICE-Laufwerke ohne Rechner), picoem (RP2350) und AVR-Simulatoren.

Stand: Version 1, 2026-11-18. Linux/macOS (POSIX `shm_open`) und Windows (`CreateFileMapping`, MinGW).

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
- Linux/macOS: POSIX-Shared-Memory (`/dev/shm/simlink-<name>` unter Linux;
  bleibt nach einem Absturz liegen und wird beim naechsten Anlegen neu
  initialisiert).
- Windows (MinGW, auch GTK-Version): benannte Dateizuordnung
  `Local\simlink-<name>`; verschwindet, wenn der letzte Teilnehmer schliesst.
  Alle Teilnehmer muessen in derselben Benutzersitzung laufen.
- GTK3: Einstellungen → Peripheral devices → External IEEE-488 (simlink).
- Grenze v1: Laufwerke mit True-Drive-Emulation in VICE werden mit dem
  Takt des PET gestempelt (Laufwerke laufen in VICE nachgezogen), nicht
  mit ihrem eigenen.

## xdrive

Eigenes Programm aus denselben Bausteinen wie x64sc (vgl. den Link
x64 -> x64sc). Als `xdrive` gestartet:

- IEC-Bus `xdrive` ist verbunden, der C64 vom IEC-Bus getrennt;
- eigene Einstellungsdatei (`xdriverc`, unter Windows `xdrive.ini`),
  sofern nicht per `-config` vorgegeben;
- sonst ein normaler C64 mit allen Laufwerkstypen und Menues.

```
xdrive -8 disk.d64                 # Bus "xdrive"
xdrive -iecsimlink pet1 -8 a.d64   # anderer Busname
```

## Am IEC-Bus (x64sc)

```
x64sc -drive8type 1541 -8 disk.d64 +busdevice8 -iecsimlink <bus> [-iecsimlinkstep <takte>]
```

- Ein externer Rechner uebernimmt die Rechnerseite des IEC-Busses; der C64
  wird ueber den vorhandenen Schalter (`c64iec_active`) vom Bus getrennt.
- Die Leitungen des externen Rechners gehen ueber den normalen Schreibweg
  (`iecbus_callback_write`) auf den Bus, wie ein Schreibzugriff des C64.
- Abgleich ohne Vorlauf: Laufwerke werden nur so weit nachgezogen, wie der
  Stand des externen Rechners feststeht; jede seiner Aenderungen wird genau
  zu ihrer Zeit eingespeist. Ausgaben der Laufwerke tragen den Zeitstempel
  ihres eigenen Takts.
- Fortschritt t heisst: alle eigenen Eintraege vor t sind endgueltig.
- Haken in VICE: `drive_ext_sync_add()` (vor jedem Nachziehen der Laufwerke,
  auch vom IEEE-Teilnehmer genutzt) und `drive_ext_iec_changed` (Laufwerk
  hat seine IEC-Ausgaenge geaendert).

## Test

Bibliothek allein (zwei Prozesse):

```
cc -O2 -std=c11 -I. -o sltest test/sltest.c simlink.c -lpthread
test/run.sh
```

Windows-Fassung der Tests (Cross-Build, z. B. unter Wine pruefbar):

```
x86_64-w64-mingw32-gcc -O2 -std=c11 -I. -o sltest.exe test/sltest.c simlink.c -static -lpthread
```

Testrechner am IEC-Bus (serielles Commodore-Protokoll auf Leitungsebene):

```
cc -O2 -std=c11 -I. -o iechost test/iechost.c simlink.c -lpthread
./iechost [-w s] [-v] iec1 status load NAME DATEI save DATEI NAME dir DATEI &
x64sc -drive8type 1541 -8 disk.d64 +busdevice8 -iecsimlink iec1
```

Hinweis: Die 1541 fragt CLK unter ATN in Schleifen von rund 50 us ab; der
Testrechner haelt jede Phase deshalb mindestens 60 us. Nach dem Start
wartet er (-w), bis das Laufwerk hochgefahren ist.

Testgeraet am IEEE-Bus (Protokollebene, Dateien im Arbeitsverzeichnis):

```
cc -O2 -std=c11 -I. -o ieeedev test/ieeedev.c simlink.c -lpthread
./ieeedev [-u 8] [-s schritt_ns] [-v] pet1 &
xpet -drive8type 0 +busdevice8 -ieeesimlink pet1
```

Relais zwischen beiden Bussen (Durchreichen auf Protokollebene, ohne
eigenes DOS): Geraet am IEEE-Bus, Rechner am IEC-Bus.

```
cc -O2 -std=c11 -I. -o relay test/relay.c simlink.c -lpthread
./relay [-u ieee_geraet] [-d iec_laufwerk] [-w anlauf_s] [-v] pet1 iec1 &
xdrive -iecsimlink iec1 -8 disk.d64 &
xpet -drive8type 0 +busdevice8 -ieeesimlink pet1
```

- Schreibdaten (OPEN-Name, Befehle, SAVE) nimmt das Relais sofort an und
  reicht sie bei UNLISTEN weiter; dabei haelt es NRFD, der PET wartet ohne
  Zeitgrenze.
- Lesekanaele holt es vor dem Annehmen des Sekundaerbefehls nach TALK
  (NRFD gehalten) von der IEC-Seite, in Portionen bis 254 Bytes (TALK ...
  UNTALK, die Floppy liefert beim naechsten TALK weiter). Keine Daten: es
  schweigt, der PET meldet ?FILE NOT FOUND. Schreibdaten gehen ebenso in
  Portionen weiter. Mit -v steht der Fortschritt im Log, so laesst sich ein
  langer Test frueh abbrechen.
- Nach dem letzten Byte einer Portion haelt es die Quittung (DATA low) noch
  100 us, sonst sendet die 1541 dieses Byte beim naechsten TALK erneut.
- Wartet es am IEEE-Bus, zieht es die IEC-Seite in 1-ms-Schritten mit,
  damit das Laufwerk weiterarbeiten kann (z. B. Verzeichniseintrag nach
  CLOSE).
- xdrive im Test nicht per kill beenden, sondern mit -limitcycles: sonst
  schreibt VICE die letzte Spur nicht zurueck.

Gegenrichtung (`relay -r`): Rechner (C64) am IEC-Bus, Laufwerk am IEEE-Bus.
x64sc bleibt dabei am IEC-Bus; das Relais haengt dort als Geraet
(`-iecsimlinkdevice`). Am IEEE-Bus ist es Controller, z. B. einer 8250 in
xpet, deren PET ruht.

```
./relay -r -v pet1 iec1 &
xpet -drive8type 8250 -drive8truedrive -8 disk.d82 -ieeesimlink pet1 &
x64sc -drive8type 0 +busdevice8 -iecsimlink iec1 -iecsimlinkdevice
```

- Am IEC-Bus haelt das Relais als Sprecher den Start (CLK low) 150 us und
  jede Bitphase mindestens 80 us: ACPTR liest $DD00 doppelt und kann dabei
  von einer Badline (bis 47 Takte) aufgehalten werden; kuerzere Phasen
  verpasst der C64 gelegentlich und zaehlt dann ein Bit zu wenig.
- Unter ATN erst warten, bis der C64 auch CLK gezogen hat (er zieht ATN
  einige us vor CLK), sonst gibt das Relais DATA zu frueh frei ("device not
  present").
- Schreibdaten gehen am IEEE-Bus in EINER Sitzung (LISTEN ... UNLISTEN) wie
  beim PET; ein neues LISTEN je 254 Bytes fiel bei der 8250 auf das Schreiben
  des vollen Blocks, danach gab sie NRFD nicht mehr frei.

Schnelllader am IEC-Bus (`relay -a ar6.crt [-B] [-8]`): LOAD und SAVE im
Vorwaertsbetrieb ueber AR6 (Action Replay 6, Host-Seite nachgebaut) bzw.
Burst-Fastload (C128). Den AR6-Laufwerkscode liest das Relais zur Laufzeit
aus Bank 2 des CRT-Abbilds; er steht nicht im Repo.

- Erkennung bei jedem LOAD/SAVE per M-R: `$FFFE` = `$03` 1581, `$67` dann
  `$FF79` = `$AA` 1541, `$E6` 1570/1571; sonst Standardprotokoll. Scheitert
  der Schnelllader, laedt bzw. speichert das Relais normal.
- 1541: AR6 LOAD (Spuren mit `$E0`, Verzeichnis und Kette im Relais) und
  SAVE (Laufwerk schreibt die Kette selbst). 1571: AR6 im 1541-Modus
  (`U0>M0`) oder mit `-B` Burst im 1571-Modus (`U0>M1`). 1581: Burst; mit
  `-8` versuchsweise AR6 (Laufwerkscode `$99B0`, bricht gelegentlich im
  letzten Block ab, dann laedt das Relais normal).
- Timing Laufwerk -> Rechner: DATA schon freigeben, solange das Laufwerk
  beschaeftigt ist (CLK low); dann erkennt es die Freigabe beim ersten
  Abfragen, die Bitpaare kommen fest nach seiner CLK-Flanke (1541 W+11,
  1581 W+7, je 8 us). Ist das Laufwerk schon bereit, dient die eigene
  Freigabe als Bezug.
- Rechner -> Laufwerk: Paare bei +8/+18/+34/+44 us wie beim C64; die
  Freigabe muss laenger stehen als ein Abfragezyklus des Laufwerks (7 us),
  bei +7 gingen einzelne Bytes verloren.
- Fast Serial: VICE bildet es byteweise nach. Ueber simlink schaltet jedes
  Byte SRQ um, der Wert steht im Datenfeld (iecbus-simlink.c,
  c64fastiec.c). Das Relais sendet das Ankuendigungsbyte `$FF` nur vor dem
  ATN des Burst-Befehls, sonst antwortet das Laufwerk auch auf normales
  TALK schnell.
- `AR_DUMP=datei`: zuletzt geladene bzw. zu speichernde Datei ablegen.

Referenz petSD+ (`test/petsdsim.c`): die unveraenderte NODISKEMU-Firmware
(ATmega1284P, 8 MHz) in simavr am IEEE-Bus, SD-Karte als Image-Datei
(MBR + FAT16, SDHC-Blockadressen). Eingaenge vom Bus wirken 2 us spaeter
(`-l`), damit beide Seiten je Runde um diese Vorausschau vorankommen; das
laeuft schneller als Echtzeit und aendert die gemessenen Zeiten nicht
(0,25 us ergab dasselbe). Ab Werk Geraet 11 (Menuesystem), Adresse per
`U0>`; das EEPROM wird nicht gespeichert.

```
make CONFIG=configs/config-petSD+          # im NODISKEMU-Baum
cc -O2 -std=c11 -I. -I/usr/include/simavr -o petsdsim test/petsdsim.c simlink.c \
   -lsimavr -lelf -lpthread
./petsdsim NODISKEMU.elf sd.img pet1 &
xpet -drive8type 0 +busdevice8 -ieeesimlink pet1
```

- Pins nach `arch-config.h` (HW_PETSDPLUS), Treiber 75160/75161 ueber TE
  (PC3) nachgebildet; Eingaenge als "extern" markiert, sonst legt simavr
  die Pull-ups der Firmware darueber (Karte fehlt, Schreibschutz).
- Tasten (ADC7) im Ruhezustand etwa 550, nicht Vcc (sonst Diagnosemodus).
- VICE meldet Fortschritt done+1, stempelt eine spaetere Aenderung des PET
  aber noch mit done; petsdsim wertet daher nur bis Fortschritt - 2 als
  endgueltig.
- Gemessen am PET 4032: FAT direkt LOAD 2 925 B/s, SAVE 1 955 B/s; D82
  LOAD 1 734 B/s, SAVE 814 B/s; D64 LOAD 1 585 B/s.

Mitschreiber (Diagnose): `buslog [-i] [-o datei] bus` haengt passiv am Bus
und schreibt jeden Wechsel mit emulierter Zeit auf; am Ende Anzahl Wechsel
und kuerzeste Phase je Leitung. Am IEC-Bus mit `x64sc ... -iecsimlinkdevice`.
Bekannt: Transwarp haengt mit angehaengtem buslog im Vorspeeder (offen).

Gesamttest: `test/check.sh [VICE-Baum]` (Bibliothek, IEEE, IEEE gemischt,
IEC, Relais; jeweils zwei Laeufe mit Vergleich). Mit `AR6_CRT=ar6.crt`
zusaetzlich die Schnelllader (AR6 1541 LOAD+SAVE, AR6 und Burst auf der
1571, Burst auf der 1581).

- Relais, Speicherbefehle: M-W/M-R im RAM der 1541 und 16 KB DOS-ROM per
  M-R (128 Bytes je Befehl, GET# je Byte) - ROM-Datei bytegleich, Pruefsumme
  stimmt. Dazu Scratch, Validate, Format (N0:NEU,99) ueber das Relais.
  Derselbe Test direkt (xpet + 2031, x64sc + 1541) liefert ebenfalls das
  jeweilige ROM bytegleich.
- Relais (xpet + relay + xdrive mit 1541): Verzeichnis, LOAD 5013 Bytes,
  SAVE bytegleich, 30 KB LOAD und SAVE in Portionen bytegleich, GET#
  einer Datei bis EOI (ST 64), Fehlerkanal mit
  62,FILE NOT FOUND nach fehlender Datei; zwei Laeufe mit gleichem Ablauf.
- IEEE-Handshake zwischen zwei Prozessen (Talker 1 MHz, Listener 2 MHz), 20 000 Bytes, fünf Läufe: keine Fehler, alle Impulse, emulierte Zeit in jedem Lauf identisch (79,999 ms), etwa 1,7-mal schneller als Echtzeit bei Lesen in jedem Takt.
- Ringpuffer: Impulse in jedem Takt, Leser liest nur alle 1/50/200 Takte – alle 5000 Impulse und alle Datenwerte kommen richtig an.

- xpet (headless, BASIC 4, 4032) mit `ieeedev` (Schritt 0,5 µs):
  `SAVE`/`LOAD` ueber den simlink-Bus, 5013 Bytes hin und zurueck
  bytegleich; zwei Laeufe ergeben eine identische Busspur (69 730 Wechsel).
- Gemischt: 2031 mit True-Drive-Emulation auf 8 und `ieeedev` auf 9
  gleichzeitig: Kopieren 8 → 9 → 8 fehlerfrei.
- Ohne `-ieeesimlink` unveraendert: Dateisystemgeraet und 2031 (TDE)
  laden und speichern wie vorher.
- Windows-Fassung der Bibliothek (MinGW, unter Wine 9.0): Handshake-Test
  ohne Fehler, emulierte Zeit in beiden Laeufen identisch; Ringtest ohne
  Abweichungen. Unter Wine deutlich langsamer als nativ unter Linux.
- IEC: x64sc mit 1541 (True-Drive-Emulation, D64) und `iechost`: Status,
  LOAD (17 und 5013 Bytes), Verzeichnis, SAVE und erneutes LOAD bytegleich,
  rund 400 Bytes/s wie beim echten Geraet; zwei Laeufe mit identischer
  Busspur.
