/*
 * simlink - Kopplung von Emulatoren auf Leitungsebene (IEEE-488, IEC)
 *
 * Gemeinsamer Speicher (Shared Memory), ein Block pro Bus. Jeder Teilnehmer
 * schreibt nur seinen eigenen Bereich:
 *   - time_ps: bis hierhin hat er gerechnet (Zusage: keine Aenderung vor time_ps)
 *   - ein Ringpuffer seiner Leitungsaenderungen mit Zeitstempel
 *
 * Lesen zum Zeitpunkt T wartet, bis alle anderen aktiven Teilnehmer T erreicht
 * haben, und liefert deren Zustand vor T (Zeitstempel < T). Damit ist jeder
 * Lauf exakt und reproduzierbar, unabhaengig von der Prozessplanung. Eine
 * Aenderung zum Zeitpunkt t wird von anderen ab dem ersten Lesen nach t gesehen.
 *
 * Alle Leitungen aktiv low / Open Collector: Bit = 1 heisst
 * "dieser Teilnehmer zieht die Leitung auf low".
 *
 * Version 1, Stand 2026-11-18.
 */
#ifndef SIMLINK_H
#define SIMLINK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIMLINK_VERSION      1
#define SIMLINK_MAX_PARTS    8
#define SIMLINK_RING         64      /* Aenderungen je Teilnehmer im Ringpuffer */

/* Bustypen */
#define SIMLINK_BUS_IEEE     0
#define SIMLINK_BUS_IEC      1

/* pull, IEEE-488 */
#define SL_IEEE_ATN   0x01
#define SL_IEEE_DAV   0x02
#define SL_IEEE_NRFD  0x04
#define SL_IEEE_NDAC  0x08
#define SL_IEEE_EOI   0x10
#define SL_IEEE_SRQ   0x20
#define SL_IEEE_IFC   0x40
#define SL_IEEE_REN   0x80

/* pull, IEC (data = Datenbyte des Parallelkabels) */
#define SL_IEC_ATN    0x01
#define SL_IEC_CLK    0x02
#define SL_IEC_DATA   0x04
#define SL_IEC_SRQ    0x08
#define SL_IEC_RESET  0x10
#define SL_IEC_PSTB   0x20   /* Parallelkabel Rechner -> Laufwerk (PC2 -> CB1)  */
#define SL_IEC_PACK   0x40   /* Parallelkabel Laufwerk -> Rechner (CA2 -> FLAG2) */

/* flags */
#define SL_F_ACTIVE      0x01
#define SL_F_CLOCKMASTER 0x02
#define SL_F_RESET       0x04
#define SL_F_DETACH      0x08
#define SL_F_ERROR       0x10

/* Impulsleitungen (Zaehler je Leitung) */
#define SL_PULSE_PSTB 0
#define SL_PULSE_PACK 1
#define SL_PULSE_SRQ  2
#define SL_PULSE_NUM  3

/* Teilnehmerarten */
#define SL_TYPE_TOOL   0
#define SL_TYPE_VICE   1
#define SL_TYPE_PICOEM 2
#define SL_TYPE_AVR    3

typedef struct simlink_s simlink_t;

/* Bus oeffnen. create != 0 legt ihn an, sonst wird ein vorhandener geoeffnet. */
simlink_t *simlink_open(const char *name, int bus, int create, uint8_t line_mask);
void       simlink_close(simlink_t *sl, int unlink_bus);

/* Anmelden: liefert die Teilnehmernummer (0..7) oder -1. */
int  simlink_attach(simlink_t *sl, uint8_t type, int clockmaster);
void simlink_detach(simlink_t *sl, int slot);

/* Eigene Leitungen ab Zeitpunkt t setzen (nur bei Aenderung noetig). */
void simlink_set(simlink_t *sl, int slot, uint64_t t_ps, uint8_t pull, uint8_t data);
/* Impuls auf einer Impulsleitung zum Zeitpunkt t melden. */
void simlink_pulse(simlink_t *sl, int slot, uint64_t t_ps, int line);

/* Fortschritt melden: bis t_ps gerechnet, keine Aenderung mehr vor t_ps.
   Wartet nicht. -1, wenn ein Teilnehmer ERROR meldet. */
int simlink_progress(simlink_t *sl, int slot, uint64_t t_ps);

/* Buspegel zum Zeitpunkt t lesen: wartet, bis alle anderen t erreicht haben.
   Ergebnis: ODER ueber eigenen aktuellen pull und den pull der anderen vor t,
   unter line_mask. *data entsprechend (falls nicht NULL). */
uint8_t simlink_read(simlink_t *sl, int slot, uint64_t t_ps, uint8_t *data);

/* Wie simlink_read, liefert aber nur die Leitungen der anderen Teilnehmer
   (ohne den eigenen Beitrag). Fuer Emulatoren, die die Fremdleitungen getrennt
   einspeisen (VICE: PARALLEL_EXT). */
uint8_t simlink_read_others(simlink_t *sl, int slot, uint64_t t_ps, uint8_t *data);

/* Neue Impulse der anderen vor t seit dem letzten Aufruf (nach simlink_read). */
unsigned simlink_pulses(simlink_t *sl, int slot, int line);

/* Statistik: Anzahl Lesevorgaenge, die warten mussten */
uint64_t simlink_waits(simlink_t *sl);

/* Startzeit eines Teilnehmers (nach attach: Zeit des langsamsten anderen).
   Eigene Zeitstempel muessen >= diesem Wert sein. */
uint64_t simlink_start_time(simlink_t *sl, int slot);

/* --- Bausteine fuer Teilnehmer, die selbst schrittweise nachziehen
       (z. B. VICE-Laufwerke am IEC-Bus) --- */

/* kleinster Fortschritt der anderen aktiven Teilnehmer (UINT64_MAX: keiner);
   *err = 1, wenn einer ERROR meldet */
uint64_t simlink_peers_progress(simlink_t *sl, int slot, int *err);

/* wartet, bis der Fortschritt der anderen groesser als 'above' ist;
   liefert den neuen Fortschritt */
uint64_t simlink_wait_peers(simlink_t *sl, int slot, uint64_t above);

/* Stand der anderen zum Zeitpunkt t einschliesslich (ohne Warten;
   nur gueltig fuer t <= simlink_peers_progress) */
uint8_t simlink_peek_others(simlink_t *sl, int slot, uint64_t t_ps, uint8_t *data);

/* frueheste Aenderung eines anderen Teilnehmers mit after < t <= upto;
   1 und *t_change, oder 0 */
int simlink_next_change(simlink_t *sl, int slot, uint64_t after, uint64_t upto,
                        uint64_t *t_change);

/* Anzahl aktiver Teilnehmer ausser slot */
int simlink_others(simlink_t *sl, int slot);

#ifdef __cplusplus
}
#endif
#endif
