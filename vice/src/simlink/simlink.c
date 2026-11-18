/*
 * simlink - Implementierung (gemeinsamer Speicher, C11-Atomics)
 *
 * POSIX:   shm_open("/simlink-<name>") + mmap
 * Windows: CreateFileMapping("Local\\simlink-<name>") + MapViewOfFile
 *          (der Speicher verschwindet, wenn der letzte Teilnehmer schliesst)
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "simlink.h"

#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#define SIMLINK_MAGIC 0x4B4E4C53u /* "SLNK" */

/* Ein Ringeintrag: ab time_ps gilt state/pulses.
   state  = pull | data << 8
   pulses = Zaehler PSTB | PACK << 8 | SRQ << 16 (je 8 Bit, laufen ueber) */
typedef struct {
    uint64_t time_ps;
    uint32_t state;
    uint32_t pulses;
} sl_entry_t;

typedef struct {
    _Atomic uint64_t time_ps;   /* Fortschritt */
    _Atomic uint32_t head;      /* Anzahl geschriebener Eintraege (laeuft weiter) */
    _Atomic uint32_t flags;     /* SL_F_*, aktuell */
    _Atomic uint64_t wait_until; /* != 0: wartet auf Aenderung, spaetestens bis hier */
    _Atomic uint64_t react_ps;  /* beim Warten: fruehestens so lange nach einer Aenderung */
    _Atomic uint32_t wait_mask; /* beim Warten: auf welche Leitungen */
    _Atomic uint64_t wait_from; /* beim Warten: ab wann */
    _Atomic uint64_t wake_ps;   /* fruehestes Umschalten einer beobachteten Leitung
                                   durch einen anderen ab wait_from (UINT64_MAX: keins);
                                   setzt der Schreiber selbst */
    _Atomic uint32_t wait_scanned; /* Ring einmal durchgesehen */
    sl_entry_t ring[SIMLINK_RING];
} sl_part_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t  bus;
    uint8_t  line_mask;
    uint8_t  type[SIMLINK_MAX_PARTS];
    uint8_t  pad[6];
    _Atomic uint64_t grant_ps;       /* Zeit des Taktgebers */
    _Atomic uint32_t slots;          /* Belegung der Teilnehmerplaetze */
    _Atomic uint32_t pad2;
    sl_part_t part[SIMLINK_MAX_PARTS];
} sl_shm_t;

struct simlink_s {
    sl_shm_t *shm;
    char      name[64];
#ifdef _WIN32
    HANDLE    map;
#endif
    uint32_t  cur_state[SIMLINK_MAX_PARTS];   /* eigener Stand (nur eigener Slot genutzt) */
    uint32_t  cur_pulses[SIMLINK_MAX_PARTS];
    uint32_t  rd_pulses[SIMLINK_MAX_PARTS];   /* beim letzten Lesen gesehene Zaehler der anderen */
    uint32_t  seen[SIMLINK_MAX_PARTS];        /* von simlink_pulses() schon gemeldet */
    uint64_t  waits;
};

#define ST_PULL(s)  ((uint8_t)((s) & 0xff))
#define ST_DATA(s)  ((uint8_t)(((s) >> 8) & 0xff))
#define ST_MAKE(p, d) ((uint32_t)(p) | ((uint32_t)(d) << 8))

static void spin(unsigned *n)
{
    if (++*n > 200) {
#ifdef _WIN32
        SwitchToThread();
#else
        sched_yield();
#endif
    }
}

/* Plattformschicht: Speicher anlegen/oeffnen. *fresh = 1, wenn neu angelegt. */
#ifdef _WIN32

static sl_shm_t *map_open(const char *name, int create, HANDLE *h, int *fresh)
{
    char path[80];
    void *p;

    snprintf(path, sizeof(path), "Local\\simlink-%.40s", name);
    *fresh = 0;
    if (create) {
        *h = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE,
                                0, (DWORD)sizeof(sl_shm_t), path);
        if (*h != NULL && GetLastError() != ERROR_ALREADY_EXISTS) {
            *fresh = 1;     /* neu: Windows liefert ihn genullt */
        }
    } else {
        *h = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, path);
    }
    if (*h == NULL) {
        return NULL;
    }
    p = MapViewOfFile(*h, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(sl_shm_t));
    if (p == NULL) {
        CloseHandle(*h);
        *h = NULL;
        return NULL;
    }
    return (sl_shm_t *)p;
}

static void map_close(simlink_t *sl, int unlink_bus)
{
    (void)unlink_bus;   /* entfaellt: verschwindet mit dem letzten Handle */
    UnmapViewOfFile(sl->shm);
    CloseHandle(sl->map);
}

#else

static sl_shm_t *map_open(const char *name, int create, char *path, int *fresh)
{
    int fd;
    void *p;

    snprintf(path, 64, "/simlink-%.40s", name);
    *fresh = create;
    fd = shm_open(path, O_RDWR | (create ? O_CREAT : 0), 0600);
    if (fd < 0) {
        return NULL;
    }
    if (create && ftruncate(fd, sizeof(sl_shm_t)) < 0) {
        close(fd);
        return NULL;
    }
    p = mmap(NULL, sizeof(sl_shm_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    return p == MAP_FAILED ? NULL : (sl_shm_t *)p;
}

static void map_close(simlink_t *sl, int unlink_bus)
{
    munmap(sl->shm, sizeof(*sl->shm));
    if (unlink_bus) {
        shm_unlink(sl->name);
    }
}

#endif

simlink_t *simlink_open(const char *name, int bus, int create, uint8_t line_mask)
{
    simlink_t *sl;
    sl_shm_t *shm;
    int fresh;

    sl = calloc(1, sizeof(*sl));
    if (!sl) {
        return NULL;
    }
#ifdef _WIN32
    snprintf(sl->name, sizeof(sl->name), "%.40s", name);
    shm = map_open(name, create, &sl->map, &fresh);
#else
    shm = map_open(name, create, sl->name, &fresh);
#endif
    if (!shm) {
        free(sl);
        return NULL;
    }
    sl->shm = shm;
    if (fresh) {
        memset(shm, 0, sizeof(*shm));
        shm->version = SIMLINK_VERSION;
        shm->bus = (uint8_t)bus;
        shm->line_mask = line_mask;
        atomic_thread_fence(memory_order_release);
        shm->magic = SIMLINK_MAGIC;
    } else if (shm->magic != SIMLINK_MAGIC || shm->version != SIMLINK_VERSION
               || shm->bus != (uint8_t)bus) {
        map_close(sl, 0);
        free(sl);
        return NULL;
    }
    return sl;
}

void simlink_close(simlink_t *sl, int unlink_bus)
{
    if (!sl) {
        return;
    }
    map_close(sl, unlink_bus);
    free(sl);
}

static int active(sl_part_t *p)
{
    return atomic_load_explicit(&p->flags, memory_order_acquire) & SL_F_ACTIVE;
}

int simlink_attach(simlink_t *sl, uint8_t type, int clockmaster)
{
    sl_shm_t *shm = sl->shm;
    sl_part_t *p;
    uint32_t old, want;
    uint64_t start = UINT64_MAX;
    int slot, i;

    do {
        old = atomic_load(&shm->slots);
        for (slot = 0; slot < SIMLINK_MAX_PARTS; slot++) {
            if (!(old & (1u << slot))) {
                break;
            }
        }
        if (slot == SIMLINK_MAX_PARTS) {
            return -1;
        }
        want = old | (1u << slot);
    } while (!atomic_compare_exchange_weak(&shm->slots, &old, want));

    /* Neueinsteiger beginnt beim langsamsten aktiven Teilnehmer */
    for (i = 0; i < SIMLINK_MAX_PARTS; i++) {
        if (i != slot && active(&shm->part[i])) {
            uint64_t t = atomic_load(&shm->part[i].time_ps);
            if (t < start) {
                start = t;
            }
        }
    }
    if (start == UINT64_MAX) {
        start = 0;
    }
    p = &shm->part[slot];
    shm->type[slot] = type;
    p->ring[0].time_ps = start;
    p->ring[0].state = 0;
    p->ring[0].pulses = 0;
    atomic_store(&p->head, 1);
    atomic_store(&p->wait_until, 0);
    atomic_store(&p->wait_mask, 0);
    atomic_store(&p->wait_from, 0);
    atomic_store(&p->wake_ps, UINT64_MAX);
    atomic_store(&p->wait_scanned, 0);
    atomic_store(&p->react_ps, 0);
    atomic_store(&p->time_ps, start);
    sl->cur_state[slot] = 0;
    sl->cur_pulses[slot] = 0;
    for (i = 0; i < SIMLINK_MAX_PARTS; i++) {
        sl->rd_pulses[i] = sl->seen[i] = 0;
    }
    atomic_store_explicit(&p->flags, SL_F_ACTIVE | (clockmaster ? SL_F_CLOCKMASTER : 0),
                          memory_order_release);
    return slot;
}

void simlink_detach(simlink_t *sl, int slot)
{
    sl_shm_t *shm = sl->shm;
    atomic_store(&shm->part[slot].flags, SL_F_DETACH);
    atomic_fetch_and(&shm->slots, ~(1u << slot));
}

/* kleinste Fortschrittszeit der anderen aktiven Teilnehmer; *err bei ERROR */
/* Grenze fuer das Freigeben von Ringeintraegen: der reine Fortschritt der
   anderen. Ein Wartender braucht alte Eintraege nicht (der Schreiber setzt
   ihm die Weckzeit selbst), nur bis zu seinem ersten Durchsehen und ab seiner
   Weckzeit. */
static uint64_t min_other_raw(sl_shm_t *shm, int self, int *err)
{
    uint64_t m = UINT64_MAX;
    int i;
    for (i = 0; i < SIMLINK_MAX_PARTS; i++) {
        sl_part_t *p = &shm->part[i];
        uint32_t f;
        uint64_t t;
        if (i == self) {
            continue;
        }
        f = atomic_load_explicit(&p->flags, memory_order_acquire);
        if (!(f & SL_F_ACTIVE)) {
            continue;
        }
        if (f & SL_F_ERROR) {
            *err = 1;
        }
        t = atomic_load_explicit(&p->time_ps, memory_order_acquire);
        if (atomic_load_explicit(&p->wait_until, memory_order_acquire)
            && atomic_load_explicit(&p->wait_scanned, memory_order_acquire)) {
            uint64_t wk = atomic_load_explicit(&p->wake_ps, memory_order_acquire);
            if (wk > t) {
                t = wk;
            }
        }
        if (t < m) {
            m = t;
        }
    }
    return m;
}

/* fruehestes Neues von q ab 'from', das die Leitungen in mask betrifft:
   bekannter Eintrag >= from, in dem q eine dieser Leitungen umschaltet,
   sonst sein Fortschritt (davor kommt nichts mehr) */
static uint64_t next_activity(sl_part_t *q, uint64_t from, uint8_t mask, int *real)
{
    uint64_t prog = atomic_load_explicit(&q->time_ps, memory_order_acquire);
    uint32_t head = atomic_load_explicit(&q->head, memory_order_seq_cst);
    uint64_t best = prog > from ? prog : from;
    uint32_t k;

    if (real) {
        *real = 0;
    }
    for (k = 0; k < SIMLINK_RING && k < head; k++) {
        sl_entry_t *e = &q->ring[(head - 1 - k) % SIMLINK_RING];
        uint64_t t = e->time_ps;
        if (t < from) {
            break;
        }
        /* auch t == best: ein Eintrag genau beim Fortschritt von q (bzw. genau
           bei from) ist schon geschrieben und echt - ohne ihn verpasste ein
           Wartender eine Aenderung, die vor seinem Wartezeichen kam */
        if (t <= best) {
            /* Umschalten gegenueber dem Eintrag davor (ohne Vorgaenger: ja) */
            int changed = 1;
            if (k + 1 < SIMLINK_RING && k + 1 < head) {
                sl_entry_t *pe = &q->ring[(head - 2 - k) % SIMLINK_RING];
                changed = ((ST_PULL(e->state) ^ ST_PULL(pe->state)) & mask) != 0;
            }
            if (changed) {
                best = t;
                if (real) {
                    *real = 1;
                }
            }
        }
    }
    return best;
}

/* Zeit, vor der von p nichts mehr kommt: sein Fortschritt, oder - wenn er auf
   eine Aenderung wartet - die naechste Aktivitaet eines anderen plus seine
   Reaktionszeit (hoechstens seine Frist) */
static uint64_t effective(sl_shm_t *shm, int pi)
{
    sl_part_t *p = &shm->part[pi];
    uint64_t prog = atomic_load_explicit(&p->time_ps, memory_order_acquire);
    uint64_t w = atomic_load_explicit(&p->wait_until, memory_order_acquire);
    uint64_t react, c = UINT64_MAX, e;
    int i;

    if (w == 0 || !atomic_load_explicit(&p->wait_scanned, memory_order_acquire)) {
        return prog;
    }
    react = atomic_load_explicit(&p->react_ps, memory_order_acquire);
    /* naechste Aktivitaet der anderen: bekanntes Umschalten (Weckzeit) oder,
       solange keins bekannt ist, deren Fortschritt (davor kommt nichts mehr) */
    for (i = 0; i < SIMLINK_MAX_PARTS; i++) {
        uint64_t a;
        if (i == pi || !active(&shm->part[i])) {
            continue;
        }
        a = atomic_load_explicit(&shm->part[i].time_ps, memory_order_acquire);
        if (a < prog) {
            a = prog;
        }
        if (a < c) {
            c = a;
        }
    }
    {
        uint64_t wk = atomic_load_explicit(&p->wake_ps, memory_order_acquire);
        if (wk < c) {
            c = wk;
        }
    }
    e = (c > UINT64_MAX - react) ? UINT64_MAX : c + react;
    if (e > w) {
        e = w;
    }
    return e > prog ? e : prog;
}

/* wirksamer Fortschritt der anderen (mit Vorausschau wartender Teilnehmer) */
static uint64_t min_other(sl_shm_t *shm, int self, int *err)
{
    uint64_t m = UINT64_MAX;
    int i;
    for (i = 0; i < SIMLINK_MAX_PARTS; i++) {
        sl_part_t *p = &shm->part[i];
        uint32_t f;
        uint64_t t;
        if (i == self) {
            continue;
        }
        f = atomic_load_explicit(&p->flags, memory_order_acquire);
        if (!(f & SL_F_ACTIVE)) {
            continue;
        }
        if (f & SL_F_ERROR) {
            *err = 1;
        }
        t = effective(shm, i);
        if (t < m) {
            m = t;
        }
    }
    return m;
}

/* Neuen Ringeintrag anhaengen; wartet, falls der Ring voll ist, d. h. ein
   langsamer Leser die aeltesten Eintraege noch braucht. */
/* Wartende wecken: schaltet ein Eintrag bei t eine Leitung um, auf die ein
   anderer wartet, traegt der Schreiber t als dessen Weckzeit ein. Erst nach
   dem Schreiben des Eintrags (seq_cst): entweder sieht der Schreiber das
   Wartezeichen, oder der Wartende sieht beim Durchsehen den Eintrag. */
static void wake_waiters(simlink_t *sl, int slot, uint64_t t, uint8_t changed)
{
    sl_shm_t *shm = sl->shm;
    int i;

    for (i = 0; changed && i < SIMLINK_MAX_PARTS; i++) {
        sl_part_t *w = &shm->part[i];
        uint64_t old;
        if (i == slot || !atomic_load_explicit(&w->wait_until, memory_order_seq_cst)) {
            continue;
        }
        if (!(changed & (uint8_t)atomic_load_explicit(&w->wait_mask, memory_order_acquire))
            || t < atomic_load_explicit(&w->wait_from, memory_order_acquire)) {
            continue;
        }
        old = atomic_load_explicit(&w->wake_ps, memory_order_acquire);
        while (t < old && !atomic_compare_exchange_weak(&w->wake_ps, &old, t)) {
        }
    }
}

static void append(simlink_t *sl, int slot, uint64_t t)
{
    sl_shm_t *shm = sl->shm;
    sl_part_t *p = &shm->part[slot];
    uint32_t head = atomic_load_explicit(&p->head, memory_order_relaxed);
    sl_entry_t *last = &p->ring[(head - 1) % SIMLINK_RING];
    uint64_t prog = atomic_load_explicit(&p->time_ps, memory_order_relaxed);
    unsigned n = 0;
    uint8_t wake_changed;

    /* Regel: vor dem eigenen Fortschritt ist alles endgueltig. Ein spaeterer
       Eintrag mit kleinerer Zeit wuerde sie brechen (die anderen koennten den
       Stand dort schon gelesen haben) - also auf den Fortschritt anheben.
       Ebenso nie vor den letzten eigenen Eintrag. */
    if (t < prog) {
        t = prog;
    }
    if (t < last->time_ps) {
        t = last->time_ps;
    }

    /* Ausgangsstand vor diesem Eintrag (fuer das Wecken) */
    {
        uint32_t base;
        if (last->time_ps == t) {
            base = (head >= 2) ? p->ring[(head - 2) % SIMLINK_RING].state : 0;
        } else {
            base = last->state;
        }
        wake_changed = (uint8_t)(ST_PULL(base) ^ ST_PULL(sl->cur_state[slot]));
    }

    if (last->time_ps == t) {
        /* gleiche Zeit: letzten Eintrag ueberschreiben. Sicher, weil Leser nur
           Eintraege mit Zeit < ihrer Lesezeit nutzen und wir t noch nicht
           als Fortschritt ueberschritten haben. */
        last->state = sl->cur_state[slot];
        last->pulses = sl->cur_pulses[slot];
        atomic_thread_fence(memory_order_seq_cst);
        wake_waiters(sl, slot, t, wake_changed);
        return;
    }
    /* Platz pruefen: Eintrag head-RING wird ueberschrieben. Er darf weg, wenn
       der naechstjuengere Eintrag (head-RING+1) schon vor dem Fortschritt aller
       Leser liegt; dann braucht niemand mehr den aelteren. */
    while (head >= SIMLINK_RING) {
        int err = 0;
        uint64_t m = min_other_raw(shm, slot, &err);
        if (m == UINT64_MAX || err
            || p->ring[(head - SIMLINK_RING + 1) % SIMLINK_RING].time_ps < m) {
            break;
        }
        atomic_store_explicit(&p->time_ps, t, memory_order_release);
        spin(&n);
    }
    {
        sl_entry_t *e = &p->ring[head % SIMLINK_RING];
        e->time_ps = t;
        e->state = sl->cur_state[slot];
        e->pulses = sl->cur_pulses[slot];
    }
    atomic_store_explicit(&p->head, head + 1, memory_order_seq_cst);
    wake_waiters(sl, slot, t, wake_changed);
}

void simlink_set(simlink_t *sl, int slot, uint64_t t_ps, uint8_t pull, uint8_t data)
{
    uint32_t st = ST_MAKE(pull, data);
    if (st == sl->cur_state[slot]) {
        return;
    }
    sl->cur_state[slot] = st;
    append(sl, slot, t_ps);
}

void simlink_pulse(simlink_t *sl, int slot, uint64_t t_ps, int line)
{
    sl->cur_pulses[slot] += 1u << (8 * line);
    append(sl, slot, t_ps);
}

int simlink_progress(simlink_t *sl, int slot, uint64_t t_ps)
{
    sl_shm_t *shm = sl->shm;
    sl_part_t *p = &shm->part[slot];
    int err = 0;
    /* Fortschritt laeuft nur vorwaerts */
    if (t_ps > atomic_load_explicit(&p->time_ps, memory_order_relaxed)) {
        atomic_store_explicit(&p->time_ps, t_ps, memory_order_release);
    }
    if (atomic_load_explicit(&p->flags, memory_order_relaxed) & SL_F_CLOCKMASTER) {
        atomic_store_explicit(&shm->grant_ps, t_ps, memory_order_release);
    }
    min_other(shm, slot, &err);
    return err ? -1 : 0;
}

/* Zustand eines anderen Teilnehmers vor Zeitpunkt t (letzter Eintrag mit Zeit < t). */
static sl_entry_t state_before(sl_part_t *p, uint64_t t)
{
    uint32_t head = atomic_load_explicit(&p->head, memory_order_acquire);
    uint32_t i = head;
    sl_entry_t e = p->ring[(head - 1) % SIMLINK_RING];
    while (i > 0 && head - i < SIMLINK_RING) {
        e = p->ring[(i - 1) % SIMLINK_RING];
        if (e.time_ps < t) {
            break;
        }
        i--;
    }
    return e;
}

static uint8_t read_impl(simlink_t *sl, int slot, uint64_t t_ps, uint8_t *data, int own)
{
    sl_shm_t *shm = sl->shm;
    uint32_t pull = 0, d = 0;
    unsigned n = 0;
    int i, waited = 0;

    if (own) {
        pull = ST_PULL(sl->cur_state[slot]);
        d = ST_DATA(sl->cur_state[slot]);
    }
    /* eigenen Fortschritt melden, dann warten, bis alle anderen t erreicht haben */
    simlink_progress(sl, slot, t_ps);
    for (;;) {
        int err = 0;
        if (min_other(shm, slot, &err) >= t_ps || err) {
            break;
        }
        waited = 1;
        spin(&n);
    }
    if (waited) {
        sl->waits++;
    }
    for (i = 0; i < SIMLINK_MAX_PARTS; i++) {
        sl_part_t *p = &shm->part[i];
        sl_entry_t e;
        if (i == slot || !active(p)) {
            continue;
        }
        e = state_before(p, t_ps);
        pull |= ST_PULL(e.state);
        d |= ST_DATA(e.state);
        sl->rd_pulses[i] = e.pulses;
    }
    if (data) {
        *data = (uint8_t)d;
    }
    return (uint8_t)pull & shm->line_mask;
}

uint8_t simlink_read(simlink_t *sl, int slot, uint64_t t_ps, uint8_t *data)
{
    return read_impl(sl, slot, t_ps, data, 1);
}

uint8_t simlink_read_others(simlink_t *sl, int slot, uint64_t t_ps, uint8_t *data)
{
    return read_impl(sl, slot, t_ps, data, 0);
}

unsigned simlink_pulses(simlink_t *sl, int slot, int line)
{
    unsigned n = 0;
    int i;
    for (i = 0; i < SIMLINK_MAX_PARTS; i++) {
        uint32_t now = sl->rd_pulses[i], old = sl->seen[i];
        uint32_t mask = 0xffu << (8 * line);
        if (i == slot) {
            continue;
        }
        n += (uint8_t)(((now & mask) >> (8 * line)) - ((old & mask) >> (8 * line)));
        sl->seen[i] = (old & ~mask) | (now & mask);
    }
    return n;
}

uint64_t simlink_waits(simlink_t *sl)
{
    return sl->waits;
}

uint64_t simlink_start_time(simlink_t *sl, int slot)
{
    return sl->shm->part[slot].ring[0].time_ps;
}

int simlink_others(simlink_t *sl, int slot)
{
    int i, n = 0;
    for (i = 0; i < SIMLINK_MAX_PARTS; i++) {
        if (i != slot && active(&sl->shm->part[i])) {
            n++;
        }
    }
    return n;
}

uint64_t simlink_peers_progress(simlink_t *sl, int slot, int *err)
{
    int e = 0;
    uint64_t m = min_other(sl->shm, slot, &e);
    if (err) {
        *err = e;
    }
    return m;
}

uint64_t simlink_wait_peers(simlink_t *sl, int slot, uint64_t above)
{
    unsigned n = 0;
    int waited = 0;
    for (;;) {
        int e = 0;
        uint64_t m = min_other(sl->shm, slot, &e);
        if (m > above || e) {
            if (waited) {
                sl->waits++;
            }
            return m;
        }
        waited = 1;
        spin(&n);
    }
}

uint8_t simlink_peek_others(simlink_t *sl, int slot, uint64_t t_ps, uint8_t *data)
{
    sl_shm_t *shm = sl->shm;
    uint32_t pull = 0, d = 0;
    int i;
    uint64_t t = (t_ps == UINT64_MAX) ? t_ps : t_ps + 1;   /* einschliesslich t */

    for (i = 0; i < SIMLINK_MAX_PARTS; i++) {
        sl_part_t *p = &shm->part[i];
        sl_entry_t e;
        if (i == slot || !active(p)) {
            continue;
        }
        e = state_before(p, t);
        pull |= ST_PULL(e.state);
        d |= ST_DATA(e.state);
        sl->rd_pulses[i] = e.pulses;
    }
    if (data) {
        *data = (uint8_t)d;
    }
    return (uint8_t)pull & shm->line_mask;
}

int simlink_next_change(simlink_t *sl, int slot, uint64_t after, uint64_t upto,
                        uint64_t *t_change)
{
    sl_shm_t *shm = sl->shm;
    int i, found = 0;
    uint64_t best = UINT64_MAX;

    for (i = 0; i < SIMLINK_MAX_PARTS; i++) {
        sl_part_t *p = &shm->part[i];
        uint32_t head, k;
        if (i == slot || !active(p)) {
            continue;
        }
        head = atomic_load_explicit(&p->head, memory_order_acquire);
        /* Eintraege vom neuesten rueckwaerts; der Ring enthaelt bis zu RING */
        for (k = 0; k < SIMLINK_RING && k < head; k++) {
            uint64_t t = p->ring[(head - 1 - k) % SIMLINK_RING].time_ps;
            if (t <= after) {
                break;
            }
            if (t <= upto && t < best) {
                best = t;
                found = 1;
            }
        }
    }
    if (found && t_change) {
        *t_change = best;
    }
    return found;
}

int simlink_wait_change(simlink_t *sl, int slot, uint64_t t_from, uint8_t mask,
                        uint64_t react_ps, uint64_t deadline, uint64_t *t_change)
{
    sl_shm_t *shm = sl->shm;
    sl_part_t *me = &shm->part[slot];
    uint64_t dl = deadline ? deadline : UINT64_MAX - 1;
    unsigned n = 0;
    int waited = 0, i;

    /* erst alle Angaben und den Fortschritt, dann das Wartezeichen */
    atomic_store_explicit(&me->wait_scanned, 0, memory_order_release);
    atomic_store_explicit(&me->wake_ps, UINT64_MAX, memory_order_release);
    atomic_store_explicit(&me->react_ps, react_ps, memory_order_release);
    atomic_store_explicit(&me->wait_mask, mask, memory_order_release);
    atomic_store_explicit(&me->wait_from, t_from, memory_order_release);
    simlink_progress(sl, slot, t_from);
    atomic_store_explicit(&me->wait_until, dl, memory_order_seq_cst);

    /* einmal durchsehen, was die anderen schon geschrieben haben, bevor sie
       das Wartezeichen sehen konnten; danach wecken sie selbst */
    for (i = 0; i < SIMLINK_MAX_PARTS; i++) {
        uint64_t a, old;
        int real = 0;
        if (i == slot || !active(&shm->part[i])) {
            continue;
        }
        a = next_activity(&shm->part[i], t_from, mask, &real);
        if (!real) {
            continue;
        }
        old = atomic_load_explicit(&me->wake_ps, memory_order_acquire);
        while (a < old && !atomic_compare_exchange_weak(&me->wake_ps, &old, a)) {
        }
    }
    atomic_store_explicit(&me->wait_scanned, 1, memory_order_seq_cst);

    for (;;) {
        int err = 0;
        uint64_t m = min_other(shm, slot, &err);
        uint64_t wk = atomic_load_explicit(&me->wake_ps, memory_order_acquire);

        /* endgueltig, wenn vor dem Fortschritt aller anderen */
        if (wk < m && wk < dl) {
            if (waited) {
                sl->waits++;
            }
            *t_change = wk;
            return 1;
        }
        if (m >= dl || err) {
            if (waited) {
                sl->waits++;
            }
            *t_change = dl;
            return 0;
        }
        waited = 1;
        spin(&n);
    }
}

void simlink_wait_end(simlink_t *sl, int slot, uint64_t t_now)
{
    sl_part_t *me = &sl->shm->part[slot];
    /* Reihenfolge: erst das Wartezeichen loeschen, dann den Fortschritt
       setzen. Die anderen lesen erst den Fortschritt, dann das Zeichen; so
       sehen sie nie den neuen Fortschritt zusammen mit dem alten Zeichen
       (das waere eine zu weite Zusage). Kurzzeitig sehen sie den alten
       Fortschritt ohne Zeichen - das ist nur vorsichtiger. */
    atomic_store_explicit(&me->wait_until, 0, memory_order_seq_cst);
    simlink_progress(sl, slot, t_now);
}
