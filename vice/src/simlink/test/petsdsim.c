/*
 * petsdsim.c - petSD+ (NODISKEMU, ATmega1284P) in simavr am simlink-IEEE-Bus
 *
 *   petsdsim [-v] [-l lookahead_us] firmware.elf sdkarte.img bus
 *
 * Die Firmware laeuft unveraendert in simavr (8 MHz). Die IEEE-Pins des AVR
 * gehen ueber ein Modell der Treiber 75160/75161 auf den simlink-Bus, die
 * SD-Karte am SPI ist eine Abbilddatei (SDHC, Blockadressen). LCD, Tasten,
 * RTC und I2C-Baustein fehlen; die Tasten (ADC7) stehen auf "keine Taste".
 *
 * Zeitkopplung: konservativ wie die VICE-Busmodule, aber mit Vorausschau
 * L: Eingaenge vom Bus wirken L spaeter auf den AVR. Damit darf der AVR bis
 * Fortschritt der anderen + L rechnen, und beide Seiten kommen je Runde um
 * L voran (statt um 1 ps). Fuer den IEEE-Handshake ist eine Verzoegerung von
 * wenigen us unerheblich.
 *
 * Pinbelegung (NODISKEMU src/avr/arch-config.h, HW_PETSDPLUS):
 *   PA0-6, PD7  D1-D8       PC3 TE (1 = senden)
 *   PC7 NRFD   PC6 NDAC   PC5 DAV   PC4 EOI   PC2 IFC   PD2 ATN (INT0)
 *   PB4 SS, PB5 MOSI, PB6 MISO, PB7 SCK (SD)   PD5 Karte da (low)
 *   PD6 Schreibschutz (high = geschuetzt)   PA7/ADC7 Tasten
 */

#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <simavr/sim_avr.h>
#include <simavr/sim_elf.h>
#include <simavr/sim_irq.h>
#include <simavr/avr_ioport.h>
#include <simavr/avr_spi.h>
#include <simavr/avr_adc.h>
#include <simavr/avr_uart.h>

#include "simlink.h"

#define F_AVR      8000000ULL
#define PS_CYCLE   (1000000000000ULL / F_AVR)       /* 125000 ps */

/* Datenadressen der Ports (I/O + 0x20) */
#define A_PINA 0x20
#define A_DDRA 0x21
#define A_PORTA 0x22
#define A_PINC 0x26
#define A_DDRC 0x27
#define A_PORTC 0x28
#define A_PIND 0x29
#define A_DDRD 0x2a
#define A_PORTD 0x2b

static volatile sig_atomic_t stop = 0;
static int verbose = 0;
static avr_t *avr;
static unsigned min_sp = 0xffff;   /* tiefster Stapelzeiger */
static simlink_t *sl;
static int slot;
static uint64_t off;            /* Startzeit am Bus */
static uint64_t look = 2000000; /* Vorausschau in ps */

static void on_sig(int s) { (void)s; stop = 1; }

static uint64_t now_ps(void)
{
    return off + (uint64_t)avr->cycle * PS_CYCLE;
}

/* ------------------------------ SD-Karte ------------------------------ */

static FILE *img;
static uint64_t img_blocks;
static uint8_t cmd[6];
static int cmdlen = 0;
static int acmd = 0;
static uint8_t out[1200];        /* Antwortbytes */
static int outlen = 0, outpos = 0;
enum { SD_CMD, SD_WRWAIT, SD_WRDATA, SD_MWRWAIT, SD_MWRDATA, SD_RDMULTI } sdst = SD_CMD;
static uint8_t wbuf[514];
static int wpos = 0;
static uint32_t waddr = 0, raddr = 0;
static int sd_ss = 1;
static int idle = 1;
static unsigned long spi_n = 0;

static uint16_t crc16(const uint8_t *p, int n)
{
    uint16_t c = 0;
    int i, k;
    for (i = 0; i < n; i++) {
        c ^= (uint16_t)p[i] << 8;
        for (k = 0; k < 8; k++) {
            c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
        }
    }
    return c;
}

static void q(uint8_t b) { if (outlen < (int)sizeof out) out[outlen++] = b; }

static void q_block(uint32_t blk)
{
    uint8_t d[512];
    uint16_t c;
    memset(d, 0, sizeof d);
    if (blk < img_blocks) {
        fseek(img, (long)blk * 512, SEEK_SET);
        if (fread(d, 1, 512, img) != 512) {
            memset(d, 0, sizeof d);
        }
    }
    c = crc16(d, 512);
    q(0xff); q(0xfe);
    if (outlen + 514 > (int)sizeof out) {
        return;
    }
    memcpy(out + outlen, d, 512);
    outlen += 512;
    q((uint8_t)(c >> 8)); q((uint8_t)c);
}

static void sd_command(void)
{
    int c = cmd[0] & 0x3f;
    uint32_t arg = ((uint32_t)cmd[1] << 24) | ((uint32_t)cmd[2] << 16) | ((uint32_t)cmd[3] << 8) | cmd[4];
    uint8_t r1 = idle ? 0x01 : 0x00;
    outlen = outpos = 0;
    q(0xff);                    /* Ncr */
    if (verbose > 2) {
        fprintf(stderr, "petsdsim: SD %sCMD%d %08x\n", acmd ? "A" : "", c, arg);
    }
    if (acmd) {
        acmd = 0;
        if (c == 41) {
            idle = 0;
            q(0x00);
        } else {
            q(r1);
        }
        return;
    }
    switch (c) {
    case 0:  idle = 1; q(0x01); break;
    case 1:  idle = 0; q(0x00); break;          /* MMC-Init, SD ignoriert */
    case 8:  q(0x01); q(0); q(0); q(0x01); q(cmd[4]); break;
    case 55: q(r1); acmd = 1; break;
    case 58: q(r1); q(0xc0); q(0xff); q(0x80); q(0x00); break;   /* CCS = 1 */
    case 59: case 16: q(r1); break;
    case 9: {                   /* CSD v2 */
        uint8_t csd[16] = { 0x40, 0x0e, 0x00, 0x32, 0x5b, 0x59, 0x00, 0, 0, 0,
                            0x7f, 0x80, 0x0a, 0x40, 0x00, 0x01 };
        uint32_t cs = (uint32_t)(img_blocks / 1024) - 1;
        uint16_t cc;
        csd[7] = (uint8_t)((cs >> 16) & 0x3f);
        csd[8] = (uint8_t)(cs >> 8);
        csd[9] = (uint8_t)cs;
        cc = crc16(csd, 16);
        q(0x00); q(0xff); q(0xfe);
        memcpy(out + outlen, csd, 16); outlen += 16;
        q((uint8_t)(cc >> 8)); q((uint8_t)cc);
        break;
    }
    case 10: {                  /* CID */
        uint8_t cid[16] = { 0x03, 'S', 'D', 'S', 'I', 'M', 'L', 'K', 0x10, 1, 2, 3, 4, 0x01, 0x1a, 0x01 };
        uint16_t cc = crc16(cid, 16);
        q(0x00); q(0xff); q(0xfe);
        memcpy(out + outlen, cid, 16); outlen += 16;
        q((uint8_t)(cc >> 8)); q((uint8_t)cc);
        break;
    }
    case 13: q(0x00); q(0x00); break;
    case 17: q(0x00); q_block(arg); break;
    case 18: q(0x00); raddr = arg; q_block(raddr++); sdst = SD_RDMULTI; break;
    case 12: sdst = SD_CMD; q(0xff); q(0x00); q(0x00); q(0xff); break;
    case 24: q(0x00); waddr = arg; sdst = SD_WRWAIT; break;
    case 25: q(0x00); waddr = arg; sdst = SD_MWRWAIT; break;
    default:
        if (verbose) {
            fprintf(stderr, "petsdsim: SD unbekannter CMD%d\n", c);
        }
        q(0x04);                /* illegal command */
        break;
    }
}

static uint8_t sd_byte(uint8_t in)
{
    uint8_t o = 0xff;
    if (sd_ss) {
        return 0xff;
    }
    if (outpos < outlen) {
        o = out[outpos++];
    } else if (sdst == SD_RDMULTI) {
        outlen = outpos = 0;
        q_block(raddr++);
        o = out[outpos++];
    }
    switch (sdst) {
    case SD_CMD:
    case SD_RDMULTI:
        if (cmdlen == 0 && (in & 0xc0) != 0x40) {
            break;
        }
        cmd[cmdlen++] = in;
        if (cmdlen == 6) {
            cmdlen = 0;
            if (sdst == SD_RDMULTI && (cmd[0] & 0x3f) != 12) {
                break;
            }
            sd_command();
        }
        break;
    case SD_WRWAIT:
    case SD_MWRWAIT:
        if (in == 0xfe || in == 0xfc) {
            wpos = 0;
            sdst = sdst == SD_WRWAIT ? SD_WRDATA : SD_MWRDATA;
        } else if (in == 0xfd && sdst == SD_MWRWAIT) {
            outlen = outpos = 0;
            q(0xff); q(0x00); q(0x00); q(0xff);
            sdst = SD_CMD;
        } else if ((in & 0xc0) == 0x40 && sdst == SD_MWRWAIT) {
            sdst = SD_CMD;
            cmd[0] = in;
            cmdlen = 1;
        }
        break;
    case SD_WRDATA:
    case SD_MWRDATA:
        wbuf[wpos++] = in;
        if (wpos == 514) {
            if (waddr < img_blocks) {
                fseek(img, (long)waddr * 512, SEEK_SET);
                fwrite(wbuf, 1, 512, img);
                fflush(img);
            }
            waddr++;
            outlen = outpos = 0;
            q(0x05); q(0x00); q(0x00); q(0xff);
            sdst = sdst == SD_WRDATA ? SD_CMD : SD_MWRWAIT;
        }
        break;
    }
    return o;
}

static void spi_out(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq; (void)param;
    uint8_t r;
    spi_n++;
    r = sd_byte((uint8_t)value);
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_INPUT), r);
}

static void ss_change(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq; (void)param;
    sd_ss = value ? 1 : 0;
    if (sd_ss) {
        cmdlen = 0;
        outlen = outpos = 0;    /* Rest einer Antwort verfaellt */
    }
}

/* ------------------------------ IEEE-Bus ------------------------------ */

static uint8_t pub_pull = 0, pub_data = 0;
static int pub_valid = 0;

/* Eingang von aussen treiben. Als "extern" markiert, damit ein Pull-up der
   Firmware den Pegel nicht ueberschreibt (simavr nimmt sonst den Pull-up). */
static uint8_t ext_mask[4], ext_val[4];

static void pin(char port, int bit, int v)
{
    avr_ioport_external_t e;
    int i = port - 'A';
    ext_mask[i] |= (uint8_t)(1 << bit);
    if (v) {
        ext_val[i] |= (uint8_t)(1 << bit);
    } else {
        ext_val[i] &= (uint8_t)~(1 << bit);
    }
    e.name = (unsigned long)port;
    e.mask = ext_mask[i];
    e.value = ext_val[i];
    avr_ioctl(avr, AVR_IOCTL_IOPORT_SET_EXTERNAL(port), &e);
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(port), bit), v ? 1 : 0);
}

/* Bus der anderen (pull/data: gesetzt = low) auf die AVR-Eingaenge */
static uint8_t ext_pull = 0xff, ext_data = 0xff;
static int ext_valid = 0;

static void apply_ext(uint8_t l, uint8_t d)
{
    int i;
    uint8_t ch = ext_valid ? (uint8_t)(l ^ ext_pull) : 0xff;
    uint8_t cd = ext_valid ? (uint8_t)(d ^ ext_data) : 0xff;
    if (ch & SL_IEEE_ATN)  pin('D', 2, !(l & SL_IEEE_ATN));
    if (ch & SL_IEEE_IFC)  pin('C', 2, !(l & SL_IEEE_IFC));
    if (ch & SL_IEEE_NRFD) pin('C', 7, !(l & SL_IEEE_NRFD));
    if (ch & SL_IEEE_NDAC) pin('C', 6, !(l & SL_IEEE_NDAC));
    if (ch & SL_IEEE_DAV)  pin('C', 5, !(l & SL_IEEE_DAV));
    if (ch & SL_IEEE_EOI)  pin('C', 4, !(l & SL_IEEE_EOI));
    for (i = 0; i < 7; i++) {
        if (cd & (1 << i)) pin('A', i, !(d & (1 << i)));
    }
    if (cd & 0x80) pin('D', 7, !(d & 0x80));
    if (verbose > 2 && (ch || cd)) {
        fprintf(stderr, "petsdsim %10.3f ms: Bus -> AVR %02x/%02x\n", now_ps() / 1e9, l, d);
    }
    ext_pull = l;
    ext_data = d;
    ext_valid = 1;
}

/* Ausgaenge des AVR ueber die 75160/75161 auf den Bus */
static int drives(uint8_t ddr, uint8_t port, int bit)
{
    return (ddr & (1 << bit)) && !(port & (1 << bit));
}

static void publish(void)
{
    uint8_t ddrc = avr->data[A_DDRC], portc = avr->data[A_PORTC];
    uint8_t ddra = avr->data[A_DDRA], porta = avr->data[A_PORTA];
    uint8_t ddrd = avr->data[A_DDRD], portd = avr->data[A_PORTD];
    int te = (portc & 0x08) && (ddrc & 0x08);
    uint8_t p = 0, d = 0;
    int i;
    if (te) {
        if (drives(ddrc, portc, 5)) p |= SL_IEEE_DAV;
        if (drives(ddrc, portc, 4)) p |= SL_IEEE_EOI;
        for (i = 0; i < 7; i++) {
            if (drives(ddra, porta, i)) d |= (uint8_t)(1 << i);
        }
        if (drives(ddrd, portd, 7)) d |= 0x80;
    } else {
        if (drives(ddrc, portc, 7)) p |= SL_IEEE_NRFD;
        if (drives(ddrc, portc, 6)) p |= SL_IEEE_NDAC;
    }
    if (!pub_valid || p != pub_pull || d != pub_data) {
        pub_valid = 1;
        pub_pull = p;
        pub_data = d;
        simlink_set(sl, slot, now_ps(), p, d);
        if (verbose > 1) {
            fprintf(stderr, "petsdsim %10.3f ms: AVR -> Bus %02x/%02x%s\n", now_ps() / 1e9, p, d,
                    te ? " (TE)" : "");
        }
    }
}

/* UART0 (Debugausgabe der Firmware, falls eingebaut) */
static void uart_out(struct avr_irq_t *irq, uint32_t value, void *param)
{
    static char line[256];
    static int n = 0;
    (void)irq; (void)param;
    if (value == '\n' || n == (int)sizeof line - 1) {
        line[n] = 0;
        fprintf(stderr, "petsdsim %10.3f ms: UART %s\n", now_ps() / 1e9, line);
        n = 0;
    } else if (value != '\r') {
        line[n++] = (char)value;
    }
}

/* ------------------------------ Hauptteil ----------------------------- */

int main(int argc, char **argv)
{
    elf_firmware_t fw;
    int c;
    uint64_t done = 0, last_ext = 0;
    uint64_t pend_t[64];
    uint8_t pend_l[64], pend_d[64];
    int pend_n = 0;

    while ((c = getopt(argc, argv, "vl:")) != -1) {
        switch (c) {
        case 'v': verbose++; break;
        case 'l': look = (uint64_t)(atof(optarg) * 1e6); break;
        default: goto usage;
        }
    }
    if (argc - optind != 3) {
usage:
        fprintf(stderr, "usage: petsdsim [-v] [-l vorausschau_us] firmware.elf sdkarte.img bus\n");
        return 1;
    }
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    memset(&fw, 0, sizeof fw);
    if (elf_read_firmware(argv[optind], &fw)) {
        fprintf(stderr, "petsdsim: %s nicht lesbar\n", argv[optind]);
        return 1;
    }
    avr = avr_make_mcu_by_name("atmega1284p");
    if (!avr) {
        return 1;
    }
    avr_init(avr);
    avr->frequency = F_AVR;
    avr->avcc = 5000;
    avr->aref = 5000;
    avr->vcc = 5000;
    avr_load_firmware(avr, &fw);
    avr->log = verbose > 2 ? 3 : 1;

    img = fopen(argv[optind + 1], "r+b");
    if (!img) {
        perror(argv[optind + 1]);
        return 1;
    }
    fseek(img, 0, SEEK_END);
    img_blocks = (uint64_t)ftell(img) / 512;

    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_OUTPUT), spi_out, NULL);
    {
        uint32_t f = 0;
        avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &f);
        f &= ~AVR_UART_FLAG_STDIO;
        avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &f);
        avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), uart_out, NULL);
    }
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 4), ss_change, NULL);
    /* feste Eingaenge: Karte da, kein Schreibschutz, I2C-Leitungen hoch,
       Tasten: ADC7 etwa 550 (Ruhewert ohne Taste, readbuttons.inc) */
    pin('D', 5, 0);
    pin('D', 6, 0);
    pin('C', 0, 1);
    pin('C', 1, 1);
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC7), 2690);

    sl = simlink_open(argv[optind + 2], SIMLINK_BUS_IEEE, 0, 0xff);
    if (!sl) {
        sl = simlink_open(argv[optind + 2], SIMLINK_BUS_IEEE, 1, 0xff);
    }
    if (!sl) {
        perror("simlink_open");
        return 1;
    }
    slot = simlink_attach(sl, SL_TYPE_AVR, 0);
    if (slot < 0) {
        fprintf(stderr, "petsdsim: kein freier Platz am Bus\n");
        return 1;
    }
    fprintf(stderr, "petsdsim: %s, Karte %s (%llu Bloecke), Bus %s\n", argv[optind], argv[optind + 1],
            (unsigned long long)img_blocks, argv[optind + 2]);
    {
        int waited = 0;
        while (simlink_others(sl, slot) == 0 && !stop) {
            usleep(10000);
            if (++waited > 3000) {
                fprintf(stderr, "petsdsim: nach 30 s keine Gegenseite, Ende\n");
                return 1;
            }
        }
    }
    off = simlink_start_time(sl, slot);
    done = off;
    last_ext = off;
    {
        uint8_t d0 = 0, l0 = simlink_peek_others(sl, slot, off, &d0);
        apply_ext(l0, d0);
    }
    publish();
    simlink_progress(sl, slot, done + 1);

    while (!stop) {
        int err = 0, st;
        uint64_t m = simlink_peers_progress(sl, slot, &err), upto, tc, target;
        if (err || simlink_others(sl, slot) == 0) {
            fprintf(stderr, "petsdsim: Gegenseite beendet, tiefster SP 0x%04x\n", min_sp);
            break;
        }
        if (m == UINT64_MAX || m - 2 + look <= done) {
            /* Vorausschau aufgebraucht: auf die anderen warten */
            m = m == UINT64_MAX ? m : simlink_wait_peers(sl, slot, done + 1 - look);
            if (simlink_others(sl, slot) == 0) {
                fprintf(stderr, "petsdsim: Gegenseite beendet, tiefster SP 0x%04x\n", min_sp);
                break;
            }
            if (m == UINT64_MAX) {
                usleep(1000);       /* Gegenseite noch nicht aktiv */
                continue;
            }
        }
        /* VICE meldet Fortschritt done+1, stempelt eine spaetere Aenderung
           des PET aber noch mit done (parallel-simlink.c, publish); daher
           gilt erst alles vor m - 1 als endgueltig */
        upto = m - 2;
        /* Aenderungen der anderen bis upto vormerken; sie wirken L spaeter */
        while (pend_n < 64 && simlink_next_change(sl, slot, last_ext, upto, &tc)) {
            uint8_t d = 0;
            pend_l[pend_n] = simlink_peek_others(sl, slot, tc, &d);
            pend_d[pend_n] = d;
            pend_t[pend_n] = tc + look;
            if (verbose > 2) {
                fprintf(stderr, "petsdsim: Aenderung bei %.3f ms: %02x/%02x\n", tc / 1e9, pend_l[pend_n], d);
            }
            pend_n++;
            last_ext = tc;
        }
        if (pend_n < 64) {
            /* Ein Eintrag genau bei der gemeldeten Fortschrittsgrenze kann
               nachtraeglich erscheinen (VICE stempelt mit done_ps); daher
               zusaetzlich den Stand bei upto vergleichen */
            uint8_t d = 0, l = simlink_peek_others(sl, slot, upto, &d);
            uint8_t ql = pend_n ? pend_l[pend_n - 1] : ext_pull;
            uint8_t qd = pend_n ? pend_d[pend_n - 1] : ext_data;
            if (l != ql || d != qd) {
                pend_l[pend_n] = l;
                pend_d[pend_n] = d;
                pend_t[pend_n] = upto + look;
                pend_n++;
            }
            last_ext = upto > last_ext ? upto : last_ext;
        }
        target = upto + look;
        while (now_ps() < target && !stop) {
            while (pend_n && pend_t[0] <= now_ps()) {
                apply_ext(pend_l[0], pend_d[0]);
                memmove(pend_t, pend_t + 1, sizeof pend_t[0] * (size_t)(pend_n - 1));
                memmove(pend_l, pend_l + 1, (size_t)(pend_n - 1));
                memmove(pend_d, pend_d + 1, (size_t)(pend_n - 1));
                pend_n--;
            }
            st = avr_run(avr);
            {
                unsigned sp = avr->data[0x5d] | (avr->data[0x5e] << 8);
                if (sp < min_sp) min_sp = sp;
            }
            if (st == cpu_Done || st == cpu_Crashed) {
                fprintf(stderr, "petsdsim: AVR angehalten (%d) bei PC %04x\n", st, avr->pc);
                stop = 1;
                break;
            }
            publish();
        }
        done = now_ps();
        simlink_progress(sl, slot, done + 1);
        if (verbose) {
            static uint64_t next_rep = 0;
            if (avr->cycle >= next_rep) {
                fprintf(stderr, "petsdsim %10.3f ms: PC %05x SP %04x PIND %02x PINC %02x PORTB %02x DDRB %02x SPI %lu ss %d ADC %d ADMUX %02x\n", now_ps() / 1e9, avr->pc,
                        avr->data[0x5d] | (avr->data[0x5e] << 8), avr->data[A_PIND], avr->data[A_PINC],
                        avr->data[0x25], avr->data[0x24], spi_n, sd_ss, avr->data[0x78] | (avr->data[0x79] << 8), avr->data[0x7c]);
                next_rep = avr->cycle + F_AVR / 4;
            }
        }
    }
    simlink_detach(sl, slot);
    simlink_close(sl, 0);
    fclose(img);
    return 0;
}
