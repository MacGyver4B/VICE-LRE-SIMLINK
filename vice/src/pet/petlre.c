/*
 * petlre.c - PET 2001 LRE: CPU socket card and 320x200 video board
 *
 * Written by Stefan (Natas) and Claude, 2026.
 *
 * This file is part of VICE, the Versatile Commodore Emulator.
 * See README for copyright notice.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */

/*
 * CPU socket card (GAL PET2001LRE_CPU):
 *
 *  - 27512 with two 32K ROM sets. The CPU sees the selected set at
 *    $A000-$FFFF, except the I/O page $E800-$E8FF. The card's data
 *    '245 isolates the mainboard for all of these accesses, so the
 *    mainboard ROMs and the I/O mirrors at $E900-$EFFF are invisible.
 *  - Set latch: a write to $FFxx sets EPROM A15 = D0. /RES clears it.
 *  - 32K SRAM at $0000-$7FFF (use -ramsize 32).
 *
 * RAM board (GAL PET2001LRE_GLUE) in the two 6550 sockets:
 *
 *  - 62256 SRAM, emulated with the same address forming as the board:
 *      text mode, CPU:   A0-9 = VA0-9, A10-12 = 0, A13 = SA13, A14 = ATTR
 *      graphics,  CPU:   A0-6 = VA3-9, A7-9 = BA10-12, A10-12 = VA0-2
 *      text video:       char  = $2000 + cell, attribute = $6000 + cell
 *      graphics video:   byte  = (RA << 10) + cell
 *    giving the planes  $0000 bitmap, $2000 text, $4000 register shadow,
 *    $6000 attributes.
 *  - Register (GAL rev 05), two windows, only one active at a time:
 *    text $8F80-$8FFF "ON":  bit 0 = 1 graphics on, bit 1 = ATTREN
 *    gfx  $9F80-$9FFF "OFF": bit 0 = 1 graphics off, bit 1 ignored
 *    A 0 in bit 0 never changes the mode. Cleared by /RES.
 *    Writes also go to the SRAM (shadow), reads come from the SRAM.
 *  - 27C512 character EPROM: A0-2 = RA, A3-9 = code bits 0-6,
 *    A10 = GRAPHIC, A11 = code bit 7, A12-15 = attribute latch.
 *    In graphics mode the SRAM byte goes straight to the shift register.
 *
 * Options:
 *  -petlre            enable the card
 *  -lrechar <file>    64K character EPROM image
 *  -lrecard <file>    64K image (set 0 at $0000, set 1 at $8000;
 *                     CPU $A000 = image offset $2000 within a set)
 *  -lreshiftlock      hold left SHIFT in the matrix, graphics keyboard
 *  -lreshiftlockb     the same for the business keyboard
 *                     (mechanical SHIFT LOCK, survives reset)
 *  -lrebasic4         emulator helper: hold a key at every reset until BOOT
 *                     has latched the set (start in set 1 / BASIC 4)
 */

#include "vice.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cartio.h"
#include "cartridge_banks.h"
#include "cmdline.h"
#include "crtc.h"
#include "crtctypes.h"
#include "crtc-draw.h"
#include "keyboard.h"
#include "lib.h"
#include "snapshot.h"
#include "log.h"
#include "maincpu.h"
#include "monitor.h"
#include "petlre.h"
#include "petmem.h"
#include "pets.h"
#include "resources.h"
#include "types.h"
#include "uiapi.h"
#include "util.h"

#define LRE_CARD_SIZE   0x10000

static log_t petlre_log = LOG_DEFAULT;

int petlre_enabled = 0;
static char *petlre_card_name = NULL;
static int petlre_shiftlock = 0;

static char *petlre_char_name = NULL;

static uint8_t *card_rom = NULL;    /* 64K */
static uint8_t *char_rom = NULL;    /* 64K */
static int char_loaded = 0;
static uint8_t lre_sram[0x8000];
static int lre_gmode = 0;           /* GAL GMODE latch  */
static int lre_attren = 0;          /* GAL ATTREN latch */

static uint8_t lre_read(uint16_t addr);
static void lre_store(uint16_t addr, uint8_t value);
static void lre_update_mapping(void);
static int card_loaded = 0;
static int card_set = 0;            /* EPROM A15 */
static int petlre_bootset = 0;      /* emulator helper: hold a key until BOOT latches the set */
static int bootkey_held = 0;

/* ------------------------------------------------------------------------- */
/* card ROM */

static int load_card(const char *name)
{
    if (card_rom == NULL) {
        card_rom = lib_malloc(LRE_CARD_SIZE);
    }
    memset(card_rom, 0xff, LRE_CARD_SIZE);
    card_loaded = 0;
    if (name == NULL || *name == 0) {
        return 0;
    }
    if (util_file_load(name, card_rom, LRE_CARD_SIZE, UTIL_FILE_LOAD_RAW) < 0) {
        log_error(petlre_log, "Could not load card image '%s'.", name);
        return -1;
    }
    card_loaded = 1;
    log_message(petlre_log, "Loaded card image '%s'.", name);
    return 0;
}

static int card_covers(uint16_t addr)
{
    if (!petlre_enabled) {
        return 0;
    }
    if (addr < 0xa000) {
        return 0;
    }
    if ((addr & 0xff00) == 0xe800) {
        return 0;
    }
    return 1;
}

static uint8_t card_read(uint16_t addr)
{
    return card_rom[(card_set << 15) | (addr & 0x7fff)];
}

static void card_store_void(uint16_t addr, uint8_t value)
{
    /* ROM area: the data '245 is off, the mainboard sees nothing */
}

static void bootkey_release(void);

static void card_store_ff(uint16_t addr, uint8_t value)
{
    int set = value & 1;

    bootkey_release();

    if (set != card_set) {
        log_message(petlre_log, "card set %d -> %d (write $%02X to $%04X, PC=$%04X)",
                    card_set, set, value, addr, (unsigned int)reg_pc);
        card_set = set;
        ramsel_changed();
    }
}

#if USE_MMU_TRANSLATE
void petlre_override_9tof(read_func_ptr_t *mem_read_tab,
                          store_func_ptr_t *mem_write_tab,
                          uint8_t **mem_read_base_tab,
                          int *mem_read_limit_tab)
#else
void petlre_override_9tof(read_func_ptr_t *mem_read_tab,
                          store_func_ptr_t *mem_write_tab)
#endif
{
    int i;

    if (!petlre_enabled || petres.model.crtc) {
        return;
    }
    /* RAM board: $8000-$8FFF always, $9000-$9FFF in graphics mode */
    for (i = 0x80; i < (lre_gmode ? 0xa0 : 0x90); i++) {
        mem_read_tab[i] = lre_read;
        mem_write_tab[i] = lre_store;
#if USE_MMU_TRANSLATE
        mem_read_base_tab[i] = NULL;
        mem_read_limit_tab[i] = 0;
#endif
    }
    for (i = 0xa0; i <= 0xff; i++) {
        if (i == 0xe8) {
            continue;
        }
        mem_read_tab[i] = card_read;
        mem_write_tab[i] = (i == 0xff) ? card_store_ff : card_store_void;
#if USE_MMU_TRANSLATE
        mem_read_base_tab[i] = NULL;
        mem_read_limit_tab[i] = 0;
#endif
    }
}

/* ------------------------------------------------------------------------- */
/* RAM board */

static void lre_draw(uint8_t *p, int xstart, int xend, int scr_rel, int ymod8);

/* SRAM address of a CPU access, or -1 if the board is not selected */
static int lre_cpu_addr(uint16_t addr, int *is_reg)
{
    int sel8 = (addr & 0xf000) == 0x8000;
    int sel9 = (addr & 0xf000) == 0x9000;
    int reg, o;

    *is_reg = 0;
    if (!(sel8 || (sel9 && lre_gmode))) {
        return -1;
    }
    reg = ((sel8 && !lre_gmode) || (sel9 && lre_gmode)) && ((addr & 0x0f80) == 0x0f80);
    *is_reg = reg;
    if (lre_gmode) {
        o = addr & 0x1fff;                  /* VA0-9 + BA10-12, SA13 = ATTR = 0 */
        return (o >> 3) | ((o & 7) << 10);
    }
    /* text: A10-12 pulled down, A13 = SA13 = !REG, A14 = ATTR = SEL8 & BA11 */
    return (addr & 0x3ff) | (reg ? 0 : 0x2000) | ((addr & 0x0800) ? 0x4000 : 0);
}

static uint8_t lre_read(uint16_t addr)
{
    int r;
    int a = lre_cpu_addr(addr, &r);

    if (a < 0) {
        return (uint8_t)(addr >> 8);        /* open bus, as read_unused */
    }
    return lre_sram[a];
}

static void lre_update_mapping(void)
{
    ramsel_changed();
    crtc_set_hires_draw_callback(petlre_enabled ? lre_draw : NULL);
}

static void lre_store(uint16_t addr, uint8_t value)
{
    int r;
    int a = lre_cpu_addr(addr, &r);

    if (a < 0) {
        return;
    }
    lre_sram[a] = value;                    /* SRAM_WE = PHI2 & !RW */
    if (r) {
        /* GAL rev 05: two windows, only one active at a time.
           text  $8F80 "ON":  bit0 = 1 -> graphics on, bit1 = ATTREN
           gfx   $9F80 "OFF": bit0 = 1 -> graphics off, bit1 ignored
           a 0 in bit 0 never changes the mode */
        int g = lre_gmode, at = lre_attren;

        if (!lre_gmode) {
            at = (value >> 1) & 1;
            if (value & 1) {
                g = 1;
            }
        } else if (value & 1) {
            g = 0;
        }
        if (g != lre_gmode || at != lre_attren) {
            log_message(petlre_log, "register $%04X <- $%02X: GMODE %d -> %d, ATTREN %d -> %d (PC=$%04X)",
                        addr, value, lre_gmode, g, lre_attren, at, (unsigned int)reg_pc);
            lre_attren = at;
            if (g != lre_gmode) {
                lre_gmode = g;
                lre_update_mapping();
            }
        }
    }
}

static int video_covers(uint16_t addr)
{
    if (!petlre_enabled) {
        return 0;
    }
    return (addr & 0xf000) == 0x8000 || ((addr & 0xf000) == 0x9000 && lre_gmode);
}

/* Side effect free read of the CPU view where the LRE replaces the
   mainboard (monitor, screen readout).  Returns 1 and sets *value if the
   address belongs to the card ROM or the RAM board, else 0. */
int petlre_peek(uint16_t addr, uint8_t *value)
{
    if (card_covers(addr)) {
        *value = card_read(addr);
        return 1;
    }
    if (video_covers(addr)) {
        *value = lre_read(addr);
        return 1;
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* monitor banks via the bank registry: lresram00 (RAM board SRAM, 32K
   mirrored) and lrecard00 (card EPROM image, set 0 at $0000, set 1 at
   $8000) */

static uint8_t bank_sram_read(unsigned int addr)
{
    return lre_sram[addr & 0x7fff];
}

static void bank_sram_write(unsigned int addr, uint8_t value)
{
    lre_sram[addr & 0x7fff] = value;
}

static uint8_t bank_card_read(unsigned int addr)
{
    return card_rom != NULL ? card_rom[addr & 0xffff] : 0xff;
}

static cart_bank_info_t lre_sram_bank_info = {
    "lresram",      /* prefix */
    1,              /* num_banks */
    bank_sram_read,
    bank_sram_write,
    NULL            /* next -- managed by registry */
};

static cart_bank_info_t lre_card_bank_info = {
    "lrecard",      /* prefix */
    1,              /* num_banks */
    bank_card_read,
    NULL,           /* read only */
    NULL            /* next -- managed by registry */
};

static void lre_register_banks(int on)
{
    if (on) {
        cartridge_bank_register(&lre_sram_bank_info);
        cartridge_bank_register(&lre_card_bank_info);
    } else {
        cartridge_bank_unregister(&lre_sram_bank_info);
        cartridge_bank_unregister(&lre_card_bank_info);
    }
}

/* ------------------------------------------------------------------------- */
/* monitor I/O register dumps */

static int video_dump(void *context, uint16_t addr)
{
    /* short lines: the SDL monitor of xpet is 40 columns wide */
    mon_out("RAM board  GMODE %d  ATTREN %d\n", lre_gmode, lre_attren);
    mon_out("mode     : %s\n", lre_gmode ? "graphics (bitmap $8000)" : "text");
    mon_out("register : %s\n", lre_gmode ? "$9F80 OFF (bit0=1: text)"
                                       : "$8F80 ON (bit0=1: gfx)");
    mon_out("last wr  : $8F80=$%02X $9F80=$%02X\n",
            (unsigned int)lre_sram[0x4380], (unsigned int)lre_sram[0x1f80 >> 3]);
    mon_out("GRAPHIC  : %d (VIA CA2)\n", crtc.chargen_offset ? 1 : 0);
    mon_out("charrom  : %s\n", char_loaded ? "loaded" : "(none)");
    mon_out("bank lresram00: 0000 gfx 2000 text\n");
    mon_out("              4000 shadow 6000 attr\n");
    return 0;
}

/* raster: called by the CRTC code after the standard text line */
static void lre_draw(uint8_t *p, int xstart, int xend, int scr_rel, int ymod8)
{
    uint32_t *pw = (uint32_t *)p;
    int graphic = crtc.chargen_offset ? 1 : 0;
    int ra = ymod8 & 7;
    int i;

    if (ymod8 >= 8) {
        return;
    }
    for (i = xstart; i < xend; i++) {
        int cell = (scr_rel + (i - xstart)) & 0x3ff;
        int d;

        if (lre_gmode) {
            d = lre_sram[(ra << 10) | cell];
        } else {
            int code = lre_sram[0x2000 | cell];
            int attr = lre_attren ? (lre_sram[0x6000 | cell] & 0x0f) : 0;

            d = char_rom[ra | ((code & 0x7f) << 3) | (graphic << 10)
                         | ((code >> 7) << 11) | (attr << 12)];
        }
        *pw++ = dwg_table[d >> 4];
        *pw++ = dwg_table[d & 0x0f];
    }
}

static int load_char(const char *name)
{
    if (char_rom == NULL) {
        char_rom = lib_malloc(LRE_CARD_SIZE);
    }
    memset(char_rom, 0x00, LRE_CARD_SIZE);
    char_loaded = 0;
    if (name == NULL || *name == 0) {
        return 0;
    }
    if (util_file_load(name, char_rom, LRE_CARD_SIZE, UTIL_FILE_LOAD_RAW) < 0) {
        log_error(petlre_log, "Could not load character EPROM '%s'.", name);
        return -1;
    }
    char_loaded = 1;
    log_message(petlre_log, "Loaded character EPROM '%s'.", name);
    return 0;
}

static int card_dump(void *context, uint16_t addr)
{
    /* short lines: the SDL monitor of xpet is 40 columns wide */
    mon_out("CPU card %s, ROM set %d\n",
            petlre_enabled ? "on" : "off", card_set);
    mon_out("image    : %s\n", card_loaded ? "loaded" : "(none)");
    mon_out("A000-E7FF E900-FFFF: card ROM\n");
    mon_out("  set %d -> image offset $%04X\n", card_set,
            (unsigned int)((card_set << 15) | 0x2000));
    mon_out("E800-E8FF: mainboard I/O\n");
    mon_out("SHIFTLOCK: %s\n", petlre_shiftlock == 1 ? "locked (graphics)" :
                               petlre_shiftlock == 2 ? "locked (business)" : "released");
    mon_out("bank lrecard00: 0000 set 0, 8000 set 1\n");
    return 0;
}

void petlre_ioreg_add_list(struct mem_ioreg_list_s **list)
{
    if (!petlre_enabled) {
        return;
    }
    mon_ioreg_add_list(list, "LRE-VIDEO", 0x8f80, 0x8f80, video_dump, NULL, IO_MIRROR_NONE);
    mon_ioreg_add_list(list, "LRE-CARD", 0xff00, 0xff00, card_dump, NULL, IO_MIRROR_NONE);
}

/* ------------------------------------------------------------------------- */
/* SHIFT LOCK: hold left SHIFT in the matrix */

static void apply_shiftlock(void)
{
    /* release both possible positions, then press the selected one.
       1 = graphics keyboard left SHIFT (row 8, col 0),
       2 = business keyboard left SHIFT (row 6, col 0) */
    keyboard_inject_key(8, 0, petlre_shiftlock == 1);
    keyboard_inject_key(6, 0, petlre_shiftlock == 2);
}

/* Boot helper: the host UI drops held keys around a reset, so emulate
   "hold a key while resetting": press left SHIFT at reset, release it as
   soon as BOOT writes the set latch ($FFxx). */
static void bootkey_press(void)
{
    if (petlre_bootset && petlre_enabled) {
        keyboard_inject_key(8, 0, 1);
        bootkey_held = 1;
    }
}

static void bootkey_release(void)
{
    if (bootkey_held) {
        bootkey_held = 0;
        if (petlre_shiftlock != 1) {
            keyboard_inject_key(8, 0, 0);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* resources */

static int set_petlre_enabled(int value, void *param)
{
    int val = value ? 1 : 0;

    if (val == petlre_enabled) {
        return 0;
    }
    if (val && petres.model.crtc) {
        ui_error("Cannot enable LRE: requires a PET without CRTC (2001/3000).");
        return -1;
    }
    petlre_enabled = val;
    card_set = 0;
    lre_gmode = lre_attren = 0;
    lre_register_banks(val);
    lre_update_mapping();
    return 0;
}

static int set_petlre_card_name(const char *val, void *param)
{
    if (util_string_set(&petlre_card_name, val)) {
        return 0;
    }
    return load_card(petlre_card_name);
}

static int set_petlre_shiftlock(int value, void *param)
{
    if (value < 0 || value > 2) {
        return -1;
    }
    petlre_shiftlock = value;
    apply_shiftlock();
    return 0;
}

static int set_petlre_bootset(int value, void *param)
{
    petlre_bootset = value ? 1 : 0;
    return 0;
}

static int set_petlre_char_name(const char *val, void *param)
{
    if (util_string_set(&petlre_char_name, val)) {
        return 0;
    }
    return load_char(petlre_char_name);
}

static const resource_string_t resources_string[] = {
    { "PETLRECharName", "", RES_EVENT_NO, NULL,
      &petlre_char_name, set_petlre_char_name, NULL },
    { "PETLRECardName", "", RES_EVENT_NO, NULL,
      &petlre_card_name, set_petlre_card_name, NULL },
    RESOURCE_STRING_LIST_END
};

static const resource_int_t resources_int[] = {
    { "PETLRE", 0, RES_EVENT_SAME, NULL,
      &petlre_enabled, set_petlre_enabled, NULL },
    { "PETLREShiftLock", 0, RES_EVENT_NO, NULL,
      &petlre_shiftlock, set_petlre_shiftlock, NULL },
    { "PETLREBootSet", 0, RES_EVENT_NO, NULL,
      &petlre_bootset, set_petlre_bootset, NULL },
    RESOURCE_INT_LIST_END
};

int petlre_resources_init(void)
{
    if (card_rom == NULL) {
        card_rom = lib_malloc(LRE_CARD_SIZE);
        memset(card_rom, 0xff, LRE_CARD_SIZE);
    }
    if (char_rom == NULL) {
        char_rom = lib_malloc(LRE_CARD_SIZE);
        memset(char_rom, 0x00, LRE_CARD_SIZE);
    }
    if (resources_register_string(resources_string) < 0) {
        return -1;
    }
    return resources_register_int(resources_int);
}

void petlre_resources_shutdown(void)
{
    lib_free(petlre_card_name);
    petlre_card_name = NULL;
    lib_free(card_rom);
    card_rom = NULL;
    lib_free(petlre_char_name);
    petlre_char_name = NULL;
    lib_free(char_rom);
    char_rom = NULL;
}

static const cmdline_option_t cmdline_options[] =
{
    { "-petlre", SET_RESOURCE, CMDLINE_ATTRIB_NONE,
      NULL, NULL, "PETLRE", (resource_value_t)1,
      NULL, "Enable the PET 2001 LRE card" },
    { "+petlre", SET_RESOURCE, CMDLINE_ATTRIB_NONE,
      NULL, NULL, "PETLRE", (resource_value_t)0,
      NULL, "Disable the PET 2001 LRE card" },
    { "-lrechar", SET_RESOURCE, CMDLINE_ATTRIB_NEED_ARGS,
      NULL, NULL, "PETLRECharName", NULL,
      "<Name>", "Specify name of the LRE character EPROM image (64K)" },
    { "-lrecard", SET_RESOURCE, CMDLINE_ATTRIB_NEED_ARGS,
      NULL, NULL, "PETLRECardName", NULL,
      "<Name>", "Specify name of the LRE card ROM image (64K, two 32K sets)" },
    { "-lreshiftlock", SET_RESOURCE, CMDLINE_ATTRIB_NONE,
      NULL, NULL, "PETLREShiftLock", (resource_value_t)1,
      NULL, "Hold SHIFT LOCK down (graphics keyboard)" },
    { "-lreshiftlockb", SET_RESOURCE, CMDLINE_ATTRIB_NONE,
      NULL, NULL, "PETLREShiftLock", (resource_value_t)2,
      NULL, "Hold SHIFT LOCK down (business keyboard)" },
    { "+lreshiftlock", SET_RESOURCE, CMDLINE_ATTRIB_NONE,
      NULL, NULL, "PETLREShiftLock", (resource_value_t)0,
      NULL, "Release SHIFT LOCK" },
    { "-lrebasic4", SET_RESOURCE, CMDLINE_ATTRIB_NONE,
      NULL, NULL, "PETLREBootSet", (resource_value_t)1,
      NULL, "Hold a key at every reset until BOOT selects set 1 (BASIC 4)" },
    { "+lrebasic4", SET_RESOURCE, CMDLINE_ATTRIB_NONE,
      NULL, NULL, "PETLREBootSet", (resource_value_t)0,
      NULL, "Do not hold a key at reset (set 0, BASIC 2)" },
    CMDLINE_LIST_END
};

int petlre_cmdline_options_init(void)
{
    return cmdline_register_options(cmdline_options);
}

/* ------------------------------------------------------------------------- */

void petlre_init(void)
{
    petlre_log = log_open("PETLRE");
}

void petlre_powerup(void)
{
    int i;

    card_set = 0;
    lre_gmode = lre_attren = 0;
    for (i = 0; i < 0x8000; i++) {
        lre_sram[i] = (uint8_t)lib_unsigned_rand(0, 255);
    }
}

void petlre_reset(void)
{
    /* the LRE needs a PET without CRTC; switching the model to a CRTC
       machine while the LRE was on turns it off here */
    if (petlre_enabled && petres.model.crtc) {
        log_message(petlre_log, "model with CRTC selected: LRE switched off.");
        petlre_enabled = 0;
        card_set = 0;
        lre_gmode = lre_attren = 0;
        bootkey_held = 0;
        lre_register_banks(0);
        crtc_set_hires_draw_callback(NULL);
        ramsel_changed();
        return;
    }
    /* /RES clears the set latch */
    if (card_set != 0) {
        log_message(petlre_log, "reset: card set %d -> 0", card_set);
    }
    if (lre_gmode || lre_attren) {
        log_message(petlre_log, "reset: GMODE %d -> 0, ATTREN %d -> 0", lre_gmode, lre_attren);
    }
    card_set = 0;
    lre_gmode = lre_attren = 0;
    if (petlre_enabled) {
        lre_update_mapping();
    }
    apply_shiftlock();
    bootkey_held = 0;
    bootkey_press();
}

void petlre_shutdown(void)
{
    lre_register_banks(0);
}

/* ------------------------------------------------------------------------- */
/* snapshot: latches, 32K SRAM of the RAM board and both EPROM images, so a
   snapshot is complete without the image files */

#define LRE_SNAP_MAJOR 1
#define LRE_SNAP_MINOR 0
static const char snap_module_name[] = "PETLRE";

int petlre_snapshot_write_module(snapshot_t *s)
{
    snapshot_module_t *m;

    if (!petlre_enabled) {
        return 0;           /* no module: older VICE versions stay compatible */
    }
    m = snapshot_module_create(s, snap_module_name, LRE_SNAP_MAJOR, LRE_SNAP_MINOR);
    if (m == NULL) {
        return -1;
    }
    if (SMW_B(m, (uint8_t)card_set) < 0
        || SMW_B(m, (uint8_t)lre_gmode) < 0
        || SMW_B(m, (uint8_t)lre_attren) < 0
        || SMW_B(m, (uint8_t)petlre_shiftlock) < 0
        || SMW_B(m, (uint8_t)petlre_bootset) < 0
        || SMW_B(m, (uint8_t)(card_rom != NULL)) < 0
        || SMW_B(m, (uint8_t)(char_rom != NULL)) < 0
        || SMW_BA(m, lre_sram, sizeof lre_sram) < 0
        || (card_rom && SMW_BA(m, card_rom, LRE_CARD_SIZE) < 0)
        || (char_rom && SMW_BA(m, char_rom, LRE_CARD_SIZE) < 0)) {
        snapshot_module_close(m);
        return -1;
    }
    return snapshot_module_close(m);
}

int petlre_snapshot_read_module(snapshot_t *s)
{
    snapshot_module_t *m;
    uint8_t vmajor, vminor;
    uint8_t set, gm, at, sl, bs, have_card, have_char;

    m = snapshot_module_open(s, snap_module_name, &vmajor, &vminor);
    if (m == NULL) {
        /* snapshot without LRE: switch the card off */
        if (petlre_enabled) {
            resources_set_int("PETLRE", 0);
        }
        return 0;
    }
    if (vmajor != LRE_SNAP_MAJOR) {
        log_error(petlre_log, "Cannot load LRE snapshot module version %d.%d.", vmajor, vminor);
        snapshot_module_close(m);
        return -1;
    }
    if (SMR_B(m, &set) < 0 || SMR_B(m, &gm) < 0 || SMR_B(m, &at) < 0
        || SMR_B(m, &sl) < 0 || SMR_B(m, &bs) < 0
        || SMR_B(m, &have_card) < 0 || SMR_B(m, &have_char) < 0
        || SMR_BA(m, lre_sram, sizeof lre_sram) < 0) {
        snapshot_module_close(m);
        return -1;
    }
    if (have_card) {
        if (card_rom == NULL) {
            card_rom = lib_malloc(LRE_CARD_SIZE);
        }
        if (SMR_BA(m, card_rom, LRE_CARD_SIZE) < 0) {
            snapshot_module_close(m);
            return -1;
        }
        card_loaded = 1;
    }
    if (have_char) {
        if (char_rom == NULL) {
            char_rom = lib_malloc(LRE_CARD_SIZE);
        }
        if (SMR_BA(m, char_rom, LRE_CARD_SIZE) < 0) {
            snapshot_module_close(m);
            return -1;
        }
        char_loaded = 1;
    }
    snapshot_module_close(m);

    if (!petlre_enabled && resources_set_int("PETLRE", 1) < 0) {
        return -1;
    }
    card_set = set & 1;
    lre_gmode = gm & 1;
    lre_attren = at & 1;
    petlre_bootset = bs ? 1 : 0;
    petlre_shiftlock = (sl <= 2) ? sl : 0;
    bootkey_held = 0;
    apply_shiftlock();
    lre_update_mapping();
    log_message(petlre_log, "snapshot: set %d, GMODE %d, ATTREN %d.", card_set, lre_gmode, lre_attren);
    return 0;
}
