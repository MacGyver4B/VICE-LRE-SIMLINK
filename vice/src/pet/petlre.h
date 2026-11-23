/*
 * petlre.h - PET 2001 LRE (320x200 graphics, CPU socket card)
 *
 * This file is part of VICE, the Versatile Commodore Emulator.
 * See README for copyright notice.
 */

#ifndef VICE_PETLRE_H
#define VICE_PETLRE_H

#include "types.h"
#include "mem.h"
#include "snapshot.h"

extern int petlre_enabled;

int petlre_resources_init(void);
void petlre_resources_shutdown(void);
int petlre_cmdline_options_init(void);

void petlre_init(void);
void petlre_powerup(void);
void petlre_reset(void);
void petlre_shutdown(void);

/* memory map: called at the end of set_std_9tof() and petmem_set_vidmem() */
#if USE_MMU_TRANSLATE
void petlre_override_9tof(read_func_ptr_t *mem_read_tab,
                          store_func_ptr_t *mem_write_tab,
                          uint8_t **mem_read_base_tab,
                          int *mem_read_limit_tab);
#else
void petlre_override_9tof(read_func_ptr_t *mem_read_tab,
                          store_func_ptr_t *mem_write_tab);
#endif

/* monitor support */
int petlre_peek(uint16_t addr, uint8_t *value);
struct mem_ioreg_list_s;
void petlre_ioreg_add_list(struct mem_ioreg_list_s **list);

int petlre_snapshot_write_module(snapshot_t *s);
int petlre_snapshot_read_module(snapshot_t *s);

#endif
