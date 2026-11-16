/** \file   settings_petlre.c
 * \brief   Settings widget for the PET 2001 LRE (CPU card + 320x200 RAM board)
 */

/*
 * $VICERES PETLRE          xpet
 * $VICERES PETLRECardName  xpet
 * $VICERES PETLRECharName  xpet
 * $VICERES PETLREShiftLock xpet
 * $VICERES PETLREBootSet   xpet
 */

/*
 * This file is part of VICE, the Versatile Commodore Emulator.
 * See README for copyright notice.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 *  02111-1307  USA.
 *
 */

#include "vice.h"
#include <gtk/gtk.h>

#include "vice_gtk3.h"

#include "settings_petlre.h"

/** \brief  File name patterns for the EPROM images */
static const char *rom_patterns[] = { "*.bin", "*.rom", NULL };

/** \brief  SHIFT LOCK choices */
static const vice_gtk3_radiogroup_entry_t shiftlock_entries[] = {
    { "Released",                         0 },
    { "Locked (graphics keyboard)",       1 },
    { "Locked (business keyboard)",       2 },
    VICE_GTK3_RADIOGROUP_ENTRY_LIST_END
};

/** \brief  Create widget to control PET LRE resources
 *
 * \param[in]   parent  parent widget (unused)
 *
 * \return  GtkGrid
 */
GtkWidget *settings_petlre_widget_create(GtkWidget *parent)
{
    GtkWidget *grid;
    GtkWidget *check;
    GtkWidget *card;
    GtkWidget *chr;
    GtkWidget *label;
    GtkWidget *shiftlock;
    GtkWidget *bootset;
    GtkWidget *note;

    grid = vice_gtk3_grid_new_spaced(VICE_GTK3_DEFAULT, VICE_GTK3_DEFAULT);

    check = vice_gtk3_resource_check_button_new("PETLRE",
            "Enable PET 2001 LRE (CPU card + 320x200 RAM board, PET without CRTC)");
    gtk_grid_attach(GTK_GRID(grid), check, 0, 0, 2, 1);

    card = vice_gtk3_resource_browser_new("PETLRECardName",
                                          rom_patterns, "EPROM images",
                                          "Select CPU card EPROM (27512)",
                                          "CPU card EPROM (64K):", NULL);
    gtk_grid_attach(GTK_GRID(grid), card, 0, 1, 2, 1);

    chr = vice_gtk3_resource_browser_new("PETLRECharName",
                                         rom_patterns, "EPROM images",
                                         "Select character EPROM (27C512)",
                                         "Character EPROM (64K):", NULL);
    gtk_grid_attach(GTK_GRID(grid), chr, 0, 2, 2, 1);

    label = gtk_label_new("SHIFT LOCK (mechanical, survives reset):");
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_grid_attach(GTK_GRID(grid), label, 0, 3, 2, 1);
    shiftlock = vice_gtk3_resource_radiogroup_new("PETLREShiftLock",
                                                  shiftlock_entries,
                                                  GTK_ORIENTATION_VERTICAL);
    gtk_widget_set_margin_start(shiftlock, 16);
    gtk_grid_attach(GTK_GRID(grid), shiftlock, 0, 4, 2, 1);

    bootset = vice_gtk3_resource_check_button_new("PETLREBootSet",
            "Hold a key at every reset until BOOT has selected set 1 (BASIC 4)");
    gtk_grid_attach(GTK_GRID(grid), bootset, 0, 5, 2, 1);

    note = gtk_label_new("Card set, graphics mode and ATTREN are reset by a machine reset.");
    gtk_widget_set_halign(note, GTK_ALIGN_START);
    gtk_grid_attach(GTK_GRID(grid), note, 0, 6, 2, 1);

    gtk_widget_show_all(grid);
    return grid;
}
