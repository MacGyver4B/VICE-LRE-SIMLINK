/** \file   settings_ieeesimlink.c
 * \brief   Settings widget for the simlink IEEE-488 bus participant
 *
 * Connects an external emulator (e.g. a microcontroller emulator running
 * petSD2 firmware) to the PET's IEEE-488 bus via shared memory.
 */

/*
 * $VICERES IEEESimlink      xpet
 * $VICERES IEEESimlinkStep  xpet
 */

/*
 * This file is part of VICE, the Versatile Commodore Emulator.
 * See README for copyright notice.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */

#include "vice.h"
#include <gtk/gtk.h>

#include "vice_gtk3.h"

#include "settings_ieeesimlink.h"


/** \brief  Create a left-aligned label
 *
 * \param[in]   text    label text (may contain Pango markup)
 *
 * \return  GtkLabel
 */
static GtkWidget *left_label(const char *text)
{
    GtkWidget *label = gtk_label_new(NULL);

    gtk_label_set_markup(GTK_LABEL(label), text);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_label_set_line_wrap(GTK_LABEL(label), TRUE);
    return label;
}


/** \brief  Create widget for the simlink settings
 *
 * \param[in]   parent  parent widget (unused)
 *
 * \return  GtkGrid
 */
GtkWidget *settings_ieeesimlink_widget_create(GtkWidget *parent)
{
    GtkWidget *grid;
    GtkWidget *name;
    GtkWidget *step;
    GtkWidget *help;

    grid = vice_gtk3_grid_new_spaced_with_label(VICE_GTK3_DEFAULT,
                                                VICE_GTK3_DEFAULT,
                                                "External IEEE-488 participant (simlink)",
                                                2);

    gtk_grid_attach(GTK_GRID(grid), left_label("Bus name"), 0, 1, 1, 1);
    name = vice_gtk3_resource_entry_new("IEEESimlink");
    gtk_widget_set_hexpand(name, TRUE);
    gtk_widget_set_tooltip_text(name,
            "Name of the shared memory bus. Empty: not connected. "
            "Press Enter to apply.");
    gtk_grid_attach(GTK_GRID(grid), name, 1, 1, 1, 1);

    gtk_grid_attach(GTK_GRID(grid), left_label("Sync step (cycles)"), 0, 2, 1, 1);
    step = vice_gtk3_resource_spin_int_new("IEEESimlinkStep", 1, 100000, 1);
    gtk_widget_set_halign(step, GTK_ALIGN_START);
    gtk_widget_set_tooltip_text(step,
            "Synchronise with the other participants at least every n CPU cycles.");
    gtk_grid_attach(GTK_GRID(grid), step, 1, 2, 1, 1);

    help = left_label("<i>Disable the built-in device on the unit served externally "
                      "(no drive, no file system device).</i>");
    gtk_grid_attach(GTK_GRID(grid), help, 0, 3, 2, 1);

    gtk_widget_show_all(grid);
    return grid;
}
