/*

Copyright (c) 2025 Carl Michal with contributions of:
Scott Nelson
Phil Eles
Matt Grinder

This file is part of Xnmr.

Xnmr is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free
Software Foundation, either version 3 of the License, or (at your
option) any later version.

Xnmr is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
for more details.

You should have received a copy of the GNU General Public License
along with Xnmr. If not, see <https://www.gnu.org/licenses/>.

*/

/* panel.h
 *
 * Part of the Xnmr software project
 *
 * UBC Physics
 * April, 2000
 * 
 * written by: Scott Nelson, Carl Michal
 */

#ifndef PANEL_H
#define PANEL_H

#include <gtk/gtk.h>
#include "param_f.h"

#define MAX_DATA_NPTS (8388608*4) // 32Mpts
/*
 *  Global Variables
 */

volatile extern char acq_in_progress;
extern GtkWidget* start_button;
extern GtkWidget* start_button_nosave;
extern GtkWidget* repeat_button;
extern GtkWidget* repeat_p_button;
extern GtkWidget* acq_label;
//extern GtkWidget* acq_2d_label;
extern GtkWidget* time_remaining_label;
//extern GtkWidget* completion_time_label;
extern char no_acq;
extern int upload_buff;


/*
 *   Possible values of variable acq_in_progress
 */

#define ACQ_STOPPED 0
#define ACQ_RUNNING 1
#define ACQ_REPEATING 2
#define ACQ_REPEATING_AND_PROCESSING 3

gint kill_button_clicked( GtkWidget *widget, gpointer *data );
gint start_button_toggled( GtkWidget *widget, gpointer *data );

gint repeat_button_toggled( GtkWidget *widget, gpointer *data );

gint repeat_p_button_toggled( GtkWidget *widget, gpointer *data );

GtkWidget* create_panels();

void check_buff_size();


#endif
