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

/* param_f.h
 *
 * Xnmr Software Project
 *
 * UBC Physics
 * April, 2000
 *
 * written by: Scott Nelson, Carl Michal
 */

#ifndef PARAM_F_H
#define PARAM_F_H

#include <gtk/gtk.h>
#include "shm_data.h"
#include "param_utils.h"
#include "xnmr_types.h"

/*
 *  External Globals
 */

extern parameter_set_t* acq_param_set;
extern parameter_set_t* current_param_set;
extern int array_popup_showing;

/*
 * Function prototypes start here
 */

GtkWidget* create_parameter_frame( );
  //create the parameter frame and returns a pointer to the outer frame widget.
  //Only one parameter frame should be created because of the use of global variables.

void show_parameter_frame( parameter_set_t *param_set ,int npts); 
  //Displays the argument parameter set on screen and makes it active

/*
 * These methods update the parameter_set memory structures when the visual objects are manipulated
 */

gint update_paths( GtkWidget* widget, gpointer data );
void update_acqn( GtkAdjustment* adj, gpointer data );
void update_param( GtkAdjustment* adj, parameter_t* param );
void update_2d_buttons();
void update_2d_buttons_from_buff( dbuff* buff );

void update_sw_dwell();

void update_npts(int npts);
void update_npts2(int npts2);
void update_acqs(int acqs);
/*
 * These methods update the shared memory structure from the active parameter set
 */

void send_paths();
void send_acqns();
void send_params();

/*
 *
 */

int load_param_file( char* fileN, parameter_set_t *param_set );
gint param_spin_pressed( GtkWidget* widget, GdkEventButton* event, gpointer data );
void reload( GtkWidget* widget, gpointer data );

/*
 *  Function prototypes for 2d parameter pop up
 */

void create_2d_popup();                                //This does not display the popup, just creates it
void show_2d_popup( GtkWidget *widget, int *button );  //uses the active parameter set

/*
 *  Callbacks for 2d parameter popup
 */

void resize_popup( GtkAdjustment *adj, gpointer data  );
void apply_pressed( GtkWidget* widget, gpointer data );
void ok_pressed( GtkWidget* widget, gpointer data );
void unarray_pressed( GtkWidget* widget, gpointer data );
void array_cancel_pressed( GtkWidget* widget, gpointer data );
void clear_pressed( GtkWidget* widget, gpointer data );
void apply_inc_pressed( GtkWidget* widget, gpointer data );
void scramble_points( GtkWidget* widget, gpointer data );

void destroy_popup( GtkWidget *widget, gpointer data );
gint popup_no_update_ok(GtkWidget *widget,gpointer *dialog);
gint popup_no_update( char* msg );
gint am_i_queued(int test_bnum);
gint allowed_to_change(int test_bnum);

#endif












