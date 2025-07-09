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

/* Xnmr.h include file with data structures and so forth 
 *
 *  UBC Physics
 * April, 2000
 * 
 * written by: Carl Michal, Scott Nelson
 */

#ifndef XNMR_H
#define XNMR_H

#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include "buff.h"

#if GTK_MAJOR_VERSION == 2
#define GdkRGBA GdkColor
#endif

#define MAX_BUFFERS 50
//NUM_COLOURS should be n*4+1 (17,21,25 etc)
#define NUM_COLOURS 25
#define EXTRA_COLOURS 4


/* global variables */
extern GdkRGBA  colours[NUM_COLOURS+EXTRA_COLOURS];
extern phase_data_struct  phase_data;
extern add_sub_struct add_sub;
extern fitting_struct fit_data;
extern queue_struct queue;

extern GtkWidget  *phase_dialog,*freq_popup,*fplab1,*fplab2,*fplab3,*fplab4,*fplab5;
extern GtkAdjustment  *phase0_ad,*phase1_ad;
extern float  phase0,phase1;
extern float  phase20,phase21;
extern GdkCursor  *cursorclock;
extern char no_acq;

extern script_widget_type script_widgets;

/* function prototypes */

//void open_phase(dbuff *buff,int action,GtkWidget *widget);
void open_phase(GtkAction *action,dbuff *buff);

gint do_phase(float *source,float *dest,float phase0,float phase1,int npts);

gint phase_changed(GtkAdjustment *widget,gpointer *data);

gint pivot_set_event (GtkWidget *widget, GdkEventButton *event,dbuff *buff);

void cursor_normal(dbuff *buff);

void cursor_busy(dbuff *buff);

gint destroy_all(GtkWidget *widget, gpointer data);

void do_destroy_all();

gint hide_phase( GtkWidget *widget, GdkEvent  *event, gpointer data );

gint popup_msg( char* msg,char modal);
gint popup_msg_wrap( char* msg );

void draw_vertical(dbuff *buff,GdkRGBA  *col, float xvalf,int xvali);
GtkWidget * gtk_hbox_new_wrap(gboolean homo, gint spacing);
GtkWidget * gtk_vbox_new_wrap(gboolean homo, gint spacing);

#define RED NUM_COLOURS
#define GREEN NUM_COLOURS+1
#define BLUE NUM_COLOURS+2
#define WHITE ((NUM_COLOURS-1)/2)
#define BLACK NUM_COLOURS+3
// WHITE is (NUM_COLOURS-1)/2

#define CHECK_ACTIVE( var )   if (var->buffnum != current) make_active(var)

#include "platform.h"

#endif








