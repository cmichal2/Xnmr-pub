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
