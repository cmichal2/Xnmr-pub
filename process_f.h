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

/* process_f.h
 *
 * header file for process.c 
 *
 * UBC Physics
 * April, 2000
 * 
 * written by: Scott Nelson, Carl Michal
 */

#ifndef PROCESS_F_H
#define PROCESS_F_H

#include <gtk/gtk.h>

#include "xnmr_types.h"

/*
 *  Globals
 */

extern process_data_t* acq_process_data;

#define PSRBMAX 18 

#define WISDOM_FILEF "fftw3f-wisdom"
#define WISDOM_FILE "fftw3-wisdom"

// max number of bits in the register for PSRB

/*
 *  Function Prototypes
 */

GtkWidget* create_process_frame();
GtkWidget* create_process_frame_2d();
GtkWidget* create_extras_frame();

/*
 *  Processing functions
 */
void load_wisdom();
void save_wisdom();
gint do_offset_cal( GtkWidget *widget, double *unused);
gint do_offset_cal_and_display( GtkWidget *widget, double *unused);

gint do_offset_cal_a( GtkWidget *widget, double *unused);
gint do_offset_cal_a_and_display( GtkWidget *widget, double *unused);

gint do_offset_cal_2D( GtkWidget *widget, double *unused);
gint do_offset_cal_2D_and_display( GtkWidget *widget, double *unused);

gint do_offset_cal_2D_a( GtkWidget *widget, double *unused);
gint do_offset_cal_2D_a_and_display( GtkWidget *widget, double *unused);

gint do_cross_correlate_mlbs( GtkWidget *widget, double *val);
gint do_cross_correlate_chu( GtkWidget *widget, double *val);
gint do_cross_correlate_frank( GtkWidget *widget, double *val);
gint do_cross_correlate( GtkWidget *widget, double *val);
gint do_cross_correlate_and_display( GtkWidget *widget, GtkAdjustment *val);

gint do_zero_imag(GtkWidget *widget, double *spare);
gint do_zero_imag_and_display(GtkWidget *widget, double *spare);

gint do_ft(GtkWidget *widget, double *spare);
gint do_ft_and_display(GtkWidget *widget, double *spare);

gint do_bft(GtkWidget *widget, double *spare);
gint do_bft_and_display(GtkWidget *widget, double *spare);


gint do_exp_mult( GtkWidget* widget, double *val);
gint do_exp_mult_and_display( GtkWidget* widget, GtkAdjustment *val);

gint do_gaussian_mult( GtkWidget* widget, double *val );
gint do_gaussian_mult_and_display( GtkWidget* widget, GtkAdjustment *val );

gint do_zero_fill(GtkWidget * widget,double *val);
gint do_zero_fill_and_display(GtkWidget * widget,GtkAdjustment *val);

gint do_left_shift(GtkWidget * widget,double *val);
gint do_left_shift_and_display(GtkWidget * widget,GtkAdjustment *val);

gint do_left_shift_2d(GtkWidget * widget,double *val);
gint do_left_shift_2d_and_display(GtkWidget * widget,GtkAdjustment *val);

gint do_truncate(GtkWidget * widget,double *val);
gint do_truncate_and_display(GtkWidget * widget,GtkAdjustment *val);

gint do_phase_and_display_wrapper( GtkWidget* widget, double *data );
gint do_phase_wrapper( GtkWidget* widget, double *data );

gint do_phase_2d_and_display_wrapper( GtkWidget* widget, double *data );
gint do_phase_2d_wrapper( GtkWidget* widget, double *data );

gint process_data( GtkWidget *widget, gpointer data ); 
  //Performs multiple processing operations
  //does processing based on which processing functions are active (boxes are ticked on panel)

// 2d routines
gint do_exp_mult_2d( GtkWidget* widget, double *val);
gint do_exp_mult_2d_and_display( GtkWidget* widget, GtkAdjustment *val);

gint do_gaussian_mult_2d( GtkWidget* widget, double *val);
gint do_gaussian_mult_2d_and_display( GtkWidget* widget, GtkAdjustment *val);

gint do_zero_fill_2d(GtkWidget * widget,double *val);
gint do_zero_fill_2d_and_display(GtkWidget * widget,GtkAdjustment *val);

gint do_ft_2d(GtkWidget *widget, double *spare);
gint do_ft_2d_and_display(GtkWidget *widget, double *spare);

gint do_mag_2d(GtkWidget *widget, double *spare);
gint do_mag_2d_and_display(GtkWidget *widget, double *spare);

gint unscramble_2d(GtkWidget *widget, double *spare);
gint unscramble_2d_and_display(GtkWidget *widget, double *spare);

gint do_hadamard1(GtkWidget *widget, double *spare);
gint do_hadamard1_and_display(GtkWidget *widget, double *spare);

gint do_hayashi1(GtkWidget *widget, double *spare);
gint do_hayashi1_and_display(GtkWidget *widget, double *spare);

gint phase_for_shear(GtkWidget *widget, double *val);
gint phase_for_shear_and_display(GtkWidget *widget, GtkAdjustment *spare);

gint plus_minus_to_hyper(GtkWidget *widget, double *val);
gint plus_minus_to_hyper_and_display(GtkWidget *widget, GtkAdjustment *spare);




gchar psrb(int bits,int init);
float get_frank_seq(int n); // here n is sqrt(sequence length)
float get_chu_seq(int n); // here n is the sequence length.


/*
 *  Visual methods
 */

gint update_active_process_data( GtkAdjustment *adj, int button );
  //Updates the processing data structure to reflect changes made by the user on the panel

void show_process_frame( process_data_t* process_set );
  //Updates the panel to display a particular set of processing options

#endif


