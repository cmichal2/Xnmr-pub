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

/* pulse.h
 *
 * header file for pulse.c, NMR pulse program toolkit
 *
 * UBC Physics 
 *
 *
 */

/*
 *  These includes are so that the user pulse program can use certain #defined names 
 */

#include "shm_data.h"  //for PHASE flags
#include "/usr/share/Xnmr/config/h_config-duelime.h"
#include "p_signals.h"
#include <stdint.h>

#define MAX_TIME 85.899

/*
 *  Macros for loading parameters
 */

#define P0 0.
#define P90 90.
#define P180 180.
#define P270 270.


#define GET_PARAMETER_FLOAT( var ) fetch_float( #var, &var )
#define GET_PARAMETER_INT( var ) fetch_int( #var, &var )
#define GET_PARAMETER_TEXT( var ) fetch_text( #var, var )
#define GET_PARAMETER_DOUBLE( var ) fetch_double( #var, &var )



/*
 *  Private Function Prototypes - Do not call these directly from your pulse program
 *  Use the above macros to load parameters instead
 *
 *  These are wrapper functions for the param_utils module
 */

int fetch_float( char* name, float* var );
int fetch_int( char* name, int* var );
int fetch_text( char* name, char* var );
int fetch_double( char* name, double* var );
/*
 *  Public Function Prototypes - Use these and the above macros in your pulse program
 */

int pulse_program_init();
  // This function load the hardware configuration into 

int event_duelime( double time, int opcode,int opdata,unsigned char num, ... );
  // creates an event in the pulse program

int ready( char phase );
  // Sends a signal to the ACQ process that the pulse program data is ready to be downloaded
  // Also causes the pulse program to sleep until it is woken up again by the ACQ Process
  // a return of 0 indicates that the pulse program should continue normally.  A return
  // of -1 indicates an error or a termination signal and the pulse program should
  // shut itself down

void done();
  // Sends a signal to the ACQ process that the pulse program is completely done
  // also performs memory cleanup
  // call this function only just before you want to exit

int begin();
  //Call this at the beginning of every program iteration - important


/* for noisy... (not yet implemented in pulseblaster version*/

void pprog_is_noisy(); /* makes acq run differently - suitable for noise spectroscopy */
void start_noisy_loop(); /* marks point to return to for noisy */



/*
 *  Acessor funcitons for use from pulse programs:
 */

unsigned long get_acqn();

unsigned int get_acqn_2d();

unsigned long get_num_acqs();

unsigned int get_num_acqs_2d();

double get_dwell();

int get_npts();



void set_receiver_model_stream();

/* internal prototypes for pulse.c  Not for pulse program use. */
void pprog_internal_timeout();

void label_to_resolve(char *label); // gives a label that must later get translated back into an instruction #.
void insert_synth_event(int device_id,double dval,int num_split,int first_synth,int ev_no);
double partial_pp_time(int start,int end); // calculates the duration of a pulse program
double partial_pp_time_new(int start,int end); // calculates the duration of a pulse program
void do_insert_dsp_sync(int ev_no);

void set_tx_offset(int channel,double offset_freq);
void set_tx_gain(int channel,int gain);
void tx_init(int channel,double freq,double nco_freq,int gain);
void rx_init(int channel,double sw,int gain);

int hint_loop_start();
int hint_loop_end();
int hint_loop_over();

int write_bgu_raw(float x, float y, float z, int b0_flag);
int init_bgu(float alpha, float beta, float gamma);
void clear_bgu();
int set_gradients(float x,float y, float z, unsigned char num, ...);
uint32_t get_default_mask(int i);
int init_hardware();
int is_last_real_prog();
int is_first_real_prog();


// void pulse_hardware_build_startup_lime(due_prog_t *due_prog,int num_boards);
// just pasted into acq-duelime
