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
#include "/usr/share/Xnmr/config/h_config.h"
#include "p_signals.h"

/*
 *  Macros for loading parameters
 */

#define P0 0.
#define P90 90.
#define P180 180.
#define P270 270.

#define SYNC_DSP go_back(get_dwell()*256,(float) 1./CLOCK_SPEED,1,AD_SYNC,1)

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

int event( double time, unsigned char num, ... );
  // creates an event in the pulse program
int go_back_one( float time, unsigned char num, ... );
int go_back( float time, float duration, unsigned char num, ... );

double set_freq1(double freq); 
double set_freq2(double freq); 
   // sets frequency synthesizer, freq in MHz, ret val is how much time used in program

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



/* these functions facilitate building pulse programs asynchronously.  Call store_position to 
   set an "anchor" where you want to return to in the program.  The call jump_to_stored to
   go back there, and return_to_present when you're done.

   No nesting - only one anchor.

   Don't go into the future...
*/

void store_position();

void jump_to_stored();
void return_to_present();
void jump_back_by(double time);

void pprog_internal_timeout();

void pprog_is_noisy(); /* makes acq run differently - suitable for noise spectroscopy */
void start_noisy_loop(); /* marks point to return to for noisy */


/*
 *  Acessor funcitons
 */

unsigned long get_acqn();

unsigned int get_acqn_2d();

unsigned long get_num_acqs();

unsigned int get_num_acqs_2d();

float get_dwell();

int get_npts();

unsigned int get_event_no();

int write_bgu_raw(float x, float y, float z, int b0_flag);
int init_bgu(float alpha, float beta, float gamma);
void clear_bgu();
int set_gradients(float x,float y, float z, unsigned char num, ...);

















