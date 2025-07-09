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

/* xnmr_ipc.h
 *
 * Part of the Xnmr software project
 *
 * UBC Physics
 * April, 2000
 * 
 * written by: Scott Nelson, Carl Michal
 */

#include "shm_data.h"
#include "buff.h"
#include <sys/time.h>
/*
 *  Global Variables
 */

extern struct data_shm_t* data_shm;


/*
 *  Function Prototypes
 */


int xnmr_init_shm();

//RETURNS 0 if acq not started, 1 if acq is already started

int init_ipc_signals();

void start_acq();

int wait_for_acq();

int release_shm();

void end_acq();

int send_sig_acq( char sig );

void set_acqn_labels(int start);

gint upload_and_draw_canvas( dbuff *buff  );

gint upload_and_draw_canvas_with_process( dbuff *buff  );

gint idle_queue( GtkWidget* button );

gint acq_signal_handler();

int upload_data( dbuff* buff );

gint release_ipc_stuff();

void last_draw();

gint draw_time_check();
