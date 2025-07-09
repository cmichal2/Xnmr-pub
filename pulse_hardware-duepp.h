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

/* pulse_hardware.h
 *
 * This file specifies function prototype to communicate with the pulse programmer hardware
 *
 *  Xnmr software project
 *
 * UBC Physics
 * April, 2000
 * 
 * written by: Scott Nelson, Carl Michal
 */

//#include "/usr/share/Xnmr/config/h_config-duelime.h"



/*
 *  Public Method prototypes
 */


int init_pulse_hardware(int num );
  //This method initializes the EPP Port at the given address

int pulse_hardware_send( due_prog_t * due_prog );
  // Sends a pulse program, the port must be initialized first with init_pulse_hardware

int pulse_hardware_start(char command);
  // Sets the pulse programmer running

int free_pulse_hardware();
  // Releases the resources allocated by init_pulse_hardware

int pulse_hardware_stop();

int pulse_hardware_wait();
int pulse_hardware_read_adc(int i); // read a due adc value.
