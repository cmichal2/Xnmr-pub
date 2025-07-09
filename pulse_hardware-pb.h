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

#include "/usr/share/Xnmr/config/h_config-pb.h"

/*
 *  Pulse hardware addresses
 */

#define CNTRL_ADDR 32
#define AD0_ADDR 34
#define AD1_ADDR 35
#define PPO_ADDR 193



/*
 *  Pulse hardware control port commands
 */

#define RESET_ROTOR 255
#define RESET_ALL 239
#define START 227
#define LOAD_TIMER 237
#define TCET 235
#define OEN 231



/*
 *  Public Method prototypes
 */


int init_pulse_hardware( );
  //This method initializes the EPP Port at the given address

int pulse_hardware_send( struct prog_shm_t* program );
  // Sends a pulse program, the port must be initialized first with init_pulse_hardware

int pulse_hardware_start(int start_address);
  // Sets the pulse programmer running

int free_pulse_hardware();
  // Releases the resources allocated by init_pulse_hardware

int stop_pulseblaster();

int check_hardware_running();


int set_attn(int gain);
