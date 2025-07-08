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
