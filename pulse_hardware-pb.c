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

/* pulse_hardware.c
 *
 * This file specifies function prototype to communicate with the pulse programmer hardware
 * Xnmr software
 *
 * UBC Physics
 * April, 2000
 * 
 * written by: Scott Nelson, Carl Michal
 */
//#define TIMING
#include "param_utils.h"
#include "p_signals.h"
#include "shm_prog-pb.h"
#include "/usr/share/Xnmr/config/h_config-pb.h"
#include "/usr/share/Xnmr/config/pulse_hardware-pb.h"
#include "spinapi.h"

#include <sys/io.h>   //for ioperm routine
//#include <asm/io.h>   //for inb, outb, etc.
#include <stdio.h>    //for printf
#include <sys/time.h>
#include <unistd.h>


#define ATTN_PORT "/dev/usb/lp0"


/*
 *  Global Variables
 */



int stop_pulseblaster()

{
  // this does an init - hopefully its good enough to reset the board properly.
  // doesn't look like it!
  int i,rval;
  const char *errs;
  //  printf("in stop_pulseblaster\n");
  for (i=NUM_BOARDS-1;i>=0;i--){ // start the first one last.
    if (pb_select_board(i) != 0){
      errs = pb_get_error();
      printf("error selecting board to stop: %s\n",errs);
    }
    pb_set_clock((double) 100.);

    if (pb_stop() != 0){
      errs = pb_get_error();
      printf("error stopping board: %s\n",errs);
    }
    
    /*  if (pb_init() != 0){
	errs = pb_get_error();
	printf("error initing board (to reset): %s\n",errs);
	}*/
    pb_start_programming(PULSE_PROGRAM);
    pb_inst_pbonly(0x0,CONTINUE,0,100);
    pb_inst_pbonly(0x0,CONTINUE,0,100);
    pb_inst_pbonly(0x0,CONTINUE,0,100);

    // this turns off the gradients: sets to lowest value.
    printf("shutting off gradient\n");
    pb_inst_pbonly(0x7c4d, CONTINUE,0,100);
    pb_inst_pbonly(0x4e, CONTINUE,0,100);
    pb_inst_pbonly(0x8000, CONTINUE,0,100);
    pb_inst_pbonly(0x0,CONTINUE,0,100);
    pb_inst_pbonly(0x0,CONTINUE,0,100);
    pb_inst_pbonly(0x0,STOP,0,100);
    if (pb_stop_programming() != 0){
      errs = pb_get_error();
      printf("error in stop_prograimming: %s\n",errs);
      return -1;
    }
    rval = pb_start();
    if (  rval != 0){
      printf("in stop_pulseblaster, pb_start returned: %i\n",rval);
    }
    //    printf("STOP: closing board: %i\n",i);
    if (pb_close() != 0){
      errs = pb_get_error();
      //      printf("error closing board (post init): %s\n",errs);
    }
    
  }
  return 0;
}

int check_hardware_running(){

  const char *errs;
  int rval,i;
  int running = 1;

  // if a board is waiting, this currently says its not running.

  // now close out the board and read back status
  for (i=0;i<NUM_BOARDS;i++){
    if (pb_select_board(i) != 0){
      errs= pb_get_error();
      printf("error on select board: %s\n",errs);
      return -1;
    }
    rval = pb_read_status();
    //    printf("after start, board %i status is: %i\n",i,rval);
    if ((rval & 4) != 4 ) running = 0;
  }
  return running;

}


int init_pulse_hardware( int interrupt_port )
{
  int count,i;
  const char *errs;
  count = pb_count_boards();
  // check to make sure we have as many as we're supposed to
  if (count != NUM_BOARDS)
    printf("compiled with NUM_BOARDS = %i, but spinapi finds %i\n",NUM_BOARDS,count);
  for (i=0;i<NUM_BOARDS;i++){
    pb_select_board(i);
    //    pb_set_clock((double) 100.);
    if ( pb_init() != 0){
      errs = pb_get_error();
      printf("error initializing board: %s\n",errs);
      return -1;
    }
    pb_bypass_FF_fix(1); // disable the FF bypass bug, I don't think our
    // boards need it.
  }

  // enable interrupt reporting on parallel port.
  // this is done on the dsp port init.
  //  outb(0x10,interrupt_port+2);


  return count;
    
}

int pulse_hardware_send( struct prog_shm_t* program )
{


  int i,j;
  const char *errs;
#ifdef TIMING
  int byte_count=0;
  int overhead=0;
  struct timeval start_time,end_time;
  struct timezone tz;
  float d_time;
  gettimeofday(&start_time,&tz);

#endif


  for(i=0;i<NUM_BOARDS;i++){
    if (pb_select_board(i) != 0){
      printf("selecting board %i failed\n",i);
      return -1;
    }

    // don't know why we have to do this again...
    pb_set_clock((double) 100.);

    if (pb_start_programming(PULSE_PROGRAM) != 0){
      errs = pb_get_error();
      printf("error in start programming: %s\n",errs);
      return -1;
    }
    for (j=0;j<program->no_events;j++){
      //      printf("event: %i, outputs: %i\n",j,program->outputs[i][j]);
      if (pb_inst_pbonly(program->outputs[i][j],program->opcodes[i][j],
			 program->opinst[i][j],program->times[i][j]*1e9) < 0){
	errs = pb_get_error();
	printf("error: %s\n",errs);
	printf("error in programming board %i, event %i, opcode: %i, opinst: %i, time: %f\n",i,j,
	       program->opcodes[i][j],program->opinst[i][j],program->times[i][j]);
      }


    }
    if (pb_stop_programming() != 0){
      errs = pb_get_error();
      printf("error in stop_prograimming: %s\n",errs);
      return -1;
    }

  }// end of this board


#ifdef TIMING
    gettimeofday(&end_time,&tz);

  //overhead+=4;
    d_time=(end_time.tv_sec-start_time.tv_sec)*1e6+(end_time.tv_usec-start_time.tv_usec);
   fprintf(stderr,"download time: %.0f us\n",d_time);
   fprintf(stderr,"downloaded: %i+%i bytes to programmer in %.0f us, rate: %.3f MB/s\n",
	 byte_count,
       	 overhead, d_time,
	 (byte_count+overhead)/d_time);  
#endif


  return 0;
}


int pulse_hardware_start(int start_address)
{
  int i;
  const char * errs;
  int rval;
  
  for (i=NUM_BOARDS-1;i>=0;i--){

    if (pb_select_board(i) != 0){
      errs= pb_get_error();
      printf("error on select board: %s\n",errs);
      return -1;
    }
    rval = pb_read_status();
    //    printf("before start, board %i status is: %i\n",i,rval);

    if (pb_start() != 0){
      errs=pb_get_error();
      printf("error on board start: %s\n",errs);
      return -1;
    }
  }

  // now close out the board and read back status
  for (i=0;i<NUM_BOARDS;i++){
    if (pb_select_board(i) != 0){
      errs= pb_get_error();
      printf("error on select board: %s\n",errs);
      return -1;
    }
    rval = pb_read_status();
    //    printf("after start, board %i status is: %i\n",i,rval);
    /*    if (pb_close() != 0){
      errs=pb_get_error();
      printf("error on board close: %s\n",errs);
      //      return -1; // this seems to always error
      }*/
  }



  return 0;
}

int free_pulse_hardware(int interrupt_port)
{
  int i;
  const char *errs;
  for (i=0;i<NUM_BOARDS;i++){
    pb_select_board(i);
    //    printf("closing board: %i\n",i);
    if (pb_close() != 0){
      errs = pb_get_error();
      //      printf("error on board close: %s\n",errs);
    }
  }
  // disable interrupt reporting on parallel port - this is done in 
  // setup_dsp since our interrupt comes in on the dsp port
  //  outb(0x00,interrupt_port+2);
  return 0;
}




int set_attn(int gain){
  static int old_gain = -1;
  int pval;
  FILE *attn_port;


  //  if (gain == old_gain) return 0;


    if (gain > 31 ) gain = 31;
  if (gain <0) gain = 0;
  
  attn_port  = fopen(ATTN_PORT,"wb");
  if (attn_port == NULL) return -1;
  pval = (gain<<1);
  fwrite(&pval,1,1,attn_port);
  pval +=64;
  //  printf("writing %i to attn\n",pval);
  fwrite(&pval,1,1,attn_port);
  pval -= 64; // this hits the latch pin on the attenuator.
  //  printf("writing %i to attn\n",pval);
  fwrite (&pval,1,1,attn_port);
  // this takes about 20ms to set.  Wait for 50...
  fclose(attn_port);
  usleep(50*1000);
  old_gain = gain; 
  return 0;
}


