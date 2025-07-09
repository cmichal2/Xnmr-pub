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

//#define TIMING



/*  pulse.c 

// time of each event in a synth event

  * 
  * This module is to be used for writing NMR pulse programs that are 
  * controlled by a separate process called acq (see acq.c) 
  * 
  * UBC Physics 
  * 
  * This version uses a message queue for communication between the acq and 
  * pprog processes 
  *   
  * 
  * written by: Scott Nelson  and Carl Michal
  * 
  * Aug 24, 2004 - fixed bug in figuring out first and last chips in hardware parsing..

  Currently, the way things work:
  When you want to start a pulseprogram, acq forks.  The second fork execs into the pulse program, 
  which has to start with root privileges (which acq has - suid) in order to lock pages into memory.
  
  The sequence of events in the pulse program then is:
  pulse_program_init
  do{
     get parameters
     begin

     events

  }while (ready(phase) == FLAG);

  That's it.

  What should happen:
  
  after acq forks, it execs into a new, single permanent pulse program which would


  memlockall (acq takes care of the real time scheduling)
  give up root permissions
  opens the pulse program:

  do{
    call's the user's pprogram routine, which is basically what's 
    in-between the do{ }while above.
  }while(ready(phase) = flag)


  To do this, need to figure out how to compile loadable modules...


 DAC and ALT words
 -----------------

 This is pretty rudimentary.
 Every time you write any dac, both dacs on all due boards are written.
 It should remember the last value you set for devices that aren't used in that particular event.

 Similarly for alt - if you set the alt output word, its written for all due boards - should remember
 the last value you used. There is no concept of devices within the alt words (yet?)
 
  
   */
//#define DUEBGU // if there's a BGU, similar symbol in acq-duesdr

#define ERROR_DONE -1

#define PROG_TIME 200e-9 

#include "platform.h"
 #include <sys/mman.h> 
 #include <stdarg.h> 
 #include <string.h> 
 #include <stdio.h> 
 #include <signal.h> 
 #include <sys/shm.h> 
 #include <sys/ipc.h> 
 #include <sys/types.h> 
 #include <sys/msg.h> 
 #include <errno.h> 
 #include <stdlib.h>
 #include <sys/stat.h>
 #include <sys/resource.h>
 #include <math.h>
 #include <glib.h>
// #include "spinapi.h"

 #include "/usr/share/Xnmr/config/h_config-duesdr.h" 
#include "due-pp-lib.h"
 #include "pulse-duesdr.h" 
 #include "p_signals.h" 
 #include "shm_data.h" 
 #include "shm_prog-duesdr.h" 
 #include "param_utils.h" 
 #include "duepp.h"
 #include <unistd.h> 
 #include <sys/time.h> 
 #include <time.h>
#include "soapy-sdr.h"

/// XXX TODO temporary skew of clock timing to make due and sdr match. Remove !
//#define FUDGE (1+2.72e-5)
#define FUDGE (1)


#ifdef CYGWIN
struct msgbuf{
  long mtype;       /* message type, must be > 0 */
  char mtext[1];    /* message data */
};
#endif

 /* 
  *  global data structures used by this module 
  * 
  *  Note that these structures are hidden from the interface 
  * 
   */

 struct hardware_config_t { 
   int start_bit; 
   int num_bits; 
   int latch; 
   unsigned int def_val; 
   double max_time; 
   char name[UTIL_LEN];
   short start_board_num;
   short end_board_num;
   short board_bit;
   unsigned int load_mask;
 }; 

int num_dev=0;
char bgu_inited = 0; // have we called init_bgu?
char bgu_cleared = 0; // did we clear it?
char in_sub = 0;
char found_exit = 0;
int bgu_last_vals_zeros; 
float bgu_alpha,bgu_beta,bgu_gamma;
float bgu_xscale,bgu_yscale,bgu_zscale;

#define MAX_LABELS 12000
#define MAX_LABEL_LEN 20

due_prog_t old_progs[NUM_BOARDS];

// for loops, subroutines and branching:
// the first five hold info about labels pointing to events
char event_labels[MAX_LABELS][MAX_LABEL_LEN];
int event_numbers[MAX_LABELS];
#ifdef DUEBGU
int gradevent_numbers[MAX_LABELS];
#endif
int num_event_labels;

//the next five hold info about events that need to be resolved (eg JSR routines would have these)
char labels_to_resolve[MAX_LABELS][MAX_LABEL_LEN];
int events_to_resolve[MAX_LABELS];
int num_events_to_resolve;
#ifdef DUEBGU
int gradevents_to_resolve[MAX_LABELS];
#endif

struct hardware_config_t *hardware_config = NULL; 
struct data_shm_t* data_shm; 
struct prog_shm_t*  prog_shm;        //These are shared memory structures and must be initialized 
unsigned int latch_mask[NUM_BOARDS],default_mask[NUM_BOARDS];

struct itimerval mytime,old; 
int data_shm_id; 
int prog_shm_id; 
int msgq_id; 
int finished; 

int tran_table[8]; 


// a few prototypes
int resolve_labels();
uint64_t points_to_receive_new(int rxno);


 /* 
  *   Method Implementations 
   */



 /* 
  *  This method ensures that the pulse program exits normally even when sent a SIGTERM or SIGINT signal. 
  *  The finished flag indicates that the pulse program should stop looping and begin 
  *  it's exit sequence 
   */

uint32_t get_default_mask(int i){
  if (i > NUM_BOARDS || i < 0) return 0;
  else return default_mask[i];
}
  

void stop() { 
  struct msgbuf message; 
  
  finished = 1; 
  
  /* 
   * this will make sure the program does not get stuck waiting for a message of 
   * of type P_PROGRAM_CALC.  A race condition can arise if a SIGTERM or SIGINT is received  
   * after checking the finished flag, but before a call to msgrcv().  This would result in 
   * the program waiting indefinately for a message.  To correct it, we send an extra message. 
   * 
   * The extra message is removed in the method done() to prevent further complications. 
   */
  
  message.mtype = P_PROGRAM_CALC; 
  message.mtext[0] = P_PROGRAM_CALC;
  msgsnd ( msgq_id, &message, 1, 0 ); 
} 


void parameter_not_found( char *name){
  struct msgbuf message;
  int result;
  fprintf(stderr,"didn't find a value for parmeter: %s\n",name);

  fprintf(stderr,"Pulse Program timed out internally\n");
  
  // let acq know we have a problem
  message.mtype = P_PROGRAM_READY;
  message.mtext[0] = P_PROGRAM_PARAM_ERROR;

  result=msgsnd ( msgq_id, &message, 1, 0 );
  if (result == -1) perror("pulse.c:msgsnd");

  // and get out

  stop();

   //There might be an extra P_PROGRAM_CALC in the message queue from stop() - Remove it 

   msgrcv ( msgq_id, &message, 1, P_PROGRAM_CALC, IPC_NOWAIT ); 

   //   data_shm->pprog_pid = -1;  // don't set this to -1 so that acq knows what to wait() for
   shmdt( (char*) data_shm ); 
   shmdt( (char*) prog_shm ); 
   // fprintf(stderr, "pulse program terminated\n" ); 
   exit(1);  
   return; 

}


int wait_for_acq_msg( ) 
{ 

struct msgbuf message; 
   int result; 

   // fprintf(stderr, "Pulse Program waiting for msg from acq\n" ); 

   result = msgrcv( msgq_id, &message, 1, P_PROGRAM_CALC, MSG_NOERROR ); 
   //   fprintf(stderr,"pprog: received message\n");

   // so we only ever get the CALC message, but it could be sent internally to force us
   // out of waiting.  We could also be woken up by acq to quit.

   /* 
    * Now that we have been woken up by a mesage, check to see what this signal is. 
    * If a SIGINT or SIGTERM signal was recieved, result will be -1, indicating the program 
    * should exit.  Alternatively, the finished flag = 1 indicates that the message was sent by 
    * the stop() routine to break out to the race condition error.  In either case, returning 
    * P_PROGRAM_END will cause the pulse program to behave as if it had recieved a normal exit 
    * command. 
     */

   if( result < 0 || finished == 1)               
     return P_PROGRAM_END;                        
                                         
   switch ( message.mtype ) 
     { 
     case P_PROGRAM_CALC : 
       // fprintf(stderr, "pprog recieved message P_PROGRAM_CALC\n" ); 
       return P_PROGRAM_CALC; 

     case P_PROGRAM_END : 
       // fprintf(stderr, "pprog recieved message P_PROGRAM_END\n" ); 
       return P_PROGRAM_END; 
     } 

   fprintf(stderr, "pprog recieved an unusual message: %ld on a result of: %d\n", message.mtype, result ); 
return -1; 

 } 

 /* 
  *  write_device() does all the dirty work for event().  
  */


 int write_device( int  device_id, unsigned int val,int event_no ) 

 { 
   unsigned int i;
   unsigned int dum2;
   unsigned int *val_c,*mask_c;

   //   fprintf(stderr,"write_device: dev: %i, val: %i, event: %i,bit %i\n",device_id,val,event_no,hardware_config[device_id].start_bit);

   if (event_no <0 || event_no >= MAX_EVENTS){
     prog_shm->event_error = 1;
     fprintf(stderr,"write_device: got an event_no out of range\n");
     return -1;
   }
   if (device_id <0){
     fprintf(stderr,"write_device got device_id <0\n");
     return 0;
   }
   if (hardware_config[device_id].start_bit < 0 ){
     return 0; // return silently, we do this all the time for phoney devices
   }



   // put our value in dum2:
   dum2 = val << hardware_config[device_id].board_bit;

   val_c = (unsigned int *) &dum2;
   mask_c = (unsigned int *) &hardware_config[device_id].load_mask;

   // now load the bytes as needed
   for (i=0; i<= hardware_config[device_id].end_board_num-hardware_config[device_id].start_board_num;i++){

     prog_shm->outputs[i+hardware_config[device_id].start_board_num][event_no] = (val_c[i] & mask_c[i] )+
       ( ~mask_c[i] & prog_shm->outputs[i+hardware_config[device_id].start_board_num][event_no]);
   }

   /*   if (device_id == SLAVEDRIVER){
     printf("for SLAVEDRIVER, outputs:%i,dum2: %i, mask_c: %i\n", prog_shm->outputs[0][event_no],dum2,*mask_c);
     } */
   return 0; 

}

int is_first_real_prog(){
  return prog_shm->is_first_real_prog;
}

int is_last_real_prog(){
  return prog_shm->is_last_real_prog;
}

int ready( char phase){
 struct msgbuf message; 
 
 int  rval, err;
 int complete;
 int result;
 rval = guts_of_ready(phase,&complete, &err);

 if (complete){
   if (err == 0 && prog_shm->event_error == 0){
     //    printf("pulse.c: telling acq program is ready\n");
     message.mtype = P_PROGRAM_READY; 
     message.mtext[0] = P_PROGRAM_READY;
   }
   else{
     printf("pulse-duesdr.c: telling acq we have an error\n");
     message.mtype = P_PROGRAM_READY;
     message.mtext[0]=P_PROGRAM_ERROR;
   }
   result=msgsnd ( msgq_id, &message, 1, 0 ); 
   if (result == -1) perror("pulse.c:msgsnd"); 
   //   fprintf(stderr,"inside pulse, just sent P_PROGRAM_READY\n"); 
   
   result = wait_for_acq_msg( ); 
   //   fprintf(stderr,"inside pulse, just got message for CALC\n"); 
   prog_shm->prog_ready = NOT_READY; 
   return result;
 }
 else
   return rval;
 

}
int guts_of_ready( char phase, int *complete, int *err) { 
  int i,j; 
  //  static int first_time = 1;
  //  int err=0;
  *err = 0;
  *complete = 0;
#ifdef DUEBGU
  gradprog_t *gradprog;
#endif
  uint64_t ppt_in_tx,tx_period;
  uint32_t extra_ticks;
  volatile char done; // dummy for pp_time.
    
#ifdef TIMING
  struct timeval start_time,end_time;
  struct timezone tz;
  float d_time;
#endif
  
  // unset my timeout timer 
  
  mytime.it_interval.tv_sec = 0; 
  mytime.it_interval.tv_usec = 0; 
  mytime.it_value.tv_sec = 0; 
  mytime.it_value.tv_usec = 0; 
  setitimer( ITIMER_REAL, &mytime, &old ); 
  
#ifdef TIMING
  gettimeofday(&start_time,&tz);
#endif
  
  // resolve the labels:

  if (resolve_labels() != TRUE){
    printf("problem in resolving labels in pulse program\n");
    *err = P_PROGRAM_ERROR;
  }

  // New for SDR. Check length of program, make sure it is integral number
  // of SDR clock ticks.
  ppt_in_tx = pp_time_new(&done);
  // figure out smallest length that will integer multiple of both TX and RX
  // sample clocks. Increase duration of last event if necessary.
  // we need ppt_in_tx x rx_rate to be a multiple of DUE_PP_CLOCK
  // something like:

  tx_period=20; // This is a fallback, of 1 microsecond? Should be safe-ish.
#if(DUE_PP_CLOCK != 50000000)
#warn "DUE_PP_CLOCK is not expected. Fix pulse-duesdr.c"
#endif
#if(RX_SAMPLE_RATE == 10000000)
  // rx sample period is 100ns, tx clock is 20ns. program needs to be a multiple of both.
  // here its 100ns, that's 5 tx periods.
  tx_period = 5; // ppt, measured in tx periods, needs to be a multiple of this
#else
#warning "RX_SAMPLE_RATE rate is unknown. Fix pulse-duesdr.c to accomodate"
#endif
  extra_ticks = (tx_period - ppt_in_tx%tx_period)%tx_period;
  printf("program length was: %li,ppt_in_tx, increasing by: %i\n",ppt_in_tx,extra_ticks);
  ppt_in_tx += extra_ticks;
  if (4294967367296 - prog_shm->times[prog_shm->no_events-1] < extra_ticks){// we'll overflow it!
      *err= P_PROGRAM_ERROR;
      printf("pulse-duesdr: ready. Last event will overflow\n");
      // set length of rx program in shm:
  }    
  prog_shm->times[prog_shm->no_events-1] += extra_ticks;
  prog_shm->prog_dur_in_due_ticks = ppt_in_tx;
  prog_shm->prog_dur_in_rx_stamps = ppt_in_tx * RX_SAMPLE_RATE/DUE_PP_CLOCK; // which should be an integer
  // double check:
  if ( prog_shm->prog_dur_in_rx_stamps * DUE_PP_CLOCK != ppt_in_tx * RX_SAMPLE_RATE){
    printf("PROBLEM, pulse sequence length doesn't seem to be an even multiple of RX samples?\n");
    *err = P_PROGRAM_ERROR;
  }
  printf("program duration: tx says: %lf, rx says: %lf\n",ppt_in_tx/(double)DUE_PP_CLOCK,prog_shm->prog_dur_in_rx_stamps/(double)RX_SAMPLE_RATE);
  printf("pulse: prog_dur_in_rx_stamps: %li\n",prog_shm->prog_dur_in_rx_stamps);
  if(prog_shm->is_noisy == 0){
  
    if (pre_gen_rx_events() == -1){
      *err = P_PROGRAM_ERROR;
    }
  }
  else{
    if (pre_gen_rx_events_noisy() == -1){
      *err = P_PROGRAM_ERROR;
    }
  }  
  if (bgu_inited)
    if ( bgu_last_vals_zeros == 0){
      printf("bgu was inited, but last value wasn't all zeros! \n");
      *err = P_PROGRAM_ERROR;
    }

  //  printf("\nSTARTING TO ASSEMBLE DUE PROGRAM\n");
  // now go through and build the due program
  for (i=0;i<NUM_BOARDS;i++)
    if (due_init_program(&prog_shm->due_prog[i],1) != 0){
      *err = P_PROGRAM_ERROR;
      printf("trouble initing due_programs\n");
    }

  for (i=0;i<prog_shm->no_events;i++){
    for(j=0;j<NUM_BOARDS;j++){
      switch (prog_shm->opcodes[i]){
      case CONTINUE: // just a plain jane event.
	if (due_add_event(&prog_shm->due_prog[j],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  *err = P_PROGRAM_ERROR;
	  printf("trouble adding event to due program\n");
	}
	break;
      case DUE_DAC:
	if (due_write_dacs(&prog_shm->due_prog[j],prog_shm->opinst[i]>>(32*j) & 0xFFFF, prog_shm->opinst[i]>>(32*j+16) & 0xFFFF,prog_shm->outputs[j][i],prog_shm->times[i]/FUDGE) != 0){
	  *err = P_PROGRAM_ERROR;
	  printf("trouble adding dac event to due program\n");
	}
	break;
      case DUE_ALT:
	if (due_write_alt(&prog_shm->due_prog[j],prog_shm->opinst[i]>>(32*j) & 0xFFFFFFFF, prog_shm->outputs[j][i],prog_shm->times[i]/FUDGE) != 0){
	  *err = P_PROGRAM_ERROR;
	  printf("trouble adding alt event to due program\n");
	}
	break;
      case LOOP: // loop start
	if (due_start_loop(&prog_shm->due_prog[j],(unsigned int) prog_shm->opinst[i],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  *err = P_PROGRAM_ERROR;
	  printf("trouble adding event to due program\n");
	}
	break;
      case END_LOOP:
	if (due_end_loop(&prog_shm->due_prog[j],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  *err = P_PROGRAM_ERROR;
	  printf("trouble adding END_LOOP event to due program\n");
	}
	break;
      case JSR: // here we need a numeric identifier for the subroutine.
	//	printf("got JSR at event %i, to event: %i, index %i\n",i,prog_shm->opinst[j][i],prog_shm->subids[j][prog_shm->opinst[j][i]]);
	if (due_call_sub(&prog_shm->due_prog[j],(unsigned int) prog_shm->subids[prog_shm->opinst[i]],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  *err = P_PROGRAM_ERROR;
	  printf("trouble adding JSR event to due program\n");
	}
	break;
      case SUBSTART:
	//	printf("got SUBSTART at event %i. With index %i\n",i,prog_shm->subids[j][i]);
	if (due_start_sub(&prog_shm->due_prog[j],prog_shm->subids[i]) != 0){
	  *err = P_PROGRAM_ERROR;
	  printf("trouble calling due_start_sub\n");
	}
	if (due_add_event(&prog_shm->due_prog[j],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  *err = P_PROGRAM_ERROR;
	  printf("trouble adding SUBSTART event to due program\n");
	}
	break;
      case RTS:
	if (due_return_from_sub(&prog_shm->due_prog[j],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  *err = P_PROGRAM_ERROR;
	  printf("trouble adding RTS event to due program\n");
	}
	break;
      case WAIT:
	if (due_wait_for_trigger(&prog_shm->due_prog[j],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  *err = P_PROGRAM_ERROR;
	  printf("trouble adding trigger event to due program\n");
	}
	break;
      case WAIT_MAX:
	if (due_wait_for_trigger_max(&prog_shm->due_prog[j],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) !=0){
	  *err = P_PROGRAM_ERROR;
	  printf("trouble adding trigger_max event to due program\n");
	}
	break;
      case EXIT:
	if (due_add_event(&prog_shm->due_prog[j],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  *err = P_PROGRAM_ERROR;
	  printf("trouble adding exit event to due program\n");
	}
	if (due_exit_program(&prog_shm->due_prog[j]) != 0){
	  *err = P_PROGRAM_ERROR;
	  printf("trouble adding exit event to due program\n");
	}
	break;
      default:
	printf("building due program, got an unknown opcode!\n");
	*err = P_PROGRAM_ERROR;
      } // end switch on opcode

    }
  }
  for (i=0;i<NUM_BOARDS;i++)
    if (due_finalize_program(&prog_shm->due_prog[i]) != 0){
      *err = P_PROGRAM_ERROR;
      printf("trouble finalizing due program\n");
    }
  //  due_dump_program(&prog_shm->due_prog[0]);
  
  
  // due program is good to go.

  // deal with clean/dirty stuff;
  for (i=0;i<NUM_BOARDS;i++)
    prog_shm->board_clean[i] = CLEAN;  
  
  // calculate how many points each receiver will receive:
  //  printf("calling points_to_receive\n");
  prog_shm->totalpoints = 0;
  for(i=0;i<NUM_RX;i++){
    prog_shm->rxpoints[i] = points_to_receive_new(i);
    printf("setting rxpoints for thread %i to %li\n",i,prog_shm->rxpoints[i]);
    prog_shm->totalpoints += prog_shm->rxpoints[i];
  }

  if (prog_shm->is_noisy == 0) {
    if (prog_shm->totalpoints > data_shm->npts){
      *err = P_PROGRAM_ERROR;
      printf("FATAL ERROR: Pulse program found a total of %li points, but you've only set npts to %i\n",prog_shm->totalpoints,data_shm->npts);
    }
  }
  // if noisy, totalpoints should be a multiple of npts. What's the multiple? ick its complicated, worked out in acq...
  
  
  // if the number of events is different, then program is different.
  // also don't bother to compare if the program hasn't been downloaded, because it will need to be anyway.
   
  for (i=0;i<NUM_BOARDS;i++){
    if (old_progs[i].dpos != prog_shm->due_prog[i].dpos ) {
      prog_shm->board_clean[i] = DIRTY;
      continue;
    }
    if (memcmp(old_progs[i].data,prog_shm->due_prog[i].data,prog_shm->due_prog[i].dpos*4) != 0)
      prog_shm->board_clean[i] = DIRTY;
    if (prog_shm->downloaded == 0)
      prog_shm->board_clean[i] = DIRTY;
  }

  // this looks like where we should now generate the TX events.
  // so here we need to sift through the event table and from tx events generate tx events - or should we just do that on the fly?
  

#ifdef TIMING

  gettimeofday(&end_time,&tz);
  d_time=(end_time.tv_sec-start_time.tv_sec)*1e6+(end_time.tv_usec-start_time.tv_usec);
  fprintf(stderr,"compare time: %.0f us\n",d_time);


#endif


   //   fprintf(stderr,"coming into ready, num events is: %i\n",prog_shm->no_events);
  // start dump
  
  if (prog_shm->is_first_real_prog == 1){
    //Dump the pulse program to a file in two formats 
    
    FILE *fid; 
    int event; 
    int board; 
    int bit; 
    
    fprintf(stderr, "dumping pulse program to file\n" ); 
    fprintf(stderr,"in dumping, first_time is: %i\n",prog_shm->is_first_real_prog);
    fid = fopen( "pprog.txt", "w" ); 
    fprintf(fid,"event, opcode, opinst, time, output bits, next board, rx_on\n");
    for( event=0; event<prog_shm->no_events; event++ ) { 
      for( board=0; board<NUM_BOARDS; board++ ) {
	fprintf(fid,"%3i %i %2li %9i ",event,prog_shm->opcodes[event],prog_shm->opinst[event],prog_shm->times[event]);
	for( bit=0; bit<24; bit++ ) { 
	  fprintf( fid, "%d", (prog_shm->outputs[ board ][ event ] >> bit) %2 ); 
	}
	fprintf( fid, " " ); 
      }
      for(i=0;i<NUM_RX;i++)
	fprintf( fid, "%2i ", prog_shm->rx_on[i][event]);

      fprintf( fid, "\n" );       

    }
#ifdef DUEBGU
    if (bgu_inited){
      // finally dump the gradient program:
      gradprog = &prog_shm->gradprog[prog_shm->txprogno];
      fprintf(fid,"gradient program: event no, opcode, opinst, is_grad_event, x, y, z\n");
      for(i=0;i<gradprog->gradevents;i++)
	fprintf(fid,"%i %i %i %i %i %i %i\n",i,gradprog->gradopcode[i],gradprog->gradopinst[i],gradprog->is_grad_event[i],gradprog->x[i],gradprog->y[i],gradprog->z[i]);
    }
#endif
  fclose( fid ); 
  }
  

    // end dump

  // begun may not have been set - if we didn't actually do anything...
  prog_shm->begun = 0;
  prog_shm->prog_ready = READY; 
 
  //   if (first_time == 1){
  //     first_time =0;
  //     //     fprintf(stderr,"got first time, setting to 0\n");
  //   }
   
   /// XX TODO: is this the right place for this?
   if( finished == 1 ) {
     return P_PROGRAM_END; 
   }
   //   fprintf(stderr, "Pulse Program calculation complete\n" ); 
     //     phase = (4-phase)%4; // reverse the frequencies.

   //   fprintf(stderr, "pprog sending message P_PROGRAM_READY\n" ); 
   *complete = 1; // made it to the end, so return should do the message thing and return that value.
   return 0; 
} 



int write_device_wrap( int start_event_no,int end_event_no ,int device_id, int intval) 
     // val is a pointer here because is might be a float
     // return value is sum of 1,2,4 for amp/phase event for channels a b c
{

   int i;

   // set up special device numbers - done in pulse_prog_init
   if (device_id >= RF_OFFSET){ 
     //fprintf(stderr,"translating device: %i ",device_id);
     device_id = tran_table[device_id-RF_OFFSET];
     //fprintf(stderr,"to device: %i\n",device_id);
   }
 
   //   printf("going to write device: %i with value: %i\n",device_id,intval);
   for (i=start_event_no;i<=end_event_no;i++){
     
     write_device( device_id, intval,i); 
     
   }
 
   return 0;
}
#define NUM_SOFT_LOOPS 50
int soft_loops_start[NUM_SOFT_LOOPS];
int soft_loops_end[NUM_SOFT_LOOPS];
int soft_loops_iterations[NUM_SOFT_LOOPS];
#ifdef DUEBGU
int soft_loops_gradstart[NUM_SOFT_LOOPS];
int soft_loops_gradend[NUM_SOFT_LOOPS];
#endif
int num_soft_loops;
char ok_to_loop = 1;
/* Usage in pulse programs is something like this:
hint_loop_start();
for (i=0;i<N;i++){
   EVENT 
   EVENT
   EVENT
   EVENT
   hint_loop_end();
}
hint_loop_over();


so the start is only called once, but the end is called once per trip through the loop.
*/
int hint_loop_start(){
  //  int i;
  //  printf("hint start current event is: %i\n",prog_shm->no_events);
  if (num_soft_loops >= NUM_SOFT_LOOPS-1){
    printf("Got too many soft loops!\n");
    prog_shm->event_error = 1;
    return -1;
  }
  printf("hint_loop_start, start event is: %i\n",prog_shm->no_events);
  soft_loops_start[num_soft_loops] = prog_shm->no_events; // this for the next event
  soft_loops_iterations[num_soft_loops] = 0;
#ifdef DUEBGU
  soft_loops_gradstart[num_soft_loops] = prog_shm->gradprog[prog_shm->txprogno].gradevents;
#endif
  num_soft_loops += 1;
  return 0;
}
int hint_loop_end(){
  int i,j,loopno,events_in_loop;
#ifdef DUEBGU
  gradprog_t *gradprog;
  gradprog = &prog_shm->gradprog[prog_shm->txprogno];
#endif
  //printf("hint end current event is: %i\n",prog_shm->no_events);
  if (num_soft_loops < 1){
    printf("got a hint_loop_end with no start?\n");
    return -1;
  }
  loopno = num_soft_loops - 1;
  soft_loops_iterations[loopno] += 1;
  
  if (soft_loops_iterations[loopno] == 1){
    //    printf("hint_loop_end: first time through. recording end point of %i\n",prog_shm->no_events-1);
    soft_loops_end[loopno] = prog_shm->no_events-1; // this is the previous event.
#ifdef DUEBGU
    soft_loops_gradend[loopno] = gradprog->gradevents-1;
#endif
    return 0;
  }

  // first time finished.
  // ok, here we need to see if what we just did can be folded into a loop.
  if (soft_loops_iterations[loopno] == 2){ // first time we need to see if we can form a loop.
    ok_to_loop = 1;
    events_in_loop = soft_loops_end[loopno] - soft_loops_start[loopno]+1; // inclusive.
    // first check to make sure that the first and last events are just simple events that can be converted
    // into loop start and end
    if (prog_shm->opcodes[soft_loops_start[loopno]] != CONTINUE ){
      printf("in hint_loop_end - first event is not CONTINUE, can't convert to hard loop\n");
      ok_to_loop = 0;
    }
    if (prog_shm->opcodes[soft_loops_end[loopno]] != CONTINUE ){
      printf("in hint_loop_end - final event in loop is not CONTINUE, can't convert to hard loop\n");
      ok_to_loop = 0;
    }
    if (prog_shm->times[soft_loops_end[loopno]] < 25){
      printf("in hint_loop_end - time of last event is too short to convert to loop\n");
      ok_to_loop = 0;
    }
    if (prog_shm->times[soft_loops_start[loopno]] < 25){
      printf("in hint_loop_end - time of first event is too short to convert to loop\n");
      ok_to_loop = 0;
    }

    // ok, start and end are ok. See if everything is the same.
    for (i=0;i<events_in_loop;i++){// these are the events to check
      if (prog_shm->opcodes[i+soft_loops_start[loopno]] != prog_shm->opcodes[i+prog_shm->no_events-events_in_loop]){
	printf("hint end: got a mismatched opcode\n");
	ok_to_loop = 0;
      }
      if (prog_shm->opinst[i+soft_loops_start[loopno]] != prog_shm->opinst[i+prog_shm->no_events-events_in_loop]){
	printf("hint end: got a mismatched opinst\n");
	ok_to_loop = 0;
      }
      if (prog_shm->times[i+soft_loops_start[loopno]] != prog_shm->times[i+prog_shm->no_events-events_in_loop]){
	printf("hint end: got a mismatched time: %i vs %i at events: %i and %i\n",prog_shm->times[i+soft_loops_start[loopno]],prog_shm->times[i+prog_shm->no_events-events_in_loop],i+soft_loops_start[loopno],i+prog_shm->no_events-events_in_loop);
	ok_to_loop = 0;
      }
      for(j=0;j<NUM_RX;j++){
	if (prog_shm->rx_on[j][i+soft_loops_start[loopno]] != prog_shm->rx_on[j][i+prog_shm->no_events-events_in_loop]){
	  printf("hint end: rx on event mismatch\n");
	  ok_to_loop = 0;
	}
      }
      for(j=0;j<NUM_BOARDS;j++){
	if (prog_shm->outputs[j][i+soft_loops_start[loopno]] != prog_shm->outputs[j][i+prog_shm->no_events-events_in_loop]){
	  printf("hint end: got a mismatched output\n");
	  ok_to_loop = 0;
	}
      }
    }
    if (soft_loops_start[loopno] == soft_loops_end[loopno]){
      printf("loop has only 1 event, can't convert to hard loop. Were events collapsed?\n");
      ok_to_loop = 0;
    }
    // if we've made it here, we're good to go!
    // convert the first pass into a loop with 2 iterations:
    if (ok_to_loop){
      printf("Converting soft loop to hard loop\n");
      //    printf("start and end events are: %i and %i\n",soft_loops_start[loopno],soft_loops_end[loopno]);
      prog_shm->opcodes[soft_loops_start[loopno]] = LOOP;
      prog_shm->opinst[soft_loops_start[loopno]] = 2;
      prog_shm->opcodes[soft_loops_end[loopno]] = END_LOOP;
      prog_shm->no_events -= events_in_loop;
    }
    else printf("Not Converting soft loop to hard loop for due pp\n");
  } // finished 2nd iteration.
  else{
    ok_to_loop = 1;
    events_in_loop = soft_loops_end[loopno] - soft_loops_start[loopno]+1; // inclusive.
    // if we're here, we're on the 3rd or higher iteration.
    if (prog_shm->opcodes[soft_loops_start[loopno]] != LOOP ){ // check first event of first iteration
      //    printf("in hint_loop_end - first event is not LOOP_START\n");
      ok_to_loop = 0;
    }
    if (prog_shm->opcodes[soft_loops_end[loopno]] != END_LOOP ){// check last event of first iteration
      printf("in hint_loop_end - final event in loop is not CONTINUE, can't convert to hard loop\n");
      ok_to_loop = 0;
    }
    if (prog_shm->opcodes[prog_shm->no_events-events_in_loop] != CONTINUE ){ // check first event of most recent iteration
      printf("in hint_loop_end - first event is not LOOP_START\n");
      ok_to_loop = 0;
    }
    if (prog_shm->opcodes[prog_shm->no_events-1] != CONTINUE ){ // check last event of most recent iteration.
      printf("in hint_loop_end - final event in loop is not CONTINUE, can't convert to hard loop\n");
      ok_to_loop = 0;
    }
    // all other opcodes should match:
    for (i=1;i<events_in_loop-1;i++){// these are the events to check - skip first and last - just did them.
      if (prog_shm->opcodes[i+soft_loops_start[loopno]] != prog_shm->opcodes[i+prog_shm->no_events-events_in_loop]){
	printf("hint end: got a mismatched opcode\n");
	ok_to_loop = 0;
      }
      if (prog_shm->opinst[i+soft_loops_start[loopno]] != prog_shm->opinst[i+prog_shm->no_events-events_in_loop]){
	printf("hint end: got a mismatched opinst\n");
	ok_to_loop = 0;
      }
    }
    for(i=0;i<events_in_loop;i++){ // check them all.
      for(j=0;j<NUM_RX;j++){
	if (prog_shm->rx_on[j][i+soft_loops_start[loopno]] != prog_shm->rx_on[j][i+prog_shm->no_events-events_in_loop]){
	  printf("hint end: rx on event mismatch\n");
	  ok_to_loop = 0;
	}
      }
      for(j=0;j<NUM_BOARDS;j++){
	if (prog_shm->outputs[j][i+soft_loops_start[loopno]] != prog_shm->outputs[j][i+prog_shm->no_events-events_in_loop]){
	  printf("hint end: got a mismatched output\n");
	  ok_to_loop = 0;
	}
      }
      if (prog_shm->times[i+soft_loops_start[loopno]] != prog_shm->times[i+prog_shm->no_events-events_in_loop]){
	printf("hint end: got a mismatched time\n");
	ok_to_loop = 0;
      }
    }
    if (ok_to_loop){
      // add one more iteration to loop:
      prog_shm->opinst[soft_loops_start[loopno]] += 1;
      prog_shm->no_events -= events_in_loop;
    }
    else printf("Not adding one more iteration to loop for duesdr\n");
  }
#ifdef DUEBGU
////////////// Now grad:
  // ok, here we need to see if what we just did can be folded into a loop.
  if (soft_loops_iterations[loopno] == 2){ // first time we need to see if we can form a loop.
    events_in_loop = soft_loops_gradend[loopno] - soft_loops_gradstart[loopno]+1; // inclusive.
    ok_to_loop = 1;
    // first check to make sure that the first and last events are just simple events that can be converted
    // into loop start and end
    if (gradprog->gradopcode[soft_loops_gradstart[loopno]] != CONTINUE ){
      printf("in hint_loop_end - first event is not CONTINUE, can't convert to hard loop\n");
      ok_to_loop = 0;
    }
    if (gradprog->gradopcode[soft_loops_gradend[loopno]] != CONTINUE ){
      printf("grad in hint_loop_end - final event in loop is not CONTINUE, can't convert to hard loop\n");
      ok_to_loop = 0;
    }
    // for grad, doesn't matter if events are short.
    // ok, start and end are ok. See if everything is the same.
    for (i=0;i<events_in_loop;i++){// these are the events to check
      if (gradprog->gradopcode[i+soft_loops_gradstart[loopno]] != gradprog->gradopcode[i+gradprog->gradevents-events_in_loop]){
	printf("grad hint end: got a mismatched opcode\n");
	ok_to_loop = 0;
      }
      if (gradprog->gradopinst[i+soft_loops_gradstart[loopno]] != gradprog->gradopinst[i+gradprog->gradevents-events_in_loop]){
	printf("grad hint end: got a mismatched opinst %i vs %i at events: %i and %i\n",gradprog->gradopinst[i+soft_loops_gradstart[loopno]],gradprog->gradopinst[i+gradprog->gradevents-events_in_loop],i+soft_loops_gradstart[loopno],i+gradprog->gradevents-events_in_loop);
	ok_to_loop = 0;
      }
      if (gradprog->is_grad_event[i+soft_loops_gradstart[loopno]] != gradprog->is_grad_event[i+gradprog->gradevents-events_in_loop]){
	printf("grad hint end: got a mismatched is_grad_event\n");
	ok_to_loop = 0;
      }
      if (gradprog->x[i+soft_loops_gradstart[loopno]] != gradprog->x[i+gradprog->gradevents-events_in_loop]){
	printf("grad hint end: got a mismatched x\n");
	ok_to_loop = 0;
      }
      if (gradprog->y[i+soft_loops_gradstart[loopno]] != gradprog->y[i+gradprog->gradevents-events_in_loop]){
	printf("grad hint end: got a mismatched y\n");
	ok_to_loop = 0;
      }
      if (gradprog->z[i+soft_loops_gradstart[loopno]] != gradprog->z[i+gradprog->gradevents-events_in_loop]){
	printf("grad hint end: got a mismatched z\n");
	ok_to_loop = 0;
      }
    
      if (soft_loops_gradstart[loopno] == soft_loops_gradend[loopno]){
	printf("grad loop has only 1 event, can't convert to hard loop. Were events collapsed?\n");
	ok_to_loop = 0;
      }
    }
    // if we've made it here, we're good to go!
    // convert the first pass into a loop with 2 iterations:
    if (ok_to_loop){
      printf("grad: Converting soft loop to hard loop\n");
      //    printf("start and end events are: %i and %i\n",soft_loops_start[loopno],soft_loops_end[loopno]);
      gradprog->gradopcode[soft_loops_gradstart[loopno]] |= LOOP;
      gradprog->gradopinst[soft_loops_gradstart[loopno]] = 2;
      gradprog->gradopcode[soft_loops_gradend[loopno]] |= END_LOOP;
      gradprog->gradevents -= events_in_loop;
    }
    else printf("Not converting soft loop to hard loop for grad\n");
    
  }// finished 2nd iteration.
  // if we're here, we're on the 3rd or higher iteration.
  else{
    events_in_loop = soft_loops_gradend[loopno] - soft_loops_gradstart[loopno]+1; // inclusive.
    ok_to_loop = 1;
    if (gradprog->gradopcode[soft_loops_gradstart[loopno]] != LOOP ){ // check first event of first iteration
      //    printf("in hint_loop_end - first event is not LOOP_START\n");
      ok_to_loop = 0;
    }
    if (gradprog->gradopcode[soft_loops_gradend[loopno]] != END_LOOP ){// check last event of first iteration
      printf("grad in hint_loop_end - final event in loop is not END_LOOP, can't convert to hard loop\n");
      ok_to_loop = 0;
    }
    if (gradprog->gradopcode[gradprog->gradevents -events_in_loop]  != CONTINUE ){ // check first event of most recent iteration
      printf("grad in hint_loop_end - first event is not CONTINUE\n");
      ok_to_loop = 0;
    }
    if (gradprog->gradopcode[gradprog->gradevents-1] != CONTINUE ){ // check last event of most recent iteration.
      printf("grad in hint_loop_end - final event in loop is not CONTINUE, can't convert to hard loop\n");
      ok_to_loop = 0;
    }
    
    // all other opcodes should match:
    for (i=1;i<events_in_loop-1;i++){// these are the events to check - skip first and last - just did them.
      if (gradprog->gradopcode[i+soft_loops_gradstart[loopno]] != gradprog->gradopcode[i+gradprog->gradevents-events_in_loop]){
	printf("grad hint end: got a mismatched opcode\n");
	ok_to_loop = 0;
      }
      if (gradprog->gradopinst[i+soft_loops_gradstart[loopno]] != gradprog->gradopinst[i+gradprog->gradevents-events_in_loop]){
	printf("grad hint end: got a mismatched opinst\n");
	ok_to_loop = 0;
      }
    }
    
    for (i=0;i<events_in_loop;i++){// these are the events to check - skip first and last - just did them.
      // x y z
      if (gradprog->is_grad_event[i+soft_loops_gradstart[loopno]] != gradprog->is_grad_event[i+gradprog->gradevents-events_in_loop]){
	printf("grad hint end: got a mismatched is_grad_event\n");
	ok_to_loop = 0;
      }
      if (gradprog->x[i+soft_loops_gradstart[loopno]] != gradprog->x[i+gradprog->gradevents-events_in_loop]){
	printf("grad hint end: got a mismatched x\n");
	ok_to_loop = 0;
      }
      if (gradprog->y[i+soft_loops_gradstart[loopno]] != gradprog->y[i+gradprog->gradevents-events_in_loop]){
	printf("grad hint end: got a mismatched y\n");
	ok_to_loop = 0;
      }
      if (gradprog->z[i+soft_loops_gradstart[loopno]] != gradprog->z[i+gradprog->gradevents-events_in_loop]){
	printf("grad hint end: got a mismatched z\n");
	ok_to_loop = 0;
      }
    }
    
    // add one more iteration to loop:
    if (ok_to_loop){
      gradprog->gradopinst[soft_loops_gradstart[loopno]] += 1;
      gradprog->gradevents -= events_in_loop;
    }
    else printf("Not adding one more iteration for grad\n");
  }// end 3rd + higher iteration.
#endif
  //////////////
  return 1;
}
int hint_loop_over(){

  if (num_soft_loops < 1){
    printf("hint_loop_over, but not in soft loop?\n");
    return -1;
  }
  num_soft_loops -= 1;
  return 1;
}
int do_event_duesdr(double time, int opcode, int opinst, unsigned char num, va_list args ){
  // this is wrapped by event_duesdr so we take the va_list as an argument here.
  // DUE_ALT and DUE_DAC opcodes can't be used coming in here. But we may use them in the shm.
  va_list myargs;
  int i,j,intval=0; 
  unsigned char device_id; 
  double dval;//,dval2;
  int num_split,synth_event_start = -1;
  int has_label = 0;
  int first_synth = 0;
  int syntha=0,synthb=0;
  int old_opcode;
  uint64_t itime;
  int is_dac = 0, is_alt = 0;
  int min_ticks;
  //int ratio;
  va_copy(myargs,args);
#ifdef DUEBGU
  gradprog_t *gradprog;
  gradprog = &prog_shm->gradprog[prog_shm->txprogno];
#endif
  //  fprintf(stderr,"\ncoming into event, number is: %i\n",prog_shm->no_events);
  if (prog_shm->begun == 0 ){
    fprintf(stderr,"problem in pulse program.  Got an event before begin()\n");
    prog_shm->event_error = 1;
    return -1;
  }
  
  if( prog_shm->no_events >= MAX_EVENTS-1 ) { 
    fprintf(stderr, "pprog: Maximum number of events exceeded\n" ); 
    prog_shm->event_error = 1;
    return -1; 
  }
#ifdef DUEBGU
  //   fprintf(stderr,"event_no: %i time: %lf \n",prog_shm->no_events,time);
  if (gradprog->gradevents >= MAX_GRAD_EVENTS-1){
    prog_shm->event_error = 1;
    printf("Too many grad events!\n");
    return -1;
  }
#endif


   // shortest event is 200ns, longest (without being a long event is 2^32 * 20ns

   if (!isnormal(time)){
     if (time != 0.){
       fprintf(stderr,"TIME IS NOT A NUMBER!\n");
       prog_shm->event_error = 1;
       return -1;
     }
   }
   if(time < 0){
     fprintf(stderr,"event: time < 0, ignored\n");
     fprintf(stderr,"time was: %lg\n",time);
     return 0;      
   }
   // measure time in 20ns ticks.
   itime = (uint64_t) ((time+19.9e-9)*DUE_PP_CLOCK);
   if (itime < 10){
     itime = 10;
     fprintf(stderr,"event time >0 but less than 200ns, increased to 200ns\n");
   }

   // here, we're going to track everything in DUE clock cycles.
   // When we want rx events, translate to stamps as best we can - will always round up


   //All the splitting and stuff to do synthesizer pre-events goes in here.

   ////////////////////
   // loop through events to see what's what.
   
   
   for (i=0;i<num;i++){
     device_id = (unsigned char) va_arg(args,int);
     //     printf("first pass, event: %i, device is: %i, %i\n",prog_shm->no_events,i,  device_id);
     if (device_id == LABEL){// just a label - this one won't be copied to all the events...
	    //       char *lab;
       has_label = 1;
       //       lab = (char *) va_arg(args,char *);
       va_arg(args,char *);
     }
     else if (device_id == FREQA){
       syntha += 6;
       dval = (double) va_arg(args,double);
     }       
     else if (device_id == FREQB){
       synthb += 6;
       dval = (double) va_arg(args,double);
     }
     else if (device_id == AMP1){
       if ( data_shm->ch1 == 'A')
	 syntha += 2;
       else if (data_shm->ch1 == 'B')
	 synthb += 2;
       dval = (double) va_arg(args,double);
     }
     else if (device_id == AMP2){
       if (data_shm->ch2 == 'A')
	 syntha += 2;
       else if (data_shm->ch2 == 'B')
	 synthb += 2;
       dval = (double) va_arg(args,double);
     }
     else if (device_id == PHASE1){
       if (data_shm->ch1 == 'A')
	 syntha += 2;
       else if (data_shm->ch1 == 'B')
	 synthb += 2;
	 
       dval = (double) va_arg(args,double);
     }
     else if (device_id == PHASE2){
       if (data_shm->ch2 == 'A')
	 syntha += 2;
       else if (data_shm->ch2 == 'B')
	 synthb += 2;
       dval = (double) va_arg(args,double);
     }
     else if (device_id == DACA1){
       intval =  va_arg(args,unsigned int);
       is_dac = 1;
       // blank out low 16 bits:
       prog_shm->last_dacA &= 0xFFFF0000;
       // insert value:
       prog_shm->last_dacA |= (intval & 0xFFFF);
     }
     else if (device_id == DACB1){
       intval =  va_arg(args,unsigned int);
       is_dac = 1;
       // blank out low 16 bits:
       prog_shm->last_dacB &= 0xFFFF0000;
       // insert value:
       prog_shm->last_dacB |= (intval & 0x0000FFFF);
     }
     else if (device_id == DACA2){
       intval =  va_arg(args,unsigned int);
       is_dac = 1;
       // blank out high 16 bits:
       prog_shm->last_dacA &= 0x0000FFFF;
       // insert value:
       prog_shm->last_dacA |= (intval & 0xFFFF)<<16 ;
     }
     else if (device_id == DACB2){
       intval =  va_arg(args,unsigned int);
       is_dac = 1;
       // blank out high 16 bits:
       prog_shm->last_dacB &= 0x0000FFFF;
       // insert value:
       prog_shm->last_dacB |= (intval & 0xFFFF)<<16;
     }
     else if (device_id == ALTA || (device_id == ALT1 &&  data_shm->ch1 == 'A') || (device_id == ALT2 &&  data_shm->ch2 == 'A')){
       intval =  va_arg(args,unsigned int);
       is_alt = 1;
       prog_shm->last_altA = intval;
     }
     else if (device_id == ALTB || (device_id == ALT1 &&  data_shm->ch1 == 'B') || (device_id == ALT2 &&  data_shm->ch2 == 'B')){
       intval =  va_arg(args,unsigned int);
       is_alt = 1;
       prog_shm->last_altB = intval;
     }
     else {// don't care for now...
       intval = va_arg(args,unsigned int);
     }
   }
   if (is_alt && is_dac){
     printf("Can't set alt and set dac in same event\n");
     return -1;
   }
   if (is_alt || is_dac){
     if(opcode != CONTINUE){
       printf("opcode must be CONTINUE to set alt or dac\n");
       return -1;
     }
     if (itime <25){
       printf("For dac or alt set, minimum time is 500 ns, increasing to 500ns\n");
       itime = 25;
     }
   }


   // DEAL WITH SPLITTING PREVIOUS EVENTS FOR SYNTH INSERTION
   // freqs have four registers, amps and phases just 2 on the AD9854
   // get one extra event to hit the update button at the end.
   // how many extra events do we need...
   num_split = 0;
   if (syntha >= synthb) num_split = syntha;
   else if (synthb > syntha) num_split = synthb;
   //   printf("num_split is: %i\n",num_split);
   syntha=0;
   synthb=0;

   // ok, synth events go into the end of the previous event.

   // this is only legal if the previous event is not EXIT
   // if the current event has a label or is a LOOP start, produce a warning - this may or may not be what the user wanted
   if (num_split > 0){
     if (opcode == LOOP){
       fprintf(stderr,"event: %i requests synthesizer setting, and is a LOOP start.  The synth setting will only "
	       "happen on the first pass through the loop!\n",prog_shm->no_events);
       has_label = 0;  // so we don't produce the second warning below.
     }
     
     if (has_label == 1) fprintf(stderr,"Event %i has a label and requests a synth setting."
				 " A BRANCH or JSR landing on this event will"
				 " not get the synth setting requested\n",prog_shm->no_events);
     for (i=0;i<NUM_BOARDS;i++){
       if(prog_shm->opcodes[prog_shm->no_events-1] == END_LOOP || prog_shm->opcodes[prog_shm->no_events-1] == JSR ||
	  prog_shm->opcodes[prog_shm->no_events-1] == RTS /* || prog_shm->opcodes[prog_shm->no_events-1] == BRANCH */){
	 fprintf(stderr,"event: %i requests a synth setting, but has an END_LOOP, BRANCH, RTS or JSR previous.  Allowed, but may not do what you want\n",prog_shm->no_events);
       }
     }
     if( prog_shm->opcodes[prog_shm->no_events-1] == EXIT){
       fprintf(stderr,"event: %i requests a synth setting, but has EXIT previous.  Not allowed\n",prog_shm->no_events);
       prog_shm->event_error = 1;
       return -1;
     }

     // so we should be legal to stick our synth event in.  Check timing of previous event:
     min_ticks = 10;
     if(prog_shm->opcodes[prog_shm->no_events-1] == DUE_DAC || prog_shm->opcodes[prog_shm->no_events-1] == DUE_ALT){
       min_ticks = 25;
     }
     if (10*num_split+min_ticks > prog_shm->times[prog_shm->no_events-1]){
       fprintf(stderr,"Time of %g too short for setting amps/phases/freqs, increasing to: %g\n",
	       prog_shm->times[prog_shm->no_events-1]/((double) DUE_PP_CLOCK),(10*num_split+25)/((double) DUE_PP_CLOCK));
       prog_shm->times[prog_shm->no_events-1]=10*num_split+25;
     }

     /*
     if (PROG_TIME*(num_split+1) > prog_shm->times[prog_shm->no_events-1]/((double) DUE_PP_CLOCK)){
       fprintf(stderr,"Time of %g too short for setting amps/phases/freqs, increasing to: %g\n",
	       prog_shm->times[prog_shm->no_events-1]/((double) DUE_PP_CLOCK),PROG_TIME*(num_split+1));
       prog_shm->times[prog_shm->no_events-1]=(PROG_TIME*(num_split+1)+19.9e-9)*DUE_PP_CLOCK;
     }
     */
     //make sure we have enough events left:
     if (prog_shm->no_events + num_split >= MAX_EVENTS-1){
       prog_shm->event_error = 1;
       fprintf(stderr,"Ran out of events!\n");
       return -1;
     }
     // if our event has a label to resolve, move the pointer num_split events forward.
     if(events_to_resolve[num_events_to_resolve-1] == prog_shm->no_events)
       events_to_resolve[num_events_to_resolve-1] += num_split;
     
     
     // now duplicate the previous event num_split times.
     for (j=0;j<num_split;j++){
       prog_shm->times[prog_shm->no_events+j] = (PROG_TIME+19.9e-9)*DUE_PP_CLOCK; // should just be 10
       prog_shm->opcodes[prog_shm->no_events+j] = CONTINUE;
       prog_shm->opinst[prog_shm->no_events+j] = 0;
       for(i=0;i<NUM_RX;i++)
	 prog_shm->rx_on[i][prog_shm->no_events+j] = prog_shm->rx_on[i][prog_shm->no_events-1];
       for(i=0;i<NUM_BOARDS;i++){
	 prog_shm->outputs[i][prog_shm->no_events+j] = prog_shm->outputs[i][prog_shm->no_events-1];
       }
     }
     prog_shm->times[prog_shm->no_events-1] -= ((uint64_t)  ((PROG_TIME+19.9e-9)*DUE_PP_CLOCK)) * num_split;
   
     // in here need to make sure that the num_split new events have UPD set to 0 
     // this actually is probably not necessary... since the previous event  shouldn't have an UPD in it...
     write_device_wrap(prog_shm->no_events,prog_shm->no_events+num_split-1,UPD_A,0);
#ifdef WR_B
     write_device_wrap(prog_shm->no_events,prog_shm->no_events+num_split-1,UPD_B,0);
#endif
     synth_event_start = prog_shm->no_events - 1;


     // if the previous was  BRANCH, JSR, RTS or END_LOOP, move the opcode and opinst to the last event, as well
     // as the label to resolve flag for it.
     old_opcode = prog_shm->opcodes[prog_shm->no_events-1];
     if(old_opcode == END_LOOP /* || old_opcode == BRANCH */ || old_opcode == JSR || old_opcode == RTS){
       prog_shm->opcodes[prog_shm->no_events+num_split-1] = prog_shm->opcodes[prog_shm->no_events-1];
       prog_shm->opinst[prog_shm->no_events+num_split-1] = prog_shm->opinst[prog_shm->no_events-1];
       
       prog_shm->opcodes[prog_shm->no_events-1] = CONTINUE;
       prog_shm->opinst[prog_shm->no_events-1] = 0;
       
       // if that previous event had a label to resolve (and is BRANCH-like opcode), move the pointer forward.
       for (j=0;j<num_events_to_resolve;j++)
	 if(events_to_resolve[j] == prog_shm->no_events-1)
	   events_to_resolve[j] += num_split;
     }
     
     
     prog_shm->no_events += num_split; // current event is now moved forward

   }



   //////////////////////
   // implement our event:   
      
   first_synth = 1; // we've used no synth events yet.
   //duplicate the previous event and apply latch mask & defaults
   if( prog_shm->no_events > 0 ) { 
     for( i=0; i<NUM_BOARDS; i++ ) { 
       prog_shm->outputs[ i ][ prog_shm->no_events  ] =  
	 (prog_shm->outputs[ i ][prog_shm->no_events-1] & latch_mask[i]) + 
	 (default_mask[i] & ~latch_mask[i]);
     }
   }
   else // just put in defaults:
     for(i=0;i<NUM_BOARDS; i++)
       prog_shm->outputs[i][prog_shm->no_events] = default_mask[i];
   for(i=0;i<NUM_RX;i++){
     prog_shm->rx_on[i][prog_shm->no_events] = 0;
   }
   //set all the specified device information 
   //   fprintf(stderr,"\nin event, no_events: %i,num things this event: %i\n",prog_shm->no_events,num);
   for( i=0; i<num; i++ ) { 
     device_id = (unsigned char) va_arg( myargs, int  );
     //     printf("event: %i, device: %i of %i devices\n",prog_shm->no_events,device_id,num);
     if (device_id == LABEL){ // its a pseudo device with just a label.
       // insert the label into the tables to be found later. Should only be SUBSTART that has labels.
       char *lab;
       lab = va_arg(myargs,char *);
       //       printf("got a label: %s at instruction: %i, index: %i\n",lab,prog_shm->no_events,num_event_labels[0]);
       strncpy(event_labels[num_event_labels],lab,MAX_LABEL_LEN);
       event_numbers[num_event_labels] = prog_shm->no_events;
#ifdef DUEBGU
       gradevent_numbers[num_event_labels] = gradprog->gradevents;
#endif
       if (num_event_labels > MAX_LABELS -1){
	 printf("too many event labels!\n");
	 prog_shm->event_error = 1;
       }
       else
	 num_event_labels += 1;
     }
     else if (device_id == RX1 ){  // if we ever have two channels this will need work.
       int chan;
       intval=  va_arg(myargs,int);
       if (data_shm->ch1 == 'A')
	 chan = 0;
       else{
	 if (NUM_RX > 1)
	   chan = 1;
	 else{
	   printf("requested RX1 on Channel B, but there is only 1 receiver?\n");
	   chan = 0;
	 }
       }
       if (intval)
	 prog_shm->rx_on[chan][prog_shm->no_events] = 1;
     }
     else if (device_id == RX2 ){  // if we ever have two channels this will need work.
       int chan;
       intval=  va_arg(myargs,int);
       if (data_shm->ch2 == 'A')
	 chan = 0;
       else{
	 if (NUM_RX > 1)
	   chan = 1;
	 else{
	   printf("requested RX2 on channel B, but there is only 1 receiver?\n");
	   chan = 1;
	 }
       }
       if(intval)
	 prog_shm->rx_on[chan][prog_shm->no_events] = 1;

     }
     else if (device_id == FREQA || device_id == FREQB || device_id == AMP1 || device_id == AMP2 || device_id == PHASE1 || device_id == PHASE2) {
       dval= (double) va_arg(myargs,double);
       //       printf("event, calling insert_synth_event with val: %f\n",dval);
       insert_synth_event(device_id,dval,num_split,first_synth,synth_event_start);
       if (first_synth == 1) first_synth = 0;
     }
     else{ // any ordinary pulse programmer device:
       //       fprintf(stderr,"got device_id: %i for event: %i\n",(int) device_id,prog_shm->no_events);
       intval =  va_arg(myargs,unsigned int);
       write_device_wrap(prog_shm->no_events,prog_shm->no_events,device_id,intval);
     }
   }

   
   // straightforward time setting.
   if (itime < 4294967367296 )
     prog_shm->times[prog_shm->no_events] = itime;
   else  if (opcode == 0 ){ // can't do a long event unless opcode comes in as 0
     // if this event is longer than the pulse prog timer can hold, split it up:
     printf("got a long event of time: %li at event: %i\n",itime,prog_shm->no_events);
     if (itime < 2* 4294967296 - 11 ){ // that number is 2^32
       prog_shm->opcodes[prog_shm->no_events] = 0;
       prog_shm->opinst[prog_shm->no_events] = 0;
       prog_shm->times[prog_shm->no_events]=itime-(429467296-11); // make sure it has at least 11 counts.
       prog_shm->no_events += 1;
       for(i=0;i<NUM_RX;i++){ // leave the rx event in the first slot, its duration field will take care of the ending.
	 prog_shm->rx_on[i][prog_shm->no_events] = prog_shm->rx_on[i][prog_shm->no_events-1];
	 if (prog_shm->rx_on[i][prog_shm->no_events-1] == 1)
	   printf("FOUND RX ON DURING A LONG EVENT. DID NOT EXPECT THIS TO EVER HAPPEN\n");
       }
       if (prog_shm->no_events >= MAX_EVENTS - 1) prog_shm->event_error = 1;
       prog_shm->times[prog_shm->no_events] = 429467296-11;
       prog_shm->opcodes[prog_shm->no_events] = 0;
       prog_shm->opinst[prog_shm->no_events] = 0;
       for (i=0;i<NUM_BOARDS;i++){
	 prog_shm->outputs[i][prog_shm->no_events] = prog_shm->outputs[i][prog_shm->no_events-1];
       }
     }
     else{
       printf("Got an event time longer than 2 full events. Someone should set up pulse-duesdr.c to build a loop here\n");
       printf("or you could change your pulse program to do a loop\n");
       prog_shm->event_error = 1;
       return -1;
     }
   }
   else{ // got a long time, but opcode not 0.
     prog_shm->event_error = 1;
     printf("Error, got a time greater than max allowed with an opcode of: %i\n",opcode);
     return -1;
   }    

  
   if (opcode == LOOP && opinst == 0){
     printf("Got a LOOP with argument of 0!\n");
     prog_shm->event_error = 1;
     return -1;
   }
   // the order of these checks matters!
   if (opcode == SUBSTART && in_sub != 0){
     printf("can't define a subroutine inside a subroutine!\n");
     prog_shm->event_error = 1;
   }
   if (opcode == SUBSTART) in_sub = 1;
   if (opcode == SUBSTART && found_exit == 0){
     printf("Must Exit before defining subroutine\n");
     prog_shm->event_error = 1;
   }
   if (found_exit == 1 && in_sub == 0 ){
     printf("Can't define events after exit unless in a subroutine\n");
     prog_shm->event_error = 1;
   }

   if (opcode == RTS) in_sub = 0 ;
   if (opcode == EXIT) found_exit += 1;

   
   // set the opcode and opinst -- this isn't needed if we split for long_delay
   // but we may have split for a synth event
   prog_shm->opcodes[prog_shm->no_events] = opcode;
   if (is_dac){
     prog_shm->opcodes[prog_shm->no_events] = DUE_DAC;
     prog_shm->opinst[prog_shm->no_events] = prog_shm->last_dacA | prog_shm->last_dacB<<32; // dacs for 2 boards, all in one.
   }
   if (is_alt){
     prog_shm->opcodes[prog_shm->no_events] = DUE_ALT;
     prog_shm->opinst[prog_shm->no_events] = prog_shm->last_altA | prog_shm->last_altB<<32; // alt outputs for 2 boards.
   }
   prog_shm->opinst[prog_shm->no_events] = opinst; // most of these are wrong now, but will get updated later by resolve
   prog_shm->no_events += 1;
#ifdef DUEBGU
   // grad events don't care about timing - they just want the flow structure (for subroutines and loops)
   if (opcode > 0 && opcode <= EXIT){ // ie, only if its LOOP, END_LOOP, JSR, SUBSTART, RTS, EXIT do we bother telling gradprog. Could leave SUBSTART out?
     gradprog->gradopcode[gradprog->gradevents] = opcode;
     gradprog->gradopinst[gradprog->gradevents] = opinst;
     gradprog->is_grad_event[gradprog->gradevents] = 0;
     gradprog->gradevents += 1;

     // if our event is an end_loop and previous is a start loop, remove them both:
     if(gradprog->gradopcode[gradprog->gradevents-1] == END_LOOP){
       if (gradprog->gradopcode[gradprog->gradevents-2] == LOOP)
	 // double check and make sure they're not gradevents:
	 if (gradprog->is_grad_event[gradprog->gradevents-1] == 0 && gradprog->is_grad_event[gradprog->gradevents -2] == 0)
	   gradprog->gradevents -= 2;
     }
   }
#endif
   // XX one problem with this approach is we can't separately collapse due and rx events...
   // though seems like we could, just need to do rx events first, then due events.
   // and here, rx events are collapsed after the entire sequence is calculated. Makes it hard
   // to collapse due events on the fly.
   
   return 0; 

}

int event_duesdr( double time, int opcode,int opinst,unsigned char num, ... ) 
 {
   // opinst here is an int. But in the shm its 64 bits to hold (multiple) dac and alt word values -
   // which come in as device values.
   int rval;
   va_list args;
   va_start(args,num);
   rval = do_event_duesdr(time,opcode,opinst,num,args);
   va_end(args);
   return rval;
 }



void get_freq_words(double freq,unsigned int *aval,unsigned int *aval2){


  //  double clkr = 60000000.;
  double clkt = 300000000.;

  unsigned long long ncot;
    
  /*
  if (freq > prog_shm->if_freq) freq = freq - prog_shm->if_freq;
  else freq = prog_shm->if_freq - freq;
  */
  
  
  if (freq > 130000000.){
    printf("asked for synth to produce a frequency > 130 MHz, %f\n",freq);
  }


  // that number is 2^48:
  ncot = rint(281474976710656ULL*freq/clkt); 

  *aval = ncot >> 32;  // take the 32 most significant bits here
  *aval2 = (unsigned long int) (ncot & 0xFFFFFFFF);// and the lower 32 bits here.
  ncot = ((unsigned long long) *aval << 32) + *aval2;
  /*  fprintf(stderr,"freq words: %u %u\n",*aval,*aval2); */
  //  fprintf(stderr,"asked for freq: %f, getting freq: %f\n",freq,clkt*ncot/((unsigned long long)1<<48));


}

void insert_synth_event(int device_id,double dval,int num_split,int first_synth,int ev_no){
  static int syntha_event_count=0;
#ifdef WR_B
  static int synthb_event_count=0;
#endif
  unsigned int aval,aval2;
  int aaval;
  double dval2;
  //  double fval;
  if (first_synth){
    syntha_event_count = 0;
#ifdef WR_B
    synthb_event_count = 0;
#endif
  }
  
  if (ev_no == -1){
    printf("got -1 as event to insert synth event at!!\n");
    prog_shm->event_error = 1;
    return;
  }

  if (device_id == FREQA) {
    // in here we need to insert our synth write events, plus the update at end
    // need to get the address values from AD9854 data sheet
    // need to get data values by calculating them,
    // hardware is set up to deliver a WR pulse on each toggle of our WR line.
    // always write to an even number of registers to that
    // we don't randomly write at some point with a branch or jsr or something...

    // tuning word = freq x 2^N / SYSCLK, here N is 48 and SYSCLK is 300MHz
    // but we'll want to confine our choices to frequencies that the receiver is capable of receiving,
    // where N=32 and SYSCLK = 60MHz.
    get_freq_words(dval,&aval,&aval2);

    write_device(ADD_A,4 ,ev_no+syntha_event_count);
    write_device(DAT_A,(aval>>8)&255 ,ev_no+syntha_event_count);
    write_device(WR_A,1,ev_no+syntha_event_count);

    write_device(ADD_A,5 ,ev_no+syntha_event_count+1);
    write_device(DAT_A,aval & 255 ,ev_no+syntha_event_count+1);
    write_device(WR_A,0,ev_no+syntha_event_count+1);

    write_device(ADD_A,6 ,ev_no+syntha_event_count+2);
    write_device(DAT_A,(aval2 >> 24)& 255 ,ev_no+syntha_event_count+2);
    write_device(WR_A,1,ev_no+syntha_event_count+2);

    write_device(ADD_A,7 ,ev_no+syntha_event_count+3);
    write_device(DAT_A,(aval2 >> 16) & 255 ,ev_no+syntha_event_count+3);
    write_device(WR_A,0,ev_no+syntha_event_count+3);

    write_device(ADD_A,8 ,ev_no+syntha_event_count+4);
    write_device(DAT_A,(aval2 >> 8) & 255 ,ev_no+syntha_event_count+4);
    write_device(WR_A,1,ev_no+syntha_event_count+4);

    write_device(ADD_A,9,ev_no+syntha_event_count+5);
    write_device(DAT_A,aval2 & 255 ,ev_no+syntha_event_count+5);
    write_device(WR_A,0,ev_no+syntha_event_count+5);

    write_device(UPD_A,1,ev_no+num_split);

    syntha_event_count += 6;
  }
#ifdef WR_B
  else if (device_id == FREQB){
    get_freq_words(dval,&aval,&aval2);

    write_device(ADD_B,4 ,ev_no+synthb_event_count);
    write_device(DAT_B,(aval >>8)&255,ev_no+synthb_event_count);
    write_device(WR_B,1,ev_no+synthb_event_count);

    write_device(ADD_B,5 ,ev_no+synthb_event_count+1);
    write_device(DAT_B,aval&255 ,ev_no+synthb_event_count+1);
    write_device(WR_B,0,ev_no+synthb_event_count+1);

    write_device(ADD_B,6 ,ev_no+synthb_event_count+2);
    write_device(DAT_B,(aval2>>24)&255 ,ev_no+synthb_event_count+2);
    write_device(WR_B,1,ev_no+synthb_event_count+2);

    write_device(ADD_B,7 ,ev_no+synthb_event_count+3);
    write_device(DAT_B,(aval2>>16)&255 ,ev_no+synthb_event_count+3);
    write_device(WR_B,0,ev_no+synthb_event_count+3);

    write_device(ADD_B,8 ,ev_no+synthb_event_count+4);
    write_device(DAT_B,(aval2>>8)&255 ,ev_no+synthb_event_count+4);
    write_device(WR_B,1,ev_no+synthb_event_count+4);

    write_device(ADD_B,9,ev_no+synthb_event_count+5);
    write_device(DAT_B,(aval2&255) ,ev_no+synthb_event_count+5);
    write_device(WR_B,0,ev_no+synthb_event_count+5);
    write_device(UPD_B,1,ev_no+num_split);
    synthb_event_count += 6;
  }
#endif
  else if ((device_id == AMP1 && data_shm->ch1 == 'A')|| (device_id == AMP2 && data_shm->ch2 == 'A')){
    if (dval < 0.0){
      dval = -dval;
      printf("got a negative amplitude.  Making positive\n");
    }
    if (dval > 1.0){
      dval = 1.0;
      printf("got an amplitude > 1 requested\n");
    }

    aval = 4095*dval;
    write_device(ADD_A,0x21 ,ev_no+syntha_event_count);
    write_device(DAT_A,((aval>>8)&15) ,ev_no+syntha_event_count);
    write_device(WR_A,1,ev_no+syntha_event_count);

    write_device(ADD_A,0x22 ,ev_no+syntha_event_count+1);
    write_device(DAT_A,(aval & 255) ,ev_no+syntha_event_count+1);
    write_device(WR_A,0,ev_no+syntha_event_count+1);

    write_device(UPD_A,1,ev_no+num_split);
    syntha_event_count += 2;
  }
#ifdef WR_B
  else if ((device_id == AMP1 && data_shm->ch1 == 'B')||(device_id == AMP2 && data_shm->ch2 == 'B')){
    if (dval < 0.0){
      dval = -dval;
      printf("got a negative amplitude.  Making positive\n");
    }
    if (dval > 1.0){
      dval = 1.0;
      printf("got a amplitude > 1 requested\n");
    }
    aval = 4095*dval;

    //    printf("writing ampb into events: %i, hitting update for: %i\n",ev_no+synthb_event_count,ev_no+num_split);
    write_device(ADD_B,0x21 ,ev_no+synthb_event_count);
    write_device(DAT_B, ((aval>>8) & 15),ev_no+synthb_event_count);
    write_device(WR_B,1,ev_no+synthb_event_count);

    write_device(ADD_B,0x22 ,ev_no+synthb_event_count+1);
    write_device(DAT_B, (aval & 255),ev_no+synthb_event_count+1);
    write_device(WR_B,0,ev_no+synthb_event_count+1);

    write_device(UPD_B,1,ev_no+num_split);
    synthb_event_count += 2;
  }
#endif
  else if ((device_id == PHASE1 && data_shm->ch1 == 'A') || (device_id == PHASE2 && data_shm->ch2 == 'A' )){
    aaval = floor(dval/360.);
    dval2 = dval - aaval*360;
    aval = rint(16383*dval2/360.); // 14 bits of phase!
    if (aval > 16383){ 
      printf("got phase out of range? This is a bug, dval is: %f,dval2 is: %f,aval: %i\n",dval,dval2,aval);
      aval = 0;
    }
    write_device(ADD_A,0x00 ,ev_no+syntha_event_count);
    write_device(DAT_A,((aval>>8)&63) ,ev_no+syntha_event_count);
    write_device(WR_A,1,ev_no+syntha_event_count);

    write_device(ADD_A,0x01 ,ev_no+syntha_event_count+1);
    write_device(DAT_A, (aval&255),ev_no+syntha_event_count+1);
    write_device(WR_A,0,ev_no+syntha_event_count+1);

    write_device(UPD_A,1,ev_no+num_split);
    syntha_event_count += 2;
  }
#ifdef WR_B
  else if ((device_id == PHASE1 && data_shm->ch1 == 'B')||(device_id == PHASE2 && data_shm->ch2 == 'B')){
    aaval = floor(dval/360.);
    dval2 = dval - aaval*360;
    aval = rint(16383*dval2/360.); // 14 bits of phase!
    if (aval > 16383){ 
      printf("got phase out of range? This is a bug, dval is: %f,dval2 is: %f,aval: %i\n",dval,dval2,aval);
      aval = 0;
    }
    //    printf("writing phaseb into events: %i, hitting update for: %i\n",ev_no+synthb_event_count,ev_no+num_split);
    write_device(ADD_B,0x00 ,ev_no+synthb_event_count);
    write_device(DAT_B,((aval>>8)&63) ,ev_no+synthb_event_count);
    write_device(WR_B,1,ev_no+synthb_event_count);

    write_device(ADD_B,0x01 ,ev_no+synthb_event_count+1);
    write_device(DAT_B,(aval&255),ev_no+synthb_event_count+1);
    write_device(WR_B,0,ev_no+synthb_event_count+1);

    write_device(UPD_B,1,ev_no+num_split);
    synthb_event_count += 2;
  }
#endif
  else {
    fprintf(stderr,"in insert synth event, but couldn't match device\n");
    prog_shm->event_error=1;
    return;
  }



  return;

}


int begin() {

  int i;
  

  // copy old due program to save place for later comparison.
  printf("arriving in begin\n");
  for(i=0;i<NUM_BOARDS;i++){
    memcpy(old_progs[i].data,prog_shm->due_prog[i].data,MAXDATA*4);
    old_progs[i].dpos = prog_shm->due_prog[i].dpos;
  }
  
  num_soft_loops = 0;
  prog_shm->no_events = 0; 
  prog_shm->event_error = 0;
  prog_shm->got_ppo = 0;
  prog_shm->is_noisy = 0;
  prog_shm->begun = 1;
#ifdef DUEBGU
  prog_shm->use_bgu = 0;
  prog_shm->gradprog[prog_shm->txprogno].gradevents = 0;
  bgu_inited = 0;
  bgu_last_vals_zeros = 1;
#endif
  in_sub = 0;
  found_exit = 0;
  /* set up to catch infinite loops */
  mytime.it_interval.tv_sec = 0; 
  mytime.it_interval.tv_usec = 0; 
  mytime.it_value.tv_sec = 3; 
  mytime.it_value.tv_usec = 0; 
  signal( SIGALRM, pprog_internal_timeout);  
  setitimer( ITIMER_REAL, &mytime, &old ); 
  
  num_events_to_resolve=0;
  num_event_labels=0;

  if (prog_shm->is_first_real_prog){
    prog_shm->last_dacA = 0;
    prog_shm->last_dacB = 0;
    prog_shm->last_altA = 0;
    prog_shm->last_altB = 0;
  }    

  return 0; 
  
}

void done(){ 
  
  struct msgbuf message; 
  
  //There might be an extra P_PROGRAM_CALC in the message queue from stop() - Remove it 
  
  msgrcv ( msgq_id, &message, 1, P_PROGRAM_CALC, IPC_NOWAIT ); 
  
  data_shm->pprog_pid = -1; 
  shmdt( (char*) data_shm ); 
  shmdt( (char*) prog_shm ); 
  // fprintf(stderr, "pulse program terminated\n" ); 
  exit(1); 
} 

int pulse_init_shm() 
     
{ 
  int data_size; 
  int prog_size; 
  
  data_shm = 0; 
  prog_shm = 0; 
  
  data_size = sizeof( struct data_shm_t ); 
  prog_size =  sizeof( struct prog_shm_t ); 
  
  data_shm_id = shmget( DATA_SHM_KEY, data_size,0); 
  prog_shm_id = shmget( PROG_SHM_KEY, prog_size,0); 
  
  if( (long)data_shm == -1 || (long)prog_shm == -1 ) { 
    perror( "pulse: Error getting shared memory segments" ); 
    exit(1); 
  } 
  //   fprintf(stderr,"data_shm_id: %i, prog_shm_id: %i\n",data_shm_id,prog_shm_id);
  
  data_shm = (struct data_shm_t*) shmat( data_shm_id, NULL  ,0 ); 
  prog_shm = (struct prog_shm_t*) shmat( prog_shm_id, NULL ,0 ); 
  
  
  if( (long)data_shm == -1 || (long)prog_shm == -1 ) { 
    perror( "pulse: Error attaching shared memory segments" ); 
    exit(1); 
  } 
  
  if( data_shm->pprog_pid != getpid() ) { 
    fprintf(stderr, "pprog: already running\n" ); 
    exit(1); 
  } 
  
  
  if (strcmp(data_shm->version,XNMR_ACQ_VERSION) != 0){
    fprintf(stderr,"pprog: XNMR_ACQ_VERSION number mismatch\n");
    shmdt( (char*) data_shm ); 
    shmdt( (char*) prog_shm ); 
    return -1;
    
  }
  
  if (strcmp(prog_shm->version,PPROG_VERSION) != 0){
    fprintf(stderr,"pprog: PPROG_VERSION number mismatch\n");
    fprintf(stderr,"pprog: got %s and %s\n",prog_shm->version,PPROG_VERSION);
    shmdt( (char*) data_shm ); 
    shmdt( (char*) prog_shm ); 
    return -1;
  }
  //   fprintf(stderr,"pprog: versions ok\n");
  
  

  
  
  return 0; 
} 




 int init_data() 

 { 
   //   int event,chip; 

   //just set the pulse program data set to all 0s 

   memset( prog_shm->outputs, 0, MAX_EVENTS*NUM_BOARDS*sizeof(int) );

   prog_shm->no_events = 0;
   prog_shm->downloaded = 0; // haven't downloaded a pulse prog yet.

   /* // replaced with memset
   for( event = 0; event<MAX_EVENTS; event++ ) 
     for( chip = 0; chip < NUM_CHIPS; chip++ ) 
       prog_shm->prog_image[ chip ][ event ] = 0; 
   */
   return 0; 
 } 

 int init_signals() 
 { 
   /* 
    * Catching these signals allows the user program to shut down normally, 
    * excuting any commands that the user program has in place after the main loop 
    * 
    * the SIGINT and SIGTERM signals will effectively only break the main loop 
    * and allow the pulse program to exit normally  
     */

   signal( SIGINT,  stop ); 
   signal( SIGTERM, stop ); 
   //   signal( SIGINT, (__sighandler_t )stop ); 
   //   signal( SIGTERM,(__sighandler_t ) stop ); 

   return 0; 
 } 

 int init_hardware() 

 { 
   FILE* fid; 
   char s[PATH_LENGTH]; 
   char name[PARAM_NAME_LEN];    
   int i = -1; 
   int d;          //dummy variables 
   char * eo;

   double f; 

   // fprintf(stderr, "initializing hardware configuration\n" ); 

   fid = fopen( "/usr/share/Xnmr/config/h_config-duesdr.h", "r" ); 

   if (fid == NULL) {
     fprintf(stderr,"pulse.c: couldn't open h_config-duesdr.h\n");
     exit(0);
   }

   // look for how many devices
   do { 
     eo = fgets( s, PATH_LENGTH, fid );  
   } while( strstr( s, "NUM_DEVICES" ) == NULL || eo == NULL ); 

   if (eo == NULL){
     fprintf(stderr,"pulse.c: didn't find the number of device in h_config-duesdr.h\n");
     exit(0);
   }

   sscanf(s,"#define NUM_DEVICES %i",&num_dev);
   //   fprintf(stderr,"found num devices = %i\n",num_dev);

   //   fprintf(stderr,"sizeof hardware_config: %i\n",sizeof(*hardware_config));
   if (hardware_config == NULL)
     hardware_config = malloc(num_dev * sizeof(*hardware_config));

   do { 
     fgets( s, PATH_LENGTH, fid );  
   } while( strcmp( s, PARSE_START ) ); 



   do { 
     fgets( s, PATH_LENGTH, fid ); 

     if( !strncmp( s, "#define",7 ) ) { 
       sscanf( s, H_CONFIG_FORMAT, name, &i, &d, &d, &d, &d, &f ); 

       if( i < num_dev && i>= 0 ) { 
 	sscanf( s, H_CONFIG_FORMAT, hardware_config[i].name, &i,  
 		&hardware_config[i].start_bit, 
 		&hardware_config[i].num_bits, 
 		&hardware_config[i].latch, 
 		&hardware_config[i].def_val, 
 		&hardware_config[i].max_time ); 
	

	/*	fprintf(stderr, "Device %d loaded: %s, start: %d, bits: %d, latch: %d, default: %d, timeout: %g\n", i,   	 hardware_config[i].name,  
 	 hardware_config[i].start_bit, 
 	 hardware_config[i].num_bits, 
 	 hardware_config[i].latch, 
	 hardware_config[i].def_val, 
 	 hardware_config[i].max_time ); 
	*/
       } 
       else fprintf(stderr, "Invalid device number %i\n",num_dev ); 
     } 

   } while( strcmp( s, PARSE_END ) ); 


   // build masks for rapid loading of pulse program..

   for (i=0; i<num_dev;i++){
     if (hardware_config[i].start_bit >= 0){ // don't bother for the phoney devices
       hardware_config[i].start_board_num =  hardware_config[i].start_bit/32;

       hardware_config[i].end_board_num = (hardware_config[i].start_bit+hardware_config[i].num_bits-1)/32;
       hardware_config[i].board_bit = hardware_config[i].start_bit % 32;



       //       fprintf(stderr,"device: %i, start bit: %i, end_bit: %i, start_chip_num: %i, end_chip_num: %i\n",i,hardware_config[i].start_bit,
       //	      hardware_config[i].start_bit+hardware_config[i].num_bits-1,hardware_config[i].start_chip_num,hardware_config[i].end_chip_num);

       // here's some bit shifting magic...
       hardware_config[i].load_mask = 
	 (((unsigned int) 0xFFFFFFFF) >> 
	  ( sizeof(unsigned int)*8 - hardware_config[i].num_bits)) 
				      << hardware_config[i].board_bit;
       
       //       fprintf(stderr,"init: dev: %i, chip: %i bit: %i ,mask: %i ",i,(int) hardware_config[i].start_board_num,(int)hardware_config[i].board_bit, (int)hardware_config[i].load_mask);
       //     fprintf(stderr,"num_bits: %i\n",hardware_config[i].num_bits);
       
       if (hardware_config[i].board_bit + hardware_config[i].num_bits-1 > sizeof(unsigned int)*8){
	 fprintf(stderr,"init_hardware:  device %i crosses a word boundary.  Not supported\n",i);
	 exit(0);
       }
     }
   }

   // build latch mask

   for( i=0 ; i<num_dev ; i++){
     if (hardware_config[i].latch == 1 ){
       //       fprintf(stderr,"device %i writing 1's to latch mask\n",i);
       write_device(i,(0xFFFFFFFF >> (sizeof(unsigned int)*8 - hardware_config[i].num_bits)),0);
     }
     else
       write_device(i,0,0);
   }
   for( i=0 ; i<NUM_BOARDS ; i++ ){
     latch_mask[i] = prog_shm->outputs[i][0];
     //          fprintf(stderr,"%3i ",(int) latch_mask[i]);
   }
   //      fprintf(stderr,"\n");



   // build default mask
   // by writing default values into event 0.
   for ( i=0 ; i<num_dev ; i++ )
     write_device(i,hardware_config[i].def_val,0);

   for( i=0 ; i<NUM_BOARDS ; i++ ){
     default_mask[i] = prog_shm->outputs[i][0];
     //     fprintf(stderr,"%3i ",(int) default_mask[i]);
   }
   //      fprintf(stderr,"\n");


   // find BGU scale factors
#ifdef DUEBGU
   rewind(fid);
   do { 
     eo = fgets( s, PATH_LENGTH, fid );  
   } while( strstr( s, "BGU_SCALE_FACTORS" ) == NULL && eo != NULL ); 
   if (eo == NULL){
     fprintf(stderr,"pulse.c: didn't find BGU_SCALE_FACTORS in h_config.h\n");
     bgu_xscale = 1.0;
     bgu_yscale = 1.0;
     bgu_zscale = 1.0;
   }
   else{
     sscanf(s,"#define BGU_SCALE_FACTORS // %f %f %f",&bgu_xscale,&bgu_yscale,&bgu_zscale);
     printf("got BGU_SCALE_FACTORS: %f %f %f\n",bgu_xscale,bgu_yscale,bgu_zscale);
   }
   if (bgu_xscale > 1.0 || bgu_xscale < -1.0){
     bgu_xscale = 1.;
     fprintf(stderr,"bgu_xscale out of range, resetting to 1\n");
   }
   if (bgu_yscale > 1.0 || bgu_yscale < -1.0){
     bgu_yscale = 1.;
     fprintf(stderr,"bgu_yscale out of range, resetting to 1\n");
   }
   if (bgu_zscale > 1.0 || bgu_zscale < -1.0){
     bgu_xscale = 1.;
     fprintf(stderr,"bgu_zscale out of range, resetting to 1\n");
   }
#endif
   fclose( fid );   
   return 0; 
 } 



 int init_msgs() 

 { 

   msgq_id = msgget( MSG_KEY, IPC_CREAT|0660 );      

   if( msgq_id < 0 )  
     done(); 

   return msgq_id; 
 } 

int pulse_program_init() 
  
{ 

  int err;
  struct stat my_buff,other_buff;
  char s[PATH_LENGTH];
  FILE *fs;
  struct msgbuf message; 
#ifndef CYGWIN
  struct rlimit my_lim;
  
  if (getrlimit(RLIMIT_MEMLOCK,&my_lim) == 0){
    my_lim.rlim_cur = RLIM_INFINITY;
    my_lim.rlim_max = RLIM_INFINITY;
    if ( setrlimit(RLIMIT_MEMLOCK,&my_lim) != 0){
      perror("pulse: setrlimit");
    }
    else{ // only do the memlock if we were able to set our limit.
      //      fprintf(stderr,"doing the mlockall\n");
      if (mlockall( MCL_CURRENT | MCL_FUTURE ) !=0 )
	perror("mlockall");
    }
  }
  
#endif
  
  //  exec's into the pulse program, but after the fork.  Can't do it
  //  here because we aren't necessarily root. - This is a problem - the mem lock doesn't work across the fork for acq...
  
  init_msgs();
  if (pulse_init_shm() == -1){
    message.mtype = P_PROGRAM_READY;
    message.mtext[0] = P_PROGRAM_RECOMPILE;
    err=msgsnd ( msgq_id, &message, 1, 0 );
    if (err == -1) perror("pulse.c:msgsnd");
    exit(1);
  }
  init_hardware(); 
  init_signals();
  init_data();
  
  
  // check to make sure that the pulse program has been compiled more recently than libxnmr.so and also more recently that h_config.
  
  err = stat("/usr/share/Xnmr/config/h_config-duesdr.h",&other_buff);
  // now find out who I am.  There must be a better way!!!
  
  path_strcpy(s,getenv(HOMEP));
  path_strcat(s,"/Xnmr/prog/");
  path_strcat(s,data_shm->pulse_exec_path);
  
  
  fs = fopen(s,"rb");
  if (fs == NULL){
    path_strcpy(s,SYS_PROG_PATH);
    path_strcat(s,data_shm->pulse_exec_path);
    fs = fopen(s,"rb");
  }
  if (fs == NULL){
    fprintf(stderr,"couldn't find my own executable??\n");
    message.mtype = P_PROGRAM_READY;
    message.mtext[0] = P_PROGRAM_ERROR;
    err=msgsnd ( msgq_id, &message, 1, 0 );
    if (err == -1) perror("pulse.c:msgsnd");
    shmdt( (char*) data_shm ); 
    shmdt( (char*) prog_shm ); 
    exit(1); 
  }
  fclose(fs);
  //    fprintf(stderr,"in pulse_prog_init, about to stat %s\n",s);
  
  err = stat(s,&my_buff);
  
  if (difftime(my_buff.st_mtime,other_buff.st_mtime) < 0){
    fprintf(stderr,"looks like h_config has changed since pprog was compiled\n");
    message.mtype = P_PROGRAM_READY;
    message.mtext[0] = P_PROGRAM_RECOMPILE;
    err=msgsnd ( msgq_id, &message, 1, 0 );
    if (err == -1) perror("pulse.c:msgsnd");
    shmdt( (char*) data_shm ); 
    shmdt( (char*) prog_shm ); 
    exit(1); 
    
  }



  
  err= stat("/usr/local/lib/libxnmr.so",&other_buff);
  if (difftime(my_buff.st_mtime,other_buff.st_mtime) < 0){
    fprintf(stderr,"looks like libxnmr.so has changed since pprog was compiled\n");
    message.mtype = P_PROGRAM_READY;
    message.mtext[0] = P_PROGRAM_RECOMPILE;
    err=msgsnd ( msgq_id, &message, 1, 0 );
    if (err == -1) perror("pulse.c:msgsnd");
    shmdt( (char*) data_shm ); 
    shmdt( (char*) prog_shm ); 
    exit(1); 
  }
  
  
  prog_shm->prog_ready = NOT_READY;
  // fprintf(stderr, "pulse program initialized on pid %d\n", getpid() );
  
  
  // now build a table to translate the rf channel devices from logical channels to real devices
  //   fprintf(stderr,"building a tran_table\n");
  if (data_shm->ch1 == 'A'){
    //     fprintf(stderr,"pulse.c found ch1 = A\n");
    tran_table[BLNK1-RF_OFFSET]=BLNK_A;
    tran_table[RCVR_GATE1-RF_OFFSET]=RCVR_GATE_A;
    tran_table[GATE1-RF_OFFSET] = GATE_A;
  }
  else if (data_shm->ch1 == 'B'){
    //     fprintf(stderr,"pulse.c found ch1 = B\n");
    tran_table[BLNK1-RF_OFFSET]=BLNK_B;
    tran_table[RCVR_GATE1-RF_OFFSET]=RCVR_GATE_B;
    tran_table[GATE1-RF_OFFSET] = GATE_B;
  }
  else {
    fprintf(stderr,"no valid channel 1 channel found!\n");
    exit(0);
  }
  if (data_shm->ch2 == 'A'){
    //     fprintf(stderr,"pulse.c found ch2 = A\n");
    tran_table[BLNK2-RF_OFFSET]=BLNK_A;
    tran_table[RCVR_GATE2-RF_OFFSET]=RCVR_GATE_A;
    tran_table[GATE2-RF_OFFSET] = GATE_A;

  }
  else if (data_shm->ch2 == 'B'){
    //     fprintf(stderr,"pulse.c found ch2 = B\n");
    tran_table[BLNK2-RF_OFFSET]=BLNK_B;
    tran_table[RCVR_GATE2-RF_OFFSET]=RCVR_GATE_B;
    tran_table[GATE2-RF_OFFSET] = GATE_B;
  }
  else {
    fprintf(stderr,"no valid channel 2 channel found!\n");
    //    exit(0); // XXX TODO this is ok for now maybe?
  }
  //   fprintf(stderr,"event, got device numbers: %i %i %i\n",clk3,phase3,i3);
    
  finished = 0;
  return 0;
}


double get_dwell()
{
  return data_shm->dwell/1000000.;
}

unsigned long get_acqn()
{
  return data_shm->acqn;
}

unsigned int get_acqn_2d()
{
  return data_shm->acqn_2d;
}

int get_npts()
{
  return data_shm->npts;
}

unsigned long get_num_acqs()
{
  return data_shm->num_acqs;
}

unsigned int get_num_acqs_2d()
{
  return data_shm->num_acqs_2d;
}

int fetch_float( char* name, float* var )
{
  int result;
  result = sfetch_float( data_shm->parameters, name, var, data_shm->acqn_2d );
  if (result == -1) parameter_not_found( name );
  return result;
}

int fetch_int( char* name, int* var )
{
  int result;
  result = sfetch_int( data_shm->parameters, name, var, data_shm->acqn_2d );
  if (result == -1) parameter_not_found( name );
  return result;
}

int fetch_text( char* name, char* var )
{
  int result;
  result = sfetch_text( data_shm->parameters, name, var, data_shm->acqn_2d );
  if (result == -1) parameter_not_found( name );
  return result;
}

int fetch_double( char* name, double* var )
{
  int result;
  result = sfetch_double( data_shm->parameters, name, var, data_shm->acqn_2d );
  if (result == -1) parameter_not_found( name );
  return result;
}


void pprog_internal_timeout(){
  struct msgbuf message;
  int result;

  fprintf(stderr,"Pulse Program timed out internally\n");
  
  // let acq know we have a problem
  message.mtype = P_PROGRAM_READY;
  message.mtext[0] = P_PROGRAM_INTERNAL_TIMEOUT;
  result=msgsnd ( msgq_id, &message, 1, 0 );

  if (result == -1) perror("pulse.c:msgsnd");


  // and get out

  stop();
  //  done(); 

   //There might be an extra P_PROGRAM_CALC in the message queue from stop() - Remove it 

   msgrcv ( msgq_id, &message, 1, P_PROGRAM_CALC, IPC_NOWAIT ); 

   //   data_shm->pprog_pid = -1;  // don't set this to -1 so that acq knows what to wait() for
   shmdt( (char*) data_shm ); 
   shmdt( (char*) prog_shm ); 
   // fprintf(stderr, "pulse program terminated\n" ); 
   exit(1);  
   return; 

}

void pprog_is_noisy(){
  
  prog_shm->is_noisy = 1;
  prog_shm->noisy_start_pos=0;
  //  fprintf(stderr,"just set is_noisy to true\n");
}


// obsolete?
void start_noisy_loop(){
  prog_shm->noisy_start_pos = prog_shm->no_events;
  //  fprintf(stderr,"just set noisy_start_pos to %i\n",prog_shm->no_events);

}



void label_to_resolve(char * label){

  // int i;
    //    printf("Got label %s to resolve at event %i\n",label,prog_shm->no_events);
  
  strncpy(labels_to_resolve[num_events_to_resolve],label,MAX_LABEL_LEN);
  events_to_resolve[num_events_to_resolve]=prog_shm->no_events;

#ifdef DUEBGU
  gradevents_to_resolve[num_events_to_resolve]=prog_shm->gradprog[prog_shm->txprogno].gradevents;
#endif
  
  num_events_to_resolve += 1;
  if (num_events_to_resolve >= MAX_LABELS){
    printf("ERROR, overrun labels - need to increase MAX_LABELS\n");
    prog_shm->event_error = 1;
  }  
}

int resolve_labels(){
  int j,k;
  int rval = TRUE;
  // main program (Due):
  for (j=0;j<num_events_to_resolve;j++){
    for (k=0;k<num_event_labels;k++){
      //	printf("Comparing %s with %s (label %i\n",labels_to_resolve[i][j],event_labels[i][k],k);
      if (strncmp(labels_to_resolve[j],event_labels[k],MAX_LABEL_LEN) == 0){
	//	  printf("found label: %s at instruction: %i\n",labels_to_resolve[i][j],event_numbers[i][k]);
	// ok so we found the label, now do the right thing:
	prog_shm->opinst[events_to_resolve[j]] = event_numbers[k];
	// and stick the subroutine number into subid for the subroutine itself
	prog_shm->subids[event_numbers[k]] = k; // ok, so now the resolved event, has an index as its subid

#ifdef DUEBGU
	prog_shm->gradprog[prog_shm->txprogno].gradopinst[gradevents_to_resolve[j]] = gradevent_numbers[k];
#endif
	
	k = num_event_labels + 10;			      
      }
    }
    if (k != num_event_labels + 11){
      // screwed up, didn't resolve
      printf("failed to resolve label: %s\n",labels_to_resolve[j]);
      rval = FALSE;
    }
  }
 
  return rval;
}


void setup_synths(double freqa,double freqb){
  // will set up the synths at start - need to do full writes of all synth registers...
  // set rcvr_chan to RF1 or RF2 for obs channel.
  printf("in setup_synths with freqs: %f, %f\n",freqa, freqb);

  if (prog_shm->do_synth_setup == 1){ //XXX TODO, maybe don't need this anymore, if startup-program is the one (and only) that calls this function.
    // do full setup
    printf("doing full synth setup\n");
#ifdef WR_B // there are two synthesizers
    //    prog_shm->do_synth_setup = 0;
    // the most interesting of all, the control registers
    // single tone mode:
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x1F,DAT_A,0,ADD_B,0x1F,DAT_B,0,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x1F,DAT_A,0,ADD_B,0x1F,DAT_B,0,WR_A,0,WR_B,0);
    event_duesdr( 100e-6,0,0,0);
    event_duesdr( PROG_TIME,0,0,2,UPD_A,1,UPD_B,1);

    // address 1D is power down some things:

    // top 8 bits: top 3 unused, next: 1 means power down comparator
    // next must always be 0
    // next 1 means power down q dac
    // full dac power-down
    // then digital power down.
    // we will power down the q dac's and comparators for now.
    //    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x1D,DAT_A,0x14,ADD_B,0x1D,DAT_B,0x14,WR_A,1,WR_B,1);
    // nope, leave qdacs on.
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x1D,DAT_A,0x10,ADD_B,0x1D,DAT_B,0x10,WR_A,1,WR_B,1);
    // next 8:
    // first is always 0, second sets range of PLL want 1 for > 200MHz
    // next is PLL bypass (if high), then 5 bits for PLL multiplier value.
    // we want a multiplier of 5 and PLL high range.
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x1E,DAT_A,5+64,ADD_B,0x1E,DAT_B,5+64,WR_A,0,WR_B,0);

    // next, top bit is clear accumulator bit (one-shot), then  clear both accumulators
    // the triangle mode bit for up and down freq sweeps, then Q dac source.  when high,
    // get Q DAC output from the Q DAC register
    // then three bits that set the overall mode: =0 for single-tone,
    // = 1 for FSK, 2 for ramped FSK, 3 for chirp 4 for BPSK
    // last is the internal update active. 0 makes I/O UPD an output.
    // turn on SRC bit.
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x1F,DAT_A,0x10,ADD_B,0x1F,DAT_B,0x10,WR_A,1,WR_B,1);

    // last byte, top bit is always 0, next when high, bypass the inverse sinc filter - big power savings
    // next is shaped keying enable (when high)
    // then internal/external shaped keying control.
    //then next two are always 0, then serial port endian
    // and finally SDO active for serial port.
    // we set 0x40 to disable the inverse sinc filter
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x20,DAT_A,0x60,ADD_B,0x20,DAT_B,0x60,WR_A,0,WR_B,0);


    //    /*
    //phase registers
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x00,DAT_A,0,ADD_B,0x00,DAT_B,0,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x01,DAT_A,0,ADD_B,0x01,DAT_B,0,WR_A,0,WR_B,0);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x02,DAT_A,0,ADD_B,0x02,DAT_B,0,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x03,DAT_A,0,ADD_B,0x03,DAT_B,0,WR_A,0,WR_B,0);

    // frequency tuning word 1 gets done later
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x0A,DAT_A,0,ADD_B,0x0A,DAT_B,0,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x0B,DAT_A,0,ADD_B,0x0B,DAT_B,0,WR_A,0,WR_B,0);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x0C,DAT_A,0,ADD_B,0x0C,DAT_B,0,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x0D,DAT_A,0,ADD_B,0x0D,DAT_B,0,WR_A,0,WR_B,0);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x0E,DAT_A,0,ADD_B,0x0E,DAT_B,0,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x0F,DAT_A,0,ADD_B,0x0F,DAT_B,0,WR_A,0,WR_B,0);
    // delta freq:
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x10,DAT_A,0,ADD_B,0x10,DAT_B,0,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x11,DAT_A,0,ADD_B,0x11,DAT_B,0,WR_A,0,WR_B,0);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x12,DAT_A,0,ADD_B,0x12,DAT_B,0,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x13,DAT_A,0,ADD_B,0x13,DAT_B,0,WR_A,0,WR_B,0);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x14,DAT_A,0,ADD_B,0x14,DAT_B,0,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x15,DAT_A,0,ADD_B,0x15,DAT_B,0,WR_A,0,WR_B,0);
    // update clock - the default it 0x40 - keep it.
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x16,DAT_A,0x40,ADD_B,0x16,DAT_B,0x40,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x17,DAT_A,0,ADD_B,0x17,DAT_B,0,WR_A,0,WR_B,0);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x18,DAT_A,0,ADD_B,0x18,DAT_B,0,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x19,DAT_A,0x40,ADD_B,0x19,DAT_B,0x40,WR_A,0,WR_B,0);

    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x1A,DAT_A,0,ADD_B,0x1A,DAT_B,0,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x1B,DAT_A,0,ADD_B,0x1B,DAT_B,0,WR_A,0,WR_B,0);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x1C,DAT_A,0,ADD_B,0x1C,DAT_B,0,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x25,DAT_A,0,ADD_B,0x25,DAT_B,0,WR_A,0,WR_B,0);


    // amplitudes:
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x21,DAT_A,0,ADD_B,0x21,DAT_B,0,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x22,DAT_A,0,ADD_B,0x22,DAT_B,0,WR_A,0,WR_B,0);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x23,DAT_A,0,ADD_B,0x23,DAT_B,0,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x24,DAT_A,0,ADD_B,0x24,DAT_B,0,WR_A,0,WR_B,0);

    // qdac values:
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x26,DAT_A,8,ADD_B,0x26,DAT_B,8,WR_A,1,WR_B,1);
    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x27,DAT_A,0,ADD_B,0x27,DAT_B,0,WR_A,0,WR_B,0);
//    */
#else
    //    prog_shm->do_synth_setup = 0;
    // the most interesting of all, the control registers
    // single tone mode:
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x1F,DAT_A,0,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x1F,DAT_A,0,WR_A,0);
    event_duesdr( 100e-6,0,0,0);
    event_duesdr( PROG_TIME,0,0,1,UPD_A,1);

    // address 1D is power down some things:

    // top 8 bits: top 3 unused, next: 1 means power down comparator
    // next must always be 0
    // next 1 means power down q dac
    // full dac power-down
    // then digital power down.
    // we will power down the q dac's and comparators for now.
    //    event_duesdr( PROG_TIME, 0,0,6,ADD_A,0x1D,DAT_A,0x14,ADD_B,0x1D,DAT_B,0x14,WR_A,1,WR_B,1);
    // nope, leave qdacs on.
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x1D,DAT_A,0x10,WR_A,1);
    // next 8:
    // first is always 0, second sets range of PLL want 1 for > 200MHz
    // next is PLL bypass (if high), then 5 bits for PLL multiplier value.
    // we want a multiplier of 5 and PLL high range.
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x1E,DAT_A,5+64,WR_A,0);

    // next, top bit is clear accumulator bit (one-shot), then  clear both accumulators
    // the triangle mode bit for up and down freq sweeps, then Q dac source.  when high,
    // get Q DAC output from the Q DAC register
    // then three bits that set the overall mode: =0 for single-tone,
    // = 1 for FSK, 2 for ramped FSK, 3 for chirp 4 for BPSK
    // last is the internal update active. 0 makes I/O UPD an output.
    // turn on SRC bit.
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x1F,DAT_A,0x10,WR_A,1);

    // last byte, top bit is always 0, next when high, bypass the inverse sinc filter - big power savings
    // next is shaped keying enable (when high)
    // then internal/external shaped keying control.
    //then next two are always 0, then serial port endian
    // and finally SDO active for serial port.
    // we set 0x40 to disable the inverse sinc filter
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x20,DAT_A,0x60,WR_A,0);


    //    /*
    //phase registers
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x00,DAT_A,0,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x01,DAT_A,0,WR_A,0);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x02,DAT_A,0,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x03,DAT_A,0,WR_A,0);

    // frequency tuning word 1 gets done later
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x0A,DAT_A,0,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x0B,DAT_A,0,WR_A,0);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x0C,DAT_A,0,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x0D,DAT_A,0,WR_A,0);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x0E,DAT_A,0,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x0F,DAT_A,0,WR_A,0);
    // delta freq:
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x10,DAT_A,0,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x11,DAT_A,0,WR_A,0);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x12,DAT_A,0,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x13,DAT_A,0,WR_A,0);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x14,DAT_A,0,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x15,DAT_A,0,WR_A,0);
    // update clock - the default it 0x40 - keep it.
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x16,DAT_A,0x40,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x17,DAT_A,0,WR_A,0);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x18,DAT_A,0,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x19,DAT_A,0x40,WR_A,0);

    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x1A,DAT_A,0,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x1B,DAT_A,0,WR_A,0);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x1C,DAT_A,0,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x25,DAT_A,0,WR_A,0);


    // amplitudes:
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x21,DAT_A,0,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x22,DAT_A,0,WR_A,0);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x23,DAT_A,0,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x24,DAT_A,0,WR_A,0);

    // qdac values:
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x26,DAT_A,8,WR_A,1);
    event_duesdr( PROG_TIME, 0,0,3,ADD_A,0x27,DAT_A,0,WR_A,0);
#endif

  }
  else
    printf("only setting frequencies in synth setup\n");

  // otherwise just set the frequencies.


  //    printf("about to init freqs\n");
#ifdef WR_B
  event_duesdr(7*PROG_TIME,0,0,0); // the frequency setting actually goes in here
  event_duesdr( 9*PROG_TIME, 0,0,2, FREQA,freqa,FREQB,freqb ); // this provides a spot for some first synth events set by program...
#else // only one synth.
  event_duesdr(7*PROG_TIME,0,0,0); // the frequency setting actually goes in here
  printf("in setup_synths, setting FREQA to %f\n",freqa);
  event_duesdr( 9*PROG_TIME, 0,0,1, FREQA,freqa); // this provides a spot for some first synth events set by program...
#endif
  
  if (prog_shm->do_synth_setup == 1){
    // do full setup
    printf("doing full synth setup\n");
    // it takes a while for all this to sink in the first time.  Wait for it
    // 200us is not long enough!
    // Perhaps this is time for PLL in the synthesizer to lock?
    event_duesdr(1000e-6,0,0,0);
    prog_shm->do_synth_setup = 0;
  }

  //  printf("SETUP_SYNTHS, set freqs to: %g %g\n",freqa,freqb);


  return;
}

#define MAX_ALL_EVENTS 100000000
#define STACK_SIZE 50
#define MAX_LOOP_LEVELS 50


uint64_t points_to_receive_new(int rxno){
  // parse the pulse program and see how many points will be received.
  unsigned int jsr_stack[STACK_SIZE],counter=0;
  uint64_t loop_rx_time[STACK_SIZE];
  int loops[MAX_LOOP_LEVELS];
  int jsr_num=0,cur_event,next_event=0,loop_level=0;
  uint64_t points = 0;

  loop_rx_time[0] = 0;
  cur_event = 0;
  counter = 0;
  while ((prog_shm->opcodes[cur_event] != EXIT) && counter < MAX_ALL_EVENTS){
    //    printf("receive points doing event: %i\n",cur_event);
    switch (prog_shm->opcodes[cur_event]){
    case CONTINUE:
    case WAIT:
    case WAIT_MAX:
    case SUBSTART:
    case DUE_ALT:
    case DUE_DAC:
    if (prog_shm->rx_on[rxno][cur_event]) loop_rx_time[loop_level] += prog_shm->times[cur_event];
      next_event = cur_event+1;
      break;
    case LOOP:
      loop_level += 1;
      if (loop_level >= MAX_LOOP_LEVELS-1){
	printf("ERROR Got a start loop too many levels deep\n");
	prog_shm->event_error =1;
	return loop_rx_time[0]; // this is so wrong...
      }
      if (prog_shm->rx_on[rxno][cur_event])
	loop_rx_time[loop_level] = prog_shm->times[cur_event];
      else
	loop_rx_time[loop_level] = 0;
      loops[loop_level] = prog_shm->opinst[cur_event];
      if (loops[loop_level] == 0){
	printf("GOT LOOP START WITH 0 LOOPS BADNESS\n");
      }
      next_event = cur_event + 1;
      break;
    case END_LOOP:
      //      printf("got an END_LOOP at event %i in thread %i, with %i loops\n",cur_event,rxno,loops[loop_level]);
      if (prog_shm->rx_on[rxno][cur_event])
	loop_rx_time[loop_level] += prog_shm->times[cur_event];
      printf("points_to_receive, end_loop, loop ticks was: %li, number of loops: %i\n",loop_rx_time[loop_level],loops[loop_level]);
      loop_level -= 1;
      loop_rx_time[loop_level] += loop_rx_time[loop_level+1]*loops[loop_level+1];
      next_event = cur_event + 1;
      break;
    case JSR:
      if (prog_shm->rx_on[rxno][cur_event])
	loop_rx_time[loop_level] += prog_shm->times[cur_event];
      jsr_stack[jsr_num] = cur_event+1;
      jsr_num += 1;
      if (jsr_num > STACK_SIZE -1){
	printf("receive points:, Too many JSR's nested. BAD THINGS WILL HAPPEN\n");
      }
      next_event = prog_shm->opinst[cur_event];
      //	printf("jsr setting next event to: %i\n",next_event);
      if (next_event >= prog_shm->no_events || next_event < 0){
	printf("got a JSR to a nonsense event\n");
      }
      break;
    case RTS:
      if (prog_shm->rx_on[rxno][cur_event])
	loop_rx_time[loop_level] += prog_shm->times[cur_event];
      if (jsr_num < 1){
	printf("got RTS, but nowhere to return to!\n");
	next_event = cur_event +1 ;
      }
      next_event = jsr_stack[jsr_num-1];
      jsr_num -=1;
      if (next_event >= prog_shm->no_events || cur_event < 0){
	printf("got a RTS to a nonsense event\n");
      }
      break;
    case EXIT:
      if (prog_shm->rx_on[rxno][cur_event])
	loop_rx_time[loop_level] += prog_shm->times[cur_event];
      break;
    default:
      printf("calc receive points got an unknown opcode of %i\n",(prog_shm->opcodes[cur_event]));
      next_event = cur_event + 1;
    } // end of switch on opcode
    cur_event = next_event;
    counter += 1;
  }
  if (prog_shm->opcodes[cur_event] != EXIT) {
    printf("didn't find EXIT in calculating points to receive\n");
  }
  // times here were measured in due ticks. There are likely rounding errors...
  points = round(loop_rx_time[0] * prog_shm->rx_sw[rxno]/DUE_PP_CLOCK);
  printf("receive point for channel %i. Found time (in ticks): %li, points: %li\n",rxno,loop_rx_time[0],points);
  
  return points;
  
}




 
// should have a flag to mark channel as inited?

void rx_init(int chan, double sw,int gain, float phase){
  if (chan >= NUM_RX){
    printf("rx_init got channel: %i, but only have %i boards\n",chan,NUM_RX);
    return;
  }
  printf("rx_init with sw: %lf, gain %i, phase %f\n",sw,gain,phase);
  prog_shm->rx_sw[chan] = sw; // sw really shouldn't change. gain should be ok, phase definitely.
  prog_shm->rx_gains[chan] = gain;
  prog_shm->rx_phase[chan] = phase;
}

#ifdef DUEBGU
int init_bgu(float alpha, float beta, float gamma){
  // alpha, beta and gamma will be euler angles to transform all gradients!
  bgu_alpha = alpha*M_PI/180.;
  bgu_beta = beta*M_PI/180.;
  bgu_gamma = gamma*M_PI/180.;

  prog_shm->use_bgu = 1;

  bgu_inited=1;
  bgu_cleared = 0;
  // ok, bgu seems to exist. Let's write the preload registers with all 0's
  return 1;
  // don't do this - the driver will do it on start of first scan.
  //  return set_gradients(0.,0.,0.,0);
}

void clear_bgu(){
  //  set_gradients(0.,0.,0.,0);
  bgu_cleared = 1;
  
}
#endif

void multiply_mat(float mat[3][3],float x,float y,float z,float *nx, float *ny, float *nz){

  *nx = x* mat[0][0]+y*mat[0][1]+z*mat[0][2];
  *ny = x* mat[1][0]+y*mat[1][1]+z*mat[1][2];
  *nz = x* mat[2][0]+y*mat[2][1]+z*mat[2][2];
  //  printf("mat: %f %f %f %f %f %f %f %f %f\n",mat[0][0],mat[0][1],mat[0][2],mat[1][0],mat[1][1],mat[1][2],mat[2][0],mat[2][1],mat[2][2]);
  //  printf("multiply got args: %f %f %f\n",x,y,z);
  //  printf("multiply returning: %f %f %f\n",*nx,*ny,*nz);
}

#ifdef DUEBGU
int set_gradients(float x,float y, float z,unsigned char num, ...){
  float nx,ny,nz;
  va_list args;
  float mat[3][3];
  gradprog_t *gradprog;
  //  transform from x,y,z to nx, ny, nz with angles alpha,beta,gamma
  //  printf("set_gradients: requested gradients: %f %f %f\n",x,y,z);
  //  printf("in set_gradients with values %f %f %f \n",x,y,z);
  gradprog = &prog_shm->gradprog[prog_shm->txprogno];
  
  if (bgu_inited != 1){
    printf("set_gradients: BGU not inited!\n");
    return -1;
  }
  
  if (bgu_cleared == 1 && in_sub == 0){
    prog_shm->event_error = 1;
    fprintf(stderr,"set_gradients: BGU has already been cleared\n");
     return -1;
  }
  if (gradprog->gradevents >= MAX_GRAD_EVENTS-1){
    prog_shm->event_error = 1;
    printf("Too many grad events!\n");
    return -1;
  }
  
  
  mat[0][0] = mat[1][1] = cos(bgu_alpha);
  mat[0][1] = sin(bgu_alpha);
  mat[1][0] = -mat[0][1];
  mat[2][2] = 1.;
  mat[0][2] = mat[1][2] = mat[2][0] = mat[2][1] = 0.;
  multiply_mat(mat,x,y,z,&nx,&ny,&nz);
  
  mat[1][1] = mat[2][2] = cos(bgu_beta);
  mat[1][2] = sin(bgu_beta);
  mat[2][1] = -mat[1][2];
  mat[0][0] = 1.;
  mat[0][1] = mat[0][2] = mat[1][0] = mat[2][0] = 0.;
  multiply_mat(mat,nx,ny,nz,&nx,&ny,&nz);

  mat[0][0] = mat[1][1] = cos(bgu_gamma);
  mat[0][1] = sin(bgu_gamma);
  mat[1][0] = -mat[0][1];
  mat[2][2] = 1.;
  mat[0][2] = mat[1][2] = mat[2][0] = mat[2][1] = 0.;
  multiply_mat(mat,nx,ny,nz,&nx,&ny,&nz);
  //  printf("set_gradients: rotated gradients: %f %f %f\n",nx,ny,nz);

  if (fabs(nx) < 1e-5 && fabs(ny) < 1e-5 && fabs(nz) < 1e-5 )
    bgu_last_vals_zeros = 1;
  else
    bgu_last_vals_zeros = 0;
  
  
  /* actually set the gradient here in Due and in grad program
   */
  //  printf("grad vals are %i %i %i\n",(int)(nx*bgu_xscale*32767),(int)(ny*bgu_yscale*32767),(int)(nz*bgu_zscale*32767));
  //  printf("nx %f ny %f nz %f\n",nx,ny,nz);
  gradprog->is_grad_event[gradprog->gradevents] = 1;
  gradprog->gradopcode[gradprog->gradevents] = 0; // just CONTINUE
  gradprog->gradopinst[gradprog->gradevents] = 0;
  gradprog->x[gradprog->gradevents] = (int16_t) (nx*bgu_xscale*32767); 
  gradprog->y[gradprog->gradevents] = (int16_t) (ny*bgu_yscale*32767); 
  gradprog->z[gradprog->gradevents] = (int16_t) (nz*bgu_zscale*32767); 
  gradprog->gradevents += 1;
  if (num > 0){ // there are other devices requested
    //    printf("in set_gradients, doing va_args and calling do_event_duesdr\n");
    va_start(args,num);
    do_event_duesdr(1.5e-6,0,0,num,args); // opcode 0 opinst 0
    write_device(BGU_TRIG,1,prog_shm->no_events-1); // turn on BGU_TRIG bit in the previous event
    do_event_duesdr(0.5e-6,0,0,num,args); // opcode 0 opinst 0
    write_device(BGU_NGI,0,prog_shm->no_events-1); // turn off NGI bit in the previous event
    va_end(args);
    //    printf("in set_gradients, done calling do_event_duesdr\n");
  }
  else{ // no other devices requested:
    event_duesdr(1.5e-6,0,0,1,BGU_TRIG,1);
    event_duesdr(0.5e-6,0,0,1,BGU_NGI,0); // NGI is negative true - it idles high.
  }
  return 1;
}


#endif
/*


we're going to write pulse.c so synth events get put in the event before the current one.
We shouldn't put synth events in any event with a label, or any LOOP command (which should have a label).

We also don't want the previous event (that we drop into) to be an END LOOP, a LONG EVENT, a BRANCH,a JSR, or an RTS.  LOOP, CONTINUE, and WAIT are ok.

*/



uint64_t pp_time_new(volatile char *done){
  int cur_event=0;
  unsigned int jsr_stack[STACK_SIZE];
  unsigned int loop_level = 0;
  double loop_time[MAX_LOOP_LEVELS];
  unsigned int loops[MAX_LOOP_LEVELS];
  
  int jsr_num=0;
  int counter = 0;
  int i;
  loop_time[0] = 0.;
  // look at time of first timer.
  while(prog_shm->opcodes[cur_event] != EXIT && counter < MAX_ALL_EVENTS){
    //    printf("event: %i, outputs: %i, opcode: %i duration: %f\n",cur_event,prog_shm->outputs[0][cur_event],prog_shm->opcodes[0][cur_event],prog_shm->times[0][cur_event]);
    switch (prog_shm->opcodes[cur_event] ){
    case CONTINUE:
    case SUBSTART:
    case DUE_ALT:
    case DUE_DAC:
      loop_time[loop_level] += prog_shm->times[cur_event];
      cur_event+=1;
      break;
    case WAIT:
    case WAIT_MAX:
      loop_time[loop_level] += prog_shm->times[cur_event];
      //      how_long += prog_shm->times[0][cur_event]*prog_shm->opinst[0][cur_event];
      cur_event+=1;
      fprintf(stderr,"Got a WAIT, times may be off\n");
      break;
    case LOOP:
      loop_level += 1;
      if (loop_level >= MAX_LOOP_LEVELS-1){ // max is one too big, since we use 0 for not looping.
	fprintf(stderr,"Got a start loop too many levels deep\n");
	*done = -1;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      loop_time[loop_level] = prog_shm->times[cur_event];
      loops[loop_level] = prog_shm->opinst[cur_event]; // load the loop counter
      cur_event+=1;
      break;
    case END_LOOP:
      if (loop_level <1){
	fprintf(stderr,"Got a loop end but not enough loop starts?\n");
	*done = -1;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      loop_time[loop_level] += prog_shm->times[cur_event];
      loop_level -= 1;
      //      printf("at end loop. loop time was: %lf, loops: %i\n",loop_time[loop_level+1],loops[loop_level+1]);
      loop_time[loop_level] += loop_time[loop_level+1]*loops[loop_level+1];
      cur_event += 1;
      break;
    case JSR:
      //      printf("got a jsr at event: %i, going to event: %i\n",
      //	     cur_event,prog_shm->opinst[0][cur_event]);
      loop_time[loop_level] += prog_shm->times[cur_event];
      jsr_stack[jsr_num] = cur_event + 1;
      jsr_num += 1;
      if (jsr_num > STACK_SIZE -1){
	fprintf(stderr,"too many JSR's nested\n");
	*done = -1;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      cur_event = prog_shm->opinst[cur_event];
      if (cur_event > MAX_EVENTS || cur_event < 0){
	fprintf(stderr,"got a jsr to an event that doesn't exist.\n");
	*done = -1;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      break;
    case RTS:
      loop_time[loop_level] += prog_shm->times[cur_event];
      if (jsr_num < 1){
	fprintf(stderr,"got a JSR, but nowhere to return to on stack!\n");
	*done = -1;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      cur_event = jsr_stack[jsr_num -1];
      jsr_num -=1;
      if (cur_event <0 || cur_event > MAX_EVENTS){
	fprintf(stderr,"got a RTS to a non-existant event!\n");
	*done = -1;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      break;
    default:
      fprintf(stderr,"got an unknown opcode!\n");
      *done = -1;
      return loop_time[0]/DUE_PP_CLOCK;
    }
    counter += 1;
  }
  // capture EXIT:
  if (prog_shm->opcodes[cur_event]  == EXIT)
    loop_time[0] += prog_shm->times[cur_event];
  // be explicit:
  else{
    printf("pp_time_new. Didn't find an EXIT!\n");
    prog_shm->event_error = 1;
    *done = -1;
  }
  
  if (counter >= MAX_ALL_EVENTS){
    fprintf(stderr,"Got too many events (looping error? or forgotten EXIT?)\n");
    *done = -1;
    prog_shm->event_error = 1;
    return loop_time[0]/DUE_PP_CLOCK;
  }

  // in case the program ends in the middle of a loop:
  for (i=1;i<=loop_level;i++)
    loop_time[0] += loop_time[i];
  printf("pp_time_new: how_long: %f\n",loop_time[0]/DUE_PP_CLOCK);
    
  return loop_time[0];
}


void add_rx_event(char was_on, int rx_num, uint32_t accum_time, uint32_t duration){
  if (was_on)
    prog_shm->rx_staging_duration[rx_num][prog_shm->rx_staging_num_events[rx_num]-1] += duration;
  else{
    prog_shm->rx_staging_duration[rx_num][prog_shm->rx_staging_num_events[rx_num]] = duration;
    prog_shm->rx_staging_start_time[rx_num][prog_shm->rx_staging_num_events[rx_num]] = accum_time;
    prog_shm->rx_staging_num_events[rx_num] += 1;
  }

}


/* this may be very slow in a long program since it traverses every single event - going through all loops.
   It should probably run in a separate thread in real time alongside the execution, so that other stuff can
   be happening at the same time. More similar to the tx threads in the lime version. */
int pre_gen_rx_events(){
  int cur_event=0,next_event = 0;
  unsigned int jsr_stack[STACK_SIZE];
  unsigned int loop_num=0;
  unsigned int loop_stack[MAX_LOOP_LEVELS];
  int loop_counter[MAX_LOOP_LEVELS];

  uint64_t accum_time = 0;
  int jsr_num=0;
  int counter = 0;
  int i;
  char was_on[NUM_RX]; // if receiver was on in previous event

  
  for(i=0;i<NUM_RX;i++){
    prog_shm->rx_staging_num_events[i]=0;
    was_on[i] = 0;
  }    
  cur_event = 0;
  counter=0;

  accum_time = 0;
  // look at time of first timer.
  while(prog_shm->opcodes[cur_event] != EXIT && counter < MAX_ALL_EVENTS){
      //    printf("event: %i, outputs: %i, opcode: %i duration: %f\n",cur_event,prog_shm->outputs[0][cur_event],prog_shm->opcodes[0][cur_event],prog_shm->times[0][cur_event]);

    for (i=0;i<NUM_RX;i++){
      if (prog_shm->rx_on[i][cur_event] == 1){
	add_rx_event(was_on[i], i, accum_time, prog_shm->times[cur_event]);
	was_on[i] = 1;
	if (prog_shm->rx_staging_num_events[i] > MAX_RX_EVENTS/2){
	  printf("Too many RX events!\n");
	  return -1;
	}
      }
      else
	was_on[i] = 0;
    }
    accum_time += prog_shm->times[cur_event];
    switch (prog_shm->opcodes[cur_event] ){
    case CONTINUE:
    case SUBSTART:
    case DUE_ALT:
    case DUE_DAC:
      next_event = cur_event+1;
      break;
    case WAIT:
    case WAIT_MAX:
      fprintf(stderr,"pulse-duesdr. Got a WAIT event. THIS WON'T WORK. RX will get out sync.\n");
      //      how_long += prog_shm->times[0][cur_event]*prog_shm->opinst[0][cur_event];
      next_event = cur_event+1;
      fprintf(stderr,"Got a WAIT, times WILL be off\n");
      return -1;
      break;
    case LOOP:
      
      next_event = cur_event+1;
      if (loop_num > 0){
	if (loop_stack[loop_num-1] == cur_event && loop_counter[loop_num-1] >0){
	  break;
	}
      }
      loop_counter[loop_num] = prog_shm->opinst[cur_event];
      loop_stack[loop_num] = cur_event;
      loop_num += 1;
      if (loop_num > MAX_LOOP_LEVELS-1){ // max is one too big, since we use 0 for not looping.
	fprintf(stderr,"Got a start loop too many levels deep. BAD THINGS WILL HAPPEN\n");
	return -1;
      }
      break;
    case END_LOOP:
      next_event = loop_stack[loop_num-1];
      if (loop_num <1){
	fprintf(stderr,"Got a loop end but not enough loop starts? BAD THINGS\n");
	return -1;
      }
      if (loop_counter[loop_num-1] <0){
	fprintf(stderr,"Got end loop, but uninitialized number of loops. BAD THINGS\n");
	return -1;
      }
      loop_counter[loop_num-1] -= 1;
      if (loop_counter[loop_num -1] <=0){
	next_event = cur_event +1;
	loop_num -= 1;
      }
      break;
    case JSR:
      jsr_stack[jsr_num] = cur_event + 1;
      jsr_num += 1;
      if (jsr_num > STACK_SIZE -1){
	fprintf(stderr,"too many JSR's nested\n");
      }
      next_event = prog_shm->opinst[cur_event];
      if (next_event > MAX_EVENTS || cur_event < 0){
	fprintf(stderr,"got a jsr to an event that doesn't exist.\n");
	return -1;
      }
      break;
    case RTS:
      if (jsr_num < 1){
	fprintf(stderr,"got a JSR, but nowhere to return to on stack!\n");
      }
      next_event = jsr_stack[jsr_num -1];
      jsr_num -=1;
      if (next_event <0 || next_event > MAX_EVENTS){
	fprintf(stderr,"got a RTS to a non-existant event!\n");
	return -1;
      }
      break;
    case EXIT:
      accum_time += prog_shm->times[cur_event];
    default:
      fprintf(stderr,"got an unknown opcode!\n");
      return -1;
    }
    counter += 1;
    cur_event = next_event;
  }    // capture EXIT:
  if (prog_shm->opcodes[cur_event]  != EXIT){
    printf("pp_time_new. Didn't find an EXIT! BIG FAT PROBLEM!\n");
    return -1;
  }
  

  return 0;
}


int pre_gen_rx_events_noisy(){
  int cur_event=0,next_event = 0;
  unsigned int jsr_stack[STACK_SIZE];
  unsigned int loop_level=0;;
  int loops[MAX_LOOP_LEVELS];
  uint64_t loop_time[MAX_LOOP_LEVELS];
  
  uint64_t accum_time = 0;
  int jsr_num=0;
  int counter = 0;
  int i;
  char was_on[NUM_RX]; // if receiver was on in previous event
  
  
  for(i=0;i<NUM_RX;i++){
    prog_shm->rx_staging_num_events[i]=0;
    was_on[i] = 0;
  }    
  cur_event = 0;
  counter=0;
  loop_time[0] = 0;
  accum_time = 0;
  // look at time of first timer.
  while(prog_shm->opcodes[cur_event] != EXIT && counter < MAX_ALL_EVENTS){
      //    printf("event: %i, outputs: %i, opcode: %i duration: %f\n",cur_event,prog_shm->outputs[0][cur_event],prog_shm->opcodes[0][cur_event],prog_shm->times[0][cur_event]);

    for (i=0;i<NUM_RX;i++){
      if ((loop_level > 0) && (prog_shm->rx_on[i][cur_event] != was_on[i])){
	printf("In noisy mode, RX has to be on or off through entire loop\n");
	return -1;
      }
      if (prog_shm->rx_on[i][cur_event] == 1){
	add_rx_event(was_on[i], i, accum_time, prog_shm->times[cur_event]);
	was_on[i] = 1;
	if (prog_shm->rx_staging_num_events[i] > MAX_RX_EVENTS/2){
	  printf("Too many RX events!\n");
	  return -1;
	}
      }
      else
	was_on[i] = 0;
    }
    accum_time += prog_shm->times[cur_event];
    switch (prog_shm->opcodes[cur_event] ){
    case CONTINUE:
    case SUBSTART:
    case DUE_ALT:
    case DUE_DAC:
      loop_time[loop_level] += prog_shm->times[cur_event];
      next_event = cur_event+1;
      break;
    case WAIT:
    case WAIT_MAX:
      loop_time[loop_level] += prog_shm->times[cur_event];
      fprintf(stderr,"pulse-duesdr. Got a WAIT event. THIS WON'T WORK. RX will get out sync.\n");
      //      how_long += prog_shm->times[0][cur_event]*prog_shm->opinst[0][cur_event];
      next_event = cur_event+1;
      fprintf(stderr,"Got a WAIT, times WILL be off\n");
      return -1;
      break;
    case LOOP:
      loop_level += 1;
      if (loop_level >= MAX_LOOP_LEVELS-1){
	fprintf(stderr, "Got a start loop too many levels deep\n");
	return -1;
      }
      loop_time[loop_level] = prog_shm->times[cur_event];
      loops[loop_level] = prog_shm->opinst[cur_event]; // load the loop counter.
      next_event = cur_event+1;
      break;
    case END_LOOP:
      if (loop_level < 1){
	fprintf(stderr,"got a loop end but not in a loop?\n");
	return -1;
      }
      loop_time[loop_level] += prog_shm->times[cur_event];
      loop_level -= 1;
      loop_time[loop_level] += loop_time[loop_level+1]*loops[loop_level+1];
      for (i=0;i<NUM_RX;i++){
	if (was_on[i]) // fix up the counter
	  prog_shm->rx_staging_duration[i][prog_shm->rx_staging_num_events[i]-1] *=
	    loops[loop_level +1];
      }
      next_event = cur_event +1;
      break;
    case JSR:
      loop_time[loop_level] += prog_shm->times[cur_event];
      jsr_stack[jsr_num] = cur_event + 1;
      jsr_num += 1;
      if (jsr_num > STACK_SIZE -1){
	fprintf(stderr,"too many JSR's nested\n");
      }
      next_event = prog_shm->opinst[cur_event];
      if (next_event > MAX_EVENTS || cur_event < 0){
	fprintf(stderr,"got a jsr to an event that doesn't exist.\n");
	return -1;
      }
      break;
    case RTS:
      loop_time[loop_level] += prog_shm->times[cur_event];
      if (jsr_num < 1){
	fprintf(stderr,"got a JSR, but nowhere to return to on stack!\n");
      }
      next_event = jsr_stack[jsr_num -1];
      jsr_num -=1;
      if (next_event <0 || next_event > MAX_EVENTS){
	fprintf(stderr,"got a RTS to a non-existant event!\n");
	return -1;
      }
      break;
    case EXIT:
      loop_time[loop_level] += prog_shm->times[cur_event];
      if (loop_level != 0){
	fprintf(stderr,"Got exit program, but appear to be inside a loop?\n");
	return -1;
      }
    default:
      fprintf(stderr,"got an unknown opcode!\n");
      return -1;
    }
    counter += 1;
    cur_event = next_event;
  }    // capture EXIT:
  if (prog_shm->opcodes[cur_event]  != EXIT){
    printf("pp_time_new. Didn't find an EXIT! BIG FAT PROBLEM!\n");
    return -1;
  }
  

  return 0;
}


// obsolete - to remove. XX TODO
void pulse_hardware_build_startup_soapy(due_prog_t *due_prog,int num_boards){ 
  int i;
  //  int complete,err;
  uint32_t default_mask;
  // need for any bits that idle high to be set high in here.
  // pretend for a moment that we're a pulse program in order to get
  // the output mask set up.

  init_hardware();
  for (i=0;i<num_boards;i++){
    default_mask = get_default_mask(i);
    //    printf("Default mask for board %i was 0x%x\n",i,(unsigned int)default_mask);
    due_init_program(&due_prog[i],1);

    due_add_event(&due_prog[i],default_mask,20000e-6*DUE_PP_CLOCK); // 20000 us of nothing
    if (i == 0){
      //due_add_event(&due_prog[i],(1<<CGEN_SYNC)|default_mask,1e-6*DUE_PP_CLOCK); // hit reset on CGEN clock generator
      //      due_add_event(&due_prog[i],default_mask,1e-6*DUE_PP_CLOCK); 
      //      due_add_event(&due_prog[i],default_mask,500e-6*DUE_PP_CLOCK); // whoa. if programmer is started first, this needs to be long.
      //      due_add_event(&due_prog[i],(1<<LIME_SYNC)|default_mask,1500e-6 *DUE_PP_CLOCK); // 1000 us pulse on 1st output needs to last at least one full packet - duration should depend on tx sample rate
      
    }
    else{
      //      due_add_event(&due_prog[i],default_mask,1e-6*DUE_PP_CLOCK); // hit reset on CGEN clock generator
      //      due_add_event(&due_prog[i],default_mask,500e-6*DUE_PP_CLOCK);
      //      due_add_event(&due_prog[i],default_mask,1500e-6 * DUE_PP_CLOCK);
    }
    due_add_event(&due_prog[i],default_mask,.025 * DUE_PP_CLOCK); // just 50ms of nothing.
    due_add_event(&due_prog[i],default_mask,.625 * DUE_PP_CLOCK);
    due_add_event(&due_prog[i],default_mask,.2 * DUE_PP_CLOCK);
    due_exit_program(&due_prog[i]);
    due_finalize_program(&due_prog[i]);
    //    due_dump_program(&due_prog[i]);
  }
}

void pulse_hardware_build_startup_soapy_new(due_prog_t *due_prog,int num_boards){
  int complete,err;
  double sf1=0,sf2=0;
  unsigned int aval;
  // need for any bits that idle high to be set high in here.
  // pretend for a moment that we're a pulse program in order to get
  // the output mask set up.
  //  pulse_program_init_min();
  init_hardware();
  init_data();
  fetch_double("sf1", &sf1);
  fetch_double("sf2", &sf2);
  if (sf1 == 0. && sf2 == 0.){
    printf("build program found both frequencies of 0???, its not going to work...\n");
  }
  else
    printf("build_startup_new, got freqs: %f, %f\n",sf1,sf2);

 
  
  begin();
  // put the frequencies where the prep_sdr will find them

  if (NUM_RX > 0)
    prog_shm->txfreqs[0] = sf1;
  if (NUM_RX > 1)
    prog_shm->txfreqs[1] = sf2;

  event_duesdr(0.1, 0,0,0); // 0.1 s of nothing.
  event_duesdr(1e-3+100e-9, 0,0,0); // 1ms of nothing.
  prog_shm->do_synth_setup = 1;
  setup_synths(sf1,sf2);
  event_duesdr(10e-3,0,0,0); // wait 10 ms

  // but here we want to turn the Q amplitude up, and the I amplitude down.
  aval = 4095; // max!
#ifdef WR_B
  // turn up Q amplitude:
  event_duesdr(1e-6,0,0,6,ADD_A,0x23,DAT_A,(aval>>8)&15,WR_A,1,ADD_B,0x23,DAT_B,(aval>>8)&15,WR_B,1);
  event_duesdr(1e-6,0,0,6,ADD_A,0x24,DAT_A,aval&255,WR_A,0,ADD_B,0x24,DAT_B,aval&255,WR_B,0);
  event_duesdr(1e-6,0,0,2,UPD_A,1,UPD_B,1);
#else
  // turn up Q amplitude:
  event_duesdr(1e-6,0,0,3,ADD_A,0x23,DAT_A,(aval>>8)&15,WR_A,1);
  event_duesdr(1e-6,0,0,3,ADD_A,0x24,DAT_A,aval&255,WR_A,0);
  event_duesdr(1e-6,0,0,1,UPD_A,1);
#endif

  // that turns on our pulse
  
  // wait, then shut it off. The rf is on for 50 us. Turns off at start of UPD event.
  // give the pulse:
  event_duesdr(50e-6-3e-6,0,0,1, BNC_0,1);

  // the duration needs to be set in rx_thread_func where the synchronization is done.
#ifdef WR_B
  // turn down Q amplitude:
  event_duesdr(1e-6,0,0,6,ADD_A,0x23,DAT_A,0,WR_A,1,ADD_B,0x23,DAT_B,0,WR_B,1);
  event_duesdr(1e-6,0,0,6,ADD_A,0x24,DAT_A,0,WR_A,0,ADD_B,0x24,DAT_B,0,WR_B,0);
  event_duesdr(1e-6,0,0,2,UPD_A,1,UPD_B,1);
#else
  // turn down Q amplitude:
  event_duesdr(1e-6,0,0,3,ADD_A,0x23,DAT_A,0,WR_A,1);
  event_duesdr(1e-6,0,0,3,ADD_A,0x24,DAT_A,0,WR_A,0);
  event_duesdr(1e-6,0,0,1,UPD_A,1);
#endif
  event_duesdr(0.4, 0,0,0); // this needs to be long enough for the data to be collected and seen in the rx thread.
  // rf ends 1us before now.
  event_duesdr(0.1,EXIT,0,0); // so time from end of rf to end of program - is this plus the previous event. 1s+1e6;
  // this needs to get set in start_offset in open_sdr()
  printf("build startup, calling ready\n");
  guts_of_ready(0,&complete,&err); 
  if (complete != 1 || err !=0){
    printf("build startup program, guts_of_ready didn't return properly??\n");
  }
  printf("build startup, back from ready, copying program\n");

  // copy the program from shm into the passed in due_prog. 
  memcpy(due_prog, prog_shm->due_prog,sizeof(due_prog_t)*NUM_BOARDS);
  printf("build startup, returning\n");

}
