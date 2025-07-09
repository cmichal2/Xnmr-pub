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
  open's the pulse program:

  do{
  call's the user's pprogram routine, which is basically what's 
  in-between the do{ }while above.
  }while(ready(phase) = flag)


  To do this, need to figure out how to compile loadable modules...

   */


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

#include "due-pp-lib.h"
 #include "pulse-duelime.h" 
 #include "/usr/share/Xnmr/config/h_config-duelime.h" 
 #include "p_signals.h" 
 #include "shm_data.h" 
 #include "shm_prog-duelime.h" 
 #include "param_utils.h" 
 #include "duepp.h"
 #include <unistd.h> 
 #include <sys/time.h> 
 #include <time.h>
#include "lime-duepp.h"


/// XXX TODO temporary skew of clock timing to make due and lime match. Remove !
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
int txevent_numbers[NUM_TX][MAX_LABELS];
int gradevent_numbers[MAX_LABELS];
int num_event_labels;

//the next five hold info about events that need to be resolved (eg JSR routines would have these)
char labels_to_resolve[MAX_LABELS][MAX_LABEL_LEN];
int events_to_resolve[MAX_LABELS];
int txevents_to_resolve[NUM_TX][MAX_LABELS];
int gradevents_to_resolve[MAX_LABELS];
int num_events_to_resolve;


struct hardware_config_t *hardware_config; 
struct data_shm_t* data_shm; 
struct prog_shm_t*  prog_shm;        //These are shared memory structures and must be initialized 
unsigned int latch_mask[NUM_BOARDS],default_mask[NUM_BOARDS];

struct itimerval mytime,old; 
int data_shm_id; 
int prog_shm_id; 
int msgq_id; 
int finished; 

int tran_table[6]; 


// a few prototypes
int resolve_labels();
uint64_t points_to_receive(int rxno);
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

int ready( char phase ) { 
  int result,i,j; 
  struct msgbuf message; 
  static int first_time = 1;
  int err=0;
  txprog_t *txprog;
  gradprog_t *gradprog;
  
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
    err = P_PROGRAM_ERROR;
  }

  if (bgu_inited)
    if ( bgu_last_vals_zeros == 0){
      printf("bgu was inited, but last value wasn't all zeros! \n");
      err = P_PROGRAM_ERROR;
    }

  //  printf("\nSTARTING TO ASSEMBLE DUE PROGRAM\n");
  // now go through and build the due program
  for (i=0;i<NUM_BOARDS;i++)
    if (due_init_program(&prog_shm->due_prog[i],1) != 0){
      err = P_PROGRAM_ERROR;
      printf("trouble initing due_programs\n");
    }

  for (i=0;i<prog_shm->no_events;i++){
    for(j=0;j<NUM_BOARDS;j++){
      switch (prog_shm->opcodes[i]){
      case CONTINUE: // just a plain jane event.
	if (due_add_event(&prog_shm->due_prog[j],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  err = P_PROGRAM_ERROR;
	  printf("trouble adding event to due program\n");
	}
	break;
      case LOOP: // loop start
	if (due_start_loop(&prog_shm->due_prog[j],prog_shm->opinst[i],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  err = P_PROGRAM_ERROR;
	  printf("trouble adding event to due program\n");
	}
	break;
      case END_LOOP:
	if (due_end_loop(&prog_shm->due_prog[j],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  err = P_PROGRAM_ERROR;
	  printf("trouble adding END_LOOP event to due program\n");
	}
	break;
      case JSR: // here we need a numeric identifier for the subroutine.
	//	printf("got JSR at event %i, to event: %i, index %i\n",i,prog_shm->opinst[j][i],prog_shm->subids[j][prog_shm->opinst[j][i]]);
	if (due_call_sub(&prog_shm->due_prog[j],prog_shm->subids[prog_shm->opinst[i]],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  err = P_PROGRAM_ERROR;
	  printf("trouble adding JSR event to due program\n");
	}
	break;
      case SUBSTART:
	//	printf("got SUBSTART at event %i. With index %i\n",i,prog_shm->subids[j][i]);
	if (due_start_sub(&prog_shm->due_prog[j],prog_shm->subids[i]) != 0){
	  err = P_PROGRAM_ERROR;
	  printf("trouble calling due_start_sub\n");
	}
	if (due_add_event(&prog_shm->due_prog[j],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  err = P_PROGRAM_ERROR;
	  printf("trouble adding SUBSTART event to due program\n");
	}
	break;
      case RTS:
	if (due_return_from_sub(&prog_shm->due_prog[j],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  err = P_PROGRAM_ERROR;
	  printf("trouble adding RTS event to due program\n");
	}
	break;
      case WAIT:
	if (due_wait_for_trigger(&prog_shm->due_prog[j],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  err = P_PROGRAM_ERROR;
	  printf("trouble adding trigger event to due program\n");
	}
	break;
      case WAIT_MAX:
	if (due_wait_for_trigger_max(&prog_shm->due_prog[j],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) !=0){
	  err = P_PROGRAM_ERROR;
	  printf("trouble adding trigger_max event to due program\n");
	}
	break;
      case EXIT:
	if (due_add_event(&prog_shm->due_prog[j],prog_shm->outputs[j][i], prog_shm->times[i]/FUDGE) != 0){
	  err = P_PROGRAM_ERROR;
	  printf("trouble adding exit event to due program\n");
	}
	if (due_exit_program(&prog_shm->due_prog[j]) != 0){
	  err = P_PROGRAM_ERROR;
	  printf("trouble adding exit event to due program\n");
	}
	break;
      } // end switch on opcode

    }
  }
  for (i=0;i<NUM_BOARDS;i++)
    if (due_finalize_program(&prog_shm->due_prog[i]) != 0){
      err = P_PROGRAM_ERROR;
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
  for(i=0;i<NUM_TX;i++){
    prog_shm->rxpoints[i] = points_to_receive_new(i);
    printf("setting rxpoints for thread %i to %li\n",i,prog_shm->rxpoints[i]);
    prog_shm->totalpoints += prog_shm->rxpoints[i];
  }

  if (prog_shm->is_noisy == 0) {
    if (prog_shm->totalpoints > data_shm->npts){
      err = P_PROGRAM_ERROR;
      printf("FATAL ERROR: Pulse program found a total of %li points, but you've only set npts to %i\n",prog_shm->totalpoints,data_shm->npts);
    }
  }
  
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
  
  if (first_time == 1){
    //Dump the pulse program to a file in two formats 
    
    FILE *fid; 
    int event; 
    int board; 
    int bit; 
    
    fprintf(stderr, "dumping pulse program to file\n" ); 
    fprintf(stderr,"in dumping, first_time is: %i\n",first_time);
    fid = fopen( "pprog.txt", "w" ); 
    fprintf(fid,"event, opcode, opinst, time, output bits, next board\n");
    for( event=0; event<prog_shm->no_events; event++ ) { 
      for( board=0; board<NUM_BOARDS; board++ ) {
	/*
	// calculate clock ticks as in spinapi.c pb_inst_pbonly64
	unsigned int delay;
	double pb_clock=0.1; // 0.1 GHz
	delay = (unsigned int) floor(0.5+( prog_shm->times[board][event]*1e9 *pb_clock) -3.0);
	if (((delay & 0xFF) == 0xFF) && (delay > 0xFF))
	  printf("PB FIRMWARE FIX TRIGGERED on event %i, requested time: %lf ns\n",event,prog_hsm->times[board][event]*1e9);
	*/							     
	fprintf(fid,"%3i %i %2i %9i ",event,prog_shm->opcodes[event],prog_shm->opinst[event],prog_shm->times[event]);
	for( bit=0; bit<24; bit++ ) { 
	  fprintf( fid, "%d", (prog_shm->outputs[ board ][ event ] >> bit) %2 ); 
	} 
	fprintf( fid, " " ); 
      }
      fprintf( fid, "\n" );       

    }
    // then dump the tx program
    txprog = &prog_shm->txprog[prog_shm->txprogno];
    fprintf(fid,"tx program: event no, time, opcode, opinst, param0, param1\n");
    for (i=0;i<txprog->txevents[0];i++)
      fprintf(fid,"%i %li %i %i %i %i\n",i,txprog->txtimes[0][i],txprog->txopcodes[0][i],txprog->txopinst[0][i],txprog->txparams[0][i][0],txprog->txparams[0][i][1]);
    if (bgu_inited){
      // finally dump the gradient program:
      gradprog = &prog_shm->gradprog[prog_shm->txprogno];
      fprintf(fid,"gradient program: event no, opcode, opinst, is_grad_event, x, y, z\n");
      for(i=0;i<gradprog->gradevents;i++)
	fprintf(fid,"%i %i %i %i %i %i %i\n",i,gradprog->gradopcode[i],gradprog->gradopinst[i],gradprog->is_grad_event[i],gradprog->x[i],gradprog->y[i],gradprog->z[i]);
    }
    
  fclose( fid ); 
  }
  

    // end dump

  // begun may not have been set - if we didn't actually do anything...
  prog_shm->begun = 0;
  prog_shm->prog_ready = READY; 
 
   if (first_time == 1){
     first_time =0;
     //     fprintf(stderr,"got first time, setting to 0\n");
   }
   
   /// XX TODO: is this the right place for this?
   if( finished == 1 ) {
     return P_PROGRAM_END; 
   }

   //   fprintf(stderr, "Pulse Program calculation complete\n" ); 

     //     phase = (4-phase)%4; // reverse the frequencies.

   //   fprintf(stderr, "pprog sending message P_PROGRAM_READY\n" ); 
  if (err == 0 && prog_shm->event_error == 0){
    //    printf("pulse.c: telling acq program is ready\n");
    message.mtype = P_PROGRAM_READY; 
    message.mtext[0] = P_PROGRAM_READY;
  }
  else{
    printf("pulse-duelime.c: telling acq we have an error\n");
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

int soft_loops_gradstart[NUM_SOFT_LOOPS];
int soft_loops_gradend[NUM_SOFT_LOOPS];

int soft_loops_txstart[NUM_TX][NUM_SOFT_LOOPS];
int soft_loops_txend[NUM_TX][NUM_SOFT_LOOPS];
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

so the start is only called once, but the end is called once per trip through the loop.
*/
int hint_loop_start(){
  int i;
  //  printf("hint start current event is: %i\n",prog_shm->no_events);
  if (num_soft_loops >= NUM_SOFT_LOOPS-1){
    printf("Got too many soft loops!\n");
    prog_shm->event_error = 1;
    return -1;
  }
  printf("hint_loop_start, start event is: %i\n",prog_shm->no_events);
  soft_loops_start[num_soft_loops] = prog_shm->no_events; // this for the next event
  soft_loops_iterations[num_soft_loops] = 0;
  for (i=0;i<NUM_TX;i++){
    soft_loops_txstart[i][num_soft_loops] = prog_shm->txprog[prog_shm->txprogno].txevents[i];
  }
  soft_loops_gradstart[num_soft_loops] = prog_shm->gradprog[prog_shm->txprogno].gradevents;
  
  num_soft_loops += 1;
  return 0;
}
int hint_loop_end(){
  int i,j,k,loopno,events_in_loop;
  txprog_t  *txprog;
  gradprog_t *gradprog;
  txprog = &prog_shm->txprog[prog_shm->txprogno];
  gradprog = &prog_shm->gradprog[prog_shm->txprogno];
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
    for (i=0;i<NUM_TX;i++)
      soft_loops_txend[i][loopno] = txprog->txevents[i]-1; // this is the previous event.
    soft_loops_gradend[loopno] = gradprog->gradevents-1;
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
    else printf("Not adding one more iteration to loop for duelime\n");
  }

  /////////////// now tx:
  // ok, here we need to see if what we just did can be folded into a loop.
  if (soft_loops_iterations[loopno] == 2){ // first time we need to see if we can form a loop.
    for (k=0;k<NUM_TX;k++){
      events_in_loop = soft_loops_txend[k][loopno] - soft_loops_txstart[k][loopno]+1; // inclusive.
      ok_to_loop = 1;
      // first check to make sure that the first and last events are just simple events that can be converted
      // into loop start and end
      if ((txprog->txopcodes[k][soft_loops_txstart[k][loopno]] & 15) != CONTINUE ){
	printf("tx in hint_loop_end - first event is not CONTINUE, can't convert to hard loop\n");
	ok_to_loop = 0;
      }
      if ((txprog->txopcodes[k][soft_loops_txend[k][loopno]] & 15) != CONTINUE ){
	printf("tx in hint_loop_end - final event in loop is not CONTINUE, can't convert to hard loop\n");
	ok_to_loop = 0;
      }
      // for tx, doesn't matter if events are short.
      // ok, start and end are ok. See if everything is the same.
      for (i=0;i<events_in_loop;i++){// these are the events to check
	if (txprog->txopcodes[k][i+soft_loops_txstart[k][loopno]] != txprog->txopcodes[k][i+txprog->txevents[k]-events_in_loop]){
	  printf("tx hint end: got a mismatched opcode\n");
	  ok_to_loop = 0;
	}
	if (txprog->txopinst[k][i+soft_loops_txstart[k][loopno]] != txprog->txopinst[k][i+txprog->txevents[k]-events_in_loop]){
	  printf("tx hint end: got a mismatched opinst\n");
	  ok_to_loop = 0;
	}
	if (txprog->txtimes[k][i+soft_loops_txstart[k][loopno]] != txprog->txtimes[k][i+txprog->txevents[k]-events_in_loop]){
	  printf("tx hint end: got a mismatched time:\n");
	  ok_to_loop = 0;
	}
	for (j=0;j<3;j++) // check the tx params
	  if (txprog->txparams[k][i+soft_loops_txstart[k][loopno]][j] != txprog->txparams[k][i+txprog->txevents[k]-events_in_loop][j]){
	    printf("tx hint end: got a mismatched tx param\n");
	    ok_to_loop = 0;
	  }
      }
      if (soft_loops_txstart[k][loopno] == soft_loops_txend[k][loopno]){
	printf("tx loop has only 1 event, can't convert to hard loop. Were events collapsed?\n");
	ok_to_loop = 0;
      }
      // if we've made it here, we're good to go!
      // convert the first pass into a loop with 2 iterations:
      if (ok_to_loop){
	printf("TX: Converting soft loop to hard loop\n");
	//    printf("start and end events are: %i and %i\n",soft_loops_start[loopno],soft_loops_end[loopno]);
	txprog->txopcodes[k][soft_loops_txstart[k][loopno]] |= LOOP;
	txprog->txopinst[k][soft_loops_txstart[k][loopno]] = 2;
	txprog->txopcodes[k][soft_loops_txend[k][loopno]] |= END_LOOP;
	txprog->txevents[k] -= events_in_loop;
      }
      else printf("Not converting soft to hard loop for tx\n");

    }
  }// finished 2nd iteration.
  // if we're here, we're on the 3rd or higher iteration.
  else{
    for(k=0;k<NUM_TX;k++){
      events_in_loop = soft_loops_txend[k][loopno] - soft_loops_txstart[k][loopno]+1; // inclusive.
      ok_to_loop = 1;
      if ((txprog->txopcodes[k][soft_loops_txstart[k][loopno]] & 15) != LOOP ){ // check first event of first iteration
	printf("tx in hint_loop_end - first event is not LOOP_START it is:%i\n",txprog->txopcodes[k][soft_loops_txstart[k][loopno]]);
	ok_to_loop = 0;
      }
      if ((txprog->txopcodes[k][soft_loops_txend[k][loopno]] & 15) != END_LOOP ){// check last event of first iteration
	printf("in tx hint_loop_end - final event in loop is not CONTINUE, can't convert to hard loop\n");
	ok_to_loop = 0;
      }
      if ((txprog->txopcodes[k][txprog->txevents[k] -events_in_loop] & 15) != CONTINUE ){ // check first event of most recent iteration
	printf("in tx hint_loop_end - first event is not LOOP_START\n");
	ok_to_loop = 0;
      }
      if ((txprog->txopcodes[k][txprog->txevents[k]-1] & 15) != CONTINUE ){ // check last event of most recent iteration.
	printf("tx in hint_loop_end - final event in loop is not CONTINUE, can't convert to hard loop\n");
	ok_to_loop = 0;
      }
      /// need to check the tx part of the opcodes of the first and last events too! TODO
      if ((txprog->txopcodes[k][soft_loops_txstart[k][loopno]] & 240) !=  (txprog->txopcodes[k][txprog->txevents[k]  -events_in_loop]& 240)){
	printf("hint tx first opcodes don't match.\n");
	ok_to_loop = 0;
      }
      if ((txprog->txopcodes[k][soft_loops_txend[k][loopno]] & 240) !=  (txprog->txopcodes[k][txprog->txevents[k]  -events_in_loop] & 240)){
	printf("tx hint tx first opcodes don't match.\n");
	ok_to_loop = 0;
      }
            
      // all other opcodes should match:
      for (i=1;i<events_in_loop-1;i++){// these are the events to check - skip first and last - just did them.
	if (txprog->txopcodes[k][i+soft_loops_txstart[k][loopno]] != txprog->txopcodes[k][i+txprog->txevents[k]-events_in_loop]){
	  printf("tx hint end: got a mismatched opcode\n");
	  ok_to_loop = 0;
	}
	if (txprog->txopinst[k][i+soft_loops_txstart[k][loopno]] != txprog->txopinst[k][i+txprog->txevents[k]-events_in_loop]){
	  printf("tx hint end: got a mismatched opinst\n");
	  ok_to_loop = 0;
	}
      }
      for(i=0;i<events_in_loop;i++){ // check  all the txparams
	for(j=0;j<3;j++){
	  if (txprog->txparams[k][i+soft_loops_txstart[k][loopno]][j] !=
	      txprog->txparams[k][i+txprog->txevents[k]-events_in_loop][j]){
	    printf("tx hint end: got a mismatched param\n");
	    ok_to_loop = 0;
	  }
	}
	if (txprog->txtimes[k][i+soft_loops_txstart[k][loopno]] != txprog->txtimes[k][i+txprog->txevents[k]-events_in_loop]){
	  printf("tx hint end: got a mismatched time\n");
	  ok_to_loop = 0;
	}
      }
      // add one more iteration to loop:
      if (ok_to_loop){
	txprog->txopinst[k][soft_loops_txstart[k][loopno]] += 1;
	txprog->txevents[k] -= events_in_loop;
      }
      else printf("Not adding one more iteration for tx\n");
    } // end loopo over tx boards
  }// end 3rd + higher iteration.
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
int do_event_duelime(double time, int opcode, int opinst, unsigned char num, va_list args ){
  // this is wrapped by event_duelime so we take the va_list as an argument here.
  int i,j,intval=0; 
   unsigned char device_id; 
   double dval,dval2;
   uint64_t itime;
   int ratio;
   txprog_t *txprog;
   gradprog_t *gradprog;
   txprog = &prog_shm->txprog[prog_shm->txprogno];
   gradprog = &prog_shm->gradprog[prog_shm->txprogno];
   
   //   fprintf(stderr,"\ncoming into event, number is: %i\n",prog_shm->no_events);
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
   for(i=0;i<NUM_TX;i++)
     if (txprog->txevents[i] >= MAX_TX_EVENTS-1){
       fprintf(stderr,"pprog: too many tx events\n");
       prog_shm->event_error = 1;
       return -1;
     }
   //   fprintf(stderr,"event_no: %i time: %lf \n",prog_shm->no_events,time);
   if (gradprog->gradevents >= MAX_GRAD_EVENTS-1){
     prog_shm->event_error = 1;
     printf("Too many grad events!\n");
     return -1;
   }


   // shortest event is 200ns, longest (without being a long event is 2^32 * 20ns

   if (!isnormal(time)){
     if (time != 0.){
       fprintf(stderr,"TIME IS NOT A NUMBER!\n");
       prog_shm->event_error = 1;
       return -1;
     }
   }
   if (time < 0.){
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
   
   // to keep limesdr sync'd, needs to be a multiple of TX_SAMPLE_RATE
   ratio = DUE_PP_CLOCK/TX_SAMPLE_RATE; // should be an integer - 50 MHz, and 10 for now, maybe 25.
   itime = (itime/ratio)*ratio;
   //   printf("incoming time of %g set to %li ticks, = %g\n",time,itime,(double)itime/DUE_PP_CLOCK);

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

   // set up for tx event:
   for(j=0;j<NUM_TX;j++){
     txprog->txtimes[j][txprog->txevents[j]] = 0;
     txprog->txopcodes[j][txprog->txevents[j]] = 0; 
     txprog->txparams[j][txprog->txevents[j]][0] = 0;
     txprog->txparams[j][txprog->txevents[j]][1] = 0;
     txprog->txparams[j][txprog->txevents[j]][2] = 0;
     

   }
     
   //set all the specified device information 
   //   fprintf(stderr,"\nin event, no_events: %i,num things this event: %i\n",prog_shm->no_events,num);
   for( i=0; i<num; i++ ) { 
     device_id = (unsigned char) va_arg( args, int  );
     //     printf("event: %i, device: %i of %i devices\n",prog_shm->no_events,device_id,num);
     if (device_id == LABEL){ // its a pseudo device with just a label.
       // insert the label into the tables to be found later. Should only be SUBSTART that has labels.
       char *lab;
       lab = va_arg(args,char *);
       //       printf("got a label: %s at instruction: %i, index: %i\n",lab,prog_shm->no_events,num_event_labels[0]);
       strncpy(event_labels[num_event_labels],lab,MAX_LABEL_LEN);
       event_numbers[num_event_labels] = prog_shm->no_events;
       for(j=0;j<NUM_TX;j++)
	 txevent_numbers[j][num_event_labels] = txprog->txevents[j];
       gradevent_numbers[num_event_labels] = gradprog->gradevents;
       if (num_event_labels > MAX_LABELS -1){
	 printf("too many event labels!\n");
	 prog_shm->event_error = 1;
       }
       else
	 num_event_labels += 1;
     }
     else if (device_id == TX1 ){
       int chan;
       dval= (double) va_arg(args,double);
       dval2= (double) va_arg(args,double);
       if (dval2 < 0 || dval2 >= 360.)
	 dval2 = dval2-360*floor(dval2/360.);
       //       printf("got a TX1 event with amp: %f, phase: %f\n",dval,dval2);
       if (data_shm->ch1 == 'A')
	 chan = 0;
       else
	 chan = 1;
       /*
       if (prog_shm->lime_sync_event[chan] == 0){
	 printf("got a TX event but lime_sync_event not set\n");
	 prog_shm->event_error = 1;
	 } */
       txprog->txopcodes[chan][txprog->txevents[chan]]  |= 16; // low bits are the pb opcodes, high bits are ours 16 = tx, 32 is set offset, 64 is RX !
       txprog->txparams[chan][txprog->txevents[chan]][0] = 32767*dval;  // amp to +/- 32767 uses 16 bits in a 32 bit num
       txprog->txparams[chan][txprog->txevents[chan]][1] = TRIGLEN*dval2/360; // phase
     }
     else if (device_id == TX2){
       int chan;
       dval= (double) va_arg(args,double);
       dval2= (double) va_arg(args,double);
       if (data_shm->ch2 == 'A')
	 chan = 0;
       else
	 chan = 1;
       /*
       if (prog_shm->lime_sync_event[chan] == 0){
	 printf("got a TX event but lime_sync_event not set\n");
	 prog_shm->event_error = 1;
	 } */
       txprog->txopcodes[chan][txprog->txevents[chan]] |= 16;
       txprog->txparams[chan][txprog->txevents[chan]][0] = 32767*dval;  // amp - to +/- 32767 from 1 to -1
       txprog->txparams[chan][txprog->txevents[chan]][1] = TRIGLEN*dval2/360; // phase as bit int.
     }
     else if (device_id == RX1 ){
       int chan;
       dval= (double) va_arg(args,double);
       if (data_shm->ch1 == 'A')
	 chan = 0;
       else
	 chan = 1;
       /*
       if (prog_shm->lime_sync_event[chan] == 0){
	 printf("got an RX event but lime_sync_event not set\n");
	 prog_shm->event_error = 1;
	 } */
       txprog->txopcodes[chan][txprog->txevents[chan]]  |= 64; // low bits are the pb opcodes, high bits are ours 16 = tx, 32 is set offset, 64 is RX !
       txprog->txparams[chan][txprog->txevents[chan]][2] = dval; // holds the phase.
     }
     else if (device_id == RX2 ){
       int chan;
       dval= (double) va_arg(args,double);
       if (data_shm->ch2 == 'A')
	 chan = 0;
       else
	 chan = 1;
       /*
       if (prog_shm->lime_sync_event[chan] == 0){
	 printf("got an RX event but lime_sync_event not set\n");
	 prog_shm->event_error = 1;
	 } */
       txprog->txopcodes[chan][txprog->txevents[chan]]  |= 64; // low bits are the pb opcodes, high bits are ours 16 = tx, 32 is set offset, 64 is RX !
       txprog->txparams[chan][txprog->txevents[chan]][2] = dval; // holds the phase.
     }
     else{ // any ordinary pulse programmer device:
       //       fprintf(stderr,"got device_id: %i for event: %i\n",(int) device_id,prog_shm->no_events);
       intval =  va_arg(args,unsigned int);
       write_device_wrap(prog_shm->no_events,prog_shm->no_events,device_id,intval);
     }
   } 


   
   // txopcode can contain a low value (for the pb opcodes) as well as 16 for tx, 32 for offset or 64 for rx. can have 16 and 64 simultaneously!
   for(i=0;i<NUM_TX;i++){
     txprog->txtimes[i][txprog->txevents[i]] = itime;
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
       if (prog_shm->no_events >= MAX_EVENTS - 1) prog_shm->event_error = 1;
       prog_shm->times[prog_shm->no_events] = 429467296-11;
       prog_shm->opcodes[prog_shm->no_events] = 0;
       prog_shm->opinst[prog_shm->no_events] = 0;
       for (i=0;i<NUM_BOARDS;i++){
	 prog_shm->outputs[i][prog_shm->no_events] = prog_shm->outputs[i][prog_shm->no_events-1];
       }
     }
     else{
       printf("Got an event time longer than 2 full events. Someone should set up pulse-duelime.c to build a loop here\n");
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
   prog_shm->opinst[prog_shm->no_events] = opinst; // most of these are wrong now, but will get 
   prog_shm->no_events += 1;
       // updated later by resolve
   for (i=0;i<NUM_TX;i++){
     txprog->txopcodes[i][txprog->txevents[i]] |= opcode;
     txprog->txopinst[i][txprog->txevents[i]] = opinst;

   }
   for(i=0;i<NUM_TX;i++)
     txprog->txevents[i] += 1;

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

   //   printf("see if we can collapse this event with the previous\n");
     
   // in here, see if current event and previous were identical. If so, collapse them.
   // this is so that events where the only thing that changes is rf amp or phase don't use up
   // duelime events.
   char ok_to_collapse = 1;
   if (prog_shm->no_events < 2) ok_to_collapse=0; // in case we're right at the start, don't check.
   else{
     if (soft_loops_start[num_soft_loops-1] == prog_shm->no_events-1)// don't collapse the first event of a hinted loop.
       ok_to_collapse = 0;
     if (num_soft_loops > 0) // don't collapse during hinted loop at all.
       ok_to_collapse = 0;
     if (prog_shm->opcodes[prog_shm->no_events-1] != CONTINUE){
       ok_to_collapse = 0;
     }
     if (prog_shm->opcodes[prog_shm->no_events-2] != CONTINUE) {
       ok_to_collapse = 0;
     }
     if (prog_shm->opinst[prog_shm->no_events-1] != prog_shm->opinst[prog_shm->no_events-2]) {
       ok_to_collapse = 0;
     }
     for (i=0;i<NUM_BOARDS;i++){
       if (prog_shm->outputs[i][prog_shm->no_events-1] != prog_shm->outputs[i][prog_shm->no_events-2]){
	 ok_to_collapse = 0;
	 break;
       }   
     }
     if (ok_to_collapse){ // on pb boards collapse all or none.
       //       printf("Collapsing event %i!\n",prog_shm->no_events-1);
       prog_shm->times[prog_shm->no_events-2] += prog_shm->times[prog_shm->no_events-1];
       prog_shm->no_events -= 1 ;
     }
   }
   // do the same for the tx events: collapse identical events:
   for(i=0;i<NUM_TX;i++){
     if(txprog->txevents[i] >2){
       ok_to_collapse = 1;
       // lime sync event is one more than the event number it actually points to (which should always be the first event (0)).
       if (prog_shm->lime_sync_event[i] == txprog->txevents[i] - 1) // don't collapse the sync event! Need it as timing ref.
	 ok_to_collapse = 0;
       if (num_soft_loops > 0) // don't collapse during hinted loop at all.
	 ok_to_collapse = 0;
       if ((txprog->txopcodes[i][txprog->txevents[i]-1] & 15) != 0) // must not have a pb branching instruction.
	 ok_to_collapse = 0;
       if (txprog->txopcodes[i][txprog->txevents[i]-1] != txprog->txopcodes[i][txprog->txevents[i]-2]) // must not have a pb branching instruction.
	 ok_to_collapse = 0;
       if (txprog->txopinst[i][txprog->txevents[i]-1]  != txprog->txopinst[i][txprog->txevents[i]-2] ) 
	 ok_to_collapse = 0;
       if (txprog->txparams[i][txprog->txevents[i]-1][0] != txprog->txparams[i][txprog->txevents[i]-2][0])
	 ok_to_collapse = 0;
       if (txprog->txparams[i][txprog->txevents[i]-1][1] != txprog->txparams[i][txprog->txevents[i]-2][1])
	 ok_to_collapse = 0;
       if (txprog->txparams[i][txprog->txevents[i]-1][2] != txprog->txparams[i][txprog->txevents[i]-2][2])
	 ok_to_collapse = 0;
       if (ok_to_collapse){
	 //	 printf("collapsing event: adding time: %i to  %i\n",	 txprog->txtimes[i][txprog->txevents[i]-2],txprog->txtimes[i][txprog->txevents[i]-1]);
	 txprog->txtimes[i][txprog->txevents[i]-2] += txprog->txtimes[i][txprog->txevents[i]-1];
	 txprog->txevents[i] -= 1 ;	 
       }
     }
   }
   
   
   return 0; 

}

int event_duelime( double time, int opcode,int opinst,unsigned char num, ... ) 
 {
   int rval;
   va_list args;
   va_start(args,num);
   rval = do_event_duelime(time,opcode,opinst,num,args);
   va_end(args);
   return rval;
 }



int begin() {

  int i;
  
  for(i=0;i<NUM_TX;i++){
    prog_shm->txprog[prog_shm->txprogno].txevents[i]=0;
    prog_shm->lime_sync_event[i] = 0;
  }

  // copy old due program to save place for later comparison.
  
  for(i=0;i<NUM_BOARDS;i++){
    memcpy(old_progs[i].data,prog_shm->due_prog[i].data,MAXDATA*4);
    old_progs[i].dpos = prog_shm->due_prog[i].dpos;
  }
  
  num_soft_loops = 0;
  prog_shm->no_events = 0; 
  prog_shm->event_error = 0;
  prog_shm->got_ppo = 0;
  prog_shm->is_noisy = 0;
  prog_shm->use_bgu = 0;
  prog_shm->begun = 1;
  
  prog_shm->gradprog[prog_shm->txprogno].gradevents = 0;
  bgu_inited = 0;
  bgu_last_vals_zeros = 1;
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

  return 0; 
  
}

void done() 
     
{ 
  
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

   fid = fopen( "/usr/share/Xnmr/config/h_config-duelime.h", "r" ); 

   if (fid == NULL) {
     fprintf(stderr,"pulse.c: couldn't open h_config-duelime.h\n");
     exit(0);
   }

   // look for how many devices
   do { 
     eo = fgets( s, PATH_LENGTH, fid );  
   } while( strstr( s, "NUM_DEVICES" ) == NULL || eo == NULL ); 

   if (eo == NULL){
     fprintf(stderr,"pulse.c: didn't find the number of device in h_config-duelime.h\n");
     exit(0);
   }

   sscanf(s,"#define NUM_DEVICES %i",&num_dev);
   //   fprintf(stderr,"found num devices = %i\n",num_dev);

   //   fprintf(stderr,"sizeof hardware_config: %i\n",sizeof(*hardware_config));
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
  
  err = stat("/usr/share/Xnmr/config/h_config-duelime.h",&other_buff);
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
    exit(0);
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

  int i;
    //    printf("Got label %s to resolve at event %i\n",label,prog_shm->no_events);
  
  strncpy(labels_to_resolve[num_events_to_resolve],label,MAX_LABEL_LEN);
  events_to_resolve[num_events_to_resolve]=prog_shm->no_events;

  for(i=0;i<NUM_TX;i++){
    txevents_to_resolve[i][num_events_to_resolve]=prog_shm->txprog[prog_shm->txprogno].txevents[i];
  }

  gradevents_to_resolve[num_events_to_resolve]=prog_shm->gradprog[prog_shm->txprogno].gradevents;

  num_events_to_resolve += 1;
  if (num_events_to_resolve >= MAX_LABELS){
    printf("ERROR, overrun labels - need to increase MAX_LABELS\n");
    prog_shm->event_error = 1;
  }  
}

int resolve_labels(){
  int i,j,k;
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

	for (i=0;i<NUM_TX;i++)
	  prog_shm->txprog[prog_shm->txprogno].txopinst[i][txevents_to_resolve[i][j]] = txevent_numbers[i][k];

	prog_shm->gradprog[prog_shm->txprogno].gradopinst[gradevents_to_resolve[j]] = gradevent_numbers[k];
	
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

#define MAX_ALL_EVENTS 100000000
#define STACK_SIZE 50
uint64_t points_to_receive(int rxno){
  // parse the pulse program and see how many points will be received.
  unsigned int jsr_stack[STACK_SIZE],loop_stack[STACK_SIZE],loop_counter[STACK_SIZE],counter=0;
  int jsr_num=0,loop_num=0,cur_event,next_event=0;
  uint64_t points = 0,rx_time=0;
  txprog_t *txprog;
  txprog = &prog_shm->txprog[prog_shm->txprogno];
  cur_event = 0;
  counter = 0;
  while ((txprog->txopcodes[rxno][cur_event] & 15) != EXIT && counter < MAX_ALL_EVENTS){
    //    printf("receive points doing event: %i\n",cur_event);
    if (txprog->txopcodes[rxno][cur_event] & 64) rx_time += txprog->txtimes[rxno][cur_event];
    switch (txprog->txopcodes[rxno][cur_event] & 15){
    case CONTINUE:
    case WAIT:
    case WAIT_MAX:
    case SUBSTART:
      next_event = cur_event+1;
      break;
    case LOOP:
      loop_counter[loop_num] = txprog->txopinst[rxno][cur_event];
      loop_stack[loop_num] = cur_event;
      loop_num += 1;
      if (loop_counter[loop_num ] == 0){
	printf("GOT LOOP START WITH 0 LOOPS BADNESS\n");
      }
      if (loop_num > STACK_SIZE -1){
	printf("receive points. Too many loops nested. BAD THINGS WILL HAPPEN\n");
      }
      next_event = cur_event + 1;
      break;
    case END_LOOP:
      if (loop_num < 1) printf("receive points got an end_loop with no loops on stack BAD THINGS WILL HAPPEN\n");
      //      printf("got an END_LOOP at event %i in thread %i, with %i loops\n",cur_event,rxno,loop_counter[loop_num-1]);
      if (loop_counter[loop_num-1] < 0){
	printf("got end loop, but uninitialized number of loops. BAD THINGS WILL HAPPEN\n");
      }
      loop_counter[loop_num-1] -= 1;
      if (loop_counter[loop_num-1] <= 0){
	next_event = cur_event + 1;
	loop_num -= 1;
      }
      else{
	next_event = loop_stack[loop_num-1]+1;
	if (txprog->txopcodes[rxno][next_event-1] & 64) rx_time += txprog->txtimes[rxno][next_event-1];
      }
      break;
      case JSR:
	jsr_stack[jsr_num] = cur_event+1;
	jsr_num += 1;
	if (jsr_num > STACK_SIZE -1){
	  printf("receive points:, Too many JSR's nested. BAD THINGS WILL HAPPEN\n");
	}
	next_event = txprog->txopinst[rxno][cur_event];
	//	printf("jsr setting next event to: %i\n",next_event);
	if (next_event > txprog->txevents[rxno] || next_event < 0){
	  printf("got a JSR to a nonsense event\n");
	}
	break;
      case RTS:
	if (jsr_num < 1){
	  printf("pulse-duelime: got RTS, but nowhere to return to!\n");
	  next_event = cur_event +1 ;
	}
	next_event = jsr_stack[jsr_num-1];
	jsr_num -=1;
	if (next_event > txprog->txevents[rxno] || cur_event < 0){
	  printf("got a RTS to a nonsense event\n");
	}
	break;
      case EXIT:
	break;
      default:
	printf("calc receive points got an unknown opcode of %i\n",(txprog->txopcodes[rxno][cur_event] & 15));
	next_event = cur_event + 1;
    } // end of switch on opcode
    cur_event =next_event;
    counter += 1;
  }
  if ((txprog->txopcodes[rxno][cur_event]&15) != EXIT) {
    printf("didn't find EXIT in calculating points to receive, instead got opcode %i at event %i, counter is: %i\n",txprog->txopcodes[rxno][cur_event],cur_event,counter);
  }
  points = round(rx_time * prog_shm->rx_sw[rxno]/DUE_PP_CLOCK);
  printf("receive point for channel %i. Found time (in ticks): %li, points: %li\n",rxno,rx_time,points);
  
  return points;
  
}
uint64_t points_to_receive_new(int rxno){
  // parse the pulse program and see how many points will be received.
  unsigned int jsr_stack[STACK_SIZE],counter=0;
  uint64_t loop_rx_time[STACK_SIZE];
  int loops[STACK_SIZE];
  int jsr_num=0,cur_event,next_event=0,loop_level=0;
  uint64_t points = 0;
  txprog_t *txprog;
  txprog = &prog_shm->txprog[prog_shm->txprogno];

  loop_rx_time[0] = 0;
  cur_event = 0;
  counter = 0;
  while ((txprog->txopcodes[rxno][cur_event] & 15) != EXIT && counter < MAX_ALL_EVENTS){
    //    printf("receive points doing event: %i\n",cur_event);
    switch (txprog->txopcodes[rxno][cur_event] & 15){
    case CONTINUE:
    case WAIT:
    case WAIT_MAX:
    case SUBSTART:
    if (txprog->txopcodes[rxno][cur_event] & 64) loop_rx_time[loop_level] += txprog->txtimes[rxno][cur_event];
      next_event = cur_event+1;
      break;
    case LOOP:
      loop_level += 1;
      if (loop_level >= STACK_SIZE-1){
	printf("ERROR Got a start loop too many levels deep\n");
	prog_shm->event_error =1;
	return loop_rx_time[0]; // this is so wrong...
      }
      if (txprog->txopcodes[rxno][cur_event] & 64)
	loop_rx_time[loop_level] = txprog->txtimes[rxno][cur_event];
      else
	loop_rx_time[loop_level] = 0;
      loops[loop_level] = txprog->txopinst[rxno][cur_event];
      if (loops[loop_level] == 0){
	printf("GOT LOOP START WITH 0 LOOPS BADNESS\n");
      }
      next_event = cur_event + 1;
      break;
    case END_LOOP:
      //      printf("got an END_LOOP at event %i in thread %i, with %i loops\n",cur_event,rxno,loops[loop_level]);
      if (txprog->txopcodes[rxno][cur_event] & 64)
	loop_rx_time[loop_level] += txprog->txtimes[rxno][cur_event];
      printf("points_to_receive, end_loop, loop ticks was: %li, number of loops: %i\n",loop_rx_time[loop_level],loops[loop_level]);
      loop_level -= 1;
      loop_rx_time[loop_level] += loop_rx_time[loop_level+1]*loops[loop_level+1];
      next_event = cur_event + 1;
      break;
    case JSR:
      if (txprog->txopcodes[rxno][cur_event] & 64)
	loop_rx_time[loop_level] += txprog->txtimes[rxno][cur_event];
      jsr_stack[jsr_num] = cur_event+1;
      jsr_num += 1;
      if (jsr_num > STACK_SIZE -1){
	printf("receive points:, Too many JSR's nested. BAD THINGS WILL HAPPEN\n");
      }
      next_event = txprog->txopinst[rxno][cur_event];
      //	printf("jsr setting next event to: %i\n",next_event);
      if (next_event > txprog->txevents[rxno] || next_event < 0){
	printf("got a JSR to a nonsense event\n");
      }
      break;
    case RTS:
      if (txprog->txopcodes[rxno][cur_event] & 64)
	loop_rx_time[loop_level] += txprog->txtimes[rxno][cur_event];
      if (jsr_num < 1){
	printf("got RTS, but nowhere to return to!\n");
	next_event = cur_event +1 ;
      }
      next_event = jsr_stack[jsr_num-1];
      jsr_num -=1;
      if (next_event > txprog->txevents[rxno] || cur_event < 0){
	printf("got a RTS to a nonsense event\n");
      }
      break;
    case EXIT:
      if (txprog->txopcodes[rxno][cur_event] & 64)
	loop_rx_time[loop_level] += txprog->txtimes[rxno][cur_event];
      break;
    default:
      printf("calc receive points got an unknown opcode of %i\n",(txprog->txopcodes[rxno][cur_event] & 15));
      next_event = cur_event + 1;
    } // end of switch on opcode
    cur_event = next_event;
    counter += 1;
  }
  if ((txprog->txopcodes[rxno][cur_event]&15) != EXIT) {
    printf("didn't find EXIT in calculating points to receive\n");
  }
  points = round(loop_rx_time[0] * prog_shm->rx_sw[rxno]/DUE_PP_CLOCK);
  printf("receive point for channel %i. Found time (in ticks): %li, points: %li\n",rxno,loop_rx_time[0],points);
  
  return points;
  
}

/*
double partial_pp_time(int start,int end){
  // this routine duplicates pp_time in acq, almost exactly.  They should probably be merged together.
  // start and end are the first and last events to include in the count

#define STACK_SIZE 50
#define MAX_ALL_EVENTS 100000000
  int cur_event=0,cur_event_new;
  unsigned int jsr_stack[STACK_SIZE],local_opinst[MAX_EVENTS+1],loop_stack[STACK_SIZE];
  int jsr_num=0,loop_num=0;
  int counter = 0;
  double how_long=0.0;
  int i;
  
  
  for(i=0;i<MAX_EVENTS;i++)
    local_opinst[i]=-1;

  cur_event = start;
  // look at time of first timer.
  while(prog_shm->opcodes[cur_event] != EXIT && counter < MAX_ALL_EVENTS){

    if  ( prog_shm->is_noisy == 1) counter = 0; // so it doesn't overflow! not great... FIXME and in acq.c

    //    printf("event: %i, outputs: %i, opcode: %i duration: %f\n",cur_event,prog_shm->outputs[0][cur_event],prog_shm->opcodes[0][cur_event],prog_shm->times[cur_event]);
    switch (prog_shm->opcodes[cur_event] ){
    case CONTINUE:
    case SUBSTART:
      how_long += prog_shm->times[cur_event];
      cur_event+=1;
      break;
    case WAIT:
      how_long += prog_shm->times[cur_event];
      cur_event+=1;
      fprintf(stderr,"Got a WAIT, times may be off\n");
      break;
    case WAIT_MAX:
      how_long += prog_shm->times[cur_event];
      cur_event+=1;
      fprintf(stderr,"Got a WAIT_MAX, times may be off\n");
      break;
    case LOOP:
      local_opinst[cur_event]=prog_shm->opinst[cur_event];
      how_long += prog_shm->times[cur_event];
      if (loop_num > STACK_SIZE -1){
	fprintf(stderr,"Too many Loops nested\n");
	prog_shm->event_error = 1;
	return how_long/DUE_PP_CLOCK;
      }
      loop_stack[loop_num]=cur_event;
      loop_num += 1;
      cur_event+=1;
      break;
    case END_LOOP:
      how_long += prog_shm->times[cur_event];
      //      cur_event_new = prog_shm->opinst[cur_event];
      if (loop_num < 1){
	printf("got an end_loop, but no loop on stack\n");
	prog_shm->event_error = 1;
	return how_long/DUE_PP_CLOCK;
      }
      cur_event_new = loop_stack[loop_num-1];
      // make sure this was a loop event
      if (local_opinst[cur_event_new] < 0){
	fprintf(stderr,"got a END_LOOP at event: %i, but didn't get a reasonable number of times to loop\n",cur_event);
	prog_shm->event_error = 1;
	//	done = ERROR_DONE; // can't do this in here...
	return how_long/DUE_PP_CLOCK;
      }
      local_opinst[cur_event_new] -= 1;
      if (local_opinst[cur_event_new] > 0){
	cur_event = cur_event_new+1;
	how_long += prog_shm->times[cur_event_new];
	counter += 1;
      }
      else {
	cur_event += 1;
	loop_num -= 1;
      }
      break;
    case JSR:
      //      printf("got a jsr at event: %i, going to event: %i\n",
      //      cur_event,prog_shm->opinst[cur_event]);
      how_long += prog_shm->times[cur_event];
      jsr_stack[jsr_num] = cur_event + 1;
      jsr_num += 1;
      if (jsr_num > STACK_SIZE -1){
	fprintf(stderr,"too many JSR's nested\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return how_long/DUE_PP_CLOCK;
      }
      cur_event = prog_shm->opinst[cur_event];
      if (cur_event > MAX_EVENTS || cur_event < 0){
	fprintf(stderr,"got a jsr to an event that doesn't exist.\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return how_long/DUE_PP_CLOCK;
      }
      break;
    case RTS:
      how_long += prog_shm->times[cur_event];
      if (jsr_num < 1){
	fprintf(stderr,"got a JSR, but nowhere to return to on stack!\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return how_long/DUE_PP_CLOCK;
      }
      cur_event = jsr_stack[jsr_num -1];
      jsr_num -=1;
      if (cur_event <0 || cur_event > MAX_EVENTS){
	fprintf(stderr,"got a RTS to a non-existant event!\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return how_long/DUE_PP_CLOCK;
      }
      break;      
    default:
      fprintf(stderr,"got an unknown opcode!\n"); // TODO: probably SUBSTART?
	prog_shm->event_error = 1;
      //      done = ERROR_DONE;
      return how_long/DUE_PP_CLOCK;
    }
  // capture EXIT:
  if (prog_shm->opcodes[cur_event]  == EXIT)
    how_long += prog_shm->times[cur_event];
    
    counter += 1;
    
}

  if (counter >= MAX_ALL_EVENTS){
    fprintf(stderr,"Got too many events (looping error? or forgotten EXIT?)\n");
	prog_shm->event_error = 1;
    //    done = ERROR_DONE;
    return how_long/DUE_PP_CLOCK;
  }


  //  fprintf(stderr,"partial_pp_time: how_long: %g\n",how_long);

  printf("partial_pp_time: %lf\n",how_long);
  return how_long/DUE_PP_CLOCK;
}
*/
/*
double partial_pp_time_new(int start,int end){
  // this routine duplicates pp_time in acq, almost exactly.  They should probably be merged together.
  // start and end are the first and last events to include in the count

#define STACK_SIZE 50
#define MAX_LOOP_LEVELS 50
#define MAX_ALL_EVENTS 100000000
  int cur_event=0;
  unsigned int jsr_stack[STACK_SIZE];
  unsigned int loop_level = 0;
  double loop_time[MAX_LOOP_LEVELS];
  unsigned int loops[MAX_LOOP_LEVELS];
  
  int jsr_num=0;
  int counter = 0;
  int i;
  
  loop_time[0] = 0.;
  cur_event = start;
  // look at time of first timer.
  while(prog_shm->opcodes[cur_event] != EXIT && counter < MAX_ALL_EVENTS){

    //    printf("event: %i, outputs: %i, opcode: %i duration: %f\n",cur_event,prog_shm->outputs[0][cur_event],prog_shm->opcodes[0][cur_event],prog_shm->times[cur_event]);
    switch (prog_shm->opcodes[cur_event] ){
    case CONTINUE:
    case SUBSTART:
      loop_time[loop_level] += prog_shm->times[cur_event];
      cur_event+=1;
      break;
    case WAIT:
      loop_time[loop_level] += prog_shm->times[cur_event];
      cur_event+=1;
      fprintf(stderr,"Got a WAIT, times may be off\n");
      break;
    case WAIT_MAX:
      loop_time[loop_level] += prog_shm->times[cur_event];
      cur_event+=1;
      fprintf(stderr,"Got a WAIT, times may be off\n");
      break;
    case LOOP:
      loop_level += 1;
      if (loop_level >= MAX_LOOP_LEVELS-1){ // max is one too big, since we use 0 for not looping.
	fprintf(stderr,"Got a start loop too many levels deep\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      loop_time[loop_level] = prog_shm->times[cur_event];
      loops[loop_level] = prog_shm->opinst[cur_event]; // load the loop counter
      cur_event+=1;
      break;
    case END_LOOP:
      if (loop_level <1){
	fprintf(stderr,"Got a loop end but not enough loop starts?\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
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
      //      cur_event,prog_shm->opinst[cur_event]);
      loop_time[loop_level] += prog_shm->times[cur_event];
      jsr_stack[jsr_num] = cur_event + 1;
      jsr_num += 1;
      if (jsr_num > STACK_SIZE -1){
	fprintf(stderr,"too many JSR's nested\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      cur_event = prog_shm->opinst[cur_event];
      if (cur_event > MAX_EVENTS || cur_event < 0){
	fprintf(stderr,"got a jsr to an event that doesn't exist.\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      break;
    case RTS:
      loop_time[loop_level] += prog_shm->times[cur_event];
      if (jsr_num < 1){
	fprintf(stderr,"got a JSR, but nowhere to return to on stack!\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      cur_event = jsr_stack[jsr_num -1];
      jsr_num -=1;
      if (cur_event <0 || cur_event > MAX_EVENTS){
	fprintf(stderr,"got a RTS to a non-existant event!\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      break;
    default:
      fprintf(stderr,"got an unknown opcode!\n");
	prog_shm->event_error = 1;
      //      done = ERROR_DONE;
      return loop_time[0]/DUE_PP_CLOCK;
    }
    
  }
  // capture EXIT:
  if (prog_shm->opcodes[cur_event]  == EXIT)
    loop_time[0] += prog_shm->times[cur_event];
    
    counter += 1;

  if (counter >= MAX_ALL_EVENTS){
    fprintf(stderr,"Got too many events (looping error? or forgotten STOP?)\n");
	prog_shm->event_error = 1;
    //    done = ERROR_DONE;
    return loop_time[0]/DUE_PP_CLOCK;
  }


  //  fprintf(stderr,"partial_pp_time: how_long: %g\n",how_long);
  // ok, in case we set the dsp_sync in the middle of a hardware loop
  for (i=1;i<=loop_level;i++)
    loop_time[0] += loop_time[i];
  printf("partial_pp_time_new: %lf\n",loop_time[0]);
  return loop_time[0]/DUE_PP_CLOCK;
}


*/



void set_tx_offset(int channel,double offset_freq){
  int chan=0;
 
  txprog_t *txprog;
  txprog = &prog_shm->txprog[prog_shm->txprogno];
  if (channel == 1){
    if (data_shm->ch1 == 'A')
      chan = 0;
    else
      chan = 1;
  }
  else if (channel == 2){
    if (data_shm->ch2 == 'A')
      chan = 0;
    else
      chan = 1;
  }
    
  if (txprog->txevents[chan] >= MAX_TX_EVENTS){
    prog_shm->event_error = 1;
    fprintf(stderr,"event: got a tx_event no out of range\n");
    return;
  }
  txprog->txopcodes[chan][txprog->txevents[chan]] = 32;
  txprog->txtimes[chan][txprog->txevents[chan]] = 0.;
  txprog->txparams[chan][txprog->txevents[chan]][0] = offset_freq*4294967296/TX_SAMPLE_RATE;// frequency tuning word
  txprog->txevents[chan] += 1;
}

 
// should have a flag to mark channel as inited?
void tx_init(int chan,double freq, double nco_freq,int gain){
  
  //  printf("tx_init, freq is: %f\n",freq);
  if (chan >= NUM_TX){
    printf("tx_init got channel: %i, but only have %i boards\n",chan,NUM_TX);
    return;
  }
  prog_shm->txfreqs[chan] = freq;
  prog_shm->txncofreqs[chan] = nco_freq;
  prog_shm->txgains[chan] = gain; 
}

void rx_init(int chan, double sw,int gain){
  if (chan >= NUM_TX){
    printf("rx_init got channel: %i, but only have %i boards\n",chan,NUM_TX);
    return;
  }
  //  printf("rx_init with sw: %f\n",sw);
  prog_shm->rx_sw[chan] = sw;
  prog_shm->rx_gains[chan] = gain;
}


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


void multiply_mat(float mat[3][3],float x,float y,float z,float *nx, float *ny, float *nz){

  *nx = x* mat[0][0]+y*mat[0][1]+z*mat[0][2];
  *ny = x* mat[1][0]+y*mat[1][1]+z*mat[1][2];
  *nz = x* mat[2][0]+y*mat[2][1]+z*mat[2][2];
  //  printf("mat: %f %f %f %f %f %f %f %f %f\n",mat[0][0],mat[0][1],mat[0][2],mat[1][0],mat[1][1],mat[1][2],mat[2][0],mat[2][1],mat[2][2]);
  //  printf("multiply got args: %f %f %f\n",x,y,z);
  //  printf("multiply returning: %f %f %f\n",*nx,*ny,*nz);
}

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
    //    printf("in set_gradients, doing va_args and calling do_event_duelime\n");
    va_start(args,num);
    do_event_duelime(1.5e-6,0,0,num,args); // opcode 0 opinst 0
    write_device(BGU_TRIG,1,prog_shm->no_events-1); // turn on BGU_TRIG bit in the previous event
    do_event_duelime(0.5e-6,0,0,num,args); // opcode 0 opinst 0
    write_device(BGU_NGI,0,prog_shm->no_events-1); // turn off NGI bit in the previous event
    va_end(args);
    //    printf("in set_gradients, done calling do_event_duelime\n");
  }
  else{ // no other devices requested:
    event_duelime(1.5e-6,0,0,1,BGU_TRIG,1);
    event_duelime(0.5e-6,0,0,1,BGU_NGI,0); // NGI is negative true - it idles high.
  }
  return 1;
}



/*
pulseblaster commands are:
CONTINUE
LOOP [# times]
END LOOP [where back to]
LONG EVENT [# times]
JSR [where]
RTS
STOP
WAIT

We also assign labels to commands to identify the wheres.


we're going to write pulse.c so synth events get put in the event before the current one.
We shouldn't put synth events in any event with a label, or any LOOP command (which should have a label).

We also don't want the previous event (that we drop into) to be and END LOOP, a LONG EVENT, a BRANCH,a JSR, or an RTS.  LOOP, CONTINUE, and WAIT are ok.

*/



void pulse_hardware_build_startup_lime(due_prog_t *due_prog,int num_boards){
  int i; 
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
    due_add_event(&due_prog[i],default_mask,.025 * DUE_PP_CLOCK);
    due_exit_program(&due_prog[i]);
    due_finalize_program(&due_prog[i]);
    //    due_dump_program(&due_prog[i]);
  }

}


