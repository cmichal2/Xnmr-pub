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
 #include "spinapi.h"

 #include "pulse-pb.h" 
 #include "/usr/share/Xnmr/config/h_config-pb.h" 
 #include "p_signals.h" 
 #include "shm_data.h" 
 #include "shm_prog-pb.h" 
 #include "param_utils.h" 

 #include <unistd.h> 
 #include <sys/time.h> 
 #include <time.h>

// these are used to figure out if phases of transmitters
// should be reveresed or not (reversed if above the IF freq).
// also the received signal needs to be reversed if above
// we assume that RF1 is always the received freq.
static double last_freqa=0.,last_freqb=0.;

#define PROG_TIME 90e-9

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

#define MAX_LABELS 32768
#define MAX_LABEL_LEN 20

static int old_outputs[NUM_BOARDS][MAX_EVENTS+1];
static int old_opcodes[NUM_BOARDS][MAX_EVENTS+1];
static int old_opinst[NUM_BOARDS][MAX_EVENTS+1];
static double old_times[NUM_BOARDS][MAX_EVENTS+1];
int old_no_events;

// for loops, subroutines and branching:
// the first three hold info about labels pointing to events
char event_labels[NUM_BOARDS][MAX_LABELS][MAX_LABEL_LEN];
int event_numbers[NUM_BOARDS][MAX_LABELS];
int num_event_labels[NUM_BOARDS];
//the next three hold info about events that need to be resolved (eg BRANCH or JSR routines would have these)
char labels_to_resolve[NUM_BOARDS][MAX_LABELS][MAX_LABEL_LEN];
int events_to_resolve[NUM_BOARDS][MAX_LABELS];
int num_events_to_resolve[NUM_BOARDS];

int sync_event_no=-1;

struct hardware_config_t *hardware_config; 
struct data_shm_t* data_shm; 
struct prog_shm_t* prog_shm;        //These are shared memory structures and must be initialized 
unsigned int latch_mask[NUM_BOARDS],default_mask[NUM_BOARDS];

struct itimerval mytime,old; 
int data_shm_id; 
int prog_shm_id; 
int msgq_id; 
int finished; 

int tran_table[4]; 
int add_a=-1,dat_a=-1,upd_a=-1,add_b=-1,dat_b=-1,upd_b=-1;
int wr_a=-1,wr_b=-1;





// a few prototypes
int shift_events(int start);
int resolve_labels();


 /* 
  *   Method Implementations 
   */





 /* 
  *  This method ensures that the pulse program exits normally even when sent a SIGTERM or SIGINT signal. 
  *  The finished flag indicates that the pulse program should stop looping and begin 
  *  it's exit sequence 
   */

void stop() { 
  struct msgbuf message; 
  
  finished = 1; 
  
  /* 
   * this will make sure the program does not get stuck waiting for a message of 
   * of type P_PROGRAM_CALC.  A race condition can arise if a SIGTERM or SIGINT is recieved  
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



int ready( char phase ) { 
  int result,i,j; 
  struct msgbuf message; 
  static int first_time = 1;
  int err=0;
  double last_freq=0.;

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

  // insert events at end to ensure outputs go to defaults
  // and the program stops.
  event_pb(PROG_TIME,0,0,0); 
  event_pb(PROG_TIME,STOP,0,0);

  /* if the pulse program requested a dsp sync, figure it out. */
  if (sync_event_no > 0) do_insert_dsp_sync(sync_event_no);

  // deal with clean/dirty stuff;
  // mark all dirty
  for (i=0;i<NUM_BOARDS;i++)
    prog_shm->board_clean[i]=DIRTY;
  
  
  // if the number of events is different, then program is different.
  // also don't bother to compare if the program hasn't been downloaded, because it will need to be anyway.
  if (old_no_events == prog_shm->no_events && prog_shm->downloaded == 1){ 
    
    for( i = 0; i < NUM_BOARDS ; i++ ){		
      for(j=0;j< prog_shm->no_events; j++ ){
	if ((prog_shm->outputs[i][j] != old_outputs[i][j]) || (prog_shm->times[i][j] != old_times[i][j]) ||
	    (prog_shm->opcodes[i][j] != old_opcodes[i][j]) || (prog_shm->opinst[i][j] != old_opinst[i][j])){
	  j = prog_shm->no_events+10;	      // break out of loop.
	  // fprintf(stderr,"ready: board %i is dirty\n",i);
	}
      }
	    
      if ( j == prog_shm->no_events){
	prog_shm->board_clean[i] = CLEAN;
	//	  fprintf(stderr,"ready: marking chip: %i as clean\n",i);
      }
    }// end NUM_BOARDS
  } // end needed to compare one by one.
    


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
	fprintf(fid,"%3i %i %2i %.9f ",event,prog_shm->opcodes[board][event],prog_shm->opinst[board][event],prog_shm->times[board][event]);
	for( bit=0; bit<24; bit++ ) { 
	  fprintf( fid, "%d", (prog_shm->outputs[ board ][ event ] >> bit) %2 ); 
	} 
	fprintf( fid, " " ); 
      }
      fprintf( fid, "\n" );       

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
   

   if( finished == 1 ) {
     return P_PROGRAM_END; 
   }

   //   fprintf(stderr, "Pulse Program calculation complete\n" ); 

   if (data_shm->ch1 == 'A')
     last_freq = last_freqa;
   else if (data_shm->ch1 == 'B')
     last_freq = last_freqb;
   else fprintf(stderr,"ch1 is neither A nor B! Don't know if we should reverse receiver phase or not\n"); 

   if (last_freq > prog_shm->if_freq)
     prog_shm->above_freq = 1;
   else prog_shm->above_freq = 0;
     //     phase = (4-phase)%4; // reverse the frequencies.

   prog_shm->phase = phase;  //let acq know what phase shift to apply to data 

   //   fprintf(stderr, "pprog sending message P_PROGRAM_READY\n" ); 
  if (err == 0 && prog_shm->event_error == 0){
    //    printf("pulse.c: telling acq program is ready\n");
    message.mtype = P_PROGRAM_READY; 
    message.mtext[0] = P_PROGRAM_READY;
  }
  else{
    //    printf("pulse.c: telling acq we have an error\n");
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
   if (add_a == -1 ){
     fprintf(stderr,"write_device_wrap: pulse_program not init'ed\n");
     prog_shm->event_error = 1;
     return -1;
   }

   
   if (device_id >= RF_OFFSET){ 
     //       fprintf(stderr,"translating device: %i ",device_id);
     device_id = tran_table[device_id-RF_OFFSET];
     //       fprintf(stderr,"to device: %i\n",device_id);
   }
 
   if ( device_id == PP_INTERRUPT ){
     prog_shm->got_ppo += 1; // to ensure there is a ppo 
     //     printf("got ppo\n");
   }
   //   printf("going to write device: %i with value: %i\n",device_id,intval);
   for (i=start_event_no;i<=end_event_no;i++){
     
     write_device( device_id, intval,i); 
     
   }
 
   return 0;
}

int event_pb( double time, int opcode,int opinst,unsigned char num, ... ) 

 { 
   va_list args; 
   int i,j,intval=0; 
   unsigned char device_id; 
   int syntha=0,synthb=0;
   double dval;
   int num_split,synth_event_start=-1;
   int first_synth;
   int has_label = 0;
   int old_opcode;

   //   fprintf(stderr,"\ncoming into event, number is: %i\n",prog_shm->no_events);
   if (prog_shm->begun == 0 ){
     fprintf(stderr,"problem in pulse program.  Got an event before begin()\n");
     prog_shm->event_error = 1;
     return -1;
   }

   if( prog_shm->no_events >= MAX_EVENTS ) { 
     fprintf(stderr, "pprog: Maximum number of events exceeded\n" ); 
     prog_shm->event_error = 1;
     return -1; 
   } 

   //   fprintf(stderr,"event_no: %i time: %lf \n",prog_shm->no_events,time);


   // shortest event is 90ns, longest (without being a long event is 2^32-4 (/2 ?)* 10ns = 42.949 s (/2 ?)

   if (!isnormal(time)){
     if (time != 0.){
       fprintf(stderr,"TIME IS NOT A NUMBER!\n");
       prog_shm->event_error = 1;
       return -1;
     }
   }
   if (time < 0){
     fprintf(stderr,"event: time < 0, ignored\n");
     fprintf(stderr,"time was: %lg\n",time);
     return 0; 
   }
   
   if (time < 90e-9){
     fprintf(stderr,"event time >0 but less than 90ns, increased to 90ns\n");
     time = 90e-9;
   }

   // here - check to see if we have special synthesizer setting events.
   // make sure we have enough time, if not, increase time to be long enough.
   // also we'll copy all events into enough events below, and have to worry about sorting multiple
   // synth events properly.  The update comes at the end.

   // do a first pass
   va_start(args,num);


   for(i=0;i<num;i++){
     device_id = (unsigned char) va_arg(args,int);
     if (device_id == LABEL){// just a label - this one won't be copied to all the events...
       char *lab;
       has_label = 1;
       lab = (char *) va_arg(args,char *);
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
     else {// don't care for now...
       intval = va_arg(args,unsigned int);
     }
   }

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

   // this is only legal if the previous event is not a STOP or LONG_DELAY
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
       if(prog_shm->opcodes[i][prog_shm->no_events-1] == END_LOOP || prog_shm->opcodes[i][prog_shm->no_events-1] == JSR ||
	   prog_shm->opcodes[i][prog_shm->no_events-1] == RTS || prog_shm->opcodes[i][prog_shm->no_events-1] == BRANCH ){
	 fprintf(stderr,"event: %i requests a synth setting, but has an END_LOOP, BRANCH, RTS or JSR previous.  Allowed, but may not do what you want\n",prog_shm->no_events);
       }
     }
     for (i=0;i<NUM_BOARDS;i++){
       if( prog_shm->opcodes[i][prog_shm->no_events-1] == STOP || prog_shm->opcodes[i][prog_shm->no_events-1] == LONG_DELAY){
	 fprintf(stderr,"event: %i requests a synth setting, but has STOP or LONG_DELAY previous.  Not allowed\n",prog_shm->no_events);
	 prog_shm->event_error = 1;
	 return -1;
       }
     }

     // so we should be legal to stick our synth event in.  Check timing of previous event:
     
     for(i=0;i<NUM_BOARDS;i++){
       if (PROG_TIME*(num_split+1) > prog_shm->times[i][prog_shm->no_events-1]){
	 fprintf(stderr,"Time of %g too short for setting amps/phases/freqs, increasing to: %g\n",
		 prog_shm->times[i][prog_shm->no_events-1],PROG_TIME*(num_split+1));
	 prog_shm->times[i][prog_shm->no_events-1]=PROG_TIME*(num_split+1);
       }
     }
     
     //make sure we have enough events left:
     if (prog_shm->no_events + num_split >= MAX_EVENTS-1){
       prog_shm->event_error = 1;
       fprintf(stderr,"Ran out of events!\n");
       return -1;
     }


     // if our event has a label to resolve, move the pointer num_split events forward.
     for (i=0;i<NUM_BOARDS;i++){
       if(events_to_resolve[i][num_events_to_resolve[i]-1] == prog_shm->no_events)
	 events_to_resolve[i][num_events_to_resolve[i]-1] += num_split;
     }

     
     // now duplicate the previous event num_split times.
     for(i=0;i<NUM_BOARDS;i++){
       for (j=0;j<num_split;j++){
	 prog_shm->outputs[i][prog_shm->no_events+j] = prog_shm->outputs[i][prog_shm->no_events-1];
	 prog_shm->times[i][prog_shm->no_events+j] = PROG_TIME;
	 prog_shm->opcodes[i][prog_shm->no_events+j] = CONTINUE;
	 prog_shm->opinst[i][prog_shm->no_events+j] = 0;
       } 
       prog_shm->times[i][prog_shm->no_events-1] -= PROG_TIME*num_split;
     }
     // in here need to make sure that the num_split new events have UPD set to 0 
     // this actually is probably not necessary... since the previous event  shouldn't have an UPD in it...
     write_device_wrap(prog_shm->no_events,prog_shm->no_events+num_split-1,UPD_A,0);
     write_device_wrap(prog_shm->no_events,prog_shm->no_events+num_split-1,UPD_B,0);
     synth_event_start = prog_shm->no_events - 1;


     // if the previous was BRANCH, JSR, RTS or END_LOOP, move the opcode and opinst to the last event, as well
     // as the label to resolve flag for it.
     for (i=0;i<NUM_BOARDS;i++){
       old_opcode = prog_shm->opcodes[i][prog_shm->no_events-1];
       if(old_opcode == END_LOOP || old_opcode == BRANCH || old_opcode == JSR || old_opcode == RTS){
	 prog_shm->opcodes[i][prog_shm->no_events+num_split-1] = prog_shm->opcodes[i][prog_shm->no_events-1];
	 prog_shm->opinst[i][prog_shm->no_events+num_split-1] = prog_shm->opinst[i][prog_shm->no_events-1];

	 prog_shm->opcodes[i][prog_shm->no_events-1] = CONTINUE;
	 prog_shm->opinst[i][prog_shm->no_events-1] = 0;

	 // if that previous event had a label to resolve (and is BRANCH-like opcode), move the pointer forward.
	 for (j=0;j<num_events_to_resolve[i];j++)
	   if(events_to_resolve[i][j] == prog_shm->no_events-1)
	     events_to_resolve[i][j] += num_split;
       }

     }
       
     
     prog_shm->no_events += num_split; // current event is now moved forward

   }
   va_end(args);
   va_start( args, num ); 

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



   //set all the specified device information 
   first_synth = 1;
   //   fprintf(stderr,"\nin event, no_events: %i,num things this event: %i\n",prog_shm->no_events,num);
   for( i=0; i<num; i++ ) { 
     device_id = (unsigned char) va_arg( args, int  ); 
     if (device_id == LABEL){ // its a pseudo device with just a label.
       char *lab;
       lab = va_arg(args,char *);
       //       printf("got a label: %s at instruction: %i\n",lab,prog_shm->no_events);
       for (j=0;j<NUM_BOARDS;j++){
	 strncpy(event_labels[j][num_event_labels[j]],lab,MAX_LABEL_LEN);
	 event_numbers[j][num_event_labels[j]] = prog_shm->no_events;
	 num_event_labels[j] += 1;
       }       
     }
     else if (device_id == FREQA || device_id == FREQB || device_id == AMP1 || device_id == AMP2 || device_id == PHASE1 || device_id == PHASE2) {
       dval= (double) va_arg(args,double);
       //       printf("event, calling insert_synth_event with val: %f\n",dval);
       insert_synth_event(device_id,dval,num_split,first_synth,synth_event_start);
       if (first_synth == 1) first_synth = 0;
     }
     else{

       //       fprintf(stderr,"got device_id: %i for event: %i\n",(int) device_id,prog_shm->no_events);
       intval =  va_arg(args,unsigned int);
       write_device_wrap(prog_shm->no_events,prog_shm->no_events,device_id,intval);
     }
   }
 
   // straightforward time setting.
   if (time < MAX_TIME){
     for (i=0;i<NUM_BOARDS;i++)
       prog_shm->times[i][prog_shm->no_events] = time;
   }   
   else    if (opcode == 0 ){ // can't do a long event unless opcode comes in as 0
   // if this event is longer than the pulse prog timer can hold, split it up:
   // if we did a synth event, time came back as the time of the final event.
     printf("got a long event\n");
     if (time < 2* MAX_TIME){
       for (i=0;i<NUM_BOARDS;i++){
	 prog_shm->times[i][prog_shm->no_events]=time-MAX_TIME; 
	 prog_shm->opcodes[i][prog_shm->no_events] = 0;
	 prog_shm->opinst[i][prog_shm->no_events] = 0;
       }
       prog_shm->no_events += 1;
       if (prog_shm->no_events >= MAX_EVENTS - 1) prog_shm->event_error = 1;
       for (i=0;i<NUM_BOARDS;i++){
	 prog_shm->outputs[i][prog_shm->no_events] = prog_shm->outputs[i][prog_shm->no_events-1];
	 prog_shm->times[i][prog_shm->no_events] = MAX_TIME;
	 prog_shm->opcodes[i][prog_shm->no_events] = 0;
	 prog_shm->opinst[i][prog_shm->no_events] = 0;
       }
     }
     else{
       int num_loops;
       num_loops = floor(time/MAX_TIME);
       for (i=0;i<NUM_BOARDS;i++){
	 prog_shm->times[i][prog_shm->no_events] = time-num_loops*MAX_TIME;
	 prog_shm->opcodes[i][prog_shm->no_events] = 0;
	 prog_shm->opinst[i][prog_shm->no_events] = 0;
       }
       prog_shm->no_events += 1;
       if (prog_shm->no_events >= MAX_EVENTS - 1) prog_shm->event_error = 1;
       for (i=0;i<NUM_BOARDS;i++){
	 prog_shm->outputs[i][prog_shm->no_events] = prog_shm->outputs[i][prog_shm->no_events-1];
	 prog_shm->times[i][prog_shm->no_events] = MAX_TIME;
	 opcode = LONG_DELAY;
	 opinst = num_loops;
       }
     }
   }
   else{ // got a long time, but opcode not 0.
     prog_shm->event_error = 1;
     printf("Error, got a time greater than max allowed with an opcode of: %i, time set to 21.4s\n",opcode);
     va_end(args); 
     return -1;
   }    


   // set the opcode and opinst -- this isn't needed if we split for long_delay
   // but we may have split for a synth event
   for (i=0;i<NUM_BOARDS;i++){
     prog_shm->opcodes[i][prog_shm->no_events] = opcode;
     prog_shm->opinst[i][prog_shm->no_events] = opinst; // most of these are wrong now, but will get 
       // updated later by resolve
   }
   if (opcode == LOOP && opinst == 0){
     printf("Got a LOOP, but argument is 0!\n");
     prog_shm->event_error = 1;
     va_end(args); 
     return -1;
   }

   prog_shm->no_events += 1; 
   va_end(args); 

   return 0; 
 }


void get_freq_words(double freq,unsigned int *aval,unsigned int *aval2){


  //  double clkr = 60000000.;
  double clkt = 300000000.;

  unsigned long long ncot;
    

  if (freq > prog_shm->if_freq) freq = freq - prog_shm->if_freq;
  else freq = prog_shm->if_freq - freq;

  if (freq > 130000000.){
    printf("asked for synth to produce a frequency > 130 MHz\n");
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
  static int syntha_event_count=0,synthb_event_count=0;
  unsigned int aval,aval2;
  int aaval;
  double dval2;
  //  double fval;
  if (first_synth){
    syntha_event_count = 0;
    synthb_event_count = 0;
  }
  
  if (ev_no == -1){
    printf("got -1 as event to insert synth event at!!\n");
    prog_shm->event_error = 1;
    return;
  }

  if (device_id == FREQA) {
    last_freqa = dval;
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
  else if (device_id == FREQB){
    last_freqb = dval;
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
  else if ((device_id == PHASE1 && data_shm->ch1 == 'A') || (device_id == PHASE2 && data_shm->ch2 == 'A' )){
    if (last_freqa < prog_shm->if_freq) dval = 360 - dval; // need to reverse phases if we're above the IF.
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
  else if ((device_id == PHASE1 && data_shm->ch1 == 'B')||(device_id == PHASE2 && data_shm->ch2 == 'B')){
    if (last_freqb < prog_shm->if_freq) dval = 360 - dval; // need to reverse phases if we're above the IF.
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
  else {
    fprintf(stderr,"in insert synth event, but couldn't match device\n");
    prog_shm->event_error=1;
    return;
  }



  return;

}

int begin() {

  int i,j;
  
  // copy old program to save place for later comparison.
  
  sync_event_no = -1;


  old_no_events = prog_shm->no_events;
  for (i=0;i<NUM_BOARDS;i++)
    for( j = 0 ; j < prog_shm->no_events;j++){
      old_outputs[i][j]=prog_shm->outputs[i][j];
      old_opcodes[i][j]=prog_shm->opcodes[i][j];
      old_opinst[i][j]=prog_shm->opinst[i][j];
      old_times[i][j]=prog_shm->times[i][j];
    }
  
  
  
  prog_shm->no_events = 0; 
  prog_shm->event_error = 0;
  prog_shm->got_ppo = 0;
  prog_shm->is_noisy = 0;
  
  prog_shm->begun = 1;

  prog_shm->receiver_model = REC_ACCUM; // old accumulate and download by default.

  /* set up to catch infinite loops */
  mytime.it_interval.tv_sec = 0; 
  mytime.it_interval.tv_usec = 0; 
  mytime.it_value.tv_sec = 3; 
  mytime.it_value.tv_usec = 0; 
  signal( SIGALRM, pprog_internal_timeout);  
  setitimer( ITIMER_REAL, &mytime, &old ); 
  
  for (i=0;i<NUM_BOARDS;i++){
    num_events_to_resolve[i]=0;
    num_event_labels[i]=0;
  }
  // insert two events at start to synchronize multiple boards
  if (NUM_BOARDS > 1){
    event_pb(2e-6,0,0,1,SLAVEDRIVER,1);
    event_pb(2e-6,0,0,1,SLAVEDRIVER,0);
    
    for(i=1;i<NUM_BOARDS;i++){
      prog_shm->opcodes[i][prog_shm->no_events-1] = WAIT;
      prog_shm->times[i][prog_shm->no_events-1]=1e-6; // 1 us
    }
    // PB manual says latency is extra 6 clock cycles, seems like its 11 though...
    //    prog_shm->times[0][prog_shm->no_events-1]=1e-6+6*10e-9; 
    prog_shm->times[0][prog_shm->no_events-1]=1e-6+110e-9; 
    
  }
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
   memset( old_outputs, 0, MAX_EVENTS*NUM_BOARDS*sizeof(int) );

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

   fid = fopen( "/usr/share/Xnmr/config/h_config-pb.h", "r" ); 

   if (fid == NULL) {
     fprintf(stderr,"pulse.c: couldn't open h_config-pb.h\n");
     exit(0);
   }

   // look for how many devices
   do { 
     eo = fgets( s, PATH_LENGTH, fid );  
   } while( strstr( s, "NUM_DEVICES" ) == NULL || eo == NULL ); 

   if (eo == NULL){
     fprintf(stderr,"pulse.c: didn't find the number of device in h_config-pb.h\n");
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

   int err,i;
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

   err = stat("/usr/share/Xnmr/config/h_config-pb.h",&other_buff);
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

    if (add_a == -1){
      //   fprintf(stderr,"in write_device() for the first time\n");
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"ADD_A") == 0)
	  add_a = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"DAT_A") == 0)
	  dat_a = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"WR_A") == 0)
	  wr_a = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"UPD_A") == 0)
	  upd_a = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"ADD_B") == 0)
	  add_b = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"DAT_B") == 0)
	  dat_b = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"WR_B") == 0)
	  wr_b = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"UPD_B") == 0)
	  upd_b = i;
      }
      
      if ( (add_a == -1) || (dat_a == -1) || (wr_a == -1)||(upd_a == -1) ){
	fprintf(stderr,"pulse_program_init: couldn't find one of my channel A devices, check h_config-pb.h\n");
	exit(0);
      }
      if ( (add_b == -1) || (dat_b == -1) || (wr_b == -1)||(upd_b == -1)){
	fprintf(stderr,"pulse_program_init: couldn't find one of my channel B devices, check h_config-pb.h\n");
	exit(0);
      }
      
      // now build a table to translate the rf channel devices from logical channels to real devices
      //   fprintf(stderr,"building a tran_table\n");
      if (data_shm->ch1 == 'A'){
	//     fprintf(stderr,"pulse.c found ch1 = A\n");
	tran_table[RF1-RF_OFFSET]=GATE_A;
	tran_table[BLNK1-RF_OFFSET]=BLNK_A;
      }
      else if (data_shm->ch1 == 'B'){
	//     fprintf(stderr,"pulse.c found ch1 = B\n");
	tran_table[RF1-RF_OFFSET]=GATE_B;
	tran_table[BLNK1-RF_OFFSET]=BLNK_B;
      }
      else {
	fprintf(stderr,"no valid channel 1 channel found!\n");
	exit(0);
      }
      if (data_shm->ch2 == 'A'){
	//     fprintf(stderr,"pulse.c found ch2 = A\n");
	tran_table[RF2-RF_OFFSET]=GATE_A;
	tran_table[BLNK2-RF_OFFSET]=BLNK_A;
      }
      else if (data_shm->ch2 == 'B'){
	//     fprintf(stderr,"pulse.c found ch2 = B\n");
	tran_table[RF2-RF_OFFSET]=GATE_B;
	tran_table[BLNK2-RF_OFFSET]=BLNK_B;
      }
      else {
	fprintf(stderr,"no valid channel 2 channel found!\n");
	exit(0);
      }
      //   fprintf(stderr,"event, got device numbers: %i %i %i\n",clk3,phase3,i3);
    } // that was the setup we do first time only.
    
    

  finished = 0;
  return 0;
}

float get_dwell()
{
  long int decimate;
  // figure out what was really meant as the dwell in here.
  // find the nearest value that is an integer decimation from the receiver clk rate.
  decimate = (long int) (data_shm->dwell/1000000 *DEFAULT_RCVR_CLK +0.5);
  //  printf("in get_dwell, had dwell of: %f, nearest was: %f\n",data_shm->dwell,1./DEFAULT_RCVR_CLK * decimate*1e6);
  return (1./DEFAULT_RCVR_CLK * decimate);
    //  return data_shm->dwell/1000000.;
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





int shift_events(int start){
  int i,j;
  //  unsigned char board,bit;
  
  // this needs to also shift labels, opcodes, opinsts, and times (as well as outputs)
  
  if (prog_shm->no_events >= MAX_EVENTS-2){
    fprintf(stderr,"shift_events: MAX number of events exceeded\n");
    prog_shm->event_error = 1;
    return -1;
  }
  
  // used to start at no-events-1, but no_events might be in progress...
  for (j=0;j<NUM_BOARDS;j++){ // shift all the events
    for (i = prog_shm->no_events; i >= start ; i--){
      prog_shm->outputs[j][i+1] = prog_shm->outputs[j][i];
      prog_shm->opcodes[j][i+1] = prog_shm->opcodes[j][i];
      prog_shm->opinst[j][i+1] = prog_shm->opinst[j][i];
      prog_shm->times[j][i+1] = prog_shm->times[j][i];
    }
    for (i = 0;i<num_event_labels[j];i++)  // move all the label related stuff too
      if (event_numbers[j][i] >= start)
	event_numbers[j][i] += 1;
    for (i = 0; i< num_events_to_resolve[j];i++)
      if (events_to_resolve[j][i] >= start)
	events_to_resolve[j][i] += 1;
  }
  
  // go through opinsts and move arguments for END_LOOP, JSR and BRANCH in case we're called after labels are already resolved
  for (j=0;j<NUM_BOARDS;j++){ 
    for (i = start+1; i <= prog_shm->no_events ; i++){
      if (prog_shm->opcodes[j][i]  == END_LOOP || prog_shm->opcodes[j][i] == JSR || prog_shm->opcodes[j][i] == BRANCH)
	if (prog_shm->opinst[j][i] >= start) prog_shm->opinst[j][i] += 1;

    }
  }

  prog_shm->no_events++;
  
  return 0;
}





void pprog_is_noisy(){
  
  prog_shm->is_noisy = 1;
  prog_shm->noisy_start_pos=0;
  prog_shm->receiver_model = 0;
  //  fprintf(stderr,"just set is_noisy to true\n");
}


// obsolete?
void start_noisy_loop(){
  prog_shm->noisy_start_pos = prog_shm->no_events;
  //  fprintf(stderr,"just set noisy_start_pos to %i\n",prog_shm->no_events);

}



void label_to_resolve(char * label){

  int i;
  for(i=0;i<NUM_BOARDS;i++){
    strncpy(labels_to_resolve[i][num_events_to_resolve[i]],label,MAX_LABEL_LEN);
    events_to_resolve[i][num_events_to_resolve[i]]=prog_shm->no_events;
    num_events_to_resolve[i] += 1;
    if (num_events_to_resolve[i] >= MAX_LABELS){
      printf("ERROR, overrun labels - need to increase MAX_LABELS\n");
      prog_shm->event_error = 1;
    }
  }

}

int resolve_labels(){
  int i,j,k;
  int rval = TRUE;
  for (i=0;i<NUM_BOARDS;i++){
    for (j=0;j<num_events_to_resolve[i];j++){
      for (k=0;k<num_event_labels[i];k++){
	if (strncmp(labels_to_resolve[i][j],event_labels[i][k],MAX_LABEL_LEN) == 0){
	  //	  printf("found label: %s at instruction: %i\n",labels_to_resolve[i][j],event_numbers[i][k]);
	  // ok so we found the label, now do the right thing:
	  prog_shm->opinst[i][events_to_resolve[i][j]] = event_numbers[i][k];
	  k = num_event_labels[i] + 10;			      
	}
      }
      if (k != num_event_labels[i] + 11){
	// screwed up, didn't resolve
	printf("failed to resolve label: %s for board: %i\n",labels_to_resolve[i][j],i);
	rval = FALSE;
      }
    }
  }
  return rval;
}




void setup_synths(double freqa,double freqb){
  // will set up the synths at start - need to do full writes of all synth registers...
  // set rcvr_chan to RF1 or RF2 for obs channel.


  if (prog_shm->do_synth_setup == 1){
    // do full setup
    printf("doing full synth setup\n");
    //    prog_shm->do_synth_setup = 0;
    // the most interesting of all, the control registers
    // single tone mode:
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x1F,DAT_A,0,ADD_B,0x1F,DAT_B,0,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x1F,DAT_A,0,ADD_B,0x1F,DAT_B,0,WR_A,0,WR_B,0);
    event_pb( 100e-6,0,0,0);
    event_pb( PROG_TIME,0,0,2,UPD_A,1,UPD_B,1);

    // address 1D is power down some things:

    // top 8 bits: top 3 unused, next: 1 means power down comparator
    // next must always be 0
    // next 1 means power down q dac
    // full dac power-down
    // then digital power down.
    // we will power down the q dac's and comparators for now.
    //    event_pb( PROG_TIME, 0,0,6,ADD_A,0x1D,DAT_A,0x14,ADD_B,0x1D,DAT_B,0x14,WR_A,1,WR_B,1);
    // nope, leave qdacs on.
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x1D,DAT_A,0x10,ADD_B,0x1D,DAT_B,0x10,WR_A,1,WR_B,1);
    // next 8:
    // first is always 0, second sets range of PLL want 1 for > 200MHz
    // next is PLL bypass (if high), then 5 bits for PLL multiplier value.
    // we want a multiplier of 5 and PLL high range.
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x1E,DAT_A,5+64,ADD_B,0x1E,DAT_B,5+64,WR_A,0,WR_B,0);

    // next, top bit is clear accumulator bit (one-shot), then  clear both accumulators
    // the triangle mode bit for up and down freq sweeps, then Q dac source.  when high,
    // get Q DAC output from the Q DAC register
    // then three bits that set the overall mode: =0 for single-tone,
    // = 1 for FSK, 2 for ramped FSK, 3 for chirp 4 for BPSK
    // last is the internal update active. 0 makes I/O UPD an output.
    // turn on SRC bit.
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x1F,DAT_A,0x10,ADD_B,0x1F,DAT_B,0x10,WR_A,1,WR_B,1);

    // last byte, top bit is always 0, next when high, bypass the inverse sinc filter - big power savings
    // next is shaped keying enable (when high)
    // then internal/external shaped keying control.
    //then next two are always 0, then serial port endian
    // and finally SDO active for serial port.
    // we set 0x40 to disable the inverse sinc filter
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x20,DAT_A,0x60,ADD_B,0x20,DAT_B,0x60,WR_A,0,WR_B,0);


    //    /*
    //phase registers
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x00,DAT_A,0,ADD_B,0x00,DAT_B,0,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x01,DAT_A,0,ADD_B,0x01,DAT_B,0,WR_A,0,WR_B,0);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x02,DAT_A,0,ADD_B,0x02,DAT_B,0,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x03,DAT_A,0,ADD_B,0x03,DAT_B,0,WR_A,0,WR_B,0);

    // frequency tuning word 1 gets done later
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x0A,DAT_A,0,ADD_B,0x0A,DAT_B,0,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x0B,DAT_A,0,ADD_B,0x0B,DAT_B,0,WR_A,0,WR_B,0);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x0C,DAT_A,0,ADD_B,0x0C,DAT_B,0,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x0D,DAT_A,0,ADD_B,0x0D,DAT_B,0,WR_A,0,WR_B,0);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x0E,DAT_A,0,ADD_B,0x0E,DAT_B,0,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x0F,DAT_A,0,ADD_B,0x0F,DAT_B,0,WR_A,0,WR_B,0);
    // delta freq:
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x10,DAT_A,0,ADD_B,0x10,DAT_B,0,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x11,DAT_A,0,ADD_B,0x11,DAT_B,0,WR_A,0,WR_B,0);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x12,DAT_A,0,ADD_B,0x12,DAT_B,0,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x13,DAT_A,0,ADD_B,0x13,DAT_B,0,WR_A,0,WR_B,0);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x14,DAT_A,0,ADD_B,0x14,DAT_B,0,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x15,DAT_A,0,ADD_B,0x15,DAT_B,0,WR_A,0,WR_B,0);
    // update clock - the default it 0x40 - keep it.
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x16,DAT_A,0x40,ADD_B,0x16,DAT_B,0x40,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x17,DAT_A,0,ADD_B,0x17,DAT_B,0,WR_A,0,WR_B,0);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x18,DAT_A,0,ADD_B,0x18,DAT_B,0,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x19,DAT_A,0x40,ADD_B,0x19,DAT_B,0x40,WR_A,0,WR_B,0);

    event_pb( PROG_TIME, 0,0,6,ADD_A,0x1A,DAT_A,0,ADD_B,0x1A,DAT_B,0,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x1B,DAT_A,0,ADD_B,0x1B,DAT_B,0,WR_A,0,WR_B,0);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x1C,DAT_A,0,ADD_B,0x1C,DAT_B,0,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x25,DAT_A,0,ADD_B,0x25,DAT_B,0,WR_A,0,WR_B,0);


    // amplitudes:
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x21,DAT_A,0,ADD_B,0x21,DAT_B,0,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x22,DAT_A,0,ADD_B,0x22,DAT_B,0,WR_A,0,WR_B,0);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x23,DAT_A,0,ADD_B,0x23,DAT_B,0,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x24,DAT_A,0,ADD_B,0x24,DAT_B,0,WR_A,0,WR_B,0);

    // qdac values:
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x26,DAT_A,8,ADD_B,0x26,DAT_B,8,WR_A,1,WR_B,1);
    event_pb( PROG_TIME, 0,0,6,ADD_A,0x27,DAT_A,0,ADD_B,0x27,DAT_B,0,WR_A,0,WR_B,0);
//    */


  }
  else
    printf("only setting frequencies in synth setup\n");

  // otherwise just set the frequencies.


  //    printf("about to init freqs\n");
  event_pb(7*PROG_TIME,0,0,0); // the frequency setting actually goes in here
  event_pb( 9*PROG_TIME, 0,0,2, FREQA,freqa,FREQB,freqb ); // this provides a spot for some first synth events set by program...

  if (prog_shm->do_synth_setup == 1){
    // do full setup
    printf("doing full synth setup\n");
    // it takes a while for all this to sink in the first time.  Wait for it
    // 200us is not long enough!
    // Perhaps this is time for PLL in the synthesizer to lock?
    event_pb (1000e-6,0,0,0);
    prog_shm->do_synth_setup = 0;
  }
  prog_shm->lfreq1 = freqa;
  prog_shm->lfreq2 = freqb;

  //  printf("SETUP_SYNTHS, set freqs to: %g %g\n",freqa,freqb);


  return;
}

void insert_dsp_sync(){
  /* this is a routine the pulse program can call to indicate that we want to 
     hit the dsp sync button for an acquisition to start at this point.
   
     Here we just store the event number, ready actually does the real
     work after all the event labels have been resolved.*/
  
  sync_event_no = prog_shm->no_events;
}

void do_insert_dsp_sync(int ev_no){
  /* this is a little tricky on the pulse blaster version, but should work
     Here's the problem, we can't simply go back a fixed time from the start
     of the acquisition to hit the sync buttons on the dsp because that
     fixed time might be in the middle of a loop, and then we'd hit the sync
     button over and over again.  Instead, what we do is figure out how long 
     the pulse program is so far, and put in a little extra time at the 
     beginning (at most 256 dwells, and do the sync before.  With two pb boards,
     the first two events are in there to sync the boards together, the next 
     two events are to sync the dsp. 

     ev_no is the event number up to which we include in our count (not including ev_no
  */

  // first figure out how the length of the program so far
  double how_long,min_time,diff_time;
  float dwell;
  int start,end,j;
  if (NUM_BOARDS > 1) start = 2;
  else start = 0;
  
  //  end = prog_shm->no_events-1;
  // June 29, CM:
  //  end = ev_no-1;
  end = ev_no;

  //  how_long = partial_pp_time(start,end);
  how_long = partial_pp_time_new(start,end);

  dwell = get_dwell();
  min_time = dwell * 256;
  
  // Nov 16, 2011  CM added the -how_long below!
  if (how_long > min_time)
    diff_time = ceil(how_long/dwell)*dwell-how_long;
  else
    diff_time = min_time - how_long;
    
  if (diff_time < PROG_TIME) diff_time += dwell;


  // now we need to shift everything from start down by 2 events.
  
  shift_events(start);
  shift_events(start);

  // our events should be init default:
  for(j=0;j<NUM_BOARDS; j++){
    prog_shm->outputs[j][start] = default_mask[j];
    prog_shm->outputs[j][start] = default_mask[j];
    prog_shm->outputs[j][start+1] = default_mask[j];
    prog_shm->outputs[j][start+1] = default_mask[j];
  }
  
  // make sure opcodes and opinsts are all boring:
  for (j=0;j<NUM_BOARDS;j++){
    prog_shm->opcodes[j][start] = CONTINUE;
    prog_shm->opinst[j][start] = 0;
    prog_shm->opcodes[j][start+1] = CONTINUE;
    prog_shm->opinst[j][start+1] = 0;

    prog_shm->times[j][start] = PROG_TIME; // the sync_cic will go in start
    prog_shm->times[j][start+1] = diff_time;
  }
  
  write_device(SYNC_CIC,1,start);

  // done.  do some checking:
  how_long = partial_pp_time_new(start+1,end+2);
  printf("added in extra: %g + %g, total prog time before acq is now: %g\n",PROG_TIME,diff_time,how_long);

}



double partial_pp_time(int start,int end){
  // this routine duplicates pp_time in acq, almost exactly.  They should probably be merged together.
  // start and end are the first and last events to include in the count

#define STACK_SIZE 8
#define MAX_ALL_EVENTS 100000000
  int cur_event=0,cur_event_new;
  unsigned int jsr_stack[STACK_SIZE],local_opinst[MAX_EVENTS+1];
  int jsr_num=0;
  int counter = 0;
  double how_long=0.0;
  int i;
  
  
  for(i=0;i<MAX_EVENTS;i++)
    local_opinst[i]=-1;

  cur_event = start;
  // look at time of first timer.
  while(counter < MAX_ALL_EVENTS && cur_event < end   ){

    if  ( prog_shm->is_noisy == 1) counter = 0; // so it doesn't overflow! not great... FIXME and in acq.c

    //    printf("event: %i, outputs: %i, opcode: %i duration: %f\n",cur_event,prog_shm->outputs[0][cur_event],prog_shm->opcodes[0][cur_event],prog_shm->times[0][cur_event]);
    switch (prog_shm->opcodes[0][cur_event] ){
    case CONTINUE:
      how_long += prog_shm->times[0][cur_event];
      cur_event+=1;
      break;
    case LONG_DELAY:
      how_long += prog_shm->times[0][cur_event]* prog_shm->opinst[0][cur_event];
      cur_event+=1;
      break;
    case WAIT:
      how_long += prog_shm->times[0][cur_event];
      cur_event+=1;
      fprintf(stderr,"Got a WAIT, times may be off\n");
      break;
    case LOOP:
      local_opinst[cur_event]=prog_shm->opinst[0][cur_event];
      how_long += prog_shm->times[0][cur_event];
      cur_event+=1;
      break;
    case END_LOOP:
      how_long += prog_shm->times[0][cur_event];
      cur_event_new = prog_shm->opinst[0][cur_event];
      // make sure this was a loop event
      if (local_opinst[cur_event_new] < 0){
	fprintf(stderr,"got a END_LOOP at event: %i, but didn't get a reasonable number of times to loop\n",cur_event);
	prog_shm->event_error = 1;
	//	done = ERROR_DONE; // can't do this in here...
	return how_long;
      }
      local_opinst[cur_event_new] -= 1;
      if (local_opinst[cur_event_new] > 0){
	cur_event = cur_event_new+1;
	how_long += prog_shm->times[0][cur_event_new];
	counter += 1;
      }
      else cur_event += 1;
      break;
    case BRANCH:
      how_long += prog_shm->times[0][cur_event];
      cur_event = prog_shm->opinst[0][cur_event];
      if (cur_event > MAX_EVENTS || cur_event < 0){
	fprintf(stderr,"got a branch to an event that doesn't exist.\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return how_long;
      }
      break;
    case JSR:
      //      printf("got a jsr at event: %i, going to event: %i\n",
      //      cur_event,prog_shm->opinst[0][cur_event]);
      how_long += prog_shm->times[0][cur_event];
      jsr_stack[jsr_num] = cur_event + 1;
      jsr_num += 1;
      if (jsr_num > STACK_SIZE -1){
	fprintf(stderr,"too many JSR's nested\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return how_long;
      }
      cur_event = prog_shm->opinst[0][cur_event];
      if (cur_event > MAX_EVENTS || cur_event < 0){
	fprintf(stderr,"got a jsr to an event that doesn't exist.\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return how_long;
      }
      break;
    case RTS:
      how_long += prog_shm->times[0][cur_event];
      if (jsr_num < 1){
	fprintf(stderr,"got a JSR, but nowhere to return to on stack!\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return how_long;
      }
      cur_event = jsr_stack[jsr_num -1];
      jsr_num -=1;
      if (cur_event <0 || cur_event > MAX_EVENTS){
	fprintf(stderr,"got a RTS to a non-existant event!\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return how_long;
      }
      break;
    default:
      fprintf(stderr,"got an unknown opcode!\n");
	prog_shm->event_error = 1;
      //      done = ERROR_DONE;
      return how_long;
    }
    
    counter += 1;
    
  }

  if (counter >= MAX_ALL_EVENTS){
    fprintf(stderr,"Got too many events (looping error? or forgotten STOP?)\n");
	prog_shm->event_error = 1;
    //    done = ERROR_DONE;
    return how_long;
  }


  //  fprintf(stderr,"partial_pp_time: how_long: %g\n",how_long);

  printf("partial_pp_time: %lf\n",how_long);
  return how_long;
}
double partial_pp_time_new(int start,int end){
  // this routine duplicates pp_time in acq, almost exactly.  They should probably be merged together.
  // start and end are the first and last events to include in the count

#define STACK_SIZE 8
#define MAX_LOOP_LEVELS 9
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
  while(counter < MAX_ALL_EVENTS && cur_event < end   ){

    //    printf("event: %i, outputs: %i, opcode: %i duration: %f\n",cur_event,prog_shm->outputs[0][cur_event],prog_shm->opcodes[0][cur_event],prog_shm->times[0][cur_event]);
    switch (prog_shm->opcodes[0][cur_event] ){
    case CONTINUE:
      loop_time[loop_level] += prog_shm->times[0][cur_event];
      cur_event+=1;
      break;
    case LONG_DELAY:
      loop_time[loop_level] += prog_shm->times[0][cur_event]* prog_shm->opinst[0][cur_event];
      cur_event+=1;
      break;
    case WAIT:
      loop_time[loop_level] += prog_shm->times[0][cur_event];
      cur_event+=1;
      fprintf(stderr,"Got a WAIT, times may be off\n");
      break;
    case LOOP:
      loop_level += 1;
      if (loop_level >= MAX_LOOP_LEVELS-1){ // max is one too big, since we use 0 for not looping.
	fprintf(stderr,"Got a start loop too many levels deep\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return loop_time[0];
      }
      loop_time[loop_level] = prog_shm->times[0][cur_event];
      loops[loop_level] = prog_shm->opinst[0][cur_event]; // load the loop counter
      cur_event+=1;
      break;
    case END_LOOP:
      if (loop_level <1){
	fprintf(stderr,"Got a loop end but not enough loop starts?\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return loop_time[0];
      }
      loop_time[loop_level] += prog_shm->times[0][cur_event];
      loop_level -= 1;
      //      printf("at end loop. loop time was: %lf, loops: %i\n",loop_time[loop_level+1],loops[loop_level+1]);
      loop_time[loop_level] += loop_time[loop_level+1]*loops[loop_level+1];
      cur_event += 1;
      break;
    case BRANCH:
      loop_time[loop_level] += prog_shm->times[0][cur_event];
      cur_event = prog_shm->opinst[0][cur_event];
      if (cur_event > MAX_EVENTS || cur_event < 0){
	fprintf(stderr,"got a branch to an event that doesn't exist.\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return loop_time[0];
      }
      break;
    case JSR:
      //      printf("got a jsr at event: %i, going to event: %i\n",
      //      cur_event,prog_shm->opinst[0][cur_event]);
      loop_time[loop_level] += prog_shm->times[0][cur_event];
      jsr_stack[jsr_num] = cur_event + 1;
      jsr_num += 1;
      if (jsr_num > STACK_SIZE -1){
	fprintf(stderr,"too many JSR's nested\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return loop_time[0];
      }
      cur_event = prog_shm->opinst[0][cur_event];
      if (cur_event > MAX_EVENTS || cur_event < 0){
	fprintf(stderr,"got a jsr to an event that doesn't exist.\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return loop_time[0];
      }
      break;
    case RTS:
      loop_time[loop_level] += prog_shm->times[0][cur_event];
      if (jsr_num < 1){
	fprintf(stderr,"got a JSR, but nowhere to return to on stack!\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return loop_time[0];
      }
      cur_event = jsr_stack[jsr_num -1];
      jsr_num -=1;
      if (cur_event <0 || cur_event > MAX_EVENTS){
	fprintf(stderr,"got a RTS to a non-existant event!\n");
	prog_shm->event_error = 1;
	//	done = ERROR_DONE;
	return loop_time[0];
      }
      break;
    default:
      fprintf(stderr,"got an unknown opcode!\n");
	prog_shm->event_error = 1;
      //      done = ERROR_DONE;
      return loop_time[0];
    }
    
    counter += 1;
    
  }

  if (counter >= MAX_ALL_EVENTS){
    fprintf(stderr,"Got too many events (looping error? or forgotten STOP?)\n");
	prog_shm->event_error = 1;
    //    done = ERROR_DONE;
    return loop_time[0];
  }


  //  fprintf(stderr,"partial_pp_time: how_long: %g\n",how_long);
  // ok, in case we set the dsp_sync in the middle of a hardware loop
  for (i=1;i<=loop_level;i++)
    loop_time[0] += loop_time[i];
  printf("partial_pp_time_new: %lf\n",loop_time[0]);
  return loop_time[0];
}

// utility functions, used by chirp2.x and nowhere else.
double get_old_freq1(){

  return prog_shm->lfreq1;
}

double get_old_freq2(){

  return prog_shm->lfreq2;
}

void set_old_freq1(double freq1){
  prog_shm->lfreq1 = freq1;
}

void set_old_freq2(double freq2){
  prog_shm->lfreq2 = freq2;
}


void mark_synth_setup_flag(){
  prog_shm->do_synth_setup = 1;
  }


void set_receiver_model_stream(){
  prog_shm->receiver_model = REC_STREAM;
}
/*
pulseblaster commands are:
CONTINUE
LOOP [# times]
END LOOP [where back to]
LONG EVENT [# times]
BRANCH [where]
JSR [where]
RTS
STOP
WAIT

We also assign labels to commands to identify the wheres.


we're going to write pulse.c so synth events get put in the event before the current one.
We shouldn't put synth events in any event with a label, or any LOOP command (which should have a label).

We also don't want the previous event (that we drop into) to be and END LOOP, a LONG EVENT, a BRANCH,a JSR, or an RTS.  LOOP, CONTINUE, and WAIT are ok.

*/


