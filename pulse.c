//#define TIMING
/*  pulse.c 


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

  A few notes on the amplitude and phase correction schemes for the new RF transmitters:

  1) amplitude was easy:  collect data at different amplitude settings: with _AMP = 0 to 1023.
     then fit it with a tanh, put the inverse in here.  no problem.
  2) phase was not really harder...
    a) to feed transmitter output through atten ( 40-60dB or so?) then into receiver.
    using pulse program phase_sweep, with dwell = 25u, pw1 = 50u. (as written, looks like
    about a 32 point left shift).
    b) export the data - make sure the first point in the data file corresponds to phase =0
    b) use read_phases to read them out (always reads from acq_tempexport.txt)
    c) reverse_pairs to reverse them. - reads from stdin, use pipes:  < infile >outfile

    presto.  the files that we look to for the phase lookups are: /usr/src/Xnmr/current/correct_phaseN.txt
    where the N is a 1 2 or 3.

notes: phase_sweep starts off at -5 deg, so you'll want to left shift to there before exporting


   November 28, 2005.  Preliminary support for gradients added.  At this point, we assume that the hardware channel that
   is not assigned to RF1 or RF2 is assigned to the gradient.



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
  dlopen's the pulse program:

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

 #include "pulse.h" 
 #include "/usr/share/Xnmr/config/h_config.h" 
 #include "p_signals.h" 
 #include "shm_data.h" 
 #include "shm_prog.h" 
 #include "param_utils.h" 

 #include <unistd.h> 
 #include <sys/time.h> 
 #include <time.h>

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
   short start_chip_num;
   short end_chip_num;
   short chip_bit;
   unsigned int load_mask;
 }; 

int num_dev=0;
char bgu_inited = 0; // have we called init_bgu?
char bgu_cleared = 0; // did we clear it?
int bgu_last_vals_zeros; 
float bgu_alpha,bgu_beta,bgu_gamma;
float bgu_xscale,bgu_yscale,bgu_zscale;

static prog_image_t old_prog_image;
int old_no_events;

struct hardware_config_t *hardware_config; 
struct data_shm_t* data_shm; 
struct prog_shm_t* prog_shm;        //These are shared memory structures and must be initialized 
unsigned char latch_mask[NUM_CHIPS],default_mask[NUM_CHIPS];

struct itimerval mytime,old; 
int data_shm_id; 
int prog_shm_id; 
int msgq_id; 
int finished; 

int tran_table[9]; 
int ampa=-1,ampb=-1,ampc=-1,phasea=-1,phaseb=-1,phasec=-1;
int clka=-1,clkb=-1,clkc=-1;
int ic=-1,qc=-1,_ampc=-1;
int ib=-1,qb=-1,_ampb=-1;
int ia=-1,qa=-1,_ampa=-1;

int grad_channel = 7; /// this is going to tell write_device_wrap which channel the gradient is attached to.
// its a 1 for a, 2 for b, 4 for c.
int gradI = -1, gradQ = -1; // these will be devices in h_config that are connected to the BGU.

// variables for going back to a spot in time...
long long how_far_back = 0;
char position_stored=0;
char currently_going_back=0;



// a few prototypes
int shift_events(int start);
int lookup_ampc(float fval);
int lookup_ampb(float fval);
int lookup_ampa(float fval);
int lookup_amp(float fval);
void lookup_phase(float fval,int *ival,int *qval);
void lookup_phasea(float fval,int *ival,int *qval);
void lookup_phaseb(float fval,int *ival,int *qval);
void lookup_phasec(float fval,int *ival,int *qval);

void deal_with_previous(int event_no,int is_amp_phase,int device_id, int intval,float fval);


 /* 
  *   Method Implementations 
   */



void set_timer(int event_no,long long time){
  if (event_no >= MAX_EVENTS) return;
  prog_shm->prog_image[0][event_no]=time % 256;
  prog_shm->prog_image[1][event_no]=(time >> 8) % 256;
  prog_shm->prog_image[2][event_no]=(time >> 16) % 256;
  prog_shm->prog_image[3][event_no]=(time >> 24 ) % 256;
}

 /* 
  *  This method ensures that the pulse program exits normally even when sent a SIGTERM or SIGINT signal. 
  *  The finished flag indicates that the pulse program should stop looping and begin 
  *  it's exit sequence 
   */

 void stop() 

 { 
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
  *  write_device() does all the dirty work for event().  It could 
  *  be updated at some point to be more effecient.  Currently, it 
  *  writes the devices 1 bit at a time.  This could be sped up by 
  *  using a long* to write all of the bits at the same time 
   */


 int write_device( int  device_id, unsigned int val,int event_no ) 

 { 
   unsigned int i;
   unsigned int dum2;
   unsigned char *val_c,*mask_c;

   //   fprintf(stderr,"write_device: dev: %i, val: %i, event: %i,bit %i ",device_id,val,event_no,hardware_config[device_id].start_bit);

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
   dum2 = val << hardware_config[device_id].chip_bit;

   val_c = (unsigned char *) &dum2;
   mask_c = (unsigned char *) &hardware_config[device_id].load_mask;

   // now load the bytes as needed
   for (i=0; i<= hardware_config[device_id].end_chip_num-hardware_config[device_id].start_chip_num;i++){
     prog_shm->prog_image[i+hardware_config[device_id].start_chip_num][event_no] = (val_c[i] & mask_c[i] )+
       ( ~mask_c[i] & prog_shm->prog_image[i+hardware_config[device_id].start_chip_num][event_no]);
   }

   return 0; 

 } 

/* 
 int write_device( int  device_id, unsigned int val,int event_no ) 

 { 
   unsigned char bit; 
   unsigned char chip; 
   unsigned char chip_bit; 
   unsigned char chip_mask; 



   //   fprintf(stderr,"write_device: dev: %i, val: %i, event: %i,bit %i ",device_id,val,event_no,hardware_config[device_id].start_bit);

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

   //Do this one bit at a time 


   for( bit = 0; bit < hardware_config[ device_id ].num_bits; bit++ ) { 

     chip = ( bit + hardware_config[ device_id ].start_bit ) /8; 
     chip_bit = ( bit + hardware_config[ device_id ].start_bit ) %8; 
     chip_mask = ~(0x01 << chip_bit); 

     prog_shm->prog_image[ chip ][ event_no  ] =  
       (prog_shm->prog_image[ chip ][ event_no ] & chip_mask) +  
       ( ( (val >> bit) %2 ) << chip_bit ); 

   }
   return 0; 

 } 
*/


 int ready( char phase ) 

 { 
   int result,i,j; 
   struct msgbuf message; 
   static int first_time = 1;
 
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


   // deal with BGU gradients:
   if (bgu_inited ){
     unsigned int cache_got_ppo;
     // write extra events after the end of the pulse program.
     printf("ready: bgu was inited, writing emergency program\n");
     prog_shm->bgu_emergency_address = prog_shm->no_events;
     event(100e-9,0); // do nothing
     write_bgu_raw(0.,0.,0.,1); // set all gradient channels to 0.
     event(100e-9,0); 
     event(150e-9,1,GRSTROBES,0x01); // set NGI low (gets inverted when downloading)
     event(100e-3,0); // 100 ms, should be long enough for start up to work out ok.
     cache_got_ppo = prog_shm->got_ppo; // we're going to add in a ppo. but don't want test for ppo to depend on ours.
     event(100e-3,1,PP_OVER,1);
     prog_shm->got_ppo = cache_got_ppo;
     if (bgu_last_vals_zeros != 1){
       prog_shm->event_error = 1;
       fprintf(stderr,"ready: The bgu_last_vals weren't all zero\n");
     }
     if (bgu_cleared == 0){
       prog_shm->event_error = 1;
       fprintf(stderr,"ready: BGU inited but not cleared.\n");
     }
     
   }


   
   // deal with clean/dirty stuff;
   // mark all dirty
   for (i=0;i<NUM_CHIPS;i++)
     prog_shm->chip_clean[i]=DIRTY;


   // if the number of events is different, then program is different.
   // also don't bother to compare if the program hasn't been downloaded, because it will need to be anyway.
  if (old_no_events == prog_shm->no_events && prog_shm->downloaded == 1){ 

    
    for( i = 0; i < NUM_CHIPS ; i++ ){		
      for(j=0;j< (prog_shm->no_events+sizeof(unsigned int *)-1)/sizeof(unsigned int); j++ ){
	  if ( ((unsigned int *)prog_shm->prog_image[i])[j] != 
	       ((unsigned int *)old_prog_image[i])[j]){
	    j = prog_shm->no_events+10;	      // break out of loop.
	    //	    fprintf(stderr,"ready: chip %i is dirty\n",i);
	  }
      }
	if ( j == (prog_shm->no_events+sizeof(unsigned int *)-1)/sizeof(unsigned int)){
	  prog_shm->chip_clean[i] = CLEAN;
	  //	  fprintf(stderr,"ready: marking chip: %i as clean\n",i);
	}
      }// end NUM_CHIPS  
  } // end needed to compare one by one.
    

  /*
    for( i = 0; i < NUM_CHIPS ; i++ ){
      
      for (j=0;j<prog_shm->no_events;j++){
	if (prog_shm->prog_image[i][j] != old_prog_image[i][j]){
	  j = prog_shm->no_events+10;	      // break out of loop.
	  //	    fprintf(stderr,"ready: chip %i is dirty\n",i);
	}
      }
      if ( j == prog_shm->no_events ){
	prog_shm->chip_clean[i] = CLEAN;
	//	  fprintf(stderr,"ready: marking chip: %i as clean\n",i);
      }
      
      
    }// end NUM_CHIPS    
  }// end needed to compare one by one

  */




#ifdef TIMING

  gettimeofday(&end_time,&tz);
  d_time=(end_time.tv_sec-start_time.tv_sec)*1e6+(end_time.tv_usec-start_time.tv_usec);
  fprintf(stderr,"compare time: %.0f us\n",d_time);


#endif



   //   fprintf(stderr,"coming into ready, num events is: %i\n",prog_shm->no_events);

     


  //  /*
     // start dump
  
     if (first_time == 1){
       //Dump the pulse program to a file in two formats 
       
       FILE* fid; 
       int event; 
       int chip; 
       int bit; 
       
       fprintf(stderr, "dumping pulse program to file\n" ); 
       //       fprintf(stderr,"in dumping, first_time is: %i\n",first_time);
       fid = fopen( "pprog.txt", "w" ); 
       
       for( event=0; event<prog_shm->no_events; event++ ) { 
	 for( chip=0; chip<NUM_CHIPS; chip++ ) { 
	   for( bit=0; bit<8; bit++ ) { 
	     fprintf( fid, "%d", (prog_shm->prog_image[ chip ][ event ] >> bit) %2 ); 
	   } 
	   fprintf( fid, " " ); 
	 } 
	 fprintf( fid, "\n" ); 
	 
       } 
       
       //The second format 
       
       for( event=0; event<prog_shm->no_events; event++ ) { 
	 long long counter;
	 counter = TIME_OF(event) + 1;
	 fprintf(fid,"%12.8f %14lli ", counter /(float)CLOCK_SPEED, counter);
	 for( chip=4; chip<NUM_CHIPS; chip++ ) { 
	   fprintf( fid, "%3d ", prog_shm->prog_image[ chip ][ event ] );  
	 } 
	 fprintf( fid, "\n" ); 
	 
       } 
       fclose( fid ); 
     }
     
     //  */  
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

   prog_shm->phase = phase;  //let acq know what phase shift to apply to data 

   //   fprintf(stderr, "pprog sending message P_PROGRAM_READY\n" ); 

   message.mtype = P_PROGRAM_READY; 
   message.mtext[0] = P_PROGRAM_READY;

   result=msgsnd ( msgq_id, &message, 1, 0 ); 
   if (result == -1) perror("pulse.c:msgsnd"); 
   //   fprintf(stderr,"inside pulse, just sent P_PROGRAM_READY\n"); 

   result = wait_for_acq_msg( ); 
   //   fprintf(stderr,"inside pulse, just got message for CALC\n"); 
   prog_shm->prog_ready = NOT_READY; 

   return result; 
 } 



int write_device_wrap( int start_event_no,int end_event_no ,int device_id, int intval,float fval) 
     // val is a pointer here because is might be a float
     // return value is sum of 1,2,4 for amp/phase event for channels a b c
{

   int i,is_amp_phase=0;
   int ival,qval,val;


   // set up special device numbers - done in pulse_prog_init
   if (clkc == -1 ){
     fprintf(stderr,"write_device_wrap: pulse_program not init'ed\n");
     prog_shm->event_error = 1;
     return -1;
   }


 if (device_id >= RF_OFFSET){ 
   //   fprintf(stderr,"translating device: %i ",device_id);
   device_id = tran_table[device_id-RF_OFFSET];
   //   fprintf(stderr,"to device: %i\n",device_id);
 }
 
 if ( device_id == PP_OVER ) prog_shm->got_ppo = 1; // to ensure there is a ppo 

 for (i=start_event_no;i<=end_event_no;i++){
   if ( device_id == ampc ){
     val = lookup_ampc(fval);
     is_amp_phase |= 4;
     write_device(_ampc,val,i);
     write_device(clkc,0,i);
     
   }
   
   else  if (device_id == phasec){
     //   fprintf(stderr,"got phasec, value: %f\n",fval);
     lookup_phasec(fval,&ival,&qval);
     is_amp_phase |= 4;
     
     write_device(ic,ival,i);
     write_device(qc,qval,i);
     write_device(clkc,0,i);
     
   }
   else if ( device_id == ampb ){
     //       fprintf(stderr,"got ampb, value: %f\n",fval);
     
     val = lookup_ampb(fval);
     is_amp_phase |= 2;
     write_device(_ampb,val,i);
     write_device(clkb,0,i);
     
   }
   
   else  if (device_id == phaseb){
     //       fprintf(stderr,"got phaseb, value: %f\n",fval);
     lookup_phaseb(fval,&ival,&qval);
     is_amp_phase |= 2;
     
     write_device(ib,ival,i);
     write_device(qb,qval,i);
     write_device(clkb,0,i);
     
   }
   else if ( device_id == ampa ){
     //       fprintf(stderr,"got amp1, value: %f\n",fval);
     
     
     val = lookup_ampa(fval);
     is_amp_phase |= 1;
     write_device(_ampa,val,i);
     write_device(clka,0,i);
     
   }
   
   else  if (device_id == phasea){
  
    //       fprintf(stderr,"got phase1, value: %f\n",fval);
     lookup_phasea(fval,&ival,&qval);
     is_amp_phase |= 1;
     
     write_device(ia,ival,i);
     write_device(qa,qval,i);
     write_device(clka,0,i);
     
   }
   
   else{ // just an ordinary device setting.
     write_device( device_id, intval,i); 
   }
 }
 
 return is_amp_phase;
}

int is_a_float_device(int device_id)
{
  if (clkc == -1){
    fprintf(stderr,"pulse program not inited!!\n");
    prog_shm->event_error = 1;
    return -1;
  }
  if (device_id >= RF_OFFSET)
    device_id = tran_table[device_id-RF_OFFSET];
  if (device_id == phasea || device_id == ampa) return 1;
  if (device_id == phaseb || device_id == ampb) return 2;
  if (device_id == phasec || device_id == ampc) return 4;
  return 0;

}

   
int event( double time, unsigned char num, ... ) 

 { 
   va_list args; 
   int i,intval=0,is_amp_phase; 
   unsigned char device_id; 
   unsigned char time_save;
   float fval=0.;
   long long int counts;   //This is a 64 bit integer 
   char long_event; // flag to say if event was split because of length.

   //   fprintf(stderr,"\ncoming into event, number is: %i, number of devices: %i\n",prog_shm->no_events, num);
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

   //   fprintf(stderr,"event_no: %i time: %lg \n",prog_shm->no_events,time);

   counts = (long long int) ( ( time *  CLOCK_SPEED ) -  0.5 ); 

   if( counts <0 ) {
     //     if (time < 0)
     fprintf(stderr,"event: time < 0, ignored\n");
     fprintf(stderr,"time was: %lg, counts: %lli\n",time,counts);
     return 0; 
   }
   if (!isnormal(time)){
     if (time != 0.){
       fprintf(stderr,"TIME IS NOT A NUMBER!\n");
       prog_shm->event_error = 1;
       return -1;
     }
     else{
       fprintf(stderr,"event: time < 0, ignored\n");
       fprintf(stderr,"time was: %lg\n",time);
       return 0; 
     }
   }

   va_start( args, num ); 

   if (currently_going_back == 1){
     if ( how_far_back > 0){
       //       fprintf(stderr,"in event, got currently_going_back by time: %.7f\n",(float) how_far_back/CLOCK_SPEED);

       // this should just be one simple go_back call, but I
       // don't see how to pass the variable args back...
       //       go_back((float)how_far_back/CLOCK_SPEED,time,num,...);
       //       return 0;
       
       for( i=0; i<num; i++ ) { 
	 device_id = (unsigned char) va_arg( args, int  ); 
	 //	 fprintf(stderr,"got device_id: %i for event: %i\n",(int) device_id,prog_shm->no_events);
	 if (is_a_float_device(device_id) >0 ){
	   fval = (float) va_arg(args,double);
	   go_back((float) how_far_back/CLOCK_SPEED,time,1,device_id,fval);
	 }
	 else{
	   intval =  va_arg(args,unsigned int);
	   go_back((float) how_far_back/CLOCK_SPEED,time,1,device_id,intval);
	 }
       }

       how_far_back -= (counts+1);

       va_end(args);
       return 0;
     }
     fprintf(stderr,"in event: you're trying to 'go_back' into the future..., just doing ordinary event\n");

   }
   

   if (position_stored == 1 && currently_going_back == 0){
     how_far_back += counts+1;
     //     fprintf(stderr,"in event: how_far_back is now: %f\n",(float) how_far_back/CLOCK_SPEED);
   }
   
   
   //duplicate the last event and apply latch mask & defaults

   if( prog_shm->no_events > 0 ) { 
     for( i=0; i<NUM_CHIPS; i++ ) { 
       prog_shm->prog_image[ i ][ prog_shm->no_events ] =  
	 (prog_shm->prog_image[ i ][prog_shm->no_events-1] & latch_mask[i]) + 
	 (default_mask[i] & ~latch_mask[i]);
     }
   }
   else // just put in defaults:
     for(i=0;i<NUM_CHIPS; i++)
       prog_shm->prog_image[i][prog_shm->no_events] = default_mask[i];


   //set all the specified device information 
   
   //   fprintf(stderr,"\nin event, no_events: %i,num things this event: %i\n",prog_shm->no_events,num);
   for( i=0; i<num; i++ ) { 
     device_id = (unsigned char) va_arg( args, int  ); 
     //     fprintf(stderr,"got device_id: %i for event: %i\n",(int) device_id,prog_shm->no_events);
     if (is_a_float_device(device_id) >0 )
       fval = (float) va_arg(args,double);
     else
       intval =  va_arg(args,unsigned int);
     is_amp_phase = write_device_wrap(prog_shm->no_events,prog_shm->no_events,device_id,intval,fval);
     
     if (is_amp_phase > 0){
       if (prog_shm->no_events == 0 && counts > 0 ) counts -= 1; 
       // going to create a new first event... not clear if this should be here or not

       deal_with_previous(prog_shm->no_events,is_amp_phase,device_id,intval,fval);
       } 
   }
 


   // if this event is longer than the pulse prog timer can hold, split it up:
   long_event = 0;
   while( counts > MAX_CLOCK_CYCLES ) { 
     set_timer(prog_shm->no_events,MAX_CLOCK_CYCLES);
     counts -= MAX_CLOCK_CYCLES; 
     long_event = 1;

     shift_events(prog_shm->no_events);
     // if there's a PP_OVER or an XTRN_TRIG in here, should get rid of it - shift_events takes care of this.
     

   }

   set_timer(prog_shm->no_events,counts);

   // ick, if this is the last event, should make the last event a long one, and the second to last shorter
   if ( long_event == 1 ) {
     // swap the counters from the last two events.
     for (i=0;i<4;i++){
       time_save = prog_shm->prog_image[i][prog_shm->no_events];
       prog_shm->prog_image[i][prog_shm->no_events]=prog_shm->prog_image[i][prog_shm->no_events-1];
       prog_shm->prog_image[i][prog_shm->no_events-1]=time_save;
     }
   }	  

   prog_shm->no_events++; 
   va_end(args); 

   return 0; 
 } 

   
 
 
 double set_freq1(double freq){ 
   /* ok freq is a double that is the frequency in MHz  */

 int dum1,dum2,dum3,dum4,dum5,dum6,dum7,dum8,dum9,dum0; 
 double freq2; 
 double time; 
 static double old_freq=0; 
 static int s1_1=-1,s1_2=-1,s1_latch=-1;
 int i;

 //  fprintf(stderr,"arriving in set_freq1 with freq: %lf\n",freq);

 // first of all, find the synth device numbers
 if (s1_1 == -1){
   for(i=0;i<num_dev;i++){
     if(strcmp(hardware_config[i].name,"SYNTH1_1") == 0)
       s1_1 = i;
   }
   
   for(i=0;i<num_dev;i++){
     if(strcmp(hardware_config[i].name,"SYNTH1_2") == 0)
       s1_2 = i;
   }
   
   for(i=0;i<num_dev;i++){
     if(strcmp(hardware_config[i].name,"SYNTH1_LATCH") == 0)
       s1_latch = i;
   }
   if ( (s1_1 == -1) || (s1_2 == -1) || (s1_latch == -1)){
     fprintf(stderr,"set_freq1: couldn't find one of my devices\n");
     exit(0);
   }
   //   fprintf(stderr,"set_freq1, got device numbers: %i %i %i\n",s1_1,s1_2,s1_latch);
 }
 



 time=5000e-9; 
 freq2=freq*1e6+.0001+234375*90.; 
 // freq2=freq*1e6+.0001+21000000.; 


 // freq2=freq*1e6+.0001; 
 // .0001 prevents roundoff error.
 // 234375 is the smallest integral value the dsp and dds can produce. 

 if (old_freq == freq2 && data_shm->force_synth == 0 ){
   //   fprintf(stderr,"set freq1: returning, force is: %i\n",data_shm->force_synth);
   return 0.0; 
 }
 old_freq = freq2;  

 // fprintf(stderr,"freq1: setting freq1 %f\n",freq2); 
 dum1= (int) freq2/1E8; 
 freq2 -= dum1*1E8; 

 dum2= (int) (freq2/1E7); 
 freq2 -= dum2*1E7; 

 dum3= (int) (freq2/1E6); 
 freq2  -=dum3*1E6; 
 dum4= (int) (freq2/1E5); 
 freq2 -= dum4*1E5; 

 dum5= (int) (freq2/1E4); 
 freq2  -=dum5*1E4; 
 dum6= (int) (freq2/1E3); 
 freq2 -= dum6*1E3; 

 dum7= (int) (freq2/1E2); 
 freq2  -=dum7*1E2; 
 dum8= (int) (freq2/1E1); 
 freq2 -= dum8*1E1; 

 dum9= (int) (freq2/1E0); 
 freq2  -=dum9*1E0; 
 dum0= (int) (freq2/1E-1); 
 freq2 -= dum0*1E-1; 


 /* fprintf(stderr, "Output Frequency: %d%d", dum1,dum2 ); 
 fprintf(stderr, "%d%d", dum3,dum4 ); 
 fprintf(stderr, "%d%d", dum5,dum6 ); 
 fprintf(stderr, "%d%d", dum7,dum8 ); 
 fprintf(stderr, "%d.%d\n", dum9,dum0 ); 
 */
 // fprintf(stderr,"set freq, calling synth events\n");
 event(time,3,s1_1,dum2,s1_2,dum1 ,s1_latch,3); 
 event(time,3,s1_1,dum2,s1_2,dum1 ,s1_latch,0); 

 event(time,3,s1_1,dum4,s1_2,dum3 ,s1_latch,4); 
 event(time,3,s1_1,dum4,s1_2,dum3 ,s1_latch,0); 

 event(time,3,s1_1,dum6,s1_2,dum5 ,s1_latch,5); 
 event(time,3,s1_1,dum6,s1_2,dum5 ,s1_latch,0); 

 event(time,3,s1_1,dum8,s1_2,dum7 ,s1_latch,6); 
 event(time,3,s1_1,dum8,s1_2,dum7 ,s1_latch,0); 

 event(time,3,s1_1,dum0,s1_2,dum9 ,s1_latch,7); 
 event(time,3,s1_1,dum0,s1_2,dum9 ,s1_latch,0); 


 return time*10.0; 
 } 



 /****************** */
 double set_freq2(double freq){ 
   /* ok freq is a double that is the frequency in MHz  
      Return value is how much pulse programmer time this routine used.*/

   int dum1,dum2,dum3,dum4,dum5,dum6,dum7,dum8,dum9,dum0; 
 double freq2; 
 double time; 
 static double old_freq=0; 
 static int s2_1=-1,s2_2=-1,s2_latch=-1;
 int i;

 if (s2_1 == -1){
   for(i=0;i<num_dev;i++){
     if(strcmp(hardware_config[i].name,"SYNTH2_1") == 0)
       s2_1 = i;
   }
   
   for(i=0;i<num_dev;i++){
     if(strcmp(hardware_config[i].name,"SYNTH2_2") == 0)
       s2_2 = i;
   }
   
   for(i=0;i<num_dev;i++){
     if(strcmp(hardware_config[i].name,"SYNTH2_LATCH") == 0)
       s2_latch = i;
   }
   if ( (s2_1 == -1) || (s2_2 == -1) || (s2_latch == -1)){
     fprintf(stderr,"set_freq1: couldn't find one of my devices\n");
     exit(0);
   }
 }


 time=5000e-9; 
 freq2=freq*1e6+.0001+234375*90.; 
 // freq2=freq*1e6+.0001+21000000.; 
 //  freq2=freq*1e6+.0001; 

 // if (old_freq==freq2) return 0; 
 if (old_freq == freq2 && data_shm->force_synth == 0 ) return 0.0; 
 old_freq=freq2;  

 //  fprintf(stderr,"freq2:  %f\n",freq2); 
 dum1= (int) freq2/1E8; 
 freq2 -= dum1*1E8; 

 dum2= (int) (freq2/1E7); 
 freq2 -= dum2*1E7; 

 dum3= (int) (freq2/1E6); 
 freq2  -=dum3*1E6; 
 dum4= (int) (freq2/1E5); 
 freq2 -= dum4*1E5; 

 dum5= (int) (freq2/1E4); 
 freq2  -=dum5*1E4; 
 dum6= (int) (freq2/1E3); 
 freq2 -= dum6*1E3; 

 dum7= (int) (freq2/1E2); 
 freq2  -=dum7*1E2; 
 dum8= (int) (freq2/1E1); 
 freq2 -= dum8*1E1; 

 dum9= (int) (freq2/1E0); 
 freq2  -=dum9*1E0; 

  dum0= (int) (freq2/1E-1); 
 freq2  -=dum0*1E-1; 


 /* fprintf(stderr, "Output Frequency: %d%d", dum1,dum2 ); 
 fprintf(stderr, "%d%d", dum3,dum4 ); 
 fprintf(stderr, "%d%d", dum5,dum6 ); 
 fprintf(stderr, "%d%d", dum7,dum8 ); 
 fprintf(stderr, "%d.\n", dum9); 
 */

 /* Rhode-Schwarz 
 event(time,3,s2_1,dum9,s2_2,dum8 ,s2_latch,7); 
 event(time,3,s2_1,dum9,s2_2,dum8 ,s2_latch,0); 

 event(time,3,s2_1,dum7,s2_2,dum6 ,s2_latch,6); 
 event(time,3,s2_1,dum7,s2_2,dum6 ,s2_latch,0); 

 event(time,3,s2_1,dum5,s2_2,dum4 ,s2_latch,5); 
 event(time,3,s2_1,dum5,s2_2,dum4 ,s2_latch,0); 

 event(time,3,s2_1,dum3,s2_2,dum2 ,s2_latch,4); 
 event(time,3,s2_1,dum3,s2_2,dum2 ,s2_latch,0); 

 event(time,3,s2_1,dum1,s2_2,15,s2_latch,3); 
 event(time,3,s2_1,dum1,s2_2,15,s2_latch,0); 

 */

 // PTS500: with Yael's cable
event(time,3,s2_1,dum2,s2_2,dum1 ,s2_latch,2); 
 event(time,3,s2_1,dum2,s2_2,dum1 ,s2_latch,0); 
 
 event(time,3,s2_1,dum4,s2_2,dum3 ,s2_latch,3); 
 event(time,3,s2_1,dum4,s2_2,dum3 ,s2_latch,0); 

 event(time,3,s2_1,dum6,s2_2,dum5 ,s2_latch,4); 
 event(time,3,s2_1,dum6,s2_2,dum5 ,s2_latch,0); 

 event(time,3,s2_1,dum8,s2_2,dum7 ,s2_latch,5); 
 event(time,3,s2_1,dum8,s2_2,dum7 ,s2_latch,0); 

 event(time,3,s2_1,dum0,s2_2,dum9 ,s2_latch,6); 
 event(time,3,s2_1,dum0,s2_2,dum9 ,s2_latch,0); 



 return time*10.0; 
 } 






 int begin() 

 { 
   int i,j;
   
   // copy old program to save place for later comparison.
   // 32 bit copies are faster...
   
   old_no_events = prog_shm->no_events;
   for (i=0;i<NUM_CHIPS;i++)
     for( j = 0 ; j < (prog_shm->no_events+sizeof(unsigned int)-1)/sizeof(unsigned int);j++)
       ((unsigned int *)old_prog_image[i])[j] = 
       ((unsigned int *) prog_shm->prog_image[i])[j]; 

   /*
   for (i=0;i<NUM_CHIPS;i++)
     for( j = 0 ; j < prog_shm->no_events;j++)
       old_prog_image[i][j] = 
	 prog_shm->prog_image[i][j];
   */

	  //   memcpy(old_prog_image,prog_shm->prog_image,NUM_CHIPS_M*prog_shm->no_events);

   prog_shm->no_events = 0; 
   prog_shm->event_error = 0;
   prog_shm->got_ppo = 0;
   prog_shm->is_noisy = 0;
   //   fprintf(stderr,"in begin, just set is_noisy to false\n");

   bgu_inited = 0;
   prog_shm->bgu_emergency_address = 0;
   bgu_last_vals_zeros = 1;
   
   prog_shm->begun = 1;

   /* set up to catch infinite loops */
   mytime.it_interval.tv_sec = 0; 
   mytime.it_interval.tv_usec = 0; 
   mytime.it_value.tv_sec = 3; 
   mytime.it_value.tv_usec = 0; 
   signal( SIGALRM, pprog_internal_timeout);  
   setitimer( ITIMER_REAL, &mytime, &old ); 


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

   memset( prog_shm->prog_image, 0, MAX_EVENTS*NUM_CHIPS );
   memset( old_prog_image, 0, MAX_EVENTS*NUM_CHIPS );

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

   fid = fopen( "/usr/share/Xnmr/config/h_config.h", "r" ); 

   if (fid == NULL) {
     fprintf(stderr,"pulse.c: couldn't open h_config.h\n");
     exit(0);
   }

   // look for how many devices
   do { 
     eo = fgets( s, PATH_LENGTH, fid );  
   } while( strstr( s, "NUM_DEVICES" ) == NULL || eo == NULL ); 

   if (eo == NULL){
     fprintf(stderr,"pulse.c: didn't find the number of device in h_config.h\n");
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
       else fprintf(stderr, "Invalid device number\n" ); 
     } 

   } while( strcmp( s, PARSE_END ) ); 


   // build masks for rapid loading of pulse program..

   for (i=0; i<num_dev;i++){
     if (hardware_config[i].start_bit >= 0){ // don't bother for the phoney devices
       hardware_config[i].start_chip_num =  hardware_config[i].start_bit/8;

       // the -1 on the next line was missing - bug...
       hardware_config[i].end_chip_num = (hardware_config[i].start_bit+hardware_config[i].num_bits-1)/8;
       hardware_config[i].chip_bit = hardware_config[i].start_bit % 8;

       //       fprintf(stderr,"device: %i, start bit: %i, end_bit: %i, start_chip_num: %i, end_chip_num: %i\n",i,hardware_config[i].start_bit,
       //	      hardware_config[i].start_bit+hardware_config[i].num_bits-1,hardware_config[i].start_chip_num,hardware_config[i].end_chip_num);

       // here's some bit shifting magic...
       hardware_config[i].load_mask = 
	 (((unsigned int) 0xFFFFFFFF) >> 
	  ( sizeof(unsigned int)*8 - hardware_config[i].num_bits)) 
				      << hardware_config[i].chip_bit;
       
       //     fprintf(stderr,"init: dev: %i, chip: %i bit: %i ,mask: %i ",i,(int) hardware_config[i].start_chip_num,(int)hardware_config[i].chip_bit, (int)hardware_config[i].load_mask);
       //     fprintf(stderr,"num_bits: %i\n",hardware_config[i].num_bits);
       
       if (hardware_config[i].chip_bit + hardware_config[i].num_bits-1 > sizeof(unsigned int)*8){
	 fprintf(stderr,"init_hardware:  device %i crosses a word boundary.  Not supported\n",i);
	 exit(0);
       }
     }
   }


   // build latch mask

   for( i=0 ; i<num_dev ; i++){
     if (hardware_config[i].latch == 1 ){
       //       fprintf(stderr,"device %i writing 1's to latch mask\n",i);
       write_device(i,(0xFFFFFFFF >> (sizeof(unsigned long)*8 - hardware_config[i].num_bits)),0);
     }
     else
       write_device(i,0,0);
   }
   for( i=0 ; i<NUM_CHIPS ; i++ ){
     latch_mask[i] = prog_shm->prog_image[i][0];
     //          fprintf(stderr,"%3i ",(int) latch_mask[i]);
   }
   //      fprintf(stderr,"\n");



   // build default mask
   // by writing default values into event 0.
   for ( i=0 ; i<num_dev ; i++ )
     write_device(i,hardware_config[i].def_val,0);

   for( i=0 ; i<NUM_CHIPS ; i++ ){
     default_mask[i] = prog_shm->prog_image[i][0];
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

   err = stat("/usr/share/Xnmr/config/h_config.h",&other_buff);
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

    if (clkc == -1){
      //   fprintf(stderr,"in write_device() for the first time\n");
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"CLKC") == 0)
	  clkc = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"PHASEC") == 0)
	  phasec = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"IC") == 0)
	  ic = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"QC") == 0)
	  qc = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"AMPC") == 0)
	  ampc = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"_AMPC") == 0)
	  _ampc = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"CLKB") == 0)
	  clkb = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"PHASEB") == 0)
	  phaseb = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"IB") == 0)
	  ib = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"QB") == 0)
	  qb = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"AMPB") == 0)
	  ampb = i;
   }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"_AMPB") == 0)
	  _ampb = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"CLKA") == 0)
	  clka = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"PHASEA") == 0)
	  phasea = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"IA") == 0)
	  ia = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"QA") == 0)
	  qa = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"AMPA") == 0)
	  ampa = i;
      }
      for(i=0;i<num_dev;i++){
	if(strcmp(hardware_config[i].name,"_AMPA") == 0)
	  _ampa = i;
      }
      
      
      if ( (clkc == -1) || (phasec == -1) || (ic == -1)||(qc == -1)||(ampc == -1)||(_ampc == -1)){
	fprintf(stderr,"pulse_program_init: couldn't find one of my channel C devices, check h_config.h\n");
	exit(0);
      }
      if ( (clkb == -1) || (phaseb == -1) || (ib == -1)||(qb == -1)||(ampb == -1)||(_ampb == -1)){
	fprintf(stderr,"pulse_program_init: couldn't find one of my channel B devices, check h_config.h\n");
	fprintf(stderr,"%i %i %i %i %i %i \n",clkb,phaseb,ib,qb,ampb,_ampb);
	exit(0);
      }
      if ( (clka == -1) || (phasea == -1) || (ia == -1)||(qa == -1)||(ampa == -1)||(_ampa == -1)){
	fprintf(stderr,"pulse_program_init: couldn't find one of my channel A devices, check h_config.h\n");
	exit(0);
      }
      
      // now build a table to translate the rf channel devices from logical channels to real devices
      //   fprintf(stderr,"building a tran_table\n");
      if (data_shm->ch1 == 'A'){
	//     fprintf(stderr,"pulse.c found ch1 = A\n");
	tran_table[RF1-RF_OFFSET]=RFA;
	tran_table[PHASE1-RF_OFFSET]=PHASEA;
	tran_table[AMP1-RF_OFFSET]=AMPA;
	tran_table[RF1_BLNK-RF_OFFSET]=RFA_BLNK;
	grad_channel -= 1;
      }
      else if (data_shm->ch1 == 'B'){
	//     fprintf(stderr,"pulse.c found ch1 = B\n");
	tran_table[RF1-RF_OFFSET]=RFB;
	tran_table[PHASE1-RF_OFFSET]=PHASEB;
	tran_table[AMP1-RF_OFFSET]=AMPB;
	tran_table[RF1_BLNK-RF_OFFSET]=RFB_BLNK;
	grad_channel -= 2;
      }
      else if (data_shm->ch1 == 'C'){
	//     fprintf(stderr,"pulse.c found ch1 = C\n");
	tran_table[RF1-RF_OFFSET]=RFC;
	tran_table[PHASE1-RF_OFFSET]=PHASEC;
	tran_table[AMP1-RF_OFFSET]=AMPC;
	tran_table[RF1_BLNK-RF_OFFSET]=RFC_BLNK;
	grad_channel -= 4;
      }
      else {
	fprintf(stderr,"no valid channel 1 channel found!\n");
	exit(0);
      }
      if (data_shm->ch2 == 'A'){
	//     fprintf(stderr,"pulse.c found ch2 = A\n");
	tran_table[RF2-RF_OFFSET]=RFA;
	tran_table[PHASE2-RF_OFFSET]=PHASEA;
	tran_table[AMP2-RF_OFFSET]=AMPA;
	tran_table[RF2_BLNK-RF_OFFSET]=RFA_BLNK;
	grad_channel -= 1;
      }
      else if (data_shm->ch2 == 'B'){
	//     fprintf(stderr,"pulse.c found ch2 = B\n");
	tran_table[RF2-RF_OFFSET]=RFB;
	tran_table[PHASE2-RF_OFFSET]=PHASEB;
	tran_table[AMP2-RF_OFFSET]=AMPB;
	tran_table[RF2_BLNK-RF_OFFSET]=RFB_BLNK;
	grad_channel -= 2;
      }
      else if (data_shm->ch2 == 'C'){
	//     fprintf(stderr,"pulse.c found ch2 = C\n");
	tran_table[RF2-RF_OFFSET]=RFC;
	tran_table[PHASE2-RF_OFFSET]=PHASEC;
	tran_table[AMP2-RF_OFFSET]=AMPC;
	tran_table[RF2_BLNK-RF_OFFSET]=RFC_BLNK;
	grad_channel -= 4;
      }
      else {
	fprintf(stderr,"no valid channel 2 channel found!\n");
	exit(0);
      }
      //      fprintf(stderr,"grad_channel is: %i \n",grad_channel);
      if (grad_channel == 1){
	gradI = ia;
	gradQ = qa;
      }
      else if (grad_channel == 2){
	gradI = ib;
	gradQ = qb;
      }
      else if (grad_channel == 4){
	gradI = ic;
	gradQ = qc;
      }
      else {
	fprintf(stderr,"didn't find a grad channel!\n");
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


int lookup_amp(float fval)
{
  
  // eventually this will need to be some good way to linearize the amplitude

  // for now, do the simplest thing.
  int val;
  if (fval > 1.0) fval = 1.0;
  if (fval < -1.0) fval = -1.0;
  val = (int)( (fval+1.0)*1023/2.);
    //    fprintf(stderr,"lookup_amp: %f %i\n",fval,val);
  return ( val);

		   
}

int lookup_ampc(float fval)
{
  

  int val;

  if (fval > 1.0) fval = 1.0;
  if (fval < -1.0) fval = -1.0;

  // comes from fit (amp3_calib.plt and .txt)

  fval = atanh(fval*0.78)/1.26556;

  val = (int)( (fval+1.0)*511.);
  //    fprintf(stderr,"lookup_amp3: %f %i\n",fval,val);
  return ( val);

		   
}

int lookup_ampb(float fval)
{
  

  int val;

  if (fval > 1.0) fval = 1.0;
  if (fval < -1.0) fval = -1.0;

  // comes from fit (amp3_calib.plt and .txt)

  fval = atanh(fval*0.72)/0.967688;

  val = (int)( (fval+1.0)*511.);
  //    fprintf(stderr,"lookup_amp3: %f %i\n",fval,val);
  return ( val);

		   
}

int lookup_ampa(float fval)
{
  

  int val;

  if (fval > 1.0) fval = 1.0;
  if (fval < -1.0) fval = -1.0;

  // comes from fit (amp3_calib.plt and .txt)

  fval = atanh(fval*0.72)/0.967688;

  val = (int)( (fval+1.0)*511.);
  //    fprintf(stderr,"lookup_amp3: %f %i\n",fval,val);
  return ( val);

		   
}

void lookup_phase(float fval,int *ival,int *qval)
{
  // eventually, this will need to do a good job.
  // for now, do the simplest thing.

  //  fprintf(stderr,"lookup phase: got fval: %f\n",fval);
  *ival = (int) ((cos(fval*M_PI/180.)+1.0)*511.51);
  *qval = (int) ((sin(fval*M_PI/180.)+1.0)*511.51);
  //    fprintf(stderr,"lookup phase: ival %i, qval: %i\n",*ival,*qval);


}


void lookup_phasec(float fval,int *ival,int *qval)
{
  // eventually, this will need to do a good job.
  // for now, do the simplest thing.
  static char first_time=1,table_valid=0;
  static float table[361];
  FILE *infile;
  int i,ivf;
  float f,frac;

  if (first_time ==1){
    first_time=0;
    infile=fopen("/usr/src/Xnmr/current/correct_phasec.txt","r");
    if (infile !=NULL){
      for(i=0;i<360;i++)
	fscanf(infile,"%i %f",&i,&table[i]);
      table_valid=1;
    }
    else fprintf(stderr,"couldn't open correct_phasec.txt\n");
  }
  table[360]=360.;
  
  if (table_valid == 1){
    if (fval < 0.)
       fval -= 360. * ((int) (fval/360)-1);

    frac = fval - floor(fval);
    ivf = (int) floor(fval) % 360;
    
    f = frac * (table[ivf+1]-table[ivf])+table[ivf];
    //  fprintf(stderr,"for %f, frac: %f, ivf: %i using: %f\n",fval,frac,ivf,f);
  }
  else (f=fval);

  *ival = (int) ((cos(f * M_PI/180.)+1.0)*511.51);
  *qval = (int) ((sin(f * M_PI/180.)+1.0)*511.51);


}



void lookup_phaseb(float fval,int *ival,int *qval)
{
  // eventually, this will need to do a good job.
  // for now, do the simplest thing.
  static char first_time=1,table_valid=0;
  static float table[361];
  FILE *infile;
  int i,ivf;
  float f,frac;

  if (first_time ==1){
    first_time=0;
    infile=fopen("/usr/src/Xnmr/current/correct_phaseb.txt","r");
    if (infile !=NULL){
      for(i=0;i<360;i++)
	fscanf(infile,"%i %f",&i,&table[i]);
      table_valid=1;
    }
    else fprintf(stderr,"couldn't open correct_phaseb.txt\n");
  }
  table[360]=360.;
  
  if (table_valid == 1){
    if (fval < 0.)
       fval -= 360. * ((int) (fval/360)-1);

    frac = fval - floor(fval);
    ivf = (int) floor(fval) % 360;
    
    f = frac * (table[ivf+1]-table[ivf])+table[ivf];
    //  fprintf(stderr,"for %f, frac: %f, ivf: %i using: %f\n",fval,frac,ivf,f);
  }
  else (f=fval);

  *ival = (int) ((cos(f * M_PI/180.)+1.0)*511.51);
  *qval = (int) ((sin(f * M_PI/180.)+1.0)*511.51);


}



void lookup_phasea(float fval,int *ival,int *qval)
{
  // eventually, this will need to do a good job.
  // for now, do the simplest thing.
  static char first_time=1,table_valid=0;
  static float table[361];
  FILE *infile;
  int i,ivf;
  float f,frac;

  if (first_time ==1){
    first_time=0;
    infile=fopen("/usr/src/Xnmr/current/correct_phasea.txt","r");
    if (infile !=NULL){
      for(i=0;i<360;i++)
	fscanf(infile,"%i %f",&i,&table[i]);
      table_valid=1;
    }
    else fprintf(stderr,"couldn't open correct_phasea.txt\n");
  }
  table[360]=360.;
  
  if (table_valid == 1){

    if (fval < 0.)
       fval -= 360. * ((int) (fval/360)-1);

    frac = fval - floor(fval);
    ivf = (int) floor(fval) % 360;
    
    f = frac * (table[ivf+1]-table[ivf])+table[ivf];
    //     fprintf(stderr,"for %f, frac: %f, ivf: %i using: %f\n",fval,frac,ivf,f);
  }
  else (f=fval);

  *ival = (int) ((cos(f * M_PI/180.)+1.0)*511.51);
  *qval = (int) ((sin(f * M_PI/180.)+1.0)*511.51);


}




int shift_events(int start){
  int i,j;
  unsigned char chip,bit;
  
  
  if (prog_shm->no_events >= MAX_EVENTS-2){
    fprintf(stderr,"shift_events: MAX number of events exceeded\n");
    prog_shm->event_error = 1;
    return -1;
  }
  
  // used to start at no-events-1, but no_events might be in progress...
  for (i = prog_shm->no_events; i >= start ; i--){
    for (j=0;j<NUM_CHIPS;j++)
      prog_shm->prog_image[j][i+1] = prog_shm->prog_image[j][i];
  }
  prog_shm->no_events++;
  
  
  // if the event we split had a ppo in it, deal...
  chip = (  hardware_config[ PP_OVER ].start_bit ) /8; 
  bit = ( hardware_config[ PP_OVER ].start_bit ) %8; 
  if (((prog_shm->prog_image[chip][start] >> bit) & 1) == 1) {
    fprintf(stderr,"shift_events:  found a ppo, putting only in last\n");
    write_device(PP_OVER,0, start);
  }
  
  // if the event we split had an XTRIG in it, deal...
  chip = (  hardware_config[ XTRN_TRIG ].start_bit ) /8; 
  bit = ( hardware_config[ XTRN_TRIG ].start_bit ) %8; 
  if (((prog_shm->prog_image[chip][start] >> bit) & 1) == 1) {
    fprintf(stderr,"shift_events:  found an XTRN_TRIG, putting only in first\n");
    write_device(XTRN_TRIG,0, start+1);
  }
  
  
  return 0;
}


// go_back split up, so it now is a wrapper for this.
// the reason is we want to be able to call this with a va_list
// as argument instead of ...
// why? because set_gradient has ... and calls this.
 int ego_back( float time, float duration, unsigned char num, va_list args ) 

 { 
   static int first_time=1;
   unsigned int intval=0;
   int i,event_no,start_event=-1,end_event=-1; 
   int device_id; 
   long long int so_far,counter,c1,c2;   
   long long int counts,count_dur;
   int temp_end_event,temp_device_id,temp_start_event;
   float fval=0.;
   int is_amp_phase;


   // this routine is to insert an event of arbitrary length into a pulse sequence - but it does it
   // at a position time before the present.

   if (prog_shm->begun == 0 ){
     fprintf(stderr,"problem in pulse program.  Got a go_back before begin()\n");
     prog_shm->event_error = 1;
     return -1;
   }


   counts = (long long int) ( ( time * (float) CLOCK_SPEED ) + 0.5 );  // yes + not - !!
   count_dur = (long long int) ( ( duration * (float) CLOCK_SPEED) + 0.5);


   if( time <= 0. || counts < 0) {
     fprintf(stderr,"go_back: time <=  0, ignored: %f\n",time);
     prog_shm->event_error = 1;
     return -1; 
   }

   if (prog_shm->no_events == 0){
     fprintf(stderr,"go_back: no events yet!! ignored\n");
     prog_shm->event_error = 1;
     return -1;
   }

   

   if (count_dur <= 0 || duration <= 0. ) count_dur = 1; // gotta stick in at least one event!
   if (count_dur > MAX_CLOCK_CYCLES){
     fprintf(stderr,"go_back: got a time longer than a single event.\n");
     fprintf(stderr,"This is not supported\n");
     prog_shm->event_error = 1;
     return -1;
   }

   //   fprintf(stderr,"in go_back: time %f, dur: %f\n",time,duration);



   // figure out where to insert our event.
   event_no = -5;
   so_far = 0;
   for (i = prog_shm->no_events-1 ; i >= 0 ; i--){
     so_far += TIME_OF(i) + 1;
     if (so_far >= counts){
       //       fprintf(stderr,"go_back: event %i\n",i);
       event_no = i;
       i=-5;
     }
   }
   if( i == -1 ){ // then the pulse program is shorter than the time specified.
     //     fprintf(stderr,"ok, go_back takes us to the first event\n");
     if (first_time == 1){
       fprintf(stderr,"go_back: increasing pulseprog length by: %12.9f s\n",(float) (counts-so_far)/(float) CLOCK_SPEED);
       //       fprintf(stderr,"so_far is: %lli\n",so_far);
       first_time = 0;
     }
     shift_events(0);  // move all events from 0 on down to 1 on down.
     //     fprintf(stderr,"go_back: shifting from event 0\n");
     event_no = 0;

     start_event = 0;

     //     set defaults into first event
     for (i = 0 ;i<num_dev;i++) 
       write_device(i,hardware_config[i].def_val,0);


     if (counts > MAX_CLOCK_CYCLES){
       fprintf(stderr,"what??? go_back: counts > MAX_CLOCK_CYCLES\n");
       prog_shm->event_error = 1;
     }

     if (counts-so_far > 0){ // this is pointless - counts must always be >= 0 here?
       set_timer(0,counts-so_far-1);
       //       fprintf(stderr,"first event, counts: %lli\n",counts);
     }
     else fprintf(stderr,"go_back: problem in timing on prepending pulse program\n");

   }
   else{ // our event goes sometime into event # event_no
     // two cases, either our event is at the start, or not at the start of an event.

     //     fprintf(stderr,"so_far: %lli, counts: %lli\n",so_far,counts);

     if (so_far == counts){ // its at the very beginning
       start_event = event_no;
       //       fprintf(stderr,"go_back: back to start of event: %i\n",start_event);
     }
     else { // have to split this event
       counter = TIME_OF(event_no) + 1;
       shift_events(event_no);
       c1 = (so_far - counts - 1 );
       set_timer(event_no,c1);
       c2 = (counter - c1 - 2);
       set_timer(event_no+1,c2);

       event_no += 1;
       start_event = event_no;
     }
   }// the start of our event should now be dealt with.
     // now write the requested devices into event_no


   // now look for the end of the event


   if (counts == count_dur){ // short circuit this case
     end_event = prog_shm->no_events-1;
     //     fprintf(stderr,"go_back: end of duration is at the present\n");
   }
   else if ( count_dur > counts ){ // this event extends into the future...
     fprintf(stderr,"go_back:  event extends into future.  Extending program by: %lf\n",(float)(count_dur-counts)/CLOCK_SPEED);

     //duplicate the last event and apply latch mask

     if( prog_shm->no_events >= MAX_EVENTS-1 ) { 
       fprintf(stderr, "pprog: Maximum number of events exceeded\n" ); 
       prog_shm->event_error = 1;
       return -1; 
     } 
     else{
       if( prog_shm->no_events > 0 ) { 
	 for( i=0; i<NUM_CHIPS; i++ ) { 
	   prog_shm->prog_image[ i ][ prog_shm->no_events ] =  
	     (prog_shm->prog_image[ i ][prog_shm->no_events-1] & latch_mask[i]) + 
	     (default_mask[i] & ~latch_mask[i]);
	 }
       }
       else // just put in defaults:
	 for(i=0;i<NUM_CHIPS; i++)
	   prog_shm->prog_image[i][prog_shm->no_events] = default_mask[i];
     }
     
     
     c1 = count_dur-counts-1;
     set_timer(prog_shm->no_events,c1);
     
     end_event = prog_shm->no_events;
     prog_shm->no_events++;
   }
   else{  // have to do the dirty work of finding where to split an event.
     event_no = -5;
     so_far = 0;
     for (i = prog_shm->no_events-1 ; i >= 0 ; i--){
       so_far += TIME_OF(i) + 1;
       if (so_far >= (counts-count_dur)){
	 //       fprintf(stderr,"go_back: event %i\n",i);
	 event_no = i;
	 i=-5;
       }
     }
     if( i == -1 ){ // then the pulse program is shorter than the time specified.
       fprintf(stderr,"go_back: this should absolutely never happen\n");
     }
     counter = TIME_OF(event_no) + 1;
     //     fprintf(stderr,"go_back: so_far - counter: %lli (counts-count_dur) %lli\n",so_far-counter,(counts-count_dur));
     if ( so_far == (counts - count_dur)){ //end is matched
       end_event = event_no-1;
       fprintf(stderr,"go_back: end_event set matches at event: %i\n",end_event);
	 }
     else {// gotta do the split:
       shift_events(event_no);
       //       fprintf(stderr,"go_back: shifting events at end\n");
       
       c1 = (so_far - (counts-count_dur) - 1 );
       set_timer(event_no,c1);
       c2 = (counter - c1 - 2);
       set_timer(event_no+1,c2);
       
       end_event = event_no;
       event_no += 1;
     }
   }
 
   // that should do it!

   
   //set all the specified device information 

   for( i=0; i<num; i++ ) { 
     device_id = (unsigned char) va_arg( args, int  ); 
     //     fprintf(stderr,"got device_id: %i\n",(int) device_id);


     // look to see if this device latches.  If so, write it to the end...
     // all the float devices latch too, taken care of below.
     if (device_id >=RF_OFFSET) temp_device_id = tran_table[device_id-RF_OFFSET];
     else temp_device_id = device_id;
     if ( hardware_config[temp_device_id].latch == 1) temp_end_event = prog_shm->no_events - 1;
     else temp_end_event=end_event;

     // look to see if this device is a PP_OVER
     temp_start_event=start_event;
     if (device_id == PP_OVER){
       fprintf(stderr,"in go_back, got a PP_OVER.  what are you thinking?\n");
       temp_start_event = end_event;
     }
     if(device_id == XTRN_TRIG){
       fprintf(stderr,"in go_back, got an XTRN_TRIG\n");
       temp_end_event = start_event;
     }

     if (is_a_float_device(device_id) >0 ){
       temp_end_event = prog_shm->no_events -1; // so that the float devices latch.
       fval = (float) va_arg(args,double);
     }
     else
       intval =  va_arg(args,unsigned int);
     //     fprintf(stderr,"go_back: writing device_wrap with events: %i, %i\n",temp_start_event,temp_end_event);
     is_amp_phase = write_device_wrap(temp_start_event,temp_end_event,device_id,intval,fval);
     
     // now need to deal with all the special cases...
     if (is_amp_phase > 0) deal_with_previous(start_event,is_amp_phase,device_id,intval,fval);


   }


   // what about if we have a ppo or an XTRN_TRIG in there?  
   // shift_events looks for these when we're splitting events up
   // we look for them here as well. should be ok.  Not well tested.


   //   va_end(args); 

   return 0; 
   
 }

int go_back(float time, float duration, unsigned char num, ...){
   va_list args; 
   int rval=0;
   va_start( args, num );

   rval = ego_back(time,duration,num,args);
   va_end(args);
   return rval;

}

void deal_with_previous(int event_no,int is_amp_phase,int device_id, int intval,float fval) {
  //  need to look back at the clock value of the previous two events.
  // there are seven possibilities: 00 11 01 _1 _0 10 __ 
  // 00, 11, 01, _1, _0 are all fine.  10 needs have the 0 event split, 
  // and __ needs to have a new event added
  // _ means the event doesn't exist (?)

  unsigned char chip,bit;
  int temp_device_id=-1 ;
  long long counter;
  
  /* this is a routine to deal with setting up the previous event for an amplitude or phase 
     change.  There needs to be some prep - the amp + phase values themselves must be 
     preloaded, and the clock toggled high.  */


  if (is_amp_phase & 1 ) temp_device_id = clka;
  if (is_amp_phase & 2 ) temp_device_id = clkb;
  if (is_amp_phase & 4 ) temp_device_id = clkc;
  if (temp_device_id == -1){
    fprintf(stderr,"deal_with_previous, is_amp_phase not 1 2 or 4\n");
  }
  
  chip = (  hardware_config[ temp_device_id ].start_bit ) /8; 
  bit = ( hardware_config[ temp_device_id ].start_bit ) %8; 
  
  //  fprintf(stderr,"deal_with_previous: chip %i, bit: %i\n",chip,bit);

  // look for first event first
  if (event_no == 0){ // well, we're the first.  split it up.
    shift_events(0); // new first event = same as old first event
    set_timer(0,0); 
    return;
  } // next is all three cases of 1 in the previous clock event, 
  // also if there's only one previous event

  else if (((prog_shm->prog_image[chip][event_no-1] >> bit) & 1) == 1 || event_no == 1) {

    //    fprintf(stderr,"found a 1 in the clock of previous event, or on second event \n");
    
    // all we do here is write the event into the previous event, and then
    // hit the clock as a one again (since write_device wrap will unhit it.)
    
    write_device_wrap(event_no-1,event_no-1, device_id,intval,fval);
    write_device(temp_device_id,1,event_no-1);
    return;
  } // five down, two to go...
  else if (((prog_shm->prog_image[chip][event_no-2] >> bit) & 1) ==  0 ){
    //    fprintf(stderr,"found 00 in previous clock bits\n"); // exact same as above
    write_device_wrap(event_no-1,event_no-1, device_id,intval,fval);
    write_device(temp_device_id,1,event_no-1);
    return;
  }
  else if ( ((prog_shm->prog_image[chip][event_no-2] >> bit) & 1) ==  1){
    //    fprintf(stderr,"got a one two bits back\n");
    // need to split/extend event_no-1

    counter = TIME_OF(event_no-1);
    if (counter > 0) counter--;
    else fprintf(stderr,"splitting an event of a single count - its length will now be two counts\n");
    shift_events(event_no-1);

    // writes the event into the second piece of the previous
    write_device_wrap(event_no,event_no, device_id,intval,fval);
    set_timer(event_no-1,counter);
    set_timer(event_no,0);

    // hit the clk on the 2nd piece of the split
    write_device(temp_device_id,1,event_no);
  }
  else fprintf(stderr,"deal_with_previous: unknown situation???\n");
}




void store_position(){
  if (currently_going_back != 0){
    fprintf(stderr,"incorrect usage of store_position - are you trying to nest them?\n");
    return;
  }
  position_stored = 1;
  how_far_back = 0;
  
}


void jump_to_stored(){
  if (currently_going_back !=0 || position_stored == 0){
    fprintf(stderr,"incorrect usage of jump_to_stored_position\n");
    return;
  }
  currently_going_back = 1;

}


void return_to_present(){
  if (currently_going_back !=1 || position_stored != 1){
    fprintf(stderr,"incorrect usage of return_to_present\n");
    return;
  }

  //  fprintf(stderr,"return to present: got up to: %.7f before present\n",(float) how_far_back/CLOCK_SPEED);
  currently_going_back = 0;
  position_stored=0;
}


void jump_back_by(double time){

  if (currently_going_back != 0 || position_stored != 0){
    fprintf(stderr,"already going back or position stored, can't jump back again\n");
    return;
  }
  how_far_back = time*CLOCK_SPEED;
  currently_going_back = 1;
  position_stored=1;

}

void pprog_is_noisy(){
  
  prog_shm->is_noisy = 1;
  prog_shm->noisy_start_pos=0;
  //  fprintf(stderr,"just set is_noisy to true\n");
}



void start_noisy_loop(){
  prog_shm->noisy_start_pos = prog_shm->no_events;
  //  fprintf(stderr,"just set noisy_start_pos to %i\n",prog_shm->no_events);

}

// this is for testing of hardware-assisted looping
unsigned int get_event_no(){
  return prog_shm->no_events;
}

int init_bgu(float alpha, float beta, float gamma){
  // alpha, beta and gamma will be euler angles to transform all gradients!
  bgu_alpha = alpha*M_PI/180.;
  bgu_beta = beta*M_PI/180.;
  bgu_gamma = gamma*M_PI/180.;
  if (gradI == -1 || gradQ == -1){
     prog_shm->event_error = 1;
     fprintf(stderr,"init_bgu: Couldn't locate a gradient device?\n");
     return -1;
  }
  bgu_inited=1;
  bgu_cleared = 0;
  // ok, bgu seems to exist. Let's write the preload registers with all 0's

  return write_bgu_raw(0.,0.,0.,1);
}

void clear_bgu(){
  write_bgu_raw(0.,0.,0.,1);
  event(250e-9,0);
  event(150e-9,1,GRSTROBES,0x1);
  bgu_cleared = 1;
  
}
  
unsigned int cache_dev(unsigned int device,unsigned int event_no){
  unsigned int val,i;
  unsigned char *mask_c,*val_c;
  val= 0;
  val_c = (unsigned char *) &val;
  mask_c = (unsigned char *) &hardware_config[device].load_mask;
  //  printf("start chip: %i, end chip: %i\n",hardware_config[device].start_chip_num,hardware_config[device].end_chip_num);
  for (i=hardware_config[device].start_chip_num;i<=hardware_config[device].end_chip_num;i++){
    //    printf("device: %i, load_mask: %i\n",device,mask_c[i-hardware_config[device].start_chip_num]);
    val_c[i-hardware_config[device].start_chip_num] |= prog_shm->prog_image[i][event_no] & mask_c[i-hardware_config[device].start_chip_num];
  }
  //  printf("before bit shifting, val is: 0x%4x\n",val);
  val = val >> hardware_config[device].chip_bit;
  return val;
  
}


int write_bgu_raw(float x, float y, float z, int b0_flag){
  // give it x, y, z gradient values as percentages.
  // if bo_flag is set to 1, then the b0 value will also be written  - used on init.

  // set_gradients calls this. It uses the duration of these events (totals 1us
  // when b0_flag = 0) internally, so if these durations change, set_gradients
  // needs to be updated.
  
  //  strobes are NGI, DAS, GDTR, WRS from lowest to highest.
  double eventtime;
  unsigned int iword,qword,strobes;
  unsigned int ia,qa,ib,qb,ic,qc; // values to cache
  int dacval;
  //  printf("in write_bgu_raw with vals: %f %f %f %i\n",x,y,z,b0_flag);
  if (bgu_inited  != 1){
     prog_shm->event_error = 1;
     fprintf(stderr,"write_bgu_raw: called but bgu not inited?\n");
     return -1;
  }

  // ok, so here we have to pack the bits into the awkward rf transmitter-organized words

  // first event will set address and data bits, leave all strobes high
  // second event will lower strobes
  // third event raises strobes and sets data and address for next 
  if (x > 1. || x < -1. || y > 1. || y< -1. ||  z > 1. || z < -1.){
     prog_shm->event_error = 1;
     fprintf(stderr,"write_bgu_raw: got a gradient out of range?\n");
     return -1;
  }

  if (b0_flag == 1) eventtime = 500e-9;
  else eventtime = 150e-9;
  
  // cache I and Q values
 
  if (prog_shm->no_events == 0 ) {  // XX todo, set ia, qa etc to default values.
    ia = hardware_config[IA].def_val;
    qa = hardware_config[QA].def_val;
    ib = hardware_config[IB].def_val;
    qb = hardware_config[QB].def_val;
    ic = hardware_config[IC].def_val;
    qc = hardware_config[QC].def_val;
    printf("in write_bgu_raw, no previous events, caching defaults\n");
  }
  else{
    ia = cache_dev(IA,prog_shm->no_events-1);
    qa = cache_dev(QA,prog_shm->no_events-1);
    ib = cache_dev(IB,prog_shm->no_events-1);
    qb = cache_dev(QB,prog_shm->no_events-1);
    ic = cache_dev(IC,prog_shm->no_events-1);
    qc = cache_dev(QC,prog_shm->no_events-1);
  }
  //  printf("write_bgu_raw caching values: %i %i %i %i %i %i\n",ia,qa,ib,qb,ic,qc);

  // could just do this, but to be safer, we'll write our I and Q to all three hardware channels
  //  event (150e-9,3,gradI,~iword, gradQ,~qword,GRSTROBES,~strobes);
  
  dacval = (int) (x*32767);
  qword  = (dacval >> 6) & 0x3FF ; // top 10 bits of the dacval
  iword = (dacval << 4) & 0x3F0; // bottom 6 bits of dacval into top 6 bits of iword
  iword |= 0 ;// for X gradient address
  strobes = 0xF; // NGI, DAS, GDTR and WRS all high
  event(eventtime,7,IA,~iword, QA,~qword,IB,~iword,QB,~qword,IC,~iword,QC,~qword,GRSTROBES,~strobes);
  strobes &= ~0xA; // lower the two strobes
  event(eventtime,7,IA,~iword, QA,~qword,IB,~iword,QB,~qword,IC,~iword,QC,~qword,GRSTROBES,~strobes);
  strobes |= 0xA; // raise the two strobes
  event(eventtime,7,IA,~iword, QA,~qword,IB,~iword,QB,~qword,IC,~iword,QC,~qword,GRSTROBES,~strobes);

  dacval = (int) (y*32767);
  qword  = (dacval >> 6) & 0x3FF ; // top 10 bits of the dacval
  iword = (dacval << 4) & 0x3F0; // bottom 6 bits of dacval into top 6 bits of iword
  iword |= 1 ;// for Y gradient address
  strobes = 0xF; // NGI, DAS, GDTR and WRS all high
  //  printf("got yval: %f, qword: 0x%04x, iword: 0x%04x,dacval %i 0x%04x\n",y,qword,iword,dacval,dacval);
  event(eventtime,7,IA,~iword, QA,~qword,IB,~iword,QB,~qword,IC,~iword,QC,~qword,GRSTROBES,~strobes);
  strobes &= ~0xA; // lower the two strobes
  event(eventtime,7,IA,~iword, QA,~qword,IB,~iword,QB,~qword,IC,~iword,QC,~qword,GRSTROBES,~strobes);
  strobes |= 0xA; // raise the two strobes
  event(eventtime,7,IA,~iword, QA,~qword,IB,~iword,QB,~qword,IC,~iword,QC,~qword,GRSTROBES,~strobes);

  dacval = (int) (z*32767);
  qword  = (dacval >> 6) & 0x3FF ; // top 10 bits of the dacval
  iword = (dacval << 4) & 0x3F0; // bottom 6 bits of dacval into top 6 bits of iword
  iword |= 2 ;// for Z gradient address
  strobes = 0xF; // NGI, DAS, GDTR and WRS all high
  //  printf("got zval: %f, qword: 0x%04x, iword: 0x%04x,dacval %i, 0x%04x\n",z,qword,iword,dacval,dacval);
  event(eventtime,7,IA,~iword, QA,~qword,IB,~iword,QB,~qword,IC,~iword,QC,~qword,GRSTROBES,~strobes);
  strobes &= ~0xA; // lower the two strobes
  event (eventtime,7,IA,~iword, QA,~qword,IB,~iword,QB,~qword,IC,~iword,QC,~qword,GRSTROBES,~strobes);
  strobes |= 0xA; // raise the two strobes
  event(eventtime,7,IA,~iword, QA,~qword,IB,~iword,QB,~qword,IC,~iword,QC,~qword,GRSTROBES,~strobes);

  if (b0_flag){
    dacval = 0;
    qword  = (dacval >> 6) & 0x3FF ; // top 10 bits of the dacval
    iword = (dacval << 4) & 0x3F0; // bottom 6 bits of dacval into top 6 bits of iword
    iword |= 0xF ;// for Z gradient address
    strobes = 0xF; // NGI, DAS, GDTR and WRS all high
    event(eventtime,7,IA,~iword, QA,~qword,IB,~iword,QB,~qword,IC,~iword,QC,~qword,GRSTROBES,~strobes);
    strobes &= ~0xA; // lower the two strobes
    event(eventtime,7,IA,~iword, QA,~qword,IB,~iword,QB,~qword,IC,~iword,QC,~qword,GRSTROBES,~strobes);
    strobes |= 0xA; // raise the two strobes
    event(eventtime,7,IA,~iword, QA,~qword,IB,~iword,QB,~qword,IC,~iword,QC,~qword,GRSTROBES,~strobes);
  }
  // finally, set strobes high and restore cached values.
  strobes = 0xF; // all strobes high, 
  event (150e-9,7,IA,ia, QA,qa,IB,ib,QB,qb,IC,ic,QC,qc,GRSTROBES,~strobes);

  // for the device that actually should be the gradients, go back and set their values to all high:

  go_back(150e-9,100e-9,2,gradI,0,gradQ,0);
  // total duration is 1us or 2.5us -> 1.5us here (if b0=0)
  // but didn't hit NGI
  return 1;
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
  va_list args,args1;
  int i,intval,device_id;
  //  float fval;
  float mat[3][3];
  //  transform from x,y,z to nx, ny, nz with angles alpha,beta,gamma
  //  printf("set_gradients: requested gradients: %f %f %f\n",x,y,z);

  if (bgu_cleared == 1){
    prog_shm->event_error = 1;
     fprintf(stderr,"set_gradients: BGU has already been cleared\n");
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

  if (nx == 0 && ny == 0 && nz == 0)
    bgu_last_vals_zeros = 1;
  else
    bgu_last_vals_zeros = 0;
  write_bgu_raw(nx*bgu_xscale,ny*bgu_yscale,nz*bgu_zscale,0);

  event(100e-9,1,GRSTROBES,0x1); // hit NGI low (gets inverted when downloading)
  event(100e-9,0);

  if (num > 0){ // there are other devices requested:
    va_start(args,num);
    va_copy(args1,args);

    // check that none of the devices are float devices:
    for( i=0; i<num; i++ ) { 
      device_id = (unsigned char) va_arg( args, int  ); 
      //      fprintf(stderr,"got device_id: %i for event: %i\n",(int) device_id,prog_shm->no_events);
      if (is_a_float_device(device_id) >0 ){
	//fval = (float) va_arg(args,double);
	prog_shm->event_error = 1;
	va_end(args);
	va_end(args1);
	fprintf(stderr,"set_gradients ERROR: requested an amplitude or phase during a set_gradients event, not doing it.\n");
	return 0;
      }
      else{
	intval =  va_arg(args,unsigned int);
      }
    }
    // this is a stupid nothing to suppress a compile warning of an unused variable...
    if (intval) intval +=1;
    // ok, do all the other devices:    
    ego_back(1.7e-6,1.7e-6,num,args1); // event time was 150ns * 10 + two 100ns events in here.
    va_end(args);
    va_end(args1);
  }
  return 1;
}

