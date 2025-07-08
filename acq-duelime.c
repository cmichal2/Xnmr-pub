/* TODO:

- make sure we bypass interp after any calibration func.
  - CGEN locking:
  - will need to split up start_lime into: prep_lime and start_lime so start_lime doesn't
  do anything that will invoke a calibration routine. 

  - Will  need to keep track so 
  if a calibration routine does get triggered, we'll need to relock the lime CGENs.
  ick. 

   - make sure number of points in a dwell works out right.
   - fix calculation of sw/dwell, and including pulse-pb in param_f.c
   - test on-fly frequency offsets
   - test frequency granularity.

   - sort out rcvr_gate, blnk, etc channel assignments. - GATE1, GATE2 etc
   - expose write to dacs and write to alt port in the pulse programming language.


   - look through MSL200 defines and rationalize for DUELIME
   - do a better job of collapsing vs hinting. Right now we shut off collapsing during hinted loops.

   - need to think some more about end of tx_thread_func.
   The rules seem to be: either keep stamp #'s consecutive, or allow a few ms (?) after
   the fifo is empty before writing more.
   Need to leave rx running till TX fifo is empty, but should then turn rx stream off first.
  
   - better filtering on LIME


   - could pulse progs add to data_image itself?
   - test offset freqs - especially negatives?
   - clean up error messages?
   - can we change gains while stream running? At lower level than set_gain?
   - WAIT and WAIT_MAX will lose sync with the limesdr! tricky, but maybe not impossible to fix.
   TESTING TESTING TESTING!


   Done:
---------
   - test multiple boards
   - figure out crash/failure with odd number in packet start.
      and ensure it doesn't happen later.
   - num_calced for last scan for noisy mode? does it make any sense?
   - look through all of /dev/tty/ACMx and figure out what's what
   - bgu thread priority
   - hinted loops for gradient program and tx programs too?
   - clean up label resolution - remove triplication where possible.
   - Sync properly on Lime, and finish the lime sync code in here. 
   - FPGA for sync!
   - maybe get rid of libusb signal errors if put open and init into separate thread? Nope.
   - fix ppo_time so it finds exit event
   - need start-up program for due, and syncing code.
   - figure out why noisy+NO_SEND_ZEROS gives L ??? Well, made it go away anyway. This might have been
   a thread synchronization problem.
   - clean up  rx function to consolidate the duplication.
   - stress test cpu - ensure we error and exit cleanly if overloaded. Kinda done.
   - make sure txprog and due prog have EXIT events that are reachable. tx gets this in pp_time_new
   - need to clean up start_lime and have it complain if gains or freqs change while running 
   - pcos and psin for noisy are very expensive to calculate use icos and isin? Nope doesn't seem to change cpu load noticably
   - maybe moving lime calib and start-up stuff into tx thread would get rid of libusb signal errors? NOPE
   - figure out how to run lime from 10MHz external clock.
   - find memory corruption on kill?
   - why do we get too late  in onep-lime? - and if ppo gets longer it hangs!
Something out of sync with due and limes?
   - deal with signals on sem_wait in start_lime
   - make sure tx ends with 0's on abort??? - should maybe destroy streams and disable channels?
   - on exit from run, should hit the txsems one last time with a clean-up flag sent to send off the last tx packet?
   - fix phase calcs in pulse-duelime so that values outside of 0-360 work.
   - set priorities right: acq highest, then tx, then rx?
   - fix check of how many points acquired so it works for noisy too - move it into acq - do something sensible
   if npts doesn't agree with points that program will produce - should error if it will make too many,
   and only request the smaller amount if it will produce less. Think this is ok now.
   - in event_duepp don't let loops be 0
   - don't calculate pulse program if not needed in noisy mode? (related to loops = 0)
   - serious errors in tx or rx threads should error out of acq!

*/


// do this till limes are well tested...
//#define NO_RT_SCHED


/*  acq.c
 *
 * Xnmr Software Project
 * * Main controlling process for NMR experiments
 * UBC Physics
 *
 * This version uses a message queue for communication between the acq and pprog
 * processes and implements all error checking scenarios except timeouts. - later included  
 * It is also the first reliable version.
 *
 * Mar 2: Made this process a "real time" priority for the scheduler.  Note that it
 *        if two signals are sent to the ui in the same cycle 
 *        without a pause in between them, only the second signal will be received
 *        because the second signal meaning will overwrite the first before the first
 *        signal has been processed by the UI.  Perhaps a future versions should use
 *        message queues for this communication as well.
 *
 * written by: Scott Nelson, Carl Michal
 *
 * April, 2000
 *
 * March 2001, updates for using AD6640, AD6620 digitizer and dsp CM
 *
 * July 2001 Block averaging added CM
 *
 *
 *
 *
 */
#include "platform.h"
#include <math.h>
#include <signal.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <stdio.h>
#include <errno.h>
#include <sys/msg.h>
#include <sys/mman.h>  //for mlockall, etc.
#include <sys/stat.h>  //for mkdir, etc
#include <sched.h>
#include <string.h>
#include <glib.h>
#include <syslog.h> // for system logger.
#include <sys/io.h>
#include <sys/resource.h> // for setrlimit for memlock
#include "due-pp-lib.h"
#include "shm_data.h"
#include "shm_prog-duelime.h"          //indirectly dependant on h_config.h
#include "p_signals.h"
#include "acq-duelime.h"
#include "/usr/share/Xnmr/config/pulse_hardware-duepp.h"    //indirectly dependant on h_config.h
#include "param_utils.h"
#include "nr.h"
#include <fftw3.h>
#include "lime-duepp.h"
#include "duepp.h"
#include "due_bgu_interface.h"
void pulse_hardware_build_startup_lime(due_prog_t *due_prog,int num_boards);

/*
 *  Global variables
 */

// uncomment to enable hardware:
#define LIMESDR
#define HDUEPP
#define DUEBGU

struct data_shm_t* data_shm;
struct prog_shm_t* prog_shm;        //These are shared memory structures and must be initialized

due_prog_t start_up_prog[NUM_BOARDS]; // holds the start-up sync program for the dues.

int data_shm_id;
int prog_shm_id;
int msgq_id;
int errcode = 0;
volatile int running = 0;
char msg_wait_flag = 0;

volatile char done = NOT_DONE; 
int euid,egid;
int bgufd;



void cant_open_file(){
  int result;

  done = ERROR_DONE;
  prog_shm->stop_lime = ERROR_DONE;

  result = seteuid(euid);
  if (result != 0) perror("acq:");
  result = setegid(egid);
  if (result != 0) perror("acq:");
  send_sig_ui( ACQ_FILE_ERROR );
  running = 0;

}

int init_msgs()

{

  struct msgbuf message;
  int i = 0;

  msgq_id = msgget( MSG_KEY, IPC_CREAT|0660 );     

  if( msgq_id < 0 ){
    tell_xnmr_fail();
    shut_down();
  }


  // empty the message queue

  while( msgrcv ( msgq_id, &message, 1, 0, IPC_NOWAIT ) >= 0 )
    i++;

  if( i> 0 ) {
    // fprintf(stderr, "acq removed %d messages from the message queue\n", i );
  }

  return msgq_id;
}

int init_shm()
{

  int existed = 0; // stores if prog shared mem already existed
  struct rlimit my_lim;

  data_shm = 0;  prog_shm = 0;
  data_shm_id = shmget( DATA_SHM_KEY, sizeof( struct data_shm_t ),0);

  if (data_shm_id < 0 ) perror("acq: error getting data shm");
  fprintf(stderr,"acq: data_shm_id: %i\n",data_shm_id);

  prog_shm_id = shmget( PROG_SHM_KEY, sizeof( struct prog_shm_t ), IPC_CREAT|IPC_EXCL|0660); // fails if it already existed
  if (prog_shm_id  < 0) {  // call failed for some reason
    existed = 1;
    prog_shm_id = shmget( PROG_SHM_KEY, sizeof( struct prog_shm_t ), IPC_CREAT|0660);
  }
  else{
    shmctl(prog_shm_id,SHM_LOCK,NULL);
 }
    fprintf(stderr,"data_shm_id: %i, prog_shm_id: %i\n",data_shm_id,prog_shm_id);

  if( data_shm_id < 0 || prog_shm_id < 0 ) {
    fprintf(stderr, "acq: Error getting shared memory segments\n" );
    fprintf(stderr,"about to call tell_xnmr_fail\n");
    tell_xnmr_fail();
    shut_down();
  }

  shmctl(data_shm_id,SHM_LOCK,NULL); // Xnmr can't do it, isn't root

  data_shm = (struct data_shm_t*) shmat( data_shm_id, (char*)data_shm ,0 );
  prog_shm = (struct prog_shm_t*) shmat( prog_shm_id, (char*)prog_shm ,0 );
  //  printf("acq-duelime: prog_shm %i\n",prog_shm);

  if( (long)data_shm == -1 || (long)prog_shm == -1 ) {
    perror( "acq: Error attaching shared memory segments" );
    //    data_shm = NULL;
    //    prog_shm = NULL;
    tell_xnmr_fail();
    shut_down();
  }

  if (existed == 0){ // we just created prog_shm
    fprintf(stderr,"acq: created prog_shm copying %s into version\n",PPROG_VERSION);
    strncpy(prog_shm->version,PPROG_VERSION,VERSION_LEN);
  }
  else{
    fprintf(stderr,"acq: found %s in prog_shm version\n",prog_shm->version);
    if (strcmp(prog_shm->version,PPROG_VERSION) != 0){
      fprintf(stderr,"acq: PPROG_VERSION mismatch\n");
      tell_xnmr_fail();
      shut_down();
    }
  }

  fprintf(stderr,"acq: found %s in data_shm version\n",XNMR_ACQ_VERSION);
  if (strcmp(data_shm->version,XNMR_ACQ_VERSION) != 0){
    fprintf(stderr,"acq: XNMR_ACQ_VERSION mismatch\n");
    fprintf(stderr,"data_shm says %s, I say: %s\n",data_shm->version,XNMR_ACQ_VERSION);
    tell_xnmr_fail();
    shut_down();
  }  

  if( data_shm->acq_pid > 0 ) {
    fprintf(stderr, "acq is already running\n" );
    tell_xnmr_fail();
    exit(1);
  }

  //Lock the memory pages as securely as possible

  if (getrlimit(RLIMIT_MEMLOCK,&my_lim) == 0){
    my_lim.rlim_cur = RLIM_INFINITY;
    my_lim.rlim_max = RLIM_INFINITY;
    if ( setrlimit(RLIMIT_MEMLOCK,&my_lim) != 0){
      perror("acq: setrlimit");
    }
    else{ // only do the memlock if we were able to set our limit.
      //      fprintf(stderr,"doing the mlockall\n");
      if (mlockall( MCL_CURRENT |MCL_FUTURE ) !=0 )
	perror("mlockall");
    }
  }
  data_shm->acq_pid = getpid();


  return 0;
}

int send_sig_ui( char sig )

{
  pid_t pid;

  pid =  data_shm->ui_pid;
  data_shm->acq_sig_ui_meaning = sig;

  if( pid > 0 ){
    //    fprintf(stderr,"in acq, sending a signal to Xnmr on pid %i\n",pid);
    kill( pid, SIG_UI_ACQ );
    return 0;
  }
  return -1;  
}

void ui_signal_handler()
    
{
  static volatile char waiting=0,queue_start=0;
  struct msgbuf message;
  //  fprintf(stderr, "acq: ui_signal_handler invoked\n" );

  switch( data_shm->ui_sig_acq_meaning ) {
    
  case ACQ_START:
    //    fprintf(stderr, "acq received signal ACQ_START\n" );
    data_shm->ui_sig_acq_meaning = NO_SIGNAL;
    if (waiting) {
      queue_start = 1;
      return;
    }
    if( running == 0) {
      queue_start = 1;
      do{
	queue_start = 0;
	running = 1;
	run();
	/* setting running off should be done before we tell Xnmr for a kind of subtle reason.
	   If we don't have real time scheduling, and there are experiments queued, we can get restarted 
	   before we finish out this call, but the restart will fail since this still says running.
	*/
	//  printf("setting RUNNING = 0\n");
	waiting=1;
	wait_till_streams_done();
	running = 0; // do this in the signal handler
	waiting=0;
	if( done >= 0 )
	  send_sig_ui( ACQ_DONE );
      }while (queue_start);
      // wait till all the lime streams are finished before marking running = 0;
      
      //      printf("returned from run\n");
    }
    else{
      fprintf(stderr, "acq: can't start, already running\n" );
      send_sig_ui( PPROG_ALREADY_RUNNING);
    }    
    break;

  case ACQ_STOP:
    //    fprintf(stderr, "acq received signal ACQ_STOP\n" );
    data_shm->ui_sig_acq_meaning = NO_SIGNAL;
    //    data_shm->mode = NO_MODE;  //this prevents last accumulation - don't do it
    done = NICE_DONE;
    //    prog_shm->stop_lime = NICE_DONE; // nope - only when all is said and done.
    break;

  case ACQ_KILL:
    //    fprintf(stderr,"acq: got kill signal, running is: %i\n",running);
    //    fprintf(stderr,"msg_wait_flag: %i\n",msg_wait_flag);

    // fprintf(stderr,"cleared EPP port\n");
    data_shm->ui_sig_acq_meaning = NO_SIGNAL; 
    //    data_shm->mode = NO_MODE; // xnmr_ipc does the rest of these.
    done = KILL_DONE;
    prog_shm->stop_lime = KILL_DONE; // tell the limes to stop too.
    // if run() is sleeping, waiting for pulse program, wake it up.
    
    if (msg_wait_flag == 1){ 
      message.mtext[0] = P_PROGRAM_ERROR;
      message.mtype = P_PROGRAM_READY;
      msgsnd ( msgq_id, &message, 1, 0 );
    }

    break;
  default:

    break;
  }
  //  fprintf(stderr,"returning from acq's ui sig handler\n");
  return;
}

int init_signals()
{
  sigset_t sigset;
  struct sigaction sa1;
  struct sigaction sa2;


  //  fprintf(stderr,"in acq init_signals\n");

  
  signal( SIGINT, shut_down  );
  signal( SIGTERM, shut_down );

  sigemptyset( &sigset );
  sa1.sa_handler = ui_signal_handler;
  sa1.sa_mask = sigset;
  //  sa1.sa_flags = SA_NOMASK; //This allows the signal handler to be interrupted by itself  
  sa1.sa_flags = SA_NODEFER; // better name for SA_NOMASK
  sa1.sa_restorer = NULL;
  
  sigaction( SIG_UI_ACQ, &sa1, &sa2  );

  sigemptyset(&sigset);
  sigaddset(&sigset,SIG_UI_ACQ);
  sigaddset(&sigset,SIGINT);
  sigaddset(&sigset,SIGTERM);

  //now that signal handlers are set up, unblock the ones we care about.
  sigprocmask(SIG_UNBLOCK,&sigset,NULL);

  return 0;
}

int wait_for_pprog_msg(  ){

  struct msgbuf message;
  int result;


  //  fprintf(stderr, "acq: retrieving message\n" );
  msg_wait_flag = 1;
  result = msgrcv ( msgq_id, &message, 1, P_PROGRAM_READY, MSG_NOERROR );
  msg_wait_flag = 0;

  /*   if(   prog_shm->prog_ready != READY)
       fprintf(stderr,"acq: got a message, but shm doesn't say ready\n");;  */
     
  if( result<0 )
    fprintf(stderr, "Acq received a bad message\n" );

  //Now that we have received a real message, stop the timeout timer and check to see what this signal is

  switch ( message.mtext[0] )
    {
    case P_PROGRAM_READY :
      return P_PROGRAM_READY;
      break;
    case P_PROGRAM_ERROR:
    case P_PROGRAM_PARAM_ERROR:
    case P_PROGRAM_INTERNAL_TIMEOUT:
    case P_PROGRAM_RECOMPILE:
      errcode = message.mtext[0];
      fprintf(stderr,"wait_for_pprog_msg: acq: got message that pulse program had an error\n");
      break;
    default:
      fprintf(stderr, "acq received an unusual message %ld\n", message.mtype );
      break;
    }

  return -1;
}


int start_pprog()

{
  pid_t pid;
  //  uid_t ruid;
  //  gid_t rgid;

  struct msgbuf message;
  int result;

  FILE *fs;
  char s[PATH_LENGTH];
  char dpath[PATH_LENGTH],spath[PATH_LENGTH],*sp,command[2*PATH_LENGTH+6];
  pid = fork();

  if( pid == 0 ) {  //start the pulse program

      data_shm->pprog_pid = getpid();



    /* don't want to be root inside the pulse program - want to inherit scheduling, but no
       special permissions */
      // hmm, this won't work though, because lock doesn't stay across exec
      mlockall(MCL_FUTURE);

      //    ruid=getuid();
    euid=geteuid();
    //    rgid=getgid();
    egid=getegid();
    //        fprintf(stderr,"real: %i, eff: %i\n, setting effective uid to %i\n",ruid,euid,ruid);

    //    seteuid(ruid);  // these two will give up root privileges.
    //    setegid(rgid);

    path_strcpy(s,getenv(HOMEP));
    path_strcat(s,"/Xnmr/prog/");
    path_strcat(s,data_shm->pulse_exec_path);

    fs = fopen(s,"rb");
    if (fs == NULL){
      path_strcpy(s,SYS_PROG_PATH);
      path_strcat(s,data_shm->pulse_exec_path);
      fs = fopen(s,"rb");
    }

    if (fs != NULL){
      fclose(fs);
      //      fprintf(stderr,"launching pprog using: %s\n",s);

      // copy the source code of the program into the user's directory.
      path_strcpy(dpath,data_shm->save_data_path);
      make_path( dpath);
      path_strcat(dpath,"program");
      //      printf("source-code dest: %s\n",dpath);

      path_strcpy(spath,s); // get the program name
      sp=strrchr(spath,'/'); // find the last /
      sp[0]=0; // wipe it out
      path_strcat(spath,"/compiled/"); // append to it
      sp=strrchr(s,'/'); // find the last / in the program
      path_strcat(spath,sp+1); // add the program
      path_strcat(spath,".x"); // put on the .x
      //      printf("source-code source is: %s\n",spath);
      sprintf(command,"cp -p %s %s",spath,dpath);
      //      printf("copy command is: %s\n",command);
      system(command);
      // if it fails, oh well.
      // if it fails, we should try to do .c instead of .x

      //      execl( s, NULL, NULL );
      execl( s, s, (char *) NULL );
    }

    // shouldn't return from here
    fprintf(stderr, "failed to launch pulse program\n" );


  
    // let acq know we have a problem
    message.mtype = P_PROGRAM_READY;
    message.mtext[0] = P_PROGRAM_ERROR;
    
    //    data_shm->pprog_pid = -1;  // don't do this now - do it in pprog_post_timeout

    result=msgsnd ( msgq_id, &message, 1, 0 );
    if (result == -1) perror("pulse.c:msgsnd");
    
    exit(1);
  }

  //fprintf(stderr, "acq: starting pulse program: %s on pid: %d\n", data_shm->pulse_exec_path, (int)pid );

  return (int) pid;
}

int accumulate_data( int64_t * buffer )

{
  int i;
  int mult=1;

  if (data_shm->npts > MAX_DATA_POINTS){
    fprintf(stderr,"acq: accumulate_data, npts > MAX_DATA_POINTS, should NEVER HAPPEN\n");
    data_shm->npts = MAX_DATA_POINTS;
  }
  //data does not accumulate in repeat mode
  if( data_shm->mode == REPEAT_MODE ) {
    //{
    for( i=0; i<data_shm->npts*2; i++ )
      data_shm->data_image[i] = 0;
  }

  for( i=0; i<data_shm->npts*2; i += 2 ) {
    data_shm->data_image[ i ] += buffer[i];
    data_shm->data_image[ i+1 ] += buffer[i+1]*mult;
  }
  return 0;
}


void post_pprog_timeout()
{
  int pid;
  pid = data_shm->pprog_pid;
  
  //  fprintf(stderr,"in post_pprog_timeout, pid: %i\n",data_shm->pprog_pid);
  if( pid>0 ) {
    //    fprintf(stderr, "acq: terminating pulse program\n" );
    kill( pid, SIGKILL ); // if the pulse program was dead, our handler
    // was catching the SIGTERM and running shut_down ??
    //    fprintf(stderr,"about to waitpid\n");
    waitpid(pid,NULL,0);
    //    fprintf(stderr,"returned from waitpid\n");
    data_shm->pprog_pid = -1; //carl added
    
  }

  done = ERROR_DONE;
  prog_shm->stop_lime = ERROR_DONE;
  fprintf(stderr,"post_pprog_timeout: set done to error_done\n");

  //  fprintf(stderr, "acq: Error, invalid pulse program or timeout occurred\n" );
  if (errcode == 0){
    //    fprintf(stderr,"in post_pprog timeout with errcode 0\n");
    send_sig_ui( P_PROGRAM_ERROR );
  }
  else{
    //    fprintf(stderr,"in post_pprog_timeout, sending errcode: %i\n",errcode);
    send_sig_ui( errcode );
  }
  errcode = 0;
}

void pprog_ready_timeout()

{
  struct msgbuf message;
  //  fprintf(stderr,"acq: in pprog ready timeout, pid: %i\n",data_shm->pprog_pid);
  errcode = P_PROGRAM_ACQ_TIMEOUT;

  //Send a mesage to wake up the sleeping run()
  if (msg_wait_flag == 1){

    message.mtext[0] = P_PROGRAM_ERROR;
    message.mtype = P_PROGRAM_READY;
    msgsnd ( msgq_id, &message, 1, 0 );
  }
  return;
}



void start_up_hardware(int *first_time,int64_t *buffers, char use_bgu){
  int i,rval;

  /// At start of acquisition, we  send the start-up sync program, run it, the send the real program.
	// limes never know, except that tx thread looks for sync event.
	if (*first_time){
#ifdef HDUEPP
	  printf("sending start-up sync prog\n");
	  if (done >= 0){
	    usleep(100); //if we don't sleep, the display doesn't get updated.
	    i = pulse_hardware_send(start_up_prog);
	    if( i < 0 ) {
	      fprintf(stderr, "acq: Error downloading pulse program to hardware\n" );
	      done = ERROR_DONE;
	      prog_shm->stop_lime = ERROR_DONE;
	      send_sig_ui( PULSE_PP_ERROR );
	    }	  
	  }
#endif

	  prog_shm->wait_for_sync = 1; // tells tx thread to wait for sync pulse. only limes care	  
	  prog_shm->etxprogno = prog_shm->txprogno; // this is the program that will be executed. limes and bgu cares
	  prog_shm->txprogno = ( prog_shm->txprogno + 1 ) %2; // calc next one in other buffer.
#ifdef LIMESDR
	  if (done >= 0){
	    rval = prep_lime(data_shm->npts,buffers) ;
	    if ( rval < 0) // master lime produces the sync pulses that reset the timestamps, and starts the due pp
	      done = ERROR_DONE;
#ifdef HDUEPP // need both limes and due for this to work:
	    if ( rval > 0) // need to synchronize sample clocks:
	      sync_lime();
#endif
	    //	    start_lime();
	    //	    usleep(100);
	  }

	  
#endif
#ifdef HDUEPP
	  printf("starting start-up sync prog\n");
	  if( done >= 0 ) {
	    i = pulse_hardware_start('E'); // E starts it so it waits for external trigger. e makes it start.
	    if( i < 0 ) {
	      fprintf(stderr, "acq: Error starting pulse program\n" );
	      done = ERROR_DONE;
	      prog_shm->stop_lime = ERROR_DONE;
	      if (i == - (TTC_ERROR))
		send_sig_ui (TTC_ERROR);
	      else
		send_sig_ui( P_PROGRAM_ERROR );
	    }
	    // the first lime will wait a few ms before it toggles its gpios to reset timestamps and start the due.
	  }
#endif
#ifdef HDUEPP
	  if (done >= 0)
	    start_lime();
#endif
	  //	  usleep(100e-3); // let the pulse programmer hit the reset button on the output divider of the clock generator. Done before limes started.
#ifdef DUEBGU
	  if (done >=0 && use_bgu){
	    if (start_bgu(&prog_shm->gradprog[prog_shm->etxprogno],bgufd) != 0){
	      done = ERROR_DONE;
	      prog_shm->stop_lime = ERROR_DONE;
	      send_sig_ui( BGU_ERROR );
	    }
	    usleep(100);
	  }
#endif

	   
#ifdef HDUEPP
	  i = -1;
	  while (done >=0 && i == -1){
	    i = pulse_hardware_wait(); // -1 says we were woken by a signal.
	  } 
#endif

#ifdef HDUEPP	  
	if( done >= 0 ) {
	  printf("sending first real prog\n");
	  //	  i = pulse_hardware_send( prog_shm );
	  i = pulse_hardware_send( prog_shm->due_prog);
	  if( i < 0 ) {
	    fprintf(stderr, "acq: Error downloading pulse program to hardware\n" );
	    done = ERROR_DONE;
	    prog_shm->stop_lime = ERROR_DONE;
	    send_sig_ui( PULSE_PP_ERROR );
	  }
	}
#endif
#ifdef HDUEPP
	if (done >= 0){
	  i = pulse_hardware_start('R');
	  if( i < 0 ) {
	    fprintf(stderr, "acq: Error starting pulse program\n" );
	    done = ERROR_DONE;
	    prog_shm->stop_lime = ERROR_DONE;
	    if (i == - (TTC_ERROR))
	      send_sig_ui (TTC_ERROR);
	    else
	      send_sig_ui( P_PROGRAM_ERROR );
	  }
	}
#endif
	*first_time = 0;
	} /// done with first time.
	else{// regular start
#ifdef HDUEPP	  
	  if( done >= 0 ) {
	    printf("sending prog\n");
	    i = pulse_hardware_send( prog_shm->due_prog);
	    if( i < 0 ) {
	      fprintf(stderr, "acq: Error downloading pulse program to hardware\n" );
	      done = ERROR_DONE;
	      prog_shm->stop_lime = ERROR_DONE;
	      send_sig_ui( PULSE_PP_ERROR );
	    }
	  }
	  if (done >= 0){
	    i = pulse_hardware_start('R');
	    if( i < 0 ) {
	      fprintf(stderr, "acq: Error starting pulse program\n" );
	      done = ERROR_DONE;
	      prog_shm->stop_lime = ERROR_DONE;
	      if (i == - (TTC_ERROR))
		send_sig_ui (TTC_ERROR);
	      else
		send_sig_ui( P_PROGRAM_ERROR );
	    }
	  }
#endif	    
	  prog_shm->etxprogno = prog_shm->txprogno; // this is the program that will be executed.
	  prog_shm->txprogno = ( prog_shm->txprogno + 1 ) %2; // calc next one in other buffer.
#ifdef LIMESDR
	  if (done >=0){
	    rval = prep_lime(data_shm->npts,buffers) ;
	    if ( rval != 0) // <0 is an error,>0 means do sync. But that should never happen.
	      done = ERROR_DONE;
	    else
	      start_lime();
	  }
#endif
#ifdef DUEBGU
	  if (done >=0 && use_bgu){
	    uint16_t in_buff,zero_count,rval;
	    printf("acq asking bgu status\n");
	    rval = bgu_get_status(&in_buff,&zero_count,bgufd);
	    printf("acq found bgu status with %i words in buffer and %i zeros sent\n",in_buff,zero_count);
	    if (rval != 0){
	      done = ERROR_DONE;
	      prog_shm->stop_lime = ERROR_DONE;
	      send_sig_ui( BGU_ERROR );
	    }
	  }
	  if (done >= 0 && use_bgu){
	    if (start_bgu(&prog_shm->gradprog[prog_shm->etxprogno],bgufd) != 0){
	      done = ERROR_DONE;
	      prog_shm->stop_lime = ERROR_DONE;
	      send_sig_ui( BGU_ERROR );
	    }
	  }
#endif
	}


}// end start_up_hardware
  


int run()

{
  int64_t *buffers = NULL;
  int i;
  int j,nblocks;
  pid_t pid;
  struct msgbuf message;
  struct itimerval time, old;
  int result;
  int ruid,euid,rgid,egid;
  //  int sweep;
  //  char force_setup;
  char reset_data;
  int first_time; // first time is for how the due gets started, also used to crank up pp priority
  double current_ppo_time=0,current_total_time=0,prev_ppo_time;

  int block_size,dummy_scans,init_dummy_scans;
  static int64_t *block_buffer= NULL; //static to help prevent mem leaks.  Shouldn't need to be.
  int current_block;
  int num_calced = 0; // number of real live pulse programs we've calculated
  FILE* fstream;
  char path[PATH_LENGTH],fileN[PATH_LENGTH],fileN2[PATH_LENGTH],fileNfid[PATH_LENGTH];
  float f;
  char end_1d_loop=0,end_2d_loop=0,use_bgu;
  
  double ppt;
  if (buffers == NULL)
    buffers = malloc(data_shm->npts * 2 * sizeof ( int64_t));
  else
    printf("WARNING: acq-duelime, staring run, but buffers weren't NULL?!\n");

  if (block_buffer != NULL){
    fprintf(stderr,"acq: on enter, block_buffer was not NULL\n");
    free(block_buffer);
    block_buffer = NULL;
  }
  prog_shm->txprogno = 0; // there are two tx pulse prog buffers. Which do we start with?
  
  done = NOT_DONE;
  prog_shm->stop_lime = NOT_DONE; 

  //First we create the directory for the data
  path_strcpy(path,data_shm->save_data_path);
  make_path( path);
 
 // want to make sure the user has write access to create data file:
 ruid=getuid();
  euid=geteuid();
  rgid=getgid();
  egid=getegid();

  seteuid(ruid);
  setegid(rgid);

  if( mkdir( path, S_IRWXU | S_IRWXG | S_IRWXO ) < 0 ) {
    //    fprintf(stderr,"acq: error in dir. creation\n");
    if( errno != EEXIST ) {
      perror( "acq couldn't make dir" );
      cant_open_file();
      running = 0;
      return 0;
    }
    else
      //      fprintf(stderr, "aq says directory %s already exists\n",path );
      errno = 0;
  }

  //Now we create a file containing some parameters
  data_shm->last_acqn = 0;
  data_shm->ct = 0;
  if( data_shm->mode == NORMAL_MODE || data_shm->mode == NORMAL_MODE_NOSAVE) {

    path_strcpy( fileN2, path);
    path_strcat( fileN2, "params" );

    //fprintf(stderr, "creating parameter file: %s\n", fileN2 );
    if (write_param_file(fileN2) == -1){
      running = 0;
      return 0;
    }

    path_strcpy( fileN, path);
    path_strcat( fileN, "proc_params" );
    fstream = fopen( fileN, "w" );
    if (fstream == NULL){
      cant_open_file();
      return 0;
    }
    fprintf(fstream,"PH: %f %f %f %f\n",0.0,0.0,0.0,0.0);
    fprintf(fstream,"FT: %i\n",0);
    fprintf(fstream,"FT2: %i\n",0);
    fclose(fstream);
    
    path_strcpy( fileN, path);
    path_strcat( fileN, "data" );
    
    //We have to empty the file 
    
    fstream = fopen( fileN, "wb" );
    if (fstream == NULL){
      cant_open_file();
      return 0;
    }
    fclose( fstream );

    // same again for backup of fid

    path_strcpy( fileNfid, path);
    path_strcat( fileNfid, "data.fid" );

    fstream = fopen( fileNfid, "wb" );
    if (fstream == NULL){
      cant_open_file();
      return 0;
    }
    fclose( fstream );
   
    // the pulse program source is copied into this directory
    // in start_pprog
  }
  
  result = seteuid(euid);
  if (result != 0) perror("acq:");
  result = setegid(egid);
  if (result != 0) perror("acq:");
  
  data_shm->acqn_2d = 0 ;
  data_shm->acqn = 0;
  
  /// look for dummy scans:
  result = sfetch_int(data_shm->parameters,"dummy_scans",&dummy_scans,0);
  if (result != 0 ) dummy_scans = 0;
  init_dummy_scans = dummy_scans;
  //  else fprintf(stderr,"acq: got %i dummy_scans\n",dummy_scans);

  // check to see if we're doing block averaging:

  result = sfetch_int(data_shm->parameters,"block_size",&block_size,0);
  if (result == 0){
    //fprintf(stderr,"acq: got block_size: %i\n",block_size);
  }
  else
    block_size = 0;

  if (block_size > data_shm->num_acqs || data_shm->num_acqs_2d == 0) block_size = 0;
  // adjust block_size if necessary in noisy mode to not request more points than fill fit in
  // 32 bits.
  if (prog_shm->is_noisy == 1){
    if (block_size == 0){
      if ((( uint64_t)(data_shm->num_acqs+ dummy_scans)) * data_shm->npts > 2e9){
	block_size = floor (2e9/data_shm->npts)-dummy_scans;
	printf("NOISY MODE: block size adjusted to: %i\n",block_size);
      }
    }
    else{
      if (((uint64_t)(block_size+dummy_scans)) * data_shm->npts > 2e9){
	block_size = floor (2e9/data_shm->npts)-dummy_scans;
	printf("NOISY MODE: block size adjusted to: %i\n",block_size);
      }
    }
  }
  
  if (block_size > 0){
    block_buffer = malloc(data_shm->num_acqs_2d * data_shm->npts * 2 * sizeof ( int64_t));
    if (block_buffer == NULL){
      fprintf(stderr,"acq: malloc for block_buff failed\n");
      done = ERROR_DONE;
      prog_shm->stop_lime = ERROR_DONE;

    }
    else{
      memset(block_buffer,0,data_shm->num_acqs_2d*data_shm->npts*2*sizeof(int64_t));
	     /*      for (i = 0;i < data_shm->num_acqs_2d * data_shm->npts *2 ; i++ )
		     block_buffer[i]=0; */
    }
  }

  // also zero out the shared memory...  this is also done later, but 
  // its nice to send 0's back to the user right away.
  memset(data_shm->data_image,0,data_shm->npts*2*sizeof(int64_t));

  current_block = 0;
  data_shm->last_acqn_2d=0;

  //  sweep = (int) rint(1.0 / data_shm->dwell*1e6);

  if (done == NOT_DONE){
#ifdef HDUEPP
    i = init_pulse_hardware(NUM_BOARDS);// finds the number of boards requested and opens them.
    printf("done with init_pulse_hardware\n");
#else
    i=1;
#endif
    if( i < NUM_BOARDS ) {
      fprintf(stderr, "acq: Error Initializing pulse programmer hardware\n" );
      done = ERROR_DONE; 
      prog_shm->stop_lime = ERROR_DONE;
      send_sig_ui( PULSE_PP_ERROR );
    }
  }
  
  first_time = 1;

  char ldo_synth_setup;
  
  // a pulse program might set the do_synth_setup flag if it gets
  // into a mode other than single tone mode.
  // don't overwrite that flag.
  //  prog_shm->do_synth_setup = 0;
  if (data_shm->reset_dsp_and_synth) prog_shm->do_synth_setup = 1;
  ldo_synth_setup = prog_shm->do_synth_setup;

  // so other programs know the spectromter is actually in use.
  fstream=fopen("/var/run/Xnmr_acq_is_running","wb");

  if (fstream != NULL) // in case we're not root.
    fclose(fstream);

  data_shm->time_remaining = 0;
  prog_shm->is_first_real_prog = 0;
  prog_shm->is_last_real_prog = 0;

  if (done == NOT_DONE){
    start_pprog();     //This will start the pulse program and calculate the first iteration
  }
  
  // ok this is where we try to calculate how long the whole thing will take:
  
  for (i=0;i< data_shm->num_acqs_2d && done == NOT_DONE;i++){

    time.it_interval.tv_sec = 0;
    time.it_interval.tv_usec = 0;
    time.it_value.tv_sec = 5;
    time.it_value.tv_usec = 0;
    //      fprintf(stderr,"about to call signal for SIGALRM\n");
    signal( SIGALRM, pprog_ready_timeout ); 
    setitimer( ITIMER_REAL, &time, &old );
    
      //      fprintf(stderr,"starting to wait for pprog in pre-calc, done = %i\n",done);
    if( wait_for_pprog_msg() <0)  //waits till a message comes back from pprog.
      post_pprog_timeout();
      
    if (done < 0){  //this may seem a little weird - reset the timer only if we're done.  Otherwise, we're
      //about to set it again immediately anyway.
	//reset the timer
	//	fprintf(stderr,"clearing timer after startup failure\n");
      time.it_interval.tv_sec = 0;
      time.it_interval.tv_usec = 0;
      time.it_value.tv_sec = 0;
      time.it_value.tv_usec = 0;
      setitimer( ITIMER_REAL, &time, &old );
      //	fprintf(stderr,"cleared timer after startup failure\n");
    }

    if (done == NOT_DONE){
      ppt = pp_time_new();
      //	ppt = pp_time();
      // so presumably, we now have the pulse program calculated
      //figure how long it is
      //NOISY FIXME: precalc might be ok.
      if (prog_shm->is_noisy == 1){
	// here, increment is the number of times we start the sequence. (~ nscans/bs)
	if (block_size > 0){
	  nblocks = data_shm->num_acqs/block_size;
	  data_shm->time_remaining += ppt*nblocks; // assume pprogram does block_size+dummy_scans.
	  printf("for record %i, adding time: %lf for %i full blocks\n",i,ppt,nblocks);
	  data_shm->time_remaining += ppt* ((float)data_shm->num_acqs-(float)nblocks*block_size +dummy_scans)/(block_size+dummy_scans); // approximate...
	  printf("for record %i, adding time: %lf for final block\n",i,ppt* ((float)data_shm->num_acqs-(float)nblocks*block_size +dummy_scans)/(block_size+dummy_scans));
	}
	else {
	  data_shm->time_remaining += ppt; // assume pprogram does block_size+dummy_scans.
	  printf("for record %i, adding time: %lf\n",i,ppt);
	}
      }
      else{
	data_shm->time_remaining +=  ppt*( data_shm->num_acqs  );
	if (data_shm->acqn_2d == 0 ) {  // take account of dummy scans
	  data_shm->time_remaining += ppt* dummy_scans;
	}
	//	else fprintf(stderr,"in acq, calc'ing pp length, got j = %i\n",j);
      }
      fprintf(stderr,"after calc'ing: %d, total time is: %f\n",data_shm->acqn_2d,data_shm->time_remaining);
      
      // increment the 2d loop
      data_shm->acqn_2d += 1;
      if (data_shm->acqn_2d == data_shm->num_acqs_2d){
	//	    data_shm->time_remaining -= (double) ppo_time(); // we don't wait for the ppo on the last scan.
	data_shm->acqn_2d = 0;
	// I don't think this is needed anymore.
	//	    if (data_shm->reset_dsp_and_synth) prog_shm->do_synth_setup = 1;
	// restore variables that remember hardware state.
	prog_shm->do_synth_setup = ldo_synth_setup;	
	// first real program is calculated here:
	prog_shm->is_first_real_prog = 1;
	num_calced = 1;
   }
     
      data_shm->acqn = 0;
   
      message.mtype = P_PROGRAM_CALC;
      message.mtext[0] = P_PROGRAM_CALC;
      //      fprintf(stderr,"acq: about to send message to calc next prog\n");
      msgsnd ( msgq_id, &message, 1, 0 );
      //    fprintf(stderr,"acq: sent message to calc next prog\n");
    }
  }
  use_bgu = prog_shm->use_bgu;
#ifdef DUEBGU
  if (use_bgu){
    bgufd = bgu_int_open();
    if ( bgufd < 0){
      printf("acq: couldn't open bgu interface\n");
      done = ERROR_DONE;
      prog_shm->stop_lime = ERROR_DONE;
      send_sig_ui(BGU_ERROR);
    }
  }
#endif
  //  fprintf(stderr,"Acq: pre-calc complete\n");
  if (prog_shm->event_error == 1){
    done = ERROR_DONE;
    prog_shm->stop_lime = ERROR_DONE;
    
    send_sig_ui(EVENT_ERROR);
  }

  /*
   *  Start the main loop
   */

  fprintf(stderr,"going into main loop, block_size: %i\n",block_size);
  prev_ppo_time = 0;

  if (prog_shm->is_noisy == 0){
    while( done == NOT_DONE && end_2d_loop == 0 ) {
      fprintf(stderr,"inside acq's main while loop, acqn_2d: %i of %i\n",data_shm->acqn_2d,data_shm->num_acqs_2d);
      
      //Reset the shared memory for a new record in 2d dimension
      reset_data=1;
      
      end_1d_loop = 0;
      data_shm->last_acqn_2d = data_shm->acqn_2d; 

      // this is the one-d loop:

      while( done == NOT_DONE && ( end_1d_loop == 0 || data_shm->mode == REPEAT_MODE )  ) {
	//            fprintf(stderr,"inside acq's 1d while loop\n");
	
	//we sent out for start already
	
	//start the timer and wait for response from pulse program
  	if( done >= 0 && data_shm->acqn_2d == 0 && data_shm->acqn == 0 ) {
	  time.it_interval.tv_sec = 0;
	  time.it_interval.tv_usec = 0;
	  time.it_value.tv_sec = 5;
	  time.it_value.tv_usec = 0;
	  //	fprintf(stderr,"about to call signal for SIGALRM\n");
	  signal( SIGALRM, pprog_ready_timeout ); 
	  setitimer( ITIMER_REAL, &time, &old );
	}
		
	if( done >= 0 ) {
	  //	  fprintf(stderr,"starting to wait for pprog in real-calc\n");
	  if( wait_for_pprog_msg() <0) 
	    post_pprog_timeout(); //waits till a message comes back from pprog.
	  //	  printf("acq: came back from waiting for msg\n");
	}
	if (done >=0){
	  current_ppo_time = ppo_time();
	  current_total_time = pp_time_new();
	}
	//reset the timer
	
	time.it_interval.tv_sec = 0;
	time.it_interval.tv_usec = 0;
	time.it_value.tv_sec = 0;
	time.it_value.tv_usec = 0;
	setitimer( ITIMER_REAL, &time, &old );
	
	if( prog_shm->prog_ready == NOT_READY )
	  fprintf(stderr, "acq: warning: pulse program not ready!\n" );
	
	//  ok, if this is the first time through, then apparently we heard from the pprog ok, now crank up its priority:
	if (first_time == 1 && done >= 0 ){
	    
#ifndef NO_RT_SCHED
	  struct sched_param sp;
	  int result,priority;
	  
	  result = sched_getparam( data_shm->pprog_pid, &sp );
	  priority = sched_get_priority_max(SCHED_FIFO);
	  sp.sched_priority = priority/2 - 4;  // set it to four less than acq's. order is: acq, tx, rx
	  result = sched_setscheduler(data_shm->pprog_pid,SCHED_FIFO,&sp); // comment out to not
	  //	fprintf(stderr,"in acq, set priority of: %i for pprog\n",priority/2);
	  
	  if( result!= 0) 
	    perror("acq: init_sched for pprog");
	  
	  result = sched_getscheduler(data_shm->pprog_pid);
	  if (result != SCHED_FIFO) fprintf(stderr,"acq: scheduler of pprog is not SCHED_FIFO: %i\n",result);
#endif
	}
	
	// check to see if there was a pulse_program event_error:
	if (done >= 0){
	  if (prog_shm->event_error == 1){
	    fprintf(stderr,"acq: got pulse_program event_error\n");
	    done = ERROR_DONE;
	    prog_shm->stop_lime = ERROR_DONE;
	    send_sig_ui(EVENT_ERROR);
	  }
	}
	
	start_up_hardware(&first_time,buffers,use_bgu);
	
	// last program is now sent to hardware, fix up variables to go calculate the next one.
	
	data_shm->acqn++;
	data_shm->last_acqn = data_shm->acqn; // used for rewrite of param file and by xnmr
	
	if (dummy_scans > 0 && (data_shm->mode == NORMAL_MODE || data_shm->mode == NORMAL_MODE_NOSAVE) ) {
	  data_shm->acqn = 0;
	  data_shm->last_acqn = -dummy_scans + 1;
	}
	
	
	//      fprintf(stderr,"acq:just incremented acqn: %li\n",data_shm->acqn);
	
	// next three lines to avoid executing % with block_size = 0
	i = 1;
	if (block_size == 0) i = 0;
	else if (data_shm->acqn % block_size != 0) i = 0;
	if (dummy_scans > 0) i = 0;
	
	if ((data_shm->acqn == data_shm->num_acqs || i == 1    )
	    && (data_shm->mode == NORMAL_MODE || data_shm->mode == NORMAL_MODE_NOSAVE)){
	  
	  if(data_shm->acqn_2d == data_shm->num_acqs_2d -1&& data_shm->acqn == data_shm->num_acqs  )
	    end_2d_loop = 1;
	  
	  end_1d_loop = 1;
	  data_shm->acqn_2d++;
	  
	  if (block_size > 0 && data_shm->acqn_2d == data_shm->num_acqs_2d ){
	    data_shm->acqn_2d =0;
	    current_block += 1;
	  }
	  
	  if (block_size > 0){
	    data_shm->acqn = block_size * current_block;
	    if (data_shm->acqn > data_shm->num_acqs) data_shm->acqn = data_shm->num_acqs; 
	    // should only happen at very end
	  }
	  else
	    data_shm->acqn = 0;
	  
	}
	
	// go calculate the next pulse program.
	if (done >=0 && ((data_shm->acqn_2d < data_shm->num_acqs_2d && data_shm->acqn < data_shm->num_acqs) || (data_shm->mode == REPEAT_MODE) )){
	  //	if (done >=0 && (data_shm->acqn_2d < data_shm->num_acqs_2d || data_shm->mode == REPEAT_MODE )){
	  //		fprintf(stderr, "acq sending message P_PROGRAM_CALC with acqn, %li acq2d %i\n" ,
	  //			data_shm->acqn,data_shm->acqn_2d);

	  prog_shm->is_first_real_prog = 0;
	  //	  printf("NUM_CALCED: %i, at %i, %i\n",num_calced,data_shm->acqn, data_shm->acqn_2d);
	  // here dummy scans shouldn't count, since the program does them itself.
	  if ((num_calced == (data_shm->num_acqs * data_shm->num_acqs_2d -1 + init_dummy_scans)) && (data_shm->mode == NORMAL_MODE || data_shm->mode == NORMAL_MODE_NOSAVE))
	    prog_shm->is_last_real_prog = 1;
	  else
	    prog_shm->is_last_real_prog = 0;
	  num_calced += 1;

	  message.mtype = P_PROGRAM_CALC;
	  message.mtext[0] = P_PROGRAM_CALC;
	  //	fprintf(stderr,"acq: about to send message to calc next prog\n");
	  msgsnd ( msgq_id, &message, 1, 0 );
	  //	fprintf(stderr,"acq: sent message to calc next prog\n");
	}
	
	
	if( done >= 0 ) { // update time remaining, and read in data.
	  //	  fprintf(stderr, "acq doing acquisition %u of dimension %u\n", data_shm->acqn, data_shm->acqn_2d );
	  
	  //	fprintf(stderr,"%f %f %f\n",current_total_time/20e6,current_ppo_time/20e6,prev_ppo_time/20e6);
	  data_shm->time_remaining -= ( (double) current_total_time - current_ppo_time + prev_ppo_time);
	  prev_ppo_time = current_ppo_time;
	  
#ifdef LIMESDR
	  printf("starting to wait for lime data\n");
	  i = wait_for_lime_data(); // ok, lime data is ready.
	  printf("done waiting for lime data\n");
	  if (i<0) {
	    done = ERROR_DONE;
	    printf("wait_for_lime data came back with error, setting done to ERROR_DONE\n");
	  }
#endif
	  } // update time remaining and read in data.

	
	if( done >= 0 ){
	  if (reset_data == 1){
	    reset_data = 0;
	    // either set the shared memory to 0, or recall the correct block from the block buffer
	    if (block_size > 0){
	      memcpy(data_shm->data_image,&block_buffer[data_shm->last_acqn_2d*data_shm->npts*2],
		     data_shm->npts*2*sizeof(int64_t));
	    }
	    else{
	      memset(data_shm->data_image,0,data_shm->npts*2*sizeof(int64_t));
	    }
	  }
	  accumulate_data( buffers );
	  data_shm->ct += 1;
	}
	
	if (dummy_scans > 0 && (data_shm->mode == NORMAL_MODE || data_shm->mode == NORMAL_MODE_NOSAVE) ) {
	  dummy_scans--;
	  reset_data = 1;
	}    
	
	//signal UI that new data is ready 
	
	//      fprintf(stderr,"at end of 1d loop, done = %i, acqn: %li\n",done,data_shm->acqn);
	if (! (end_2d_loop == 1 && end_1d_loop == 1)) // don't bother on the last one - ACQ_DONE will also upload	    
	  if( done >= 0 ){
	    send_sig_ui( NEW_DATA_READY );
	    //      fprintf(stderr,"coming up to end acq's 1d loop, just told ui, new data\n");
	  }

	//	if (! (end_2d_loop == 1 &&  end_1d_loop == 1)){
#ifdef HDUEPP
	i = -1;
	while (done >=0 && i == -1){
	  i = pulse_hardware_wait(); // -1 says we were woken by a signal.
	  //	  printf("awake from pulse_hardware wait with rval: %i and done: %i\n",i,done);
	} 
#endif
      

      } //End of 1d while loop
      //      fprintf(stderr,"just out of acq's 1d loop\n");
      
      
      // added kill_done in here to not touch file when we hit kill
      if(( data_shm->mode == NORMAL_MODE || data_shm->mode == NORMAL_MODE_NOSAVE) && done != KILL_DONE) {
	
	// for blocks, store the data in the block buffer
	if ( block_size > 0)
	for ( i = 0 ; i < data_shm->npts * 2 ; i++ )
	   block_buffer[data_shm->last_acqn_2d*data_shm->npts *2 +i] = data_shm->data_image[i];
		
	//	fprintf(stderr, "acq appending file: %s\n", fileN );
	
	fstream = fopen( fileN, "r+b" );
	//      fprintf(stderr,"seeking to: %i\n",data_shm->last_acqn_2d*data_shm->npts*2*sizeof(float));
	fseek(fstream,data_shm->last_acqn_2d*data_shm->npts*2 * sizeof (float),SEEK_SET);
	for( i=0; i<data_shm->npts*2; i++ ) {
	  f = (float) data_shm->data_image[i];
	  fwrite( &f, sizeof(float), 1, fstream );
	}
	
	fclose( fstream );

	fstream = fopen( fileNfid, "r+b" );
	//      fprintf(stderr,"seeking to: %i\n",data_shm->last_acqn_2d*data_shm->npts*2*sizeof(float));
	fseek(fstream,data_shm->last_acqn_2d*data_shm->npts*2 * sizeof (float),SEEK_SET);
	for( i=0; i<data_shm->npts*2; i++ ) {
	  f = (float) data_shm->data_image[i];
	  fwrite( &f, sizeof(float), 1, fstream );
	}
	
	fclose( fstream );
	// rewrite the param file
	write_param_file (fileN2);
      }
      
    }  //End of 2d while loop - normal operation
  }// end of normal operation 
  else{   // doing the noisy thing
    int increment = 0; // number of scans we do in inner loop: block_size or num_acqs, or remainder.
    while( done == NOT_DONE && end_2d_loop == 0 ) {
      fprintf(stderr,"inside acq's main while loop, acqn_2d: %i of %i\n",data_shm->acqn_2d,data_shm->num_acqs_2d);
      
      //Reset the shared memory for a new record in 2d dimension
      reset_data=1;
      
      end_1d_loop = 0;
      data_shm->last_acqn_2d = data_shm->acqn_2d; 

      // this is the one-d loop:

      while( done == NOT_DONE && ( end_1d_loop == 0 || data_shm->mode == REPEAT_MODE )  ) {
	//            fprintf(stderr,"inside acq's 1d while loop\n");
	//we sent out for start already
	//start the timer and wait for response from pulse program
	
	if( done >= 0 && data_shm->acqn_2d == 0 && data_shm->acqn == 0 ) {
	  time.it_interval.tv_sec = 0;
	  time.it_interval.tv_usec = 0;
	  time.it_value.tv_sec = 5;
	  time.it_value.tv_usec = 0;
	  //	fprintf(stderr,"about to call signal for SIGALRM\n");
	  signal( SIGALRM, pprog_ready_timeout ); 
	  setitimer( ITIMER_REAL, &time, &old );
	}
	if( done >= 0 ) {
	  fprintf(stderr,"starting to wait for pprog in real-calc\n");
	  if( wait_for_pprog_msg() <0) 
	    post_pprog_timeout(); //waits till a message comes back from pprog.
	  fprintf(stderr,"acq: came back from waiting for msg, event_error is: %i\n",prog_shm->event_error);
	}
	//	current_ppo_time = ppo_time();
	ppt = pp_time_new();
	current_total_time = ppt;
	
	//reset the timer
	
	time.it_interval.tv_sec = 0;
	time.it_interval.tv_usec = 0;
	time.it_value.tv_sec = 0;
	time.it_value.tv_usec = 0;
	setitimer( ITIMER_REAL, &time, &old );
	
	if( prog_shm->prog_ready == NOT_READY )
	  printf("acq: warning: pulse program not ready!\n" );
	
	//  ok, if this is the first time through, then apparently we heard from the pprog ok, now crank up its priority:
	if (first_time == 1 && done >= 0 ){
	    
#ifndef NO_RT_SCHED
	  struct sched_param sp;
	  int result,priority;
	  
	  result = sched_getparam( data_shm->pprog_pid, &sp );
	  priority = sched_get_priority_max(SCHED_FIFO);
	  sp.sched_priority = priority/2 - 4;  // set it to four less than acq's 
	  result = sched_setscheduler(data_shm->pprog_pid,SCHED_FIFO,&sp); // comment out to not
	  //	fprintf(stderr,"in acq, set priority of: %i for pprog\n",priority/2);
	  
	  if( result!= 0) 
	    perror("acq: init_sched for pprog");
	  
	  result = sched_getscheduler(data_shm->pprog_pid);
	  if (result != SCHED_FIFO) fprintf(stderr,"acq: scheduler of pprog is not SCHED_FIFO: %i\n",result);
#endif
	  
	}
	// check to see if there was a pulse_program event_error:
	if (done >= 0){
	  if (prog_shm->event_error == 1){
	    fprintf(stderr,"acq: got pulse_program event_error\n");
	    done = ERROR_DONE;
	    prog_shm->stop_lime = ERROR_DONE;
	    send_sig_ui(EVENT_ERROR);
	  }	  
	}

	

	/* calculation of number of loops in pulse programs should look like this:

	   if (block_size == 0) loops = get_num_acqs() + dummy_scans;
	   else{
	   if (get_acqn() +block_size > get_num_acqs())
   	      loops =  get_num_acqs() - get_acqn() + dummy_scans;
	   else
	      loops = block_size + dummy_scans;
	   }

	   Hmm - that assumes get_acqn is correct. Which I think it will be.
	*/
	
	// next few lines are part of setting up for next program, but need them now too!
	increment = block_size;
	if (increment == 0) increment = data_shm->num_acqs;
	if (data_shm->acqn + increment > data_shm->num_acqs)
	  increment = data_shm->num_acqs - data_shm->acqn;
	data_shm->last_acqn = data_shm->acqn; // different from non-noisy

	if ( data_shm->mode != REPEAT_MODE)
	  data_shm->acqn += increment;
	
	// now, we're going to request blocks 
	uint64_t total_pts = data_shm->npts*(increment+dummy_scans);
	//	uint64_t total_pts = prog_shm->totalpoints; // how many points the pulse program will actually produce
	uint64_t pts_in_request=0,pts_in_total = 0;
	uint64_t pts_request = data_shm->npts*512; // ?
	
	
	// make sure that the pulse program isn't producing too many points. This is done in ready()
	// if not noisy.
	if (prog_shm->totalpoints > (increment+dummy_scans)*data_shm->npts ){
	  printf("pulse program will produce more data points than will fit!\n");
	  done = ERROR_DONE;
	  prog_shm->stop_lime = ERROR_DONE;
	  send_sig_ui( EVENT_ERROR );
	}
	if (prog_shm->totalpoints != (increment+dummy_scans)*data_shm->npts )
	  printf("WARNING acq noisy mode: pulse program will produce %li points, was expecting: %i\n",prog_shm->totalpoints,(increment+dummy_scans)*data_shm->npts);
	
	// aim to update display every 3s:
	//	pts_request = ceil(3e6/data_shm->dwell/data_shm->npts)*data_shm->npts;
	pts_request = ceil(3e6/data_shm->dwell/data_shm->npts)*data_shm->npts;
	
	// current_total_time is for increment+dummy_scans 
	// NOISY FIXME: the number of points here might overflow 32 bits ?
	// for now, ask for all the points in the block_size.
	if (pts_request > total_pts) pts_request = total_pts;
	printf("NOISY: display should update every %f s, or %li points\n",pts_request*data_shm->dwell*1e-6,pts_request);
	fprintf(stderr,"about to request %li pts\n",total_pts);
	

	// manipulate how many points the limes think they need to return:
	for (i=0;i<NUM_TX;i++){
	  if (increment+dummy_scans > 0){
	  printf("acq-duelime noisy mode: rxpoints %i was %li. Making it: %li\n",i,prog_shm->rxpoints[i],prog_shm->rxpoints[i]/(increment+dummy_scans));
	  prog_shm->rxpoints[i] = prog_shm->rxpoints[i]/(increment+dummy_scans);
	  }
	}

	start_up_hardware(&first_time,buffers,use_bgu);

	// last program is now sent to hardware, fix up variables to go calculate the next one.
	// calculate how many data points we want to acquire.

	  /* in noisy mode, dummy scans are taken care of in pulse program	
	if (dummy_scans > 0 && (data_shm->mode == NORMAL_MODE || data_shm->mode == NORMAL_MODE_NOSAVE) ) {
	  data_shm->acqn = 0;
	  data_shm->last_acqn = -dummy_scans + 1;
	}
	*/
	
	// no matter what block_size is, we just calculate next 2d increment.
	// unless we're in repeat mode in which case we always calculate the same one.

	//FIXME: repeat mode in noisy is messed up?
	if (data_shm->mode == NORMAL_MODE || data_shm->mode == NORMAL_MODE_NOSAVE){ // not repeat mode
	  
	  if(data_shm->acqn_2d == data_shm->num_acqs_2d -1 && data_shm->acqn == data_shm->num_acqs  )
	    end_2d_loop = 1;
	  
	  end_1d_loop = 1;
	  data_shm->acqn_2d++;
	  
	  if (block_size > 0 && data_shm->acqn_2d == data_shm->num_acqs_2d ){
	    data_shm->acqn_2d =0;
	    current_block += 1;
	  }
	  
	  if (block_size > 0){
	    data_shm->acqn = block_size * current_block;
	    if (data_shm->acqn > data_shm->num_acqs) data_shm->acqn = data_shm->num_acqs; 
	    // should only happen at very end
	  }
	  else
	    data_shm->acqn = 0;
	  
	}
	
	// go calculate the next pulse program.
	if (done >=0 && ((data_shm->acqn_2d < data_shm->num_acqs_2d && data_shm->acqn < data_shm->num_acqs) || (data_shm->mode == REPEAT_MODE) )){
	  printf("ACQ: calcing next pp with : %li %i\n",data_shm->acqn, data_shm->acqn_2d);
	  //		fprintf(stderr, "acq sending message P_PROGRAM_CALC with acqn, %li acq2d %i\n" ,
	  //			data_shm->acqn,data_shm->acqn_2d);
	  prog_shm->is_first_real_prog = 0;
	  if ((num_calced == (ceil(data_shm->num_acqs/block_size) * data_shm->num_acqs_2d) - 1) && (data_shm->mode == NORMAL_MODE || data_shm->mode == NORMAL_MODE_NOSAVE))
	    prog_shm->is_last_real_prog = 1;
	  else
	    prog_shm->is_last_real_prog = 0;
	  num_calced += 1;
	  message.mtype = P_PROGRAM_CALC;
	  message.mtext[0] = P_PROGRAM_CALC;
	  //	fprintf(stderr,"acq: about to send message to calc next prog\n");
	  msgsnd ( msgq_id, &message, 1, 0 );
	  //	fprintf(stderr,"acq: sent message to calc next prog\n");
	}
	
	for (j=0;j<increment+dummy_scans;j++){
	  data_shm->time_remaining -= ( (double) current_total_time*data_shm->npts/total_pts );
	  //	  printf("setting time_remaining to: %f\n",data_shm->time_remaining);
	  //	  printf("current total time: %f, npts is : %i, total_pts: %li\n",current_total_time,data_shm->npts,total_pts);
	//	printf("subtracting off %lf s\n",( (double) current_total_time*data_shm->npts/total_pts ));
	  if( done >= 0 ) { // update time remaining, and read in data.
	  // had i = read_fifo(npts in here)
#ifdef LIMESDR
	    //	    printf("noisy, waiting for data:\n");
	    i = wait_for_lime_data();
	    if (i<0) done = ERROR_DONE;

	    // ok, lime data is ready. Need to have told limes that they should only return npts
#endif
	  } // update time remaining and read in data.
	    
	  pts_in_total += data_shm->npts; // total is this block.
	  pts_in_request += data_shm->npts;
	  //	  printf("noisy, got data. pts_in_total: %li, pts_in_request: %li\n",pts_in_total,pts_in_request);
	  if (done >= 0 && pts_in_request == pts_request){
	    if (pts_request > total_pts-pts_in_total) pts_request = total_pts-pts_in_total;
	    //	      if (pts_request > 0){
	    //		i = dsp_request_data(pts_request,lreceiver_model);
	    //	      }
	    pts_in_request = 0;
	    printf("not requested %li points\n",pts_request);
	  }
	
	  if( done >= 0 ){
	    if (reset_data == 1){
	      reset_data = 0;
	      // either set the shared memory to 0, or recall the correct block from the block buffer
	      if (block_size > 0){
		memcpy(data_shm->data_image,&block_buffer[data_shm->last_acqn_2d*data_shm->npts*2],
		       data_shm->npts*2*sizeof(int64_t));
	      }
	      else{
		memset(data_shm->data_image,0,data_shm->npts*2*sizeof(int64_t));
	      }
	    }
	    if (j >= dummy_scans){
	      data_shm->last_acqn += 1;
	      accumulate_data( buffers );
	    }
	    data_shm->ct += 1;
	  }
	  
	/*
	if (dummy_scans > 0 && (data_shm->mode == NORMAL_MODE || data_shm->mode == NORMAL_MODE_NOSAVE) ) {
	  dummy_scans--;
	  reset_data = 1;
	}    
	*/
	//signal UI that new data is ready 
	//      fprintf(stderr,"at end of 1d loop, done = %i, acqn: %li\n",done,data_shm->acqn);
	  if (pts_in_request == 0 ){
	    if (! (end_2d_loop == 1 && pts_in_total == total_pts)) // don't bother on the last one - ACQ_DONE will also upload
	      if( done >= 0 ){
		send_sig_ui( NEW_DATA_READY );
		//	      fprintf(stderr,"just told ui, new data\n");
	      }
	  }
	}// done inner 1d noisy loop.


#ifdef HDUEPP
	i = -1;
	while (done >=0 && i == -1){
	  i = pulse_hardware_wait(); // -1 says we were woken by a signal.
	  //	  printf("awake from pulse_hardware wait with rval: %i and done: %i\n",i,done);
	} 
#endif

      } //End of 1d while loop
      //      fprintf(stderr,"just out of acq's 1d loop\n");
      
      /*
       *  This is where the data for this set of acqusitions should be saved before
       *  advancing to the next dimension
       */
      // added kill_done in here to not touch file when we hit kill
      if(( data_shm->mode == NORMAL_MODE || data_shm->mode == NORMAL_MODE_NOSAVE) && done != KILL_DONE) {
	

	// for blocks, store the data in the block buffer
	if ( block_size > 0)
	for ( i = 0 ; i < data_shm->npts * 2 ; i++ )
	   block_buffer[data_shm->last_acqn_2d*data_shm->npts *2 +i] = data_shm->data_image[i];
	
	fprintf(stderr, "acq appending file: %s\n", fileN );
	
	// need to change to fseek 
	fstream = fopen( fileN, "r+b" );
	//      fprintf(stderr,"seeking to: %i\n",data_shm->last_acqn_2d*data_shm->npts*2*sizeof(float));
	fseek(fstream,data_shm->last_acqn_2d*data_shm->npts*2 * sizeof (float),SEEK_SET);
	for( i=0; i<data_shm->npts*2; i++ ) {
	  f = (float) data_shm->data_image[i];
	  fwrite( &f, sizeof(float), 1, fstream );
	}
	
	fclose( fstream );

	fstream = fopen( fileNfid, "r+b" );
	//      fprintf(stderr,"seeking to: %i\n",data_shm->last_acqn_2d*data_shm->npts*2*sizeof(float));
	fseek(fstream,data_shm->last_acqn_2d*data_shm->npts*2 * sizeof (float),SEEK_SET);
	for( i=0; i<data_shm->npts*2; i++ ) {
	  f = (float) data_shm->data_image[i];
	  fwrite( &f, sizeof(float), 1, fstream );
	}
		
	fclose( fstream );
	// rewrite the param file
	write_param_file (fileN2);
      }
    }  //End of 2d while loop noisy
  } // end noisy
  
  // do some cleaning
  //  join_limes();// waits till they've all exited.
  if (block_buffer != NULL){
    free(block_buffer);
    block_buffer = NULL;
  }
  free(buffers);
  buffers = NULL;
  
  
#ifdef HDUEPP
  printf("ACQ: calling pulse_hardware_stop at end of run\n");
  pulse_hardware_stop();
  free_pulse_hardware();
#endif
#ifdef DUEBGU
  if (use_bgu && bgufd >= 0){
    printf("acq: closing due bgu interface\n");
    bgu_int_close(bgufd);
  }
#endif
  
  unlink("/var/run/Xnmr_acq_is_running");
  //terminate the pulse program
  
  //  fprintf(stderr,"acq: at terminate pp\n");
  pid = data_shm->pprog_pid;
  
  if( pid>0 ) {
    
    //    fprintf(stderr, "acq: terminating pulse program: pid is %i\n",pid );
      kill( pid, SIGTERM );
      
      // this is a normal pprog termination, causes pulse to call
      // wait for pulse program to terminate
      //      fprintf(stderr,"acq:about to wait for child\n");
      wait( NULL );

  }
  //  fprintf(stderr,"done killing pp\n");
  //  else fprintf(stderr,"acq: not bothering to try to kill pprog, it claims to be dead\n");
 

  //Now manually empty the wait queue of any extra P_PROGRAM_READY messages that could be present
  //if a timeout error occurred

  while( msgrcv ( msgq_id, &message, 1, 0, IPC_NOWAIT ) >= 0 )
    i++;

  //  scope_close_port();
  //  dsp_close_port();  this throws away the port bad...

  //send ACQ_DONE to ui


  //  fprintf(stderr,"returning from run\n");
  return 0;
} // end of run

void shut_down()

{
  pid_t c;

  //  fprintf(stderr, "acq starting shut down sequence\n" );

  if( data_shm != NULL ) {

    data_shm->acq_pid = -1;
    c = data_shm->pprog_pid;

    if( c > 0 ) {
      //      fprintf(stderr, "acq: attempting to shut down pprog\n" );
      kill( c, SIGKILL );
      //      fprintf(stderr,"acq: about to wait for child(2)\n");
      wait( NULL );
      printf("returned from shutting down pprog\n");
    }
  }

  munlockall();
#ifdef HDUEPP
  free_pulse_hardware();
#endif
  closelog(); // system logger messages



  iopl(0);
#ifdef LIMESDR
  join_limes();
  deinit_limes();
  close_limes();
#endif

  fprintf(stderr, "acq: releasing memory\n" );

  release_mem();

  //remove the message queue

  //fprintf(stderr, "removing message queue\n" );

  msgctl( msgq_id, IPC_RMID, NULL );

  fprintf(stderr, "acq main terminated\n" );
  exit(1);
  return;
}


int init_sched()

{

  //  struct timespec tp

#ifndef NO_RT_SCHED
  struct sched_param sp;
  int priority;
  int result;

  result = sched_getparam(0, &sp );
  priority = sched_get_priority_max(SCHED_FIFO);
   sp.sched_priority = priority/2;
   result = sched_setscheduler(0,SCHED_FIFO,&sp);  // comment out to not crank up
  //  fprintf(stderr,"set priority of: %i\n",priority/2);

  if( result!= 0) {
    perror("acq: init_sched");
    return result;
  }

  // result = sched_rr_get_interval(0,&tp);
  //  fprintf(stderr,"rr_interval: %li s, %li ns\n",tp.tv_sec,tp.tv_nsec);

  result = sched_getscheduler(0);

  if ( result == SCHED_FIFO) {
    //    fprintf(stderr,"confirmed SCHED_FIFO\n");
  }
  else return -1;
#endif
  
  return 0;

}

void release_mem()
{
  // Mark the shared segments for removal - This will only remove the segment
  // once all user processes have detached from it.
  
  shmctl ( prog_shm_id, SHM_UNLOCK, NULL ); // if another Xnmr was running, 
  // the shm won't vanish till it exits, but no need to keep it locked in mem now
  shmctl ( data_shm_id, SHM_UNLOCK, NULL );
  shmctl ( prog_shm_id, IPC_RMID, NULL );  
  shmctl ( data_shm_id, IPC_RMID, NULL );

  shmdt( (char*) data_shm );
  shmdt( (char*) prog_shm );
  return;
}

int main(){

  pid_t pid;

  // open lime device at start:
#ifdef LIMESDR
  printf("calling init_limes\n");
  open_limes();
  init_limes();
  printf("returned from init_limes\n");
#endif

  /*
   *  Initialization stuff
   */
#ifdef NO_RT_SCHED
  fprintf(stderr,"\n\n********************************\n\n");
  fprintf(stderr,"            Acq started with realtime scheduling Disabled\n");
  fprintf(stderr,"\n\n********************************\n\n");
#endif
#ifndef LIMESDR
  fprintf(stderr,"\n\n********************************\n\n");
  fprintf(stderr,"            Acq started without LIMESDR\n");
  fprintf(stderr,"\n\n********************************\n\n");
#endif
#ifndef HDUEPP
  fprintf(stderr,"\n\n********************************\n\n");
  fprintf(stderr,"            Acq started without HDUEPP \n");
  fprintf(stderr,"\n\n********************************\n\n");
#endif
  init_shm();
  init_signals();
  init_msgs();

  data_shm->reset_dsp_and_synth = 1; // so that first time we try to do an acquisition, it will happen.
  init_sched();
  openlog("Xnmr acq",0,LOG_USER); //for system logging of all zeros events.

  printf("ACQ: Calling build_startup\n");
  pulse_hardware_build_startup_lime(start_up_prog,NUM_BOARDS); // build due startup-sync program.
  /*
   *   Go to sleep and deal with the world through signal handlers
   */
  
  pid = data_shm->ui_pid;
  data_shm->acq_sig_ui_meaning = ACQ_LAUNCHED;
  prog_shm->stop_lime = 0;
  if( pid > 0 ) {
    //    fprintf(stderr,"acq startup:  sending signal to pid: %i\n",data_shm->ui_pid);
    kill( data_shm->ui_pid, SIG_UI_ACQ );
  }
  //fprintf(stderr, "acq main started\n" );

  while(1) {
    pause();
    //    fprintf(stderr, "acq main unpaused\n" );
  }
  //  fprintf(stderr, "acq: quitting main\n" );
    
  shut_down();

 
  return 0;
}


void tell_xnmr_fail(){
  int pid;

  pid = data_shm->ui_pid;
  fprintf(stderr,"acq: ui_pid is: %i\n",pid);
  data_shm->acq_sig_ui_meaning = ACQ_LAUNCH_FAIL;
  if( pid > 0 ) {
    //    fprintf(stderr,"acq: sending fail signal put: %i into sig\n",data_shm->acq_sig_ui_meaning);
    kill( data_shm->ui_pid, SIG_UI_ACQ );
  }

}

int write_param_file( char * fileN){

FILE *fstream;

 fstream = fopen( fileN, "w" );
 if (fstream == NULL){
   cant_open_file();
   return -1;
 }
 fprintf( fstream, "%s\n", data_shm->pulse_exec_path );
 //    fprintf(stderr,"doing save from within acq, dwell: %f\n",data_shm->dwell);
 fprintf( fstream, 
	  "npts = %u\nacq_npts = %u\nna = %lu\nna2 = %u\nsw = %lu\ndwell = %f\nct = %lu\n", 
	  data_shm->npts, data_shm->npts,data_shm->num_acqs, 
	  data_shm->num_acqs_2d,(long unsigned int) 
	  (1./data_shm->dwell*1000000),data_shm->dwell,data_shm->ct);
 fprintf( fstream,"ch1 = %c\nch2 = %c\n",data_shm->ch1,data_shm->ch2);

 fprintf( fstream, "%s",data_shm->parameters );
 fclose( fstream );

return 0;

}

double pp_time(){
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
  
  // look at time of first timer.
  while(prog_shm->opcodes[cur_event] != EXIT && counter < MAX_ALL_EVENTS){
    //    printf("event: %i, outputs: %i, opcode: %i duration: %f\n",cur_event,prog_shm->outputs[0][cur_event],prog_shm->opcodes[0][cur_event],prog_shm->times[0][cur_event]);
    if  ( prog_shm->is_noisy == 1) counter = 0; // so it doesn't overflow! not great... FIXME (and in pulse.c)
    switch (prog_shm->opcodes[cur_event] ){
    case CONTINUE:
    case SUBSTART:
      how_long += prog_shm->times[cur_event];
      cur_event+=1;
      break;
    case WAIT:
    case WAIT_MAX:
      how_long += prog_shm->times[cur_event]*prog_shm->opinst[cur_event];
      cur_event+=1;
      fprintf(stderr,"Got a WAIT, times may be off\n");
      break;
    case LOOP:
      local_opinst[cur_event]=prog_shm->opinst[cur_event]; // loads the loop counter
      how_long += prog_shm->times[cur_event];
      if (loop_num > STACK_SIZE -1){
	fprintf(stderr,"Too many Loops nested\n");
	done = ERROR_DONE;
	prog_shm->stop_lime = ERROR_DONE;
	return how_long/DUE_PP_CLOCK;
      }
      loop_stack[loop_num]=cur_event;
      loop_num += 1;
      cur_event+=1;
      break;
    case END_LOOP:
      how_long += prog_shm->times[cur_event];
      //      cur_event_new = prog_shm->opinst[0][cur_event];
      if (loop_num < 1){
	printf("got an end_loop, but no loop on stack\n");
	prog_shm->event_error = 1;
	done = ERROR_DONE;
	prog_shm->stop_lime = ERROR_DONE;
	return how_long/DUE_PP_CLOCK;
      }
      cur_event_new = loop_stack[loop_num-1];
      // make sure this was a loop event
      if (local_opinst[cur_event_new] < 0){ // check that the loop counter is >= 0.
	fprintf(stderr,"got a END_LOOP at event: %i, but didn't get a reasonable number of times to loop\n",cur_event);
	done = ERROR_DONE;
	prog_shm->stop_lime = ERROR_DONE;
	return how_long/DUE_PP_CLOCK;
      }
      local_opinst[cur_event_new] -= 1;
      if (local_opinst[cur_event_new] > 0){ //looping back.
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
      //	     cur_event,prog_shm->opinst[0][cur_event]);
      how_long += prog_shm->times[cur_event];
      jsr_stack[jsr_num] = cur_event + 1;
      jsr_num += 1;
      if (jsr_num > STACK_SIZE -1){
	fprintf(stderr,"too many JSR's nested\n");
	done = ERROR_DONE;
	prog_shm->stop_lime = ERROR_DONE;
	return how_long/DUE_PP_CLOCK;
      }
      cur_event = prog_shm->opinst[cur_event];
      if (cur_event > MAX_EVENTS || cur_event < 0){
	fprintf(stderr,"got a jsr to an event that doesn't exist.\n");
	done = ERROR_DONE;
	prog_shm->stop_lime = ERROR_DONE;
	return how_long/DUE_PP_CLOCK;
      }
      break;
    case RTS:
      how_long += prog_shm->times[cur_event];
      if (jsr_num < 1){
	fprintf(stderr,"got a JSR, but nowhere to return to on stack!\n");
	done = ERROR_DONE;
	prog_shm->stop_lime = ERROR_DONE;
	return how_long/DUE_PP_CLOCK;
      }
      cur_event = jsr_stack[jsr_num -1];
      jsr_num -=1;
      if (cur_event <0 || cur_event > MAX_EVENTS){
	fprintf(stderr,"got a RTS to a non-existant event!\n");
	done = ERROR_DONE;
	prog_shm->stop_lime = ERROR_DONE;
	return how_long/DUE_PP_CLOCK;
      }
      break;
    default:
      fprintf(stderr,"got an unknown opcode!\n");
      done = ERROR_DONE;
      prog_shm->stop_lime = ERROR_DONE;
      return how_long/DUE_PP_CLOCK;
    }
    

    counter += 1;
    
  }
  // capture EXIT:
  if (prog_shm->opcodes[cur_event]  == EXIT)
    how_long += prog_shm->times[cur_event];
  else{
    printf("pp_time_new. Didn't find an EXIT!\n");
    prog_shm->event_error = 1;
    done = ERROR_DONE;
    prog_shm->stop_lime = ERROR_DONE;
  }

  counter += 1;

  if (counter >= MAX_ALL_EVENTS){
    fprintf(stderr,"Got too many events (looping error? or forgotten EXIT?)\n");
    done = ERROR_DONE;
    prog_shm->stop_lime = ERROR_DONE;
    prog_shm->event_error = 1;
    return how_long/DUE_PP_CLOCK;
  }

  printf("pp_time: how_long: %f\n",how_long);

  return how_long/DUE_PP_CLOCK;
}

double pp_time_new(){
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
  // look at time of first timer.
  while(prog_shm->opcodes[cur_event] != EXIT && counter < MAX_ALL_EVENTS){
    //    printf("event: %i, outputs: %i, opcode: %i duration: %f\n",cur_event,prog_shm->outputs[0][cur_event],prog_shm->opcodes[0][cur_event],prog_shm->times[0][cur_event]);
    switch (prog_shm->opcodes[cur_event] ){
    case CONTINUE:
    case SUBSTART:
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
	done = ERROR_DONE;
	prog_shm->stop_lime = ERROR_DONE;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      loop_time[loop_level] = prog_shm->times[cur_event];
      loops[loop_level] = prog_shm->opinst[cur_event]; // load the loop counter
      cur_event+=1;
      break;
    case END_LOOP:
      if (loop_level <1){
	fprintf(stderr,"Got a loop end but not enough loop starts?\n");
	done = ERROR_DONE;
	prog_shm->stop_lime = ERROR_DONE;
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
	done = ERROR_DONE;
	prog_shm->stop_lime = ERROR_DONE;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      cur_event = prog_shm->opinst[cur_event];
      if (cur_event > MAX_EVENTS || cur_event < 0){
	fprintf(stderr,"got a jsr to an event that doesn't exist.\n");
	done = ERROR_DONE;
	prog_shm->stop_lime = ERROR_DONE;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      break;
    case RTS:
      loop_time[loop_level] += prog_shm->times[cur_event];
      if (jsr_num < 1){
	fprintf(stderr,"got a JSR, but nowhere to return to on stack!\n");
	done = ERROR_DONE;
	prog_shm->stop_lime = ERROR_DONE;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      cur_event = jsr_stack[jsr_num -1];
      jsr_num -=1;
      if (cur_event <0 || cur_event > MAX_EVENTS){
	fprintf(stderr,"got a RTS to a non-existant event!\n");
	done = ERROR_DONE;
	prog_shm->stop_lime = ERROR_DONE;
	return loop_time[0]/DUE_PP_CLOCK;
      }
      break;
    default:
      fprintf(stderr,"got an unknown opcode!\n");
      done = ERROR_DONE;
      prog_shm->stop_lime = ERROR_DONE;
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
    done = ERROR_DONE;
    prog_shm->stop_lime = ERROR_DONE;
  }
  
  if (counter >= MAX_ALL_EVENTS){
    fprintf(stderr,"Got too many events (looping error? or forgotten EXIT?)\n");
    done = ERROR_DONE;
    prog_shm->stop_lime = ERROR_DONE;
    prog_shm->event_error = 1;
    return loop_time[0]/DUE_PP_CLOCK;
  }

  // in case the program ends in the middle of a loop:
  for (i=1;i<=loop_level;i++)
    loop_time[0] += loop_time[i];
  printf("pp_time_new: how_long: %f\n",loop_time[0]/DUE_PP_CLOCK);
    
  return loop_time[0]/DUE_PP_CLOCK;
}

double ppo_time(){
  double how_long=0.;
  int i;
  // find the first EXIT event, and return its duration.
  for (i=0;i<prog_shm->no_events-1;i++)
    if (prog_shm->opcodes[i] == EXIT){
      how_long = prog_shm->times[i];
      return how_long/DUE_PP_CLOCK;
    }
  return 0;
}
