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

/* xnmr_ipc.c
 *
 * Part of the Xnmr software project
 *
 * UBC Physics
 * April, 2000
 * 
 * written by: Scott Nelson, Carl Michal
 */

#include "xnmr_ipc.h"
#include "shm_data.h"
#include "p_signals.h"
#include "panel.h"
#include "buff.h"
#include "process_f.h"
#include "param_f.h"
#include "xnmr.h"
#ifdef MSL200
//#include "h_config-pb.h" 
#elif defined DUELIME
//#include "h_config-duelime.h"
#elif defined DUESDR

#else
#include "h_config.h" // for CLOCK_SPEED
#endif
#include <gtk/gtk.h>
#include <signal.h>
#ifndef MINGW
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/errno.h>
#include <sys/wait.h>
#endif
#include <sys/types.h>
#include <stdio.h>

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#ifndef MINGW
#include <pthread.h>
#endif

/*
 *  Global Variables
 */

struct data_shm_t* data_shm;
int data_shm_id;
volatile char redraw=0;
char connected=FALSE; //for buff.c - clone_from_acq
struct timeval last_time, current_time;




#ifndef MINGW
void *sig_handler_thread_routine(void *dummy)
{ 
  sigset_t sigset;
  int sig;

  sigemptyset( &sigset );
  // this duplicates what's done in xnmr.c:
  sigaddset(&sigset,SIGQUIT);
  sigaddset(&sigset,SIGTERM);
  sigaddset(&sigset,SIGINT);
  sigaddset(&sigset,SIG_UI_ACQ);
  sigaddset(&sigset,SIGTTIN);
  // we don't actually ever get SIGTTIN.
  // if its blocked, it never gets delivered, but the 
  // read call that triggers it returns an error (EIO)

  // this blocking shouldn't be necessary - done in xnmr.c before threads created.
  //  sigprocmask(SIG_UNBLOCK,&sigset,NULL);
  pthread_sigmask(SIG_BLOCK,&sigset,NULL);

  // that's right - signals are blocked before we wait for them.

  data_shm->ui_pid = getpid();
  
  do{
    //    fprintf(stderr,"xnmr_ipc: about to wait for signal\n");
    sigwait(&sigset,&sig);
    //    fprintf(stderr,"xnmr_ipc: got a signal\n");
    //    gdk_threads_enter();
    if (sig == SIG_UI_ACQ)
      g_idle_add((GSourceFunc) acq_signal_handler,NULL);
    else if (sig == SIGQUIT || sig == SIGTERM || sig == SIGINT){
      fprintf(stderr,"signal thread, got a QUIT TERM or INT\n");
      data_shm->ui_pid = -1; // also done elsewhere but what the heck
      g_idle_add((GSourceFunc)do_destroy_all,NULL);
    }
    //    gdk_threads_leave();
  }
    while(TRUE);
  return dummy;
}



int init_ipc_signals()
{
  gint dummy=0;
  pthread_t sig_handler_thread;
  //  fprintf(stderr,"Xnmr in init_ipc_signals\n");
  sigset_t sigset;

  sigemptyset( &sigset );

  sigaddset(&sigset,SIG_UI_ACQ);
  sigaddset(&sigset,SIGQUIT);;
  sigaddset(&sigset,SIGTERM);
  sigaddset(&sigset,SIGINT);
  sigaddset(&sigset,SIGTTIN);
  //  sigprocmask(SIG_BLOCK,&sigset,NULL);
  pthread_sigmask(SIG_BLOCK,&sigset,NULL);

  /*
  sigblock(SIG_UI_ACQ); // block it explicitly, the new thread will catch it.
  sigblock(SIGQUIT);
  sigblock(SIGTERM);
  sigblock(SIGINT);
  */

  pthread_create(&sig_handler_thread,NULL,&sig_handler_thread_routine,&dummy);
  fprintf(stderr,"just created thread, mypid is: %i\n",getpid());



  return 0;
}
/*  old version
int init_ipc_signals()
{

  struct sigaction sa1;
  sigset_t sigset;

  sigemptyset( &sigset );


  sa1.sa_handler = acq_signal_handler;
  sa1.sa_mask = sigset;
  sa1.sa_flags = 0;
  sa1.sa_restorer = NULL;
  
  sigaction( SIG_UI_ACQ, &sa1, NULL );

  data_shm->ui_pid = getpid();

  return 0;
}
*/
int xnmr_init_shm() 

{

  int existed=0;

  data_shm = 0;

  // this will fail if shm exists:
  data_shm_id = shmget( DATA_SHM_KEY, sizeof( struct data_shm_t ), IPC_CREAT | IPC_EXCL|0660);
  fprintf(stderr,"xnmr_ipc: data_shm_id: %i\n",data_shm_id);

  if( data_shm_id < 0 ) { // so if shared mem exists:
    existed = 1;
    data_shm_id = shmget( DATA_SHM_KEY, sizeof( struct data_shm_t ), 0);

    if( data_shm_id < 0 ) {
      perror( "Xnmr_ipc: init_shm can't get shm id\n" );
      exit(1);
    }
    data_shm = (struct data_shm_t*) shmat( data_shm_id, (char*)data_shm ,0 );
    
    if( (long) data_shm == -1 ) {
      shmctl ( data_shm_id, IPC_RMID, NULL );
      perror( "xnmr_ipc : Error attaching shared memory segment" );
      fprintf(stderr,"Probably this means that another user is running Xnmr\n or Xnmr crashed, try Xnmr --noacq or ipcs\n");
      exit(1);
    }
       
    //fprintf(stderr, "Xnmr linked into acq that was already running on pid %d\n", data_shm->acq_pid );
    
    //don't set pid yet!
    connected = TRUE;
    return 1; // linked into already built shm
  }


  data_shm = (struct data_shm_t*) shmat( data_shm_id, (char*)data_shm ,0 );

  if( (long)data_shm == -1 ) {
    perror( "xnmr_ipc : Error attaching shared memory segment" );
      exit(1);
  }
  connected = TRUE;

  if (existed == 0){  // then we created above
    fprintf(stderr,"Xnmr: created shared memory, copying string: %s into version\n",XNMR_ACQ_VERSION);
    strncpy(data_shm->version,XNMR_ACQ_VERSION,VERSION_LEN);
  }
  else{
    fprintf(stderr,"Xnmr: shared memory existed, version string contained: %s\n",data_shm->version);
    if (strcmp(data_shm->version,XNMR_ACQ_VERSION) != 0){
      fprintf(stderr,"Xnmr: XNMR_ACQ_VERSION mismatch\n");
      shmctl(data_shm_id,IPC_RMID,NULL); // mark it for removal if we detect a mismatch
      release_shm();
      exit(1);
    }
  }
    



  /*
   *  Now set up some default values
   */

  //fprintf(stderr, "shared memory created\n" );
  
  //  data_shm->ui_pid = getpid();

  strncpy( data_shm->parameters,  "\ndummy_int = 7\ndummy_float = 5.3\n" ,PARAMETER_LEN);
  data_shm->parameter_count = 2;
  
  data_shm->mode = NO_MODE;
  data_shm->acq_pid = -1; 
  data_shm->pprog_pid = -1;

  data_shm->ui_pid = -1;

  /*
   *  These next two values will be overwritten when the first buffer is created
   */

  path_strcpy ( data_shm->pulse_exec_path, "");
  path_strcpy ( data_shm->save_data_path, "");
  
  data_shm->num_acqs = 1;
  
  data_shm->ui_sig_acq_meaning = NO_SIGNAL;
  data_shm->acq_sig_ui_meaning = NO_SIGNAL;
  return 0;
}

void start_acq()

{
  pid_t pid;

  pid = data_shm->acq_pid;

  if( pid == -1 ) {
    pid = fork();

    if( pid == 0 ) {  //start the pulse program
      //      char bell=7;
      // now set process group id to be our own so that acq doesn't
      // get any signals meant for the Xnmr parent
      setpgid(0,0);

      
      execl( "/usr/local/bin/acq", "/usr/local/bin/acq",(char *) NULL );
      // reaching here meant that launching acq failed
      // we can't pop up a dialog because we're a new program...
      {
	char mystr[2],i;
	mystr[0]=7;
	mystr[1]=0;
	fprintf(stderr,"*********************************************\n");
	fprintf(stderr,"*                                           *\n");
	fprintf(stderr,"*             FAILED TO LAUNCH ACQ          *\n");
	fprintf(stderr,"*                                           *\n");
	fprintf(stderr,"* You'll need to use ipcclean in the Xnmr   *\n");
	fprintf(stderr,"*           directory and put               *\n");
	fprintf(stderr,"*      a valid acq in /usr/local/bin/       *\n");
	fprintf(stderr,"*                                           *\n");
	fprintf(stderr,"*********************************************\n");
	for(i=0;i<5;i++){
	  fprintf(stderr,"%s",mystr);
	  fflush(stdout);
	  sleep(1);
	}
      }
      exit(1);
      
    }

  }

  else {
    fprintf(stderr, "Xnmr_ipc: acq already launched on pid %d\n", pid );
  }
  return;
}

int wait_for_acq()

{
  char c;

  //Now go to sleep and wait to be woken up by a signal from ACQ
  //If the signal is void (NO_SIGNAL), just go back to sleep 

  c = data_shm->acq_sig_ui_meaning;
 
  //  while( c == NO_SIGNAL ) {
    fprintf(stderr, "ui on pid %d: waiting for response from acq\n", getpid() );
    if (c == 0)
      pause( );
    c = data_shm->acq_sig_ui_meaning;
        fprintf(stderr, "ui: received a signal %i\n",c );
    //  } 
  data_shm->acq_sig_ui_meaning = NO_SIGNAL;

  if( c == ACQ_LAUNCHED ) {
    //fprintf(stderr, "Xnmr: verified that acq has started\n" );
      return ACQ_LAUNCHED;
  }
  if (c == ACQ_LAUNCH_FAIL) {
    fprintf(stderr,"ui: acq said it didn't start\n");
    return -1;
  }

  fprintf(stderr, "ui received an unusual signal %d\n", c );
  return -1;
}

int release_shm()
{
  // we don't mark anything for removal in case we want to leave acq in memory and reattach later
  // also, we set ui_pid to -1 so that acq stops sending signals

  data_shm->ui_pid = -1;

  shmdt( (char*) data_shm );
  return 0;
}

void end_acq()
{
  pid_t c;

  if (data_shm->ui_pid !=-1) fprintf(stderr,"in end_acq and ui_pid = %i\n",data_shm->ui_pid);
  //  data_shm->ui_pid = -1;

  c = data_shm->acq_pid;
  if( c > 0 ) {
    //    fprintf(stderr, "Xnmr_ipc is terminating acq\n" );
    kill( c, SIGTERM );
    wait( NULL );
  }
}

int send_sig_acq( char sig )
{
  pid_t pid;

  pid =  data_shm->acq_pid;
  if( pid > 0 ){
    //    fprintf(stderr, "xnmr_ipc sending signal %d to acq on pid %d\n", sig, pid );
    data_shm->ui_sig_acq_meaning = sig;
    kill( pid, SIG_UI_ACQ );
    return 0;
  }
  

  fprintf(stderr, "signal not sent!!\n" );
  return -1;  

}
#endif


void set_acqn_labels(int start)
{
  int hour,min,sec,count=0;
  char s[UTIL_LEN*2];

  time_t completion_time;  
  char *comp_time,*cpoint;

  struct timeval tv;
  struct timezone tz;
  long long my_time;
  
  snprintf( s, UTIL_LEN,"na %ld / %ld\nna2 %d / %d", data_shm->last_acqn, data_shm->num_acqs,
	    data_shm->last_acqn_2d+1,data_shm->num_acqs_2d);
  gtk_label_set_text( GTK_LABEL (acq_label), s );
  //  snprintf( s,UTIL_LEN, "na2 %d / %d", data_shm->last_acqn_2d+1, data_shm->num_acqs_2d );
  //  gtk_label_set_text( GTK_LABEL (acq_2d_label), s );
  // this is an ugly hack...
  if (start == 1 && data_shm->time_remaining == 0)
    do{
      usleep(20);
      count += 1;
    }while (count < 50 && data_shm->time_remaining == 0 );
#if defined MSL200 || defined DUELIME || defined DUESDR
  printf("in xnmr_ipc.c, time_remain is: %lf\n",data_shm->time_remaining);
  hour = (int) (data_shm->time_remaining/3600.);
  min = (int) (data_shm->time_remaining/60. - hour*60.);
  sec = (int) (data_shm->time_remaining*1.0 - hour*3600. - min*60.);
#else
  hour = (int) (data_shm->time_remaining/3600./CLOCK_SPEED);
  min = (int) (data_shm->time_remaining/60./CLOCK_SPEED - hour*60.);
  sec = (int) (data_shm->time_remaining*1.0/CLOCK_SPEED - hour*3600. - min*60.);
#endif
  //  printf("in set_acqn_labels, time remaining is: %f\n",data_shm->time_remaining/20000000.);

  // set completion time label:
  /*
  time(&completion_time);
  completion_time += (time_t) (data_shm->time_remaining*1.0/CLOCK_SPEED);

  need to keep more precision than just the current second, otherwise
  the expected completion time oscillates.

  */
  gettimeofday(&tv,&tz);
#if defined MSL200 || defined DUELIME || defined DUESDR
  my_time = tv.tv_sec+tv.tv_usec/1e6 +data_shm->time_remaining;
  completion_time = (time_t) my_time;
#else
  my_time = tv.tv_sec*1e6+tv.tv_usec +data_shm->time_remaining*1e6/CLOCK_SPEED;
  completion_time = (time_t) (my_time/1e6);
#endif
  comp_time = ctime(&completion_time);
  //  fprintf(stderr,"%s\n",comp_time);
  if(strlen(comp_time)>6) comp_time[strlen(comp_time)-6] = 0; // get rid of the year
  if(strlen(comp_time)>10) {
    cpoint = strchr(comp_time+9,' '); // looks like there is always a space for 2 digits of date
    if (cpoint != NULL)
      *cpoint = '\n';
  }
  
  
  //  snprintf(s,UTIL_LEN,"completion:\n%s",comp_time);
  //  gtk_label_set_text( GTK_LABEL(completion_time_label),s);
  
  snprintf( s,UTIL_LEN*2, "%2d:%2d:%2d\n%s\nacq buff: %i",hour,min,sec,comp_time,upload_buff);
  gtk_label_set_text( GTK_LABEL(time_remaining_label),s);

  //  fprintf(stderr,"left set labels\n");


}




gint upload_and_draw_canvas_with_process( dbuff *buff )
{

  if( redraw == 1 ) {
    set_acqn_labels(0);
    if (upload_buff ==current) update_2d_buttons();
    upload_data( buff );
    // I think this was leftover from Nicolet
    //    update_sw_dwell(); 
    process_data (NULL,NULL);
    draw_canvas( buff );
    redraw = 0;
    last_draw();
  }
  return FALSE;
}


gint upload_and_draw_canvas( dbuff *buff )
{
  if( redraw == 1 ) {
    set_acqn_labels(0); 
    if (upload_buff == current) update_2d_buttons();
    upload_data( buff );
    // I think this was leftover from Nicolet
    //    update_sw_dwell();
    //    fprintf(stderr,"last: %ld\n",data_shm->last_acqn);
    draw_canvas( buff );
    redraw = 0;
    last_draw();
  }

  return FALSE;   //This should prevent the idle function from being called more than once for each installation
}

gint idle_button_up( GtkWidget *button )

{
  //  gdk_threads_enter();
  //fprintf(stderr, "in idle_button_up\n" );
  gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( button ), FALSE );
  //fprintf(stderr,"leaving idle_button up\n");
  //  gdk_threads_leave();
  return FALSE;
}

gint idle_queue( GtkWidget *button )

{
  int valid,bnum;
  //  gdk_threads_enter();

  //  fprintf(stderr,"in idle_queue\n");
  if (queue.num_queued ==0){
    fprintf(stderr,"in idle_queue, but none in queue!\n");

  }
  else{
    valid =  gtk_tree_model_get_iter_first(GTK_TREE_MODEL(queue.list),&queue.iter);
    if (valid != 1){
      fprintf(stderr,"in idle queue but nothing in queue!\n");
      return 0;
    }
    gtk_tree_model_get(GTK_TREE_MODEL(queue.list),&queue.iter,
		       BUFFER_COLUMN,&bnum,-1);
    //    fprintf(stderr,"got buff at top of queue: %i\n",bnum);
    
    gtk_list_store_remove(GTK_LIST_STORE(queue.list),&queue.iter);
    
    
    queue.num_queued -= 1;
    make_active(buffp[bnum]);
    
    set_queue_label();
    //    fprintf(stderr,"num left in queue: %i\n",queue.num_queued);
    
    gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( start_button ), TRUE );
  }
  //  gdk_threads_leave();
  return FALSE;
}

gint reload_wrapper( gpointer data )
{
  //  gdk_threads_enter();
  reload( NULL, NULL );
  redraw = 0;
  last_draw();
  // reload_wrapper only gets called at the end of a sequence when things exit normally

  // check to make sure dwell is done correctly - Obsolete - from Nicolet scope days.
  /*  if (data_shm->time_per_point < 0){
    fprintf(stderr,"Scope didn't return dwell time\n");
  }
  else{
    if( (int) 100*buffp[upload_buff]->param_set.dwell != (int) (100*data_shm->time_per_point*1e6 +1) ){ // how can this ever be?  seems like upload on every scan should pick it up?
      popup_msg("looks like you had the wrong dwell");

      //    fprintf(stderr,"old dwell: %f\n",buffp[upload_buff]->param_set.dwell);
      // fprintf(stderr,"new dwell: %f\n",data_shm->time_per_point*1e6);
      buffp[upload_buff]->param_set.dwell=data_shm->time_per_point*1e6;
      buffp[upload_buff]->param_set.sw=1.0/data_shm->time_per_point;
      if (upload_buff == current) update_sw_dwell();
    }
  }
  */
  //  gdk_threads_leave();
  return FALSE;
}


gint acq_signal_handler()

{
  char sig;
  char result;

  sig = data_shm->acq_sig_ui_meaning;
  
  if( sig != ACQ_LAUNCHED ) data_shm->acq_sig_ui_meaning = NO_SIGNAL;

  switch( sig )
    {
    case  PPROG_ALREADY_RUNNING:
      popup_msg("Acq says it was already running.  This should never happen.  You'll probably have to quit Xnmr, kill acq if it is still running and restart.",TRUE);
      break;
    case ACQ_ERROR:
      popup_msg("Acq wasn't able to create the file",TRUE);
      break;
    case P_PROGRAM_ERROR:
      popup_msg("Pulse Program Error",TRUE);
      break;
    case P_PROGRAM_INTERNAL_TIMEOUT:
      popup_msg("Pulse Program internal timeout - probably there's an infinte loop in it, or the syntax is wrong" ,TRUE);
      break;
    case P_PROGRAM_ACQ_TIMEOUT:
      popup_msg("Pulse Program acq timeout - pulse program didn't start up properly",TRUE);
      break;
    case P_PROGRAM_PARAM_ERROR:
      popup_msg( "Pulse Program Parameter Error - couldn't find a parameter value",TRUE);
      break;
    case P_PROGRAM_RECOMPILE:
      popup_msg("Recompile Error\ntry recompiling your pulse program, else acq/Xnmr version mismatch",TRUE);
      break;

    case TTC_ERROR:
      popup_msg( "TTC Error",TRUE );
      break;

    case DSP_FILE_ERROR:
      popup_msg("Couldn't find filter file for dsp",TRUE );
      break;

    case DSP_INIT_ERROR:
      popup_msg("Error: Couldn't init dsp\nacq not suid?" ,TRUE);
      break;
    case FIFO_READ_ERROR:
      popup_msg("Error reading data from fifo",TRUE );
      break;
    case FIFO_ZERO_ERROR:
      popup_msg( "Got all zeros from Receiver" ,TRUE);
      break;
    case DSP_DGAIN_OVERFLOW:
      popup_msg("dgain setting too high",TRUE );
      break;
    case ACQ_FILE_ERROR:
      popup_msg( "acq couldn't open a file to save in",TRUE );
      break;

    case PULSE_PP_ERROR:
      //      popup_msg( "Pulse Programmer Parallel port error\nMaybe acq isn't suid?",TRUE );
      popup_msg( "Pulse Programmer hardware error, see console",TRUE );
      break;
    case EVENT_ERROR:
      popup_msg("Pulse Program Event Error",TRUE );
      break;
    case PPO_ERROR:
      popup_msg("Pulse Program No PPO error",TRUE );
      break;
    case BGU_ERROR:
      popup_msg("BGU Interface error",TRUE );
      break;
    case RX_ERROR:
      popup_msg("Receiver error, see console",TRUE );
      break;
    case PERMISSION_ERROR:
      popup_msg("Acq failed on iopl\nprobably doesn't have root permissions",TRUE);

    default:
      break;
    }

  switch( sig )
    {
    case NEW_DATA_READY:

      // fprintf(stderr, "xnmr received signal NEW_DATA_READY\n" );
      if (redraw == 0 && draw_time_check()  ){ 
	/* only set it to draw again if its finished from the last time.

	   otherwise, we could potentially get thousands of idle add calls stacked up,
	   and sure, most of them would go by very quickly, but we'd rather not stack them up like that.

	   there is, I think a potential race here.  Since this routine is running as a signal handler,
	   it may call g_idle_add while gtk_main is dealing with idle functions.  To minimize this
	   possibility, and to have Xnmr not eat the cpu, we do the draw_time_check above, that will
	   only schedule another draw 100ms after a previous one. 

	   The onle safe way to do this that I can think of so far would be to have Xnmr run a second 
	   thread, and have the second thread wait for messages from acq.  Sounds like a fairly major 
	   reworking of things here though.
	   
	   Also added the gtk_events_pending check in there - that helps but doesn't quite eliminate the
	   problem - we could still be interrupting the gui trying to add an idle function
	   Ah ha - this causes problems, gtk recognizes thread problem.

	   Ok - did the multi threaded thing. Give it a try for a while.  A second thread is started that 
	   catches signals and nothing else.  So this routine - acq_signal_handler is only
	   ever called within the 'signal' thread.  All it will do is add idle functions.
	   Each idle function *must* be protected from races with gdk_threads_enter and gdk_threads_leave.

	   Adding the idle functions themselves is protected internally. - or maybe doesn't need to be?
	   for future portability (?)  added threads protection to signal thread (sig_handler_thread_routine).
	*/

	redraw = 1;
	
	if( acq_in_progress == ACQ_REPEATING_AND_PROCESSING ) {
	  upload_and_draw_canvas_with_process(buffp[upload_buff]);
	}
	else{
	  upload_and_draw_canvas(buffp[upload_buff]);
	}
      }

      return FALSE;
      break;
    
    
    case ACQ_DONE:
      //      fprintf(stderr, "Xnmr_ipc received signal ACQ_DONE\n" );

      redraw = 1;
      
      switch( acq_in_progress ) {
  
      case ACQ_REPEATING_AND_PROCESSING:
	upload_and_draw_canvas_with_process(buffp[ upload_buff ]);
	break;
      case ACQ_REPEATING:
	upload_and_draw_canvas(buffp[ upload_buff ] );
	break;
      case ACQ_RUNNING:    
	result = acq_in_progress;
	acq_in_progress = ACQ_STOPPED;
	reload_wrapper( buffp[ upload_buff ] );
	set_acqn_labels(0);
	acq_in_progress=result;
	path_strcpy(buffp[upload_buff]->path_for_reload,data_shm->save_data_path);
	// provide user a bell so they know we're done.
	printf("\a");
	fflush(stdout);
	break;
      }
    case ACQ_ERROR:
    case PPROG_ALREADY_RUNNING:
    case P_PROGRAM_ERROR:  //moved these below so if exit with error, we don't bother to upload
    case TTC_ERROR:
    case DSP_FILE_ERROR:
    case DSP_INIT_ERROR:
    case FIFO_READ_ERROR:
    case FIFO_ZERO_ERROR:
    case DSP_DGAIN_OVERFLOW:
    case PULSE_PP_ERROR:
    case EVENT_ERROR:
    case PPO_ERROR:
    case RX_ERROR:
    case BGU_ERROR:
    case P_PROGRAM_PARAM_ERROR:
    case P_PROGRAM_RECOMPILE:
    case P_PROGRAM_INTERNAL_TIMEOUT:
    case P_PROGRAM_ACQ_TIMEOUT:
    case ACQ_FILE_ERROR:
    case PERMISSION_ERROR:
      // This stuff happens for errors and ACQ_DONE stops!
      // provide user a bell so they know we're done with error.
      printf("\a");
      fflush(stdout);
      usleep(250000);
      printf("\a");
      fflush(stdout);

      result = acq_in_progress;
      acq_in_progress = ACQ_STOPPED;
      redraw = 0;  //CM dec 2, 2001 - hopefully this will fix the "failure to draw after stop with
                   // error" bug.
      //send_paths( );
      //send_acqns( );
      //send_params( );

      switch( result ) 
	{
	  //set the appropriate toggle button back to inactive

	case ACQ_RUNNING:
	  if (data_shm->mode == NORMAL_MODE){
	    gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( start_button ), FALSE );
	    // idle_button_up(start_button);
	    //	    fprintf(stderr,"calling idle button up for acq and save\n");
	  }
	  else{
	    gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( start_button_nosave ), FALSE );
	    //	    idle_button_up(start_button_nosave );
	    //	    fprintf(stderr,"calling idle_button up for acq nosave\n");
	  }
	  //	  data_shm->mode = NO_MODE; // do this when the button comes up

	  // if there's experiments in the queue, deal with them in the main thread.
	  if (queue.num_queued > 0) // the low is to ensure that we're last (after the button up)
	    g_idle_add_full(G_PRIORITY_LOW, (GSourceFunc) idle_queue,NULL,NULL);
	  
	  break;
	case ACQ_REPEATING:
	  gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( repeat_button ), FALSE );
	  //idle_button_up( repeat_button );
	  //	  data_shm->mode = NO_MODE;
	  break;
	case ACQ_REPEATING_AND_PROCESSING:
	    gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( repeat_p_button ), FALSE );
	    //	  idle_button_up(repeat_p_button );
	  //	  data_shm->mode = NO_MODE;
	  break;
	default:
	  break;
	}
      return FALSE;
      break;
    case NO_SIGNAL:
      // this happens (I think) if we get queued by acq a second time before we're done with the first.
      //      fprintf(stderr,"xnmr_ipc: got NO_SIGNAL\n");
      return FALSE;
      break;
    default:
      break;
    }
  
  //other signals are not picked up by this routine, but by wait_for_acq instead

  if( data_shm->acq_sig_ui_meaning != ACQ_LAUNCHED ) {
    fprintf(stderr, "Xnmr_ipc received an unidentified signal %d\n", data_shm->acq_sig_ui_meaning );
    data_shm->acq_sig_ui_meaning = NO_SIGNAL;
  }

  return FALSE;
}

int upload_data( dbuff* buff )    //uploads the shm data to the active buffer, replacing existing data

{
  int i;
  char s[UTIL_LEN];

  // fprintf(stderr, "uploading data to buffer %d\n", upload_buff );
  /*  if( data_shm->time_per_point > 0 ){
    //    fprintf(stderr, "xnmr_ipc: upload: uploading dwell time to buffer %d\n", upload_buff );
    buff->param_set.dwell=data_shm->time_per_point*1000000;
    buff->param_set.sw=(long int)1.0/data_shm->time_per_point;
  }
  */

  buff->phase0_app = 0;
  buff->phase1_app = 0;
  buff->phase20_app = 0;
  buff->phase21_app = 0;

  buff->flags &= ~FT_FLAG; //turn off ft flag
  buff->ct = data_shm->ct;

 
  if (buff->win.ct_label != NULL){
    snprintf(s,UTIL_LEN,"ct: %li",buff->ct);
    gtk_label_set_text(GTK_LABEL(buff->win.ct_label),s);
  }

  // grab channels
  set_ch1(buff,data_shm->ch1);
  set_ch2(buff,data_shm->ch2);


  // check to make sure the user didn't resize the buffer !
  if (data_shm->npts != buff->npts  || buff->npts2 != 1){
    buff_resize(buff,data_shm->npts,1);
    if (current_param_set == acq_param_set)
      update_npts(data_shm->npts);
  }
  for( i=0; i<buff->npts*2; i++ ){
    buff->data[i] = (float)  data_shm->data_image[i];
    // fprintf(stderr,"%li %f %f\n",data_shm->data_image[i],buff->data[i], (float )data_shm->data_image[i]);
  }

  // fprintf(stderr, "upload completed\n" );

  return 0;
  }

#ifndef MINGW

gint release_ipc_stuff()
{
  //  fprintf(stderr, "release_ipc_stuff, acq_in_progress is %i\n",acq_in_progress );

  if( acq_in_progress == ACQ_STOPPED )
    {
      end_acq();
      wait( NULL );
    }
  release_shm();
  return 0;
}
#endif

void last_draw(){
  struct timezone tz;
  gettimeofday(&last_time,&tz);
}

gint draw_time_check(){
  struct timezone tz;
  long long tdiff;

  // the minimum time between frames is 100ms.
  // keep tdiff in ms.
  gettimeofday(&current_time,&tz);
  tdiff = (current_time.tv_sec-last_time.tv_sec)*1000LL
    +(current_time.tv_usec-last_time.tv_usec)/1000LL;
  if (tdiff > 100LL) return TRUE;
  else return FALSE;
}

