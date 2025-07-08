/* acq.h
 *
 * Xnmr Software Project,
 *
 * UBC Physcis,
 * April, 2000
 * 
 * written by: Scott Nelson, Carl Michal
 */

/*
 * possible states of variable done
 */

#define NOT_DONE 0
#define NICE_DONE 1
#define ERROR_DONE -1
#define KILL_DONE -2
#define THREAD_EXIT -3
// last one for lime

volatile extern char done;

int init_shm();
int init_signals();
int init_msgs();
  //These three methods setup some IPC structures

int init_sched();
  //This method set up acq to run with run time priority

void ui_signal_handler();
  //This is the signal handler method for communcation with the user interface

int send_sig_ui( char sig );
  //This method performs some error checking and sends the specified signal to the UI process

void pprog_ready_timeout();
  //This method is called when a timeout occurs

int wait_for_pprog_msg( );
  //blocks until a message is recieved from the pprog process

int start_pprog();
  // forks and launches the pulse program specified in the shared mem.

int run();
  // This is the main acq loop.  Manages NMR experiment events.

int accumulate_data( int64_t * buffer );
  // accumulated uploaded data into the shared memory with the correctly applied phase.

void shut_down();
  // terminates acq and pulse program processes

void release_mem();
  // detaches and marks IPC strucutures for removal

void tell_xnmr_fail();
  // tells the ui if acq didn't start up right...
int write_param_file(char *fileN);

double pp_time();  // calculates the duration of the pulse program
double pp_time_new();  // calculates the duration of the pulse program
double ppo_time(); // calculates the duration of last event in pulse program

