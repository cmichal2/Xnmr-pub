/* shm_prog.h
 *
 * This file defines the shared memory parameters and
 * the data structures in the shared memory
 * 
 * Part of the Xnmr software project
 *
 * UBC Physics
 * April, 2000
 * 
 * written by: Scott Nelson, Carl Michal
 */
#include <stdbool.h>
#ifndef SHM_PROG_H
#define SHM_PROG_H

#include <sys/types.h>
#include <stdint.h>

#include "/usr/share/Xnmr/config/h_config-duesdr.h"

#define PPROG_VERSION "0.99.6 June 23, 2025"

/*
 * memory related defs
 */

#define PROG_SHM_KEY 0x43
#define MSG_KEY 0x49

/*
 * Flag defs
 */

#define CLEAN 0
#define DIRTY 1

#define READY 1
#define NOT_READY 0

/*
 * This shared memory structure is used by acq_proc and pprog to share pprog data
 * We don't want the UI process to touch this
 */


typedef struct gradprog{
  unsigned char gradopcode[MAX_GRAD_EVENTS+1]; // same low bits as for due
  uint32_t gradopinst[MAX_GRAD_EVENTS+1]; // same low bits as for due
  uint16_t x[MAX_GRAD_EVENTS+1],y[MAX_GRAD_EVENTS+1],z[MAX_GRAD_EVENTS+1];
  bool is_grad_event[MAX_GRAD_EVENTS+1]; // if its just a start loop or start sub etc, but not actually a loop?
  int gradevents; // how many events in the program
} gradprog_t;

struct prog_shm_t {
  due_prog_t due_prog[NUM_BOARDS];
  int outputs[NUM_BOARDS][MAX_EVENTS+1];
  int opcodes[MAX_EVENTS+1];
  uint64_t opinst[MAX_EVENTS+1];
  int subids[MAX_EVENTS+1];
  uint32_t times[MAX_EVENTS+1];                // times for each events - in DUE pp clock ticks
  uint64_t prog_dur_in_rx_stamps; // length of th current program, measured in rx stamps
  uint64_t prog_dur_in_due_ticks; // length of program, in due clock ticks (50MHz).
  uint64_t rx_stamp_count; // length of all previous programs, measured in rx stamps. 0 was at start of first real program.
  char rx_on[NUM_RX][MAX_EVENTS+1]; // is rx on during this event?
  uint64_t rx_staging_start_time[NUM_RX][MAX_EVENTS+1]; // collapsed version of rx events. start times only
  uint64_t rx_staging_duration[NUM_RX][MAX_EVENTS+1]; // duration of corresponding events. STILL IN DUE ticks.
  uint32_t rx_staging_num_events[NUM_RX]; // how many events are there
  char downloaded;  // set true by acq after a sequence has been downloaded.
  char begun;      // set true by begin, false by ready, checked by event and go_back
  char use_bgu;
  unsigned int no_events;                 //The number of events in the program
  unsigned int event_error;              // if we get an error during an event call = 1
  unsigned int got_ppo;                  // if the program has no PPO in it, this stays 0
  unsigned char board_clean[ NUM_BOARDS ];   //Stores the clean or dirty status of the chips
  unsigned char prog_ready;               //A Flag indicating whether the program is ready
  unsigned char is_noisy; // a flag that indicates this is a noise spectrscopy type sequence. See  CHANGELOG
  unsigned int noisy_start_pos; // where we return to when doing a noisy loop.
  char version[VERSION_LEN];
  char do_synth_setup;
  
  gradprog_t gradprog[2]; // two of these so one can fill while the other is executing.
  
  int txprogno,etxprogno; // which is filling, which is executing.
  uint64_t rxpoints[NUM_RX],totalpoints;
  double txfreqs[NUM_RX]; // this is frequency as requested from UI

  double rx_sw[NUM_RX];
  float rx_phase[NUM_RX];
  int rx_gains[NUM_RX];
  char stop_rx;
  char is_first_real_prog;
  char is_last_real_prog;
  uint64_t last_dacA, last_dacB; // these kinda only need to be 32 bits each. the dacs hold dac 1 and 2. 1 is unshifted, 2 is up 16 bits.
  uint64_t last_altA, last_altB;

};


#if __GLIBC__ >= 2
 #if __GLIBC_MINOR__ >=2
struct msgbuf {
  long mtype;     
  char mtext[1]; 
};
 #endif
#endif


#endif











