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

#include "/usr/share/Xnmr/config/h_config-duelime.h"

#define PPROG_VERSION "0.99.5 Dec 15, 2017"

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


#define REC_ACCUM 2 // 2 used explicitly in adepp.c ( shame )
#define REC_STREAM 1 // also, 0 is in noisy mode

// put a spare event in ther just in case
typedef struct mytxprog {
  unsigned char txopcodes[NUM_TX][MAX_TX_EVENTS+1]; // 16 = transmit event, 32= set offset freq. 0 = nothing, low bits are same as for due. will need RX events in there too!
  uint32_t txopinst[NUM_TX][MAX_TX_EVENTS];
  uint32_t txparams[NUM_TX][MAX_TX_EVENTS+1][3];// holds I and Q values or offset freq, and rx phase.
  uint64_t txtimes[NUM_TX][MAX_TX_EVENTS+1];
  int txevents[NUM_TX]; // number of events in the table for the first pass of tx setting events.
} txprog_t;

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
  uint32_t opinst[MAX_EVENTS+1];
  int subids[MAX_EVENTS+1];
  uint32_t times[MAX_EVENTS+1];                // times for each events - in DUE pp clock ticks
  char downloaded;  // set true by acq after a sequence has been downloaded.
  char begun;      // set true by begin, false by ready, checked by event and go_back
  char use_bgu;
  char wait_for_sync; // tells tx thread if its the start of a new acquisition, and should do the sync thing.
  unsigned int no_events;                 //The number of events in the program
  unsigned int event_error;              // if we get an error during an event call = 1
  unsigned int got_ppo;                  // if the program has no PPO in it, this stays 0
  unsigned char board_clean[ NUM_BOARDS ];   //Stores the clean or dirty status of the chips
  unsigned char prog_ready;               //A Flag indicating whether the program is ready
  unsigned char is_noisy; // a flag that indicates this is a noise spectrscopy type sequence. See  CHANGELOG
  unsigned int noisy_start_pos; // where we return to when doing a noisy loop.
  char version[VERSION_LEN];
  char do_synth_setup;
  unsigned char receiver_model; // live stream receiver  = 1, accumulate + download = 2
  
  txprog_t txprog[2]; // two of these so we can fill one while the other is executing.
  gradprog_t gradprog[2]; // two of these so one can fill while the other is executing.
  
  int txprogno,etxprogno; // which is filling, which is executing.
  uint64_t rxpoints[NUM_TX],totalpoints;
  double txfreqs[NUM_TX];
  double txncofreqs[NUM_TX];
  int txgains[NUM_TX];
  int lime_sync_event[NUM_TX];
  double rx_sw[NUM_TX];
  float rx_phase[NUM_TX];
  int rx_gains[NUM_TX];
  char stop_lime;
  char is_first_real_prog;
  char is_last_real_prog;
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











