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

#ifndef SHM_PROG_H
#define SHM_PROG_H

#include <sys/types.h>

#include "/usr/share/Xnmr/config/h_config.h"

#define PPROG_VERSION "0.99.4 Sep 17, 2017"

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

typedef unsigned char prog_image_t[ NUM_CHIPS ][ MAX_EVENTS + 1 ];  
// put a spare event in ther just in case

struct prog_shm_t {
  prog_image_t prog_image;                 //This is the pulse program
  char phase;                             //This is the phase shift for accumulating data 
  char downloaded;  // set true by acq after a sequence has been downloaded.
  char begun;      // set true by begin, false by ready, checked by event and go_back
  unsigned int no_events;                 //The number of events in the program
  unsigned int event_error;              // if we get an error during an event call = 1
  unsigned int got_ppo;                  // if the program has no PPO in it, this stays 0
  unsigned char chip_clean[ NUM_CHIPS ];   //Stores the clean or dirty status of the chips
  unsigned char prog_ready;               //A Flag indicating whether the program is ready
  unsigned char is_noisy; // a flag that indicates this is a noise spectrscopy type sequence. See  CHANGELOG
  unsigned int noisy_start_pos; // where we return to when doing a noisy loop.
  unsigned int bgu_emergency_address; // program event number where we'll jump to in case of a KILL event to reset an gradients
  char version[VERSION_LEN];
};

#define TIME_OF(a) prog_shm->prog_image[0][a] + (prog_shm->prog_image[1][a] << 8) + (prog_shm->prog_image[2][a] << 16) + (((unsigned long long)prog_shm->prog_image[3][a]) << 24)

#if __GLIBC__ >= 2
 #if __GLIBC_MINOR__ >=2
struct msgbuf {
  long mtype;     
  char mtext[1]; 
};
 #endif
#endif


#endif











