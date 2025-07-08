/* p_signals.h
 *
 * This file defines signals for Pulse Program software
 *
 * The signals are organized by their receiver, not their sender
 *
 * Message meanings are carried on the message type field, not in the message text
 */

#ifndef P_SIGNALS_H
#define P_SIGNALS_H


// whenever signal numbers in here change, the version numbers in shm_data.h and shm_prog.h 
// should also change


/*
 * The actual Signals
 */

#define SIG_UI_ACQ SIGUSR1

/*
 * Signal meanings used by all modules
 */

#define NO_SIGNAL 0

/*
 * Acquire Process signal meanings
 */

#define ACQ_START 1
#define ACQ_STOP 2
#define ACQ_KILL 3

/*
 *  UI signal meanings
 */

#define NEW_DATA_READY 4
#define ACQ_LAUNCHED 5
#define ACQ_LAUNCH_FAIL 6
#define ACQ_DONE 7
#define ACQ_ERROR 8
#define TTC_ERROR 9
#define DSP_FILE_ERROR 10
#define DSP_INIT_ERROR 11
#define PULSE_PP_ERROR 12
#define FIFO_READ_ERROR 13
#define DSP_DGAIN_OVERFLOW 14
#define EVENT_ERROR 25
#define FIFO_ZERO_ERROR 26
#define PPO_ERROR 27
#define BGU_ERROR 30
#define RX_ERROR 31
/*
 * Message meanings (types)
 */


#define P_PROGRAM_ERROR 15
#define P_PROGRAM_CALC 16
#define P_PROGRAM_END 17
#define P_PROGRAM_READY 18       //These can also be sent to UI as signals
#define P_PROGRAM_LAUNCHED 19
#define P_PROGRAM_PARAM_ERROR 20
#define P_PROGRAM_ACQ_TIMEOUT 21
#define P_PROGRAM_INTERNAL_TIMEOUT 22
#define ACQ_FILE_ERROR 23
#define P_PROGRAM_RECOMPILE 24
#define PPROG_ALREADY_RUNNING 28
#define PERMISSION_ERROR 29
#endif




