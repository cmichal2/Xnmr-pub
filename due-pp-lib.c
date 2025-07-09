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

/*

  pulse program assembler for Due pulse programmer.
  port C pins available C1-9, C12-19, 21-26 and 28-30 =9+8+6+3 = 26 pins
  could use A: 0-4 6-7 8-9 (RX0, TX0), 10-29 ?  = 5+2+20 = 29 if give up RX/TX
  A.21 and C.30 are likely hard to use - hooked to LEDs, no headers.

  (8/9 = Uart, could maybe use, 21 maybe used for usb led?
  portB 12-21, 25-27, port D0-10
  A.29 and C.26 are same pin
  A.28 and C.29 are same pin  

  stick with port C for now.

  so - we have 25 C pins, 28 A pins but 2 that are shared so 26/27 or 25/28 or 24/29   
 

C1-9, 12-19, 21-26 and 28-30. These are:
      C.0 =           A.0 = D69 CANTX0
      C.1 = D33       A.1 = D68 CANRX0
      C.2 = D34       A.2 = A7 = D61
      C.3 = D35       A.3 = A6 = D60
      C.4 = D36       A.4 = A5 = D59
      C.5 = D37       A.5 =
      C.6 = D38       A.6 = A4 = D58
      C.7 = D39       A.7 = D31
      C.8 = D40       A.8 = D0 (RX0 no go?)
      C.9 = D41       A.9 = D1 (TX0 no go?)
      C.10 =          A.10 = D19
      C.11 =          A.11 = D18
      C.12 = D51      A.12 = D17
      C.13 = D50      A.13 = D16
      C.14 = D49      A.14 = D23
      C.15 = D48      A.15 = D24
      C.16 = D47      A.16 = A0 = D54
      C.17 = D46      A.17 = D70
      C.18 = D45      A.18 = D71
      C.19 = D44      A.19 = D42
      C.20 =          A.20 = D43
      C.21 = D9       A.21 = D73 (not easy)
      C.22 = D8       A.22 = A3 = D57
      C.23 = D7       A.23 = A2 = D56
      C.24 = D6       A.24 = A1 = D55
      C.25 = D5       A.25 = D74 (MISO)
      C.26 = D4/D87   A.26 = D75 (MOSI)
      C.27 =          A.27 = D76 (SCK)
      C.28 = D3       A.28 = D77/D10 
      C.29 = D10/D77  A.29 = D87/D4
      C.30 = D72 (not easy)
      C.31 =

pins D.0 to D.3 (D25-D28) are ID pins to identify the board.

  */

#include "due-pp-lib.h"
#include <math.h>   						       
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdio.h>
#include <string.h>
#include <sys/file.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>

#define BUFFLEN 80
#define NUM_OPCODES 14

#define START_LOOP 0
#define END_LOOP 1
#define BRANCH 2
#define EXIT 3
#define SUB_START 4
#define SUB_END 5
#define EXT_TRIG 6
#define TRIG_MAX 7
#define WRITE_DACS 8
#define WRITE_ALT 9
#define SWAP_TO_ALT 10
#define SWAP_TO_DEFAULT 11
#define SWAP_TO_DACS 12
#define WRITE_DEFAULT 13
#define CALL_SUB 255


// branch and exit assigned to 15?
// this isn't a great system since the minimums don't necessarily line up
// with opcodes. Here the NUM_OPCODES element is for an ordinary event,
// and NUM_OPCODES+1 is for CALL_SUB. EXT_TRIG and TRIG_MAX
// have restrictions on both the event themselves and the preceding.
// Some of these are for the events preceding. Take the bigger of the two and
// use for both.
int min_ticks[NUM_OPCODES+2] = {20,20,20,25,0,25,20,25,25,25,25,25,25,20,10,25};

#define STATE_INITIALIZED 1
#define STATE_FINALIZED 2
#define STATE_EXITED 3


// This is the maximum length of an unrolled loop in the flash program:
#define MAX_QUE_LEN 12000

void checksum_data(unsigned char *c1, unsigned char *c2, int len, unsigned int *data) {
  // calculate checksums for the data. Based on what Bruker does in SBS.
  unsigned int i, ch1 = 0, ch2 = 0;
  //  unsigned char *cdata;
  /* d0 d1 d2 d3 d4 d5 ... dn
      1  2  3  4  5  6 ...  n
     n+1 n n-1 n-2 ...      2

    for the first checksum, multiply top row elements by middle row elements. Sum, and keep low 8 bits.
    For the second, use the lower row.
  */
  //  cdata = (unsigned char *) data;
  /*  for (i = 0; i < len*4; i++) {
    ch1 += (i+1)*cdata[i];
    ch2 += (len*4+1-i)*cdata[i];
    } */
  for (i = 0; i < len; i++) {
    ch1 += (i+1)*data[i];
    ch2 += (len+1-i)*data[i];
  }

  *c1 = ch1 & 0xff;
  *c2 = ch2 & 0xff;
}

unsigned int due_shift_bits(unsigned int inputs, unsigned int port){
  // look at active port, shift accordingly
  unsigned int outputs=0;
  switch (port){
  case DEFAULT_PORT:
    //this assumes port C, missing bits 0, 10, 11, 20, 27, 30, 31
    // 25 useful bits.
    /*
      0-8   ->  1-9
      9-16  -> 12-19
      17-22 -> 21-26
      23-24 -> 28-29
    */

    //31-28 27-24 23-20 19-16 15-12 11-8 7-4 3-0
    outputs |= ((inputs & 0x01800000) << 5);
    outputs |= ((inputs & 0x007e0000) << 4);
    outputs |= ((inputs & 0x0001fe00) << 3);
    outputs |= ((inputs & 0x000001ff) << 1);
    //    printf("shifting bits for default port was: 0x%x, now: 0x%x\n",inputs,outputs);
    return outputs;
    break;
  case ALT_PORT:
    // this assumes port A, missing bits: 5, 8, 9, 21, 28-31.
    //24 useful bits.
    /*
    0-4   -> 0-4
    5-6   -> 6-7
    7-17  -> 10-20
    18-23 -> 22-27
    */
    outputs |= ((inputs & 0x00fc0000) << 4 );
    outputs |= ((inputs & 0x0003ff80) << 3 );
    outputs |= ((inputs & 0x00000070) << 1 );
    outputs |= ((inputs & 0x0000000f));
    //    printf("shifting bits for alt port was: 0x%x, now: 0x%x\n",inputs,outputs);
    return outputs;
    break;
  case DAC_PORT:
    // do nothing
    return inputs;
    break;
  default:
    printf("duepp: in shift bits with unknown port to shift for, doing nothing!\n");
    printf("This can happen if you have a subroutine that is never called\n");
    return inputs;
    
  }  
  //  return inputs;
}


int do_play_queue(due_prog_t *program, uint32_t whats_next){
  /* this fills in the jump address for the events that have been queued, these events end
with a function determined by whats_next: either a loop start, a loop end, a bare branch, or
a branch to exit. */
  // need to deal with CALL_SUB and SUB_END.
  
  if (program->queued_events > 12000 ){
    printf("duepp program->queued_events out of range: %i\n",program->queued_events);
    program->error = 1;
    return -1;
  }
  if (program->queued_events > 12000){
    printf("duepp got program->queued_events = %i, is > 12000, can't handle\n",program->queued_events);
    program->error = 1;
    return -1;
  }

  if (program->queued_events == 0){
    if (program->dpos > MAXDATA-2){
      printf("duepp: program length overrun\n");
      program->error = 1;
      return -1;
    }
    program->data[program->dpos] = (whats_next << 16);
    program->dpos += 1;
    // should only ever get: START_LOOP, EXIT, SUB_END, SWAP_TO_ALT, SWAP_TO_DEFAULT, CALL_SUB, SWAP_TO_DACS
  }
  else if (program->queued_events > 0){ // these have direct endings, optimized for speed.
    if (whats_next <= BRANCH){
      program->data[program->queue_pos] = (whats_next << 16 ) | program->queued_events;
      program->queued_events = 0;
    }
    else if (whats_next < NUM_OPCODES){ // all these do a branch to get to the operation.
      if (program->dpos > MAXDATA-2){
	printf("duepp: program length overrun\n");
	program->error = 1;
	return -1;
      }
      program->data[program->queue_pos] = (BRANCH << 16 ) | program->queued_events;
      program->queued_events = 0;
      program->data[program->dpos] = (whats_next << 16 ) ;
      program->dpos += 1;
    }
    else if (whats_next == CALL_SUB){// this one's special
      program->data[program->queue_pos] = (BRANCH <<16) | program->queued_events;
      // call_sub gets three more instructions inserted
      //      printf("queue for call_sub, queued events: %i\n",program->queued_events);
      program->queued_events = 0; 
    }
    else{
      printf("duepp: play_queue got unknown what's next\n");
      program->error = 1;
      return -1;
    }
  }
  return 0;
}

int play_queue(due_prog_t *program, int whats_next){
  int rval = 0;
  int i,dposi,j,cache_queued_events;
  if (program->queued_events <= MAX_QUE_LEN){
    return do_play_queue(program, whats_next);
  }
  // ok, so we have more than the max number of events,
  // (it can never be more than double though) Look for an event that's long enough
  // to break the queue up.
  if (program->dpos > MAXDATA-2){
    printf("duepp: not enough room left to split events\n");
    program->error = 1;
    return -1;
  }
  for (i=MAX_QUE_LEN-1;i >= program->queued_events-MAX_QUE_LEN-1;i--){
    // program->dpos points at the next event, each event has two entries - an output word and a timer delay
    dposi = program->dpos-2*program->queued_events+2*i; // points to the output word of the i'th event in the queue.
    if ( program->data[dposi+1] >= min_ticks[BRANCH] ) // use a 20 tick minimum for breaking up.
      break;
  }
  // i is the index of the last event in the first queue.
  if ( i >= program->queued_events-MAX_QUE_LEN-1 ){
    i+=1; // now i is the first event in the second queue.
    // slide all events from i to the end down a slot
    for (j = program->queued_events; j>=i ; j--){
      dposi = program->dpos-2*program->queued_events+2*j;
      program->data[dposi+2]=program->data[dposi+1];
      program->data[dposi+1]=program->data[dposi];
      // that leaves a slot just before the i'th event to stick in a new jump address
    }
    printf("duepp: breaking %i into two queues of length: %i and %i\n",program->queued_events,i,program->queued_events-i);
    cache_queued_events = program->queued_events;
    program->queued_events = i;
    rval = do_play_queue(program, BRANCH);
    if (rval < 0) return rval;
    // then do the rest
    program->queue_pos = program->dpos-2*cache_queued_events + 2*i; 
    program->queued_events = cache_queued_events-i;
    do_play_queue(program, whats_next);
    if (rval < 0) return rval;
    program->dpos += 1; // add one for the new jump address 
  }
  else{
    printf("duepp: Got %i events, more than %i, and couldn't find a spot to break it up\n",program->queued_events,MAX_QUE_LEN);
    program->error = 1;
    return -1;
  }
  return 0;
}

int due_add_event(due_prog_t *program, unsigned int outputs, unsigned  int ticks) {
  //  printf("due_add_event with outputs: 0x%x, ticks: %i\n",outputs,ticks);
  if (ticks < min_ticks[NUM_OPCODES]){
    printf("duepp: Got due_add_event with %i ticks. Must be at least: %i\n",ticks,min_ticks[NUM_OPCODES]);
    program->error = 1;
    return -1;
  }
  if (program->state != STATE_INITIALIZED && program->in_sub == 0){
    printf("duepp: to add an event, the program must be in STATE_INITIALIZED or defining a subroutine\n");
    program->error = 1;
    return -1;
  }
  
  if (program->queued_events == 0){
    if (program->dpos > MAXDATA-2){
      printf("duepp: program length overrun\n");
      program->error = 1;
      return -1;
    }
    program->queue_pos = program->dpos;
    program->dpos += 1;
  }
  program->queued_events += 1;
  
  program->events += 1;
  if (program->dpos > MAXDATA-3){
    printf("duepp: program length overrun\n");
    program->error = 1;
    return -1;
  }
  if (program->auto_shift) outputs = due_shift_bits(outputs,program->active_port);
  program->data[program->dpos] = outputs;
  program->dpos += 1;
  program->data[program->dpos] = ticks;
  program->dpos += 1;
  program->last_ticks = ticks;
  //  printf("due add_event, putting outputs 0x%x in event: %i\n",outputs,program->dpos);
  return 0;
}

int due_start_loop(due_prog_t *program, unsigned int loops,unsigned int outputs, unsigned int ticks) {
  int rval;
  if (program->last_ticks < min_ticks[START_LOOP]){
    printf("duepp: Got start_loop with %i ticks in previous event. Must be at least: %i\n",program->last_ticks,min_ticks[START_LOOP]);
    program->error = 1;
    program->error = 1;
    return -1;
  }
  if (program->state != STATE_INITIALIZED && program->in_sub == 0){
    printf("duepp: to start a loop, the program must be in STATE_INITIALIZED, or must be defining a subroutine\n");
    program->error = 1;
    return -1;
  }

  // if there are queued events, do them
  rval = play_queue(program, START_LOOP);
  if (rval < 0 ) return rval;
  if (program->dpos > MAXDATA-2){
    printf("duepp: program length overrun\n");
    program->error = 1;
    return -1;
  }
  program->data[program->dpos] = loops;
  program->dpos += 1;
  program->loop_level +=1;
  return due_add_event(program, outputs, ticks);
  
}

int due_end_loop(due_prog_t *program, unsigned int outputs, unsigned int ticks) {
  int rval;
  if (ticks < min_ticks[END_LOOP]){
    printf("duepp: Got end_loop with %i ticks. Must be at least: %i\n",ticks,min_ticks[END_LOOP]);
    program->error = 1;
    return -1;
  }
  if (program->state != STATE_INITIALIZED && program->in_sub == 0){
    printf("duepp: to end a loop, the program must be in STATE_INITIALIZED, or must be defining a subroutine\n");
    program->error = 1;
    return -1;
  }
  if (program->loop_level < 1){
    printf("duepp: Got end_loop, but no loop to end!\n");
    program->error = 1;
    return -1;
  }
  rval = due_add_event(program, outputs, ticks);
  if (rval < 0 ) return rval;
  rval = play_queue(program, END_LOOP);
  program->loop_level -=1;
  return rval;
}

int due_init_program(due_prog_t *program,char auto_shift){
  int i;
  program->auto_shift = auto_shift;
  program->active_port = DEFAULT_PORT;
  program->dpos = 0;
  program->events = 0;
  program->queued_events = 0; // how many events to do in the continuous sequence loop.
  program->state = STATE_INITIALIZED;
  program->in_sub = 0;
  program->error = 0;
  for (i=0;i<MAXSUB;i++){
    program->sub_table[i]=0;
    program->sub_entry_port[i] = -1;
  }
  return 0;
}

int due_exit_program(due_prog_t *program){
  int rval;
  if (program->last_ticks < min_ticks[EXIT]){
    printf("duepp: Got exit_program with %i ticks in previous event. Must be at least: %i\n",program->last_ticks,min_ticks[EXIT]);
    program->error = 1;
    return -1;
  }

  if (program->state != STATE_INITIALIZED ){
    printf("duepp: got exit_program, but the program must be in STATE_INITIALIZED\n");
    program->error = 1;
    return -1;
  }
  if (program->loop_level != 0){
    printf("duepp: Loop starts and ends don't match, expect trouble!\n");
    program->error = 1;
    return -1;
  }
  rval = play_queue(program, EXIT);
  if (rval < 0 ) return rval;
  program->state = STATE_EXITED;
  return 0;
}

int due_swap_to_alt(due_prog_t *program, unsigned int outputs, unsigned int ticks){
  int rval;
  if (program->last_ticks < min_ticks[SWAP_TO_ALT]){
    printf("duepp: Got swap_to_alt with %i ticks in previous event. Must be at least: %i\n",program->last_ticks,min_ticks[SWAP_TO_ALT]);
    program->error = 1;
    return -1;
  }
  if (program->state != STATE_INITIALIZED && program->in_sub == 0){
    printf("duepp: to swap_to_alt, the program must be in STATE_INITIALIZED, or must be defining a subroutine\n");
    program->error = 1;
    return -1;
  }
  
  rval = play_queue(program, SWAP_TO_ALT);
  if (rval < 0 ) return rval;
  if (program->active_port == ALT_PORT){
    printf("duepp: WARNING: got swap_to_alt when port was already alt?\n");
  }
  program->active_port = ALT_PORT;
  return due_add_event(program, outputs,ticks);
}

int due_swap_to_default(due_prog_t *program, unsigned int outputs, unsigned int ticks){
  int rval;
  if (program->last_ticks < min_ticks[SWAP_TO_DEFAULT]){
    printf("duepp: Got swap_to_default with %i ticks in previous event. Must be at least: %i\n",program->last_ticks,min_ticks[SWAP_TO_DEFAULT]);
    program->error = 1;
    return -1;
  }
  if (program->state != STATE_INITIALIZED && program->in_sub == 0){
    printf("duepp: to swap_to_default, the program must be in STATE_INITIALIZED, or must be defining a subroutine\n");
    program->error = 1;
    return -1;
  }
  rval = play_queue(program, SWAP_TO_DEFAULT);
  if (rval < 0 ) return rval;
  if (program->active_port == DEFAULT_PORT){
    printf("duepp: WARNING: got swap_to_default when port was already default?\n");
  }
  program->active_port = DEFAULT_PORT;
  return due_add_event(program, outputs,ticks);
}
int due_swap_to_dacs(due_prog_t *program, unsigned int dac0, unsigned int dac1, unsigned int ticks){
  int dacword,rval;
  if (program->last_ticks < min_ticks[SWAP_TO_DACS]){
    printf("duepp: Got swap_to_dacs with %i ticks in previous event. Must be at least: %i\n",program->last_ticks,min_ticks[SWAP_TO_DACS]);
    program->error = 1;
    return -1;
  }
  if (program->state != STATE_INITIALIZED && program->in_sub == 0){
    printf("duepp: to swap_to_dacs, the program must be in STATE_INITIALIZED, or must be defining a subroutine\n");
    program->error = 1;
    return -1;
  }
  dacword = dac0 | dac1<<16 | 1<<28;
  rval = play_queue(program, SWAP_TO_DACS);
  if (rval < 0 ) return rval;
  if (program->active_port == DAC_PORT){
    printf("duepp: WARNING: got swap_to_dacs when port was already dacs?\n");
  }
  program->active_port = DAC_PORT;
  return due_add_event(program, dacword,ticks);
}

// There are a few things that need to get filled in.
// In the subroutines themselves, the first word will contain a typical branch to be resolved.
// that needs to get resolved, then copied in to the callers branch spot
// the SUB_END instructions also need to get resolved
// Finally, we need to calculate the data offsets and give them to the caller.

// the sub_table contains the information needed to do the data offsets.

int due_finalize_program(due_prog_t *program){
  int inst,elements,i;
  // in here we resolve subroutines.
  if (program->state != STATE_EXITED || program->in_sub == 1){
    printf("duepp: to finalize program, the program must be in STATE_EXITED, and must not be defining a subroutine\n");
    program->error = 1;
    return -1;
  }
  if (program->loop_level != 0){
    printf("duepp: Loop starts and ends don't match!\n");
    program->error = 1;
    return -1;
  }

  if (program->state != STATE_EXITED){
    printf("duepp: got finalize_program, but haven't yet exited\n");
    program->error = 1;
    return -1;

  }
  if (program->in_sub){
    printf("duepp: got finalize_program, but still inside a subroutine!\n");
    program->error = 1;
    return -1;
  }
  // now in here, we need to go through and get the subroutine calls
  // ready. We find each subroutine call, copy the number of event and
  // the what's next code in from the subroutine header (which we can
  // locate in the sub_table) we can also calculate the data offset
  inst = 0 ;
  i=0;
  while( i < program->dpos){
      inst = program->data[i]>>16;
      elements = program->data[i] & 0xffff;
      //      printf("got inst: %i, with elements: %i at pos: %i\n",inst,elements,i);
      if (inst == CALL_SUB ){ // elements is the subroutine_id - which we only need to look up the data address.
	// leave program->data[i] alone. run-time resolver will take care of it.
	// 2.  calculate the address for the jump -into the generic branch
	//	printf("got sub call at pos %i ",i);
	if (program->sub_table[elements] == 0){
	  printf("duepp: trying to resolve a subroutine, id: %i, but subroutine not found.\n",elements);
	  program->error = 1;
	  return -1;
	}
	program->data[i+1] = ((program->sub_table[elements]+1) - (i+3))*4; // data offset - point it one past the jump header
	program->data[i+2] = program->data[program->sub_table[elements]]; // copy the jump header from the start of the subroutine.
	//	printf(" data offset is %i, target is: %i, current is: %i\n",program->data[i+1],program->sub_table[elements]+1,(i+3));
	i+=3;
      }
      else{
	i += 1; // advance past the address
	i += 2*elements; // advance past the events
	//		if (inst == START_LOOP || inst == EXIT || inst == WRITE_DACS || inst == WRITE_ALT || inst == WRITE_DEFAULT ) i += 1; // for start loop, advance past the loop count.
	if (inst == START_LOOP || inst == WRITE_DACS || inst == WRITE_ALT || inst == WRITE_DEFAULT  ) i += 1; // for start loop, advance past the loop count.
      }
  }
  program->state = STATE_FINALIZED;
  return 0;
}


// during pulse prog creation, the caller's three words are:
// CALL_SUB<<16 | subroutine_id
// blank
// blank

// after finalize_program, these are replaced with:
// CALL_SUB <<16
// data offset
// what's_next << 16 | events  [ for subroutine_id] - copied 

// run time resolver replaces them with:
// address for start_sub code
// data_offset (leaves alone)
// resolves this as any other event.

int due_call_sub(due_prog_t *program,unsigned  int subroutine_id, unsigned int outputs,unsigned int ticks){
  int rval;
  if (ticks < min_ticks[NUM_OPCODES+1]){
    printf("duepp: Got call_sub with %i ticks. Must be at least: %i\n",ticks,min_ticks[NUM_OPCODES+1]);
    program->error = 1;
    return -1;
  }
  if (program->state != STATE_INITIALIZED && program->in_sub == 0){
    printf("duepp: to call a subroutine, the program must be in STATE_INITIALIZED, or must be defining a subroutine\n");
    program->error = 1;
    return -1;
  }
  if (subroutine_id >= MAXSUB){
    printf("duepp: subroutine id of %i is too big, max is: %i\n",subroutine_id,MAXSUB-1);
    return -1;
}

  if (program->sub_entry_port[subroutine_id] == -1){
    program->sub_entry_port[subroutine_id] = program->active_port;
  }
  else if (program->sub_entry_port[subroutine_id] != program->active_port){
    printf("duepp: WARNING: Got call to subroutine %i with active port %i. Previous call had active port: %i\n",subroutine_id,
	   program->active_port,program->sub_entry_port[subroutine_id]);
  }
  rval = due_add_event(program, outputs,ticks); 
  if (rval < 0) return rval;
  rval = play_queue(program, CALL_SUB);
  // play_queue will insert 
  if (program->dpos > MAXDATA-4){
    printf("duepp: program length overrun\n");
    program->error = 1;
    return -1;
  }
  program->data[program->dpos] = CALL_SUB <<16 | subroutine_id; // 
  // next is the data offset to the start of the subroutine events
  // and after that is the branch address.
  // we don't know enough at this point to fill either of those in.
  program->dpos += 3;
  return rval;


}
// start_sub does not create an event! Call this before created the events in the subroutine.
int due_start_sub(due_prog_t *program, unsigned int subroutine_id){
  if (program->state != STATE_EXITED){
    printf("duepp: in start_sub. Program state needs to be STATE_EXITED\n");
    program->error = 1;
    return -1;
  }
  if (program->in_sub) {
    printf("duepp: subroutine can't be defined inside a subroutine\n");
    program->error = 1;
    return -1;
  }
  if (program->queued_events != 0){
    printf("duepp: got start_sub but had events queued. Shouldn't happen\n");
    program->error = 1;
    return -1;
  }
  if (subroutine_id >= MAXSUB){
    printf("duepp: subroutine id of %i is too big, max is: %i\n",subroutine_id,MAXSUB-1);
    return -1;
}
  // record where the subroutine data starts. This actually points to the jump
  // address that starts the subroutine data section.
  // This address should never be read from this position, it gets copied into the
  // the data stream at the caller. 
  program->sub_table[subroutine_id] = program->dpos;
  // Don't want due_add_event to  leave space for a jump address
  // The in_sub flags tells it not to set program->queue_pos.
  program->in_sub = 1;
  program->in_sub_num = subroutine_id;
  program->active_port = program->sub_entry_port[subroutine_id];
  if (program->active_port == -1){
    printf("duepp: WARNING: port for entry of subroutine %i is -1, indicating the subroutine was never called\n",subroutine_id);
  }
  return 0;
}
int due_return_from_sub(due_prog_t *program, unsigned int outputs, unsigned int ticks){
  int rval;
  if (program->in_sub != 1){
    printf("duepp: got return from sub, but wasn't in a subroutine!\n");
    program->error = 1;
    return -1;
  }

  if (program->sub_entry_port[program->in_sub_num] != program->active_port){
    printf("duepp: WARNING: Subroutine: %i entered with active port: %i. Leaving with active port: %i\n",program->in_sub_num,
	   program->sub_entry_port[program->in_sub_num],program->active_port);
  }

  rval = due_add_event(program, outputs,ticks);
  if (rval < 0) return rval;
  rval = play_queue(program, SUB_END);
  program->in_sub = 0;
  return rval;
}

int due_wait_for_trigger(due_prog_t *program,unsigned  int outputs,unsigned int ticks){
  int rval;
  // waits for an external trigger. Will wait forever. The requested delay
  // starts when the trigger is received. The requested outputs are set
  // just before we start waiting
  if (program->last_ticks < min_ticks[EXT_TRIG]){
    printf("duepp: Got wait_for_trigger with %i ticks in previous event. Must be at least: %i\n",program->last_ticks,min_ticks[EXT_TRIG]);
    program->error = 1;
    return -1;
  }
  if (ticks < min_ticks[EXT_TRIG]){
    printf("duepp: Got wait_for_trigger with %i ticks. Must be at least: %i\n",ticks,min_ticks[EXT_TRIG]);
    program->error = 1;
    return -1;
  }
  if (program->state != STATE_INITIALIZED && program->in_sub == 0){
    printf("duepp: to wait for trigger, the program must be in STATE_INITIALIZED, or must be defining a subroutine\n");
    program->error = 1;
    return -1;
  }
  rval = due_add_event(program, outputs,ticks);
  if (rval < 0) return rval;
  return play_queue(program, EXT_TRIG);
  
}

int due_wait_for_trigger_max(due_prog_t *program,unsigned  int outputs,unsigned int ticks){
  int rval;
  // Waits for an external trigger, but will only wait for a maximum of the
  // requested delay time. There's 1 us delay after the trigger is received.
  // The requested outpus are set just before we start waiting.
  if (program->last_ticks < min_ticks[TRIG_MAX]){
    printf("duepp: Got wait_for_trigger_max with %i ticks in previous event. Must be at least: %i\n",program->last_ticks,min_ticks[TRIG_MAX]);
    program->error = 1;
    return -1;
  }
  if (ticks < min_ticks[TRIG_MAX]){
    printf("duepp: Got wait_for_trigger with %i ticks. Must be at least: %i\n",ticks,min_ticks[TRIG_MAX]);
    program->error = 1;
    return -1;
  }
  if (program->state != STATE_INITIALIZED && program->in_sub == 0){
    printf("duepp: to wait for trigger max, the program must be in STATE_INITIALIZED, or must be defining a subroutine\n");
    program->error = 1;
    return -1;
  }
  rval = due_add_event(program, outputs,ticks);
  if (rval < 0) return rval;
  return play_queue(program, TRIG_MAX);
  
}

int due_write_dacs(due_prog_t *program, unsigned int dac0, unsigned int dac1,unsigned int outputs,unsigned int ticks){
  int rval;
  if (ticks < min_ticks[WRITE_DACS]){
    printf("duepp: Got write_dacs with %i ticks. Must be at least: %i\n",ticks,min_ticks[WRITE_DACS]);
    program->error = 1;
    return -1;
  }
  if (program->state != STATE_INITIALIZED && program->in_sub == 0){
    printf("duepp: to write_dacs, the program must be in STATE_INITIALIZED, or must be defining a subroutine\n");
    program->error = 1;
    return -1;
  }
  if (program->active_port == DAC_PORT){
    printf("duepp: WARNING: Got write dacs while active port is already dacs!\n");
  }
  rval = due_add_event(program, outputs,ticks);
  if (rval < 0) return rval;
  rval =play_queue(program, WRITE_DACS);
  if (program->dpos > MAXDATA-2){
    printf("duepp: program length overrun\n");
    program->error = 1;
    return -1;
  }
  program->data[program->dpos] = dac0 | dac1<<16 | 1<<28;
  program->dpos += 1;
  return rval;

}
int due_write_alt(due_prog_t *program,unsigned  int outputs_alt,unsigned  int outputs,unsigned int ticks){
  // first arg is the outputs for port A. Second arg is for whatever was last swapped to.
  // port A outputs will not be synchronized. Should not be latched.
  int rval;
  if (ticks < min_ticks[WRITE_ALT]){
    printf("duepp: Got write_alt with %i ticks. Must be at least: %i\n",ticks,min_ticks[WRITE_ALT]);
    program->error = 1;
    return -1;
  }
  if (program->state != STATE_INITIALIZED && program->in_sub == 0){
    printf("duepp: to write_alt, the program must be in STATE_INITIALIZED, or must be defining a subroutine\n");
    program->error = 1;
    return -1;
  }
  if (program->active_port == ALT_PORT){
    printf("duepp: WARNING: Got write alt while active port is already alt!\n");
  }
  rval = due_add_event(program, outputs,ticks);
  if (rval < 0) return rval;
  rval = play_queue(program, WRITE_ALT);
  if (program->dpos > MAXDATA-2){
    printf("duepp: program length overrun\n");
    program->error = 1;
    return -1;
  }
  if (program->auto_shift) outputs_alt = due_shift_bits(outputs_alt,ALT_PORT);
  program->data[program->dpos] = outputs_alt;
  program->dpos += 1;
  return rval;
}  

int due_write_default(due_prog_t *program, unsigned int outputs_def,unsigned int outputs,unsigned int ticks){
  // first arg is the outputs for port C. 
  // port C outputs will be synchronized. 
  int rval;
  if (program->last_ticks < min_ticks[WRITE_DEFAULT]){
    printf("duepp: Got write_default with %i ticks in previous event. Must be at least: %i\n",program->last_ticks,min_ticks[WRITE_DEFAULT]);
    program->error = 1;
    return -1;
  }
  if (program->state != STATE_INITIALIZED && program->in_sub == 0){
    printf("duepp: to write_default, the program must be in STATE_INITIALIZED, or must be defining a subroutine\n");
    program->error = 1;
    return -1;
  }
  if (program->active_port == DEFAULT_PORT){
    printf("duepp: WARNING: Got write default while active port is already default!\n");
  }
  rval = play_queue(program, WRITE_DEFAULT); // play out previous events
  if (rval < 0) return rval;
  if (program->dpos > MAXDATA-2){
    printf("duepp: program length overrun\n");
    program->error = 1;
    return -1;
  }
  if (program->auto_shift) outputs_def = due_shift_bits(outputs_def,DEFAULT_PORT);
  program->data[program->dpos] = outputs_def; // stick the new output word in place - it will get latched with the next event.
  program->dpos += 1;
  return due_add_event(program, outputs,ticks);
  
}

 
//this happens on the arduino:
// this is the starting address in flash of our unrolled loops:
#define BASE_ADDR 0x802f8
#define EVSIZE 12					 

void resolve_jumps(due_prog_t *program){
  // program->dpos is how many elements there are in data
  int i,inst=0,elements;
  uint32_t base_addrs[NUM_OPCODES];
  int code_lengths[NUM_OPCODES + 1] = {12000 * EVSIZE, 16 + 12000 * EVSIZE, 20 + 12000 * EVSIZE,
				       4, 2, 10, 8, 50, 58, 10,10,6,6,8,10};

  base_addrs[0] = BASE_ADDR+code_lengths[0];
  for (i=0;i<NUM_OPCODES;i++)
    base_addrs[i] = base_addrs[i-1] + code_lengths[i];
  
  for (i=0;i<NUM_OPCODES;i++)
    printf("duepp: base addrs: %i 0x%x\n",i,base_addrs[i]);
  i=0;
  while( i < program->dpos){
      inst = program->data[i]>>16;
      elements = program->data[i] & 0xffff;
      printf("duepp: got inst: %i, with elements: %i\n",inst,elements);
      if (inst == CALL_SUB){
	// have to do three things in here:
	// 1. set program->data[i] to the sub start address
	program->data[i] = base_addrs[SUB_START] +1;
	// 2.  calculate the address for the jump -into the generic branch
	inst = program->data[i+2] >> 16;
	// i+2 is like a usual one, but don't advance past the elements afterwards.
	elements = program->data[i+2] & 0xffff;
	program->data[i+2] = base_addrs[inst] - EVSIZE*elements +1;
	//3. find the end of the subroutine and stick the SUB_END address in at its end.
	// program->data[i+1] holds the offset, in bytes from the start of the subroutine to the the word following the subroutine call args.

	// advance past the start address, the data offset, and the jump address.
	i+=3;
      }
      else{
	program->data[i] = base_addrs[inst] - EVSIZE* elements + 1; // +1 for bx.
	i += 1; // advance past the address
	i += 2*elements; // advance past the events
	if (inst == START_LOOP || inst == WRITE_DACS || inst == WRITE_ALT ) i += 1; // for start loop or write_DACS, advance past the argument.
	if (inst == WRITE_DEFAULT) i += 2;
      }
  }
}

int due_dump_program(due_prog_t *program){
  int i=0, inst, elements,j;
  if (program->state != STATE_FINALIZED ){
    printf("duepp: WARNING. Got dump_program, but program has not been finalized. Subroutine calls are not complete.\n");
  }
  printf("\nProgram Dump\n");
  while( i < program->dpos){
      inst = program->data[i]>>16;
      elements = program->data[i] & 0xffff;
      switch (inst){
      case CALL_SUB:
	printf("%i CALL SUB: to sub id: %i, data offset: %i, sub data starts at: %i,",i,elements,program->data[i+1]/4,i+program->data[i+1]/4+3);
	inst = program->data[i+2]>>16;
	elements = program->data[i+2] & 0xffff;
	printf(" subroutine starts with call at header in position: %i with %i events\n",i+program->data[i+1]/4+2,elements);
	i+=3;
	break;
      case START_LOOP:
	printf("%i START_LOOP header, %i events\n",i,elements);
	for(j=0;j<elements;j++)
	  printf("%i outputs: 0x%x time: 0x%x\n",i+2*j+1,program->data[i+1+2*j],program->data[i+1+2*j+1]);
	printf("%i START_LOOP with %i iterations\n",i,program->data[i+1+2*elements]);
	//	printf(" start_loop here\n");
	i += 1+2*elements +1; // the header, the elements and the loop counter
	break;
      case END_LOOP:
	printf("%i END_LOOP header, %i events\n",i,elements);
	for(j=0;j<elements;j++)
	  printf("%i outputs: 0x%x time: 0x%x\n",i+2*j+1,program->data[i+1+2*j],program->data[i+1+2*j+1]);
	i += 1 +2*elements; // the header, the elements
	printf("%i END_LOOP\n",i);
	//	printf(" end_loop here\n");
	break;
      case BRANCH:
	printf("%i BRANCH header, %i events\n",i,elements);
	for(j=0;j<elements;j++)
	  printf("%i outputs: 0x%x time: 0x%x\n",i+2*j+1,program->data[i+1+2*j],program->data[i+1+2*j+1]);
	//	printf(" branch here\n");
	printf("%i BRANCH\n",i);
	i += 1 +2*elements; // the header, the elements
	break;
      case EXIT:
	printf("%i EXIT\n",i);
	if (elements > 0) printf("ERROR, got elements > 0: %i\n",elements);
	i += 1;
	break;
      case SUB_START:
	printf("%i SUB_START\n",i);
	if (elements > 0) printf("ERROR, got elements > 0: %i\n",elements);
	i += 1;
	break;
      case SUB_END:
	printf("%i SUB_END\n",i);
	if (elements > 0) printf("ERROR, got elements > 0: %i\n",elements);
	i += 1;
	break;
      case EXT_TRIG:
	printf("%i EXT_TRIG\n",i);
	if (elements > 0) printf("ERROR, got elements > 0: %i\n",elements);
	i += 1;
	break;
      case TRIG_MAX:
	printf("%i TRIG_MAX\n",i);
	if (elements > 0) printf("ERROR, got elements > 0: %i\n",elements);
	i += 1;
	break;
      case WRITE_DACS:
	j = program->data[i+1];
	printf("%i WRITE_DACS with dac vals: %i and %i for dacs %i and %i\n",i,j & 0xfff, (j>>16) & 0xfff, (j >> 12) & 1, (j>>28) & 1 );
	if (elements > 0) printf("ERROR, got elements > 0: %i\n",elements);
	i += 2;
	break;
      case WRITE_ALT:
	j = program->data[i+1];
	printf("%i WRITE_ALT with value: 0x%x\n",i,j);
	if (elements > 0) printf("ERROR, got elements > 0: %i\n",elements);
	i += 2;
	break;
      case SWAP_TO_ALT:
	printf("%i SWAP_TO_ALT\n",i);
	if (elements > 0) printf("ERROR, got elements > 0: %i\n",elements);
	i += 1;
	break;
      case SWAP_TO_DACS:
	printf("%i SWAP_TO_DACS\n",i);
	if (elements > 0) printf("ERROR, got elements > 0: %i\n",elements);
	i += 1;
	break;
      case SWAP_TO_DEFAULT:
	printf("%i SWAP_TO_DEFAULT\n",i);
	if (elements > 0) printf("ERROR, got elements > 0: %i\n",elements);
	i += 1;
	break;
      case WRITE_DEFAULT:
	printf("%i WRITE_DEFAULT with value: 0x%x - gets latched in with next event.\n",i,program->data[i+1]);
	if (elements > 0) printf("ERROR, got elements > 0: %i\n",elements);
	i+=2;
	break;
      default:
	printf("GOT AN UNKNOWN OPCODE %i and position %i, aborting\n",inst,i);
	i+=1;
	return -1;
	break;
      }
  }
  return 0;  
}

int my_read(int fd,char *buff,int timeout){
  // reads from serial port till it sees a newline or timeout runs out.
  // timeout measured in 0.1 s intervals
  // buff is assumed to be able to hold 80 characters.
  // returns the number of characters found.
  int i,pos=0,newbytes;
  do{
    i=0;
    do{
      newbytes = read(fd,&buff[pos],1);
      if (newbytes == -1){ //probably woken by signal
	return -1;
      }
      if (newbytes == 1){
	if (buff[pos] == '\n') {
	  buff[pos+1] = 0;
	  return pos+1;
	}
	pos += 1;
	i=0;
      }
      else
	i += 1;
    }while ((i<timeout || timeout == 0) && newbytes == 0);
    
  }while ((i < timeout || timeout == 0) && pos < BUFFLEN-1);
  buff[pos] = 0;
  return pos;
}

int my_read0(int fd,char *buff,int timeout){
  // reads from serial port till it sees a newline or timeout runs out.
  // timeout measured in 0.1 s intervals
  // buff is assumed to be able to hold 80 characters.
  // returns the number of characters found.
  int i,pos=0,newbytes;
  for (i=0;i<timeout;i++){
    newbytes=read(fd,&buff[pos],BUFFLEN-pos-1);
    pos += newbytes;
    buff[pos] = 0;
    if (timeout == 0) i = 0; // wait forever.
    if (pos > BUFFLEN-2 ) return pos;
    if (strstr(buff,"\n") != NULL)
      break;
  }
  //  printf("my_read, returning: %s, length: %i",buff,pos);
  return pos;
}

void due_close_prog(int fd){
  if (fd >=0){
    flock(fd,LOCK_UN);
    close(fd);
  }
}

int due_open_prog(char *device){
  struct termios myterm;
  int fd0,bytes_read,rval;
  char sbuff[BUFFLEN];
  
  fd0 = open(device, O_RDWR | O_NOCTTY);
  if (fd0 < 0){
    printf("duepp: can't open port to programmer %s\n",device);
    return -1;
  }
  rval = flock(fd0,LOCK_EX|LOCK_NB); // exclusive lock, don't block if we can't.
  if (rval < 0){
    printf("duepp: Couldn't obtain lock on due programmer board\n");
    close(fd0);
    return -1;
  }
  
 tcgetattr(fd0,&myterm);
 myterm.c_iflag = 0;
 myterm.c_oflag= CR0;
 myterm.c_cflag = CS8 |CLOCAL|CREAD|B38400; // speed doesn't matter for usb
 myterm.c_lflag=0;
 myterm.c_cc[VMIN]=0; // non-blocking
 myterm.c_cc[VTIME]=1; // returns after 0.1s if no characters available
 
 tcsetattr(fd0,TCSANOW, &myterm);
 tcflush(fd0,TCIFLUSH);
 
 
 printf("duepp: writing Q: ");
 write(fd0,"Q",1);
 bytes_read = my_read(fd0,sbuff,25);
 if (bytes_read > 0 ){
   printf("duepp: Got: %s",sbuff);
   if (strncmp(sbuff,"Due pulse programmer v1",23) == 0)
     return fd0;
 }
 rval=flock(fd0,LOCK_UN);
 close(fd0);
 return -1;
}

int due_download_prog(int fd,due_prog_t *program){
  char cbyte[3];
  char sbuff[BUFFLEN];
  unsigned char c1,c2;
  int c1d,c2d;
  int i,bytes_read;
  struct timeval start_time,end_time;
  struct timezone tz;
  double d_time;
  if (fd <= 0){
    printf("duepp: due_download: got invalid file descriptor\n");
    return -1;
  }
  if (program->error != 0){
    printf("duepp: pulse program has an error flag set, will not download!\n");
    return -1;
  }
  if (program->state != STATE_FINALIZED){
    printf("duepp: WARNING. Program has not been finalized. If it contains any subroutines, bad things will happen.\n");
  }
  gettimeofday(&start_time,&tz);
  printf("duepp: Sending prog size: %i ",program->dpos);
  fflush(stdout);
  //  write(fd,"D",1);
  
  // send data length, low byte, high byte
  cbyte[0] = 'D';
  cbyte[1] = program->dpos & 0xff;
  cbyte[2] = (program->dpos>>8)&0xff;
  write(fd,cbyte,3);
  
  bytes_read = my_read(fd,sbuff,5000);
  if (bytes_read > 0 ){
    printf("duepp: Got: %s",sbuff);
    if (strstr(sbuff,"size ok") == NULL){
      printf("duepp: didn't get size ok, aborting\n");
      return -1;
    }
  }
  else{
    printf("duepp: no response to program size\n");
    return -1;
    }
    //     then data.
  
  printf("duepp: Sending program: \n");
  fflush(stdout);

    // first do 512 byte blocks: = 128 4-byte words
    for ( i = 0 ; i+128 < program->dpos ; i += 128){
      // printf("512 byte block at pos: %i ",i);
      write(fd,&(program->data[i]),512);
      bytes_read = my_read(fd,sbuff,100);
      //printf("got: %s",sbuff);
    }
    //write the rest:
    //    printf("writing final %i words\n",(program->dpos-i));
    write(fd,&(program->data[i]),(program->dpos-i)*4);
    
    //write(fd,&data[i],dpos*4);
    bytes_read = my_read(fd,sbuff,10000);
    if (bytes_read > 0){
      printf("duepp: got: %s",sbuff);
      if (strstr(sbuff,"data received") != NULL){ // first two bytes are checksums
	gettimeofday(&end_time,&tz);

	d_time=(end_time.tv_sec-start_time.tv_sec)*1e6
	  +(end_time.tv_usec-start_time.tv_usec);
	//	fprintf(stderr,"downloaded %i bytes in:  %.0f us\n",program->dpos*4,d_time);
	printf("duepp: downloaded %i bytes in:  %.0f us\n",program->dpos*4,d_time);
	// check checksums
	checksum_data(&c1,&c2,program->dpos,program->data);
	sscanf(sbuff,"%i %i",&c1d,&c2d);
	if (c1 != c1d || c2 != c2d) {
	  printf("duepp: checksums don't match! I calculate: %i %i, received: %i %i\n",c1&0xff,c2&0xff,c1d,c2d);
	  return -1;
	}
	//	else printf("checksum match: %i %i, %i %i\n",c1&0xff, c2&0xff,c1d,c2d);
	return 0;
      }
    }
    printf("duepp: no response to sent program\n");
    return -1;
}

int due_run_program(int fd, char start_command){
  // start_command is e, E or R for software start, triggered start and restart (resume when final event is still running).
  int bytes_read;
  char sbuff[BUFFLEN];
  if (fd <= 0){
    printf("duepp: due_run: got invalid file descriptor\n");
    return -1;
  }
  printf("duepp: writing %c: ",start_command);
  write(fd,&start_command,1);
  bytes_read = my_read(fd,sbuff,25);
  if (bytes_read > 0 ){
   printf("duepp: Got: %s",sbuff);
   if (strncmp(sbuff,"Starting",8) == 0 || strncmp(sbuff,"Restarting",10) == 0)
     return 0;
  }
  return -1;
 
}

int due_wait_for_completion(int fd, int timeout){
  char sbuff[BUFFLEN];
  int bytes_read;
  if (fd <= 0){
    printf("duepp: due_wait_for_completion: got invalid file descriptor\n");
    return -1;
  }
  printf("duepp: waiting for completion: \n");
  fflush(stdout);
  bytes_read = my_read(fd,sbuff,timeout);
  if (bytes_read == -1) return -1; // woken by signal.
  if (bytes_read == 0) return 1; // timeout 
  printf("duepp: Got: %s\n",sbuff);
  if (strncmp(sbuff,"Final Event started",19)==0) return 0;
  if (strncmp(sbuff,"Was interrupted",15)==0) return 2;
  return -1;
}

int due_interrupt_program(int fd){

  // don't read anything. Follow this with due_wait_for_completion.
  printf("duepp: writing K: ");
  write(fd,"K",1);
  return 0;
}

int due_get_status(int fd){
  char sbuff[BUFFLEN];
  int bytes_read;
  if (fd <= 0){
    printf("duepp: due_get_status: got invalid file descriptor\n");
    return -1;
  }
  printf("duepp: writing S: \n");
  write(fd,"S",1);
  bytes_read = my_read(fd,sbuff,100);
  if (bytes_read == 0) return -1; //
  printf("duepp: Got: %s\n",sbuff);
  return 0;
}
  

int due_write_dacs_now(int fd, unsigned int dac0,unsigned int dac1){
  unsigned int dval,bytes_read,i;
  char sbuff[BUFFLEN];
  char cbyte[5],*obytes;
  if (fd <= 0){
    printf("duepp: due_write_dacs_now: got invalid file descriptor\n");
    return -1;
  }
  dval = dac0 | dac1<<16 | 1<<28;
  //  write(fd,"A",1);
  //  write(fd,&dval,4);
  cbyte[0] = 'A';
  obytes = (char *) &dval;
  for (i=0;i<4;i++)
    cbyte[i+1] = obytes[i];
  write(fd,cbyte,5);
  bytes_read = my_read(fd,sbuff,100);
  if (bytes_read >0){
    printf("duepp: Got: %s\n",sbuff);
    if (strncmp(sbuff,"OK",2) == 0) return 0;
  }
  return -1;
}

int due_write_alt_now(int fd, unsigned int output){
  unsigned int bytes_read,i;
  char sbuff[BUFFLEN];
  char cbyte[5],*obytes;
  
  if (fd <= 0){
    printf("duepp: due_write_alt_now: got invalid file descriptor\n");
    return -1;
  }
  //  write(fd,"P",1);
  //  write(fd,&output,4);
  cbyte[0] = 'P';
  obytes = (char *) &output;
  for (i=0;i<4;i++)
    cbyte[i+1] = obytes[i];
  write(fd,cbyte,5);
  bytes_read = my_read(fd,sbuff,100);
  if (bytes_read >0){
    printf("duepp: Got: %s\n",sbuff);
    if (strncmp(sbuff,"OK",2) == 0) return 0;
  }
  return -1;
  

}


int due_read_analog(int fd, unsigned char pin){
  // pin number should be one of 62, 63, 64, 65 for A8-A11 (D62-D65 or B.17-B.20)
 unsigned int bytes_read;
  char sbuff[BUFFLEN];
  char cbyte[2];
  unsigned int rval=0;
  if (fd <= 0){
    printf("duepp: due_read_analog: got invalid file descriptor\n");
    return -1;
  }
  cbyte[0] = 'a';
  cbyte[1] = pin;
  write(fd,cbyte,2);
  bytes_read = my_read(fd,sbuff,100);
  if (bytes_read >0){
    rval = atoi(sbuff);
    //    printf("duepp: Got: %s, converted to int: %i\n",sbuff,rval);
  }
  return rval;
  


}
