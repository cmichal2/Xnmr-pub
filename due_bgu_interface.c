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

#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <dirent.h>
#include <semaphore.h>
#include "param_utils.h"
#include "due-pp-lib.h"
#ifdef DUELIME
#include "shm_prog-duelime.h"
#define STOP_RADIO stop_lime
#elif defined DUESDR
#include "shm_prog-duesdr.h"
#define STOP_RADIO stop_rx
#endif
#include "duepp.h"

#define MAX_DATA (6144*3)
#define EVENTS_IN_PACKET 42
#define BUFFLEN 100
#define STACK_SIZE 50

char thread_launched = 0;
extern struct prog_shm_t*  prog_shm; 
sem_t fd_sem;
char kill_send_packet = 0;
uint32_t words_sent;

int bgu_read(int fd,char *buff,int timeout){
  // reads from serial port till it sees a newline or timeout runs out. or stop_lime is set
  // timeout measured in 0.1 s intervals
  // returns the number of characters found.
  int i,pos=0,newbytes;
  do{
    i=0;
    do{
      newbytes = read(fd,&buff[pos],1);
      if (newbytes == -1 || kill_send_packet){ 
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

int bgu_readn(int fd,unsigned char *buff,int num,int timeout){
  // reads from serial port till it sees num bytes or timeout runs out.
  // timeout measured in 0.1 s intervals
  // returns the number of characters found.
  int i,pos=0,newbytes;
  do{
    i=0;
    do{
      newbytes = read(fd,&buff[pos],1);
      if (newbytes == -1 ){ 
	return -1;
      }
      if (newbytes == 1){
	//	printf("myreadn, got byte: %i\n",buff[pos]&0xff);
	pos += 1;
	if (pos == num)
	  return pos;
      }
      else
	i += 1;
    }while ((i<timeout || timeout == 0) && newbytes == 0);
  }while ((i < timeout || timeout == 0) && pos < BUFFLEN-1);
  return pos;
}


uint32_t bgu_shift_bits(uint32_t input){
uint32_t output=0;
//this assumes port C, missing bits 0, 10, 11, 20, 27, 30, 31
// 25 useful bits.
/*    0-8   ->  1-9 
      9-16  -> 12-19
      17-22 -> 21-26
      23-24 -> 28-29

    */
//31-28 27-24 23-20 19-16 15-12 11-8 7-4 3-0                                                                                                                            
 output |= ((input & 0x01800000) << 5);
 output |= ((input & 0x007e0000) << 4);
 output |= ((input & 0x0001fe00) << 3);
 output |= ((input & 0x000001ff) << 1);
 
 return output;
}


int bgu_reset(int fd){
  char sbuff[BUFFLEN];
  int bytes_read;
  printf("writing R to reset bgu interface\n");
  if (fd > 0){
    sem_wait(&fd_sem);
    write(fd,"R",1);
    bytes_read = bgu_read(fd,sbuff,10);
    sem_post(&fd_sem);
    if (bytes_read >0){
      if (strncmp(sbuff,"OK",2) != 0){
	printf("didn't get OK from bgu reset, instead got %s\n",sbuff);
	return -1;
      }
      return 0;
    }
    printf("no response to bgu reset\n");
    return -1;
  }
  else{
    printf("bgu_reset got invalid fd\n");
    return -1;
  }
}


int do_bgu_int_open(char *device){
  struct termios myterm;
  int fd,bytes_read,rval;
  char sbuff[BUFFLEN];
  
  fd = open(device, O_RDWR | O_NOCTTY);
  if (fd < 0){
    printf("can't open port to bgu interface %s\n",device);
    return -1;
  }
  rval = flock(fd,LOCK_EX|LOCK_NB); // exclusive lock, don't block if we can't.
  if (rval < 0){
    printf("Couldn't obtain lock on due bgu interface board\n");
    close(fd);
    return -1;
  }
  
 tcgetattr(fd,&myterm);
 myterm.c_iflag = 0;
 myterm.c_oflag= CR0;
 myterm.c_cflag = CS8 |CLOCAL|CREAD|B38400; // speed doesn't matter for usb
 myterm.c_lflag=0;
 myterm.c_cc[VMIN]=0; // non-blocking
 myterm.c_cc[VTIME]=1; // returns after 0.1s if no characters available
 
 tcsetattr(fd,TCSANOW, &myterm);
 tcflush(fd,TCIFLUSH);
 
 
 printf("bgu: writing Q: ");
 write(fd,"Q",1);
 bytes_read = bgu_read(fd,sbuff,25);
 if (bytes_read > 0 ){
   printf("bgu Got: %s",sbuff);
   if (strncmp(sbuff,"Due BGU interface v1",20) != 0){
     flock(fd,LOCK_UN);
     close(fd);
     return -1;
   }
 }

 printf("bgu: writing E:\n");
 write(fd,"E",1);
 bytes_read = bgu_read(fd,sbuff,25);
 if (bytes_read > 0 ){
   printf("bgu Got: %s",sbuff);
   if (strncmp(sbuff,"OK",2) != 0){
     close(fd);
     return -1;
   }
 }

 printf("bgu: writing R:\n");
 write(fd,"R",1);
 bytes_read = bgu_read(fd,sbuff,25);
 if (bytes_read > 0 ){
   printf("bgu Got: %s",sbuff);
   if (strncmp(sbuff,"OK",2) != 0){
     close(fd);
     return -1;
   }
 }

 printf("bgu: writing Z:\n");
 write(fd,"Z",1);
 bytes_read = bgu_read(fd,sbuff,25);
 if (bytes_read > 0 ){
   printf("bgu Got: %s",sbuff);
   if (strncmp(sbuff,"OK",2) != 0){
     close(fd);
     return -1;
   }
 }
 printf("bgu open got fd %i\n",fd);
 return fd;

}

int bgu_write_zero(int fd){
  char sbuff[BUFFLEN];
  int bytes_read;
  printf("bgu write_zero: writing Z: ");
  sem_wait(&fd_sem);
  words_sent = 0;
  write(fd,"Z",1);
  bytes_read = bgu_read(fd,sbuff,25);
  sem_post(&fd_sem);
  if (bytes_read > 0 ){
    printf("bgu Got: %s",sbuff);
    if (strncmp(sbuff,"OK",2) != 0){
      printf("bgu_write_zero didn't get OK, got: %s\n",sbuff);
      return -1;
    }
    else return 0;
  }
  printf("bgu_write_zero didn't get OK, got no response\n");
  return 1;
  
}

int bgu_int_open(){

 char devname[266];
 int fd;
 
 struct dirent *de; 
 DIR *dr = opendir("/dev");

 if (dr == NULL){
   printf("couldn't open /dev\n");
   return 0;
 }
 

 while ((de = readdir(dr)) != NULL){
      printf("checking for ttyACM in: %s\n",de->d_name);
      if (strncmp("ttyACM",de->d_name,6) == 0){// its a ttyACM, see if its a bgu interface
	sprintf(devname,"/dev/%s",de->d_name);;
	printf("checking for ttyACM in: %s\n",devname);
	fd = do_bgu_int_open(devname);
	if (fd >= 0) {
	  printf("found the bgu interface\n");
	  return fd;
	}
	else printf("wasn't a bgu interface\n");
      }
    }
 return -1;
}
int bgu_get_status(uint16_t *inbuff,uint16_t *zero_count,int fd){
  // returns 0 if both inbuff and zero_count are 0.
  // returns 1 if there is a difference, and -1 if something went wrong.

  unsigned char sbuff[BUFFLEN];
  int bytes_read;
  /*
  struct timeval start_time,end_time;
  struct timezone tz;
  double d_time;
  gettimeofday(&start_time,&tz);
  */
  sem_wait(&fd_sem);
  /*
  gettimeofday(&end_time,&tz);
  d_time=(end_time.tv_sec-start_time.tv_sec)*1e6
    +(end_time.tv_usec-start_time.tv_usec);
  printf("bgu get_status:sem acquired at:  %.0f us\n",d_time);
  */
  
  // this is unnecessary if all conversation with the due-bgu interface is working properly.
  /* eek, and it takes 100 ms to time out!
  bytes_read = bgu_read(fd,(char *)sbuff,1);
  if (bytes_read > 0){
    printf("BGU, got unexpected message of length %i %s\n",bytes_read,sbuff);
  }
  */
  //  printf("bgu: writing S with fd %i\n",fd);
  write(fd,"S",1);
  /*
  gettimeofday(&end_time,&tz);
  d_time=(end_time.tv_sec-start_time.tv_sec)*1e6
    +(end_time.tv_usec-start_time.tv_usec);
  printf("bgu get_status: write complete at  %.0f us\n",d_time);
  */
  bytes_read = bgu_readn(fd,sbuff,4,2);
  sem_post(&fd_sem);
  /*
  gettimeofday(&end_time,&tz);
  d_time=(end_time.tv_sec-start_time.tv_sec)*1e6
    +(end_time.tv_usec-start_time.tv_usec);
  printf("bgu get_status: read complete %.0f us\n",d_time);
  */
  if (bytes_read == 4 ){
    *inbuff = sbuff[0]+256*sbuff[1];
    *zero_count = sbuff[2]+256*sbuff[3];
    if (*zero_count != 0)
      printf("bgu status, FOUND %i ZEROS\n",*zero_count);
    //    printf("bgu status: %i words in buff, %i zeros sent %s\n",*inbuff,*zero_count,sbuff);
    if (*inbuff == 0 && *zero_count == 0)
      return 0;
    else return 1;
  }
  if (bytes_read > 0)
    printf("bgu wrong number of bytes returned, got: %i, and they were: %s\n",bytes_read,sbuff);
  else
    printf("bgu wrong number of bytes returned, got: 0\n");
  return -1;
}

int bgu_send_packet(int num,uint16_t *words,int fd){
  // pass it the number of gradient events - max should is 42, since 12 bytes*42=504
  uint32_t packet[42*3];
  char sbuff[BUFFLEN];
  unsigned char *cpacket;
  int i,bytes_read;
  //  double d_time;
  //  struct timeval start_time,end_time;
  //  struct timezone tz;


  //  gettimeofday(&start_time,&tz);

  cpacket = (unsigned char *) packet;
  //  printf("bgu send_packet with fd %i and %i events\n",fd,num);
  // assemble the packet:
  if (num > 42 || num < 0){
    printf("bgu send_packet got num of events out of range: %i\n",num);
    return -1;
  }

  cpacket[0] = 'D';
  cpacket[1] = num;
  cpacket[2] = 0;
  cpacket[3] = 0;
  // copy the dac values into the packet, turning on address and strobe bits:
  for(i=1;i<=num*3;i+=3){
    packet[i] = words[i-1]   | (0 << 16) | (0xf <<20); //X
    packet[i+1] = words[i]   | (1 << 16) | (0xf <<20); //Y
    packet[i+2] = words[i+1] | (2 << 16) | (0xf <<20); //Z
  }
  // bit shift to avoid the dead port pins:
  for (i=0;i<num*3;i++){
    // we send the compliment of everthing so due doesn't need to invert
    // before doing CODR (clear output data register
    packet[i+1] = ~bgu_shift_bits(packet[i+1]);
  }

  /*
  gettimeofday(&end_time,&tz);
  d_time=(end_time.tv_sec-start_time.tv_sec)*1e6
    +(end_time.tv_usec-start_time.tv_usec);
  printf("bgu: packet_prepared at:  %.0f us\n",d_time);
  */
  /*
  {
    uint16_t inbuff,zeros;
    bgu_get_status(&inbuff,&zeros,fd);
  }
  */
  /*
  gettimeofday(&end_time,&tz);
  d_time=(end_time.tv_sec-start_time.tv_sec)*1e6
    +(end_time.tv_usec-start_time.tv_usec);
  printf("duebgu: status retrieved at:  %.0f us\n",d_time);
  */
  sem_wait(&fd_sem);

  /*
  gettimeofday(&end_time,&tz);
  d_time=(end_time.tv_sec-start_time.tv_sec)*1e6
    +(end_time.tv_usec-start_time.tv_usec);
  printf("duebgu: acquired sem at:  %.0f us\n",d_time);
  */
  // ok packet is ready
  //  printf("BGU Sending data packet of length: %i\n",12*num+4);
  write(fd,packet,12*num+4);
  /*
  gettimeofday(&end_time,&tz);
  d_time=(end_time.tv_sec-start_time.tv_sec)*1e6
    +(end_time.tv_usec-start_time.tv_usec);
  printf("duebgu: packet sent at  %.0f us\n",d_time);
  */
  bytes_read = bgu_read(fd,sbuff,0); // waits forever till message returned, or a signal (eg KILL from UI)
  sem_post(&fd_sem);
  if (bytes_read > 0 ){
    if (strncmp(sbuff,"GO",2) == 0){
      /*
      gettimeofday(&end_time,&tz);
      d_time=(end_time.tv_sec-start_time.tv_sec)*1e6
	+(end_time.tv_usec-start_time.tv_usec);
	printf("duepp: downloaded packet and got response in:  %.0f us\n",d_time);
      */
      words_sent += num*3;
      //      printf("BGU returning from send_packet got %s, total words sent: %i\n",sbuff,words_sent);
      return 0;
    }
  }
  printf("bgu write packet didn't get GO, got %s\n",sbuff);
  return -1;
  
}

sem_t bgu_sem;
int exit_thread=0;
gradprog_t *gradprog;
pthread_t bgu_thread;
int fd0;

void *bgu_thread_func(void *threadno){
  unsigned int jsr_stack[STACK_SIZE],loop_stack[STACK_SIZE],loop_counter[STACK_SIZE];
  int jsr_num,loop_num;
  int next_event,cur_event,last_event;
  int events_in_packet;
  uint16_t words[EVENTS_IN_PACKET*3];

  sigset_t sigset;
  sigemptyset(&sigset);
  sigaddset(&sigset,SIGUSR1);
  sigaddset(&sigset,SIGALRM);
  
  sigprocmask(SIG_BLOCK,&sigset,NULL);

  {
    struct sched_param sp;
    int result,priority;
    pthread_t this_thread;
    
    // set our priority to 1 lower than acq - same as tx
    this_thread = pthread_self();
    priority = sched_get_priority_max(SCHED_FIFO);
    sp.sched_priority = priority/2-1; // 1 less than acq;
    result = pthread_setschedparam(this_thread, SCHED_FIFO,&sp);
    if( result!= 0) 
      perror("rx thread: init_sched");
  }

  
  
  while(1){
    printf("bgu: going to sleep\n");
    sem_wait(&bgu_sem);
    printf("bgu: awake\n");
    
    if (exit_thread){
      printf("bgu thread found exit_thread, exiting\n");
      return NULL;
    }
    
    cur_event = 0;
    loop_num = 0;
    jsr_num = 0;
    events_in_packet = 0;
    // parse the program.
    
    do{
      switch (gradprog->gradopcode[cur_event] ){
      case CONTINUE:
      case SUBSTART:
      case WAIT:
      case WAIT_MAX:
	next_event = cur_event + 1;
	break;
      case LOOP: 
	next_event = cur_event + 1;
	// if we've already started this loop, don't start it again!
	// we've started this loop if: loop_num > 0 and  loop_stack[loop_num-1] = cur_event and loop_counter[loop_num-1] > 0
	if (loop_num > 0){
	  if (loop_stack[loop_num-1] == cur_event && loop_counter[loop_num-1] > 0){
	    break;
	  }
	}
	loop_counter[loop_num] = gradprog->gradopinst[cur_event];
	loop_stack[loop_num] = cur_event;
	loop_num += 1;
	if (loop_num > STACK_SIZE -1){
	  printf("in bgu thread. Too many loops nested. BAD THINGS WILL HAPPEN\n");
	}
	break;
      case END_LOOP:
	if (loop_num < 1) printf("grad thread got an end_loop with no loops on stack BAD THINGS WILL HAPPEN\n");
	next_event = loop_stack[loop_num-1];
	if (loop_counter[loop_num-1] < 0){
	  printf("got end loop, but uninitialized number of loops. BAD THINGS WILL HAPPEN\n");
	}
	loop_counter[loop_num-1] -= 1;
	if (loop_counter[loop_num-1] <= 0){
	  next_event = cur_event + 1;
	  loop_num -= 1;
	}
	break;
      case JSR:
	jsr_stack[jsr_num] = cur_event+1;
	jsr_num += 1;
	if (jsr_num > STACK_SIZE -1){
	  printf("in grad thread, Too many JSR's nested. BAD THINGS WILL HAPPEN\n");
	}
	next_event = gradprog->gradopinst[cur_event];
	//	printf("grad: jsr at event: %i to event %i, set jsr_num to %i\n",cur_event,next_event,jsr_num);
	if (next_event > gradprog->gradevents || next_event < 0){
	  printf("BGU: got a JSR to a nonsense event (%i) at event %i, but max event is %i\n",next_event,cur_event,gradprog->gradevents);
	}
	break;
      case RTS:
	//	printf("tx: rts at event %i, jsr_num is %i, return to %i\n",cur_event,jsr_num,jsr_stack[jsr_num-1]);
	if (jsr_num < 1){
	  printf("bgu thread: got RTS, but nowhere to return to!\n");
	  next_event = cur_event + 1 ;
	}
	else{
	  next_event = jsr_stack[jsr_num-1];
	  jsr_num -=1;
	}
	if (next_event > gradprog->gradevents || next_event < 0){
	  printf("got a RTS to a nonsense event\n");
	}
	//	printf("leaving jsr_num at: %i\n",jsr_num);
	break;
      case EXIT: // I guess we're done!
	next_event = gradprog->gradevents;
	//	printf("TX got EXIT event at %i\n",cur_event);
	break;
      default:
	printf("grad thread, got an unknown opcode of %i\n",(gradprog->gradopcode[cur_event] & 15));
	// something is seriously wrong, and we should tell acq somehow.?? TODO
	next_event = cur_event + 1;
      } // end of switch on pb opcodes
      
      // ok, so event is parsed, next event is known. Is there a gradient to deliver in our event?
      if (gradprog->is_grad_event[cur_event]){// there is an event!
	words[3*events_in_packet] = gradprog->x[cur_event];
	words[3*events_in_packet+1] = gradprog->y[cur_event];
	words[3*events_in_packet+2] = gradprog->z[cur_event];
	events_in_packet += 1;
	if (events_in_packet == EVENTS_IN_PACKET){
	  bgu_send_packet(events_in_packet,words,fd0); // this may block if there's no space available.
	  events_in_packet = 0;
	}
      }// done processing the gradient event
      
      last_event = cur_event;
      cur_event = next_event;
    } while ((gradprog->gradopcode[last_event] & 15) != EXIT && prog_shm->STOP_RADIO >= 0 && exit_thread == 0);
    
    // done parsing program, are there events left to send?
    if (events_in_packet > 0  && prog_shm->STOP_RADIO >= 0){
      bgu_send_packet(events_in_packet,words,fd0); // this may block if there's no space available.
      events_in_packet = 0;
    }
    //    if (prog_shm->stop_lime)
    //      bgu_write_zero(fd0);
  }
  // never get here:
  return NULL;
}

int start_bgu_thread(){
  int threadno;
  if (thread_launched == 0){
    pthread_create(&bgu_thread,NULL,&bgu_thread_func,(void *) &threadno);
    sem_init(&bgu_sem,0,0);// tells bgu thread there's a pulse program to execute
    sem_init(&fd_sem,0,1);
    thread_launched = 1;
    exit_thread = 0;
    return 0;
  }
  return -1;
}

int end_bgu_thread(){
  int *retval;
  if (thread_launched == 1){
    printf("end_bgu_thread waiting for thread to exit\n");
    exit_thread = 1;
    sem_post(&bgu_sem);
    thread_launched = 0;
    pthread_join(bgu_thread,(void **)&retval);
    printf("end_bgu_thread done\n");
    return 0;
  }
  return -1;
}

int start_bgu(gradprog_t *ingradprog,int fd){
  if (thread_launched == 0)
    if (start_bgu_thread() != 0) 
      return -1;
  gradprog = ingradprog;
  fd0 = fd;
  sem_post(&bgu_sem);
  return 0;
}

void bgu_int_close(int fd){
  char sbuff[BUFFLEN];
  int bytes_read;
  if (fd >= 0){
    if (thread_launched){
      kill_send_packet = 1; // if send_packet is blocked waiting for GO, we want to unblock it.
      printf("bgu_int_close, shutting down thread:\n");
      end_bgu_thread();
      kill_send_packet = 0;
    }

    {
      uint16_t inbuff,zeros;
      bgu_get_status(&inbuff,&zeros,fd);
    }
    fflush(stdout);
    printf("in bgu_close with fd: %i, doing write_zeros\n",fd);
    bgu_write_zero(fd);
    fflush(stdout);

    printf("bgu: writing K:\n");
    write(fd,"K",1);
    bytes_read = bgu_read(fd,sbuff,25);
    if (bytes_read > 0)
      printf("closing bgu interface, got: %s\n",sbuff);
    else
      printf("bgu interface: no response to K\n");
    fflush(stdout);


    flock(fd,LOCK_UN);
    close(fd);
  }
}
