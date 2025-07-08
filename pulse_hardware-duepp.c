/* pulse_hardware.c
 *
 * This file specifies function prototype to communicate with the pulse programmer hardware
 * Xnmr software
 *
 * UBC Physics
 * April, 2000
 * 
 * written by: Scott Nelson, Carl Michal
 */
//#define TIMING
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>
#include "param_utils.h"
#include "p_signals.h"
#include "due-pp-lib.h"
//#include "shm_prog-duesdr.h" // XX HELP
//#include "pulse-duesdr.h" // XX HELP

//#include "/usr/share/Xnmr/config/h_config-duelime.h"
#include "/usr/share/Xnmr/config/pulse_hardware-duepp.h"
//#include "spinapi.h"

#include <sys/io.h>   //for ioperm routine
//#include <asm/io.h>   //for inb, outb, etc.
#include <stdio.h>    //for printf
#include <sys/time.h>
#include <unistd.h>


#define ATTN_PORT "/dev/usb/lp0"


/*
 *  Global Variables
 */

// file descriptors for due pulse program boards
int *fds = NULL,num_fds;



int pulse_hardware_stop(){
  int i;
  for (i=0;i<num_fds;i++){ 
    due_interrupt_program(fds[i]);
  }
  
  for (i=0;i<num_fds;i++){
    due_wait_for_completion(fds[i],100);
  }
  
  return 0;
}



int init_pulse_hardware( int num )
{
  int count=0;
  char devname[266];

  struct dirent *de; 
  DIR *dr = opendir("/dev");

  if (dr == NULL){
    printf("couldn't open /dev\n");
    return 0;
  }
  
  fds = malloc(num*sizeof(int));

  while ((de = readdir(dr)) != NULL && count < num){
    //      printf("checking for ttyACM in: %s\n",de->d_name);
      if (strncmp("ttyACM",de->d_name,6) == 0){// its a ttyACM, see if its a pulse programmer
	sprintf(devname,"/dev/%s",de->d_name);;
	//	printf("checking for ttyACM in: %s\n",devname);
	fds[count] = due_open_prog(devname);
	if (fds[count] >=0) {
	  count+=1;
	  printf("found a pulse programmer!\n");
	}
	else printf("wasn't a pulse programmer\n");
      }
    }
  // so now we've either found the requested number of boards, or they aren't there.
  num_fds = count;
  if (count < num) {
    free_pulse_hardware();
    count = 0;
    return 0;
  }
  
  // TODO - should have some way of ordering them in here - using the alt port inputs
  // to label which is which? 
  

  
  return count;
    
}

//int pulse_hardware_send( struct prog_shm_t* program )
int pulse_hardware_send( due_prog_t * due_prog )
{


  int i,rval=0;
#ifdef TIMING
  int byte_count=0;
  int overhead=0;
  struct timeval start_time,end_time;
  struct timezone tz;
  float d_time;
  gettimeofday(&start_time,&tz);
  rval = 0;
  
#endif
  for (i=0;i<num_fds;i++){
    //    rval += due_download_prog(fds[i],&program->due_prog[i]);
    rval += due_download_prog(fds[i],&due_prog[i]);
  }
  if (rval != 0){
    printf("Error downloading program!\n");
    return -1;
  }



#ifdef TIMING
    gettimeofday(&end_time,&tz);

  //overhead+=4;
    d_time=(end_time.tv_sec-start_time.tv_sec)*1e6+(end_time.tv_usec-start_time.tv_usec);
   fprintf(stderr,"download time: %.0f us\n",d_time);
   fprintf(stderr,"downloaded: %i+%i bytes to programmer in %.0f us, rate: %.3f MB/s\n",
	 byte_count,
       	 overhead, d_time,
	 (byte_count+overhead)/d_time);  
#endif

  return 0;
}


int pulse_hardware_start(char command)
{
  int i;
  int rval=0;
  // XX TODO - need to set up for multiple boards - so boards beyond the first are syncd to it.
  // assumes that first board is the master - but started without
  // sync here, for now
  for (i=num_fds-1;i>=0;i--){
    //    printf("starting fd index %i, %i\n",i,fds[i]);
    rval += due_run_program(fds[i],command);
  }
  if (rval != 0){
    pulse_hardware_stop();
    return -1;
  }
  return 0;
}


int free_pulse_hardware()
{
  int i;
  printf("free_pulse_hardware with %i boards\n",num_fds);

  for(i=0;i<num_fds;i++)
    due_close_prog(fds[i]);
  if (fds != NULL){
    free(fds);
    fds = NULL;
  }
  num_fds = 0;
  return 0;
}

int pulse_hardware_wait(){
  int i,rval;
  for (i=0;i<num_fds;i++){
    rval = due_wait_for_completion(fds[i],0);
    if (rval == -1) 
      return -1;
  }
  return 0;
}


int pulse_hardware_read_adc(int i){ // read a due adc value.
  // pins here are numbered from 0-3 for each board.
  // so in total, 0 up to 4*number of due boards.
  unsigned char board,pin;
  board = i/4;
  pin = i-board*4 + 62; // since pins are 62-65
  if (board >= num_fds){
    printf("asked for pin %i from board %i, can't do it!\n",pin,board);
    return -1;
  }
  return due_read_analog(fds[board],pin);

}
