// File ADepp.c
// Created March 1, 2001 by Kesten Broughton
// Initialization code borrowed from Carl Michal and Craig Peacock 
// modified for inclusion into Xnmr by  Carl Michal

// outb should maybe be outb_p

// August 5, 2015:
// TODO:  
//        - speed up?
//        - set up a timeout on read_fifo_epp so we don't block forever
//        but as long as the pulse program is.
// Otherwise, this is working with due_pp_interface10, and new ad6620 evb arrangement.
// ie with hacked FIFO R and OEA signals to run directly from the arduino.

#include <sched.h>
#include <stdio.h>
#include "adepp-pb.h"
#include <stdlib.h>
// Note: The \RD line on the DSP command latch is only used for writing in mode 1.  
// We will be using DSP mode 0 always so the \RD line should always be high.
// Therefore DSP_NRD should be set in every write to the DSP command latch


#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include "acq-pb.h"  // needs to get "done" from acq.
int fd0 = -1; 
struct termios myterm;
#define ID_MESSAGE_LENGTH 32
#define ID_MESSAGE "USB to 6620EVB Due adapter v10\r\n"
#define ARDUINO_PORT "/dev/ttyACM0"


#include <sys/io.h>
static int port=0;
unsigned char mask;

int query_ready_epp(){
  char sbuff[ID_MESSAGE_LENGTH];
  int j,bytes_received = 0;
  // configure the port:
  if (fd0 < 0){
    fprintf(stderr,"query_ready_epp: port not open\n");
    return -1;
  }

  tcgetattr( fd0, &myterm);

  myterm.c_iflag = 0;
  myterm.c_oflag = CR0;
  myterm.c_cflag = CS8 |CLOCAL | CREAD |B2000000; // speed doesn't matter, its a USB device - no UART anywhere
  myterm.c_lflag = 0;
  myterm.c_cc[VMIN] = 0;
  myterm.c_cc[VTIME] = 2; // wait up to 0.2s for an answer

  j = tcsetattr(fd0,TCSANOW, &myterm);
  if (j<0) perror("error setting serial port attributes");
  printf("DUE: port opened and configured, verifying arduino 'Q'\n");
  write(fd0,"Q",1); // query the port, make sure it is what we think:
  tcdrain(fd0);

  do{
    sched_yield();
    j = read(fd0,sbuff+bytes_received,ID_MESSAGE_LENGTH-bytes_received);
    if (j>0) bytes_received += j;
    printf("query_ready_epp: read %i bytes\n",j);
    sched_yield();
    //    usleep(500);
    //    usleep(500);
  }while (j > 0 && bytes_received < ID_MESSAGE_LENGTH);

  if (strncmp(sbuff,ID_MESSAGE,ID_MESSAGE_LENGTH) != 0){
    fprintf(stderr,"\nXXX Didn't receive expected query reponse string from arduino, got: %s\n\n",sbuff);
    close(fd0);
    return -1;
  }


  myterm.c_cc[VMIN] = 0;
  myterm.c_cc[VTIME] = 1; // we wait forever for a first byte, then up to .1 s between bytes.
  j = tcsetattr(fd0,TCSANOW, &myterm);
  return 0;

}

int initialize_dsp_epp(int p){
  int j;
  char buff;
  port = p;
  // do the parallel port stuff - need to hit reset on the synths,
  // and set up to allow interrupts to be reported:
  // initialize EPP by writing to control port - there's something missing here!!
  //  outb(0x00, SPP_CTRL+port); 

  // the 1 in the 14 allows interrupts to be reported.
  // the lack of the 4 above hits the reset button - only affects the synths.

  //  outb(0x14, SPP_CTRL+port);
  

  if (fd0 > 0){
    fprintf(stderr,"open initialize_dsp_epp, but fd0 is already open?\n");
  }
  else{
    printf("attempting to open arduino port %s\n",ARDUINO_PORT);
    // try to open serial port:
    fd0 = open(ARDUINO_PORT, O_RDWR|O_NOCTTY);
    if (fd0 < 0){
      fprintf(stderr,"in initialize_dsp_epp, couldn't open port to arduino\n");
      return -1;
    }
  }
  tcflush(fd0,TCIOFLUSH);
  if (query_ready_epp() == -1) return -1;
  printf("DUE, opened, flush, sending T ");
  write(fd0,"T",1); // hit the reset button on the synths
  tcdrain(fd0);
  
  j = read(fd0,&buff,1);
  if (j !=1 || buff != 'T')
    printf("\n\n XXX initialize_dsp_epp got: %c (%i), expected 'T', got %i bytes\n\n",buff,buff,j);
  else
    printf("got 'T'\n");
  fflush(stdout);
 
  return 0;
}
void select_device(unsigned char address){
  char sbuff[2];
  int j;
  if (fd0 < 0){
    fprintf(stderr,"select_device: port not open\n");
    return;
  }

  sbuff[0] = 'D';
  if (address == DSP) sbuff [1] = '1';
  else sbuff[1] = '0';
  printf("DUE: select device: %i ",sbuff[1]);
  write(fd0,sbuff,2);
  tcdrain(fd0);

  fflush(stdout);
  sched_yield();
  
  j = read(fd0,&sbuff,1);
  if (j !=1 || sbuff[0] != 'D')
    printf("\n\n XXX Select device, got: %c (%i), expected 'D', got %i bytes\n\n",sbuff[0],sbuff[0],j);
  else
    printf("got 'D'\n");
  fflush(stdout);
  

  
}

void reset_fifo(){
  // not needed, done in the arduino code in start_acquire_pulse
}

void start_acquire_pulse(){
  char buff,j;
  if (fd0 < 0){
    fprintf(stderr,"start_acquire_pulse: port not open\n");
    return;
  }
  //  tcflush(fd0,TCIOFLUSH); //throw away anything that was sitting in receive buffer?
  printf("DUE: start acquire 'S' ");
  write(fd0,"S",1);
  tcdrain(fd0);
  fflush(stdout);

  sched_yield();
  j = read(fd0,&buff,1);
  if (j !=1 || buff != 'S')
    printf("\n\n XXX Start acquire pulse, got: %c (%i), expected 'S', got %i bytes\n\n",buff,buff,j);
  else
    printf("got 'S'\n");
  fflush(stdout);

}


int myread(int npts, int *data){
  unsigned char mydata[512];
  int j,i,bytes_done=0;
  if (npts*4 > 512){
    printf("EEK: adepp.c, myread was asked for %i bytes. Max is 512\n",npts*4);
    return -1;
  }
  while(bytes_done < npts*4 ){
    j = read(fd0,mydata+bytes_done,npts*4-bytes_done);
    /* cases 1) interrupted with a signal NICE_DONE - just restart.
             2) interrupted with KILL_DONE
	     3) normal read
	     4) other random error?
	     5) j=0 timeout (??)
    */
    if (j < 0 ){
      printf("adepp: myread: return from read with j = %i, bytes_done is %i\n",j,bytes_done);
      if (errno != EAGAIN && errno != EINTR){ // EAGAIN shouldn't happen, but means no data available. EINTR is a signal
	perror("adepp read error");
	printf("adepp: myread1: read %i bytes\n",bytes_done);
	fflush(stdout);
	return bytes_done/2;
      } // random errors
    } // j<0
    else
      bytes_done += j;
    // if we had a short read, sleep a moment then try again.
    if (bytes_done < npts*4){
      sched_yield();
      usleep(500);
      //      printf("sleeping in myread\n");
    }

    if (done == KILL_DONE){
      printf("DUE: read_fifo_epp: KILL_DONE? writing 'A'\n");
      write(fd0,"A",1); // tell arduino to abort, then read till there's nothing left.
      tcdrain(fd0);
      // we throw away all remaining data.
      myterm.c_cc[VMIN] = 0;
      myterm.c_cc[VTIME] = 1; // wait up to 0.1s for an answer
      j = tcsetattr(fd0,TCSANOW, &myterm);
      do{
	sched_yield();
	usleep(500);
	j = read(fd0,mydata,512);
	printf("adepp: myread: KILL_DONE, read %i bytes draining\n",j);
      }while (j > 0);
      
      myterm.c_cc[VMIN] = 0;
      myterm.c_cc[VTIME] = 1; // we wait forever for a first byte, then up to .2 s between bytes - er, no - wait for max of 0.1s.
      j = tcsetattr(fd0,TCSANOW, &myterm);
      printf("adepp: myread2: read %i bytes\n",bytes_done);
      fflush(stdout);
      return 0;
    } // end of KILL_DONE


  } // end of main while loop.
  for (i=0;i<bytes_done/2;i++){
    data[i] = mydata[i*2]+256*mydata[i*2+1];
    if (data[i] > 32767) data[i] -= 65536;
  }
  return bytes_done;

}


int request_data_epp(int npts,int receiver_model){
  if (fd0 < 0){
    fprintf(stderr,"read_fifo_epp: port not open\n");
    return -1;
  }
  char sbuff[5],buff,j;
  printf("request_data_epp: receiver model is: %i\n",receiver_model);
  if (receiver_model != 2) // 2 is REC_ACCUM
    sbuff[0] = 'C'; // does look at empty flag
  else{
    sbuff[0] = 'c'; // does not look at empty flag 0 and gives two zero words at beginning.
    npts += 1; // makes room for two zero words. data should have enough room, since MAX is huge now.
  }

  sbuff[1] = (npts & 255);
  sbuff[2] = ((npts >> 8) & 255);
  sbuff[3] = (npts >> 16) & 255;
  sbuff[4] = (npts >> 24) & 255;
  printf("DUE: %c ",sbuff[0]);
  write(fd0,sbuff,5);
  tcdrain(fd0);
  fflush(stdout);
  
  sched_yield();
  j = read(fd0,&buff,1);
  if (j !=1 || buff != sbuff[0])
    printf("\n\nXXX Request data epp, got: %c, expected %c, got %i bytes\n\n",buff,sbuff[0],j);
  else
    printf("got %c\n",sbuff[0]);
  printf("DUE: adepp: asking arduino for %i points\n",npts);
  fflush(stdout);
  return 0;
}

int read_fifo_epp(int npts,int *data,int receiver_model){
  int j,bytes_complete=0,points_wanted;
  int status = 0;

  /* if receiver model is 1 we use 'c' rather than 'C'
     we collect two extra words at the front and throw them away. */

  if (fd0 < 0){
    fprintf(stderr,"read_fifo_epp: port not open\n");
    return -1;
  }
  printf("in read_fifo_epp, receiver_model is: %i, asking for %i points\n",receiver_model,npts);
  if (receiver_model == 2) npts += 1;
  /* this is tricky. need to worry about telling arduino to abort and draining what its written so far...
  
     Some caveats: for 'c', 'A' doesn't do anything, but it shouldn't matter.

     There is a race condition in here. If a KILL_DONE signal arrives
     and kills the pulse programmer when we're outside of a read call,
     we could get stuck in here. This should be clearable by hitting
     kill a second time I think.

     But you know, maybe this isn't a race after all. acq is a real
     time process and Xnmr isn't. Xnmr will only execute when acq is
     blocked somewhere, and the only place for it to block in here is
     during a read call. This should be ok. So signals will only be
     generated (and delivered) when acq is blocked.

     Ah, but the signal could arrive when we go to sleep - should always check for KILL_DONE after a sleep.


  */

  while ((bytes_complete < npts*4) && (done != ERROR_DONE)){
    if (npts*4 - bytes_complete > 512)
      points_wanted = 128;
    else
      points_wanted = npts-bytes_complete/sizeof(int);

  // four bytes per point. 
    j = myread(points_wanted,data+bytes_complete/2);
    if (j != points_wanted*4) {
      printf("in read_fifo_epp, error, asked for %i points, got: %i?\n",points_wanted*4,j);
      break;
    }
    bytes_complete += j;
    printf("\radepp: read %i of %i ",bytes_complete,npts*4);
    fflush(stdout);
    if (bytes_complete == npts*4)
      printf("\n");
  }


  if (receiver_model == 2){ // 2 is REC_ACCUM
    int i;
    printf("receiver_model is 2\n");
    // this first bit is temporary for testing:
    FILE *lfile;
    if (data[npts*2-1] == data[npts*2-2]){
      status += 1;
    if (data[npts*2-3] == data[npts*2-2])
      status += 2;
    }
    if (data[0] != 0 || data[1] !=0 )
      status +=4;

    if (status > 0){
      lfile = fopen("/tmp/xnmr.log","a");
      if (lfile != NULL){
	fclose(lfile);
	system("date >> /tmp/xnmr.log");
	lfile = fopen("/tmp/xnmr.log","a");
	fprintf(lfile,"status = %i\n",status);
	if (status & 1)
	  fprintf(lfile,"adepp: last two words are identical: %i, %i\n",data[npts*2-1],data[npts*2-2]);
	if (status & 2)
	  fprintf(lfile,"adepp: last three words are identical: %i %i, %i\n",data[npts*2-3],data[npts*2-1],data[npts*2-2]);
	if (status & 4)
	  fprintf(lfile,"first points not both zero! %i and %i\n",data[0],data[1]);
	//      for (i=0;i<npts;i++)
	//fprintf(lfile,"%i %i\n",data[2*i],data[2*i+1]);
      }
      fclose(lfile);
    }

    // get rid of the first two zeros:
    //    if (status == 1){
    //     for (i=0;i<2*npts-2;i++)
    //	data[i] = data[i+1];
    // }
    //else{
      for (i=0;i<2*npts-2;i++)
	data[i] = data[i+2];
      //}
    bytes_complete -=4;
  }
  printf("adepp: end read_fifo_epp: read %i bytes\n",bytes_complete);
  fflush(stdout);
  return bytes_complete/2;
}

// *********************** DSP MICROPORT COMMANDS
void reset_dsp(){
  char buff;
  int j;
  if (fd0 < 0){
    fprintf(stderr,"reset_dsp: port not open\n");
    return;
  }
  printf("DUE: reset, 'R' ");

  write(fd0,"R",1);
  tcdrain(fd0);
  j = read(fd0,&buff,1);
  if (j !=1 || buff != 'R')
    printf("\n\n XXX reset_dsp, got: %c (%i), expected 'R', got %i bytes\n\n",buff,buff,j);
  else
    printf("got 'R'\n");
  fflush(stdout);

}

void write_micro(unsigned char addr, unsigned char data){
  char sbuff[3];
  int j;
  if (fd0 < 0){
    fprintf(stderr,"write_micro: port not open\n");
    return;
  }
  sbuff[0] = 'W';
  sbuff[1] = addr;
  sbuff[2] = data;
  printf("DUE: write_micro ");
  write(fd0,sbuff,3);
  tcdrain(fd0);
  
  j = read(fd0,sbuff,1);
  if (j !=1 || sbuff[0] != 'W')
    printf("\n\n XXX write_micro, got: %c (%i), expected 'W', got %i bytes\n\n",sbuff[0],sbuff[0],j);
  else
    printf("got 'W'\n");
  fflush(stdout);

}

void dsp_close_port_epp(){
  if (port != 0){
    outb(0x00, SPP_CTRL+port);  // shuts down synths.
    outb(0x04, SPP_CTRL+port);  // turn off interrupts.
  }
  //result = ioperm(port,8,0);
  port=0;
  if (fd0 > 0){
    close(fd0);
    fd0 = -1;
  }
}
 
void clear_interrupt(){
  char buff,j;
  
  if (fd0 < 0){
    fprintf(stderr,"reset_dsp: port not open\n");
    return;
}
  printf("DUE: clear interrupt 'i' ");

  write(fd0,"i",1);
  tcdrain(fd0);
  fflush(stdout);

  sched_yield();
  j = read(fd0,&buff,1);
  if (j !=1 || buff != 'i')
    printf("\n\nXXX Clear interrupt, got: %c, expected 'i', got %i bytes\n\n",buff,j);
  else
    printf("got %c\n",buff);
  fflush(stdout);
}
 
void wait_interrupt(){
  int j=0,count=0;
  unsigned char buff;
  if (fd0 < 0){
    fprintf(stderr,"reset_dsp: port not open\n");
    return;
  }
  if (done < 0 ) return;

  printf("DUE: wait interrupt 'I' ");
  fflush(stdout);
  write(fd0,"I",1);
  tcdrain(fd0);
  sched_yield();
  j = read(fd0,&buff,1);
  if (j !=1 || buff != 'I')
    printf("\n\n XXX wait_interrupt, got: %c, expected 'I' got %i bytes\n\n",buff,j);
  else
    printf(" got 'I'\n");
  fflush(stdout);
  j=0;
  // now wait till we get something back
  while (j <= 0 && done >= 0){
    printf("\rwaiting for arduino interrupt, %i ",count++);
    fflush(stdout);
    j = read(fd0,&buff,1);
    //    printf("returned from arduino interrupt with rval: %i, done: %i\n",j,done);
    /* cases 1) interrupted with a signal NICE_DONE - just restart.
       2) interrupted with KILL_DONE
       3) normal read
       4) other random error?
       5) j=0 timeout (??)
    */
  }
  printf("\n");
  // j = -1 if no data, but interrupted by a signal.
  // should really do a better job of finding out what the error might be if j = -1. Here we assume its a signal
  if (j == 0){ // no data
    printf("DUE: in wait interrupt 'A'\n");
    write(fd0,"A",1);
    tcdrain(fd0);
  }
}
  

  
