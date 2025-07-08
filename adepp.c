// File ADepp.c
// Created March 1, 2001 by Kesten Broughton
// Initialization code borrowed from Carl Michal and Craig Peacock 
// modified for inclusion into Xnmr by  Carl Michal

// outb should maybe be outb_p


#include <sys/perm.h>
#include <sys/io.h>
#include <stdio.h>
#include "adepp.h"

// Note: The \RD line on the DSP command latch is only used for writing in mode 1.  
// We will be using DSP mode 0 always so the \RD line should always be high.
// Therefore DSP_NRD should be set in every write to the DSP command latch


static int port=0;
static int last_word = 0;  // last words read from dsp

unsigned char mask;


void reset_timeout(){
  unsigned   char b;
  b = inb(SPP_STAT+port);
  outb(b | 0x01,SPP_STAT+port); // from parport_pc - try to clear timeout bit.
  outb(b & 0xfe,SPP_STAT+port);
  b = inb(SPP_STAT+port);
  if (b & 1){
    printf("failed to reset timeout bit\n");
  }
  else printf("succeeded at resetting timeout bit\n");
  
}
void check_timeout(char *s){
  unsigned char b;
      b = inb(SPP_STAT+port);
    if (b & 1){
      printf("\n%s, timeout bit is set\n",s);
      reset_timeout();
    }
}

int initialize_dsp_epp(int p)
// Must be called before any epp related functions
{ 
  static char got_perm=0;
  //  int i;
  unsigned char b;

  // get permission to use the EPP port
  if (got_perm != 1 ){
    port=p;
    fprintf(stderr,"adepp, setting port to: 0x%x\n",port);
    /*    i = ioperm(port, 8, 1);
    if(i != 0)
      { 
	fprintf(stderr,"Can't get permission to access the port 0x%x\n", port);
	return -1;
	}*/
  }
  got_perm = 1;
  if (port != p ){
    fprintf(stderr,"in initialize_dsp_epp, got a different port address?\n");
    return -1;
  }

  // initialize EPP by writing to control port - there's something missing here!!
  outb(0x00, SPP_CTRL+port); 
  outb(0x04, SPP_CTRL+port);
  

  // read status register; 
  inb( SPP_STAT+port );   
  
  b = inb(SPP_STAT+port);
  outb(b | 0x01,SPP_STAT+port); // from parport_pc - try to clear timeout bit.
  outb(b & 0xfe,SPP_STAT+port);

  b = inb(SPP_STAT+port);

  if (b & 0x01) {
    fprintf(stderr,"doing a manual address write to try to clear port\n");
    mask = DSP_NCS | DSP_NWR | DSP_NRD | DSP_NRESET ;
    outb(mask, port);
    
    outb(0x04+0x08,port+SPP_CTRL);
    outb(0x04,port+SPP_CTRL);
  }


  b = inb(SPP_STAT+port);
  
  //fprintf(stderr,"SPP status register: 0x%x\n", b);
  //  outb(b & 0xfe, SPP_STAT+port);
  if(b & 0x01)
    {
      fprintf(stderr,"Couldn't reset EPP timeout bit on port 0x%x\n", port);
      return -1;
    }
  return 0;
}

void select_device(unsigned char address)
//This function
// is needed on the AD board because the first time we switch from one command latch to
// the other the lines active on both latches.  Bit of a strange hardware choice.

     // looks like this routine will reset the fifo.
{
  mask = DSP_NCS | DSP_NWR | DSP_NRD | DSP_NRESET | address;
  outb(mask, EPP_ADDR+port);

}

// On the AD board, fifo control is achieved by an address write
// with line A7 = 0.  The address lines correspond to the following:
// A0       A1         A2       A3        A4      A5        A6        A7
// FIFOON   OEA        STAQ     MR        R       LOADMODE  not used  0

void reset_fifo()
// Pre: None
// Post: Fifo is reset and STAQ is turned off
{
  mask = FIFO | FIFO_OFF | DSP_NRESET;
  outb(mask, EPP_ADDR+port);
  last_word = 0;

}

void start_acquire_bypass()
// Pre: the fifo should probably be reset before starting each aquisition
// LOADMODE must be off to bypass the DSP
// MR_OFF and R_OFF and FIFO should be set; other values don't matter
// Post: fifo begins to acquire data directly from digitizer
{
  reset_fifo();
  mask = FIFO  | MR_OFF | LOADMODE_OFF | STAQ_ON | DSP_NRESET;
  outb(mask, EPP_ADDR+port);
}

void start_acquire_dsp()
// Pre: the fifo should probably be reset before starting each aquisition
// LOADMODE must be on to allow dsp to control aquisition
// MR_OFF and R_OFF and FIFO should be set; other values don't matter
// Post: fifo begins to acquire data
{
  reset_fifo();
  mask = FIFO | FIFO_OFF | MR_OFF | STAQ_ON | DSP_NRESET;
  outb(mask, EPP_ADDR+port);



}
void start_acquire_pulse()
// Pre: the fifo should probably be reset before starting each aquisition
// LOADMODE must be on to allow dsp to control aquisition
// MR_OFF and R_OFF and FIFO should be set; other values don't matter
// Post: fifo begins to acquire data
{

  reset_fifo();
  //  fprintf(stderr,"start_acquire_pulse, just reset fifo\n");
  mask = FIFO | FIFO_OFF | MR_OFF  | DSP_NRESET;
  outb(mask, EPP_ADDR+port);



}

int read_fifo_epp(int npts,int *data)
// Pre: Fifo control lines are active, R_OFF is set
// Post: reads 1 byte from data lines coming from fifo
{
  // should return the number of points read - need to add checking on 
  //  the fifo empty/full flags.

  unsigned short high;
  unsigned short low;
  unsigned char mask3,mask4; //,mask1,mask2;
  int i,istart=0;

  //  fprintf(stderr,"in read_fifo_epp, port is: %i\n",port);

  if (npts == 0 ) return 0;

    // should check status in here...
    // start off by clocking first byte into latches:

  //  mask1 = FIFO | FIFO_OFF | MR_OFF | DSP_NRESET | R_ON;
  //  mask2 = FIFO | FIFO_OFF | MR_OFF | DSP_NRESET;
  
  mask3 = FIFO | FIFO_OFF | MR_OFF | DSP_NRESET | OEA;
  mask4 = FIFO | FIFO_OFF | MR_OFF | DSP_NRESET | R_ON;

  /* here is 'the deal'
     If the fifo was empty, we need two extra read clock cycles before
     the third one loads the first word into the registers for reading.

     The only way we have to see if the fifo was empty or not is two check
     the first two words against the last word.  If all three are identical,
     we throw the first two away.
  */
  
  /* do the first two reads separately: */

  outb(mask4, EPP_ADDR+port);
  low =inb(EPP_DATA+port);
  
  outb(mask3, EPP_ADDR+port);
  high = inb(EPP_DATA+port);
  
  data[0] = low +256 * high;
  if (data[0] > 32767) data[0]=data[0]-65536;


  outb(mask4, EPP_ADDR+port);
  low =inb(EPP_DATA+port);
  
  outb(mask3, EPP_ADDR+port);
  high = inb(EPP_DATA+port);
    
  data[1] = low +256 * high;
  if (data[1] > 32767) data[1]=data[1]-65536;
    

  if ((data[0] == data[1]) && (data[0] == last_word)){
    // need to start over
    istart = 0;
    //    fprintf(stderr,"apparently fifo was empty\n");
  }
  else{
    istart = 2;
    //    fprintf(stderr,"fifo was not empty\n");
  }
  

  for(i=istart;i<npts*2;i++){
    
    outb(mask4, EPP_ADDR+port);
    low =inb(EPP_DATA+port);
    
    outb(mask3, EPP_ADDR+port);
    high = inb(EPP_DATA+port);
    
    data[i] = low +256 * high;
    
    if (data[i] > 32767) data[i]=data[i]-65536;
  }

  last_word = data[2*npts-1]; //save our last word.

  return npts*2; // return the number of points read

}


// *********************** DSP MICROPORT COMMANDS

void reset_dsp()
{ 
  mask = DSP  | DSP_NWR | DSP_NRD ;
  outb(mask, EPP_ADDR+port);  
  mask = DSP  | DSP_NWR | DSP_NRD | DSP_NRESET;
  outb(mask, EPP_ADDR+port);
}


void write_micro(unsigned char addr, unsigned char data)
// Pre: DSP microport data lines selected (control lines are active)
// Post: epp data lines D0-D7 are sent to an internal address location
// in the DSP specified by A0-A2
{
  unsigned char shift;
  shift = addr*2;
  mask = DSP | DSP_NCS | shift | DSP_NWR | DSP_NRD | DSP_NRESET;
  outb(mask, EPP_ADDR+port);
  outb(data, EPP_DATA+port);
  mask = DSP  | shift | DSP_NRD | DSP_NRESET;
  outb(mask, EPP_ADDR+port);
  mask = DSP | DSP_NCS | shift | DSP_NWR | DSP_NRD | DSP_NRESET;
  outb(mask, EPP_ADDR+port);
}



unsigned char read_full()
{
  return  inb(SPP_CTRL+port);
  
}

void dsp_close_port_epp(){
  //int result;

//result = ioperm(port,8,0);
port=0;
}


