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

/* AD9850.c  program to load frequencies into the AD9850 
   evaluation board */

#include <signal.h>
#include <stdio.h>
#include <sys/perm.h>
#include <sys/io.h>   //for inb, outb, etc.
#include <string.h>
#include <math.h>
#include <unistd.h>
#include "ad9850.h"

#define SPP_DATA 0x00
#define SPP_STAT 0x01
#define SPP_CTRL 0x02
#define EPP_ADDR 0x03
#define EPP_DATA 0x04


#define TWOTHIRTYTWO 4294967296.
// values for the control port
#define nRRESET 1
#define WWCLK 4
#define nFFQUD 2
#define nSTROBE 8

void toggle_strobe();
void toggle_wwclk();

static unsigned char control;
unsigned int t=10;
static int have_port = 0;
static int port;


int close_port_ad9850(){
  if (have_port == 1)
    ioperm(port,8,0);
    
  return 0;
}


int init_port_ad9850(int new_port){
  int i;
  if (have_port == 0){
    i = ioperm(new_port,8,1);
    if (i<0){
      fprintf(stderr,"Can't get permission to access ad9850 port: %o\n",port);
      return -1;
      }
    have_port = 1;
    port = new_port;
  }
  else fprintf(stderr,"init_port_ad9850: already have port\n");
  return 0;
}

int reset_ad9850(){

  //  fprintf(stderr,"in reset_ad9850\n");
  outb(0,port);  // set port data values to all 0
  control = nRRESET+nFFQUD+nSTROBE;
  outb(control,port+SPP_CTRL); // set all the control port to 0 as well
  toggle_strobe();
  // these 3 are hardware inverted.
  // now do a reset
  control = control ^ nRRESET;
  //  fprintf(stderr,"to do reset, control = %i\n",(int) control);
  outb(control,port+SPP_CTRL); // set all the control port to 0 as well

  // now toggle the strobe
  toggle_strobe();
  // ok, so now the reset bit on the output should be set high, and this should be latched 
  return 0;

}
int setup_ad9850()
{
  unsigned long nco,clk=120000000;
  double an;
  int i;
  unsigned char status,bb[4];
  //  unsigned long an2;
  double got_freq;


  if (have_port == 0){
    fprintf(stderr,"Called setup_ad9850, but don't have port\n");
    return 0;
  }

    an = 21093750.; // gives an integral frequency
    //an = 21000000.;

  //  fprintf(stderr,"ad9850: using clk: %li and an: %li\n",(long int) clk,(long int) an);

  nco = TWOTHIRTYTWO  * (double) an/(double)clk;

 
   got_freq = ((double) nco) * ((double) clk) / TWOTHIRTYTWO;

   fprintf(stderr,"ad9850: nco phase inc: %li, actual freq: %12.8f\n",nco, got_freq);

   reset_ad9850();



  // read in the status register and see if its there on the "check" line:

  status=inb(port+SPP_STAT);
  //  fprintf(stderr,"status: %i\n",status&255);
  // the check line is bit 3 and should now be high 

  if ( (status & 8) == 0)
    fprintf(stderr,"after setting reset high, check bit is not high\n");


  // now set reset low and latch it in

  control = control ^ nRRESET;
  //  fprintf(stderr,"un resetting: control = %i\n",(int) control);
  outb(control,port+SPP_CTRL); // set all the control port to 0 as well
  toggle_strobe();

  // ok, so now the reset bit on the output should be set low, and this should be latched 
  // read in the status register and see if its there on the "check" line:

  status=inb(port+SPP_STAT);
  //  fprintf(stderr,"status: %i\n",status&255);

  // the check line is bit 3 and should now be low
  if ( (status & 8) == 8)
    fprintf(stderr,"after setting reset low, check bit is not low\n");

  // ok, now feed out the 5 data bytes:
  for (i=3;i>=0;i--){
    bb[i] = nco & 0xff;
    nco= nco >>8;
  }
  //  an2 = bb[3]+256*bb[2]+bb[1]*256*256+bb[0]*256*256*256;


  outb(0,port); // top five bits are phase
  toggle_strobe();

  toggle_wwclk();


  for(i=0;i<4;i++){
    //    fprintf(stderr,"byte: %x\n",bb[i]);
    outb(bb[i],port);
    toggle_strobe();
    toggle_wwclk();
  }


  // toggles the FFQUD which should set it going.
  control = control ^ nFFQUD;
  outb(control,port+SPP_CTRL);
  toggle_strobe();
  control = control ^ nFFQUD;
  outb(control,port+SPP_CTRL);
  toggle_strobe();

  //  fprintf(stderr,"done, just toggled the FFQUD line\n");


    

  return 0;
}

void toggle_strobe(){
  int i;
    //  usleep(t); // apparently, this is (un)necessary.
  for (i=0;i<5000;i++);
  control = control ^ nSTROBE;
  //  fprintf(stderr,"to toggle strobe, control = %i\n",(int) control);
  outb(control,port+SPP_CTRL); // set all the control port to 0 as well

  control = control ^ nSTROBE;
  //  fprintf(stderr,"to toggle strobe, control = %i\n",(int) control);
  outb(control,port+SPP_CTRL); // set all the control port to 0 as well
}

void toggle_wwclk(){

  control = control ^ WWCLK;
  outb(control,port+SPP_CTRL);
  toggle_strobe();

  control = control ^ WWCLK;
  outb(control,port+SPP_CTRL);
  toggle_strobe();
}

