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

// March 2001 Elsa Hansen
// Modified by Carl Michal for inclusion in Xnmr

// this should be fixed so that it only stops the clock when absolutely 
// necessry - ie, almost never.

//ARDUINO_DSP should be defined in the make file since we need it in a few places.
// using arduino to talk to the dsp board instead of parallel port

#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "param_utils.h"
#include "adepp-pb.h"
#include "dsp-pb.h"
// globals:  

static int last_sw=0;
static double last_rcvr_clk=0;
static double last_freq=0;
static int last_dgain=0;
static double last_dsp_ph=0;

char address [7] = {0,1,2,3,4,6,7};// 6 external microport address

void wr(void);
void writing ( int arrays [ ] [7], int num);

// if we're using arduino, then the port is in /dev/tty*
int setup_dsp(int sw, int p,double freq,int dgain, double dsp_ph, char force_setup,double rcvr_clk){
  // return values: 0 success
  //                -1 couldn't find filter file
  //                1 didn't do anything because sw and freq were the same
  //                2 changed frequency - need to resync
  //                -2 error setting up the port
  //                -3 dgain overflow
  int i,j,retval=0;
  int rcf_c [256] [7];
  int contr [14] [7];
  int data [256] [7];
  int taps;
  int phase;
  long int rcf [256];
  int pass; //DSPpass;
  int ssf[3];
  FILE* fptr1;
  int value;
  long int lvalue;
  long int b;
  double div;
  unsigned long int clk;
  unsigned long int nco;
  long unsigned int bb;
  int eo;
  char filter_name[PATH_LENGTH];

  float input_level,ol2,ol5; //olr;
  float mcic2,mcic5; //mrcf;
  float sum;


  if ((last_rcvr_clk == rcvr_clk) && (last_sw == sw) && (last_freq == freq) && (last_dgain == dgain) && (last_dsp_ph == dsp_ph) && force_setup == 0) {
    //    fprintf(stderr,"setup_dsp: sw and freq and dgain and dsp_ph the same, returning\n");
    if (query_ready_epp() == -1) return -2;
    select_device(FIFO);
    start_acquire_pulse(); // reset the fifo...
      return 1;
  }

  for( i=0; i<14; i++){
    for(j=1; j<7; j++)
      contr[i][j] = 0;
  }

  // otherwise, build a file name to look for a filter.
  
  // bypass the nco? 0 = no
  pass = 0;
  //bypass the dsp? 0 = no
  //  DSPpass = 0;
  
  snprintf(filter_name,PATH_LENGTH,"/usr/share/Xnmr/filters/%d.imp",sw);
  //  fprintf(stderr,"setup_dsp: filter_name is %s\n",filter_name);
  
  fptr1 = fopen(filter_name, "r");
  if ( fptr1 == NULL){
    fprintf (stderr,"dsp.c: filter file not found: %s\n",filter_name);
    return -1;
  }
  
  
  fscanf(fptr1,"%d&",&value);
  contr[6][0]=value-1; //MCIC2 decimation
  //  fprintf(stderr,"CIC2 dec: %d\n",value);
  mcic2=value;
  
  fscanf(fptr1,"%d&",&value);
  contr[8][0]=value-1; //MCIC5 decimation
  //  fprintf(stderr,"CIC5 dec: %d\n",value);
  mcic5=value;
  
  fscanf(fptr1,"%d&",&value);
  contr[10][0]=value-1; //RCF decimation
  //  fprintf(stderr,"RCF dec: %d\n",value);
  //  mrcf=value; //not used
  
  fscanf(fptr1,"%ld",&lvalue);
  clk = lvalue;
  //  fprintf(stderr,"clk speed from file: %ld, ignoring, using 60000000\n",clk);
  clk=rcvr_clk;
  
  fscanf(fptr1,"%d",&value);  
  if (value != 1) {
    fprintf(stderr,"reading filter file: value not 1\n");
    return -1;
  }
  fscanf(fptr1,"%d",&value);
  if (value != 1) {
    fprintf(stderr,"reading filter file: value not 1\n");
    return -1;
  }
  
  
  //    fseek (fptr1,35,0);
  // now read in the taps, and count them - can be a max of 256 I think.
  taps=256;
  for (i=0; i<256; i++)
    {
      eo=fscanf (fptr1, "%d", &value);
      if (eo != EOF){
	rcf [i]=value;
	//	fprintf(stderr,"tap: %d ",value);
      }
      else{
	taps=i;
	i=256;
      }
    }
  //  fprintf(stderr,"found %d taps\n",taps);
  fclose(fptr1);
  
  // process the taps into the bytes the dsp wants
  for (i=0; i<taps;i++){
    b=rcf [i];
    // tap value proper
    rcf_c[i][0] = b & 0xff;
    b = b>>8;
    rcf_c[i][1] = b & 0xff;
    b=b>>8;
    rcf_c[i][2] = b & 0x0f; 
    
    // the address it goes into:
    rcf_c [i] [4] = 0;
    rcf_c [i] [5] =i;
    rcf_c [i] [6] = 0;
  }
  
  for (i=0; i<256; i++){
    for(j=0; j<7;j++)
      data[i][j]=0;
  }
  
  for (i=0; i<256; i++){
    data [i][5] = i;
    data [i][6] = 1;
  }


  
  contr [0] [0]=1; // keep in soft reset for now
  contr [1] [0] = pass+6 ; // bypass nco? 6= enable phase and amp dither.
  contr [2] [0]=255;  // these 4 bytes are the NCO Sync mask- should be FFFFFFFF
  contr [2] [1]=255;
  contr [2] [2]=255;
  contr [2] [3]=255;
  contr [2] [4]=0; // what the heck is this?
  // 3 is the frequency, set elsewhere.
  // phase offset
  phase = (int)floor(dsp_ph/360. * (double)0xffff);    //recast dsp_phase for transfer
  contr [4] [0]= phase & 0xff;
  phase=phase>>8;
  contr [4] [1]= phase & 0xff;
  contr [4] [2]=0;
  contr [4] [3]=0;
  contr [4] [4]=0;
  //  fprintf(stderr,"phases:%i %i %i %i %i %i\n",phase, contr[4][0],contr[4][1],contr[4][2],contr[4][3],contr[4][4]); 
  // 5 and 6 are the CIC2 scale, CIC2 decimation,
  // 7 and 8 are CIC5 scale and decimations
  // 9 and 10 are output scale and decimation
  contr [11] [0]=0; //RCF address offset
  contr [11] [1]=0;
  contr [12] [0] = taps -1; 
  contr [13] [0]=0; // reserved, but 0
  /* done above
  for( i=0; i<2; i++){
    for(j=1; j<5; j++)
      contr [i] [j] = 0;
  }
  */
  for (i=0; i<14; i++){
    contr [i] [5] =i;
    contr [i] [6] =3;
  }
  /* done above
  for(i=5; i<14; i++){
    for( j=1; j<5; j++)
      contr [i] [j] = 0;
  }
  */

  // look out for under sampling:
  freq = freq - floor(freq/clk)*clk;
  if (freq > clk/2) freq=clk-freq;
  
  // 0-30 and 60-90 work by themselves
  //  fprintf(stderr,"receiver trying to get freq:%f\n",freq);
  div = freq/(double)clk;
  nco = (1ULL<<32)*div;
  //  fprintf (stderr,"receiver: NCO = %lu, freq is: %12.8f \n", nco,(double)((double) clk) * nco/4294967296ULL);
  bb=nco;
  
  // load the frequency bytes
  for (i=0; i<4; i++){
    contr [3] [i] = 0xff & bb;
    bb = bb >>8;
    //    fprintf(stderr,"freq byte: %x\n",contr[3][i]);
  }
  
  //calculate the scale factors
  //  ok, dgain is the total gain we're asked to get by the user

  input_level = 1/pow(2,dgain);

  ssf[0] = (int)  ceil ( log( mcic2*mcic2*input_level )/0.693148) -2;
  if (ssf[0] < 0) ssf[0] = 0;
  if (ssf[0] > 6) ssf[0] = 6;

  ol2 = input_level/pow(2,ssf[0]+2)*mcic2*mcic2;
  // this isn't what the data sheet shows, but it doesn't make any sense.

  ssf[1] = (int) ceil ( log( pow(mcic5,5)*ol2 )/0.693148 ) - 5;
  if (ssf[1] < 0) ssf[1] = 0;
  if (ssf[1] > 20) ssf[1] = 20; // was if > 20 then = 6 ???


  ol5 = pow( mcic5, 5.0)*ol2/pow( 2.0, ssf[1]+5.0);

  // the rcf gain is a bit of a mess
  sum = 0.;
  for(i = 0 ; i < taps ; i++)
    sum += (float) rcf[i];
  sum /= 524288; // 2^19 ?

  ssf[2] = (int) ceil( log( sum*ol5 )/0.693147 ) + 4;
  if (ssf[2] > 7) ssf[2] = 7; // was if > 7 then = 6 ??
  //  olr = sum*ol5 * pow(2,(4-ssf[2])); // not used
  if (ssf[2] < 0)  return -3;
  

  //  fprintf(stderr,"dgain: %d  input_level: %f  \n",dgain,input_level);
  //  fprintf(stderr,"scale factors: %d %d %d\n",ssf[0],ssf[1],ssf[2]);
  //  fprintf(stderr,"output levels: %f, %f %f\n",ol2,ol5,olr);
  //  fprintf(stderr,"sum of taps:  %f\n",sum);

  contr [5] [0] = (0x07 & ssf[0]);
  contr [7] [0] = (0x1f & ssf[1]);
  contr [9] [0] = (0x07 & ssf[2]);

  // if we're changing frequency, do the whole deal
  if (freq != last_freq || force_setup == 1 || last_rcvr_clk != rcvr_clk){
    /*    if (force_setup == 1)
	  fprintf(stderr,"in setup_dsp, got force_setup\n"); */
    //*********** initialize dsp ***********//
    retval = +2;
    i = initialize_dsp_epp(p); // passes in the port to use.
    if (i != 0) return -2;
    select_device(DSP);
    reset_dsp();

  //********* program dsp *************//
    writing (rcf_c,taps);
    writing (data,256);
    //    fprintf(stderr,"RCF written\n\n");

    writing (contr,14);
    //    fprintf(stderr,"\n0x300-0x30D registers written");
    // fprintf (stderr,"\nDownLoad Completed");
  
    contr [0] [0] =0;
    writing (contr,1); // take out of soft reset
  //  fprintf(stderr,"\nAD6620 taken out of soft reset\n");
  }
  else{
    if (query_ready_epp() == -1) return -2;
    select_device(DSP);
    if (sw != last_sw ){ // this isn't recommended in the ad6620 manual
      fprintf(stderr,"got new sw, updating taps, ntaps, and decimations\n");
      writing(rcf_c,taps);
      writing( &(contr [6]),1); // decimations:
      writing( &(contr [8]),1);
      writing( &(contr [10]),1);
      writing(&(contr [5]),1); //scale factors
      writing(&(contr [7]),1);
      writing(&(contr [9]),1);
      writing(&(contr [12]),1); // number of taps
    }
    if (dgain != last_dgain ){ 
      // we're just changing gain - just write the scale factors
      fprintf(stderr,"Skipped most programming of DSP, updating scale factors only\n");
      writing(&(contr [5]),1);
      writing(&(contr [7]),1);
      writing(&(contr [9]),1);
    }
    if (dsp_ph != last_dsp_ph ){ 
      // we're just changing dsp_ph - just write the phases
      fprintf(stderr,"Skipped most programming of DSP, updating NCO phase only\n");
      writing(&(contr [4]),1);
    }
  }

  //************** capture data into fifo ************//
  
  select_device(FIFO);
  //  reset_fifo();  this is done in start_acquire_pulse

  //  ffprintf(stderr,stderr,"start acquire \n");
  start_acquire_pulse(); // waits for pulse prog to do acq

  last_freq = freq;
  last_sw = sw;
  last_rcvr_clk = rcvr_clk;
  last_dgain = dgain;
  last_dsp_ph = dsp_ph;


  return retval; // gets set to +1 if we set the frequency.
}

void writing(int ar[][7],int num)
  {
    int d,i;
    for(d=0;d<num;d++)
      for (i=6;i>-1;i--)
	  write_micro(address[i], ar[d][i]);
  }

int dsp_request_data(int npts, int receiver_model){

  return request_data_epp(npts,receiver_model);
}

int read_fifo(int npts,int *data, int mode,int receiver_model){
  int result;
  // read npts complex points from the fifo
  result=read_fifo_epp(npts,data,receiver_model);
  if (mode == STANDARD_MODE) // don't do this in noisy mode - messes us up.
    start_acquire_pulse();
  return result;
}

void dsp_close_port(){
  dsp_close_port_epp();
}

void dsp_reset_fifo(){
  start_acquire_pulse();
}
