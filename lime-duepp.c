/* TODO:
- if try to init a lime beyond NUM_TX, ignore it, give error.

- fix up join limes to make sure that streams aren't running. If they
are, set stop_lime to kill and wait a while before setting to thread_exit?

- to try: set flushPartialPacket bit on last buffer before stop sending!

-- think about KILL signal during sync_lime

-- Clean up stream start/end!
right now we start streams everytime, even if they were already running?

Need to not overflow read buffer during phase lock?

calibration - phase, leakage, receiver? Do short version of amp change if freq is the same, cache amp value...

(gain and freq changes alter the phase!)

Collect multi receiver data?

read back frequency, make sure its right - maybe update ui - Ug.
Better might be to have ui know what frequencies can be chosen.

Filtering - do better!

error checking on lime function calls

Test - pulse program elements
Test - frequency offsets
Test - multiple boards.
Test - bandwidth capable? Multiple boards?
Test - SNR (vs bandwidth...)


Done:
- Longer term: synchronize by setting timestamp to 0 with GPIO?
- block signals from lime's threads. (block signals in before starting lime stuff then unblock)
  hmm. Still sometimes see unwanted messages about libusb calls interrupted, but I don't think
  they are dangerous. Maybe it comes from closing down streams before the TX fifos are empty?
- start-up crap. Hooray!
- gain setting on TX and RX?

 */
/* some lessons:

1. to get rid of startup crap: run transmit channels with gain turned down and without synchronization
2. to make transmit work reliably, don't go turning on tx stream off
3. maybe want fifo size not too big for reliability?

4. fifofilled seems to change in increments of 1360? Hmm. maybe make buffsize a multiple of 1360. There's a reason for this - packet size to LMS7002m when
data is packed as 12 bit.
making buffsize=9520 seems to have helped a lot? This seems crucial

5. seems like when we're stopping sending, get most reliable behaviour if we send a full fifo's worth of 0's
before stopping.
6. don't start sending with a non-zero first element in the first buffer.
- seems need to need all streams on all the time.
- large fifo size seems fine.
- skipping transmit of zeros seems to cause trouble. Weird. Maybe can't have
samples queued for different times?


Crap - how to deal with  KILL? can't close then open - segfault - works if you
set device to NULL before reopening.
     - can't deinit then init. Doesn't seem to work afterwards.
     - maybe set tx gain down to 0, but then need to wait for end of seq.

- need to do something smarter with phase for receiver.

To complain about:
 - crap at startup
 - how to interrupt already loaded tx samples?
 - changing gain causes more crap to come out.
 - can't get less than x2 oversampling on transmit.
 - weirdness of buffsize - needs to be multiple of 1360? Or needs to be multiple of 1020 for 16 bit samples?

*/

#define LIME_SERIAL
#include "h_config-duelime.h"
#undef LIME_SERIAL

#include <lime/LimeSuite.h>
#include <stdio.h>
#include <string.h>
#include "param_utils.h"
#include "due-pp-lib.h"
#include "shm_prog-duelime.h"
#include <semaphore.h>
#include <math.h>
#include "acq-duelime.h"
#include <pthread.h>
#include "duepp.h"
#include <errno.h>
#include <signal.h>
#include <unistd.h>
extern struct prog_shm_t*  prog_shm;        //These are shared memory structures and must be initialized
#include "lime-duepp.h"
#include "pulse_hardware-duepp.h"

// strangely, we seem to get data underruns with NO_SEND_ZEROS in noise spectroscopy. Even though the point is
// to lighten the load. Weird. This seems to be mostly avoided as long as we ensure that there is enough room in the
// fifo to hold the data before SendStream.
// Strange things happen sometimes though - we seem to get unwanted rf output occasionally.
//#define NO_SEND_ZEROS

#define SYNC_LIMES

// to use external 10 MHz ref on first board, and chain the 30.72 signal from others.
#define LIME_EXT_REF

void *tx_thread_func(void *threadno);
void *rx_thread_func(void *threadno);

lms_device_t *limes[NUM_TX];


lms_stream_t rx_streams[NUM_TX];// arguments are board, channel
lms_stream_t tx_streams[NUM_TX];

lms_stream_meta_t rx_metadata[NUM_TX],tx_metadata[NUM_TX];
#define THRESH 1000
#define STACK_SIZE 50
int16_t *isin,*icos;
int16_t *store_isin;
//int buffsize = 1024*8; //complex samples per transfer
int buffsize = 9520;
int16_t *sbuff[NUM_TX],*rbuff[NUM_TX]; // buffers
int16_t *buff0; // buffer full of 0's 
volatile char streams_running = 0;
#define PHASE_TARGET 900 //at 10 MHz, 1124 is on a boundary where the timing varies by a 20ns step
// 1.6us = 8192, aim to move by 10ns. About 5.12 units per ns. 1000 seems good!

// local copy of pulse prog:
/*
volatile unsigned char l_txopcodes[NUM_TX][MAX_TX_EVENTS+1]; // 1 = transmit event, 2= set offset freq. 0 = end
volatile uint16_t l_txopinst[NUM_TX][MAX_TX_EVENTS];
volatile uint32_t l_txparams[NUM_TX][MAX_TX_EVENTS+1][3];// holds I and Q values or offset freq and rx phase
volatile uint64_t l_txtimes[NUM_TX][MAX_TX_EVENTS+1];
volatile int l_txevents[NUM_TX]; // number of events in the table for the first pass of tx setting events.
*/

volatile double l_txfreqs[NUM_TX];
volatile double l_txncofreqs[NUM_TX];
volatile int l_txgains[NUM_TX];
volatile int l_rxgains[NUM_TX];
volatile int l_lime_sync_event[NUM_TX];
volatile uint64_t l_rxpoints[NUM_TX];
float l_rx_sw[NUM_TX];
volatile char do_tx_cleanup[NUM_TX] = {[0 ... NUM_TX-1] = 0} ;
volatile int stop_acq=0;

int64_t *l_rxbuffer = NULL; // this is where we store our received data on the fly.
int64_t *l_buffer; // the data gets copied here when we're done with it.
int  l_npts;


// these should be mutex protected? These are a ring buffer. num_rx_events is the write pointer.
//uint64_t rx_start_stamps[NUM_TX][MAX_TX_EVENTS];
//uint64_t rx_end_stamps[NUM_TX][MAX_TX_EVENTS];
//float rx_phases[NUM_TX][MAX_TX_EVENTS];
uint64_t *rx_start_stamps[NUM_TX];
uint64_t *rx_end_stamps[NUM_TX];
float *rx_phases[NUM_TX];
unsigned int num_rx_events[NUM_TX];
char rx_reset[NUM_TX];


/* Semaphores:
 txsems tell the tx threads to start up. Used each scan. The tx threads go to sleep
 when they run out of events.

 rxsems tell the rx threads to start up. Used once per acquisition. After the txthreads
 are done with synchronization, they start the rx threads.
 
 rxwrite is used by the rx threads to tell acq that all threads that have data to collect
 are finished collecting it for the current scan. Based on number of points.

 streams is used by all tx and rx threads to indicate that they have shut-off streaming.
 streams get hit by each thread at end of acquisition (not scan).
 acq uses it before calling start_limes - hmm, could just be part of start_limes

 txdone tells acq that all the tx threads finished their run. Ick.

*/

volatile bool threads_launched = 0;
pthread_t tx_threads[NUM_TX],rx_threads[NUM_TX];
sem_t txsems[NUM_TX],rxsems[NUM_TX],rxwrite,streams,txdone;
volatile uint64_t ending_stamp[NUM_TX]; // tx thread calculates the timestamp where the rx thread will finish. May be temporary.
int threadnos[NUM_TX];
int nlime=0; // how many lime sdr's we find.
int limes_opened=0; // how many we actually used
void block_signals(){
  sigset_t sigset;
  sigemptyset(&sigset);
  sigaddset(&sigset,SIGUSR1);
  sigaddset(&sigset,SIGALRM);
  
  sigprocmask(SIG_BLOCK,&sigset,NULL);
}

void unblock_signals(){
  sigset_t sigset;
  sigemptyset(&sigset);
  sigaddset(&sigset,SIGUSR1);
  sigaddset(&sigset,SIGALRM);
  
  sigprocmask(SIG_UNBLOCK,&sigset,NULL);
}



int open_limes(){
  lms_info_str_t lmslist[8]; // info about the limes
  int i,j,count = 0;
  printf("open_limes, calling LMS_GetDeviceList\n");
  block_signals();

  // LMS driver creates some threads when we get the device list. Want them
  // to ignore SIGUSR1.  New threads also created when streaming is started.
  // looks like we need to block SIGUSR1 when we get device list and when we
  // call StreamStart. Looks like that's all that should be needed - nope. doesn't do it...
  
  nlime = LMS_GetDeviceList(lmslist);
  unblock_signals();

  for (i=0;i<nlime;i++)
    printf("Lime %i, %s\n",i,lmslist[i]);
  // NUM_TX is how many limes we're expecting to find, nlime is how many we actually find on the system.

  // now, the lmslist entries contain unique board serial numbers. Use them to identify which board is which.
  // serials configured in h_config-duelime.h as lime_serials
  
  printf("Found %i lime SDRs\n",nlime);
  if (nlime  < NUM_TX) {
    printf("\n\n\nFOUND %i LIMES, BUT WANTED %i\n",nlime,NUM_TX);
    return -1;
  }
  for(i=0;i<NUM_TX;i++){
    // open the sdr:
    printf("Need to look for lime: %s\n",lime_serials[i]);
    for(j=0;j<nlime;j++){
      if (strstr(lmslist[j],lime_serials[i]) != NULL){// found it!
	block_signals();
	if (LMS_Open(&limes[i],lmslist[j],NULL)){ // should return 0
	  printf("Failed to open lime %i\n",i);
	}
	else {
	  count += 1;
	  printf("found lime: %s\n",lime_serials[i]);
	}
	unblock_signals();
	//    if (LMS_Reset(&limes[i])) // should return 0
	//      printf("Failed to open lime 0\n");

      }
    }
  }
  if (count < NUM_TX){
    for (i=0;i<count;i++)
      LMS_Close(limes[i]);
    return -1;
  }
  limes_opened = count;

  for (i=0;i<NUM_TX;i++){
    rx_start_stamps[i] = (uint64_t *) malloc(sizeof(uint64_t) *MAX_TX_EVENTS);
    rx_end_stamps[i] = (uint64_t *) malloc(sizeof(uint64_t) *MAX_TX_EVENTS);
    rx_phases[i] = (float *) malloc(sizeof(float) *MAX_TX_EVENTS);
  }
   
  buff0 = malloc((sizeof(int16_t))*buffsize*2);
  memset(buff0,0,2*buffsize*sizeof(int16_t));

  // build sin/cos table for frequency offsets
  icos = malloc(sizeof(*icos) * TRIGLEN);
  isin = malloc(sizeof(*isin) * TRIGLEN);
  store_isin = isin;
  for (i=0;i<TRIGLEN;i++){
    icos[i] = 8191*cos(2*M_PI*i/TRIGLEN);
    isin[i] = 8191*sin(2*M_PI*i/TRIGLEN);
  }

  printf("open limes opened %i\n",count);
  return count;
}

void kill_gains(){
  int i;
  for (i=0; i<NUM_TX;i++)
    LMS_SetGaindB(limes[i],LMS_CH_TX,0,0);
}


int init_limes(){
  int i;
  //  uint16_t val;
  unsigned char gpiobuff;
  printf("in init_limes\n");
  if (limes_opened < NUM_TX)
    return -1;
  block_signals();
  for (i=0;i<NUM_TX;i++){
    //this seems to cause problems with my pulse sync? Weird.
    LMS_SetAntenna(limes[i],LMS_CH_TX,0,LMS_PATH_NONE); // wrong channel - send rf start-up glitch to nowhere. 
    
    if (LMS_Init(limes[i]) != 0){
      printf("Failed to initialize lime %i\n",i);
    }
    gpiobuff=0;
    LMS_GPIOWrite(limes[i],&gpiobuff,1);
    // set all GPIOS to inputs - er, no
    //        gpiobuff = (1<<6) | (1<<7) | (1<<5) | (1<<1); // enable divided ADC clock output and XOR output and startup trigger and trigger for startup trigger.
    gpiobuff = (1<<6) | (1<<1); // enable divided ADC clock output and XOR output and startup trigger and trigger for startup trigger.
    LMS_GPIODirWrite(limes[i],&gpiobuff,1);

  }

  // except on master (first channel) GPIO 0 and GPIO1 are outputs
  /*
  gpiobuff = 0; // set both low
  LMS_GPIOWrite(limes[0],&gpiobuff,1);
  gpiobuff = 3;
  LMS_GPIODirWrite(limes[0],&gpiobuff,1);
  */
#ifdef LIME_EXT_REF
  /*
  for(i=0;i<limes_opened;i++){
    LMS_VCTCXORead(limes[i],&val);
    printf("before set clock freq, do VCTCXORead found val %i for lime %i\n",val,i);
  }
  */
  // set external 10 MHz reference
  printf("init_limes setting up external references\n");
  for (i=0;i<limes_opened;i++)
    LMS_SetClockFreq(limes[i], LMS_CLOCK_EXTREF, 10000000);
  printf("init_limes done setting up external references\n");
  /* 
  // others use 30.72 MHz output chained from previous.
// no - all use 10MHz reference.
  for (i=1;i<limes_opened;i++)
    LMS_SetClockFreq(limes[i], LMS_CLOCK_EXTREF, 30720000);
  */
#else
  //  for (i=0;i<limes_opened;i++)
  //    LMS_VCTCXOWrite(limes[i],125);
#endif
  
  printf("lime init complete,setting up streams\n");
  
  for (i=0;i<NUM_TX;i++){ 
    double rfrate,hostrate;
    LMS_SetGaindB(limes[i],LMS_CH_TX,0,0);// so nothing comes out
    
    LMS_EnableChannel(limes[i], LMS_CH_RX, 0, true);
    LMS_EnableChannel(limes[i], LMS_CH_TX, 0, true);
    LMS_SetLOFrequency(limes[i], LMS_CH_RX, 0, 2e9);
    LMS_SetLOFrequency(limes[i], LMS_CH_TX, 0, 2e9);
    LMS_SetAntenna(limes[i],LMS_CH_RX,0,LMS_PATH_LNAL);
    LMS_SetSampleRate(limes[i], TX_SAMPLE_RATE,1); // device, rate and oversample
    LMS_GetSampleRate(limes[i],LMS_CH_TX,0,&hostrate,&rfrate);
    printf("got sample rates of: %f %f\n",hostrate,rfrate);
    //    LMS_SetLPFBW(limes[i],LMS_CH_TX,0,100e6);
    //    LMS_SetLPFBW(limes[i],LMS_CH_TX,1,100e6); // hmmm- breaks things?
    //    LMS_SetLPFBW(limes[i],LMS_CH_RX,0,10e6);
    //    LMS_SetLPFBW(limes[i],LMS_CH_RX,1,10e6); // hmmm- breaks things?
    printf("\n\ninit lime: Called to set up LPF\n");
    LMS_WriteParam(limes[i], LMS7_MAC,2); // locks TX and RX together
    LMS_WriteParam(limes[i],LMS7_PD_LOCH_T2RBUF,0);
    LMS_WriteParam(limes[i],LMS7_MAC,1);
    LMS_WriteParam(limes[i],LMS7_PD_VCO,1);

    LMS_WriteParam(limes[i],LMS7_HBI_OVR_TXTSP,7); // bypass interpolator (need MAC set to 1 for channel A - this does make oversampling = 1
    // wow: bypassing decimator makes very, very bad things happen...
    //    LMS_WriteParam(limes[i],LMS7_HBD_OVR_RXTSP,7); // bypass decimator (need MAC set to 1 for channel A - this does make oversampling = 1

    //    LMS_WriteParam(limes[i],LMS7_TXRDCLK_MUX,0); // sets so reads out of txfifo
    // are timed by the RX clock. 1 = rx clk, 0 = clk from fpga
    
    // set up streaming:
    
    rx_streams[i].channel = 0;
    //    rx_streams[i].fifoSize = 9520*256; // .24 s at 10e6 sps
    //    rx_streams[i].fifoSize = 9520*256*2.5; // .24 s at 25e6
    rx_streams[i].fifoSize = 9520*256*5; // .24 s at 50e6
    
    rx_streams[i].throughputVsLatency = 0.5;
    rx_streams[i].isTx = false;
    rx_streams[i].dataFmt = LMS_FMT_I12;
    LMS_SetupStream(limes[i], &rx_streams[i]);
    
    tx_streams[i].channel = 0;
    tx_streams[i].fifoSize = 9520*32; // 30 ms at 10e6
    //    tx_streams[i].fifoSize = 9520*32*2.5; // 30 ms at 25e6
    //    tx_streams[i].fifoSize = 9520*32*5; // 30 ms at 50e6
    tx_streams[i].throughputVsLatency = 0.2; // 0 = best latency 1= best throughput
    tx_streams[i].isTx = true;
    tx_streams[i].dataFmt = LMS_FMT_I12;
    LMS_SetupStream(limes[i], &tx_streams[i]);
    
    // malloc data buffers:
    rbuff[i] = malloc((sizeof(int16_t))*buffsize*2);
    
    sbuff[i] = malloc((sizeof(int16_t))*buffsize*2);
    memset(sbuff[i],0,2*buffsize*sizeof(int16_t));
    
    rx_metadata[i].flushPartialPacket = false; // no effect in RX
    rx_metadata[i].waitForTimestamp = false; // no effect in RX
    tx_metadata[i].flushPartialPacket = false;
    tx_metadata[i].waitForTimestamp = true;


    LMS_SetAntenna(limes[i],LMS_CH_TX,0,LMS_PATH_TX1); // right channel

  }
      // later we'll set the frequency and nco frequency correctly, gains too.

  printf("returning from init_lime\n");
  unblock_signals();
  return limes_opened;
}

void deinit_limes(){
  int i;
  for (i=0;i<NUM_TX;i++){
    if (limes[i] != NULL){
      printf("deiniting lime %i\n",i);
      LMS_DestroyStream(limes[i],&tx_streams[i]);
      LMS_DestroyStream(limes[i],&rx_streams[i]);
      LMS_EnableChannel(limes[i], LMS_CH_RX, 0, false);
      LMS_EnableChannel(limes[i], LMS_CH_TX, 0, false);
    }
  }
  printf("done deiniting\n");
}
void close_limes(){
  int i;
  if (limes_opened == 0) return;
  printf("in close_limes\n");
  for (i=0;i<NUM_TX;i++){
    if (limes[i] != NULL){
      free(sbuff[i]);
      free(rbuff[i]);
      printf("calling close on lime %i\n",i);
      LMS_Close(limes[i]);
      limes[i] = NULL;
    }
    free(rx_phases[i]);
    free(rx_start_stamps[i]);
    free(rx_end_stamps[i]);

  }
  limes_opened = 0; 
  free(isin);
  free(icos);
  free(buff0);
  if (l_rxbuffer != NULL){
    free(l_rxbuffer);
    l_rxbuffer = NULL;
  }

}

int wait_till_streams_done(){
  int j,eo;
  if (threads_launched && streams_running){
    printf("waiting for all streams to be shutdown\n");
    for (j=0;j<2*NUM_TX;j++) // TODO what about getting signals in here?
      do{
	eo = sem_wait(&streams);
	printf("start-lime: sem_wait returned\n");
      } while (eo == -1 && errno == EINTR);// woken by a signal
    printf("start-lime, done waiting for all streams to be shutdown\n");
  }
  //streams_running = 0;
  return 0;
}


int prep_lime(int npts, int64_t *buffer){
  int i,j,eo;
  static double last_freq[NUM_TX];
  static double last_nco_freq[NUM_TX];
  static int last_txgain[NUM_TX];
  static int last_rxgain[NUM_TX];
  static double nco_freqs[16];
  int lock_needed  = 0;
  
  // if its the first time in an acquisition:
  /*  
  if (threads_launched && prog_shm->wait_for_sync){
    printf("start-lime, waiting for all streams to be shutdown\n");
    for (j=0;j<2*NUM_TX;j++) // TODO what about getting signals in here?
      do{
	eo = sem_wait(&streams);
	printf("start-lime: sem_wait returned\n");
      } while (eo == -1 && errno == EINTR);// woken by a signal
    printf("start-lime, done waiting for all streams to be shutdown\n");
  }
  */
  // this is if we're not doing the first acquisition. ie streams should all be running.
  if (threads_launched && prog_shm->wait_for_sync == 0){
    for (j=0;j<NUM_TX;j++)
      do{
	eo = sem_wait(&txdone);
	printf("start-lime: sem_wait returned\n");
      } while (eo == -1 && errno == EINTR);// woken by a signal
    printf("start-lime, done waiting for tx threads from previous to be done\n");
  }
  printf("in prep_lime\n");

  // if we were woken by a signal, just quit:
  if (prog_shm->stop_lime < 0){
    printf("prep_lime aborting\n");
    return -1;
  }
  
  /* do this in acq proper.
  prog_shm->etxprogno = prog_shm->txprogno; // this is the program that will be executed.
  prog_shm->txprogno = ( prog_shm->txprogno + 1 ) %2; // calc next one in other buffer.
  */
  
  l_npts = npts;
  l_buffer = buffer; 

  if (l_rxbuffer != NULL){
    free(l_rxbuffer);
  }
  l_rxbuffer = (int64_t *) malloc(sizeof(int64_t) * npts*2);
  

  /*
  if (prog_shm->lime_sync_event[0] == 0) {
    printf("start_lime: aborting, no sync_event found\n");
    return -1; // no lime_sync event was specified.
  }
  */
  // set data to 0.
  memset(buffer,0,2*sizeof(int64_t)*npts);
  memset(l_rxbuffer,0,2*sizeof(int64_t)*npts);


  // copy bits from the pulse program into module local variables:
  for(j=0;j<NUM_TX;j++){
    //    printf("lime_start, number of events for thread %i is %i\n",j,prog_shm->txprog[prog_shm->etxprogno].txevents[j]);
    do_tx_cleanup[j] = 0;
    l_rxpoints[j] = prog_shm->rxpoints[j];
    l_txfreqs[j] = prog_shm->txfreqs[j];
    l_txncofreqs[j] = prog_shm->txncofreqs[j];
    l_txgains[j] = prog_shm->txgains[j];
    l_lime_sync_event[j] = prog_shm->lime_sync_event[j];

    l_rx_sw[j] = prog_shm->rx_sw[j];
    l_rxgains[j] = prog_shm->rx_gains[j];
    
  }
  
// deal with changes of gain and freq  - only do any of it if its the first scan of an acquisition
  for(i=0;i<NUM_TX;i++){
    if (last_freq[i] != l_txfreqs[i] || last_nco_freq[i] != l_txncofreqs[i] || last_txgain[i] != l_txgains[i] || last_rxgain[i] != l_rxgains[i]){
      static char first[NUM_TX] ={1,1};
      if (prog_shm->wait_for_sync == 0){
	printf("start_lime: got a change of frequency or gain on lime %i, but acquistion is running. Ignoring!\n",i);
	continue;
      }
      printf("start lime %f, nco: %f\n",l_txfreqs[i],l_txncofreqs[i]);
      fflush(stdout);
      block_signals();
      // now, in here, we should flip into loopback mode and calibrate the tx/rx phase?
      lock_needed |= (1<<i);
      LMS_SetAntenna(limes[i],LMS_CH_TX,0,LMS_PATH_TX2); // wrong channel - hope glitch goes nowhere.
      if (last_freq[i] != l_txfreqs[i] || last_nco_freq[i] != l_txncofreqs[i]){
	if (first[i]){
	LMS_SetGaindB(limes[i],LMS_CH_TX,0,l_txgains[i]);
	LMS_SetGaindB(limes[i],LMS_CH_RX,0,70);
	}
	nco_freqs[0] = l_txncofreqs[i];
	LMS_SetLOFrequency(limes[i],LMS_CH_TX,0, l_txfreqs[i]+l_txncofreqs[i]);
	LMS_SetNCOFrequency(limes[i], LMS_CH_TX,0,nco_freqs,0.); 
	LMS_SetNCOFrequency(limes[i], LMS_CH_RX,0,nco_freqs,0.);
	LMS_SetNCOIndex(limes[i], LMS_CH_TX,0,-1,0); // disable
	LMS_SetNCOIndex(limes[i], LMS_CH_RX,0,-1,0); // disable NCO for calibration?
	//	LMS_SetNCOIndex(limes[i], LMS_CH_TX,0,0,1); // downconvert
	//	LMS_SetNCOIndex(limes[i], LMS_CH_RX,0,0,1); // downconvert
	last_freq[i] = l_txfreqs[i];
	last_nco_freq[i] = nco_freqs[0];
      
      //      LMS_SetNCOFrequency(limes[i], LMS_CH_TX,1,nco_freqs,0.); // on sync channel go opposite way
      //      LMS_SetNCOFrequency(limes[i], LMS_CH_RX,1,nco_freqs,0.);
      //      LMS_SetNCOIndex(limes[i], LMS_CH_TX,1,0,0); // upconvert
      //      LMS_SetNCOIndex(limes[i], LMS_CH_RX,1,0,1); // downconvert
      
      ////// do calibration of TX
	//	if (first[i] == 1)
	LMS_Calibrate(limes[i],LMS_CH_TX,0,2.5e6,0);
	
	//      LMS_Calibrate(limes[i],LMS_CH_RX,0,2.5e6,1); //hmm - this sends out crap. Darn

	LMS_SetNCOIndex(limes[i], LMS_CH_TX,0,0,1); // downconvert
	LMS_SetNCOIndex(limes[i], LMS_CH_RX,0,0,1); // downconvert
      

	//	if (first[i] == 1)
	  LMS_SetGaindB(limes[i],LMS_CH_TX,0,0);
      
	tx_metadata[i].waitForTimestamp= false;
	// looks like changing freq requires running out the start-up crap again?
	
	LMS_StartStream(&tx_streams[i]);
	printf("startup, sending: %i buffers of 0's\n",2*tx_streams[i].fifoSize/buffsize);
	for (j=0;j<2* tx_streams[i].fifoSize/buffsize;j++){ // send two full fifos of zeros.
	  LMS_SendStream(&tx_streams[i],buff0,buffsize,&tx_metadata[i],1000);
	}
	LMS_StopStream(&tx_streams[i]);
      
	printf("turning on RX DC corrector bypass:\n");
	// needs mac 1:
	LMS_WriteParam(limes[i],LMS7_MAC,1);
	LMS_WriteParam(limes[i],LMS7_DC_BYP_RXTSP,1);
  
	tx_metadata[i].waitForTimestamp= true;
      }

      // It is setting the TX gain that causes our timing variation!
      //      if (first[i] == 1)
	LMS_SetGaindB(limes[i],LMS_CH_TX,0,l_txgains[i]);

      LMS_SetGaindB(limes[i],LMS_CH_RX,0,l_rxgains[i]); 

      LMS_SetAntenna(limes[i],LMS_CH_TX,0,LMS_PATH_TX1); 
      ////// end calibration of TX
      first[i] = 0;
      last_rxgain[i] = l_rxgains[i];
      last_txgain[i] = l_txgains[i];
    }
    unblock_signals();
  }

  stop_acq = 0;
  printf("returning from prep_lime\n");
  return lock_needed;
}
// some service functions for sync_limes
void set_divider(int limeno,int gtotal){
  uint16_t val;
  //  val = gtotal >> 20;
  //LMS_WriteParam(limes[limeno],LMS7_INT_SDM_CGEN,val);
  //  val = (gtotal >> 16) & 0xf;
  //  LMS_WriteParam(limes[limeno],LMS7_FRAC_SDM_CGEN_MSB,val);
  val = (gtotal & 0xffff);
  LMS_WriteParam(limes[limeno],LMS7_FRAC_SDM_CGEN_LSB, val);
}

unsigned int calc_phase(int adcval,char direction){
  // returns a value between 0 and 4095 with best guess of current phase

  // seems like we need a total of 4200*2 dac units for a full cycle.
  // approximate with 4095
  if (direction >0)
    return adcval/2;
  else
    return 4095-adcval/2;
}

int lime_get_adc(int limeno){
  int current=0,j;
  for (j = 0 ; j < 4 ; j++ )
    current += pulse_hardware_read_adc(limeno);
  return (current+2)/4;

}
int gtotal[NUM_TX];
void catch_up_read(){
  int i,samplesRead;
  lms_stream_status_t status;

  for (i=0;i<NUM_TX;i++){
    do{// do some catch up...
      samplesRead = LMS_RecvStream(&rx_streams[i],rbuff[i],buffsize, &rx_metadata[i],1000);
      LMS_GetStreamStatus(&rx_streams[i],&status);
    }while (status.fifoFilledCount > 2*buffsize && prog_shm->stop_lime >= 0 );
  }

}

int lime_do_shift_delay(int limeno,int shift,unsigned int duration){
  set_divider(limeno,gtotal[limeno]+shift); // up by 80 means 1.65 s should give 1600ns full cycle.
  usleep(duration);
  set_divider(limeno,gtotal[limeno]);
  return 0;
  ///XXX todo, time it, and see if the usleep went way too long or short.
}

int lime_measure_phase(int limeno,unsigned  int *phase, char *direction){
  int val1,val2;
  *direction = 0;
    while (*direction == 0){
      //      usleep(7000); // 7 ms to settle.
      val1 = lime_get_adc(limeno);
      catch_up_read();

      lime_do_shift_delay(limeno,160,20000);

      catch_up_read();
      usleep(7000); // 7 ms to settle.
      catch_up_read();
      val2 = lime_get_adc(limeno);
      
      if (val2 > val1 && val1 > 200 && val2 < 3900)
	*direction = 1;
      else if (val2 < val1 && val2 > 200 && val1 < 3900)
	*direction = -1;
    }
    *phase = calc_phase(val2,*direction);
    printf("get phase for lime %i, with values: %i and %i\n",limeno,val1,val2);
    return 0;
}

int sync_lime(char *device){ // do pll-ish thing to bring sample clocks into sync.
  int i;
  unsigned char gpiobuff;
  uint16_t val;
  int gint[NUM_TX];
  int gfrac[NUM_TX];
  int gdiv[NUM_TX];
  int tries,phase_tries;
  unsigned int my_phase,delay;
  char direction = 0;
  int targetl,targeth;
  // make sure streams aren't running:
  //  return 0;
  if (streams_running){
    printf("in sync_lime, but streams already runnning?\n");
    return 0;
  }
  printf("in sync_lime\n");

  // fix tx/rx phase difference:
  for(i=0;i<NUM_TX;i++){
    gpiobuff = (1<<6)| (1<<7) |  (1<<1); // enable divided ADC clock output and XOR output and startup trigger and trigger for startup trigger.
    LMS_GPIODirWrite(limes[i],&gpiobuff,1);
    usleep(1000);
    phase_tries = 0;
    val = lime_get_adc(NUM_TX+i);
    //    catch_up_read();
    printf("lime %i, rx/tx phase diff is: %i\n",i,val);
    if (i == 0){
       targetl=00;
      targeth=500;
      //targetl=3500;
      //targeth=4095;
    }
    else{
      //targetl=0;
      //targeth=500;
      targetl=3500;
      targeth=4095;
    }
    while( (val < targetl || val > targeth) && phase_tries < 20){
      LMS_SetGaindB(limes[i],LMS_CH_TX,0,l_txgains[i]);
      usleep(1000);
      val = lime_get_adc(NUM_TX+i);
      //      catch_up_read();
      phase_tries += 1;
      printf("lime %i, rx/tx phase diff is: %i\n",i,val);
    }
    gpiobuff = (1<<6)|  (1<<1); // enable divided ADC clock output and XOR output and startup trigger and trigger for startup trigger.
    LMS_GPIODirWrite(limes[i],&gpiobuff,1);
  }


  
  for (i=0;i<NUM_TX;i++){
    // start streams
    /*
    // power off the CGEN output divider
    LMS_WriteParam(limes[i],LMS7_PD_FDIV_O_CGEN,1);
    usleep(100);
    LMS_WriteParam(limes[i],LMS7_PD_FDIV_O_CGEN,0);
    usleep(100);
    */
    /*    
    // or, hit reset button on whole CGEN module:
    LMS_WriteParam(limes[i],LMS7_RESET_N_CGEN,0);
    LMS_WriteParam(limes[i],LMS7_RESET_N_CGEN,1);
    usleep(500);
    */
    // then set up sample rate:
    //    LMS_SetSampleRate(limes[i], TX_SAMPLE_RATE,1); // device, rate and oversampl

    // hopefully that puts dividers all in same place?
    LMS_StartStream(&rx_streams[i]);
    
    // find the divider values
   LMS_ReadParam(limes[i],LMS7_DIV_OUTCH_CGEN,&val);
   gdiv[i] = val;

   LMS_ReadParam(limes[i],LMS7_INT_SDM_CGEN,&val);
   gint[i] = val;
   printf("value of reg INT_SDM_CGEN is: 0x%x\n",gint[i]);
   LMS_ReadParam(limes[i],LMS7_FRAC_SDM_CGEN_LSB,&val);
   gfrac[i] = val;
   printf("value of reg FRAC_SDM_CGEN_LSB is: 0x%x\n",gfrac[i]);
   LMS_ReadParam(limes[i],LMS7_FRAC_SDM_CGEN_MSB,&val);
   printf("value of reg FRAC_SDM_CGEN_MSB is: 0x%x\n",val);
   gfrac[i] |= val<<16;
   printf("divider int: %i frac: %i\n",gint[i],gfrac[i]);
   gtotal[i] = gfrac[i] | (gint[i]<<20);

   LMS_ReadParam(limes[i],LMS7_TXRDCLK_MUX,&val); // sets so reads out of txfifo
    // are timed by the RX clock.
   printf("lime %i, TXRDCLK_MUX is: %i\n",i,val);
   
   LMS_ReadParam(limes[i],LMS7_HBD_DLY,&val); 
   printf("lime %i, HBD_DLY is: %i\n",i,val);


  }
  /*

    
  // fix tx/rx phase difference:
  for(i=0;i<NUM_TX;i++){
    gpiobuff = (1<<6)| (1<<7) |  (1<<1); // enable divided ADC clock output and XOR output and startup trigger and trigger for startup trigger.
    LMS_GPIODirWrite(limes[i],&gpiobuff,1);
    usleep(1000);
    phase_tries = 0;
    val = lime_get_adc(NUM_TX+i);
    //    catch_up_read();
    printf("lime %i, rx/tx phase diff is: %i\n",i,val);
    while( val > 500 && phase_tries < 20){
      LMS_SetGaindB(limes[i],LMS_CH_TX,0,l_txgains[i]);
      usleep(1000);
      val = lime_get_adc(NUM_TX+i);
      //      catch_up_read();
      phase_tries += 1;
      printf("lime %i, rx/tx phase diff is: %i\n",i,val);
    }
    gpiobuff = (1<<6)|  (1<<1); // enable divided ADC clock output and XOR output and startup trigger and trigger for startup trigger.
    LMS_GPIODirWrite(limes[i],&gpiobuff,1);
  }

*/
  //  printf("initial 1s delay before sync\n");
  //  usleep(1000000);

  // need to be careful to only fiddle with lsb's - don't cross a boundary!

  // with 10MHz clock, rate increment of 4 gives 33 s to move a whole 1600ns period.
  // all values here assume 10MHz sample clock!

    
  catch_up_read();
  for (i=0;i<NUM_TX;i++){ // do each lime sequentially.
    // first, find direction
    gpiobuff = (1<<6)| (1<<5) | (1<<1); // enable  XOR output for rx (5), and startup trigger (1) and trigger for startup trigger (6).
    LMS_GPIODirWrite(limes[i],&gpiobuff,1);
    usleep(7000);
    
    tries = 0;
    do{
      lime_measure_phase(i,&my_phase,&direction);
      phase_tries = 0;
      while(my_phase > PHASE_TARGET && phase_tries < 100){
	lime_measure_phase(i,&my_phase,&direction);
	phase_tries += 1;
	// each measurement moves phase along. keep doing it till phase is less than 1024
	printf("measured direction, current phase is :%i\n",my_phase);
      }
      // ok, so now we know where we are, and how far to go to get there.
      
    /* // this should no longer be needed.
       if ( my_phase > 1024){
       delay = (4395-my_phase)*1650000/4095 ;
       printf("going to wait: %f seconds\n",delay/1000000.);
       set_divider(i,gtotal[i]+80); // up by 80 means 1.65 s should give
       // aim to get to ~300: So phase difference is 4395 - my_phase
       // then at rate of 80, the time is (4395 -my_phase) * 1.65s / 4095
       usleep(delay);
       set_divider(i,gtotal[i]);
       direction = 1; // assume!
       
       usleep(10000);
       
       my_phase = calc_phase(lime_get_adc(i),direction);
       printf("did long delay to get on positive slope, current phase is :%i\n",my_phase);
       }
    */
      
    // now go in smaller steps:
      phase_tries = 0;
      while (my_phase < PHASE_TARGET-100 && phase_tries < 20){
	delay = (PHASE_TARGET-my_phase)*1650000/4095/2;
	lime_do_shift_delay(i,80,delay);
	catch_up_read();
	usleep(7000);
	catch_up_read();
	my_phase = calc_phase(lime_get_adc(i),direction);
	printf("did delay, current phase is :%i\n",my_phase);
      }
      // and then smaller steps.
      phase_tries=0;
      while (my_phase < PHASE_TARGET && phase_tries < 20){
	delay = (PHASE_TARGET-my_phase)*3330000/4095/2;
	lime_do_shift_delay(i,40,delay);
	catch_up_read();
	usleep(7000);
	catch_up_read();
	my_phase = calc_phase(lime_get_adc(i),direction);
	printf("did delay, current phase is :%i\n",my_phase);
      }
      tries += 1;
      printf("final phase for lime %i is %i:\n",i,my_phase);
    } while (tries < 5 && (my_phase > PHASE_TARGET+4 || my_phase < PHASE_TARGET));
    gpiobuff = (1<<6)|  (1<<1); // enable divided ADC clock output and XOR output and startup trigger and trigger for startup trigger.
    LMS_GPIODirWrite(limes[i],&gpiobuff,1);

  }
  /*
  // make some measurements
  for (j=1;j<5;j++)
    for (i=0;i<NUM_TX;i++){
      set_divider(i,gtotal[i]+j);
    }
  
    for (j=0;j<200;j++){
    
      usleep(100000);

      for (i=0;i<NUM_TX;i++){
	adc_vals[i] = pulse_hardware_read_adc(i);// assuming lime 0 is hooked to first adc pin of due 0 (pin A8)
	printf("%i adc for lime %i\n",adc_vals[i],i);
      }
    }

    // one tick up at 10 MHz shifts ~36 ns in 3s.
    for (j=3;j>=0;j--)
    for (i=0;i<NUM_TX;i++){
      set_divider(i,gtotal[i]+j);
    }
  */    

  catch_up_read();
  
  for(i=0;i<NUM_TX;i++)
    LMS_StopStream(&rx_streams[i]);

  // done!

  
  return 1; // success!
}


// aiee. On first start - sync_lime starts the streams
// on first scan of later acqs, need to do it in start lime.
// wait - no, don't do this. Start streams in tx thread, but only if needed.
// other option would be to leave streams running at all times.
// has some advantages... but need to be reading data in at all times...

int start_lime(){
  int sval,i,j;
  // then fire up the limes for the synchronization pulses from the pb
  if (threads_launched == 0){
    printf("launching threads\n");
    sem_init(&rxwrite,0,0); // an extra for acq to know when data is ready.
    sem_init(&streams,0,0);// and one to know if there are still streams running.
    sem_init(&txdone,0,0);// tells acq that tx_thread has finished all its events.
    //    block_signals();
    for (i=0;i<NUM_TX;i++){
      num_rx_events[i] = 0;
      sem_init(&txsems[i],0,0);// acq tells tx to start looking for sync pulses.
      sem_init(&rxsems[i],0,0);  // tx tells rx threads they can start reading from the lime
      //      sem_init(&rxwrite[i],0,0); // tells rx that it can dump data into shm.
      // each rx thread does a post on the next one. acq will wait on last one.
      threadnos[i] = i;
      pthread_create(&rx_threads[i],NULL,&rx_thread_func,(void *) &threadnos[i]);
      pthread_create(&tx_threads[i],NULL,&tx_thread_func,(void *) &threadnos[i]);
      printf("start_lime: launching tx and rx threads %i\n",threadnos[i]);
    }
    //    unblock_signals();
    threads_launched = 1;
  }
  else{ // tell the threads to start the next sequence 
    //clean up leftover posts to txdone and rxwrite
    sem_getvalue(&txdone,&sval);
    if (sval > 0 )
      for (j = 0;j<sval;j++)
	sem_wait(&txdone); // these shouldn't block, so checking for interruption shouldn't be needed
    sem_getvalue(&rxwrite,&sval);
    if (sval > 0 )
      for (j = 0;j<sval;j++)
	sem_wait(&rxwrite);    
    /*
    if (prog_shm->wait_for_sync)
      for (i=0;i<NUM_TX;i++){
	LMS_StartStream(&rx_streams[i]);
	LMS_StartStream(&tx_streams[i]);
	} */
    printf("start_lime, doing sem_post to restart tx threads\n");
    
    for (i=0;i<NUM_TX;i++)
      sem_post(&txsems[i]);
  }
  //streams_running = 1;
  return 1;
}




// eventually inline this function?, but need to remove last_tx_stamp
void  send_packet(int mythreadno, int16_t *buffA){
  lms_stream_status_t status;
  int pause_count = 0,pause2=0;
  int txfifosize = 0;
  uint64_t  firststamp;
  static uint64_t last_tx_stamp[NUM_TX]={[ 0 ... NUM_TX-1] = 0};


  LMS_GetStreamStatus(&tx_streams[mythreadno],&status);
  txfifosize = status.fifoSize;
  if ((tx_metadata[mythreadno].timestamp %2) == 1){
    printf("\n\n\n\n\n\n send_packet: found an odd timestamp. This is should never happen!\n Lime will probably hang.\n\n\n\n\n");
  }
  if (last_tx_stamp[mythreadno] > 0){
    if (last_tx_stamp[mythreadno] + buffsize != tx_metadata[mythreadno].timestamp){
#ifdef NO_SEND_ZEROS
      //      printf("send packet for thread %i with non-consecutive stamps: %li and %li, expected with NO_SEND_ZEROS\n",mythreadno,last_tx_stamp[mythreadno],tx_metadata[mythreadno].timestamp);
#else
      printf("send packet for thread %i with non-consecutive stamps, should only happen on startup: %li and %li\n",mythreadno,last_tx_stamp[mythreadno],tx_metadata[mythreadno].timestamp);
#endif
    }
    
  }
  // This is all about not letting the tx fifo get completely full:
  last_tx_stamp[mythreadno] = tx_metadata[mythreadno].timestamp;
  do{
    LMS_GetStreamStatus(&tx_streams[mythreadno],&status);
    //    if (pause_count == 0) firststamp = status.timestamp;
    if (pause_count == 0) firststamp = rx_metadata[mythreadno].timestamp;
    //	      if (status.fifoFilledCount < buffsize) printf("tx: BUFFER EMPTY? at timeval: %i\n",timeval);
    if (status.underrun > 0) printf("tx: got underruns %i\n",status.underrun);
    if (status.active != 1) printf("got status not active!\n");
    //	    printf("status overruns: %i, underruns: %i\n",status.overrun,status.underrun);
    //	      printf("status %i of %i at stamp: %li\n",status.fifoFilledCount,status.fifoSize,(long) status.timestamp);
    
    if (status.fifoSize - status.fifoFilledCount < buffsize*2 ){ // don't let it ever fill.
      pause_count += 1;
      //		if (deb_count < 50) printf("offset 0 pausing\n");
      usleep(2000);
    }
  }while(status.fifoSize-status.fifoFilledCount <buffsize);
  if (pause_count > 2) {
    printf("pause count of %i at stamp %li\n",pause_count,tx_metadata[mythreadno].timestamp);
    //    printf("stamp at first pause: %li, last pause: %li\n",firststamp,status.timestamp);
    printf("stamp at first pause: %li, last pause: %li\n",firststamp,rx_metadata[mythreadno].timestamp);
    
  }
  //	      if (j == length-1) printf("releasing packet on final frame of TX event\n");
  /*
    if (deb_count < 50){
    printf("DEB_COUNT: %i sending stamp: %li\n",deb_count,tx_metadata[mythreadno].timestamp);
    fflush(stdout);
    deb_count++;
    } */
  // next we want to make sure that we don't load the fifo too early - it can make
  // aborting take too long.
#ifdef NO_SEND_ZEROS
  do{
    //    LMS_GetStreamStatus(&rx_streams[mythreadno],&status);
    if (tx_metadata[mythreadno].timestamp > rx_metadata[mythreadno].timestamp+2*txfifosize && prog_shm->stop_lime >= 0 ){
      //      if (pause2 == 0) printf("tx %i pausing to not fill fifo too early my stamp is %i, found %i\n",mythreadno,tx_metadata[mythreadno].timestamp,rx_metadata[mythreadno].timestamp);
      usleep(2000);
      pause2+=1;
    }
  }while(tx_metadata[mythreadno].timestamp>rx_metadata[mythreadno].timestamp+2*txfifosize && prog_shm->stop_lime >= 0);
  //  if (pause2 > 0) printf("thread %i done waiting to not fill fifo too early\n",mythreadno);
#endif
  if (prog_shm->stop_lime >=0 || do_tx_cleanup[mythreadno]){
    LMS_SendStream(&tx_streams[mythreadno],buffA,buffsize,&tx_metadata[mythreadno],1000);
    if (pause_count > 2) printf("done with packet after pause\n");
    tx_metadata[mythreadno].timestamp += buffsize;
  }
  else printf("thread %i send_packet - didn't because found stop_lime: %i\n",mythreadno,prog_shm->stop_lime);
}
uint64_t laststamp[NUM_TX];

void *tx_thread_func(void *threadno){
  int mythreadno;
  int cur_event,next_event,last_event;
  int i,j;
  int samplesRead;
  double temperature;
  //  int sval;
  // we look for sync pulses, generate the tx sequence, then block with a sem_wait. Need to worry about kill and cancelling...
  uint32_t offset;
  uint64_t offset_start=0;

  unsigned int jsr_stack[STACK_SIZE],loop_stack[STACK_SIZE],loop_counter[STACK_SIZE];
  int jsr_num,loop_num;
  uint64_t timeval=0,ntimeval=0; // length of the program, will use to set timestamps
  uint64_t position = 0,length;
  lms_stream_status_t status;
  int catchcount;
  int ratio=DUE_PP_CLOCK/TX_SAMPLE_RATE;
  int packet_count;
  int deb_count;
  char sending = 1;
  mythreadno =  *((int *)threadno);
  txprog_t *txprog;
  
  printf("TX thread %i starting\n",mythreadno);
  {
    struct sched_param sp;
    int result,priority;
    pthread_t this_thread;

    block_signals();
    
    // set our priority to 1 lower than acq, one higher than rx
    this_thread = pthread_self();
    priority = sched_get_priority_max(SCHED_FIFO);
    sp.sched_priority = priority/2-1; // 1 less than acq;
    result = pthread_setschedparam(this_thread, SCHED_FIFO,&sp);
    if( result!= 0) 
      perror("rx thread: init_sched");
  }

  //XXX set ending_stamp = 0 here?
  
  do{ // once started, we stay in here forever.
    // start of a new acquisition:
    // XXX move the stream start into sync_limes() and not here!
    if (streams_running == 0){
      static int start_count = 0;
      LMS_WriteParam(limes[mythreadno],LMS7_MAC,1);
      /*
      */    
      /*
      // randomize my jitter?
      LMS_WriteParam(limes[mythreadno],LMS7_CLKH_OV_CLKL_CGEN,0);
      usleep(100);
      LMS_WriteParam(limes[mythreadno],LMS7_CLKH_OV_CLKL_CGEN,2);
      */     
      /*
      LMS_WriteParam(limes[mythreadno],LMS7_HBI_OVR_TXTSP,2); // bypass interpolator (need MAC set to 1 for channel A - this does make oversampling = 1
      usleep(100);
      LMS_WriteParam(limes[mythreadno],LMS7_HBI_OVR_TXTSP,7); // bypass interpolator (need MAC set to 1 for channel A - this does make oversampling = 1
      usleep(100);
      */
      /*
      LMS_WriteParam(limes[mythreadno],LMS7_MCLK2SRC,2); // set mclk to run from txtsp
      LMS_WriteParam(limes[mythreadno],LMS7_RXRDCLK_MUX,2); // and feed the rx port
      */

      printf("thread %i starting streams\n",mythreadno);
      LMS_StartStream(&rx_streams[mythreadno]);
      LMS_StartStream(&tx_streams[mythreadno]);
      printf("thread %i done starting streams\n",mythreadno);
      streams_running = 1;
      /*
      LMS_WriteParam(limes[mythreadno],LMS7_EN_ADCCLKH_CLKGN,0);
      usleep(100);
      LMS_WriteParam(limes[mythreadno],LMS7_EN_ADCCLKH_CLKGN,1);
      */
      /*
      LMS_WriteParam(limes[mythreadno],LMS7_REV_CLKDAC_CGEN,1);
      LMS_WriteParam(limes[mythreadno],LMS7_REV_CLKDAC_CGEN,0);
      */
      /*
      LMS_WriteParam(limes[mythreadno],LMS7_HBD_DLY,(start_count/2)%4);
      printf("lime %i set HBD_DLY to %i\n",mythreadno,(start_count/2)%4);
      start_count += 1;
      */
      /*
      LMS_WriteParam(limes[mythreadno],LMS7_HBI_OVR_TXTSP,2); // bypass interpolator (need MAC set to 1 for channel A - this does make oversampling = 1
      LMS_WriteParam(limes[mythreadno],LMS7_HBI_OVR_TXTSP,7); // bypass interpolator (need MAC set to 1 for channel A - this does make oversampling = 1
      */
      /* hmm, this randomizes, but in 50ns steps, not 12.5...
      LMS_WriteParam(limes[mythreadno],LMS7_CLKH_OV_CLKL_CGEN,1);
      usleep(100);
      LMS_WriteParam(limes[mythreadno],LMS7_CLKH_OV_CLKL_CGEN,2);
      */
    }
    if (isin != store_isin) printf("\n\n\nTX: isin and store_isin are different! 1 \n");
    ending_stamp[mythreadno] = 0;  // XXX move to two other spots
    txprog = &prog_shm->txprog[prog_shm->etxprogno];
    offset = 0;

    LMS_GetChipTemperature(limes[mythreadno],0,&temperature);
    printf("tx thread %i, chip temperature is: %f\n",mythreadno,temperature);
    
    fflush(stdout);
    if (prog_shm->wait_for_sync){
#ifdef NO_SEND_ZEROS
      sending = 0;
#else
      sending = 1;
#endif
      catchcount = 0;
      do{// do some catch up...
	samplesRead = LMS_RecvStream(&rx_streams[mythreadno],rbuff[mythreadno],buffsize, &rx_metadata[mythreadno],1000);
	//      printf("catchup timestamp0: %lli\n",rx_metadata[mythreadno].timestamp);
	//      printf("catchup timestamp1: %lli\n",rx_metadata[mythreadno].timestamp);
	LMS_GetStreamStatus(&rx_streams[mythreadno],&status);
	catchcount +=1 ;
      }while (status.fifoFilledCount > 2*buffsize && prog_shm->stop_lime >= 0 );
      printf("tx thread: did %i reads to catchup.\n",catchcount);
      // do one extra read:
      samplesRead = LMS_RecvStream(&rx_streams[mythreadno],rbuff[mythreadno],buffsize, &rx_metadata[mythreadno],1000);


      laststamp[mythreadno] = rx_metadata[mythreadno].timestamp;
      printf("thread %i first stamp was: %li\n",mythreadno, laststamp[mythreadno]);
      // now  we're just looking for a timestamp that is less than the previous since it will have been reset.
      
#ifdef SYNC_LIMES
      printf("tx thread %i waiting for sync pulse\n",mythreadno);
      

      if (mythreadno == 0){
	unsigned char gpiobuff;
	printf("doing GPIO write to trigger start trigger\n");
	usleep(5000); // wait 5 ms to be sure everybody else is ready before we hit the reset buttons.
	gpiobuff = 2;  // hits the pre-trigger on GPIO1, which will trigger GPIO6
	LMS_GPIOWrite(limes[mythreadno],&gpiobuff,1);
	usleep(1);
	gpiobuff = 0; // back to idle on both.
	LMS_GPIOWrite(limes[mythreadno],&gpiobuff,1);
      }
      
      do{
	// yes before, so it doesn't get it if it has the reset value.
	laststamp[mythreadno] = rx_metadata[mythreadno].timestamp;
	samplesRead = LMS_RecvStream(&rx_streams[mythreadno],rbuff[mythreadno],buffsize, &rx_metadata[mythreadno],1000);
	//	if (rx_metadata[mythreadno].timestamp & (1LL<63) == 0)	  // found a reset
	//	  laststamp[mythreadno] = rx_metadata[mythreadno].timestamp;
	//	else printf("tx thread %i found sync event! timestamp went from: %li to %li\n",mythreadno,laststamp[mythreadno],rx_metadata[mythreadno].timestamp);
      } while (((rx_metadata[mythreadno].timestamp & (1LL<<63)) == 0) && prog_shm->stop_lime >= 0); 

      if (rx_metadata[mythreadno].timestamp & (1LL << 63)){
	timeval = (rx_metadata[mythreadno].timestamp ^ (1LL<<63)); // turn off top bit
	printf("FOUND sync event for lime %i at timestamp: %li!\n",mythreadno,timeval);
	// XXX CM HACK WTF WHY IS THIS HERE!!!
	if (timeval%2 == 0){
	  timeval += 2 ;
	  printf("Fudging start stamp by 2 on lime: %i\n",mythreadno);
	  printf("FOR REASONS I DON'T UNDERSTAND THIS SEEMS NECESSARY NOW TO PROPERLY SYNC LIME1 to LIME0!!!\n");
	}
	//	else
	//	  timeval -= 1;
	//	if (mythreadno == 0) timeval += 1;
	
	fflush(stdout);
      }
      else printf("out of wait for sync, but stamp doesn't have high bit set?\n");
      // now read till stamps don't have that leading bit set anymore:
      do{
	samplesRead = LMS_RecvStream(&rx_streams[mythreadno],rbuff[mythreadno],buffsize, &rx_metadata[mythreadno],1000);
      }while(rx_metadata[mythreadno].timestamp & (1LL<<63) && prog_shm->stop_lime >=0);
      laststamp[mythreadno] = rx_metadata[mythreadno].timestamp;
      
      printf("tx thread %i, first post-sync stamp is: %li\n",mythreadno,rx_metadata[mythreadno].timestamp);
      
      //      timeval = (0.05+46.6e-6)*TX_SAMPLE_RATE; // start of 1st event is
      //for 25 MHz sample rate:
      //      timeval += (0.0515-1.6e-6)*TX_SAMPLE_RATE; // start of 1st event is 50 ms after the sync pulse goes low. + 1.5ms for sync pulse
      // for 10 MHz sample rate:
      
      timeval += (0.07+50e-6-3.5e-6)*TX_SAMPLE_RATE; // start of 1st event is 50 ms after the sync pulse goes low. + 1.5ms for sync pulse
      if (TX_SAMPLE_RATE == 50000000) timeval += 2.8e-6*TX_SAMPLE_RATE;

      
#else // this is just fake for testing without sync line hooked up:
      printf("tx thread NOT waiting for sync pulses\n");
      timeval = rx_metadata[mythreadno].timestamp+43.2e-3*TX_SAMPLE_RATE; // go in 50ms !? 
#endif
      if (prog_shm->stop_lime != 0) printf("thread %i out of sync loop, but found stop_lime = %i\n",mythreadno,prog_shm->stop_lime);
      cur_event = 0;

      // ODD FIX here if timeval is odd, set the metadata to one less,
      // set position to 1, and put 0's in first sample in tx buffer
      if (timeval %2 == 0){
	tx_metadata[mythreadno].timestamp = timeval-2; // first packet leaves at timeval (unless its odd)
	sbuff[mythreadno][0]=0;
	sbuff[mythreadno][1]=0;
	sbuff[mythreadno][2]=0;
	sbuff[mythreadno][3]=0;
	position = 2; // position in send buffer.
      }
      else{
	printf("doing ODDFIX   1 on tx %i\n",mythreadno);
	tx_metadata[mythreadno].timestamp = timeval-3;
	sbuff[mythreadno][0]=0;
	sbuff[mythreadno][1]=0;
	sbuff[mythreadno][2]=0;
	sbuff[mythreadno][3]=0;
	sbuff[mythreadno][4]=0;
	sbuff[mythreadno][5]=0;
	position = 3;
      }
      printf("Tx: %i, first sending stamp is %li at pos: %i\n",mythreadno,tx_metadata[mythreadno].timestamp,position);
      // tell our rx thread to start reading:
      printf("TX thread %i: doing sem_post to let rx thread take over rx on chan 1\n",mythreadno);
      sem_post(&rxsems[mythreadno]);
      prog_shm->wait_for_sync = 0;
    }
    else{
      printf("tx thread %i: starting without sync, just setting cur_event\n",mythreadno);
      cur_event = 0;
    }
    

    // now we need to start generating the tx events

    // now we parse the pulse program and generate events.
    rx_reset[mythreadno] = 1; // tell rx we're at the start of a new scan.
    jsr_num = 0;
    loop_num = 0;
    for (i=0;i<STACK_SIZE;i++)
      loop_counter[i] = -1;

    // XXX protect this:
    LMS_GetStreamStatus(&tx_streams[mythreadno],&status);
    printf("Tx thread %i starting main loop last rx stamp was: %li, next event starts at: %li\n",mythreadno,status.timestamp,timeval);
    //    printf("events in program: %i\n",txprog->txevents[mythreadno]);
    deb_count = 0;

    do{
      //      printf("lime tx %i, doing event: %i, metastamp: %li\n",mythreadno,cur_event,tx_metadata[mythreadno].timestamp);
      if (isin != store_isin) printf("\n\n\nTX: isin and store_isin are different! at event: %i\n",cur_event);
      //      printf("tx thread: event: %i, opcode: %i\n",cur_event,l_txopcodes[mythreadno][cur_event]);
      if (prog_shm->etxprogno == prog_shm->txprogno){
	printf("\n\n\nTX: found txprogno and etxprogno the same!\n");
      }

      switch (txprog->txopcodes[mythreadno][cur_event] & 15){
      case CONTINUE:
      case SUBSTART:
	ntimeval = timeval + txprog->txtimes[mythreadno][cur_event]/ratio;
	next_event = cur_event + 1;
	break;
      case WAIT:
      case WAIT_MAX:
	printf("tx_thread: Got a WAIT event. Don't know what to do!\n");
	ntimeval = timeval + txprog->txtimes[mythreadno][cur_event]/ratio;
	next_event = cur_event+1;
	break;
      case LOOP: 
	ntimeval += txprog->txtimes[mythreadno][cur_event]/ratio;
	next_event = cur_event + 1;
	// if we've already started this loop, don't start it again!
	// we've started this loop if: loop_num > 0 and  loop_stack[loop_num-1] = cur_event and loop_counter[loop_num-1] > 0
	if (loop_num > 0){
	  if (loop_stack[loop_num-1] == cur_event && loop_counter[loop_num-1] > 0){
	    break;
	  }
	}
	loop_counter[loop_num] = txprog->txopinst[mythreadno][cur_event];
	loop_stack[loop_num] = cur_event;
	//	printf("loaded loop_stack level %i with event no %i and %i loops\n",loop_num,cur_event,l_txopinst[mythreadno][cur_event]);
	loop_num += 1;
	if (loop_num > STACK_SIZE -1){
	  printf("in tx thread. Too many loops nested. BAD THINGS WILL HAPPEN\n");
	}
	break;
      case END_LOOP:
	ntimeval = timeval+ txprog->txtimes[mythreadno][cur_event]/ratio;
	if (loop_num < 1) printf("Tx thread got an end_loop with no loops on stack BAD THINGS WILL HAPPEN\n");
	next_event = loop_stack[loop_num-1];
	//	printf("got an END_LOOP at event %i in thread %i, with %i loops\n",cur_event,mythreadno,loop_counter[loop_num-1]);
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
	ntimeval = timeval + txprog->txtimes[mythreadno][cur_event]/ratio;
	jsr_stack[jsr_num] = cur_event+1;
	jsr_num += 1;
	if (jsr_num > STACK_SIZE -1){
	  printf("in tx thread, Too many JSR's nested. BAD THINGS WILL HAPPEN\n");
	}
	next_event = txprog->txopinst[mythreadno][cur_event];
	//	printf("tx: jsr at event: %i to event %i, set jsr_num to %i\n",cur_event,next_event,jsr_num);
	if (next_event > txprog->txevents[mythreadno] || next_event < 0){
	  printf("got a JSR to a nonsense event\n");
	}
	break;
      case RTS:
	//	printf("tx: rts at event %i, jsr_num is %i, return to %i\n",cur_event,jsr_num,jsr_stack[jsr_num-1]);
	ntimeval = timeval + txprog->txtimes[mythreadno][cur_event]/ratio;
	if (jsr_num < 1){
	  printf("lime tx: got RTS, but nowhere to return to!\n");
	  next_event = cur_event + 1 ;
	}
	else{
	  next_event = jsr_stack[jsr_num-1];
	  jsr_num -=1;
	}
	if (next_event > txprog->txevents[mythreadno] || next_event < 0){
	  printf("got a RTS to a nonsense event\n");
	}
	//	printf("leaving jsr_num at: %i\n",jsr_num);
	break;
      case EXIT: // I guess we're done!
	ntimeval = timeval + txprog->txtimes[mythreadno][cur_event]/ratio;
	next_event = txprog->txevents[mythreadno];
	//	printf("TX got EXIT event at %i\n",cur_event);
	break;
      default:
	printf("tx thread, got an unknown opcode of %i\n",(txprog->txopcodes[mythreadno][cur_event] & 15));
	stop_acq = 1;
	prog_shm->stop_lime = 1;
	next_event = cur_event + 1;
	ntimeval = timeval + txprog->txtimes[mythreadno][cur_event]/ratio;
      } // end of switch on pb opcodes
      // ok, so now we have current event, next event, current time and next time.
      // parse the tx part of the opcode:
      if (txprog->txopcodes[mythreadno][cur_event] & 64){ // queue up an rx event.
	rx_start_stamps[mythreadno][num_rx_events[mythreadno]] = timeval;
	rx_end_stamps[mythreadno][num_rx_events[mythreadno]] = ntimeval;
	rx_phases[mythreadno][num_rx_events[mythreadno]] = txprog->txparams[mythreadno][cur_event][2]*M_PI/180.;
	//	printf("found an rx event, starts at stamp: %li. ends at:: %li\n",timeval,ntimeval);
	if (num_rx_events[mythreadno] == MAX_TX_EVENTS - 1)
	  num_rx_events[mythreadno] = 0;
	else
	  num_rx_events[mythreadno] += 1;	
      } 
      if (txprog->txopcodes[mythreadno][cur_event] & 32 ){
	offset = txprog->txopinst[mythreadno][cur_event];
	offset_start = timeval+position; // I think...
	if (txprog->txtimes[mythreadno][cur_event] != 0)
	  printf("got an offset setting without time 0\n");
	if ((txprog->txopcodes[mythreadno][cur_event] & 15) != 0)
	  printf("got an offset setting without CONTINUE\n");
      }
      else if (txprog->txopcodes[mythreadno][cur_event] & 16){
	// generate the events and stream them!
	uint32_t phase;
	int16_t i1,q1;
	length = ntimeval-timeval;
#ifdef NO_SEND_ZEROS
	if (sending == 0){
	  sending = 1;
	  printf("AT STAMP: %li, starting sending\n",timeval);
	}
#endif
	if (offset == 0){
	  i1 = (txprog->txparams[mythreadno][cur_event][0] * icos[txprog->txparams[mythreadno][cur_event][1]])/131072;
	  q1 = (txprog->txparams[mythreadno][cur_event][0] * isin[txprog->txparams[mythreadno][cur_event][1]])/131072;
	  //	  printf("TX i and q are: %i, %i, phase index was %i \n",i1,q1,txprog->txparams[mythreadno][cur_event][1]);
	  //	  printf("cos and sin are: %i %i\n",icos[txprog->txparams[mythreadno][cur_event][1]],isin[txprog->txparams[mythreadno][cur_event][1]]);
	  //	  printf("tx thread %i, event of length %i, starting in buff at position: %i\n",mythreadno,length,position);
	  if (position == 1) printf("sbuff at 0 was: %i, %i\n",sbuff[mythreadno][0],sbuff[mythreadno][1]);
	  for (j=0;j<length ;j++){
	    //icos and isin have values of  +/-32767, with indices up to 65535 (16 bits)
	    // amp is up to +/- 32767
	    // phase is a 32 bit number
	    //printf("putting in pulse for tx %i, position: %i meta stamp: %li\n",mythreadno,position,tx_metadata[mythreadno].timestamp);
	    sbuff[mythreadno][2*position] = i1;
	    sbuff[mythreadno][2*position+1] = q1;
	    position+=1;
	    if (position == buffsize){	      
	      if (prog_shm->stop_lime < 0) break;
	      send_packet(mythreadno,sbuff[mythreadno]);
	      position = 0;
	    }
	  }
	}
	else{ // there is an offset
	  phase = (tx_metadata[mythreadno].timestamp+position-offset_start)*offset; // phase is a 32 bit number
	  for (j=0;j<length ;j++){
	    sbuff[mythreadno][2*position] = txprog->txparams[mythreadno][cur_event][0] * icos[(phase + txprog->txparams[mythreadno][cur_event][1])]/131072;  
	    sbuff[mythreadno][2*position+1] = txprog->txparams[mythreadno][cur_event][0] * isin[(phase + txprog->txparams[mythreadno][cur_event][1])]/131072;
	    phase += 1;
	    position+=1;
	    if (position == buffsize){
	      if (prog_shm->stop_lime < 0) break;
	      send_packet(mythreadno,sbuff[mythreadno]);
	      position = 0;
	    }
	  }
	}
      }
      else { // no opcode - send 0's
	length = ntimeval-timeval;
	//	printf("zeros event, starting at %li should be %li, length: %li\n",tx_metadata[mythreadno].timestamp + position,timeval,length);
	//	if (next_event == txprog->txevents[mythreadno]) length = 1; // don't do all of last event it its 0's.
	  // make sure to do at least one though.
#ifdef NO_SEND_ZEROS
	if (sending == 1) {// if its already zero, there's no need to do anything but advance the timestamp.
	  if (length  > tx_streams[mythreadno].fifoSize + 2*buffsize) { // length of zero event is more than 2 buffers more than the fifo length

	    printf("\n\n\nTx thread %i about to start end_sending event at position: %li\n",mythreadno,position);
	    fflush(stdout);
	    for (j=position;j< buffsize;j++){
	      sbuff[mythreadno][2*j] = 0;
	      sbuff[mythreadno][2*j+1] = 0;
	    }
	    send_packet(mythreadno,sbuff[mythreadno]);
	    position = 0;
	    printf("Tx thread %i in NO_SEND_ZEROS, sending 0\n",mythreadno);
	    for (j=0;j<tx_streams[mythreadno].fifoSize/buffsize+1;j++){ // send a full fifo of zeros.
	      send_packet(mythreadno,buff0);
	    }
	    printf("Tx thread %i blocking till send buffer empty\n",mythreadno);
	    fflush(stdout);
	    do{
	      usleep(2000);
	      LMS_GetStreamStatus(&tx_streams[mythreadno],&status);
	    }while (status.fifoFilledCount > 0); // let this continue even if we get stop_lime. Its the quickest way to 0's anyway.
	    printf("Tx thread %i done blocking till send buffer empty\n",mythreadno);
	    fflush(stdout);
	    sending = 0;
	    //	    printf("tx thread %i AT STAMP: %li, ending  sending, till: %li\n",mythreadno,timeval,ntimeval);
	      // ODD FIX in here if ntimeval is odd, set it to one less, then
	      // set position to 1 and put 0's in first sample in tx buffer
	      // TODO should position get set to 0 here? Or is it already?
	    if (ntimeval %2 == 0){
	      tx_metadata[mythreadno].timestamp = ntimeval-2; // first packet leaves at timeval (unless its odd)
	      sbuff[mythreadno][0]=0;
	      sbuff[mythreadno][1]=0;
	      sbuff[mythreadno][2]=0;
	      sbuff[mythreadno][3]=0;
	      position = 2; // position in send buffer.
	    }
	    else{
	      printf("doing ODDFIX 2 on tx %i\n",mythreadno);
	      tx_metadata[mythreadno].timestamp = ntimeval-3;
	      sbuff[mythreadno][0]=0;
	      sbuff[mythreadno][1]=0;
	      sbuff[mythreadno][2]=0;
	      sbuff[mythreadno][3]=0;
	      sbuff[mythreadno][4]=0;
	      sbuff[mythreadno][5]=0;
	      position = 3;
	    }
	  }
	  else{ // do ordinary 0's event:
	    for (j=0;j<length  ; j++){
	      while(position == 0 && length-j > buffsize && prog_shm->stop_lime >= 0 ){// there are length-(j-1) points left to go. Want make sure we leave at least one more for sbuff.
		/*		if (deb_count < 50){
		  printf("DEB_COUNT: all 0's event %i sending stamp: %li\n",deb_count,tx_metadata[mythreadno].timestamp);
		  fflush(stdout);
		  deb_count++;
		  } */
		send_packet(mythreadno,buff0);
		j += buffsize; 
	      } 
	      sbuff[mythreadno][2*position] = 0;
	      sbuff[mythreadno][2*position+1] = 0;
	      position+=1;
	      if (position == buffsize){
		if (prog_shm->stop_lime < 0) break;
		send_packet(mythreadno,sbuff[mythreadno]);
		position = 0;
	      }
	    }
	  }
	}
	else{ // sending == 0 - we weren't sending...
	  //	  printf("had sending of 0, just advancing metadata timestamp from %li to %li\n",timeval,ntimeval);
	  // ODD FIX in here if ntimeval is odd, set it to one less, then
	  // set position to 1 and put 0's in first sample in tx buffer
	  if (ntimeval %2 == 0 ){
	    tx_metadata[mythreadno].timestamp = ntimeval-2; // first packet leaves at timeval (unless its odd)
	    sbuff[mythreadno][0]=0;
	    sbuff[mythreadno][1]=0;
	    sbuff[mythreadno][2]=0;
	    sbuff[mythreadno][3]=0;
	    position = 2; // position in send buffer.
	  }
	  else{
	    printf("doing ODDFIX 3 on tx %i\n", mythreadno);
	    tx_metadata[mythreadno].timestamp = ntimeval-3;
	    sbuff[mythreadno][0]=0;
	    sbuff[mythreadno][1]=0;
	    sbuff[mythreadno][2]=0;
	    sbuff[mythreadno][3]=0;
	    sbuff[mythreadno][4]=0;
	    sbuff[mythreadno][5]=0;
	    position = 3;
	  }

	}
#else
	for (j=0;j<length ; j++){
	  while(position == 0 && length-j > buffsize && prog_shm->stop_lime >= 0 ){// there are length-(j-1) points left to go. Want make sure we leave at least one more for sbuff.
	    send_packet(mythreadno,buff0);
	    j += buffsize; 
	  } 
	  sbuff[mythreadno][2*position] = 0;
	  sbuff[mythreadno][2*position+1] = 0;
	  position+=1;
	  if (position == buffsize){
	    if (prog_shm->stop_lime < 0) break;
	    send_packet(mythreadno,sbuff[mythreadno]);
	    position = 0;
	  }
	}
#endif
      } // no tx
      last_event = cur_event;
      cur_event = next_event;
      timeval = ntimeval;
      //      printf("tx end of loop cur_event is %i, opcode was: %i\n",last_event,txprog->txopcodes[mythreadno][last_event]);
    } while ((txprog->txopcodes[mythreadno][last_event] & 15) != EXIT && prog_shm->stop_lime >= 0 );

        
    // end of tx program loop.

    // if we dropped out on error, send zero packets immediately
    if (prog_shm->stop_lime < 0 && sending == 1){
	for (j=0;j<tx_streams[mythreadno].fifoSize/buffsize+1;j++){
	  send_packet(mythreadno,buff0);
	}
    }
    printf("Tx thread %i out of main loop, num_rx_events is: %i\n",mythreadno,num_rx_events[mythreadno]);
    fflush(stdout);
    sem_post(&txdone);
    
    // all events finished.
    // tell the RX thread when the sequence ends:
    ending_stamp[mythreadno] = timeval;
    // after all tx events have been generated,
  
    printf("tx thread %i, at end.\n",mythreadno);

    sem_wait(&txsems[mythreadno]); // wait here for next scan to begin. or for rx to tell us to cleanup.

    // XXX set ending_stamp = 0 here?
    
    if (do_tx_cleanup[mythreadno] == 1){
      printf("tx: thread %i got do_tx_cleanup\n",mythreadno);
      fflush(stdout);
      // let buffer empty before we send 0's:
      packet_count = 0;
      do{
	usleep(2000);
	packet_count += 1;
	LMS_GetStreamStatus(&tx_streams[mythreadno],&status);
      }while (status.fifoFilledCount > 0 && packet_count < 1000);
      if (packet_count == 1000)  printf("tx thread %i unclean exit on first block\n",mythreadno);
      else usleep(20000);
      tx_metadata[mythreadno].waitForTimestamp = false;
      for (j=0;j<tx_streams[mythreadno].fifoSize/buffsize+1;j++){
	send_packet(mythreadno,buff0);
      }
      tx_metadata[mythreadno].waitForTimestamp = true;
      // block till buffer is empty:
      printf("tx thread %i, waiting till send buffer is empty\n",mythreadno);
      packet_count = 0;
      do{
	usleep(2000);
	packet_count += 1;
	LMS_GetStreamStatus(&tx_streams[mythreadno],&status);
	//	printf("while waiting, filledCount is: %li\n",status.fifoFilledCount);
      }while (status.fifoFilledCount > 0 && packet_count < 1000);
      if (packet_count == 1000) printf("UNCLEAN EXIT FROM TXCLEANUP. fifoFilled count never went to 0\n");
    
      fflush(stdout);
      printf("thread %i done tx_cleanup\n",mythreadno);
      //      usleep(2000000);
      do_tx_cleanup[mythreadno] = 0;
      // tell acq the streams are stopped
      sem_post(&streams);
      fflush(stdout);
      usleep(5000); // wait a bit before we tell rx.
      sem_wait(&txsems[mythreadno]); // wait here for next scan to begin.
    }
  
    
    printf("tx thread %i woken\n",mythreadno);
    fflush(stdout);
    if (prog_shm->stop_lime == THREAD_EXIT) {
      printf("txthread - woken to find THREAD_EXIT");
      return NULL;
    }
    // we only get woken up to start the next scan.
    // it is possible we're woken up but we're too late...
    
  } while (1);//while (prog_shm->stop_lime == NOT_DONE && prog_shm->lime_error == 0);
  return NULL;
}
#define PROCESS_BUFF  end_samp = rx_end_stamps[mythreadno][my_event] - rx_metadata[mythreadno].timestamp;\
  if (end_samp > buffsize) end_samp = buffsize;\
  for( i=start_samp;i<end_samp;i++){\
    if (my_point+my_start_in_buff < l_npts){\
      l_rxbuffer[2*(my_point+my_start_in_buff)] += rbuff[mythreadno][2*i];\
      l_rxbuffer[2*(my_point+my_start_in_buff)+1] += rbuff[mythreadno][2*i+1];\
      samp_in_point += 1;\
      if (samp_in_point == samples_in_point){\
      /* fix phase */ \
	ival = l_rxbuffer[2*(my_point+my_start_in_buff)];\
	qval = l_rxbuffer[2*(my_point+my_start_in_buff)+1];\
	l_rxbuffer[2*(my_point+my_start_in_buff)] = ival * pcos + qval * psin;\
	l_rxbuffer[2*(my_point+my_start_in_buff)+1] = qval * pcos - ival * psin;\
	samp_in_point = 0;		  \
	my_point += 1;\
	if (my_point == l_rxpoints[mythreadno]){\
	  /* copy our data into l_buffer */		\
	  for (k=0;k<l_rxpoints[mythreadno];k++){\
	    l_buffer[2*(k+my_start_in_buff)] = l_rxbuffer[2*(k+my_start_in_buff)];\
	    l_buffer[2*(k+my_start_in_buff)+1] = -l_rxbuffer[2*(k+my_start_in_buff)+1];\
	    l_rxbuffer[2*(k+my_start_in_buff)] = 0;\
	    l_rxbuffer[2*(k+my_start_in_buff)+1] = 0;\
	  }\
	  /*	      printf("rx: posting rxwrite\n");*/	\
	  sem_post(&rxwrite);\
	  my_point = 0;\
	}\
      }\
    }\
     } /* we've finished either the rx event or the buffer. */ 		\
  if (rx_end_stamps[mythreadno][my_event] > rx_metadata[mythreadno].timestamp+buffsize){\
    state = 1; /* mid receive */					\
    done_with_buff = 1;\
  }\
  else if (rx_end_stamps[mythreadno][my_event] <= rx_metadata[mythreadno].timestamp+buffsize){\
    state = 0;\
    my_event += 1; /* we finished the event. There might be another event within the same buffer though! */ \
    if (my_event == MAX_TX_EVENTS) my_event = 0;\
  }

void *rx_thread_func(void *threadno){
  int mythreadno;
  int samplesRead,eo,sval;
  int my_point=0, samp_in_point=0, samples_in_point;
  int state=0,my_start_in_buff;
  uint64_t start_samp,end_samp;
  unsigned int my_event=0;
  bool done_with_buff = 0;
  int i,k;
  float pcos=0,psin=0,ival,qval;

  block_signals();
  mythreadno =  *((int *)threadno);
  sem_getvalue(&rxsems[mythreadno],&sval); // starts with 1.
  printf("RX thread %i, start up, sem value is %i\n",mythreadno,sval);
  {
    struct sched_param sp;
    int result,priority;
    pthread_t this_thread;
     
 
    // set our priority to 2 lower than acq, one lower than tx
    this_thread = pthread_self();
    priority = sched_get_priority_max(SCHED_FIFO);
    sp.sched_priority = priority/2-2; // 2 less than acq;
    result = pthread_setschedparam(this_thread, SCHED_FIFO,&sp);
    if( result!= 0) 
      perror("rx thread: init_sched");
  
  }
  
  my_event = 0; // which event in the rx_events queue we're working on.
  
  while(1){
    // a new acquisition starts here.
    
    // should check that all is well in here. For now, just read data.
    do{
      printf("rx thread %i going to sleep\n",mythreadno);
      eo = sem_wait(&rxsems[mythreadno]);
      printf("rx thread awake\n");
      if (eo == -1 && errno == EINTR) printf("rx thread, sem woken by signal, shouldn't be!\n");
    }while (eo == -1 && errno == EINTR && prog_shm->stop_lime >= 0);// woken by a signal
    printf("rx thread %i woken and running\n",mythreadno);

    // sigh is this ok? If tx was running it could cause problems... But that would mean
    // that acq set THREAD_EXIT while we were running. I suppose it could happen if
    // a clean kill wasn't finished?
    if (prog_shm->stop_lime == THREAD_EXIT){
      if (streams_running){ // hmm. They shouldn't be, unless tx was running.
	LMS_StopStream(&rx_streams[mythreadno]);
	LMS_StopStream(&tx_streams[mythreadno]);
	streams_running = 0;
      }
      printf("rx thread %i exited after finding THREAD_EXIT\n",mythreadno);
      return NULL;
    }


    // figure out where we put our data in the data buffer.
    my_start_in_buff = 0;
    for(i=0;i<mythreadno;i++)
      my_start_in_buff += l_rxpoints[i];
    printf("thread: %i, starting at point %i\n",mythreadno,my_start_in_buff);

    
    if (l_rx_sw[mythreadno] <= 0) // hopefully means it just wasn't set?
      samples_in_point = 1;
    else
      samples_in_point = TX_SAMPLE_RATE/l_rx_sw[mythreadno];
    if (samples_in_point == 0) samples_in_point = 1;
    // FIXME - try to match SW
    printf("RX: for sw of %f, using %i points per sample\n",l_rx_sw[mythreadno],samples_in_point);
    // should skip this if we are woken with THREAD_EXIT!
    do{
      if (rx_reset[mythreadno]){ // tx tells us we're starting a new acquisition.
	my_point = 0; // how many points we've filled in our buffer
	samp_in_point = 0; // for averaging of samples into points.
	state = 0; // 0 means nothing happening. 1 means we're acquiring.
	rx_reset[mythreadno] = 0;
	memset(l_rxbuffer,0,2*sizeof(int64_t)*l_npts); // set all data to 0. This is a bit wasteful as it gets done on the fly too.
      }
      samplesRead = LMS_RecvStream(&rx_streams[mythreadno],rbuff[mythreadno],buffsize, &rx_metadata[mythreadno],100);
      if (laststamp[mythreadno] > rx_metadata[mythreadno].timestamp)
	printf("in normal reading found a stamp went backwards from %li to %li\n",laststamp[mythreadno],rx_metadata[mythreadno].timestamp);
      //      printf("rx, got stamp: %li\n",rx_metadata[mythreadno].timestamp);
      // in here we'll need to figure out when to keep data, and filter it etc.
      // also need to look for flag to quit.
      laststamp[mythreadno] = rx_metadata[mythreadno].timestamp;
      done_with_buff = 0;
      
      while (my_event != num_rx_events[mythreadno] && done_with_buff == 0  ){ // there's an event to work on!
	if (state == 0){ // hopefully we haven't missed the start. check for it
	  //	  printf("in rx, found an event start: %li, end: %li\n",rx_start_stamps[mythreadno][my_event],rx_end_stamps[mythreadno][my_event]);
	  if (rx_metadata[mythreadno].timestamp > rx_start_stamps[mythreadno][my_event]){
	    // EEEK, this could mess up lots of stuff...
	    printf("MISSED START OF RX EVENT?? my_event is %i, and num_rx_events is %i\n",my_event,num_rx_events[mythreadno]);
	    printf("got timestamp: %li, start was: %li\n",rx_metadata[mythreadno].timestamp,rx_start_stamps[mythreadno][my_event]);
	    // nope if we missed it, we quit:
	    stop_acq = 1;
	    prog_shm->stop_lime = ERROR_DONE;
	    
	    my_point += (rx_end_stamps[mythreadno][my_event]-rx_start_stamps[mythreadno][my_event])/samples_in_point;
	    my_event += 1; // skip this event and move on. That's a bit of a disaster for signal averaging. Try to fix it by just giving 0's:
	    if (my_event == MAX_TX_EVENTS) my_event = 0;
	  }
	  else if (rx_metadata[mythreadno].timestamp + buffsize > rx_start_stamps[mythreadno][my_event] ){ // our rx event starts in the current data
	    //	    printf("rx data in current frame. start stamp is %li, current stamp is %li\n",rx_start_stamps[mythreadno][my_event],rx_metadata[mythreadno].timestamp);
	    //	    printf("rx using phase: %f\n",rx_phases[mythreadno][my_event]);
	    pcos = cos(rx_phases[mythreadno][my_event]);
	    psin = sin(rx_phases[mythreadno][my_event]);
	    //	    printf("RX THREAD: %i, using phase: %f, vals: %f %f\n",mythreadno,rx_phases[mythreadno][my_event],pcos,psin);
	    /* Don't want this. Want receive points to be able to straddle rx events - eg for noisy stuff.
	       if (samp_in_point != 0){
	       samp_in_point = 0; // in case we left a point unfinished somewhere.
	       my_point += 1;
	       } */
	    start_samp = rx_start_stamps[mythreadno][my_event]-rx_metadata[mythreadno].timestamp; // first sample
	    PROCESS_BUFF;
	  }// had some points to capture. 
	  else done_with_buff = 1;// need to wait for our data.
	} // end of state = 0
	else if (state == 1){// we're mid capture.
	  start_samp = 0;
	  PROCESS_BUFF;
	}// done state 1 
      }// done while loop dealing with read.
      
      // go back and read again.
    } while(((rx_metadata[mythreadno].timestamp < ending_stamp[mythreadno]) ||
            (ending_stamp[mythreadno] == 0)) && (prog_shm->stop_lime >= 0) );
    
    //XXX move set do_tx_cleanup to here    
    
    // if we're here, either we got to ending_stamp, or stop_lime was set to cancel.
    // either way, we're done.
    printf("RX: thread %i out of rx loop with my_point: %i and rxpoints: %li\n",mythreadno,my_point,l_rxpoints[mythreadno]);
    printf("RX: thread %i finished with samp_in_point: %i\n",mythreadno,samp_in_point);
    fflush(stdout);

    // if we errored out of tx or rx and we have points we were supposed to generate, wake up acq.
    if (stop_acq && l_rxpoints[mythreadno] > 0)
      sem_post(&rxwrite);
    
    if (rx_metadata[mythreadno].timestamp >= ending_stamp[mythreadno]){
      // tell the tx thread:
      prog_shm->stop_lime = ERROR_DONE;
      // wake up our tx - nope, don't bother, tx is already asleep. 
      //      sem_post(&txsems[mythreadno]);
    }
    ending_stamp[mythreadno] = 0; //XXX unnecessary?

    my_event = num_rx_events[mythreadno];
    printf("rx, thread %i syncing my_event and num_rx_events at: %i\n",mythreadno,my_event);
    do_tx_cleanup[mythreadno] = 1; // tell tx to spit out some zeros and shutdown. //XXX move up a little
    sem_post(&txsems[mythreadno]);

    // wait till tx is finished cleaning up:
    do{
      samplesRead = LMS_RecvStream(&rx_streams[mythreadno],rbuff[mythreadno],buffsize, &rx_metadata[mythreadno],100);
    }while (do_tx_cleanup[mythreadno]);

    /* 
       XXX there's a potential race here.
       if acq restarts tx threads at same time we hit ending stamp
       rx could think we're done, but tx might not.
       Still need to protect the tx thread from this.


       I think the easiest way to protect against this race is:
       before tx thread goes to sleep, it lowers its priority below
       that of the rx thread. Then when it wakes up, it does a sched_yield
       to make sure that rx thread can run if it wants to, then
       sets ending_stamp to 0 and resets its priority.

       Should do it!
    */
    
    printf("thread %i stopping rx streams\n",mythreadno);
    LMS_StopStream(&rx_streams[mythreadno]);
    usleep(50000); // wait a bit before we shut down our streams
    // it seems we want to shut off rx stream first to avoid "L" messages.
    // should guarantee that somehow. But need to leave rx running
    // till tx buffer is empty.
    
    LMS_StopStream(&tx_streams[mythreadno]);
    streams_running = 0;
    printf("thread %i stopped tx streams\n",mythreadno);
    
    printf("RX %i: streams stopped.\n",mythreadno);
    fflush(stdout);
    // tell acq the streams are stopped:
    usleep(10000);// wait a few ms after stream stop
    sem_post(&streams);
	    
  }
}
  


int wait_for_lime_data(){
  int eo,i,count=0;
  //    printf("wait for lime_data: about to wait for last rx thread\n");
  for(i=0;i<NUM_TX;i++)
    count += (l_rxpoints[i] > 0);

  // we wait for as many threads as have data to collect.
  // there is a disaster here though if we miss some data - we'll never wake up.
  // should error out of rx_thread_func if that happens TODO
  for(i=0;i<count;i++){
    do{
      eo = sem_wait(&rxwrite);
      if (eo == -1 && errno == EINTR)
	printf("wait for lime, woken by a signal. Going back to sleep\n");
    } while (eo == -1 && errno == EINTR && prog_shm->stop_lime >= 0);
    if (prog_shm->stop_lime < 0) return 0;
  }
  //  printf("wait for lime_data: done waiting for last rx thread\n");

  // if we get here and stop_acq is 1, tx or rx errored out.
  if (stop_acq) return -1;
  else return 0;
}

void join_limes(){
  int i;
  if (threads_launched == 0 ) return;
  
  printf("joining_limes, waiting for threads to exit\n");
  prog_shm->stop_lime = THREAD_EXIT; // will shut down lime threads

  
  // wake up anybody sleeping:
  for (i=0;i< NUM_TX;i++){
    sem_post(&txsems[i]);
    sem_post(&rxsems[i]);
    //    sem_post(&trxsems[i]);
  }
  
  for (i=0;i< NUM_TX;i++)
    pthread_join(tx_threads[i], NULL);
  for (i=0;i< NUM_TX;i++)
    pthread_join(rx_threads[i], NULL);
  for (i=0;i< NUM_TX;i++){

    //LMS_StopStream(&rx_streams[i]); // whoa? what are these doing here? XXX
    //  LMS_StopStream(&tx_streams[i]);
    sem_destroy(&txsems[i]);// starts off at 0. we do a post here to tell tx thread to get going
    sem_destroy(&rxsems[i]); 
    //    sem_destroy(&trxsems[i]);
    //    sem_destroy(&rxwrite); // this doesn't seem right! should be destroy &streams, but after loop? XXX
  }
  sem_destroy(&rxwrite);
  sem_destroy(&streams);

  threads_launched = 0;
  printf("join_limes, threads done\n");
}
       	
