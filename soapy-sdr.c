#include "h_config-duesdr.h"

#include <quadmath.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "param_utils.h"
#include "due-pp-lib.h"
#include "shm_prog-duesdr.h"
#include <semaphore.h>
#include <math.h>
#include "acq-duesdr.h"
#include <pthread.h>
#include "duepp.h"
#include <errno.h>
#include <signal.h>
#include <unistd.h>
extern struct prog_shm_t*  prog_shm;        //These are shared memory structures and must be initialized
#include "soapy-sdr.h"
#include "pulse_hardware-duepp.h"

#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <complex.h>

uint64_t start_offset=0; // how many RX sample stamps are there between sync event and t=0.
//#define NOHARDWARE

//#define NOSYNCHARDWARE // remove when sync hardware is in place!

uint64_t diff_nco[NUM_RX]; 
uint64_t diff_ftw[NUM_RX];// these hold ftws for difference between transmit and receive frequencies.

// XXX TODO stop_acq vs stop_rx ???

/* order of things:

   From lime, used to be:
   -----
   open_sdr

   for each experiment:
   acq: pulse_hardware_start
      acq: pulse_hardware_send [startup prog]
      prep_lime - set gain, freq, etc.
      sync_rxtx - during initial program to sync with tx and find phase.
      acq: pulse_hardware_start
      start_sdr
      acq: pulse_hardware_wait - wait for last event from due.
      acq: pulse_hardware_send [real program] - reload due
      acq: pulse_hardware_start - restart due
  wait_for_lime_data
  accumulate_data
  pulse_harware_wait

      
  if not first time:
      acq: pulse_hardware_send [real program]
      acq: pulse_hardware_start
      prep_lime
      start_lime
  wait_for_lime_data
  accumulate_data
  pulse_harware_wait
 
But there, the sync function started streaming then stopped. Will change that -
need to stream continuously after starting.

------------------
New for SOAPY:

   for each experiment:
   acq: pulse_hardware_start
      acq: pulse_hardware_send [startup prog]
      prep_sdr - set gain, freq, etc - set up for init scan
      start_sdr_threads - during initial program to sync with tx and find phase, will start threads and streaming.
                  threads will need to do the work to find start?
      acq: pulse_hardware_start - to start the startup prog.

      acq: pulse_hardware_wait - wait for last event from due.
      // check that sync worked?
      
      
      prep_sdr - set up for each real scan.
      acq: pulse_hardware_send [real program] - reload due
      acq: pulse_hardware_start - restart due
      start_sdr - tells threads to look for new data...

  wait_for_sdr_data
  accumulate_data
  pulse_harware_wait

      
  if not first time:
      acq: pulse_hardware_send [real program]
      acq: pulse_hardware_start
      prep_sdr
      start_sdr
  wait_for_sdr_data
  accumulate_data
  pulse_harware_wait

*/



void *rx_thread_func(void *threadno);

SoapySDRDevice *sdr[NUM_RX];
SoapySDRStream *rxStream[NUM_RX];


float *fsin,*fcos;
int buffsize = 1024; // complex samples per transfer
volatile char streams_running = 0;
int num_sdrs;


volatile int l_rxgains[NUM_RX];
volatile int last_rxgains[NUM_RX];

volatile int l_sdr_sync_event[NUM_RX];
volatile uint64_t l_rxpoints[NUM_RX];
volatile double l_txfreqs[NUM_RX];

float l_rx_sw[NUM_RX];
volatile int stop_acq=0;
volatile uint64_t ending_stamp=0; // how many stamps to read from device after t=0 (start of first real program).
// gets added to by generate_rx_events each time.

float *l_rxbuffer = NULL; // this is where we store our received data on the fly.
int64_t *l_buffer; // the data gets copied here when we're done with it.
int  l_npts;


// these should be mutex protected? These are a ring buffer. num_rx_events is the write pointer.
uint64_t *rx_start_stamps[NUM_RX];
uint64_t *rx_end_stamps[NUM_RX];
float *rx_event_phase[NUM_RX];
volatile unsigned int num_rx_events[NUM_RX];


/* Semaphores:
 
 rxwrite is used by the rx threads to tell acq that all threads that have data to collect
 are finished collecting it for the current scan. Based on number of points.

*/

volatile bool threads_launched = 0;
pthread_t rx_threads[NUM_RX];
sem_t rxwrite;
int threadnos[NUM_RX];
int nsdr=0; // how many sdr's we find.
int sdrs_opened=0; // how many we actually used

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



int open_sdr(){

  
  // heavily borrowed from soapy example code.
  size_t length;
  int i ;

  // the amount of time from the end of the sync pulse till t=0 is set here:
  start_offset = (0.5+1e-6)*RX_SAMPLE_RATE;
  
  //enumerate devices
  SoapySDRKwargs *results = SoapySDRDevice_enumerate(NULL, &length);
  if (length == 0){
    printf("No Soapy devices found\n");
    return 0;
  }
  
  // in case Soapy starts some threads:
  // we want them 
  // to ignore SIGUSR1.
  block_signals();

  for (i = 0; i < length; i++)
    {
      printf("Found device #%d: ", (int)i);
      for (size_t j = 0; j < results[i].size; j++)
        {
	  printf("%s=%s, ", results[i].keys[j], results[i].vals[j]);
        }
      printf("\n");
    }
  SoapySDRKwargsList_clear(results, length);

  //create device instance
  //args can be user defined or from the enumeration result
  // serial numbers XXX SOAPY TODO


  SoapySDRKwargs args = {};
#ifdef AIRSPY
  SoapySDRKwargs_set(&args, "driver", "airspy");
#endif
#ifdef RX888
  SoapySDRKwargs_set(&args, "driver", "SDDC");
#endif
  num_sdrs = 0;
  if (length > NUM_RX){
    printf("We seem to have found more SDR devices than expected.\n");
    printf("Found %i, expected %i, only using %i\n",(int)length,NUM_RX,NUM_RX);
    length = NUM_RX;
  }
  if (length < NUM_RX){
    printf("We seem to have found fewer SDR devices than expected.\n");
    printf("Found: %i, expected %i. Will not proceed\n",(int)length, NUM_RX);
  }

  for (size_t i=0;i<length;i++){
    sdr[num_sdrs] = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);
   
    if (sdr[num_sdrs] == NULL){
      printf("SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
      continue;
    }

  // XXX SOAPY TODO: for RX888 need to set antenna - probably in prep_sdr, where we know the frequency.
  // for now, just do airspy...
    
    if (SoapySDRDevice_setSampleRate(sdr[num_sdrs], SOAPY_SDR_RX, 0, RX_SAMPLE_RATE) !=0){
      printf("Set sample rate fail: %s\n", SoapySDRDevice_lastError());
      SoapySDRDevice_unmake(sdr[num_sdrs]);
      continue;
    }
    
    rxStream[num_sdrs] = SoapySDRDevice_setupStream(sdr[num_sdrs], SOAPY_SDR_RX, SOAPY_SDR_CF32, NULL, 0, NULL);
    if (rxStream[num_sdrs] == NULL){
      printf("setupstream fail: %s\n", SoapySDRDevice_lastError());
      SoapySDRDevice_unmake(sdr[num_sdrs]);
    }
    num_sdrs += 1;
  }
  
  // end loop over devices here?
  
    
  unblock_signals();

  if (num_sdrs < NUM_RX){
    for (i=0;i<num_sdrs;i++){
      SoapySDRDevice_closeStream(sdr[i], rxStream[i]);
      SoapySDRDevice_unmake(sdr[i]);
    }
    return 0;
  }
  
  
  for (i=0;i<NUM_RX;i++){
    rx_start_stamps[i] = (uint64_t *) malloc(sizeof(uint64_t) *MAX_RX_EVENTS);
    rx_end_stamps[i] = (uint64_t *) malloc(sizeof(uint64_t) *MAX_RX_EVENTS);
    rx_event_phase[i] = (float *) malloc(sizeof(float) *MAX_RX_EVENTS);
  }
  
  // build sin/cos table for frequency offsets
  fcos = malloc(sizeof(*fcos) * TRIGLEN);
  fsin = malloc(sizeof(*fsin) * TRIGLEN);
  for (i=0;i<TRIGLEN;i++){
    fcos[i] = cos(2*M_PI*i/TRIGLEN);
    fsin[i] = sin(2*M_PI*i/TRIGLEN);
  }

  printf("open sdrs opened %i\n",num_sdrs);

  return num_sdrs;
}




void deinit_sdrs(){
  int i;
  for (i=0;i<NUM_RX;i++){
  }
  
  printf("done deiniting\n");
}
void close_sdrs(){
  int i;
  for (i=0;i<NUM_RX;i++){
    if (sdr[i] != NULL){
      SoapySDRDevice_closeStream(sdr[i], rxStream[i]);
      SoapySDRDevice_unmake(sdr[i]);
      free(rx_start_stamps[i]);
      free(rx_end_stamps[i]);
      free(rx_event_phase[i]);
    }
  }

  free(fsin);
  free(fcos);
  if (l_rxbuffer != NULL){
    free(l_rxbuffer);
    l_rxbuffer = NULL;
  }

}

#define THRESH (0.09) // threshold for start pulse detection.
#define SYNC_GAIN 25 // XX TODO what should this be?
// these are good for airspy with 1Vpp less 30dB.

int prep_sdr(int npts, int64_t *buffer, int first){

  // XXX TODO SOAPY - set up filter method - boxcar vs FFT?
  
  /* set up gain, freq, antenna.
     get variables ready so rx thread can do its work.

     on first go, we set it all. For subsequent, the only hardware setting that might change is gain.
   */
  int j;
  //  static double last_freq[NUM_RX];
  // if its the first time in an acquisition: - set gain to known value (should be configured in .h file...)
  // if its not the first time, then use selected gain value.
 

  for (j=0;j<NUM_RX;j++){
    l_rxgains[j] = prog_shm->rx_gains[j];
  }
  
  if (first){
    first = 0;
    // copy bits from the pulse program into module local variables:
    for(j=0;j<NUM_RX;j++){
      l_rxpoints[j] = prog_shm->rxpoints[j];
      l_rx_sw[j] = prog_shm->rx_sw[j];
      l_txfreqs[j] = prog_shm->txfreqs[j]; // frequency as requested from UI.
    }

    // XXX TODO, if RX888, set antenna!

    // set freq:
    // fun stuff. want to store frequency as a 64 bit integer, which is the phase increment
    // based on our sample clock. For now we're just doing airspy, 20MHz sample clock.

    // so we need to know the exact tx freq, the exact tuning frequency, and then the offset.
    // tx freq with ad9854 - get_freq_words in pulse-pb.c
    // tuning word is freq*2^48 /clkt, clkt is AD9854 internal clock - 300MHz probably.

    
    // the airspy itself will go up 5MHz from what we ask for, but that should get taken out.

    // we need to know the difference frequency...
    for (j=0;j<NUM_RX;j++){
      // first, what is the exact tx freq: from pulse-pb get_freq_words:
      double clkt = 30000000; 
      uint64_t ncot;

      // what frequency will the airspy (or rx888 use?)

      ncot = rint(281474976710656ULL*l_txfreqs[j]*1e6/clkt); // ok, that's the transmitter freq.

      /////////
      uint32_t xtal_freq = 25000000; // 25 for airspy, 27 for rx888
      const uint32_t vco_min = 1770000000;
      const uint32_t vco_max = 3900000000;
      uint32_t pll_ref = (xtal_freq >> 1);
      uint32_t pll_ref_2x = xtal_freq;
      uint32_t vco_exact, vco_frac, con_frac, div_num, n_sdm;
      uint16_t sdm=0;
      //      uint8_t ni,si;
      uint32_t nint;
      __float128 actual_freq, actual_tx_freq;

      for (div_num = 0; div_num < 5; div_num++){
	vco_exact = l_txfreqs[j]*1e6 * (1 << (div_num + 1));
	if (vco_exact >= vco_min && vco_exact <= vco_max)
	  break;
      }
      vco_exact = l_txfreqs[j]*1e6 * (1<< (div_num + 1));
      nint = (uint8_t) ((vco_exact + (pll_ref >> 16)) / pll_ref_2x);
      vco_frac = vco_exact - pll_ref_2x * nint;
      nint -= 13;
      // ni = (nint >> 2);
      // si = nint - (ni<<2);
      
      if (vco_frac != 0){
	vco_frac += pll_ref >> 16;
	sdm = 0;
	for(n_sdm = 0; n_sdm < 16; n_sdm++)
	  {
	    con_frac = pll_ref >> n_sdm;
	    if (vco_frac >= con_frac)
	      {
		sdm |= (uint16_t) (0x8000 >> n_sdm);
		vco_frac -= con_frac;
		if (vco_frac == 0)
		  break;
	      }
	  }
      }
      // XXX Wonky 0.25 in freq calc. Is maybe true? For airspy, got from a forum post!
      actual_freq = ((((nint+13) << 16) + sdm+0.25) * (__float128) pll_ref_2x) / (__float128)(1u << (div_num + 1 + 16));
      actual_tx_freq = ncot * ((__float128) clkt)/281474976710656ULL; 

      diff_ftw[j] = (1ULL<<32) *  (actual_tx_freq - actual_freq)* (1ULL<<32)/RX_SAMPLE_RATE; // do negatives work? Maybe...
      printf("receive freq %f, tx freq: %f, diff_nco: %li\n",(float) actual_freq,(float)actual_tx_freq,diff_nco[j]);


      ////////

      if (SoapySDRDevice_setFrequency(sdr[j],SOAPY_SDR_RX, 0, l_txfreqs[j]*1e6, NULL) != 0){
	printf("Soapy Set freq fail: %s\n",SoapySDRDevice_lastError());
	return -1;
      }
      printf("set soapy freq to: %f\n",l_txfreqs[j]*1e6);
      // set gain to known value for sync:
      if (SoapySDRDevice_setGain(sdr[j], SOAPY_SDR_RX, 0, SYNC_GAIN) !=0){
	printf("Soapy set gain fail: %s\n",SoapySDRDevice_lastError());
	return -1;
      }
      last_rxgains[j] = SYNC_GAIN;
    } // end loop through devices

  } // end first
  else{
    // set gain to requested value, if it wasn't
    for (j=0;j<NUM_RX;j++){
      if (l_rxgains[j] != last_rxgains[j]){
	if (SoapySDRDevice_setGain(sdr[j], SOAPY_SDR_RX, 0, l_rxgains[j]) !=0){
	  printf("Soapy set gain fail: %s\n",SoapySDRDevice_lastError());
	  return -1;
	}
	last_rxgains[j] = l_rxgains[j];
      }
    }
  }// done?
  
  
  l_npts = npts;
  l_buffer = buffer; 

  if (l_rxbuffer != NULL){
    free(l_rxbuffer);
  }
  l_rxbuffer = (float *) malloc(sizeof(int64_t) * l_npts*2);
  
  // set data to 0.
  memset(buffer,0,2*sizeof(int64_t)*npts);
  memset(l_rxbuffer,0,2*sizeof(int64_t)*npts);

  printf("returning from prep_sdr\n");
  stop_acq = 0;

  return 0;
}


int start_sdr_threads(char *device){

  // plan here is to start threads, and they will then look for sync.
  // thread func will do the hard work and then wait for signal to
  // start real scans.

  
  int i;
  // make sure streams aren't running:
  if (threads_launched){
    printf("in start_sdr_threads, but threads already launched!\n");
    return -1;
  }

  printf("launching threads\n");
  sem_init(&rxwrite,0,0); // an extra for acq to know when data is ready.
  
  //    block_signals();
  for (i=0;i<NUM_RX;i++){
    threadnos[i] = i;
    printf("start_sdr_threads: launching rx thread %i\n",threadnos[i]);
    pthread_create(&rx_threads[i],NULL,&rx_thread_func,(void *) &threadnos[i]);
  }
  threads_launched = 1;

  return 1; // success!
}


//  On first start - start_sdr_threads starts the threads, which start streaming and looking for sync signal.
// then they just keep reading...


// sync_stamp is the first tick after the sync pulse.
// start_offset is the time from the start of the sync pulse till the end of the start-up-sync program
// so sync_stamp+start_offset is the rx time stamp of t=0 - the start of the first real program.



#define PROCESS_BUFF  end_samp = rx_end_stamps[mythreadno][my_event] +sync_stamp+start_offset- stamp; \
  if (end_samp > buffsize) end_samp = buffsize;\
  for( i=start_samp;i<end_samp;i++){\
    if (my_point+my_start_in_buff < l_npts){\
      l_rxbuffer[2*(my_point+my_start_in_buff)] += buff[bnum][2*i];\
      l_rxbuffer[2*(my_point+my_start_in_buff)+1] += buff[bnum][2*i+1];\
      /*      printf("adding sample: %i to point %i\n",stamp+i,my_point); */ \
      samp_in_point += 1;\
      if (samp_in_point == samples_in_point){\
	/* do accumulated phase difference between tx and rx */		\
	uint32_t phaseval = ((diff_ftw[mythreadno]*(i+stamp))>>48) ; /* 64 bit phase, 16-bit lookup table */ \
	ival = l_rxbuffer[2*(my_point+my_start_in_buff)];		\
	qval = l_rxbuffer[2*(my_point+my_start_in_buff)+1];		\
	l_rxbuffer[2*(my_point+my_start_in_buff)] = ival * fcos[phaseval] + qval * fsin[phaseval]; \
	l_rxbuffer[2*(my_point+my_start_in_buff)+1] = qval * fcos[phaseval] - ival * fsin[phaseval]; \
      /* apply event phase and synchronization phase */ \
	ival = l_rxbuffer[2*(my_point+my_start_in_buff)];	\
	qval = l_rxbuffer[2*(my_point+my_start_in_buff)+1];\
	l_rxbuffer[2*(my_point+my_start_in_buff)] = ival * pcos + qval * psin;\
	l_rxbuffer[2*(my_point+my_start_in_buff)+1] = qval * pcos - ival * psin;\
	samp_in_point = 0;		  \
	my_point += 1;\
	if (my_point == l_rxpoints[mythreadno]){\
	  /* copy our data into l_buffer and phase correct */		\
	  printf("used phase correction: %f %f is %f degrees\n",pcos,psin,180*atan2(psin,pcos)/M_PI); \
	  for (k=0;k<l_rxpoints[mythreadno];k++){\
	    l_buffer[2*(k+my_start_in_buff)] = l_rxbuffer[2*(k+my_start_in_buff)]*200000; \
	    l_buffer[2*(k+my_start_in_buff)+1] = -l_rxbuffer[2*(k+my_start_in_buff)+1]*200000; \
	    l_rxbuffer[2*(k+my_start_in_buff)] = 0;\
	    l_rxbuffer[2*(k+my_start_in_buff)+1] = 0;\
	  }\
	  printf("rx: POSTING rxwrite, num_rx_events is %i, stamp: %li\n",num_rx_events[mythreadno],stamp); \
	  printf("start stamp %li, end stamp: %li\n",rx_start_stamps[mythreadno][my_event],rx_end_stamps[mythreadno][my_event]);\
	  sem_post(&rxwrite);\
	  my_point = 0;\
	  /* check that we're actually at the end of the event */\
	  if (rx_end_stamps[mythreadno][my_event] + sync_stamp+start_offset != stamp+end_samp){ \
	    if (prog_shm->is_noisy == 0)				\
	      printf("end_samp didn't end at the end of an RX event? %li vs %li\n",rx_end_stamps[mythreadno][my_event] + sync_stamp+start_offset, stamp+end_samp); } \
	  else printf("end_samp DID end at end of RX event, hooray!\n"); \
	}\
      }\
    }\
     } /* we've finished either the rx event or the buffer. */ 		\
  if (rx_end_stamps[mythreadno][my_event]+sync_stamp+start_offset > stamp+buffsize){\
    state = 1; /* mid receive */					\
    done_with_buff = 1;\
  }\
  else if (rx_end_stamps[mythreadno][my_event] +sync_stamp+start_offset <= stamp+buffsize){\
    state = 0;/* Seems like some of this should be above, done after we've finished a point. */ \
    my_event += 1; /* we finished the event. There might be another event within the same buffer though! */ \
    if (my_event == MAX_RX_EVENTS) my_event = 0;  \
  }

void *rx_thread_func(void *threadno){
  // we start streaming, read data - look for sync, then go into loop.
  // if we get exit flag we stop streaming and exit.
  // use rxwrite to tell acq we have data ready

  void *buffs[1];
  float buff[2][buffsize*2];
  //  int32_t buff[2][buffsize*2];
  
  int start_found=0,end_found = 0;
  uint64_t start_stamp=0, end_stamp=0, sync_stamp=0; // details for the sync pulse
  uint64_t stamp = 0; // stamp at the start of the next read.
  int mythreadno, ret, flags;
  long long timeNs;
  
  int my_point=0, samp_in_point=0, samples_in_point;
  int state=0,my_start_in_buff;
  uint64_t start_samp,end_samp;
  unsigned int my_event=0;
  bool done_with_buff = 0;
  int i,k;
  float pcos=0,psin=0,ival,qval;
  int bnum = 0;
  float real_val=0,imag_val=0,sync_phase = 0;
  
  printf("rx_thread_func %i started\n", *((int *) threadno));
  block_signals();
  mythreadno =  *((int *)threadno);
  printf("RX thread %i, start up\n",mythreadno);
  {
    struct sched_param sp;
    int result,priority;
    pthread_t this_thread;
     
 
    // set our priority to 2 lower than acq,
    this_thread = pthread_self();
    priority = sched_get_priority_max(SCHED_FIFO);
    sp.sched_priority = priority/2-2; // 2 less than acq;
    result = pthread_setschedparam(this_thread, SCHED_FIFO,&sp);
    if( result!= 0) 
      perror("rx thread: init_sched");
  
  }

  // start the stream:
  if (SoapySDRDevice_activateStream(sdr[mythreadno], rxStream[mythreadno], 0,0,0) != 0){
    printf("activate stream fail: %s\n",SoapySDRDevice_lastError());
    stop_acq = 1;
    prog_shm->stop_rx = ERROR_DONE;
    return NULL;
  }
  printf("RX thread %i, stream is active, looking for sync event\n",mythreadno);
  buffs[0] = buff[bnum];
  
  do{ // look for sync event
    // read some data
    ret = SoapySDRDevice_readStream(sdr[mythreadno], rxStream[mythreadno], buffs, buffsize, &flags, &timeNs, 100000);
    if (ret != buffsize){
      SoapySDRDevice_deactivateStream(sdr[mythreadno], rxStream[mythreadno],0,0);
      printf("reading from rx stream %i failed!\n",mythreadno);
      stop_acq = 1;
      prog_shm->stop_rx = ERROR_DONE;
      return NULL;
    }
      
    printf("RX thread %i read stamp %li\r",mythreadno, stamp);
    // in here we should correct the frequency offset - though it should be tiny and not matter.
    for (i=0;i<buffsize;i++){
      float mag;
      mag = buff[bnum][2*i]*buff[bnum][2*i] + buff[bnum][2*i+1]*buff[bnum][2*i+1];
      if (start_found == 0 ){
	if (mag > THRESH*THRESH){
	  start_found = 1;
	  start_stamp = stamp + i;
	  printf("rx thread, found start at stamp: %li\n",start_stamp);
	}
      }
      else if (end_found == 0){
	if (mag < THRESH*THRESH){
	  end_found = 1;
	  end_stamp = stamp+i;
	  sync_stamp = end_stamp;
	  printf("\nrx thread, found end at stamp: %li\n", end_stamp);
	  // the pulse duration should be 50us - check that its close.
	  if ( (end_stamp-start_stamp)/((double) RX_SAMPLE_RATE) < 48e-6 ||
	       (end_stamp-start_stamp)/((double) RX_SAMPLE_RATE) > 52e-6){
	    printf("SYNC event duration of %f seems off...\n",(end_stamp-start_stamp)/((double) RX_SAMPLE_RATE));
	  }
	  // start and end stamps are saved. Now, get the average phase in between
	  printf("num stamps: %li\n",end_stamp-start_stamp);
	  printf("averages: %f, %f\n",real_val/(end_stamp-start_stamp),imag_val/(end_stamp-start_stamp));
	  break;
	}
	else{ // start found, but not yet end
	  real_val += buff[bnum][2*i]; // really, this could phase correct for freq difference from TX.
	  imag_val += buff[bnum][2*i+1];
	}
      }
    }
    stamp += buffsize; // start of the next read
    bnum = (bnum+1)%2;
    buffs[0] = buff[bnum];
  }
#ifndef NOSYNCHARDWARE
  while (prog_shm->stop_rx >=0 && end_found == 0); 
#else
  while (prog_shm->stop_rx >=0 && end_found == 0 && stamp < 10000);
  sync_stamp = stamp;
  end_found = 1;
#endif
  printf("\n");

  // let acq know that we found the sync signal.
  if (end_found){
    sem_post(&rxwrite);
    printf("thread: posting that end was found\n");
  }
  
  sync_phase = -atan2(imag_val,real_val); // sync_phase in radians
  printf("sync_phase is %f degrees, from %f %f\n",sync_phase*180/M_PI,real_val,imag_val);

  if (prog_shm->stop_rx < 0){
    printf("soapy-sdr got stop while looking for sync pulse\n");
    if (SoapySDRDevice_deactivateStream(sdr[mythreadno], rxStream[mythreadno],0,0) != 0)
      printf("deactivate stream fail: %s\n", SoapySDRDevice_lastError());
    return NULL;
  }

  // normally, prep_sdr will change gain (and phase). But after the sync pulse we need to set the gains here
  if (SoapySDRDevice_setGain(sdr[mythreadno], SOAPY_SDR_RX, 0, l_rxgains[mythreadno]) !=0){
    printf("Soapy set gain fail: %s\n",SoapySDRDevice_lastError());
    // XX TODO, should we get out of here?
  }
  last_rxgains[mythreadno] = l_rxgains[mythreadno];
  
  printf("rx thread %i, out of sync section, starting real acq.\n",mythreadno);
  printf("ending stamp is %li, event: %i of %i, event start is: %li. current stamp is %li\n",ending_stamp, my_event, num_rx_events[mythreadno],rx_start_stamps[mythreadno][my_event], stamp);
    //  acquisition proper starts here. We need to read continuously and
    // throw away data till we know its time to start reading.
  
  if (prog_shm->stop_rx < 0){ // don't proceed, get out of here.
      if (SoapySDRDevice_deactivateStream(sdr[mythreadno], rxStream[mythreadno],0,0) != 0)
	printf("deactivate stream fail: %s\n", SoapySDRDevice_lastError());
      return NULL;
    }
    
 
    // get ready to start executing the read program.

    // XXX TODO for the moment, we're only doing boxcar averaging.
    // figure out where we put our data in the data buffer.
    my_start_in_buff = 0;
    
    for(i=0;i<mythreadno;i++)
      my_start_in_buff += l_rxpoints[i];
    printf("thread: %i, starting at point %i\n",mythreadno,my_start_in_buff);
    
    if (l_rx_sw[mythreadno] <= 0) // hopefully means it just wasn't set?
      samples_in_point = 1;
    else
      samples_in_point = RX_SAMPLE_RATE/l_rx_sw[mythreadno];
    // XXX TODO somewhere need to check that sw is multiple of SAMPLE_RATE...
    // XXX maybe in UI we just specify a divisor? Though if we do FT filtering there is more flexibility...
    
    if (samples_in_point == 0) samples_in_point = 1;
    // FIXME - try to match SW
    printf("RX: for sw of %f, using %i points per sample\n",l_rx_sw[mythreadno],samples_in_point);
    
    my_event = 0; // which event in the rx_events queue we're working on.
    my_point = 0; // how many points we've filled in our buffer
    samp_in_point = 0; // for averaging of samples into points.
    state = 0; // 0 means nothing happening. 1 means we're acquiring.
    memset(l_rxbuffer,0,2*sizeof(int64_t)*l_npts); // set all data to 0. This is a bit wasteful as it gets done on the fly too. done in each thread? XXX

    printf("RX, going in to read loop, my_event: %i, num_rx_events: %i\n",my_event,num_rx_events[mythreadno]);
   // execute the program: read data, find events, store the data.

    do{
      ret = SoapySDRDevice_readStream(sdr[mythreadno], rxStream[mythreadno], buffs, buffsize, &flags, &timeNs, 100000);
      //      printf("rx thread %i stamp %li\r",mythreadno, stamp);
      //      printf("current stamp: %li, looking for: %li\n",stamp, rx_start_stamps[mythreadno][my_event]+start_stamp+start_offset);
      done_with_buff = 0;
      while (my_event != num_rx_events[mythreadno] && done_with_buff == 0  ){ // there's an event to work on!
	//	printf("in while loop with state: %i\n",state);
	if (state == 0){ // hopefully we haven't missed the start. check for it
	  //	  printf("in rx, found an event start: %li, end: %li\n",rx_start_stamps[mythreadno][my_event],rx_end_stamps[mythreadno][my_event]);
	  if (stamp > rx_start_stamps[mythreadno][my_event] + sync_stamp+start_offset){
	    // EEEK, this could mess up lots of stuff...
	    printf("\nMISSED START OF RX EVENT?? my_event is %i, and num_rx_events is %i\n",my_event,num_rx_events[mythreadno]);
	    printf("\ngot timestamp: %li, start was: %li\n", stamp, rx_start_stamps[mythreadno][my_event]);
	    // nope if we missed it, we quit:
	    stop_acq = 1;
	    prog_shm->stop_rx = ERROR_DONE;
	    
	  }// start_offset is the difference between the sync start and the 0 of the program.
	  else if (stamp + buffsize > rx_start_stamps[mythreadno][my_event]+sync_stamp+start_offset ){ // our rx event starts in the current data
	    printf("rx data in current frame. start stamp is %li, current stamp is %li\n",rx_start_stamps[mythreadno][my_event],stamp);
	    //	    printf("rx using phase: %f\n",rxphase[mythreadno][my_event]);
	    //	    printf("RX THREAD: %i, using phase: %f, vals: %f %f\n",mythreadno,rxphase[mythreadno],pcos,psin);
	    start_samp = rx_start_stamps[mythreadno][my_event]+sync_stamp+start_offset-stamp; // first sample
	    //	    pcos = cos(rx_event_phase[mythreadno][my_event]-sync_phase); // XX TODO is sign of sync_phase correct? 
	    //	    psin = sin(rx_event_phase[mythreadno][my_event]-sync_phase);
	    {
	      // need phase to be between 0 and 360. Does this do the job?
	      int index = (int) ((rx_event_phase[mythreadno][my_event] - sync_phase)/2/M_PI * TRIGLEN);
	      if (index < 0) printf("index is negative: %i\n",index);
	      index += TRIGLEN*(-index/TRIGLEN + 1)*(index<0); // if its less than 0, then make it positive
	      printf("event phase %f, sync_phase %f\n",rx_event_phase[mythreadno][my_event],sync_phase);
	      index = index%TRIGLEN; // phase could be > 2pi
	      pcos = fcos[index];
	      psin = fsin[index];
	      printf("fcos, fsin, mag: %f %f %f, index %i\n",pcos,psin,pcos*pcos+psin*psin,index);
	    }
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
      stamp += buffsize;

      
    } while((stamp < ending_stamp+sync_stamp+start_offset)   && (prog_shm->stop_rx >= 0) );
    printf("rx done loop, my_event is: %i, num_rx_events: %i\n",my_event,num_rx_events[mythreadno]);
    
    // either we reached the end of our events or we're passed the stop stamp, or we've been told to stop.
    
    // if we errored out of tx or rx and we have points we were supposed to generate, wake up acq.
    // XXX TODO is stop_acq right? or prog_shm->stop_rx ?
    if (stop_acq && l_rxpoints[mythreadno] > 0)
      sem_post(&rxwrite);
   
  
    // if we're here, either we got to ending_stamp, or stop_rx was set to cancel.
    // either way, we're done.
    printf("RX: thread %i out of rx loop with my_point: %i and rxpoints: %li\n",mythreadno,my_point,l_rxpoints[mythreadno]);
    printf("RX: thread %i finished with samp_in_point: %i\n",mythreadno,samp_in_point);
    fflush(stdout);

    if (SoapySDRDevice_deactivateStream(sdr[mythreadno], rxStream[mythreadno],0,0) != 0)
      printf("deactivate stream fail: %s\n", SoapySDRDevice_lastError());
    
    printf("RX %i: streams stopped.\n",mythreadno);
	    
    return NULL;
  }
  

int check_sync_was_found(){
  int count = 0,rval;
  rval = sem_getvalue(&rxwrite, &count);
  if (rval !=0){
    printf("check_sync_was_found: sem_getvalue failed\n");
  }
  else // clear whatever is there in the semaphore.
    for(rval=0;rval<count;rval++)
      sem_wait(&rxwrite);
  return rval;
}

int wait_for_sdr_data(){
  int eo,i,count=0;
  //    printf("wait for sdr_data: about to wait for last rx thread\n");
  for(i=0;i<NUM_RX;i++)
    count += (l_rxpoints[i] > 0);
  printf("in wait for sdr data, waiting for %i receivers to finish\n",count);
  // we wait for as many threads as have data to collect.
  for(i=0;i<count;i++){
    do{
      eo = sem_wait(&rxwrite);
      if (eo == -1 && errno == EINTR)
	printf("wait for sdr, woken by a signal.\n");
    } while (eo == -1 && errno == EINTR && prog_shm->stop_rx >= 0);
    printf("wait_for_sdr_data, stop_rx is: %i\n",prog_shm->stop_rx);
    if (prog_shm->stop_rx < 0) return -1; // XXX is this right?
  }

  // if we get here and stop_acq is 1, rx errored out.
  printf("wait for sdr data, returning. stop_acq is: %i\n",stop_acq);
  if (stop_acq) return -1;
  else return 0;
}

void join_sdrs(){
  int i;
  if (threads_launched == 0 ) return;
  
  printf("joining sdrs, waiting for threads to exit\n");
  prog_shm->stop_rx = THREAD_EXIT; // will shut down rx threads
  
  for (i=0;i< NUM_RX;i++)
    pthread_join(rx_threads[i], NULL);
  sem_destroy(&rxwrite);


  threads_launched = 0;
  printf("join_sdrs, threads done\n");
}
       	
       	

void generate_rx_sdr_events(int first_time){

  // so, if we're here - there is a sequence calculated by the pulse program, and here we want to feed it into the thread.
  int i,j;
  for(i=0;i<NUM_RX;i++){
    if (first_time){
      num_rx_events[i] = 0;
      ending_stamp = 0;
      prog_shm->rx_stamp_count = 0;
    }
    printf("in generate_rx_sdr_events, there are %i events to process for rx %i\n",prog_shm->rx_staging_num_events[i],i); 
    for(j=0;j<prog_shm->rx_staging_num_events[i];j++){
      printf("processing event: %i\n",j);
      // the times in rx_staging_start_time are in due ticks since start of scan.
      // durations in rx_staging_duration are in due ticks.
      // here we need to generate start and end times in RX ticks since start of first real program.
      //      printf("start calc: %li\n", prog_shm->rx_staging_start_time[i][j] *RX_SAMPLE_RATE);
      printf("rx event duration (in pp ticks): %li\n",prog_shm->rx_staging_duration[i][j]);
      printf("start: %li\n", prog_shm->rx_stamp_count + (prog_shm->rx_staging_start_time[i][j]*RX_SAMPLE_RATE+DUE_PP_CLOCK-1)/DUE_PP_CLOCK); // first stamp of event
      printf("dur: %li\n",(prog_shm->rx_staging_duration[i][j]*RX_SAMPLE_RATE+DUE_PP_CLOCK-1)/DUE_PP_CLOCK);   // first stamp after end of event XX double check!
#ifndef NOHARDWARE
      rx_start_stamps[i][num_rx_events[i]] = prog_shm->rx_stamp_count + (prog_shm->rx_staging_start_time[i][j]*RX_SAMPLE_RATE+DUE_PP_CLOCK-1)/DUE_PP_CLOCK; // first stamp of event
      rx_end_stamps[i][num_rx_events[i]] = rx_start_stamps[i][num_rx_events[i]]+(prog_shm->rx_staging_duration[i][j]*RX_SAMPLE_RATE+DUE_PP_CLOCK-1)/DUE_PP_CLOCK;   // first stamp after end of event XX double check!
      rx_event_phase[i][num_rx_events[i]] = prog_shm->rx_phase[i]*M_PI/180.; // rx_phase is in degrees. rx_event_phase in radians
      printf("event phase is %f\n",rx_event_phase[i][num_rx_events[i]]);
#endif
      num_rx_events[i] += 1;
      if (num_rx_events[i] == MAX_RX_EVENTS) num_rx_events[i] = 0; // loop the ring buffer
      printf("generated rx event %i for rx %i. Start: %li end: %li\n",num_rx_events[i],i,rx_start_stamps[i][num_rx_events[i]-1],rx_end_stamps[i][num_rx_events[i]-1]);
    }
  }
  printf("generate_rx_sdr_events, rx_stamp_count going from: %li ",prog_shm->rx_stamp_count);
  prog_shm->rx_stamp_count += prog_shm->prog_dur_in_rx_stamps;
  printf("to: %li\n",prog_shm->rx_stamp_count);
  ending_stamp = prog_shm->rx_stamp_count; // hmm, are ending_stamp and prog_shm->rx_stamp_count identical?
  printf("returning from generate_rx_sdr_events, ending stamp is: %li\n",ending_stamp);
}

