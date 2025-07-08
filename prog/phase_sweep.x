/* phase_sweep.x - for measuring transmitter phases
 *
 *
 */

#define CHANNEL1_C

#include "pulse.h"
#include <math.h>

int main()
{
  float pw1;
  float PPO;
  float rd;
  float sf1offset;
  double sf1,sf,sf2;
  int acqn,i;
  
  
  pulse_program_init();
  
  do {
    GET_PARAMETER_FLOAT (sf1offset);
    GET_PARAMETER_FLOAT( pw1);
    GET_PARAMETER_FLOAT( PPO );
    GET_PARAMETER_FLOAT( rd );
    GET_PARAMETER_DOUBLE(sf1); // this is in MHz 
    GET_PARAMETER_DOUBLE(sf2); // this is in MHz 
    

    

    acqn = get_acqn();
    
    begin();
    
    sf= sf1+((double) sf1offset)/1e6;
    
    set_freq1(sf+21.);
    set_freq2(sf2+21.);

    EVENT 50e-9 {AMP1,1.0} ;
    EVENT 50e-6 {RCVR_GATE,1}{RF1,1};
    for (i=-5;i<359;i++)
      EVENT pw1 {RF1,1} {PHASE1,(float) i} {AD_STAQ,1}{RCVR_GATE,1};
      

    for (i=0;i<359;i++)
      EVENT pw1 {RF1,1} {PHASE1,(float) i} {AD_STAQ,1}{RCVR_GATE,1};

      
      EVENT rd ;
      //    EVENT get_dwell()*get_npts() {AD_STAQ, 1} {RCVR_GATE,1} ;
      
      
      
      EVENT PPO  {PP_OVER, 1};
      
  } while( ready( 0 ) == P_PROGRAM_CALC );
  
  done();
  
  return 0;
}













