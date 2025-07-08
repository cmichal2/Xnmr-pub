/* one_pulse.c - a very simple pulse program
 *
 */

#include "pulse.h"
int main()
{
  int i;
  float pulse_time;
  float pulse_padding;
  float post_trig_time;
  float PPO_time;
  float recovery;
  float sf1offset;
  double time;
  double sf1,sf,sf2;
  int lock_up;

  pulse_program_init();
  do {
    GET_PARAMETER_FLOAT (sf1offset);
    GET_PARAMETER_FLOAT( pulse_time);
    GET_PARAMETER_FLOAT( pulse_padding );
    //    GET_PARAMETER_FLOAT( trig_time );
    GET_PARAMETER_FLOAT( post_trig_time );
    GET_PARAMETER_FLOAT( PPO_time );
    GET_PARAMETER_FLOAT( recovery );
    GET_PARAMETER_DOUBLE(sf1); // this is in MHz 
    GET_PARAMETER_DOUBLE(sf2); // this is in MHz 
    GET_PARAMETER_INT(lock_up);
    
    begin();

    sf= sf1+((double) sf1offset)/1e6;

    set_freq1(sf*1.0);
    set_freq2(sf2);

    event( 0.001, 1, BNC_0, 0 );
    
    event( pulse_padding, 2, BNC_3, 1, PHASE, (get_acqn() %4) );   
    //This should latch the phase output

    event( pulse_time, 3, BNC_0, 1, BNC_3, 1, PHASE, (get_acqn() %4) );  //the pulse

    for(i=0;i<20500;i++)
      event( recovery, 0 );
    if (lock_up ==1)
       for(i=0;;i++);

    event( get_dwell()*get_npts(), 1, SCOPE_TRIG, 1 );
   
    event( post_trig_time,0);                 //waiting for scope to finish its scan
  
    event( PPO_time, 1, PP_OVER, 1);
   
  } while( ready( get_acqn() %4 ) == P_PROGRAM_CALC );

  done();

  return 0;
}





