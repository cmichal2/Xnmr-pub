/* timel.c
 *
 *
 *
 *
 */

#include "pulse.h"

#include <stdio.h>

int main()
{

  float pulse_time;
  float trig_time;
  float post_trig_time;
  float PPO_time;
  float recovery;
  double time;

  pulse_program_init();
 
  do {

    GET_PARAMETER_FLOAT( pulse_time);
    GET_PARAMETER_FLOAT( trig_time );
    GET_PARAMETER_FLOAT( post_trig_time );
    GET_PARAMETER_FLOAT( PPO_time );
    GET_PARAMETER_FLOAT( recovery );

    begin();
   
    time = pulse_time / 1000000;
    event( 0.001, 1, BNC_0, 0 );
    event( time, 2, BNC_0, 1, PHASE, (get_acqn() %4) );           //the pulse

    time = recovery / 1000000;
    event( time, 0 );

    time = trig_time / 1000000;
    event( time, 1, SCOPE_TRIG, 1 );
   
    time = post_trig_time / 1000000;
    event( time,0);                           //waiting for scope to finish its scan
  
    time = PPO_time / 1000000;
    event( time, 1, PP_OVER, 1 );
   
  } while( ready( get_acqn() % 4 ) == P_PROGRAM_CALC );

  //printf( "pprog exiting\n" );

  done();

  return 0;
}















