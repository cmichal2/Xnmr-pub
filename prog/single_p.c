/* single_p.c
 *
 *
 *
 *
 */

#include "pulse.h"

#include <stdio.h>

int main()
{

  float duration_us;
  unsigned long cycles;

  pulse_program_init();

  GET_PARAMETER_FLOAT( duration_us);
 
  cycles = (long) ( duration_us * 20 );

  do {

    begin();

    event( 1e-6, 1, BNC_0, 0 );
    event( duration_us/1e6, 1, BNC_0, 1 );           //the pulse
    event (1e-6,0);
    event( 0.1024, 1, SCOPE_TRIG, 1 );
    event( 0.008,0);                   //waiting for scope to finish its scan
    event( 0.2, 1, PP_OVER, 1 );
   
  } while( ready( PHASE0 ) == P_PROGRAM_CALC );

  printf( "pprog exiting\n" );

  done();

  return 0;
}















