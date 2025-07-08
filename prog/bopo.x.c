/* bopo.c
 *
 *
 *
 *
 */

#include "pulse.h"

#include <stdio.h>

int main()
{

  pulse_program_init();

  do {

    begin();

event(2.048e-1 ,1, SCOPE_TRIG, 1);
event(7e-3,0);
event(0.5 ,1, PP_OVER,1);
   
  } while( ready( PHASE0 ) == P_PROGRAM_CALC );

  printf( "pprog exiting\n" );

  done();

  return 0;
}















