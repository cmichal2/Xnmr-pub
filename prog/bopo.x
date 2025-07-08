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

    EVENT 2.048e-1 { SCOPE_TRIG, 1};
    EVENT 7e-3;                   //waiting for scope to finish its scan
    EVENT 0.5 { PP_OVER,1};
   
  } while( ready( PHASE0 ) == P_PROGRAM_CALC );

  printf( "pprog exiting\n" );

  done();

  return 0;
}















