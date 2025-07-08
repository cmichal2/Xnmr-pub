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

    //    event( 0.001, 1, BNC_0, 1 );
    /* event( 1e-6, 1, B0P0, 1 );           //the pulse
    event( 1e-6, 1, B0P0, 0 );           //the pulse
    event( 5e-7, 1, B0P0, 1 );           //the pulse
    event( 5e-7, 1, B0P0, 0 );           //the pulse
    event( 2.5e-7, 1, B0P0, 1 );           //the pulse
    event( 2.5e-7, 1, B0P0, 0 );           //the pulse
    event( 1.5e-7, 1, B0P0, 1 );           //the pulse
    event( 1.5e-7, 1, B0P0, 0 );           //the pulse
    event( 5e-8, 1, B0P0, 1 );           //the pulse
    event( 5e-8, 1, B0P0, 0 );           //the pulse
    event (.1,1,B0P0,1 ); */
    event( 2.048e-1, 1, SCOPE_TRIG, 1 );
    event( 7e-3,0);                   //waiting for scope to finish its scan
    event( 0.5, 1, PP_OVER, 1 );
   
  } while( ready( PHASE0 ) == P_PROGRAM_CALC );

  printf( "pprog exiting\n" );

  done();

  return 0;
}















