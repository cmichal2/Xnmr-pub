/* pulse_test.c
 *
 * a sample pulse program for Xnmr software
 * UBC Physics, 2000
 */

#include "pulse.h"

#include <stdio.h>

int main()
{

  int dummy_int;
  float dummy_float;
  char dummy_text[20];
  double dummy_double;

  pulse_program_init();


  do {

    begin();

    GET_PARAMETER_FLOAT( dummy_float );
    GET_PARAMETER_INT( dummy_int );
    GET_PARAMETER_TEXT( dummy_text );
    GET_PARAMETER_DOUBLE( dummy_double );

    printf( "pprog parameters: dummy_int = %d, dummy_float = %f, dummy_text = %s, dummy_double = %f\n", dummy_int, dummy_float, dummy_text, dummy_double );

    event( 5e-4, 1, BNC_0, 0 );
    event( 1e-5, 1, BNC_0, 1 );           //the pulse
    event (1e-5,0);
    event( 0.1024, 1, SCOPE_TRIG, 1 );
    event( 0.008,0);                   //waiting for scope to finish its scan
    event( 0.25, 1, PP_OVER, 1 );
   
  } while( ready( PHASE0 ) == P_PROGRAM_CALC );

  printf( "pprog exiting\n" );

  done();

  return 0;
}















