/* excercise.c
 *
 * a complicated but otherwise useless pulse program
 *
 *
 */

#include <stdio.h>

#include "pulse.h"


int main ()
{
  int i,j;
  float last_time;

  pulse_program_init();

  GET_PARAMETER_FLOAT( last_time );
  do {
    begin();

    event( 5.0e-7, 1, BNC_0, 1 );
    
    for( i=0; i<8; i++ )
      for( j=0; j<16; j++ )
	event( 5.0e-7, 1, B1P0+i, 1<<j );
    
    for( i=0; i<10000; i++ ) {
      event( 1.0e-7, 4, B1P0, 65535, B1P0, 65535, B2P0, 65535, B3P0, 65535 );
      event( 1.0e-7, 4, B1P16, 65535, B1P16, 65535, B2P16, 65535, B3P16, 65535 );
    }

    event( 0.0085, 1, SCOPE_TRIG, 1 );
    event( last_time/1000, 1, PP_OVER, 1 );

  } while( ready( PHASE0 ) == P_PROGRAM_CALC );

  printf( "pprog exiting\n" );

  done();
  

  return 0;
}
