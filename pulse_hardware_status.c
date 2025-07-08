#include "pulse_hardware.c"
#include <sys/io.h>   //for ioperm routine
//#include <asm/io.h>   //for inb, outb, etc.
#include <stdio.h>    //for printf
#include <sys/time.h>
#include <unistd.h>

main(){

int ph_base=0x278;
int i;
int b;

  i = ioperm( ph_base, 8, 1 );

  b = inb( ph_base+SPP_STAT );
  printf( "after start: Status register bits: %d, %d, %d\n", b & 0x08, b&0x10, b&0x20 );
  i = ioperm( ph_base, 8, 0);


}
