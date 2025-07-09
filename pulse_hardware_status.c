/*

Copyright (c) 2025 Carl Michal with contributions of:
Scott Nelson
Phil Eles
Matt Grinder

This file is part of Xnmr.

Xnmr is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free
Software Foundation, either version 3 of the License, or (at your
option) any later version.

Xnmr is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
for more details.

You should have received a copy of the GNU General Public License
along with Xnmr. If not, see <https://www.gnu.org/licenses/>.

*/

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
