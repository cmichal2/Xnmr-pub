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

#include <stdio.h>
#include <math.h>


main(){

  // set up is 50us per point, dwell=25us 
  //collect 750 points - 720 should be good

FILE *infile;
float time,real,imag,phase,mag,last_phase=-360;
int i,point;
 char buff[200];


infile = fopen("acq_tempexport.txt","r");
 fgets(buff,200,infile);
 for (i=0;i<720;i++){
   fscanf(infile,"%i %f %f %f",&point,&time,&real,&imag);
   //fprintf(stderr,"%i %f %f %f\n",point,time,real,imag);
   phase = -atan2(imag,real)*180./M_PI;
   if (phase < last_phase-5) phase+=360;
   last_phase = phase;
   if (i%2 == 0)
     printf("%f %f %f\n",i/2.,phase,sqrt(real*real+imag*imag));
 }



}

