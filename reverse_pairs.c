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

FILE *infile;
float inarray[360];
float outarray[360];

int i,j;
float f,m,b,junk,first_phase;

//infile=fopen("phases1.txt","r");

 fprintf(stderr,"Program reads from stdin.  must pipe input in\n");
 for (i=0;i<360;i++){
   scanf("%f %f %f",&f,&inarray[i],&junk);
   if (i==0)
     first_phase = inarray[0];
   // This is a phase shift to make 0 be 0 - just pick the first value
   inarray[i]-= first_phase;
   //   printf("got: %f %f\n",f,inarray[i]);
 }

 for(i=0;i<360;i++){
   // need to find the output value
   
   for(j=1;j<360;j++){
     if (inarray[j] > (float)i){ // bingo, its between j and j-1
       m = inarray[j]-inarray[j-1];
       b = inarray[j]-m*j;
       outarray[i] = ((float)i-b)/m;
       printf("%i %f\n",i,outarray[i]);

       // longer version, with the bit values in there:
       //       printf("%i %f %i %i\n",i,outarray[i],(int) ((cos(outarray[i]*M_PI/180.)+1.0)*511.51), (int) ((sin(outarray[i]*M_PI/180.)+1.0)*511.51));
       j=365;// break out of loop
     }
   }
   if (j == 360) { // didn't find it, assume its at the end
     m = 360.-inarray[359];
     b= 360. - m*360.;
     outarray[i] = ((float) i-b)/m;
     printf("%i %f\n",i,outarray[i]);
   }

   


 }
}


