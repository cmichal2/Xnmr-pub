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

#define MAX_EVENTS 32767
#define NUM_CHIPS 21
#include <stdio.h>
#include <sys/time.h>
#include <sys/mman.h>
main(){
struct timeval start_time,end_time;
  struct timezone tz;
  float d_time;

int i,j;

unsigned char prog_image[MAX_EVENTS+1][NUM_CHIPS],prog_imagea[MAX_EVENTS+1][NUM_CHIPS];
unsigned char image2[NUM_CHIPS][MAX_EVENTS+1],image2a[NUM_CHIPS][MAX_EVENTS+1];

  mlockall( MCL_FUTURE );

for (i=0;i<NUM_CHIPS;i++)
  for (j=0;j<MAX_EVENTS;j++)
    prog_image[j][i] = i*j;



 gettimeofday(&start_time,&tz);

for (i=0;i<NUM_CHIPS;i++)
  for (j=0;j<MAX_EVENTS;j++)
    image2[i][j]=prog_image[j][i]+1;

 gettimeofday(&end_time,&tz);
 d_time=(end_time.tv_sec-start_time.tv_sec)*1e6+(end_time.tv_usec-start_time.tv_usec);
 printf("transpose time1: %.0f us\n",d_time);



 gettimeofday(&start_time,&tz);
   for (i=0;i<NUM_CHIPS;i++)
     for (j=0;j<MAX_EVENTS;j++)
       prog_image[j][i]=image2[i][j]+1;

 gettimeofday(&end_time,&tz);
 d_time=(end_time.tv_sec-start_time.tv_sec)*1e6+(end_time.tv_usec-start_time.tv_usec);
 printf("transpose time2: %.0f us\n",d_time);



 gettimeofday(&start_time,&tz);
 for (i=0;i<NUM_CHIPS;i++)
   for (j=0;j<MAX_EVENTS;j++)
     prog_imagea[j][i]=prog_image[j][i]+1;

 gettimeofday(&end_time,&tz);
 d_time=(end_time.tv_sec-start_time.tv_sec)*1e6+(end_time.tv_usec-start_time.tv_usec);
 printf("copy 1 time: %.0f us\n",d_time);

 gettimeofday(&start_time,&tz);
 for (i=0;i<NUM_CHIPS;i++)
   for (j=0;j<MAX_EVENTS;j++)
     prog_imagea[j][i]=prog_image[j][i]+1;

 gettimeofday(&end_time,&tz);
 d_time=(end_time.tv_sec-start_time.tv_sec)*1e6+(end_time.tv_usec-start_time.tv_usec);
 printf("copy 1a time: %.0f us\n",d_time);
 
 gettimeofday(&start_time,&tz);
 for (i=0;i<NUM_CHIPS;i++)
   for (j=0;j<MAX_EVENTS;j++)
     image2a[i][j]=image2[i][j]+1;

 gettimeofday(&end_time,&tz);
 d_time=(end_time.tv_sec-start_time.tv_sec)*1e6+(end_time.tv_usec-start_time.tv_usec);
 printf("copy2 time: %.0f us\n",d_time);



 gettimeofday(&start_time,&tz);
 for (i=0;i<NUM_CHIPS;i++)
   for (j=0;j<MAX_EVENTS/4;j++)
     ((unsigned int *) image2a[i])[j]=((unsigned int *)image2[i])[j] + 1;

 gettimeofday(&end_time,&tz);
 d_time=(end_time.tv_sec-start_time.tv_sec)*1e6+(end_time.tv_usec-start_time.tv_usec);
 printf("copy2a time: %.0f us\n",d_time);




 gettimeofday(&start_time,&tz);
 memcpy(image2,prog_image,MAX_EVENTS*NUM_CHIPS);
 gettimeofday(&end_time,&tz);
 d_time=(end_time.tv_sec-start_time.tv_sec)*1e6+(end_time.tv_usec-start_time.tv_usec);
 printf("copy time: %.0f us\n",d_time);

 printf("prog: %i %i\n",image2a[5][3],image2a[7][4]);

}
