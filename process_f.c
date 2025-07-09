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

#define _GNU_SOURCE
/* process_f.c
 *
 * Implementation of the process panel page in Xnmr
 *
 * UBC Physics
 * April, 2000
 * 
 * written by: Scott Nelson, Carl Michal
 */

#include "param_utils.h"
#include "process_f.h"
#include "panel.h"
#include "xnmr.h"
#include "buff.h"
#include "nr.h"
#include "xnmr.h"
#include "fftw3.h"

#include <gtk/gtk.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

process_button_t process_button[ MAX_PROCESS_FUNCTIONS ];
process_data_t* active_process_data;
process_data_t* acq_process_data;
char wisdom_imported=0;

GtkWidget *r_local_button,*r_global_button;

/*
 *  Data Processing Functions
 *
 *  If the function was called from gtk by pressing a button, the *widget pointer will
 *  point to the calling Gtk object.  If the process was called explicitely by the process
 *  data function, the pointer will be NULL
 */


void load_wisdom(){
  char s[PATH_LENGTH],hname[PATH_LENGTH]; 
  int rval;
  
  if (wisdom_imported == 0 ){
    path_strcpy(s,getenv(HOMEP));
    path_strcat(s,DPATH_SEP "Xnmr" DPATH_SEP WISDOM_FILEF "-" );
    gethostname(hname, PATH_LENGTH);
    path_strcat(s,hname);
    fftwf_import_wisdom_from_filename(s);
    
    path_strcpy(s,getenv(HOMEP));
    path_strcat(s,DPATH_SEP "Xnmr" DPATH_SEP WISDOM_FILE "-");
    path_strcat(s,hname);
    rval = fftw_import_wisdom_from_filename(s);
    printf("wisdom import from file: %s, returned %i\n",s,rval);
    wisdom_imported = 1;
  }
     
}
void save_wisdom(){
  char s[PATH_LENGTH], hname[PATH_LENGTH]; 
  
  path_strcpy(s,getenv(HOMEP));
  path_strcat(s,DPATH_SEP "Xnmr" DPATH_SEP WISDOM_FILEF "-");
  gethostname(hname, PATH_LENGTH);
  path_strcat(s,hname);
  fftwf_export_wisdom_to_filename(s);

  path_strcpy(s,getenv(HOMEP));
  path_strcat(s,DPATH_SEP "Xnmr" DPATH_SEP WISDOM_FILE "-");
  path_strcat(s,hname);
  fftw_export_wisdom_to_filename(s);
}

gint do_offset_cal_and_display( GtkWidget *widget, double *unused )
{
  dbuff *buff;
  gint result;

  result = do_offset_cal( widget, unused );

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  draw_canvas( buff );
  return result;
}


gint do_offset_cal( GtkWidget *widget, double *unused )

{
  int i, j;
  int count;
  float offset;
  dbuff *buff;
  int is_symm, start;
  double group_delay = 0;
  

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  if (buff == NULL){
    popup_msg("do_offset_cal panic! buff is null!",TRUE);
    return 0;
  }

  is_symm = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.symm_check));
  pfetch_float(&buff->param_set, "group_delay", &group_delay,  0);
  if (group_delay > 0 && !is_symm && ((buff->flags & FT_FLAG) == 0))
    start = group_delay;
  else start = 0;
  
  for( j=0; j<buff->npts2; j++ ) {

    count = 0;
    offset = 0.0;

    //first determine the offset for the real channel

    for( i= (buff->npts*9/10)*2+j*2*buff->npts; i < (j+1)*2*buff->npts; i+=2 ) {
      offset += buff->data[i];
      count ++;
    }

    offset = offset / count;
    
    for( i=(j+start)*2*buff->npts; i < (j+1)*2*buff->npts; i+= 2 )
      buff->data[i] -= offset;
    
    //Now do the imaginary channel
    count = 0;
    offset = 0.0;

    for( i= (buff->npts*9/10)*2+1+j*2*buff->npts; i < (j+1)*2*buff->npts; i+=2 ) {
      offset += buff->data[i];
      count ++;
    }

    offset = offset / count;

    for( i=1+(j+start)*2*buff->npts; i < (j+1)*2*buff->npts; i+= 2 )
      buff->data[i] -= offset;
    /*
      if (group_delay > 0 && !is_symm && ((buff->flags & FT_FLAG) == 0)){ 
      // do the ring-up separately.
      count = 0;
      offset = 0.0;
      
      //first determine the offset for the real channel
      
      for( i= j*2*buff->npts; i < j*2*buff->npts+2*group_delay; i+=2 ) {
	offset += buff->data[i];
	count ++;
      }
      offset = offset / count;
      
      for( i= j*2*buff->npts; i < j*2*buff->npts+2*group_delay; i+=2 ) 
	buff->data[i] -= offset;
      
      //Now do the imaginary channel
      count = 0;
      offset = 0.0;
      for( i= 1+j*2*buff->npts; i < j*2*buff->npts+1+2*group_delay; i+=2 ) {
	offset += buff->data[i];
	count ++;
      }
      
      offset = offset / count;
      
      for( i= 1+j*2*buff->npts; i < j*2*buff->npts+1+2*group_delay; i+=2 )
	buff->data[i] -= offset;
	} */
  }
  return 0;
}



gint do_offset_cal_a_and_display( GtkWidget *widget, double *unused )
{
  dbuff *buff;
  gint result;

  result = do_offset_cal_a( widget, unused );


  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  draw_canvas( buff );
  return result;
}


gint do_offset_cal_a( GtkWidget *widget, double *unused )

     /* this version does the BC by doing the ft, setting the center point to be the average 
	of those on either side, and then doing the reverse ft */

{
  int j;
  dbuff *buff;
  
  
  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  if (buff == NULL){
    popup_msg("do_offset_cal panic! buff is null!",TRUE);
    return 0;
  }

  
  do_ft(widget, unused);
  for ( j = 0;j<buff-> npts2; j++){
    buff->data[(2*j+1)*buff->npts]= (buff->data[(2*j+1)*buff->npts+2] + buff->data[(2*j+1)*buff->npts-2])/2.0;
    buff->data[(2*j+1)*buff->npts+1]= (buff->data[(2*j+1)*buff->npts+3] + buff->data[(2*j+1)*buff->npts-1])/2.0;
  }

  do_ft(widget, unused);
  cursor_normal(buff);
  
  return 0;
}


gint do_zero_imag_and_display( GtkWidget *widget, double *unused )
{
  dbuff *buff;
  gint result;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  result = do_zero_imag( widget, unused );

  draw_canvas( buff );
   return result;
     }

gint do_zero_imag(GtkWidget *widget, double *unused)

{

  dbuff *buff;
  int i,j;

  if( widget == NULL ) {
    buff = buffp[ upload_buff ];
    // fprintf(stderr,"do_ft- on buffer %i\n",upload_buff );
      }      
  else {
    buff = buffp[ current ];
    // fprintf(stderr,"do_ft- on buffer %i\n",current );
  }
  if (buff == NULL){
    popup_msg("do_zero_imag panic! buff is null!",TRUE);
    return 0;
  }


  for(i=0;i<buff->npts2;i++){
    for(j=0;j<buff->npts;j++)
      buff->data[2*j+1+i*buff->npts*2] = 0.;
  }


  

  return TRUE;
  
}

gint do_ft_and_display( GtkWidget *widget, double *unused )
{
  dbuff *buff;
  gint result;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  result = do_ft( widget, unused );

  draw_canvas( buff );
   return result;
}

gint do_ft(GtkWidget *widget, double *unused)

{
  /* do fourier transform in one dimension */
  dbuff *buff;
  int i,j,shift;
  float scale, swap;
  double group_delay=0;
  double *newdata;
  
  static fftw_plan plan1;
  int n[] = {0};
 

  if( widget == NULL ) {
    buff = buffp[ upload_buff ];
    // fprintf(stderr,"do_ft- on buffer %i\n",upload_buff );
  }
  else {
    buff = buffp[ current ];
    // fprintf(stderr,"do_ft- on buffer %i\n",current );
  }
  if (buff == NULL){
    popup_msg("do_ft panic! buff is null!",TRUE);
    return 0;
  }
  
  pfetch_float(&buff->param_set, "group_delay", &group_delay,  0);

  buff->flags ^= FT_FLAG; //toggle the ft flag
  
  cursor_busy(buff);
  newdata = g_malloc(sizeof(double) *2*buff->npts*buff->npts2);  // array to hold new data.
  load_wisdom();
  n[0] = buff->npts;  // elements in each transform
  if (buff->flags & FT_FLAG){
    plan1 = fftw_plan_many_dft(1,n,buff->npts2,
				(fftw_complex *) newdata, NULL,1,buff->npts,
				(fftw_complex *) newdata,NULL,1,buff->npts,
				FFTW_FORWARD,FFTW_MEASURE);
    scale = buff->npts/2.0;
  }
  else{
    plan1 = fftw_plan_many_dft(1,n,buff->npts2,
				(fftw_complex *) newdata, NULL,1,buff->npts,
				(fftw_complex *) newdata,NULL,1,buff->npts,
				FFTW_BACKWARD,FFTW_MEASURE);
    scale = 2.0;
  }

	   // args are rank = 1, n - number of elements it transform, howmany transforms to do (npts2)
	   // the input data, inembed =NULL, istride=1,idist=buff->npts
	   // output = data, onembed=NULL,ostride=1, odist=npts
	   // sign , flags.



  // if ((symmetric and FT forward) or reverse), swap
  if ((buff->flags & FT_FLAG && gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.symm_check))) || ((buff->flags & FT_FLAG)  == 0))
    shift = buff->npts/2 + buff->npts%2;
  else
    shift = 0.;

    // if reverse, not symm, and there is a group delay, then unwind the group delay phase shift
  if ( ((buff->flags & FT_FLAG) ==0) && !gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.symm_check)) && group_delay > 0){
    for (j=0;j<buff->npts;j++){
      for (i=0;i<buff->npts2;i++){
	double angle = -2*M_PI*(j-buff->npts/2)*group_delay/buff->npts;
	swap = buff->data[2*j + i*2*buff->npts] * cos(angle) - buff->data[2*j+1 + i*2*buff->npts]*sin(angle);
	buff->data[2*j+1 +i*2*buff->npts] = buff->data[2*j + 1+ i*2*buff->npts] * cos(angle) + buff->data[2*j + i*2*buff->npts] * sin(angle);
	buff->data[2*j + i*2*buff->npts] = swap;
      }
    }
  }
      

  
  for(i=0;i<buff->npts2;i++){
    for(j=0;j<buff->npts;j++){
      newdata[2*j+i*2*buff->npts] = buff->data[2*((j+shift)%buff->npts) + i*2*buff->npts];
      newdata[2*j+1+i*2*buff->npts] = buff->data[2*((j+shift)%buff->npts) +1+ i*2*buff->npts];
    }
  }


  
      // don't do the /2 thing.  Not doing it only ever messes with the baseline.  Doing it can mess things
      // up worse.
      //      new_data[i*2*buff->npts] /= 2;
      // new_data[i*2*buff->npts+1] /= 2;
    //    four1(&buff->data[i*2*buff->npts]-1,buff->npts,-1);
  fftw_execute(plan1); 
  // unscramble the data in frequency order.
  if (((buff->flags & FT_FLAG) == 0 && gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.symm_check))) || ((buff->flags & FT_FLAG) )){
    shift = buff->npts/2 + buff->npts %2;
    for(i=0;i<buff->npts2;i++){
      for(j=0;j<buff->npts;j++){
	buff->data[2*((j+shift)%buff->npts)+i*2*buff->npts] = newdata[2*j + i*2*buff->npts]/scale;
	buff->data[2*((j+shift)%buff->npts)+1+i*2*buff->npts] = newdata[2*j +1+ i*2*buff->npts]/scale;
      }
    }
  }
  else //just scale
    for(i=0;i<buff->npts2;i++){
      for(j=0;j<buff->npts;j++){
	buff->data[2*j+i*2*buff->npts] = newdata[2*j + i*2*buff->npts]/scale;
	buff->data[2*j+1+i*2*buff->npts] = newdata[2*j +1+ i*2*buff->npts]/scale;
      }
    }

    // if forward, not symm, and there is a group delay, then wind the group delay phase shift
  if ( (buff->flags & FT_FLAG) && !gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.symm_check)) && group_delay > 0){
    for (j=0;j<buff->npts;j++){
      for (i=0;i<buff->npts2;i++){
	double angle = 2*M_PI*(j-buff->npts/2)*group_delay/buff->npts;
	swap = buff->data[2*j + i*2*buff->npts] * cos(angle) - buff->data[2*j+1 + i*2*buff->npts]*sin(angle);
	buff->data[2*j+1 +i*2*buff->npts] = buff->data[2*j + 1+ i*2*buff->npts] * cos(angle) + buff->data[2*j + i*2*buff->npts] * sin(angle);
	buff->data[2*j + i*2*buff->npts] = swap;
      }
    }
  }
  
  save_wisdom();
  fftw_destroy_plan(plan1);
  g_free(newdata);
  cursor_normal(buff);
  return TRUE;
  
}




gint do_flip_ring_up(GtkWidget *widget, double *unused)
{
  dbuff *buff;
  double group_delay = 0;
  int i,j, gd;
  
  // flip the filter ring-up over
  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  pfetch_float(&buff->param_set, "group_delay", &group_delay,  0);
  gd = round(group_delay);
  if (group_delay <= 0) return 0;
  for(i=0;i<buff->npts2;i++){
    for (j=0;j<gd;j++){
      buff->data[2*(gd*2-j)+i*2*buff->npts] +=
	buff->data[2*j + i*2*buff->npts];
      buff->data[2*(gd*2-j)+i*2*buff->npts+1] -=
	buff->data[2*j + i*2*buff->npts+1];
      buff->data[2*j + i*2*buff->npts]=0;
      buff->data[2*j + i*2*buff->npts+1]=0;
    }
  }
  return 1;
}

gint do_flip_ring_up_and_display( GtkWidget *widget, double *unused )
{
  dbuff *buff;
  gint result;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  result = do_flip_ring_up( widget, unused );

  draw_canvas( buff );
   return result;
}

gint do_bft_hide(GtkWidget *widget, double *unused)

{
  /* do fourier transform in one dimension */
  dbuff *buff;
  int i,j;
  //  float spare;
  float scale;
  float *data;

  static fftwf_plan plan1;


  if( widget == NULL ) {
    buff = buffp[ upload_buff ];
    // fprintf(stderr,"do_ft- on buffer %i\n",upload_buff );
  }
  else {
    buff = buffp[ current ];
    // fprintf(stderr,"do_ft- on buffer %i\n",current );
  }
  if (buff == NULL){
    popup_msg("do_bft panic! buff is null!",TRUE);
    return 0;
  }


  buff->flags ^= FT_FLAG; //toggle the ft flag
  
  printf("TESTME. Bruker FT not tested since switch to fftw\n");
  cursor_busy(buff);
  load_wisdom();
  data = g_malloc(sizeof(float)* buff->npts*2*2);
  // plan the FT:
  plan1 = fftwf_plan_dft_1d(buff->npts*2,(fftwf_complex *) data,
			    (fftwf_complex *) data,FFTW_FORWARD,FFTW_MEASURE);

  if (buff->flags & FT_FLAG){
    //    fprintf(stderr,"FT_FLAG is true\n");
    scale = 2.0;
  }
  else{
    //    fprintf(stderr,"FT_FLAG is false\n");
    scale = buff->npts/2.0;
  }

  for(i=0;i<buff->npts2;i++){
    /* do ft for each 1d spectrum */
    // copy data out 
    memset(data,0,4*buff->npts*sizeof(float));

    for (j=0;j<buff->npts;j++){
      data[j*4]=buff->data[i*2*buff->npts + j*2];
      data[j*4+3] = -1.0*buff->data[i*2*buff->npts +j*2+1];
    }
    fftwf_execute(plan1);
    //four1(data-1,buff->npts*2,-1);
    for(j=0;j<buff->npts;j++){
      buff->data[i*2*buff->npts+j*2]=data[2*j+buff->npts]/scale;
      buff->data[i*2*buff->npts+j*2+1]=data[2*j+buff->npts+1]/scale;
    }
    /*    for(j=0;j<buff->npts;j++){
      spare=buff->data[j+i*2*buff->npts]/scale;
      buff->data[j+i*2*buff->npts]=
	buff->data[j+i*2*buff->npts+buff->npts]/scale;
      buff->data[j+i*2*buff->npts+buff->npts]=spare;
      }*/
  }
  
  
  save_wisdom();
  fftwf_destroy_plan(plan1);
  g_free(data);
  cursor_normal(buff);
  return TRUE;
  
}






gint do_exp_mult_and_display( GtkWidget *widget, GtkAdjustment *adj )
{
  dbuff *buff;
  gint result;
  double val;
  val = gtk_adjustment_get_value(adj);


  result = do_exp_mult( widget, &val );

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  draw_canvas( buff );
  return result;
}

gint do_exp_mult( GtkWidget* widget, double* val )
{
  int i,j,i2;
  float factor;
  dbuff* buff;
  char is_symm;
  double group_delay=0;
  factor = (float) *val;

  // fprintf(stderr, "doing exp_mult by %f\n", factor );

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];

  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_exp_mult panic! buff is null!",TRUE);
    return 0;
  }

  is_symm = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.symm_check));
  pfetch_float(&buff->param_set, "group_delay", &group_delay,  0);

  // this is repeated in the fitting routine!

  for( j=0; j<buff->npts2; j++ )
    for( i=0; i<buff->npts; i++ ){
      if (is_symm){
	i2 = abs(i-buff->npts/2-(buff->npts%2)); //handle odd and even
      }
      else if (group_delay > 0)
	i2 = abs(i-round(group_delay));
      else i2 = i;
      
      buff->data[2*i+j*2*buff->npts] *= exp( -1.0 * factor * i2 * buff->param_set.dwell/1000000 * M_PI ); 
      buff->data[2*i+1+j*2*buff->npts] *= exp( -1.0 * factor * i2 * buff->param_set.dwell/1000000 * M_PI ); 
    }
  return 0;
}


gint do_gaussian_mult_and_display( GtkWidget *widget, GtkAdjustment *adj )
{
  dbuff *buff;
  gint result;
  double val;
  val = gtk_adjustment_get_value(adj);

  result = do_gaussian_mult( widget, &val );

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  draw_canvas( buff );
  return result;
}

gint do_gaussian_mult( GtkWidget* widget, double * val)

{
  int i,j,i2;
  float factor;
  float temp;
  dbuff* buff;
  double group_delay=0;
  char is_symm,sign=1;

  factor = *val;
  if (factor < 0) sign = -1;
  // fprintf(stderr, "doing gaussian mult by %f\n",factor );

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_gaussian_mult panic! buff is null!",TRUE);
    return 0;
  }
  // this is repeated in the fitting routine!
  is_symm = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.symm_check));
  pfetch_float(&buff->param_set, "group_delay", &group_delay,  0);

  for( j=0; j<buff->npts2; j++ )
    for( i=0; i<buff->npts; i++ ) {
      if (is_symm){
	i2 = abs(i-buff->npts/2-(buff->npts%2)); // handle odd and even
      }
      else if (group_delay > 0)
	i2 = abs(i-round(group_delay));
      else i2 = i;

      temp = i2*buff->param_set.dwell/1000000 * M_PI * factor / 1.6651;
      buff->data[2*i+j*2*buff->npts] *= exp( -1 * temp * temp *sign);
      buff->data[2*i+1+j*2*buff->npts] *= exp( -1 * temp * temp*sign);
    }

  return 0;
}

gint do_zero_fill(GtkWidget * widget,double *val)
{
  int new_npts,old_npts,acq_points;
  float factor;
  dbuff *buff;
  double ls;

  factor = *val;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_zero_fill panic! buff is null!",TRUE);
    return 0;
  }

  /* zero fill will always round up to the next power of 2
     so you can zero fill with factor of 1 to get next size up */
  // not anymore! powers of two no longer special.
  //  temp=buff->npts*factor;
  //  new_npts = pow(2,ceil(log(temp-0.001)/log(2.0)));
  new_npts = buff->npts*factor;

  old_npts=buff->npts;
  acq_points=buff->acq_npts;
  
  if (new_npts ==old_npts) return 0;  
  if (old_npts>= MAX_DATA_NPTS) return 0;

  cursor_busy(buff);
  //let's try  something else
  buff_resize(buff,new_npts,buff->npts2);

  // ok, if we're here because the user pressed a button or we're actively running, then
  // change the npts.  
  // The other possibility is from an automatic upload_and_process while we're not focussed on the current buff.
  if (widget !=NULL || upload_buff == current) update_npts(new_npts); 


  // zero out the new stuff, but don't trust the new_npts

  new_npts = buff->npts;

  // reset this since the update_npts call may have messed it up.
  buff->acq_npts=acq_points;
  /* shouldn't have to do this anymore as buff resize zeros stuff out by itself
  for(j=0;j<buff->npts2;j++)
    for(i=old_npts;i<new_npts;i++){
      buff->data[2*i+2*j*new_npts] = 0.;
      buff->data[2*i+1+2*j*new_npts] = 0.;
      } */  


  // if we're symm, we want to add before and after the data set, not all at the end.
  // if the npts was odd, and now its even, how to keep the middle in the middle?
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.symm_check))){
    ls = -(buff->npts-old_npts)/2;
    do_left_shift((GtkWidget *)buff,(double *) &ls);
  }
      
  cursor_normal(buff);
  			    
  // fprintf(stderr,"do_zero_fill: got factor: %f, rounding to: %i\n",factor,new_npts);

 return 0;
}

gint do_zero_fill_and_display(GtkWidget * widget,GtkAdjustment *adj)
{
  dbuff *buff;
  gint result;
  gint old_npts;
  double val;
  val = gtk_adjustment_get_value(adj);

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  old_npts=buff->npts;
  result = do_zero_fill( widget, &val );

  if (old_npts != buff->npts)
    draw_canvas( buff );

  return result;

}

gint do_left_shift(GtkWidget * widget,double *val)
{
  int i,j,shift;
  dbuff* buff;

  shift = (int) *val;
  

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_left_shift panic! buff is null!",TRUE);
    return 0;
  }

  // fprintf(stderr,"left_shift: shift is: %i\n",shift);
  if (shift ==0) return 0;
  if (shift > 0){
    for( j=0; j<buff->npts2; j++ ){
      for( i=shift; i<buff->npts; i++ ) {
	buff->data[2*(i-shift)+j*2*buff->npts]=
	  buff->data[2*i+j*2*buff->npts];
	buff->data[2*(i-shift)+1+j*2*buff->npts]=
	  buff->data[2*i+1+j*2*buff->npts];
      }
      for(i=buff->npts-shift;i<buff->npts;i++){
	buff->data[2*i+j*2*buff->npts]=0.;
	buff->data[2*i+1+j*2*buff->npts]=0.;
      }

      /* do circular shift for now */
      /*
      float temp_buff[shift*2];
      for (i=0;i<shift*2;i++)
	temp_buff[i]=buff->data[i+j*2*buff->npts];
      for( i=shift; i<buff->npts; i++ ) {
	buff->data[2*(i-shift)+j*2*buff->npts]=
	  buff->data[2*i+j*2*buff->npts];
	buff->data[2*(i-shift)+1+j*2*buff->npts]=
	  buff->data[2*i+1+j*2*buff->npts];
      }
      for(i=0;i<shift*2;i++)
	buff->data[i+2*(buff->npts-shift) +j*2*buff->npts] = temp_buff[i];
      */
    }
  }
  else
    for( j=0; j<buff->npts2; j++ ){
      for( i=buff->npts+shift-1;i>=0; i-- ) {
	buff->data[2*(i-shift)+j*2*buff->npts]=
	  buff->data[2*i+j*2*buff->npts];
	buff->data[2*(i-shift)+1+j*2*buff->npts]=
	  buff->data[2*i+1+j*2*buff->npts];
      }
      for(i=0;i<-shift;i++){
	buff->data[2*i+j*2*buff->npts]=0.;
	buff->data[2*i+1+j*2*buff->npts]=0.;
      }
    }

  return 0;


  }

gint do_left_shift_and_display(GtkWidget * widget,GtkAdjustment *adj)
{
  dbuff *buff;
  gint result;
  double val;
  val = gtk_adjustment_get_value(adj);
  //  fprintf(stderr,"in do_left_shift_and_display with val = %lf\n",*val);
  result = do_left_shift( widget, &val );

  if( widget == NULL ) {
    fprintf(stderr,"widget is null\n");
    buff = buffp[ upload_buff ];
  }
  else 
    buff = buffp[ current ];

  draw_canvas( buff );
  return result;

  }

gint do_left_shift_2d(GtkWidget * widget,double *val)
{
  int i,j,shift;
  dbuff* buff;

  shift = (int) *val;
  

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_left_shift panic! buff is null!",TRUE);
    return 0;
  }

  // fprintf(stderr,"left_shift: shift is: %i\n",shift);
  if (shift ==0) return 0;
  if (shift*(1+buff->is_hyper) >= buff->npts2 ||
      shift*(1+buff->is_hyper) <= -1*buff->npts2 ){
    popup_msg("Left shift 2D too big!",TRUE);
    return 0;
  }
  if (shift > 0){ // left shifting - add zeros in at end.
    for(i=0;i<2*buff->npts;i++){
      for(j=shift*(1+buff->is_hyper);j<buff->npts2;j += (1+buff->is_hyper)){
	//	printf("dest is %i\n",j-shift*(1+buff->is_hyper));
	buff->data[i+(j-shift*(1+buff->is_hyper))*2*buff->npts] 
	  = buff->data[i+j*2*buff->npts];
	if (buff->is_hyper)
	  buff->data[i+(j+1-shift*(1+buff->is_hyper))*2*buff->npts] 
	    = buff->data[i+(j+1)*2*buff->npts];
      }
      // add in the zeros at the end
      for(j= buff->npts2-shift*(1+buff->is_hyper) ; j<buff->npts2;j++)
	buff->data[i+j*2*buff->npts] = 0.;
    }
  }
    else{ //right shifting.  resize buffer to keep all the data
      //      buff_resize(buff,buff->npts,buff->npts2-shift*(1+buff->is_hyper));
      for (i=0;i<2*buff->npts;i++){
	for (j=buff->npts2-1;j>=-shift*(1+buff->is_hyper);j--)
	  buff->data[i+j*2*buff->npts] 
	    = buff->data[i+(j+shift*(1+buff->is_hyper))*2*buff->npts];
	
	// add zeros in at beginning
	for(j=0;j<-shift*(1+buff->is_hyper);j++)
	  buff->data[i+j*2*buff->npts]=0.;
			    

      }
    }
	
  return 0;


  }

gint do_left_shift_2d_and_display(GtkWidget * widget,GtkAdjustment *adj)
{
  dbuff *buff;
  gint result;
  double val;

  val = gtk_adjustment_get_value(adj);

  //  fprintf(stderr,"in do_left_shift_and_display with val = %lf\n",*val);
  result = do_left_shift_2d( widget, &val );

  if( widget == NULL ) {
    fprintf(stderr,"widget is nyull\n");
    buff = buffp[ upload_buff ];
  }
  else 
    buff = buffp[ current ];

  draw_canvas( buff );
  return result;

  }


gint do_truncate(GtkWidget * widget,double *val)
{
  dbuff* buff;
  int old_npts,acq_points;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_truncate panic! buff is null!",TRUE);
    return 0;
  }

  old_npts=buff->npts;
  acq_points=buff->acq_npts;
  
  if ((int) *val ==old_npts) return 0;  

  cursor_busy(buff);
  buff_resize(buff,(int) *val,buff->npts2);

  // ok, if we're here because the user pressed a button or we're actively running, then
  // change the npts.  
  // The other possibility is from an automatic upload_and_process while we're not focussed on the current buff.
  if (widget !=NULL || upload_buff == current) update_npts((int) *val); 

  

  // reset this since the update_npts call may have messed it up.
  buff->acq_npts=acq_points;

  return 0;


  }

gint do_truncate_and_display(GtkWidget * widget,GtkAdjustment *adj)
{
  dbuff *buff;
  gint result;
  double val;

  val = gtk_adjustment_get_value(adj);

  result = do_truncate( widget, &val );

  if( widget == NULL ) {
    fprintf(stderr,"widget is nyull\n");
    buff = buffp[ upload_buff ];
  }
  else 
    buff = buffp[ current ];

  draw_canvas( buff );
  return result;

  }

////


gint do_phase_wrapper( GtkWidget* widget, double *unused )

{
  dbuff* buff;
  int i;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_phase_wrapper panic! buff is null!",TRUE);
    return 0;
  }

  if ( ((int)buff->process_data[PH].val & GLOBAL_PHASE_FLAG)==0) {
    // fprintf(stderr, "Setting phase from %f, %f, to %f, %f, (local)\n", buff->phase0_app, buff->phase1_app, buff->phase0, buff->phase1 );

    for( i=0; i<buff->npts2; i++ )
      do_phase( &buff->data[i*2*buff->npts], &buff->data[ i*2*buff->npts ], 
		buff->phase0 - buff->phase0_app, buff->phase1 - buff->phase1_app, 
		buff->npts );
    buff->phase0_app = buff->phase0;
    buff->phase1_app = buff->phase1;
  }
  else {
    // fprintf(stderr, "Setting phase from %f, %f, to %f, %f, (global)\n", buff->phase0_app, buff->phase1_app, phase0, phase1 );
    for( i=0; i<buff->npts2; i++ )
      do_phase( &buff->data[i*2*buff->npts], &buff->data[ i*2*buff->npts ], 
		phase0 - buff->phase0_app, phase1 - buff->phase1_app, buff->npts );
    buff->phase0_app = phase0;
    buff->phase1_app = phase1;
  }

  return 0;

}

gint do_phase_and_display_wrapper( GtkWidget* widget, double *unused )

{
  dbuff *buff;
  gint result;

  result = do_phase_wrapper( widget,unused );

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  draw_canvas( buff );
  return result;
}


///////
gint do_phase_2d_wrapper( GtkWidget* widget, double *unused )

{
  dbuff* buff;
  int i,j;
  float *pdat;
  float dp0,dp1;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_phase_wrapper panic! buff is null!",TRUE);
    return 0;
  }

  if(buff->is_hyper == 0 && gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.true_complex)) == 0){
    popup_msg("Can't phase 2d on non-complex data",TRUE);
    return 0;
  }

  if(buff->npts2 %2 ==1 && buff->is_hyper){
    popup_msg("Can't phase hypercomplex 2d on odd number of npts2",TRUE);
    return 0;
  }

  if (buff->is_hyper) pdat=g_malloc(buff->npts2*sizeof(float));
  else
    pdat=g_malloc(buff->npts2*2*sizeof(float));
  // borrow 1-d global/local flag
  if ( ((int)buff->process_data[PH].val & GLOBAL_PHASE_FLAG)==0) {
    dp0=buff->phase20-buff->phase20_app;
    dp1=buff->phase21-buff->phase21_app;
    buff->phase20_app = buff->phase20;
    buff->phase21_app = buff->phase21;
  }
  else{
    dp0=phase20-buff->phase20_app;
    dp1=phase21-buff->phase21_app;
    buff->phase20_app = phase20;
    buff->phase21_app = phase21;
  }
  if (buff->is_hyper){
    for( i=0; i<2*buff->npts; i++ ){
      for(j=0;j<buff->npts2;j++)
	pdat[j]=buff->data[i+j*2*buff->npts];
      do_phase(pdat,pdat,dp0,dp1,buff->npts2/2);
      for(j=0;j<buff->npts2;j++)
	buff->data[i+2*j*buff->npts] = pdat[j];
    }
  }
  else{// true complex
    for( i=0; i<buff->npts; i++ ){
      for(j=0;j<buff->npts2;j++){
	pdat[2*j]=buff->data[2*i+j*2*buff->npts];
	pdat[2*j+1]=buff->data[2*i+1+j*2*buff->npts];
      }
      do_phase(pdat,pdat,dp0,dp1,buff->npts2);
      for(j=0;j<buff->npts2;j++){
	buff->data[2*i+2*j*buff->npts]=pdat[2*j];
	buff->data[2*i+1+2*j*buff->npts]=pdat[2*j+1];
      }

    }
  }

  g_free(pdat);
  return 0;

}

gint do_phase_2d_and_display_wrapper( GtkWidget* widget, double *unused )

{
  dbuff *buff;
  gint result;

  result = do_phase_2d_wrapper( widget,unused );

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  draw_canvas( buff );
  return result;
}

////////
gint process_button_toggle(GtkWidget *widget, int button )

{
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON (widget))) {

    if( process_button[ button ].adj == NULL )
      active_process_data[button].status = PROCESS_ON;
    else
      active_process_data[ button ].status = SCALABLE_PROCESS_ON;
  }

  else {
    if( process_button[ button ].adj == NULL )
      active_process_data[ button ].status = PROCESS_OFF;
    else
      active_process_data[ button ].status = SCALABLE_PROCESS_OFF;
  }

  return 0;
}


gint process_local_global_toggle(GtkWidget *widget, int button )

{
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON (widget))) {
    active_process_data[PH].val =0;
  }
  else {
    active_process_data[PH].val = GLOBAL_PHASE_FLAG;
    //    fprintf(stderr,"in toggle local/global, setting active to global\n");
  }

  return 0;
}



gint update_active_process_data( GtkAdjustment *adj, int button )

{

  double f;

  if( active_process_data == NULL )
    return -1;

  // fprintf(stderr, "updating active process data\n" );
  
  f = gtk_adjustment_get_value(adj);

  active_process_data[ button ].val = f;

  return 0;
}


GtkWidget* create_process_frame()

{

  char title[UTIL_LEN];
  GtkWidget *table, *frame, *button;
  GSList *group;
  int i;
  long nu;
  GtkWidget *hbox;

  active_process_data = NULL;

  for( i=0; i<MAX_PROCESS_FUNCTIONS; i++ ) {
    process_button[i].func = NULL;
    process_button[i].adj = NULL;
    process_button[i].button = NULL;
  }

  // fprintf(stderr, "creating process frame\n" );

  snprintf(title,UTIL_LEN,"Process");

  /* arguments are homogeneous and spacing */
  
  table = gtk_table_new(12,6, TRUE); /* rows, columns */
 
  /*
   *  This is where all the process buttons are set up
   */ 


  /*
   *  The first Baseline correct
   */

  nu=BC1;
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu,nu+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle), (void*) nu);
  gtk_widget_show(process_button[nu].button);

  button = gtk_button_new_with_label( "Baseline Correct" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_offset_cal_a_and_display), NULL);
  process_button[nu].func = do_offset_cal_a;
  gtk_widget_show(button);

  //

  /*
   * left shift
   */
  nu=LS;
  process_button[nu].adj= (GtkAdjustment *) gtk_adjustment_new( 0, -1000000, 1000000, 1, 2, 0 );
  g_signal_connect (G_OBJECT (process_button[nu].adj), "value_changed", G_CALLBACK (update_active_process_data), (void*) nu);
  button = gtk_spin_button_new(  process_button[nu].adj , 1.00, 0 );
  gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( button ), GTK_UPDATE_IF_VALID );
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu,nu+1);
  gtk_widget_show( button );

  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu,nu+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle), 
      (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "Left Shift" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_left_shift_and_display), process_button[nu].adj );
  process_button[nu].func = do_left_shift;
  gtk_widget_show(button);

  /*
   * truncate 
   */
  nu=TR;
  process_button[nu].adj= (GtkAdjustment *) gtk_adjustment_new( 0, 0, 10000000, 1, 2, 0 );
  g_signal_connect (G_OBJECT (process_button[nu].adj), "value_changed", G_CALLBACK (update_active_process_data), (void*) nu);
  button = gtk_spin_button_new( GTK_ADJUSTMENT(  process_button[nu].adj ), 1.00, 0 );
  gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( button ), GTK_UPDATE_IF_VALID );
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu,nu+1);
  gtk_widget_show( button );

  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu,nu+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle), 
      (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "Truncate" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_truncate_and_display),  process_button[nu].adj  );
  process_button[nu].func = do_truncate;
  gtk_widget_show(button);

  /*
   * cross correlation
   */
  nu=CR;
  // default value actually set in buff.c 
  //  process_button[nu].adj= gtk_adjustment_new( 9, 1, PSRBMAX , 1, 2, 0 );
  process_button[nu].adj= (GtkAdjustment *) gtk_adjustment_new( 9, 1, 4096 , 1, 2, 0 );
  g_signal_connect (G_OBJECT (process_button[nu].adj), "value_changed", G_CALLBACK (update_active_process_data), (void*) nu);
  button = gtk_spin_button_new(  process_button[nu].adj , 1.00,0 );
  gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( button ), GTK_UPDATE_IF_VALID );
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu,nu+1);
  gtk_widget_show( button );
  //  gtk_adjustment_set_value( GTK_ADJUSTMENT( process_button[nu].adj ), 11 );

  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu,nu+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle), (void*) nu);
  gtk_widget_show(process_button[nu].button);

  button = gtk_button_new_with_label( "Cross correlation" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_cross_correlate_and_display), process_button[nu].adj );
  process_button[nu].func = do_cross_correlate;
  gtk_widget_show(button);





  //
  /*
   *  Exp Mult
   */
  nu=EM;
  process_button[nu].adj =  (GtkAdjustment *) gtk_adjustment_new( 0, -1e6, 1e6, 1, 10, 0 );
  g_signal_connect (G_OBJECT (process_button[nu].adj), "value_changed", G_CALLBACK (update_active_process_data), (void*) nu);
  button = gtk_spin_button_new( GTK_ADJUSTMENT( process_button[nu].adj ), 1.00, 2 );
  gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( button ), GTK_UPDATE_IF_VALID );
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu,nu+1);
  gtk_widget_show( button );
   
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu,nu+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle), (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "Exp Mult" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_exp_mult_and_display), process_button[nu].adj  );
  process_button[nu].func = do_exp_mult;
  gtk_widget_show(button);

  /*
   *  Gaussian Mult
   */
  nu=GM;
  process_button[nu].adj= (GtkAdjustment *) gtk_adjustment_new( 0, -1e6, 1e6, 1, 10, 0 );
  g_signal_connect (G_OBJECT (process_button[nu].adj), "value_changed", G_CALLBACK (update_active_process_data), (void*) nu );
  button = gtk_spin_button_new( GTK_ADJUSTMENT(  process_button[nu].adj ), 1.00, 2 );
  gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( button ), GTK_UPDATE_IF_VALID );
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu,nu+1);
  gtk_widget_show( button );

  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu,nu+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle),  (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "Gaussian Mult" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_gaussian_mult_and_display), process_button[nu].adj  );
  process_button[nu].func = do_gaussian_mult;
  gtk_widget_show(button);

  /*
   * Zero fill
   */
  nu=ZF;
  process_button[nu].adj= (GtkAdjustment *) gtk_adjustment_new( 2, 1, 10, 1, 2, 0 );
  // the 2 is the default value but its actually set up in buff init in buff.c.
  g_signal_connect (G_OBJECT (process_button[nu].adj), "value_changed", G_CALLBACK (update_active_process_data), (void*) nu);
  button = gtk_spin_button_new( GTK_ADJUSTMENT(  process_button[nu].adj ), 1.00, 1 );
  gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( button ), GTK_UPDATE_IF_VALID );
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu,nu+1);
  gtk_widget_show( button );

  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu,nu+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle), 
      (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "Zero Fill" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_zero_fill_and_display), process_button[nu].adj  );
  process_button[nu].func = do_zero_fill;
  gtk_widget_show(button);



  /*
   * ZI zero imaginary
   */
  nu=ZI;
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu,nu+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle),  (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "Zero Imag" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_zero_imag_and_display), NULL);
  process_button[nu].func = do_zero_imag;
  gtk_widget_show(button);

  /*
   * FT
   */
  nu=FT;
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu,nu+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle),  (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "FT" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_ft_and_display), NULL);
  process_button[nu].func = do_ft;
  gtk_widget_show(button);


  /*  Flip ring up. */
  // this moves the filter ring-up from negative times to positive times.
  // phase 0 needs to be set "right" first. Not totally clear what right means.
  // Seems to mean that the signals are mostly in phase with a first order phase correction
  // only. After this, can do ft and first order phase correction.
  nu=BFT;
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu,nu+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle),  (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "Flip ring-up" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_flip_ring_up_and_display), NULL);
  process_button[nu].func = do_flip_ring_up;
  gtk_widget_show(button);




  /*
   * Another Baseline correct
   */  
  nu=BC2;
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE( table ), process_button[nu].button,0,1,nu,nu+1);
  g_signal_connect(G_OBJECT( process_button[nu].button ), "toggled", G_CALLBACK( process_button_toggle ),  (void*) nu);
  gtk_widget_show( process_button[nu].button );
  button = gtk_button_new_with_label( "Baseline Correct" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_offset_cal_and_display), NULL);
  process_button[nu].func = do_offset_cal;
  gtk_widget_show(button);

  /*
   *  Phase processing
   */
  nu=PH;
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE( table ), process_button[nu].button,0,1,nu,nu+1);
  g_signal_connect(G_OBJECT( process_button[nu].button ), "toggled", G_CALLBACK( process_button_toggle ),  (void*) nu);
  gtk_widget_show( process_button[nu].button );
  button = gtk_button_new_with_label( "Phase" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK( do_phase_and_display_wrapper ), NULL);
  process_button[nu].func = do_phase_wrapper;
  gtk_widget_show(button);

  /*
   *  Phase radio buttons
   */

  hbox = gtk_hbox_new_wrap(FALSE,1);
  
  button = gtk_radio_button_new_with_label( NULL, "local" );
  gtk_box_pack_start(GTK_BOX(hbox),button,FALSE,FALSE,1);
  g_signal_connect(G_OBJECT(button),"toggled",G_CALLBACK(process_local_global_toggle),
		     (void *) 0);
  gtk_widget_show (button);
  r_local_button = button;

  group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (button) );
  button = gtk_radio_button_new_with_label( group, "global" );
  gtk_box_pack_start(GTK_BOX(hbox),button,FALSE,FALSE,1);

  gtk_table_attach_defaults(GTK_TABLE(table),hbox,2,3,nu,nu+1);
  gtk_widget_show (button);
  gtk_widget_show(hbox);
  r_global_button=button;


//  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu+1,nu+2);

/*
  button = gtk_radio_button_new_with_label( NULL, "local" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"toggled",G_CALLBACK(process_local_global_toggle),
		     (void *) 0);
  gtk_widget_show (button);

  r_local_button = button;

  group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (button) );

  button = gtk_radio_button_new_with_label( group, "global" );
  r_global_button=button;
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu+1,nu+2);
  gtk_widget_show (button);
*/


  gtk_widget_show (table);

  snprintf(title,UTIL_LEN,"Process");

  frame = gtk_frame_new(title);

  gtk_container_set_border_width(GTK_CONTAINER (frame),5);
  //  gtk_widget_set_size_request(frame,800,300);

  gtk_container_add(GTK_CONTAINER(frame),table);
  gtk_widget_show(frame);

  return frame;
}


gint process_data( GtkWidget *widget, gpointer data )
{
  int i;
  process_data_t *p_data;

  //decide which process set to use

  if( widget == NULL ) {
    p_data = acq_process_data;
    // fprintf(stderr, "processing acq data\n" );
  }

  else {
    p_data = active_process_data;
    // fprintf(stderr, "processing active buffer data\n" );
  }

  for( i=0; i<MAX_PROCESS_FUNCTIONS; i++ ) {
    switch( p_data[i].status )
      {
	/*
	 *  Pass *widget to the process functions so they too know whether they are being called
	 *  by a button press or by an automatic processing sequence
	 */

      case PROCESS_ON:
	process_button[i].func( widget, NULL );
	break;

      case SCALABLE_PROCESS_ON:
	process_button[i].func( widget, &p_data[i].val );
	break;

      case PROCESS_OFF:
      case SCALABLE_PROCESS_OFF:
	break;
      default:
	fprintf(stderr, "Bad processing command\n" );
	break;
      }
  }

  // fprintf(stderr, "done processing data\n" );

  //draw canvas if this was a gtk button callback

  if( widget != NULL ) {
    draw_canvas( buffp[current] );
  }
  return 0;
}

void show_process_frame( process_data_t* process_set )
{
  int i;

  // fprintf(stderr, "Showing process frame\n" );

  active_process_data = process_set;

  for( i=0; i<MAX_PROCESS_FUNCTIONS; i++ ) {
    if( process_button[i].button != NULL ) {
      switch( process_set[i].status ) 
	{
	case NO_PROCESS:
	  break;
	  
	case PROCESS_ON:
	case SCALABLE_PROCESS_ON:
	  gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( process_button[i].button ), TRUE ); 
	  break;
	  
	case PROCESS_OFF:
	case SCALABLE_PROCESS_OFF:
	  gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( process_button[i].button ), FALSE ); 
	  break;
    
	default:
	  fprintf(stderr, "process button %i: invalid status is %i\n" ,i,process_set[i].status);
	  break;
	}
    }

    if( process_button[i].adj != NULL )
      gtk_adjustment_set_value( GTK_ADJUSTMENT( process_button[i].adj ), process_set[i].val );

  }

  // finally set the local/global phase flag to the value for this buff.

  if (((int)process_set[PH].val & GLOBAL_PHASE_FLAG) == 0)
    {
      gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON(r_local_button  ), TRUE ); 
    }
  else{
    gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON(r_global_button  ), TRUE ); 
  // fprintf(stderr, "done showing process frame\n" );
  }
  return;
}


gint do_cross_correlate_and_display( GtkWidget *widget, GtkAdjustment *adj )
{
  dbuff *buff;
  gint result;
  double bits;
  bits = gtk_adjustment_get_value(adj);

  result = do_cross_correlate( widget, &bits );

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  draw_canvas( buff );
  return result;
}

float get_chu_seq(int n){

  static int pos=-1;
  float phase;
  int np;
  
  if (n < 0){
    pos=-1;
    return 0.;
  }

  if (pos == -1)
    pos = n/2;

  pos += 1;
  pos = pos %n;
  if (n %2 == 0)
    phase =180.*pos*pos/n;
  else
    phase = 180.*pos*(pos+1)/n;
  np=phase/360.;
  phase=phase-np*360;
  return phase;
}

float get_frank_seq(int n){

  static int pos=-1;
  static int f=0;
  float phase;
  int np;
  if (n < 0) {
    pos=-1;
    f=0;
    return 0.;
  }

  pos += 1;
  if (pos == n){
    f += 1;
    if ( f== n)
      f = 0;
  }
  pos = pos % n;
  phase = 360.*pos*(f+n/2)/n;
  np=phase/360.;
  phase=phase-np*360;
  //  fprintf(stderr,"pos: %i, phase: %f\n",pos,phase);
  return phase;
}

/*

   First version based on taps listed in The Art of Electronics later
   added entries for sequences that require more than two taps.  Got
   the taps out of mseq.m.

mseq.m is:
% (c) Giedrius T. Buracas, SNL-B, Salk Institute
% Register values are taken from: WDT Davies, System Identification
% for self-adaptive control. Wiley-Interscience, 1970
% When using mseq code for design of FMRI experiments, please, cite:
% G.T.Buracas & G.M.Boynton (2002) Efficient Design of Event-Related fMRI
% Experiments Using M-sequences. NeuroImage, 16, 801-813.

*/

char psrb(int bits,int init){
  static char inited = 0;
  static int reg=0; /* the register we're gonna use */
  static int inited_bits=0;
  static int ntaps[PSRBMAX]=    {0,0,2,2,2,2,2,4,2,2,2, 4, 4, 4, 2, 4, 2, 2};
  static int m[3][PSRBMAX] =   {{0,0,2,3,3,5,6,7,5,7,9,11,12,13,14,15,14,11},
				{0,0,0,0,0,0,0,2,0,0,0, 8,10, 8, 0,13, 0, 0},
				{0,0,0,0,0,0,0,1,0,0,0, 6, 9, 4, 0, 4, 0, 0}};
  int i,j;
  char first;

  if (bits < 3 || bits > PSRBMAX){
    printf("psrb error: bits = %i. bailing\n",bits);
    return 0;
  }
  if (inited == 0 && init == 0){
    printf("psrb error.  Not init'ed yet, and init = 0.  Initing anyway\n");
    init = 1;
  }

  if (init == 0 && inited_bits != bits){
    printf("psrb error.  Inited but with wrong number of bits.  Re-initing\n");
    init = 1;
  }

  if(init == 1){
    reg = 0;
    inited_bits= bits;
    for (i=0;i<bits;i++){
      reg += 1 << i;
    }
    inited = 1;
  }

  /* do the shifting: */

  first = (  (reg >> (bits-1)) ^ (reg >> ((m[0][bits-1])-1))  ) & 1;
  for (j=1;j<ntaps[bits-1]-1;j++)
    first = (first ^ (reg >> (m[j][bits-1]-1)) ) & 1;
  reg = (reg << 1) + first;
  return (reg>> ((bits-1))&1)*2-1;

}


gchar psrb_o(int bits,int init){
  // routine that generates a pseudo random bit sequence.  See Horowitz and Hill
  // pg 657 or CM-I-98

  static char inited = 0;
  static char mreg[PSRBMAX];
  static int inited_bits=0;
  static int m[PSRBMAX] = {0,0,2,3,3,5,6,0,5,7,9,0,0,0,14,0,14,11};
  static int position;

  int i;
  char first;

  if (bits < 3 || bits > PSRBMAX || bits == 8 || bits ==12 || bits == 13 || bits == 14 || bits == 16){
    fprintf(stderr,"psrb error: bits = %i. bailing\n",bits);
    return 0;
  }

			   

  if (inited == 0 && init == 0){
    fprintf(stderr,"psrb error.  Not init'ed yet, and init = 0.  Initing anyway\n");
    init = 1;
  }


  if (init == 0 && inited_bits != bits){
    fprintf(stderr,"psrb error.  Inited but with wrong number of bits.  Re-initing\n");
    init = 1;
  }
  

  if(init == 1){
    inited_bits= bits;
    for (i=0;i<PSRBMAX;i++)
      mreg[i]=1;
    inited = 1;
    position = 0;
    
  }
  /*
  if (position == len[bits-1]){
    position = 0;
    return -1;
  } // makes it so our sequence has an equal number of +1 and -1, 
  // and has a length of a power of 2. - I think this causes problems actually
  */
  position += 1;

    
  // do the shifting:


  first = mreg[bits-1]^mreg[m[bits-1]-1];
  for(i= bits-1;i>0;i--)
    mreg[i]=mreg[i-1];
  mreg[0] = first;
  return mreg[ bits-1]*2-1;


}
gint do_cross_correlate( GtkWidget *widget, double *bits ){
  // figure out if we should be doing an mlbs, frank or chu correlation, and do the right one.
  // we do think by looking for 'frank' or 'chu' in the pulse program name

  dbuff *buff;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_cross_correlate panic! buff is null!",TRUE);
    return 0;
  }
#ifndef MINGW
  if (strcasestr(buff->param_set.exec_path,"frank")){
    printf("doing frank cross correlate\n");
    return do_cross_correlate_frank(widget,bits);
  }
  else
    if (strcasestr(buff->param_set.exec_path,"chu")){
    printf("doing frank cross correlate\n");
    return do_cross_correlate_chu(widget,bits);
    }
    else{
      printf("doing mlbs cross correlate\n");
      return do_cross_correlate_mlbs(widget,bits);
    }
      
#endif


}


gint do_cross_correlate_mlbs( GtkWidget *widget, double *bits )
// this is the new mlbs version
{
  static int len[PSRBMAX] = {0,0,7,15,31,63,127,255,511,1023,2047,4095,8191,16383,32767,65535,131071,262143};

  int i, j,k;
  dbuff *buff;
  double *new_data;
  char *mreg;
  float avg1=0.,avg2=0.;
  int num_seq;
  int num_bits;
  int xsize;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_cross_correlate panic! buff is null!",TRUE);
    return 0;
  }
  
  num_bits = (int) *bits;

  if (len[num_bits-1] == 0 ){
    popup_msg("Cross Correlate: invalid number of bits",TRUE);
    return -1;
  }

  num_seq = (int) buff->npts/len[ num_bits-1 ];
  xsize = num_seq*len[ num_bits - 1 ];
  printf("mlbs correlate: len: %i, xsize: %i\n",len[num_bits-1],xsize);

  if (buff->npts < len[ num_bits-1 ]){
    popup_msg("do_cross_correlate: too few points for mlbs\n",TRUE);
    return 0;
  }
  new_data = g_malloc(sizeof(double) * len[num_bits-1]);
  //reals are stored in: buff->data[2*i+j*2*buff->npts]
  //imag in:             buff->data[2*i+1+j*2*buff->npts]

  // array for storing bit coefficients
  mreg = g_malloc(sizeof(char) * xsize);


  //  fprintf(stderr,"in do_cross_correlate, bits: %i\n",(int) *bits);

  mreg[0] = psrb(num_bits,1);
  for (i=1;i<xsize;i++)
    mreg[i]=psrb(num_bits,0);

  for( j=0; j<buff->npts2; j++ ){  // j loops through the 2d records

    // do the reals:
    //    for(i=0;i< xsize ;i++) 
    //      new_data[i] = buff->data[2*(i%buff->npts)+j*2*buff->npts]/xsize;

    // do the cross-correlation:
    for (i = 0 ; i < len[num_bits-1] ; i++){ 
      new_data[i]=0.;
      for( k = 0 ; k < xsize ; k++ ){
	new_data[i] += mreg[k] *buff->data[2*((i+k)%xsize)+j*2*buff->npts];
      }
    }
    for(i=0;i< len[num_bits-1] ;i++) 
      buff->data[2*(i%buff->npts)+j*2*buff->npts] = new_data[i]/xsize ;
    for (i=len[num_bits-1];i<buff->npts;i++)
      buff->data[2*i+j*2*buff->npts]= 0.;

    
    // now the imag:
    // do the cross-correlation:
    for (i=0;i< len[num_bits-1];i++){ 
      new_data[i]=0.;
      for(k=0;k<xsize;k++){
	new_data[i] += mreg[k] *buff->data[2*((i+k)%xsize)+1+j*2*buff->npts];
      }
    }
    for(i=0;i<len[num_bits-1];i++) 
      buff->data[2*(i%buff->npts)+j*2*buff->npts+1] = new_data[i]/xsize;
    for (i=len[num_bits-1];i<buff->npts;i++)
      buff->data[2*i+j*2*buff->npts+1]=0.;
 
  }
  fprintf(stderr,"cross_correlate: avg1/n %f 2: %f\n",avg1/len[num_bits-1],avg2/len[num_bits-1]);
  fprintf(stderr,"cross_correlate: len[num_bits]: %i\n",len[num_bits-1]);

  buff_resize(buff,len[num_bits-1],buff->npts2);
  if (widget !=NULL || upload_buff == current) update_npts(len[num_bits-1]); 
  g_free(new_data);
  g_free(mreg);
  return 0;
}

// new version for single pass through.
gint do_cross_correlate_frank( GtkWidget *widget, double *bits ){

  int i, j,k;
  dbuff *buff;
  float *new_data;
  float *mreg,pha;
  //  int num_seq;
  int xsize; // the size of the data set we'll be cross correlating.

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_cross_correlate panic! buff is null!",TRUE);
    return 0;
  }
  
  // for frank, *bits is the sqrt of the length of the sequence.
  
  if (buff->npts < (int) (*bits * *bits) ){
    popup_msg("do_cross_correlate: too few points\n",TRUE);
    return 0;
  }
  //  num_seq = (int) buff->npts/ (*bits * *bits); // how many sequences we actually will use in the correlation.

  //  xsize = (int)(*bits* *bits)*num_seq;  // how many points we'll use
  xsize = (int)(*bits* *bits);  // length of the sequence
    
  new_data = g_malloc(sizeof(float) * buff->npts*2);
  //reals are stored in: buff->data[2*i+j*2*buff->npts]
  //imag in:             buff->data[2*i+1+j*2*buff->npts]

  // array for storing bit coefficients
  mreg = g_malloc(sizeof(float) * xsize*2);


  //  fprintf(stderr,"in do_cross_correlate, bits: %i\n",(int) *bits);
  get_frank_seq(-1); // reset it.
  for (i=0;i<xsize;i++){
    pha  = get_frank_seq((int) *bits);
    //    printf("phase: %f\n",pha);
    mreg[2*i]=cos(M_PI*pha/180.);
    mreg[2*i+1]=sin(M_PI*pha/180.);
  }
  for( j=0; j<buff->npts2; j++ ){  // j loops through the 2d records

    // copy the data to new_data
    for(i=0;i< buff->npts ;i++) {
      new_data[2*i] = buff->data[2*i+j*2*buff->npts];
      new_data[2*i+1] = buff->data[2*i+j*2*buff->npts+1];
    }

    // do the cross-correlation:
    for (i = 0 ; i < xsize ; i++){ 
      buff->data[2*i+j*2*buff->npts]=0.;
      buff->data[2*i+j*2*buff->npts+1]=0.;
      for( k = 0 ; k < 2*xsize ; k++ ){
	buff->data[2*i+j*2*buff->npts] += mreg[2*(k%xsize)]*new_data[2*((k+i)%(2*xsize))]-mreg[2*(k%xsize)+1]*new_data[2*((k+i)%(2*xsize))+1];
	buff->data[2*i+j*2*buff->npts+1] += mreg[2*(k%xsize)]*new_data[2*((k+i)%(2*xsize))+1]+mreg[2*(k%xsize)+1]*new_data[2*((k+i)%(2*xsize))];
      }

    }
    //    for (i= (int)(*bits* *bits);i<buff->npts;i++){
    //      buff->data[2*i+j*2*buff->npts]= 0.;
    //      buff->data[2*i+j*2*buff->npts+1]= 0.;
    //    }


  }

  buff_resize(buff,xsize, buff->npts2);
  if (widget !=NULL || upload_buff == current) update_npts(buff->npts); 

  g_free(mreg);
  g_free(new_data);
  return 0;
}
// new version for single pass through.
gint do_cross_correlate_chu( GtkWidget *widget, double *bits ){

  int i, j,k;
  dbuff *buff;
  float *new_data;
  float *mreg,pha;
  int xsize; // the size of the data set we'll be cross correlating.

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_cross_correlate panic! buff is null!",TRUE);
    return 0;
  }
  
  // for chu, *bits is the length of the sequence
  
  if (buff->npts < (int) (*bits *2) ){
    popup_msg("do_cross_correlate: too few points\n",TRUE);
    return 0;
  }

  xsize = (int)(*bits);  // length of the sequence
    
  new_data = g_malloc(sizeof(float) * buff->npts*2);
  //reals are stored in: buff->data[2*i+j*2*buff->npts]
  //imag in:             buff->data[2*i+1+j*2*buff->npts]

  // array for storing bit coefficients
  mreg = g_malloc(sizeof(float) * xsize*2);


  //  fprintf(stderr,"in do_cross_correlate, bits: %i\n",(int) *bits);
  get_chu_seq(-1); // reset it.
  for (i=0;i<xsize;i++){
    pha  = get_chu_seq((int) *bits);
    //    printf("phase: %f\n",pha);
    mreg[2*i]=cos(M_PI*pha/180.);
    mreg[2*i+1]=sin(M_PI*pha/180.);
  }
  for( j=0; j<buff->npts2; j++ ){  // j loops through the 2d records

    // copy the data to new_data
    for(i=0;i< buff->npts ;i++) {
      new_data[2*i] = buff->data[2*i+j*2*buff->npts];
      new_data[2*i+1] = buff->data[2*i+j*2*buff->npts+1];
    }

    // do the cross-correlation:
    for (i = 0 ; i < xsize ; i++){ 
      buff->data[2*i+j*2*buff->npts]=0.;
      buff->data[2*i+j*2*buff->npts+1]=0.;
      for( k = 0 ; k < 2*xsize ; k++ ){
	buff->data[2*i+j*2*buff->npts] += mreg[2*(k%xsize)]*new_data[2*((k+i)%(2*xsize))]-mreg[2*(k%xsize)+1]*new_data[2*((k+i)%(2*xsize))+1];
	buff->data[2*i+j*2*buff->npts+1] += mreg[2*(k%xsize)]*new_data[2*((k+i)%(2*xsize))+1]+mreg[2*(k%xsize)+1]*new_data[2*((k+i)%(2*xsize))];
      }

    }
    //    for (i= (int)(*bits* *bits);i<buff->npts;i++){
    //      buff->data[2*i+j*2*buff->npts]= 0.;
    //      buff->data[2*i+j*2*buff->npts+1]= 0.;
    //    }


  }

  buff_resize(buff,xsize, buff->npts2);
  if (widget !=NULL || upload_buff == current) update_npts(buff->npts); 

  g_free(mreg);
  g_free(new_data);
  return 0;
}

/* original for two passes with small pulses
gint do_cross_correlate_frank( GtkWidget *widget, double *bits ){
// this is the frank version


  int i, j,k;
  dbuff *buff;
  float *new_data;
  float *mreg,pha;
  int num_seq;
  int xsize; // the size of the data set we'll be cross correlating.

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_cross_correlate panic! buff is null!",TRUE);
    return 0;
  }
  
  // for frank, *bits is the sqrt of the length of the sequence.
  
  if (buff->npts < (int) (*bits * *bits) ){
    popup_msg("do_cross_correlate: too few points\n",TRUE);
    return 0;
  }
  num_seq = (int) buff->npts/ (*bits * *bits); // how many sequences we actually will use in the correlation.

  xsize = (int)(*bits* *bits)*num_seq;  // how many points we'll use
    
  new_data = g_malloc(sizeof(float) * xsize*2);
  //reals are stored in: buff->data[2*i+j*2*buff->npts]
  //imag in:             buff->data[2*i+1+j*2*buff->npts]

  // array for storing bit coefficients
  mreg = g_malloc(sizeof(float) * xsize*2);


  //  fprintf(stderr,"in do_cross_correlate, bits: %i\n",(int) *bits);
  get_frank_seq(-1); // reset it.
  for (i=0;i<xsize;i++){
    pha  = get_frank_seq((int) *bits);
    //    printf("phase: %f\n",pha);
    mreg[2*i]=cos(M_PI*pha/180.);
    mreg[2*i+1]=sin(M_PI*pha/180.);
  }
  for( j=0; j<buff->npts2; j++ ){  // j loops through the 2d records

    // copy the data to new_data
    for(i=0;i< xsize ;i++) {
      new_data[2*i] = buff->data[2*(i%buff->npts)+j*2*buff->npts];
      new_data[2*i+1] = buff->data[2*(i%buff->npts)+j*2*buff->npts+1];
    }
    printf("in frank cross correlate.  Building data set of %i points, using %i points\n",(int)( *bits * *bits),xsize);
    // do the cross-correlation:
    for (i = 0 ; i < (int)(*bits* *bits) ; i++){ 
      buff->data[2*i+j*2*buff->npts]=0.;
      buff->data[2*i+j*2*buff->npts+1]=0.;      
      for( k = 0 ; k < xsize ; k++ ){
	buff->data[2*i+j*2*buff->npts] += mreg[2*k]*new_data[2*((k+i)%xsize)]-mreg[2*k+1]*new_data[2*((k+i)%xsize)+1];
	buff->data[2*i+j*2*buff->npts+1] += mreg[2*k]*new_data[2*((k+i)%xsize)+1]+mreg[2*k+1]*new_data[2*((k+i)%xsize)];
      }

    }
    for (i= (int)(*bits* *bits);i<buff->npts;i++){
      buff->data[2*i+j*2*buff->npts]= 0.;
      buff->data[2*i+j*2*buff->npts+1]= 0.;
    }


  }

  buff_resize(buff,(int) (*bits* *bits),buff->npts2);
  if (widget !=NULL || upload_buff == current) update_npts((int)(*bits* *bits)); 

  g_free(new_data);
  g_free(mreg);
  return 0;
}
*/


/*
gint do_cross_correlate_chu( GtkWidget *widget, double *bits ){
// this is the chu version


  int i, j,k;
  dbuff *buff;
  float *new_data;
  float *mreg,pha;
  int num_seq;
  int xsize; // the size of the data set we'll be cross correlating.

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_cross_correlate panic! buff is null!",TRUE);
    return 0;
  }
  
  // for chu, *bits is the length of the sequence.
  
  if (buff->npts < (int) *bits ){
    popup_msg("do_cross_correlate: too few points\n",TRUE);
    return 0;
  }
  num_seq = (int) buff->npts/ *bits ; // how many sequences we actually will use in the correlation.

  xsize = *bits*num_seq;  // how many points we'll use
    
  new_data = g_malloc(sizeof(float) * xsize*2);
  //reals are stored in: buff->data[2*i+j*2*buff->npts]
  //imag in:             buff->data[2*i+1+j*2*buff->npts]

  // array for storing bit coefficients
  mreg = g_malloc(sizeof(float) * xsize*2);


  //  fprintf(stderr,"in do_cross_correlate, bits: %i\n",(int) *bits);
  get_chu_seq(-1); // reset it.
  for (i=0;i<xsize;i++){
    pha  = get_chu_seq((int) *bits);
    //    printf("phase: %f\n",pha);
    mreg[2*i]=cos(M_PI*pha/180.);
    mreg[2*i+1]=sin(M_PI*pha/180.);
  }
  for( j=0; j<buff->npts2; j++ ){  // j loops through the 2d records

    // copy the data to new_data
    for(i=0;i< xsize ;i++) {
      new_data[2*i] = buff->data[2*(i%buff->npts)+j*2*buff->npts];
      new_data[2*i+1] = buff->data[2*(i%buff->npts)+j*2*buff->npts+1];
    }
    printf("in chu cross correlate.  Building data set of %i points, using %i points\n",(int) *bits,xsize);
    // do the cross-correlation:
    for (i = 0 ; i < *bits ; i++){ 
      buff->data[2*i+j*2*buff->npts]=0.;
      buff->data[2*i+j*2*buff->npts+1]=0.;      
      for( k = 0 ; k < xsize ; k++ ){
	buff->data[2*i+j*2*buff->npts] += mreg[2*k]*new_data[2*((k+i)%xsize)]-mreg[2*k+1]*new_data[2*((k+i)%xsize)+1];
	buff->data[2*i+j*2*buff->npts+1] += mreg[2*k]*new_data[2*((k+i)%xsize)+1]+mreg[2*k+1]*new_data[2*((k+i)%xsize)];
      }

    }
    for (i= *bits;i<buff->npts;i++){
      buff->data[2*i+j*2*buff->npts]= 0.;
      buff->data[2*i+j*2*buff->npts+1]= 0.;
    }


  }

  buff_resize(buff,*bits,buff->npts2);
  if (widget !=NULL || upload_buff == current) update_npts(*bits); 

  g_free(new_data);
  g_free(mreg);
  return 0;
}
*/

GtkWidget* create_process_frame_2d()

{

  char title[UTIL_LEN];
  GtkWidget *table, *frame, *button;
  //  GSList *group;
  //  int i;
  long nu;


  /* already done in create 1d 

  active_process_data = NULL;

  for( i=0; i<MAX_PROCESS_FUNCTIONS; i++ ) {
    process_button[i].func = NULL;
    process_button[i].adj = NULL;
    process_button[i].button = NULL;
  }

  */


  snprintf(title,UTIL_LEN,"Process 2D");

  /* arguments are homogeneous and spacing */
  
  table = gtk_table_new(12,6, TRUE); /* rows, columns */
 
  /*
   *  This is where all the process buttons are set up
   */ 


  /* first BC in 2nd D */


  nu=BC2D1;
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE( table ), process_button[nu].button,0,1,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT( process_button[nu].button ), "toggled", G_CALLBACK( process_button_toggle ),  (void*) nu);
  gtk_widget_show( process_button[nu].button );
  button = gtk_button_new_with_label( "Baseline Correct" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_offset_cal_2D_a_and_display), NULL);
  process_button[nu].func = do_offset_cal_2D;
  gtk_widget_show(button);


  /*
   * Left shift 2d
   */
  nu=LS2D;
  process_button[nu].adj= (GtkAdjustment *) gtk_adjustment_new( 0, -10000, 10000, 1, 2, 0 );
  g_signal_connect (G_OBJECT (process_button[nu].adj), "value_changed", G_CALLBACK (update_active_process_data), (void*) nu);
  button = gtk_spin_button_new( GTK_ADJUSTMENT(  process_button[nu].adj ), 1.00, 0 );
  gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( button ), GTK_UPDATE_IF_VALID );
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu-P2D,nu-P2D+1);
  gtk_widget_show( button );

  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle), 
      (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "Left shift 2D" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_left_shift_2d_and_display), process_button[nu].adj  );
  process_button[nu].func = do_left_shift_2d;
  gtk_widget_show(button);

  /*
   * Exp multiply
   */
  nu=EM2D;
  process_button[nu].adj= (GtkAdjustment *) gtk_adjustment_new( 2, -1E6, 1E6, 1, 2, 0 );
  g_signal_connect (G_OBJECT (process_button[nu].adj), "value_changed", G_CALLBACK (update_active_process_data), (void*) nu);
  button = gtk_spin_button_new( GTK_ADJUSTMENT(  process_button[nu].adj ), 1.00, 2 );
  gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( button ), GTK_UPDATE_IF_VALID );
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu-P2D,nu-P2D+1);
  gtk_widget_show( button );

  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle), 
      (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "Exp Mult 2D" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_exp_mult_2d_and_display), process_button[nu].adj  );
  process_button[nu].func = do_exp_mult_2d;
  gtk_widget_show(button);

  /*
   * Gauss multiply
   */
  nu=GM2D;
  process_button[nu].adj= (GtkAdjustment *) gtk_adjustment_new( 2, -1E6, 1E6, 1, 2, 0 );
  g_signal_connect (G_OBJECT (process_button[nu].adj), "value_changed", G_CALLBACK (update_active_process_data), (void*) nu);
  button = gtk_spin_button_new( GTK_ADJUSTMENT(  process_button[nu].adj ), 1.00, 2 );
  gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( button ), GTK_UPDATE_IF_VALID );
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu-P2D,nu-P2D+1);
  gtk_widget_show( button );

  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle), 
      (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "Gauss Mult 2D" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_gaussian_mult_2d_and_display),  process_button[nu].adj );
  process_button[nu].func = do_gaussian_mult_2d;
  gtk_widget_show(button);




  /*
   * Zero fill
   */
  nu=ZF2D;
  process_button[nu].adj= (GtkAdjustment *) gtk_adjustment_new( 2, 1, 100, 1, 2, 0 );
  g_signal_connect (G_OBJECT (process_button[nu].adj), "value_changed", G_CALLBACK (update_active_process_data), (void*) nu);
  button = gtk_spin_button_new( GTK_ADJUSTMENT(  process_button[nu].adj ), 1.00, 1 );
  gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( button ), GTK_UPDATE_IF_VALID );
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu-P2D,nu-P2D+1);
  gtk_widget_show( button );

  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle), 
      (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "Zero Fill 2D" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_zero_fill_2d_and_display), process_button[nu].adj  );
  process_button[nu].func = do_zero_fill_2d;
  gtk_widget_show(button);

  /*
   * UNSCRAMBLE 
   */


  nu=UNSCRAMBLE;
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle),  (void*) nu);
  gtk_widget_show(process_button[nu].button);
  //  button = gtk_button_new_with_label( "Unscramble 2D" );
  button = gtk_button_new_with_label( "Divide first row by 2" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(unscramble_2d_and_display), NULL);
  process_button[nu].func = unscramble_2d;
  gtk_widget_show(button);


  /*
   * FT
   */

  nu=FT2D;
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle),  (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "FT2D" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_ft_2d_and_display), NULL);
  process_button[nu].func = do_ft_2d;
  gtk_widget_show(button);

  nu=MAG2D;
    /* 
     * convert hyper complex to simple 2d taking the magnitude in the second dimension
     * 
     */
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle),  (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "MAG2D" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_mag_2d_and_display), NULL);
  process_button[nu].func = do_mag_2d;
  gtk_widget_show(button);





  
  /*
   *  Baseline correct in indirect dimension
   */  
  nu=BC2D2;
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE( table ), process_button[nu].button,0,1,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT( process_button[nu].button ), "toggled", G_CALLBACK( process_button_toggle ),  (void*) nu);
  gtk_widget_show( process_button[nu].button );
  button = gtk_button_new_with_label( "Baseline Correct" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(do_offset_cal_2D_and_display), NULL);
  process_button[nu].func = do_offset_cal_2D;
  gtk_widget_show(button);
  /*
   *  Phase processing 2D
   */
  nu=PH2D;
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE( table ), process_button[nu].button,0,1,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT( process_button[nu].button ), "toggled", G_CALLBACK( process_button_toggle ),  (void*) nu);
  gtk_widget_show( process_button[nu].button );
  button = gtk_button_new_with_label( "Phase 2D" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK( do_phase_2d_and_display_wrapper ), NULL);
  process_button[nu].func = do_phase_2d_wrapper;
  gtk_widget_show(button);

  /*
   *  hadamard transform of the rows.
   */
  nu=HAD1;
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE( table ), process_button[nu].button,0,1,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT( process_button[nu].button ), "toggled", G_CALLBACK( process_button_toggle ),  (void*) nu);
  gtk_widget_show( process_button[nu].button );
  button = gtk_button_new_with_label( "Hadamard1" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK( do_hadamard1_and_display ), NULL);
  process_button[nu].func = do_hadamard1;
  gtk_widget_show(button);

  /*
   * decode Hayashi/chu sequence
   */

  nu=HAY1;
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE( table ), process_button[nu].button,0,1,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT( process_button[nu].button ), "toggled", G_CALLBACK( process_button_toggle ),  (void*) nu);
  gtk_widget_show( process_button[nu].button );
  button = gtk_button_new_with_label( "Hayashi1" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu-P2D,nu-P2D+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK( do_hayashi1_and_display ), NULL);
  process_button[nu].func = do_hayashi1;
  gtk_widget_show(button);




  /*
   *  Phase processing
   *
  nu=PH;
  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE( table ), process_button[nu].button,0,1,nu,nu+1);
  g_signal_connect(G_OBJECT( process_button[nu].button ), "toggled", G_CALLBACK( process_button_toggle ),  (void*) nu);
  gtk_widget_show( process_button[nu].button );
  button = gtk_button_new_with_label( "Phase" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK( do_phase_and_display_wrapper ), NULL);
  process_button[nu].func = do_phase_wrapper;
  gtk_widget_show(button);

  
  //   Phase radio buttons
   
  button = gtk_radio_button_new_with_label( NULL, "local" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu,nu+1);
  g_signal_connect(G_OBJECT(button),"toggled",G_CALLBACK(process_local_global_toggle),
		     (void *) 0);
  gtk_widget_show (button);

  r_local_button = button;

  group = gtk_radio_button_group (GTK_RADIO_BUTTON (button) );

  button = gtk_radio_button_new_with_label( group, "global" );
  r_global_button=button;
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu+1,nu+2);
  gtk_widget_show (button);


*/
  gtk_widget_show (table);

  snprintf(title,UTIL_LEN,"Process 2D");

  frame = gtk_frame_new(title);

  gtk_container_set_border_width(GTK_CONTAINER (frame),5);
  //  gtk_widget_set_size_request(frame,800,300);

  gtk_container_add(GTK_CONTAINER(frame),table);
  gtk_widget_show(frame);

  return frame;
}



gint do_zero_fill_2d(GtkWidget * widget,double *val)
{
  int new_npts2,old_npts2,acq_points2;
  float factor;
  dbuff *buff;
  int i,j,shift;

  factor = (float ) *val;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_zero_fill_2d panic! buff is null!",TRUE);
    return 0;
  }
  
  /* zero fill will always round up to the next power of 2
     so you can zero fill with factor of 1 to get next size up */

  // nope - powers of 2 no long special. Just does the val
  
  //  temp=buff->npts2*factor; 
  //  new_npts2 = pow(2,ceil(log(temp-0.001)/log(2.0)));
  new_npts2 = buff->npts2*factor;
  
  old_npts2=buff->npts2;
  acq_points2=buff->param_set.num_acqs_2d;
  
  if (new_npts2 ==old_npts2) return 0;  
  if (old_npts2>= MAX_DATA_NPTS) return 0;

  cursor_busy(buff);
  //let's try  something else
  buff_resize(buff,buff->npts,new_npts2);
  if (widget !=NULL || upload_buff == current) update_npts2(new_npts2); 



  // zero out the new stuff, but don't trust the new_npts

  new_npts2 = buff->npts2;

  // the update will have screwed this up, fix it:
  buff->param_set.num_acqs_2d=acq_points2; 

  // what to do if the number of points was originally odd?
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.symm_check))){
    // move the data back to the center
    shift = (new_npts2-old_npts2)/2;
    for(i=0;i<buff->npts*2;i++)
      for(j=old_npts2-1;j>=0;j--){
	buff->data[i+(shift+j)*buff->npts*2]=buff->data[i+j*buff->npts*2];
	buff->data[i+j*buff->npts*2] = 0.;
      }

  }

      
  cursor_normal(buff);

  // fprintf(stderr,"do_zero_fill: got factor: %f, rounding to: %i\n",factor,new_npts);

 return 0;
}

gint do_zero_fill_2d_and_display(GtkWidget * widget,GtkAdjustment *adj)
{
  dbuff *buff;
  gint result;
  gint old_npts2;
  double val;
  val = gtk_adjustment_get_value(adj);

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  old_npts2=buff->npts2;
  result = do_zero_fill_2d( widget, &val );

  if (old_npts2 != buff->npts2)
    draw_canvas( buff );

  return result;

}


//un

gint unscramble_2d_and_display( GtkWidget *widget, double *unused )
{
  dbuff *buff;
  gint result;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  result = unscramble_2d( widget, unused );

  draw_canvas( buff );
   return result;
}

gint unscramble_2d(GtkWidget *widget, double *unused)

{

  // routine and button hijacked to divide the first row (or two if hyper complex)
  // by 2 to try to get baseline correct in indirect dimension.

  /* unscramble the different pieces of the data for Varian 2D n-type data
     (eg gHMQC data 
     for Varian's ft2d(1,0,1,0,0,1,0,-1)

     makes the data set a true complex set  - no don't do this
     make into hypercomplex instead

     either is fine, but with hyper complex you can phase in both dimensions
     after the ft.
*/

  dbuff *buff;
  int i;
  int npts;
  //  int j;
  //  int npts2;
  //  float *nbuff;

  if( widget == NULL ) {
    buff = buffp[ upload_buff ];
    // fprintf(stderr,"do_ft- on buffer %i\n",upload_buff );
  }
  else {
    buff = buffp[ current ];
    // fprintf(stderr,"do_ft- on buffer %i\n",current );
  }
  if (buff == NULL){
    popup_msg("unscramble! buff is null!",TRUE);
    return 0;
  }
  npts= buff->npts;
  //  npts2=buff->npts2;

  //// new stuff here

  for (i=0;i<2*npts;i++)
    buff->data[i] /=2;
  if  (buff->is_hyper)
    for (i=0;i<2*npts;i++)
      buff->data[i+2*npts] /=2;
    

  //// end
  
  /*
  nbuff = (float *)malloc(sizeof(float)*2*npts*npts2);
  for (i=0;i<npts*2*npts2;i++)
    nbuff[i]=buff->data[i];
  for(i=0;i<npts;i++){
    for(j=0;j<npts2/2;j++){
      //buff->data[2*i+j*npts*2] = nbuff[2*i+(2*j)*(npts*2)]+ 
      //nbuff[2*i+(2*j+1)*(npts*2)];

      //      buff->data[2*i+j*npts*2+1] = nbuff[2*i+(2*j)*(npts*2)+1]- 
      //nbuff[2*i+(2*j+1)*(npts*2)+1]; 
      buff->data[2*i+2*j*npts*2] = nbuff[2*i+(2*j)*(npts*2)]+ 
	nbuff[2*i+(2*j+1)*(npts*2)];
      buff->data[2*i+2*j*npts*2+1]=nbuff[2*i+(2*j)*(npts*2)+1]+ 
	nbuff[2*i+(2*j+1)*(npts*2)+1];

      buff->data[2*i+(2*j+1)*npts*2] = nbuff[2*i+(2*j)*(npts*2)+1]- 
	nbuff[2*i+(2*j+1)*(npts*2)+1];
      buff->data[2*i+(2*j+1)*npts*2+1]=-nbuff[2*i+(2*j)*(npts*2)]+ 
	nbuff[2*i+(2*j+1)*(npts*2)];
    }
  }
  free(nbuff);

  // now turn on true complex flag, and ensure the hypercomplex is off  
  // no - backwards
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(buff->win.hypercheck),TRUE);
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(buff->win.true_complex),FALSE);
*/
  return TRUE;
  
}





gint do_hadamard1_and_display( GtkWidget *widget, double *unused )
{
  dbuff *buff;
  gint result;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  result = do_hadamard1( widget, unused );

  draw_canvas( buff );
   return result;
}

void hadamard(char *had_mat,int order){
  char *mat1,dim;
  int i,j,k,nm,om;

  mat1= malloc(sizeof(char)*(1<<order)*(1<<order));
  dim = 1<<order;


  mat1[0]=1;
  for (k=0;k<order;k++){
    nm = 1<<(k+1);
    om = 1<<k;
    printf("building matrix dim: %i from %i\n",nm,om);

    for(i=0;i<om;i++)
      for(j=0;j<om;j++){
	had_mat[i+dim*j]=mat1[i+dim*j];
	had_mat[i+om+dim*j]=mat1[i+dim*j];
	had_mat[i+dim*(j+om)]=mat1[i+dim*j];
	had_mat[i+om+dim*(j+om)]=-1*mat1[i+dim*j];
      }

    for(i=0;i<nm;i++){
      for(j=0;j<nm;j++){
	mat1[i+dim*j]=had_mat[i+dim*j];
	//	printf("%2i ",mat1[i+dim*j]);
      }
      //      printf("\n");
    }
  }
  free(mat1);   
}


gint do_hadamard1(GtkWidget *widget, double *unused)

{
  /* do hadamard unscramble for imaging. */

  dbuff *buff;
  int i,j,k,np2d,order;
  float *new_data;
  char *had_mat;
  if( widget == NULL ) {
    buff = buffp[ upload_buff ];
    // fprintf(stderr,"do_ft- on buffer %i\n",upload_buff );
  }
  else {
    buff = buffp[ current ];
    // fprintf(stderr,"do_ft- on buffer %i\n",current );
  }
  if (buff == NULL){
    popup_msg("do_ft_2d panic! buff is null!",TRUE);
    return FALSE;
  }

  cursor_busy(buff);

  // want a hadmard matrix that is acqn2d x acqn2d
  np2d =  buff->npts2;
  order=0;
  for(i=1;i<10;i++){
    if ((1<<i) == np2d){
      printf("we have a winner! Hadmard matrix is %i on a side, order: %i\n",np2d,order);
      order=i;
    }
  }
  if (order == 0){
    printf("couldn't build hadamard matrix\n");
    return 0;
  }
  had_mat = malloc(sizeof(char)*np2d*np2d);
  new_data = malloc(sizeof(float)*np2d*2*buff->npts);
  hadamard(had_mat,order);

  // copy data over
  for(i=0;i<np2d*buff->npts*2;i++)
    new_data[i] = buff->data[i];
  
  // zero our old memory
  memset(buff->data,0,buff->npts*2*np2d*sizeof(float));
  // do the matrix multiplication
  for(i=0;i<buff->npts;i++)
    for(j=0;j<np2d;j++)
      for(k=0;k<np2d;k++){
	buff->data[2*i + buff->npts*2*j] += had_mat[j+np2d*k]*new_data[2*i + buff->npts*2*k];
	buff->data[2*i+1 + buff->npts*2*j] += had_mat[j+np2d*k]*new_data[2*i+1 + buff->npts*2*k];
      }

  free(new_data);
  free(had_mat);
  cursor_normal(buff);
  return TRUE;
}



///
gint do_hayashi1_and_display( GtkWidget *widget, double *unused )
{
  dbuff *buff;
  gint result;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  result = do_hayashi1( widget, unused );

  draw_canvas( buff );
   return result;
}

void generate_hayashi_seq(float *seqs,int n, int k){

  int *a, *b,r,j;
  float *chuseq;
  int chulen;
  int seqlen,seqnum,nseqs;
  float *seqst;
  chulen=(2*k+1)*(2*n+1);
  seqlen=4*chulen;
  nseqs=4*n+2;
  seqst=malloc(sizeof(float)*seqlen*nseqs);
  memset(seqst,0,sizeof(float)*seqlen*nseqs);

  chuseq = malloc(sizeof(float)*chulen);
  a=malloc(sizeof(int)*(2*n+1));
  b=malloc(sizeof(int)*(2*n+1));


  // get the chu seq:
  get_chu_seq(-1); // reset the chu seq.
  for(j=0;j<chulen;j++)
    chuseq[j]=get_chu_seq(chulen);
  // build the indicies
  for(r=0;r<2*n+1;r++){
    a[r]=(2*k+1)*((r+n)%(2*n+1))+k;
    b[r] = (2*k+1)*r;
  }
  
  // now build the sequences
  for(r=0;r<(2*n+1);r++){
    for(j=0;j<2*(2*k+1)*(2*n+1);j++){
      seqnum=2*r;
      // Hayashi's f^0
      seqst[seqnum*seqlen+2*j] = chuseq[(a[r]+j)%chulen];
      seqst[seqnum*seqlen+2*j+1] = chuseq[(b[r]+j)%chulen];

      seqnum = 2*r+1;
      seqst[seqnum*seqlen+2*j] = chuseq[(a[r]+j)%chulen];
      //      seqst[seqnum*seqlen+2*j+1] = -chuseq[(b[r]+j)%chulen];
      seqst[seqnum*seqlen+2*j+1] = chuseq[(b[r]+j)%chulen]+180.;

      // Hayashi's f^1
      seqnum=2*r;
      seqs[seqnum*seqlen+2*j] = seqst[(2*r)*seqlen+j];
      seqs[seqnum*seqlen+2*j+1] = seqst[(2*r+1)*seqlen+j];

      seqnum = 2*r+1;
      seqs[seqnum*seqlen+2*j] = seqst[(2*r)*seqlen+j];
      //      seqs[seqnum*seqlen+2*j+1] = -seqst[(2*r+1)*seqlen+j];
      seqs[seqnum*seqlen+2*j+1] = seqst[(2*r+1)*seqlen+j]+180.;
    }
  }

  free(a);
  free(b);
  free(chuseq);
  free(seqst);
}


gint do_hayashi1(GtkWidget *widget, double *unused)

{

  /* unscramble a single 1D data set into the records using hayashi/chu sequences 
   */

  dbuff *buff;
  int i,j,k,seqlen,n,echolen,nseqs; // ,chulen;
  float *new_data;
  float *seqs;
  float *my_seq;
  if( widget == NULL ) {
    buff = buffp[ upload_buff ];
    // fprintf(stderr,"do_ft- on buffer %i\n",upload_buff );
  }
  else {
    buff = buffp[ current ];
    // fprintf(stderr,"do_ft- on buffer %i\n",current );
  }
  if (buff == NULL){
    popup_msg("do_ft_2d panic! buff is null!",TRUE);
    return FALSE;
  }

  cursor_busy(buff);

  // get the n and k parameters from the panel.
  n=0;
  k=0;
  i = pfetch_int(&buff->param_set,"n",&n,0);
  i = pfetch_int(&buff->param_set,"k",&k,0);
  if (n == 0 || k == 0 ){
    popup_msg("Couldn't find n or k in the panel, can't do Hayashi decode",TRUE);
    return FALSE;
  }
  printf("got: n = %i, k= %i\n",n,k);
  seqlen = 4*(2*n+1)*(2*k+1);
  nseqs = 4*n+2;
  //  chulen = (2*n+1)*(2*k+1);
  echolen = 4*k+2;

  // check to make sure there are enough points
  if (buff->npts < seqlen+echolen){
    popup_msg("there don't seem to be enough data points for the Hayashi decode",TRUE);
    return FALSE;
  }


  seqs = malloc(sizeof(float)*seqlen*nseqs); // these are angles in degrees
  my_seq=malloc(sizeof(float)*seqlen*2); // these are the sines and cosines for cross corr.
  new_data = malloc(sizeof(float)*buff->npts*2*buff->npts2);
  memset(seqs,0,sizeof(float)*seqlen*nseqs);

  printf("process_f: about to generate hayashi, with n: %i, k: %i, seqs allocated: %i\n",n,k,seqlen*nseqs);
  generate_hayashi_seq(seqs,n,k);
  
  // we're only going to use the first sequence for the time being.  Maybe later we'll use them all.
  // any one gives the same thing as all the others, just reorganized.

  for(i=0;i<seqlen;i++){
    my_seq[2*i] = cos(seqs[i]*M_PI/180.);
    my_seq[2*i+1] = sin(seqs[i]*M_PI/180.);
  }
  
  // copy data over old data to new spot.
  for(i=0;i<buff->npts2*buff->npts*2;i++)
    new_data[i] = buff->data[i];
  
  // zero our old memory
  memset(buff->data,0,buff->npts*2*buff->npts2*sizeof(float));

  //  printf("seqlen: %i, echolen: %i\n",seqlen,echolen);
  
  // do the cross correlation
  for(k=0;k<buff->npts2;k++){ 
    for (i=0;i<seqlen;i++){
      for(j=0;j<seqlen;j++){
	buff->data[k*buff->npts*2+2*i] += new_data[2*((i+j)%seqlen+echolen)+k*buff->npts*2]*my_seq[2*((j+echolen)%seqlen)]
	  +new_data[2*((i+j)%seqlen+echolen)+1+k*buff->npts*2]*my_seq[2*((j+echolen)%seqlen)+1];
	buff->data[k*buff->npts*2+2*i+1] += new_data[2*((i+j)%seqlen+echolen)+k*buff->npts*2]*my_seq[2*((j+echolen)%seqlen)+1]
	  -new_data[2*((i+j)%seqlen+echolen)+1+k*buff->npts*2]*my_seq[2*((j+echolen)%seqlen)];
      }
    }
  }


  free(new_data);
  free(seqs);
  free(my_seq);

  cursor_normal(buff);
  return TRUE;
}


/*
gint unwind_2d_and_display( GtkWidget *widget, double *unused )
{
  dbuff *buff;
  gint result;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  result = unwind_2d( widget, unused );

  draw_canvas( buff );
   return result;
}


gint unwind_2d(GtkWidget *widget, double *unused)

{
  // unwind some phase from the second dimension 
  //  for crackpot autoshimming feature. 

  dbuff *buff;
  int i,j;
  float re,im,freq,phase;
  float mag,ph;
  double tau;
  int npts,npts2;

  if( widget == NULL ) {
    buff = buffp[ upload_buff ];
    // fprintf(stderr,"do_ft- on buffer %i\n",upload_buff );
  }
  else {
    buff = buffp[ current ];
    // fprintf(stderr,"do_ft- on buffer %i\n",current );
  }
  if (buff == NULL){
    popup_msg("unwind_2d panic! buff is null!",TRUE);
    return 0;
  }
  
   
  if (buff->is_hyper == FALSE){
    popup_msg("Not hyper, can't unwind phase",TRUE);
    return TRUE;
  }
  
  // need to find a tau in the parameter set.

  i = pfetch_float(&buff->param_set,"tau",&tau,0);
  if (i == 0) {
    popup_msg("no parameter tau found for phase unwind",TRUE);
    return TRUE;
  }
  fprintf(stderr,"got tau: %lf\n",tau);
  
  npts = buff->npts;
  npts2 = buff->npts2;
  
  for (i=0;i<npts;i++){
    // what's the freq at this point?
    freq = - ( (double) i * 1./buff->param_set.dwell*1e6/npts
	       - (double) 1./buff->param_set.dwell*1e6/2.);
    // so the phase to unwind is:
    phase = freq*tau*2*M_PI;
    
    for(j=0;j<npts2/2;j++){
      re = buff->data[(2*j)*npts*2+2*i];
      im = buff->data[(2*j+1)*npts*2+2*i];
      mag = sqrt(re*re+im*im);
      ph = atan2(im,re);
      ph = ph-phase;
      buff->data[(2*j)*npts*2+2*i] = mag*cos(ph);
      buff->data[(2*j+1)*npts*2+2*i] = mag*sin(ph);
      
      re = buff->data[(2*j)*npts*2+2*i+1];
      im = buff->data[(2*j+1)*npts*2+2*i+1];
      mag = sqrt(re*re+im*im);
      ph = atan2(im,re);
      ph = ph-phase;
      buff->data[(2*j)*npts*2+2*i+1] = mag*cos(ph);
      buff->data[(2*j+1)*npts*2+2*i+1] = mag*sin(ph);
      
      
      
      
    }
  }
  
  
  return TRUE;
  
}
*/


gint do_ft_2d_and_display( GtkWidget *widget, double *unused )
{
  dbuff *buff;
  gint result;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  result = do_ft_2d( widget, unused );

  draw_canvas( buff );
   return result;
     }

gint do_ft_2d(GtkWidget *widget, double *unused)

{
  /* do fourier transform in 2nd dimension */

  dbuff *buff;
  int i,j,shift=0;
  double spared;
  float scale;
  float *new_data;
  char is_symm;
  char true_complex;

  static fftwf_plan plan1;
  int n[] = {0};

 
  if( widget == NULL ) {
    buff = buffp[ upload_buff ];
    // fprintf(stderr,"do_ft- on buffer %i\n",upload_buff );
  }
  else {
    buff = buffp[ current ];
    // fprintf(stderr,"do_ft- on buffer %i\n",current );
  }
  if (buff == NULL){
    popup_msg("do_ft_2d panic! buff is null!",TRUE);
    return 0;
  }

  is_symm = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.symm_check));
  true_complex = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.true_complex));
  

  // can't have hyper and true complex at the same time.
  // if true complex is active, then we say its true complex.

			      
  buff->flags ^= FT_FLAG2; //toggle the ft flag

  if ( !buff->is_hyper && !true_complex && ((buff->flags & FT_FLAG2) == 0)){
    popup_msg("Trying to do inverse FT where we don't have any imaginary",TRUE);
    buff->flags ^= FT_FLAG2; //toggle the ft flag
    return 0;
  }
  
  cursor_busy(buff);
  
  /* do a zero fill with a factor of 1 to make 
     sure we have a power of two as npts 

  - or, do a factor of 2 so we have room to put in the imaginaries - if not already hyper and not true complex*/
  
  if (buff->is_hyper || true_complex )
    spared= 1.0;
  else{
    spared=2.0;
    do_zero_fill_2d(widget,&spared);
  }

    //  fprintf(stderr,"in 2dft just did zero fill\n");

  load_wisdom();
  new_data = g_malloc(2*buff->npts*buff->npts2*sizeof(float));
  if (true_complex){
    printf("doing true_complex FT2d\n");
    printf("TESTME: true_complex FT2d not tested since switch to FFTW\n");
    // do in place FFTW FT.
    n[0] = buff->npts2;  // elements in each transform
    if (buff->flags & FT_FLAG2){
      plan1 = fftwf_plan_many_dft(1,n,buff->npts,
				  (fftwf_complex *) new_data, NULL,buff->npts,1,
				  (fftwf_complex *) new_data,NULL,buff->npts,1,
				  FFTW_FORWARD,FFTW_MEASURE);
      scale = buff->npts2/2.;
    }
    else{
      plan1 = fftwf_plan_many_dft(1,n,buff->npts,
				  (fftwf_complex *) new_data, NULL,buff->npts,1,
				  (fftwf_complex *) new_data,NULL,buff->npts,1,
				  FFTW_BACKWARD,FFTW_MEASURE);
      scale =2.0;
    }
    	   // args are rank = 1, n - number of elements it transform, howmany transforms to do (npts2)
	   // the input data, inembed =NULL, istride=buff->npts,idist=1
	   // output = data, onembed=NULL,ostride=npts, odist=1
	   // sign , flags.
    
    // if ((symmetric and FT forward) or reverse), swap
    if (((buff->flags & FT_FLAG2) && is_symm) || ((buff->flags & FT_FLAG2)  == 0)){
      shift = buff->npts2/2 + buff->npts2 % 2;
      for (i=0;i<buff->npts;i++)
	for (j=0;j<buff->npts2;j++){
	  new_data[2*i+j*2*buff->npts] = buff->data[2*i + ((j+shift)%buff->npts2)*2*buff->npts];
	  new_data[2*i+j*2*buff->npts+1] = buff->data[2*i + ((j+shift)%buff->npts2)*2*buff->npts+1];
	}
    }
    else
      memcpy(new_data,buff->data,sizeof(float)*2*buff->npts*buff->npts2);
    
    // now do the FT:
    fftwf_execute(plan1);
    
    // then unscramble
    if ((((buff->flags & FT_FLAG2) == 0) && is_symm) || ((buff->flags & FT_FLAG2) ))
      shift = buff->npts2/2 + buff->npts2 %2;
    else
      shift =0.;
    for (i=0;i<buff->npts;i++)
      for (j=0;j<buff->npts2;j++){
	buff->data[2*i+((j+shift)%buff->npts2)*2*buff->npts] = new_data[2*i + j*2*buff->npts]/scale;
	buff->data[2*i+((j+shift)%buff->npts2)*2*buff->npts+1] = new_data[2*i + j*2*buff->npts+1]/scale;
      }
  
  } //not true complex:
  else if (buff->is_hyper == FALSE){ 
    //    popup_msg("hypercomplex flag not set, doing real FT?",TRUE);
    fprintf(stderr,"hypercomplex flag not set, doing real FT\n");
    new_data = g_malloc(buff->npts2 * sizeof(float) );
    //  fprintf(stderr,"2dft did malloc, 2dnpts = %i\n",buff->npts2);
    if (new_data == NULL) fprintf(stderr,"failed to malloc!\n");
    if (buff->flags & FT_FLAG2){
      plan1 = fftwf_plan_dft_1d(buff->npts2/2,(fftwf_complex *) new_data, (fftwf_complex *) new_data,FFTW_FORWARD,FFTW_MEASURE);
      scale = buff->npts2/4.;
    }
    else{
      plan1 = fftwf_plan_dft_1d(buff->npts2/2,(fftwf_complex *) new_data, (fftwf_complex *) new_data,FFTW_BACKWARD,FFTW_MEASURE);
      scale = 2.0;
    }


    for(i=0;i<buff->npts*2 ;i++){
      if (((buff->flags & FT_FLAG2) && is_symm) || ((buff->flags & FT_FLAG2)  == 0))
	shift = buff->npts2/4 + buff->npts2/2 %2;
      else
	shift = 0;
      // copy data out	
      //      printf("pre-ft, shift is: %i\n",shift);
      for(j=0;j<buff->npts2/2;j++){
	new_data[j*2] = buff->data[((j+shift)%(buff->npts2/2))*2*buff->npts+i];
	new_data[j*2+1] =0.;
      }


      // correct the first point if we're going forward... and not symmetric
      //      if (buff->flags & FT_FLAG2 & !is_symm)
      //	new_data[0] /= 2.;
      // naw dont.  Not doing this only ever introduces a baseline offset.

      fftwf_execute(plan1);
      //      four1(new_data-1,buff->npts2/2,1);

      if ((((buff->flags & FT_FLAG2) == 0) && is_symm) || ((buff->flags & FT_FLAG2) ))
	shift = buff->npts2/4 + buff->npts2/2 %2;
      else
	shift = 0;
      // descramble
      //      printf("post-ft, shift is: %i\n",shift);
      for(j=0;j<buff->npts2/2;j++){
	 buff->data[2*((j+shift)%(buff->npts2/2))*buff->npts*2+i] =new_data[j*2]/scale;
	 buff->data[(2*((j+shift)%(buff->npts2/2))+1)*buff->npts*2+i] =new_data[j*2+1]/scale;
      }
    }
      
    // turn on the is_hyper_flag!
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(buff->win.hypercheck),TRUE);

  }
  else {// is hypercomplex
    if (buff->npts2 %2 == 1){
      printf("YOU'VE GOT HYPERCOMPLEX SET, BUT AN ODD NUMBER OF POINTS. IGNORING LAST POINT\n");
    }
    new_data = g_malloc(buff->npts2 * sizeof(float));
    //  fprintf(stderr,"2dft did malloc, 2dnpts = %i\n",buff->npts2);
    if (new_data == NULL) fprintf(stderr,"failed to malloc!\n");
    if (buff->flags & FT_FLAG2){
      plan1 = fftwf_plan_dft_1d(buff->npts2/2,(fftwf_complex *) new_data, (fftwf_complex *) new_data,FFTW_FORWARD,FFTW_MEASURE);
      scale = buff->npts2/4.;
    }
    else{
      plan1 = fftwf_plan_dft_1d(buff->npts2/2,(fftwf_complex *) new_data, (fftwf_complex *) new_data,FFTW_BACKWARD,FFTW_MEASURE);
      scale = 2.0;
    }

    for(i=0;i<buff->npts*2  ;i++){
      if (((buff->flags & FT_FLAG2) && is_symm) || ((buff->flags & FT_FLAG2)  == 0)) // HERE
	shift = buff->npts2/4 + buff->npts2/2 % 2;
      else
	shift = 0;

      for(j=0;j<buff->npts2/2;j++){
	new_data[j*2] = buff->data[2*((j+shift)%(buff->npts2/2))*buff->npts*2+i];
	new_data[j*2+1] = buff->data[(2*((j+shift)%(buff->npts2/2))+1)*buff->npts*2+i];
      }
      // divide the first point by 2:
      //      if (buff->flags & FT_FLAG2 & !is_symm){
      //	printf("dividing first point by 2\n");
      //      	new_data[0] /= 2.;
      //}

	  
      fftwf_execute(plan1);
	//      four1(new_data-1,buff->npts2/2,1);
      if ((((buff->flags & FT_FLAG2) == 0) && is_symm) || ((buff->flags & FT_FLAG2) ))
      	shift = buff->npts2/4 + buff->npts2/2 %2;
      else
	shift = 0;
      // descramble
      for(j=0;j<buff->npts2/2;j++){
	 buff->data[2*((j+shift)%(buff->npts2/2))*buff->npts*2+i] =new_data[j*2]/scale;
	 buff->data[(2*((j+shift)%(buff->npts2/2))+1)*buff->npts*2+i] =new_data[j*2+1]/scale;
      }
    }
  }
  //  fprintf(stderr,"2dft about to free buffer\n");
  save_wisdom();
  fftwf_destroy_plan(plan1);
  g_free(new_data);
  cursor_normal(buff);

  return TRUE;
  
}



gint do_mag_2d_and_display( GtkWidget *widget, double *unused )
{
  dbuff *buff;
  gint result;

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  result = do_mag_2d( widget, unused );

  draw_canvas( buff );
   return result;
}

gint do_mag_2d(GtkWidget *widget, double *unused)

{
  /* take magnitude in second dimension */

  dbuff *buff;
  int i,j;

  if( widget == NULL ) {
    buff = buffp[ upload_buff ];
    // fprintf(stderr,"do_ft- on buffer %i\n",upload_buff );
  }
  else {
    buff = buffp[ current ];
  }
  if (buff == NULL){
    popup_msg("do_mag_2d panic! buff is null!",TRUE);
    return 0;
  }


  if(  gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.true_complex))){
    for (j=0;j<buff->npts2;j++){
      for(i=0;i<buff->npts;i++){
	buff->data[j*buff->npts*2+2*i] = 
	  sqrt(buff->data[j*buff->npts*2+2*i]*buff->data[j*buff->npts*2+2*i]
	       +buff->data[j*buff->npts*2+2*i+1]*buff->data[j*buff->npts*2+2*i+1]);
	buff->data[j*buff->npts*2+2*i+1] = 0;
      }
    }
    return TRUE;
  }
  
  if (!buff->is_hyper){
    popup_msg("buff wasn't hyper, can't take mag2d",TRUE);
    return TRUE;
  }

  // turn off the is_hyper flag:
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(buff->win.hypercheck),FALSE);

  for (j=0;j<buff->npts2/2;j++){
    for(i=0;i<buff->npts*2;i++)
      buff->data[j*buff->npts*2+i] = 
	sqrt(buff->data[2*j*buff->npts*2+i]*buff->data[2*j*buff->npts*2+i]
	     +buff->data[(2*j+1)*buff->npts*2+i]*buff->data[(2*j+1)*buff->npts*2+i]);
  }				     
    // free the extra memory:
    buff_resize(buff,buff->npts,buff->npts2/2);
  


  return TRUE;
  
}

gint do_exp_mult_2d_and_display( GtkWidget *widget, GtkAdjustment *adj )
{
  dbuff *buff;
  gint result;
  double val;
  val = gtk_adjustment_get_value(adj);

  result = do_exp_mult_2d( widget, &val );

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  draw_canvas( buff );
  return result;
}

gint do_exp_mult_2d( GtkWidget* widget, double * val )
{
  int i,j,i2;
  float factor;
  double dwell2,sw2;
  dbuff* buff;
  char is_symm;

  factor = (float) *val;

  // fprintf(stderr, "doing exp_mult by %f\n", factor );

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];

  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_exp_mult_2d panic! buff is null!",TRUE);
    return 0;
  }

  is_symm = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.symm_check));
  //eegads, what do I use as sw in second dimension? look for a parameter called dwell2

  i = pfetch_float(&buff->param_set,"dwell2",&dwell2,0);
  if (i == 1)
    sw2 =1.0/dwell2;
  else
    pfetch_float(&buff->param_set,"sw2",&sw2,0);
  if (i==1)
    dwell2=1./sw2;
  else {
    fprintf(stderr,"can't find dwell2 or sw2, bailing out of em2d\n");
    return 0;
  }



  for( j=0; j<buff->npts2; j++ ){
    i2 = floor(j/(1+buff->is_hyper));
    if (is_symm){
      if (buff->is_hyper)
	i2 -= buff->npts2/4+0.5*((buff->npts2/2)%2);
      else	
	i2 -= buff->npts2/2+0.5*(buff->npts2%2);
    }
    factor = exp ( - *val * i2 * dwell2 *M_PI);

    for( i=0; i<buff->npts; i++ ){
      buff->data[2*i+j*2*buff->npts] *= factor  ; 
      buff->data[2*i+1+j*2*buff->npts] *= factor ; 
    }
  }
  return 0;
}




gint do_gaussian_mult_2d_and_display( GtkWidget *widget, GtkAdjustment *adj )
{
  dbuff *buff;
  gint result;
  double val;
  val = gtk_adjustment_get_value(adj);

  result = do_gaussian_mult_2d( widget, &val );

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  draw_canvas( buff );
  return result;
}

gint do_gaussian_mult_2d( GtkWidget* widget, double * val )
{
  int i,j,i2;
  float factor;
  double dwell2,sw2;
  dbuff* buff;
  char is_symm,sign=1;
  factor = (float) *val;
  if (factor < 0) sign = -1;
  // fprintf(stderr, "doing exp_mult by %f\n", factor );

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];

  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_exp_mult_2d panic! buff is null!",TRUE);
    return 0;
  }
  is_symm = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.symm_check));

  //eegads, what do I use as sw in second dimension? look for a parameter called dwell2

  i = pfetch_float(&buff->param_set,"dwell2",&dwell2,0);
  if (i == 1)
    sw2 =1.0/dwell2;
  else
    pfetch_float(&buff->param_set,"sw2",&sw2,0);
  if (i==1)
    dwell2=1./sw2;
  else {
    fprintf(stderr,"can't find dwell2 or sw2, bailing out of em2d\n");
    return 0;
  }


  for( j=0; j<buff->npts2; j++ ){
    i2 = floor(j/(1+buff->is_hyper));
    if (is_symm){
      if (buff->is_hyper)
	i2 -= buff->npts2/4+0.5*((buff->npts2/2)%2);
      else	
	i2 -= buff->npts2/2+0.5*(buff->npts2%2);
    }
    //    fprintf(stderr,"gauss 2d %i %i\n",j,i2);

    factor = dwell2 *M_PI * *val/1.6651 * i2;

    for( i=0; i<buff->npts; i++ ){
      buff->data[2*i+j*2*buff->npts] *= exp(-1*factor*factor*sign) ; 
      buff->data[2*i+1+j*2*buff->npts] *= exp(-1*factor*factor*sign) ; 
    }
  }
  return 0;
}


gint do_offset_cal_2D_and_display( GtkWidget *widget, double *unused )
{
  dbuff *buff;
  gint result;

  result = do_offset_cal_2D( widget, unused );


  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  draw_canvas( buff );
  return result;
}


gint do_offset_cal_2D( GtkWidget *widget, double *unused )

{
  int i, j;
  int count;
  float offset;
  dbuff *buff;
  

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_ft_offset_cal_2D panic! buff is null!",TRUE);
    return 0;
    
  }
  //  fprintf(stderr,"%d %d\n\n", buff->npts, buff->npts2);
  if (buff->is_hyper == 1){
    for( j=0;j<2*buff->npts;j++) {
      
      count = 0;
      offset = 0.0;
      
      //first determine the offset for the real channel (in indirect dimension)
      for(i=buff->npts2/2*0.9 ; i<buff->npts2/2;i++){
	offset += buff->data[4*i*buff->npts+j];
	count ++;
      }
      offset=offset/count;
      for(i=0;i<buff->npts2/2;i++)
	buff->data[4*i*buff->npts+j] -= offset;


      // now do imag
      count = 0;
      offset = 0.0;
      
      //first determine the offset for the real channel (in indirect dimension)
      for(i=buff->npts2/2*0.9 ; i<buff->npts2/2;i++){
	offset += buff->data[(2*i+1)*2*buff->npts+j];
	count ++;
      }
      offset=offset/count;
      for(i=0;i<buff->npts2/2;i++)
	buff->data[(2*i+1)*2*buff->npts+j] -= offset;
    }
  }


  else{ // isn't hypercomplex
    fprintf(stderr,"doing real baseline correct\n");
    for( j=0;j<2*buff->npts;j++) {
      count = 0;
      offset = 0.0;
      for (i=buff->npts2*0.9 ; i<buff->npts2; i++){
	offset += buff->data[2*i*buff->npts+j];
	count++;
      }
      offset = offset/count;
      for(i=0;i<buff->npts2;i++){
	buff->data[2*i*buff->npts+j] -= offset;
      }
    }
  }


  

  return 0;
}



gint do_offset_cal_2D_a_and_display( GtkWidget *widget, double *unused )
{
  dbuff *buff;
  gint result;

  result = do_offset_cal_2D_a( widget, unused );


  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  draw_canvas( buff );
  return result;
}


gint do_offset_cal_2D_a( GtkWidget *widget, double *unused )

{
  dbuff *buff;
  int i,j;
  float *new_data,scale;

    // fftw3 stuff:
  static fftwf_plan plan1,plan2;
  int n[] = {0};

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("do_ft_offset_cal_2D panic! buff is null!",TRUE);
    return 0;
    
  }
  //  fprintf(stderr,"%d %d\n\n", buff->npts, buff->npts2);
  fprintf(stderr,"npts2: %i\n",buff->npts2);


  if (buff->npts2 < 8 ) return 0; // forget it!

  cursor_busy(buff);


  load_wisdom();
  new_data=g_malloc(sizeof(float)*2*buff->npts*buff->npts2);
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.true_complex))){
    scale = buff->npts2;
    new_data = g_malloc(buff->npts2 * sizeof(float)*2*buff->npts  );

    n[0] = buff->npts2;  // elements in each transform
    plan1 = fftwf_plan_many_dft(1,n,buff->npts,
	   (fftwf_complex *) new_data, NULL,buff->npts,1,
           (fftwf_complex *) new_data,NULL,buff->npts,1,
	   FFTW_FORWARD,FFTW_MEASURE);
    plan2 = fftwf_plan_many_dft(1,n,buff->npts,
	   (fftwf_complex *) new_data, NULL,buff->npts,1,
           (fftwf_complex *) new_data,NULL,buff->npts,1,
	   FFTW_BACKWARD,FFTW_MEASURE);
	   // args are rank = 1, n - number of elements it transform, howmany transforms to do (npts2)
	   // the input data, inembed =NULL, istride=buff->npts,idist=1
	   // output = data, onembed=NULL,ostride=npts, odist=1
	   // sign , flags.
    memcpy(new_data,buff->data,sizeof(float)*2*buff->npts*buff->npts2);

    scale = buff->npts2;
    fftwf_execute(plan1);
    for ( j = 0;j<buff-> npts; j++){
	new_data[2*j ]= (new_data[2*j+2*buff->npts] + new_data[2*j+(buff->npts2-1)*2*buff->npts])/2.0;
	new_data[2*j+1 ]= (new_data[2*j+2*buff->npts+1] + new_data[2*j+(buff->npts2-1)*2*buff->npts+1])/2.0;
      
      for (i=0;i<buff->npts2;i++){
	new_data[2*i*buff->npts+2*j] /= scale;
	new_data[2*i*buff->npts+2*j+1] /= scale;
      }
    
	   
    }
    fftwf_execute(plan2);
    memcpy(buff->data,new_data,sizeof(float)*2*buff->npts*buff->npts2);

  }
  else   if (buff->is_hyper == FALSE){
    new_data = g_malloc(sizeof(float) * buff->npts2 *2);
    scale = buff->npts2;

    plan1 = fftwf_plan_dft_1d(buff->npts2,(fftwf_complex *) new_data, (fftwf_complex *) new_data,FFTW_FORWARD,FFTW_MEASURE);
    plan2 = fftwf_plan_dft_1d(buff->npts2,(fftwf_complex *) new_data, (fftwf_complex *) new_data,FFTW_BACKWARD,FFTW_MEASURE);

    // copy out
    for(i=0;i<buff->npts*2  ;i++){
      for(j=0;j<buff->npts2;j++){
	new_data[j*2] = buff->data[j*buff->npts*2+i];
	new_data[j*2+1] = 0.;
      }
    
      // do the ft
      fftwf_execute(plan1);
      //      four1(new_data-1,buff->npts2,1);
      new_data[0] = (new_data[2]+new_data[2*buff->npts2-2])/2.0;
      new_data[1] = (new_data[3]+new_data[2*buff->npts2-1])/2.0;
      fftwf_execute(plan2);
      //four1(new_data-1,buff->npts2,-1);
      for(j=0;j<buff->npts2;j++){
	 buff->data[j*buff->npts*2+i] = new_data[j*2]/scale;
      }	   
    }
  }
  else{ // hypercomplex
    new_data = g_malloc(sizeof(float) * buff->npts2);
    scale = buff->npts2/2;

    plan1 = fftwf_plan_dft_1d(buff->npts2/2,(fftwf_complex *) new_data, (fftwf_complex *) new_data,FFTW_FORWARD,FFTW_MEASURE);
    plan2 = fftwf_plan_dft_1d(buff->npts2/2,(fftwf_complex *) new_data, (fftwf_complex *) new_data,FFTW_BACKWARD,FFTW_MEASURE);
    // copy out
    for(i=0;i<buff->npts*2  ;i++){
      for(j=0;j<buff->npts2/2;j++){
	new_data[j*2] = buff->data[2*j*buff->npts*2+i];
	new_data[j*2+1] = buff->data[(2*j+1)*buff->npts*2+i];
      }
      // do the ft
      fftwf_execute(plan1);
      //      four1(new_data-1,buff->npts2/2,1);
      new_data[0] = (new_data[2]+new_data[buff->npts2-2])/2.0;
      new_data[1] = (new_data[3]+new_data[buff->npts2-1])/2.0;
      fftwf_execute(plan2);
      //      four1(new_data-1,buff->npts2/2,-1);

      
      // copy back 
      for(j=0;j<buff->npts2/2;j++){
	buff->data[2*j*buff->npts*2+i] = new_data[j*2]/scale;
	buff->data[(2*j+1)*buff->npts*2+i] = new_data[j*2+1]/scale;
      }


    }
  }
  //  fprintf(stderr,"2dft about to free buffer\n");

  save_wisdom();
  fftwf_destroy_plan(plan1);
  fftwf_destroy_plan(plan2);
  g_free(new_data);
  cursor_normal(buff);

  
  return 0;
}



//////////////////////
GtkWidget* create_extras_frame()

{

  char title[UTIL_LEN];
  GtkWidget *table, *frame, *button;
  //  GSList *group;
  //  int i;
  long nu;



  /* arguments are homogeneous and spacing */
  
  table = gtk_table_new(12,6, TRUE); /* rows, columns */
 
  /*
   *  This is where all the process buttons are set up
   */ 


  /*
   *  phase shift for shear
   */
  nu=INCPH1;
  process_button[nu].adj= (GtkAdjustment *) gtk_adjustment_new( 0, -100000, 100000, 1, 2, 0 );
  g_signal_connect (G_OBJECT (process_button[nu].adj), "value_changed", G_CALLBACK (update_active_process_data), (void*) nu);
  button = gtk_spin_button_new( GTK_ADJUSTMENT(  process_button[nu].adj ), 1.00, 1);
  gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( button ), GTK_UPDATE_IF_VALID );
  gtk_table_attach_defaults(GTK_TABLE(table),button,2,3,nu-INCPH1,nu-INCPH1+1);
  gtk_widget_show( button );

  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu-INCPH1,nu-INCPH1+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle), 
      (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "phase for shear" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu-INCPH1,nu-INCPH1+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(phase_for_shear_and_display), process_button[nu].adj  );
  process_button[nu].func = phase_for_shear;
  gtk_widget_show(button);
  ////////////
  /*
   *  convert a +time/-time 2D dataset to hypercomplex
   *  first record of each pair becomes the sum, second is the difference
   */
  nu=PM_TO_HYPER;

  process_button[nu].button = gtk_check_button_new();
  gtk_table_attach_defaults(GTK_TABLE(table),process_button[nu].button,0,1,nu-INCPH1,nu-INCPH1+1);
  g_signal_connect(G_OBJECT(process_button[nu].button),"toggled",G_CALLBACK(process_button_toggle), 
      (void*) nu);
  gtk_widget_show(process_button[nu].button);
  button = gtk_button_new_with_label( "PM To Hyper" );
  gtk_table_attach_defaults(GTK_TABLE(table),button,1,2,nu-INCPH1,nu-INCPH1+1);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(plus_minus_to_hyper_and_display), process_button[nu].adj  );
  process_button[nu].func = plus_minus_to_hyper;
  gtk_widget_show(button);

  ///////////
  gtk_widget_show (table);

  snprintf(title,UTIL_LEN,"Extras");

  frame = gtk_frame_new(title);

  gtk_container_set_border_width(GTK_CONTAINER (frame),5);
  //  gtk_widget_set_size_request(frame,800,300);

  gtk_container_add(GTK_CONTAINER(frame),table);
  gtk_widget_show(frame);

  return frame;
}

gint plus_minus_to_hyper_and_display( GtkWidget *widget, GtkAdjustment *adj)
{
  dbuff *buff;
  gint result;
  
  result = plus_minus_to_hyper( widget, NULL );

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  draw_canvas( buff );
  return result;
}

gint plus_minus_to_hyper( GtkWidget *widget, double *val)
{
  dbuff *buff;
  gint  i,j;
  double *pbuff,*mbuff;
  //  printf("in pm_to_hyper\n");

  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("plus minus to hyper panic! buff is null!",TRUE);
    return 0;
  }

  
  // need memory to store 2 records
  pbuff = g_malloc(sizeof(double) *2*buff->npts);  // array to hold new data.
  mbuff = g_malloc(sizeof(double) *2*buff->npts);  // array to hold new data.
  
  for (i=0;i<buff->npts2/2;i++){
    // copy data out to spare buffer
    for(j=0;j<buff->npts*2;j++){
      pbuff[j] = buff->data[2*buff->npts*(2*i)+j];
      mbuff[j] = buff->data[2*buff->npts*(2*i+1)+j];;
    }
    
    for(j=0;j<buff->npts;j++){
      // do the sum
      buff->data[2*buff->npts*(2*i)+2*j] = pbuff[2*j]+mbuff[2*j];
      buff->data[2*buff->npts*(2*i)+2*j+1] = pbuff[2*j+1]+mbuff[2*j+1];
      // and do the difference - include 90 degree phase shift of direct dimension here.
      buff->data[2*buff->npts*(2*i+1)+2*j] = pbuff[2*j+1]-mbuff[2*j+1];
      buff->data[2*buff->npts*(2*i+1)+2*j+1] = -pbuff[2*j]+mbuff[2*j];
    }
  }
  
 
  g_free(pbuff);
  g_free(mbuff);
  return 0;  
  
}

gint phase_for_shear_and_display( GtkWidget *widget, GtkAdjustment *adj )
{
  dbuff *buff;
  gint result;
  double val;
  
  val = gtk_adjustment_get_value(adj);
  result = phase_for_shear( widget, &val );


  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];

  draw_canvas( buff );
  return result;
}


gint phase_for_shear( GtkWidget *widget, double *val )

{
  int i,j;
  dbuff *buff;
  double ph2;
  float s1,s2;
  /* this applies a phase shift ramp to shear a spectrum. Should be done
  after the direct dimension FT and direct dimension phasing, but
  before the indirect dimension FT. It assumes hypercomplex data.
  The parameter value is the same as daslp on the Varian.

  It is the increment in first order phase applied to subsequent (hypercomplex) records in the indirect dimension.
  */

  
  if( widget == NULL ) 
    buff = buffp[ upload_buff ];
  else 
    buff = buffp[ current ];
  if (buff == NULL){
    popup_msg("phase_for_shear panic! buff is null!",TRUE);
    return 0;  
  }
  //printf("in phase_for_shear, val is: %lf\n",*val);
  
  
  for( i=0; i<buff->npts2/2 ; i++ ){
    ph2 = *val * i *M_PI/180.;
    printf("ph2 is: %lf\n",ph2);

    for (j=0;j<buff->npts*2;j++){
      s1 = buff->data[buff->npts*2*2*i+j];
      s2 = buff->data[buff->npts*2*(2*i+1)+j];
      buff->data[buff->npts*2*2*i+j]= s1*cos(-ph2/2+ph2*(j/2)/buff->npts)
	+s2*sin(-ph2/2+ph2*(j/2)/buff->npts);
      buff->data[buff->npts*2*(2*i+1)+j]= -s1*sin(-ph2/2+ph2*(j/2)/buff->npts)
	+s2*cos(-ph2/2+ph2*(j/2)/buff->npts);
    }
 
  }
  
  return 0;
}
