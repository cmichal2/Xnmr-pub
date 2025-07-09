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

/* param_utils.c
 * 
 * These utility functions are used to pick the appropriate parameter
 * value out of the shared memory parameter string
 *
 * Xnmr software
 *
 * UBC Physics
 * April, 2000
 * 
 * written by: Scott Nelson, Carl Michal
 */



#include <ctype.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <glib.h>

#include "platform.h"
#include "param_utils.h"
#include "shm_data.h"



int sfetch_float( char* params, char* name, float* var, unsigned int acqn_2d )

{
  char* start;
  char s[PARAM_NAME_LEN],nname[PARAM_NAME_LEN+3];
  unsigned int i;
  char result = -1;
  char *breaker, *match;

  if ( params[0] != '\n') printf("sfetch, params doesn't start with nl\n");
  //  strncpy(nname,name,PARAM_NAME_LEN);
  //  strcat(nname," ="); //safe - for end, not for start.
  snprintf(nname,PARAM_NAME_LEN+3,"\n%s =",name); // safe!
  start = params;
  i=0;
  
  for ( i = 0; i <= acqn_2d ; i++){
    breaker = strstr ( start , PARAMETER_2D_BREAK ); // find the 2d line break
    if ( breaker == NULL) {
      breaker = &params[ strlen(params)-1 ]; // if there's no 2d break, look to end 
      i = acqn_2d;
    }
    else breaker += 1;
    match = strstr ( start , nname); 
    if (match < breaker-1 && match != NULL){
      sscanf( match+1, PARAMETER_FORMAT_FLOAT , s , var);
      result = 0;
    }
    start = breaker;
  }
  //  fprintf(stderr,"returning: %s, %f\n",name,*var);

  return result;
}

int sfetch_double( char* params, char* name, double* var, unsigned int acqn_2d )
{
  char* start;
  char s[PARAM_NAME_LEN],nname[PARAM_NAME_LEN+3];
  unsigned int i;
  char result = -1;
  char *match, *breaker;

  // both text double and true doubles come here...
  //  printf("coming into sfetch_double for %s, %i\n",name,acqn_2d);
  if ( params[0] != '\n') printf("sfetch, params doesn't start with nl\n");
  //  strncpy(nname,name,PARAM_NAME_LEN);
  //  strcat(nname," ="); //safe
  snprintf(nname,PARAM_NAME_LEN+3,"\n%s =",name); // safe!

  start = params;
  i=0;


  //parse the params string one line at a time
  for ( i = 0; i <= acqn_2d ; i++){
    breaker = strstr ( start , PARAMETER_2D_BREAK ); // find the 2d line break
    if ( breaker == NULL){
      breaker = &params[ strlen(params)-1 ]; // if there's no 2d break, look to end 
      i = acqn_2d;
    }
    else breaker += 1;
    match = strstr ( start ,nname); 
    if ((match < breaker-1) && (match != NULL)){
      //      printf("found match: %s\n",match);
      match += 1; // get past the \n
      if (match[strlen(nname)+1] == '\''){ // this is to deal with old text-style doubles
	sscanf(match,PARAMETER_FORMAT_DOUBLET, s , var);
      }
      else{
	sscanf( match, PARAMETER_FORMAT_DOUBLE , s , var);
	//	printf("just put %lf in var\n",*var);
      }
      result = 0;
    }
    start = breaker;
  }

  //  fprintf(stderr,"for record: %i returning %s, %f\n",acqn_2d,name,*var);

  return result;
}

int sfetch_int( char* params, char* name, int* var, unsigned int acqn_2d )

{
  char* start;
  char s[PARAM_NAME_LEN],nname[PARAM_NAME_LEN+3];
  unsigned int i;
  char result = -1;
  char *match, *breaker;

  if ( params[0] != '\n') printf("sfetch, params doesn't start with nl\n");
  start = params;
  i=0;
  //  strncpy(nname,name,PARAM_NAME_LEN);
  //  strcat(nname," =");  //safe
  snprintf(nname,PARAM_NAME_LEN+3,"\n%s =",name); // safe!

  //parse the params string one line at a time
  for ( i = 0; i <= acqn_2d ; i++){
    breaker = strstr ( start , PARAMETER_2D_BREAK ); // find the 2d line break
    if ( breaker == NULL){
      breaker = &params[ strlen(params)-1 ]; // if there's no 2d break, look to end 
      i = acqn_2d;
    }
    else breaker += 2;
    match = strstr ( start , nname); 
    if (match < breaker-1 && match != NULL){
      sscanf( match+1, PARAMETER_FORMAT_INT , s , var);
      //      fprintf(stderr,"sfetch int, record: %i, var: %s, i is %i, value: %i\n",acqn_2d,name,i,*var);
      result = 0;
    }
    start = breaker - 1;
  }
  //  fprintf(stderr,"returning: %s, %i for record %i\n",name,*var, acqn_2d);

  return result;
}

int sfetch_text( char* params, char* name, char* var, unsigned int acqn_2d )
{
  char* start;
  char s[PARAM_NAME_LEN],nname[PARAM_NAME_LEN+3];
  unsigned int i;
  char result = -1;
  char *match, *breaker;

  start = params;
  i=0;

  if ( params[0] != '\n') printf("sfetch, params doesn't start with nl\n");
  //  strncpy(nname,name,PARAM_NAME_LEN);
  //  strcat(nname," ="); //safe

  snprintf(nname,PARAM_NAME_LEN+3,"\n%s =",name); // safe!


  //parse the params string one line at a time
  for ( i = 0; i <= acqn_2d ; i++){
    breaker = strstr ( start , PARAMETER_2D_BREAK ); // find the 2d line break
    if ( breaker == NULL){
      breaker = &params[ strlen(params)-1 ]; // if there's no 2d break, look to end 
      i = acqn_2d;
    }
    else breaker += 1;
    match = strstr ( start , nname); 
    if (match < breaker-1 && match != NULL){
      if (sscanf( match+1, PARAMETER_FORMAT_TEXT_S , s , var) < 2){
	// means we couldn't find an opening quote.  try the old way
	sscanf(match+1, PARAMETER_FORMAT_TEXT_O, s, var);
      }
      if (var[0] == '\'') var[0] = 0;
      result = 0;
    }
    start = breaker;
  }
  //  fprintf(stderr,"returning: %s, %s\n",name,var);

  return result;
}



int is_2d_param( char* params, char* name )
{
  char *start;
  char compstring[PARAM_NAME_LEN+3];

  snprintf(compstring,PARAM_NAME_LEN+3,"\n%s =",name);

  start = strstr( params, PARAMETER_2D_BREAK );
  
  if( start == NULL )  // if no 2d break exists, its not a 2d param
    return 0;

  start = start -1; // start looking at the line break.

  if( strstr( start, compstring ) == NULL ) // if no substring match, its not a 2d param
    return 0;

  return 1;
}


int make_param_string( const parameter_set_t* p_set, char* dest )

{

  char p[ PARAMETER_LEN ];
  char s[PARAM_NAME_LEN + PARAM_T_VAL_LEN +4];
  int i;
  unsigned int acqn;
  int max_useful_records=0;


  strcpy( p, "\n" );

  //do the simple parameters first

  for( i=0; i< p_set->num_parameters; i++ ) {
    switch( p_set->parameter[i].type )
      {
      case 'i':	
	snprintf( s, PARAM_NAME_LEN,PARAMETER_FORMAT_INT, p_set->parameter[i].name, p_set->parameter[i].i_val );
	param_strcat( p, s );
	break;

      case 'f':
	snprintf( s, PARAM_NAME_LEN,PARAMETER_FORMAT_DOUBLEP, p_set->parameter[i].name, 
		 p_set->parameter[i].f_digits,p_set->parameter[i].f_val,p_set->parameter[i].unit_s);
	param_strcat( p, s );
	break;

      case 'I':
	snprintf( s, PARAM_NAME_LEN,PARAMETER_FORMAT_INT, p_set->parameter[i].name, p_set->parameter[i].i_val_2d[ 0 ] );
	param_strcat( p, s );
	if ( max_useful_records < p_set->parameter[i].size ) max_useful_records = p_set->parameter[i].size;
	break;

      case 'F':
	snprintf( s, PARAM_NAME_LEN,PARAMETER_FORMAT_DOUBLEP, p_set->parameter[i].name, 
		 p_set->parameter[i].f_digits,p_set->parameter[i].f_val_2d[ 0 ] ,p_set->parameter[i].unit_s);
	param_strcat( p, s );
	if ( max_useful_records < p_set->parameter[i].size ) max_useful_records = p_set->parameter[i].size;
	break;

      case 't':
	snprintf( s, PARAM_NAME_LEN + PARAM_T_VAL_LEN +4, PARAMETER_FORMAT_TEXT_P, p_set->parameter[i].name, p_set->parameter[i].t_val );
	printf("string to add is: %s\n",p_set->parameter[i].t_val);
	param_strcat( p, s );
	printf("param string is: %s\n",p);
	break;
	  
      default:
	fprintf(stderr, "invalid type for parameter %d\n, name: %s", i ,p_set->parameter[i].name);
	
	break;
      }
  }

  //now do the 2d parameters

  //  for( acqn = 1; acqn < p_set->num_acqs_2d; acqn++ ) {
  for( acqn = 1; acqn < max_useful_records; acqn++ ) {
    param_strcat( p, PARAMETER_2D_BREAK );
    for( i=0; i< p_set->num_parameters; i++ ) {
      switch( p_set->parameter[i].type )
	{
	case 'i':
	case 'f':
	case 't':
	  break;
	case 'I':
	  if( acqn < p_set->parameter[i].size ) {
	    if( p_set->parameter[i].i_val_2d[ acqn ] != p_set->parameter[i].i_val_2d[ acqn-1 ] ) {
	      snprintf( s, PARAM_NAME_LEN,PARAMETER_FORMAT_INT, p_set->parameter[i].name, p_set->parameter[i].i_val_2d[ acqn ] );
	      param_strcat( p, s );
	    }
	  }
	  break;
	case 'F':
	  if( acqn < p_set->parameter[i].size ) {
	    if( p_set->parameter[i].f_val_2d[ acqn ] != p_set->parameter[i].f_val_2d[ acqn-1 ] ) {
	      snprintf( s, PARAM_NAME_LEN,PARAMETER_FORMAT_DOUBLEP, p_set->parameter[i].name, 
		       p_set->parameter[i].f_digits,p_set->parameter[i].f_val_2d[ acqn ],p_set->parameter[i].unit_s);
	      param_strcat( p, s );
	    }
	  }
	  break;

	default:
	  fprintf(stderr, "invalid type for parameter %d\n", i );
	  break;
	}
    }
  }

  strncpy( dest, p, PARAMETER_LEN );

  return 0;

}

void clear_param_set_2d( parameter_set_t* param_set )
{
int i;

 for (i=0;i<param_set->num_parameters;i++){
   if (param_set->parameter[i].type == 'I'){
     if (param_set->parameter[i].i_val_2d != NULL)
       free(param_set->parameter[i].i_val_2d);
     param_set->parameter[i].type = 'i';
   }
   else if (param_set->parameter[i].type == 'F'){
     if ( param_set->parameter[i].f_val_2d != NULL)
       free(param_set->parameter[i].f_val_2d);
     param_set->parameter[i].type = 'f';
   }
 }
 
 



}

int load_p_string( char* params, unsigned int acqs_2d, parameter_set_t* param_set )

{
  int i;
  int j;

  clear_param_set_2d( param_set );

  for( i=0; i < param_set->num_parameters; i++ ) {
    switch( param_set->parameter[i].type )
      {
      case 'I':
      case 'i':
	switch( is_2d_param( params, param_set->parameter[i].name )  )
	  {
	  case 0:
	    sfetch_int( params, param_set->parameter[i].name, &param_set->parameter[i].i_val,0 );
	    break;
	    
	  case 1:
	    sfetch_int( params, param_set->parameter[i].name, &param_set->parameter[i].i_val,0 );
	    param_set->parameter[i].size = acqs_2d;
	    param_set->parameter[i].type = 'I';
	    param_set->parameter[i].i_val_2d = malloc( acqs_2d * sizeof(gint) );
	    //	    fprintf(stderr,"load_p_string: malloc\n");
	    for( j=0; j<acqs_2d; j++ )
	      sfetch_int( params, param_set->parameter[i].name, &param_set->parameter[i].i_val_2d[ j ], j );
	    break;
	    
	  default:
	    fprintf(stderr, "bad result in attempting to read parameters from shm\n" );
	    break;
	  }
	break;
	
      case 'F':
      case 'f':
	switch( is_2d_param( params, param_set->parameter[i].name )  )
	  {
	  case 0:
	    if (sfetch_double( params, param_set->parameter[i].name, &param_set->parameter[i].f_val,0 ) == 0)
	      param_set->parameter[i].f_val /= param_set->parameter[i].unit; // only do this if we actually find the right value!!
	    //	    fprintf(stderr,"param: %s, unit: %f\n",param_set->parameter[i].name,param_set->parameter[i].unit);
	    break;
	    
	  case 1:
	    if (sfetch_double( params, param_set->parameter[i].name, &param_set->parameter[i].f_val,0 ) == 0)
	      param_set->parameter[i].f_val /= param_set->parameter[i].unit;
	    param_set->parameter[i].size = acqs_2d;
	    param_set->parameter[i].type = 'F';
	    param_set->parameter[i].f_val_2d = malloc( acqs_2d * sizeof(double) );
	    //	    fprintf(stderr,"load_p_string: malloc\n");

	    for( j=0; j<acqs_2d; j++ ){
	      sfetch_double( params, param_set->parameter[i].name, &param_set->parameter[i].f_val_2d[ j ], j );
	      param_set->parameter[i].f_val_2d[j] /= param_set->parameter[i].unit;
	      //	      fprintf(stderr,"param: %s, unit: %f\n",param_set->parameter[i].name,param_set->parameter[i].unit);
	    }
	    break;
	    
	  default:
	    fprintf(stderr, "bad result in attempting to read parameters from shm\n" );
	    break;
	  }
	break;

      case 'T':
      case 't':
	switch( is_2d_param( params, param_set->parameter[i].name )  )
	  {
	  case 0:
	    sfetch_text( params, param_set->parameter[i].name, param_set->parameter[i].t_val,0 );
	    break;
	    
	  case 1:
	    fprintf(stderr, "Error, can't have a 2d text parameter: %s\n",param_set->parameter[i].name );
	    break;
	    
	  default:
	    fprintf(stderr, "bad result in attempting to read parameters from shm\n" );
	    break;
	  }
	break;
	
      default:
	fprintf(stderr, "invalid parameter type: %d, %s\n",param_set->parameter[i].type,param_set->parameter[i].name );
	break;
      }
    
  }  //end for
  return 0; 
}

void make_path( char* s )

{
  char c;

  c = s[ strlen( s )-1 ];

  if( c != PATH_SEP )
    path_strcat( s, DPATH_SEP );
  return;

}



// routines to fetch parameter values by name from designated buffer's panel

int pfetch_float( parameter_set_t *param_set, char* name, double* var, unsigned int acqn_2d )

{

  unsigned int i;

  // first see if we have a parameter name match

  for (i=0;i< param_set->num_parameters;i++){
    if (strcmp(name,param_set->parameter[i].name)==0) {
      //      fprintf(stderr,"in pfetch, found a match to: %s\n",param_set->parameter[i].name);
      
      // if it's a 1d float we're golden
      
      if (param_set->parameter[i].type == 'f'){
	*var = param_set->parameter[i].f_val*param_set->parameter[i].unit;
	//	fprintf(stderr,"its a 1-d float value returned: %f\n",*var);
	return TRUE;
      }
      if (param_set->parameter[i].type != 'F'){
	//	fprintf(stderr,"param is of type: %i\n",param_set->parameter[i].type);
	return 0;
      }
      
      // ok so its 2d.
      if ( param_set->parameter[i].size >= acqn_2d)
	*var = param_set->parameter[i].f_val_2d[acqn_2d]*param_set->parameter[i].unit;
      
      else 
	*var = param_set->parameter[i].f_val_2d[param_set->parameter[i].size]*param_set->parameter[i].unit;
      return TRUE;
    }

  }
  // should never get here

return FALSE;
}

int pfetch_int( parameter_set_t *param_set, char* name, int* var, unsigned int acqn_2d )

{

  unsigned int i;

  // first see if we have a parameter name match

  for (i=0;i< param_set->num_parameters;i++){
    if (strcmp(name,param_set->parameter[i].name)==0) {
      //      fprintf(stderr,"in pfetch, found a match to: %s\n",param_set->parameter[i].name);
      
      // if it's a 1d float we're golden
      
      if (param_set->parameter[i].type == 'i'){
	*var = param_set->parameter[i].i_val;
	//	fprintf(stderr,"its a 1-d float value returned: %f\n",*var);
	return TRUE;
      }
      if (param_set->parameter[i].type != 'I'){
	//	fprintf(stderr,"param is of type: %i\n",param_set->parameter[i].type);
	return 0;
      }
      
      // ok so its 2d.
      if ( param_set->parameter[i].size >= acqn_2d)
	*var = param_set->parameter[i].i_val_2d[acqn_2d];
      
      else 
	*var = param_set->parameter[i].i_val_2d[param_set->parameter[i].size];
      return TRUE;
    }

  }
  // should never get here

return FALSE;
}

void path_strcat(char *dest, char *source){
  // routine assumes that the dest string is of length PATH_LENGTH.
  //  fprintf(stderr,"in path_strcat, lengths: %i %i\n",strlen(dest),strlen(source));

  if ( strlen(dest) + strlen(source) >= PATH_LENGTH){ //need room for the 0 at end
    fprintf(stderr,"Overrun while appending string %s onto %s\n",source,dest);
    strncat(dest,source,PATH_LENGTH-1-strlen(dest));
    //    dest[strlen(dest)]=0; // appears to be unnecessary.
    return;
  }
  strcat(dest,source);
  return;
}

void param_strcat(char *dest, char *source){
  // routine assumes that the dest string is of length PARAMETER_LEN.
  //  fprintf(stderr,"in param_strcat, lengths: %i %i\n",strlen(dest),strlen(source));

  if ( strlen(dest) + strlen(source) >= PARAMETER_LEN){ //need room for the 0 at end
    fprintf(stderr,"Overrun while appending string %s onto %s\n",source,dest);
    strncat(dest,source,PARAMETER_LEN-1-strlen(dest));
    //    dest[strlen(dest)]=0; // appears to be unnecessary.
    return;
  }
  strcat(dest,source);
  return;
}


void path_strcpy(char *dest,const char*source){
  // again assume that dest string is of length PATH_LENGTH
  
  if (source ==  NULL){
    printf("FixMe: got a null path in path_strcpy\n"); 
    return;
  }
  if ( strlen(source) >= PATH_LENGTH){
    fprintf(stderr,"Overrun while copy string %s into %s\n",source,dest);
    strncpy(dest,source,PATH_LENGTH-1);
    dest[PATH_LENGTH]=0;
    return;
  }
  strcpy(dest,source);
  return;
}
