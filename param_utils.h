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

/* param_utils.h
 * 
 * Part of the Xnmr Project
 * UBC Physics
 * April 2000
 *
 * These utility functions are used to pick the appropriate parameter
 * value out of the shared memory parameter string
 *
 * 
 * written by: Scott Nelson, Carl Michal
 */

#ifndef PARAM_UTILS_H
#define PARAM_UTILS_H

// these three are also defined in xnmr_types ?
#define MAX_PARAMETERS 100
#define PATH_LENGTH 800
#define UTIL_LEN 60

#define VERSION_LEN 30
#define PARAM_T_VAL_LEN 100
#define PARAM_NAME_LEN 50

typedef struct{
  char name[ PARAM_NAME_LEN ];
  char type;
  int i_val;
  int i_min;
  int i_max;
  int i_step;
  int i_page;
  char t_val[PARAM_T_VAL_LEN];
  double f_val;
  double f_min;
  double f_max;
  double f_step;
  double f_page;
  double unit;
  char unit_c;
  char unit_s[5]; // yes need five for e-15
  int f_digits;
  unsigned int size;
  int *i_val_2d;
  double *f_val_2d;
} parameter_t;

typedef struct {
  parameter_t parameter[ MAX_PARAMETERS ];
  int num_parameters;
  //  int tnpts; /* number of points in data set - used only temporarily in show_parameter_frame_mutex_wrap */
  long unsigned int num_acqs;  // this thing needs to be arrayable
  unsigned int num_acqs_2d;
  unsigned long sw;
  float dwell;
  char exec_path[ PATH_LENGTH ];
  char save_path[ PATH_LENGTH ];
} parameter_set_t;

#define PARAMETER_FORMAT_INT    "%s = %d\n"
#define PARAMETER_FORMAT_FLOAT  "%s = %g\n"
#define PARAMETER_FORMAT_TEXT_P   "%s = '%s'\n"
#define PARAMETER_FORMAT_TEXT_S   "%s = '%[^'\n]'\n"
#define PARAMETER_FORMAT_TEXT_O   "%s = %s'\n"
#define PARAMETER_FORMAT_DOUBLE "%s = %lg\n"
#define PARAMETER_FORMAT_DOUBLET "%s = '%lg'\n"
#define PARAMETER_FORMAT_DOUBLEP "%s = %.*f%s\n"
#define PARAMETER_2D_BREAK      ";\n"

/*
 *  Parameter file format is <name> <type> <default> <min> <max> <step> <page> [digits] <units> \n
 */

#define PARAMETER_FILE_FORMAT "%s %c"
#define PARAMETER_FILE_FORMAT_INT "%s i %d %d %d %d %d\n" 
//#define PARAMETER_FILE_FORMAT_FLOAT "%s f %g %g %g %g %g %d %c\n"
#define PARAMETER_FILE_FORMAT_DOUBLE "%s f %lg %lg %lg %lg %lg %d %c\n"
#define PARAMETER_FILE_FORMAT_TEXT "%s t %s\n"

int sfetch_float( char* params, char* name, float* var, unsigned int acqn_2d );
int sfetch_int( char* params, char* name, int* var, unsigned int acqn_2d );
int sfetch_text( char* params, char* name, char* var, unsigned int acqn_2d );
int sfetch_double( char* params, char* name, double* var, unsigned int acqn_2d );

// pfetch routines defined in xnmr_types because this file is *always* read in before dbuff is defined
int pfetch_float( parameter_set_t *param_set, char* name, double* var, unsigned int acqn_2d );
int pfetch_int( parameter_set_t *param_set, char* name, int* var, unsigned int acqn_2d );

int is_2d_param( char* params, char* name );
//returns 1 if parameter is 2d, 0 if not

int make_param_string( const parameter_set_t* p_set, char* dest );
void clear_param_set_2d( parameter_set_t* param_set );
int load_p_string( char* params, unsigned int acqs_2d, parameter_set_t* param_set );

void make_path( char* s );
void path_strcat(char *dest, char *source);
void path_strcpy(char *dest, const char *source);
void param_strcat(char *dest, char *source);

#endif


