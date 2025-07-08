/* buff.h include file with data structures and so forth 
 *
 * UBC Physics
 * April, 2000
 * 
 * written by: Carl Michal, Scott Nelson
 */

#ifndef BUFF_H
#define BUFF_H

#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include <semaphore.h>
#include "param_f.h"
#include "process_f.h"
#include "xnmr_types.h"

#define MAX_BUFFERS 50

#define  BUFF_KEY "buffer key"

// maximum number of peaks for fitting.
#define MAX_FIT 20

/* display styles */

#define SLICE_ROW 0
#define SLICE_COL 1
#define RASTER 2
#define STACKED 3
#define CONTOUR 4

/* for baseline_spline: */
#define PICK_SPLINE_POINTS 0
#define DO_SPLINE 1
#define SHOW_SPLINE_FIT 2
#define CLEAR_SPLINE_POINTS 3
#define UNDO_SPLINE 4

typedef struct {
  float ophase0,ophase1;
  float last_phase1;
  float pivot; /* between 0 and 1 */
  int buffnum;
  float *data,*data2;
  GtkWidget *ok,*apply_all,*update,*cancel;
  GtkWidget *pscroll0,*pscroll1;
  GtkWidget *pbut,*mbut;
  int is_open;
} phase_data_struct;

typedef struct {
  GtkWidget *dialog,*s_buff1,*s_buff2,*dest_buff,
    *s_record1,*s_record2,*dest_record,*apply,*close,*mult1,*mult2;
  char index[MAX_BUFFERS]; /* the index numbers tell you which buffer is 
			      in which spot in the list for the buffer number.
			      for the destination, there's always a 'new' at the top */
  int  s_rec_c1,s_rec_c2,dest_rec_c; 
  // number of records in the list for the combo box
  char shown ;
    } add_sub_struct;

typedef struct{
  GtkWidget *dialog,*s_buff,*d_buff,*s_record,*d_record,*components,*add_dialog,
    *start_clicking,*end_clicking,*run_fit,*run_fit_range,*precalc,*close,*enable_proc_broad,*store_fit,
    *center[MAX_FIT],*amplitude[MAX_FIT],*gauss_wid[MAX_FIT],*include_imag,
    *lorentz_wid[MAX_FIT],*enable_gauss[MAX_FIT],*enable_lorentz[MAX_FIT],*hbox[MAX_FIT];
  int shown,num_components,s_rec,d_rec;
} fitting_struct;


typedef struct{
  GtkWidget *dialog,*label;
  GtkListStore *list;
  GtkTreeIter iter;
  GtkTreeSelection *select;
  int num_queued;
    } queue_struct;


enum
  {
    BUFFER_COLUMN,
    FILE_COLUMN,
    N_COLUMNS,
  };

typedef struct{
  char iline[PATH_LENGTH];
  char oline[PATH_LENGTH];
  unsigned char source;
  unsigned char rval;
  sem_t sem;
  int bnum;
} script_data;
/* global variables */

extern int current;
extern int last_current;
extern int num_buffs;
extern int active_buff;
extern dbuff *buffp[MAX_BUFFERS];
extern char data_path[PATH_LENGTH];
extern char from_do_destroy_all;
extern char no_update_open;
extern GtkWidget *panwindow;
extern int from_make_active;

/* function prototypes */
dbuff *create_buff();
//void file_open(dbuff *buff,int action,GtkWidget *widget);
void file_open(GtkAction *action,dbuff *buff);
//void file_close(dbuff *buff,int action,GtkWidget *widget);
void file_close(GtkAction *action,dbuff *buff);
//void file_export(dbuff *buff,int action,GtkWidget *widget);
void file_import_text(GtkAction *action,dbuff *buff);
void file_export(GtkAction *action,dbuff *buff);
void file_export_binary(GtkAction *action,dbuff *buff);
void file_export_image(GtkAction *action,dbuff *buff);
void file_export_magnitude_image(GtkAction *action,dbuff *buff);
//void file_save(dbuff *buff,int action,GtkWidget *widget);
void file_save(GtkAction *action,dbuff *buff);
//void file_save_as(dbuff *buff,int action,GtkWidget *widget);
void file_save_as(GtkAction *action,dbuff *buff);
//void file_new(dbuff *buff,int action,GtkWidget *widget);
void file_new(GtkAction *action,dbuff *buff);
//void file_exit(dbuff *buff,int action,GtkWidget *widget);
void file_exit(GtkAction *action, dbuff *buff);
//void file_append(dbuff *buff,int action,GtkWidget *widget);
int file_append(GtkAction *action, dbuff *buff);


//void do_scales(dbuff *buff,int action,GtkWidget *widget);
void store_scales(GtkAction *action,dbuff *buff);
void apply_scales(GtkAction *action,dbuff *buff);
void user_scales(GtkAction *action,dbuff *buff);
gint do_user_scales(GtkWidget *widget, float *range);
gint do_user_scales(GtkWidget *widget, float *range);
gint do_wrapup_user_scales(GtkWidget *widget, dbuff *buff);
gint do_update_scales(GtkWidget *widget, dbuff *buff);
//void toggle_disp(dbuff *buff,int action,GtkWidget *widget);
void toggle_real(GtkAction *action,dbuff *buff);
void toggle_imag(GtkAction *action,dbuff *buff);
void toggle_mag(GtkAction *action,dbuff *buff);
void toggle_base(GtkAction *action,dbuff *buff);
void toggle_points(GtkAction *action,dbuff *buff);

#if GTK_MAJOR_VERSION == 2
gint expose_event(GtkWidget *widget,GdkEventExpose *event,dbuff *buff);
#else
gint expose_event(GtkWidget *widget,cairo_t *event,dbuff *buff);
#endif
gint configure_event(GtkWidget *widget,GdkEventConfigure *event,
			 dbuff *buff);
void draw_oned(dbuff *buff,float extrayoff,float extraxoff,float *data
	       ,int npts);
void draw_oned2(dbuff *buff,float extrayoff,float extraxoff);
void draw_canvas(dbuff *buff);
gint destroy_buff(GtkWidget *widget,GdkEventAny *event,dbuff *num);
gint full_routine(GtkWidget *widget,dbuff *buff);
gint expand_routine(GtkWidget *widget,dbuff *buff);
gint expandf_routine(GtkWidget *widget,dbuff *buff);
gint expand_press_event (GtkWidget *widget, GdkEventButton *event,dbuff *buff);
gint expandf_press_event (GtkWidget *widget, GdkEventButton *event,dbuff *buff);

gint Bbutton_routine(GtkWidget *widget,dbuff *buff);
gint bbutton_routine(GtkWidget *widget,dbuff *buff);
gint Sbutton_routine(GtkWidget *widget,dbuff *buff);
gint sbutton_routine(GtkWidget *widget,dbuff *buff);
gint auto_routine(GtkWidget *widget,dbuff *buff);
gint autocheck_routine(GtkWidget *widget,dbuff *buff);
gint do_auto(dbuff *buff);
gint do_auto2(dbuff *buff);
gint offset_routine(GtkWidget *widget,dbuff *buff);
gint offset_press_event (GtkWidget *widget, GdkEventButton *event,dbuff *buff);

gint scroll_in_win_event(GtkWidget *widget,GdkEventScroll *event,dbuff *buff);
gint press_in_win_event(GtkWidget *widget,GdkEventButton *event,dbuff *buff);

void show_active_border();
gint phase_buttons(GtkWidget *widget,gpointer data);

gint hyper_check_routine(GtkWidget *widget,dbuff *buff);
gint complex_check_routine(GtkWidget *widget,dbuff *buff);

gint plus_button(GtkWidget *widget,dbuff *buff);
gint minus_button(GtkWidget *widget,dbuff *buff);
gint row_col_routine(GtkWidget *widget,dbuff *buff);

void draw_raster(dbuff *buff);
gint slice_2D_routine(GtkWidget *widget,dbuff *buff);
gint buff_resize( dbuff* buff, int npts1, int npts2 );

gint do_load( dbuff* buff, char* path, int fid);


gint do_save( dbuff* buff, char* fileN );

gint check_overwrite( dbuff* buff, char* path );

gint pix_to_x(dbuff * buff,int xval);
gint pix_to_y(dbuff * buff,int yval);
gint x_to_pix(dbuff * buff,int xpt);
gint y_to_pix(dbuff * buff,int ypt);

void set_window_title(dbuff *buff);
gint set_cwd(char *dir);
void update_param_win_title(parameter_set_t *param_set);
gint put_name_in_buff(dbuff *buff,char *fname);


//void signal2noise(dbuff *buff,int action,GtkWidget *widget);
void signal2noise(GtkAction *action,dbuff *buff);
void s2n_press_event(GtkWidget *widget,GdkEventButton *event,dbuff *buff);
//void signal2noiseold( dbuff *buff, int action, GtkWidget *widget );
void signal2noiseold(GtkAction *action, dbuff *buff);
//void integrate(dbuff *buff,int action,GtkWidget *widget);
void integrate(GtkAction *action, dbuff *buff);
void integrate_press_event(GtkWidget *widget,GdkEventButton *event,dbuff *buff);
//void integrateold( dbuff *buff, int action, GtkWidget *widget );
void integrateold(GtkAction *action, dbuff *buff);
void integrate_from_file( dbuff *buff, int action, GtkWidget *widget );
//void clone_from_acq(dbuff *buff, int action, GtkWidget *widget );
void clone_from_acq(GtkAction *action,dbuff *buff);
//void set_sf1(dbuff *buff, int action, GtkWidget *widget);
void set_sf1(GtkAction *action,dbuff *buff);
//void calc_rms(dbuff *buff, int action, GtkWidget *widget);
void calc_rms(GtkAction *action,dbuff *buff);
//void add_subtract(dbuff *buff, int action, GtkWidget *widget);
void add_subtract(GtkAction *action,dbuff *buff);
//void fitting(dbuff *buff, int action, GtkWidget *widget);
void fitting(GtkAction *action,dbuff *buff);

//void reset_dsp_and_synth(dbuff *buff, int action, GtkWidget *widget);
void reset_dsp_and_synth(GtkAction *action,dbuff *buff);

void do_svd_fit_baseline(GdkEventButton * action, dbuff *buff, int records);
void svd_fit_baseline(GdkEventButton * action, dbuff *buff);
void svd_fit_baseline_all(GdkEventButton * action, dbuff *buff);
void baseline_spline(dbuff *buff, GdkEventButton * action, GtkWidget *widget);
void pick_spline_points(GtkAction *action,dbuff *buff);
void do_spline(GtkAction *action,dbuff *buff);
void clear_spline(GtkAction *action,dbuff *buff);
void undo_spline(GtkAction *action,dbuff *buff);
void show_spline_fit(GtkAction *action,dbuff *buff);


char get_ch1(dbuff *buff);
char get_ch2(dbuff *buff);

void set_ch1(dbuff *buff,char ch1);
void set_ch2(dbuff *buff,char ch2);

void check_for_overrun_timeout( gpointer data);
void make_active(dbuff *buff);
gint channel_button_change(GtkWidget *widget,dbuff *buff);

void add_sub_changed(GtkWidget *widget,gpointer data);
void fit_data_changed(GtkWidget *widget,gpointer data);
void add_sub_buttons(GtkWidget *widget,gpointer data);
void fitting_buttons(GtkWidget *widget,gpointer data);
gint hide_add_sub(GtkWidget *widget,gpointer data);
gint hide_fit(GtkWidget *widget,gpointer data);
void set_sf1_press_event(GtkWidget *widget, GdkEventButton *event,dbuff *buff);

void calc_spectrum_residuals(int *n,int *p,float *x,int *nf, float *r,int *lty,float *ty,void *uf);
void add_gauss_lorentz_line(float center,float amp,float gauss_wid,float lorentz_wid,float *spect,int np,float dwell);

void queue_expt(GtkAction *action, dbuff *buff);
void queue_window(GtkAction *action, dbuff *buff);
void remove_queue(GtkWidget *widget, gpointer dum);
void set_queue_label();

void readscript(GtkAction *action,dbuff *buff);
void *readscript_thread_routine(void *buff);
void *readsocket_thread_routine(void *buff);
void *readscript2_thread_routine(void *buff);
void *readsocket2_thread_routine(void *buff);
//int script_handler(char *input,char *output,int source,int *bnum);
void script_handler(script_data *myscript_data);
gint script_notify_acq_complete();
void socket_script(GtkAction *action,dbuff *buff);
void socket_script2(GtkAction *action,dbuff *buff);
void shim_integrate(GtkWidget *action, dbuff *buff);
gint do_shim_integrate(dbuff *buff,double *int1,double *int2,double *int3);
void scale_data(dbuff *buff,int pt,float scale);
void first_point_auto_phase();


void zero_points(GtkAction *action,dbuff *buff);
void zero_points_handler(dbuff *buff, GdkEventButton * action, GtkWidget *widget);
int grab_canvas(dbuff *buff, int id);
int ungrab_canvas(dbuff *buff, int id);
void fit_add_components(dbuff *buff, GdkEventButton  *action, GtkWidget *widget);
gint motion_event (GtkWidget *widget, GdkEventButton *event,dbuff *buff);
void do_integrate(dbuff *buff);



// this lets the unix sockets stuff compile under MinGW, but it doesn't run.
#ifdef MINGW
#define UNIX_PATH_LEN 108
typedef struct sockaddr_un {
  unsigned short sun_family; /* address family AF_LOCAL/AF_UNIX */
  char sun_path[UNIX_PATH_LEN]; /* 108 bytes of socket address */
} ;
 
/* Evaluates the actual length of `sockaddr_un' structure. */
 #define SUN_LEN(p) ((size_t)(((struct sockaddr_un *) NULL)->sun_path) \
		      + strlen ((p)->sun_path))
#endif 

#endif


