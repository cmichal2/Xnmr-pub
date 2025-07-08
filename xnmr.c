/* Xnmr.c
 *
 * X windows NMR Control center - main program
 * 
 * UBC Physics
 * April, 2000
 * 
 * written by: Carl Michal, Scott Nelson
 */
/*

 'ported' for GTK+-2.0 Oct 27, 2002 CM
    not much to change but:

1)    array box didn't resize correctly,
 replaced:    
    gtk_window_set_policy( GTK_WINDOW( popup_data.win ), 0, 0, 1 );   //This makes the window autoshrink 
 with:
    gtk_window_set_resizable(GTK_WINDOW(popup_data.win),0);
in param_f.c

2) Also in param_f.c, param_spin_pressed type changed to gint and now returns false.  
It was preventing the cursor from landing in the box.

3) in show_2d_popup there was a break missing between i,I and f,F.
4) make_active moved to before do_load call in do_load_wrapper to avoid 
having the active buffer hiding the dialogs.

5) had to change a DIALOG to a POPUP to make it compile.
6) biggest changes in the Makefile, change from gtk-config --cflags and --libs
to pkg-config --cflags gtk+-2.0 and --libs gthread-2.0 gtk+-2.0 

7) changed gtk_accel_group_attach to gtk_window_add_accel_group



Oct 24, 2011
- GtkFunction ->GSourceFunc

*/
/*


April 23, 2013 - start work for migrating to GTK3.

on todo list: 

1) redo drawing in cairo.

n) cursor_busy and cursor_normal borked?


*/


#include "xnmr.h"
#include "buff.h"
#include "panel.h"
#include "xnmr_ipc.h"
#include "p_signals.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <signal.h>
#ifndef MINGW
#include <sys/wait.h>
#endif
#include <stdio.h>
#include <gtk/gtk.h>
#include <getopt.h>
#include <string.h>
#ifndef MINGW
#include <sys/shm.h>
#endif
#include <unistd.h>
/*
 *  Global Variables
 */

#if GTK_MAJOR_VERSION == 2
#define GdkRGBA GdkColor
#endif

GdkRGBA colours[NUM_COLOURS+EXTRA_COLOURS]; // matches in xnmr.h
//GdkGC *colourgc;
phase_data_struct phase_data;
add_sub_struct add_sub;
fitting_struct fit_data;
queue_struct queue;

int phase_npts=0;
GtkWidget *phase_dialog,*freq_popup,*fplab1,*fplab2,*fplab3,*fplab4,*fplab5;

GtkAdjustment *phase0_ad,*phase1_ad;
float phase0,phase1;
float phase20,phase21;
GdkCursor *cursorclock;
char no_acq=FALSE;
GtkWidget *panwindow;
script_widget_type script_widgets;

char from_do_destroy_all=0; // global flags to figure out what to do on exit.


extern int main(int argc,char *argv[]);

int main(int argc,char *argv[])
//int MAIN__(int argc,char *argv[])
{

  int i,ic;
  int result,result_shm;
  GtkWidget  *pantable;
  GtkWidget *button,*hbox,*vbox;
  GtkAdjustment *adjust;
  GtkWidget *label;
  char title[UTIL_LEN],command[PATH_LENGTH];
  int width,height;
  GtkTreeViewColumn *column;
  GtkWidget *tree;
  GtkCellRenderer *renderer;

  //  int timeout_tag;
  //  gpointer timeout_data;

  int ar,longindex;
  struct option cmds[]={
    {
      "noacq",
      0,
      NULL,
      'n'
    }
  }
  ;

#ifndef MINGW
  //first, we have to block SIGUSR1 in case ACQ is already launched and running
  // or just not enable the signals till later?
  //block_signal();
  { // May 6, 2006 CM
    sigset_t sigset;
    
    sigemptyset(&sigset);
    sigaddset(&sigset,SIG_UI_ACQ);
    sigaddset(&sigset,SIGQUIT);
    sigaddset(&sigset,SIGTERM);
    sigaddset(&sigset,SIGINT);
    sigaddset(&sigset,SIGTTIN);
    
    sigprocmask(SIG_BLOCK,&sigset,NULL);
  }
#endif

  /* initialize the gtk stuff */

  //    gdk_threads_init();
  // mutex to ensure that routines added with idle_add don't collide.
    gtk_init(&argc, &argv);

#ifndef MINGW
#ifndef DUELIME
  // see if /dev/PP_irq0 exists.  if not, then imply noacq.
  {
    struct stat sstat;
    if (stat("/dev/PP_irq0",&sstat) == -1){
      no_acq = TRUE;
      printf("couldn't find /dev/PP_irq0, forcing noacq\n");
    }
    else
      printf("found /dev/PP_irq0\n");
  }
#endif
#else
  no_acq = TRUE;
#endif

  /* look for command line arguments  that gtk didn't want*/
  do{
    ar=getopt_long(argc,argv,"n",cmds,&longindex);
    if (ar == 'n') {
      fprintf(stderr,"got noacq\n");
      no_acq=TRUE;
    }
  }while (ar !=EOF );

#if GTK_MAJOR_VERSION == 2
  gtk_rc_parse("/usr/share/Xnmr/config/xnmrrc");
  path_strcpy(command,getenv(HOMEP));
  path_strcat(command,"/.xnmrrc");
	 
  //  fprintf(stderr,"looking for rc file: %s\n",command);
  gtk_rc_parse( command );
#else
  //  GtkStyleContext *context;
  GtkCssProvider *provider;
  GdkScreen *screen;
  GdkDisplay *display;

  provider=gtk_css_provider_new();

  // these two should work but don't
  //  context=gtk_widget_get_style_context(panwindow);
  //    gtk_style_context_add_provider(context, GTK_STYLE_PROVIDER(provider),
  //				 GTK_STYLE_PROVIDER_PRIORITY_USER);

  display = gdk_display_get_default ();
  screen = gdk_display_get_default_screen (display);                                                                                   
  gtk_style_context_add_provider_for_screen (screen, GTK_STYLE_PROVIDER (provider),    GTK_STYLE_PROVIDER_PRIORITY_USER);

  gtk_css_provider_load_from_data(GTK_CSS_PROVIDER(provider),
				  "GtkButton#mybutton {\n"
				  "background-image: none;\n"
				  "}\n"
				  "GtkButton#mybutton:active {\n"
				  " background-color: #f00\n"
				  "}\n"
				  "GtkButton#mybutton:hover {\n" // icing
 				  " background-color: shade (@bg_color,1.04);\n"
 				  "}\n"
 				  "GtkButton#mybutton:active:hover {\n" // icing
 				  " background-color: #f33;\n"
 				  "}\n"
				  "GtkArrow#littlearrow {\n"
				  "-GtkArrow-arrow-scaling:  0.4\n;"
				  "}\n"
				  "GtkArrow#bigarrow {\n"
				  "-GtkArrow-arrow-scaling:  0.9\n;"
				  "}\n",
				  -1,NULL);
  g_object_unref(provider);
#endif
  

  /* initialize my stuff */
  for(i=0;i<MAX_BUFFERS;i++)
    {
    buffp[i]=NULL;
    }



  /*
   *  Start acq, etc
   */



  result_shm=-1;
#ifndef MINGW
  if (!no_acq){

    result_shm = xnmr_init_shm();  //this tells us whether acq is already launched


    if (result_shm != 0){                // shared mem exists
      fprintf(stderr,"shm already exists, looking for running Xnmr\n");
      if ( data_shm->ui_pid >0 ){   // pid in shm is valid
	path_strcpy(command,"ps auxw|grep -v grep|grep Xnmr|grep ");
	snprintf(&command[strlen(command)],PATH_LENGTH-strlen(command),"%i", data_shm->ui_pid);
	// fprintf(stderr,"using command: %s\n",command);
	ic = system(command); // returns zero when it finds something, 256 if nothing?
	if (ic == 0 ) { // Xnmr is still running
	  no_acq = TRUE; 
	  g_idle_add((GSourceFunc) popup_msg_wrap,"There appears to be another living Xnmr, started noacq");
	  // shmdt((char *)data_shm);  if we don't detach, then we should be able to clone from acq.
	  // not detaching does cause one problem - if we quit the active Xnmr after a second has attached,
	  // can't restart an active session because the shm exists.

	}
	else{ // try a -m on the ps command to show threads
	  path_strcpy(command,"ps auxwm|grep -v grep|grep Xnmr|grep ");
	  snprintf(&command[strlen(command)],PATH_LENGTH-strlen(command),"%i", data_shm->ui_pid);
	  ic = system(command); // returns zero when it finds something, 256 if nothing?
	  if (ic == 0 ) { // Xnmr is still running
	    no_acq = TRUE; 
	    g_idle_add((GSourceFunc) popup_msg_wrap,"There appears to be another living Xnmr, started noacq");
	  }
	}
      }
    }
  }

  //  popup_msg("Caution: new version using Analog Devices DSP receiver");
  if ((strcmp(":0.0",getenv("DISPLAY")) != 0) && strcmp(":0",getenv("DISPLAY")) != 0){
    fprintf(stderr,"Your display is set to go elsewhere???\n");
  }
  

	     
  if (!no_acq){ // want to have an acq running

    ic = 0;
    // now see if there is a living acq
    if ( data_shm->acq_pid > 0){ // there is a valid pid
      path_strcpy(command,"ps auxw|grep -v grep|grep [acq]|grep ");
      snprintf(&command[strlen(command)],PATH_LENGTH-strlen(command),"%i",data_shm->acq_pid);
      //      fprintf(stderr,"using command: %s\n",command);
      ic = system(command); // returns zero when it finds something, 256 if nothing?
      if (ic == 0 ) { //acq is still alive
	g_idle_add((GSourceFunc) popup_msg_wrap,"There appears to be a running acq");
      }
    }
    if (data_shm->acq_pid <1 || ic != 0){ 
	fprintf(stderr, "Xnmr will start acq\n" );
	data_shm->acq_pid = -1;
	
	init_ipc_signals();


	start_acq();
	/*	if ( wait_for_acq() != ACQ_LAUNCHED ){
	  fprintf(stderr,"Acq not launched successfully???\n");
	  g_idle_add((GSourceFunc) popup_msg_wrap,"Trouble starting up acq: started noacq");
	  no_acq = TRUE;
	  } */
    }
  }
#else
  no_acq = TRUE;
#endif

  /* build the notebook/panel window */
  panwindow=gtk_window_new( GTK_WINDOW_TOPLEVEL );

  
  
  

  gtk_window_set_default_size(GTK_WINDOW(panwindow),1000,300);

  g_signal_connect(G_OBJECT(panwindow),"delete_event",
		     G_CALLBACK (destroy_all),NULL);

  pantable = create_panels( );

  gtk_container_add(GTK_CONTAINER(panwindow),pantable);

  //  gtk_window_set_position(GTK_WINDOW(panwindow),GTK_WIN_POS_NONE);
  //  gtk_window_set_gravity(GTK_WINDOW(panwindow),GDK_GRAVITY_NORTH_WEST);
  gtk_window_set_icon_from_file(GTK_WINDOW(panwindow),ICON_PATH,NULL);

  // want size of buffer window to set placement of panel window.
  // so postpone placing this window
  
  script_widgets.acquire_notify = 0;

  /* create a first buffer */

#ifndef MINGW
  path_strcpy(command,getenv(HOMEP));
  make_path( command);
  path_strcat(command,"Xnmr/data/");
  result = chdir(command);

  if (result == -1){
    fprintf(stderr,"creating directories...");
    path_strcpy (command,getenv(HOMEP));
    path_strcat(command,"/Xnmr");
#ifndef MINGW
    result = mkdir(command,0755);
#else
    result = mkdir(command);
#endif
    if (result != 0) perror("making ~/Xnmr:");
    path_strcat(command,DPATH_SEP "data");
#ifndef MINGW
    result = mkdir(command,0755);
#else 
    result = mkdir(command);
#endif
    if (result != 0) perror("making ~/Xnmr/data:");
  }
#endif
  /* build the queue window */

  queue.dialog = gtk_dialog_new();
  gtk_window_set_transient_for(GTK_WINDOW(queue.dialog),GTK_WINDOW(panwindow));
  gtk_window_set_position(GTK_WINDOW(queue.dialog),GTK_WIN_POS_CENTER_ON_PARENT);
  gtk_window_set_title(GTK_WINDOW(queue.dialog),"Queueing");


  queue.num_queued = 0;
  queue.label=gtk_label_new("0 Experiments in Queue");
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(queue.dialog))),queue.label,FALSE,FALSE,0);


  queue.list =  gtk_list_store_new(N_COLUMNS,G_TYPE_INT,G_TYPE_STRING);

  /* add data to list: */
  /*
  gtk_list_store_append(queue.list,&queue.iter);
  gtk_list_store_set(queue.list,&queue.iter,BUFFER_COLUMN,12,
		     FILE_COLUMN,"temp file name",-1);


  gtk_list_store_append(queue.list,&queue.iter);
  gtk_list_store_set(queue.list,&queue.iter,BUFFER_COLUMN,14,
		     FILE_COLUMN,"temp file name2",-1);

  gtk_list_store_append(queue.list,&queue.iter);
  gtk_list_store_set(queue.list,&queue.iter,BUFFER_COLUMN,16,
		     FILE_COLUMN,"temp file name3",-1);
  */
  tree = gtk_tree_view_new_with_model(GTK_TREE_MODEL(queue.list));

  queue.select = gtk_tree_view_get_selection(GTK_TREE_VIEW(tree));

  

  renderer = gtk_cell_renderer_text_new();
  column = gtk_tree_view_column_new_with_attributes("Buffer",renderer,"text",0,NULL);
  gtk_tree_view_append_column(GTK_TREE_VIEW(tree),column);

  renderer = gtk_cell_renderer_text_new();
  column = gtk_tree_view_column_new_with_attributes("File Name",renderer,"text",1,NULL);
  gtk_tree_view_append_column(GTK_TREE_VIEW(tree),column);

  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(queue.dialog))),tree,FALSE,FALSE,2);


  button = gtk_button_new_with_label("Remove from Queue");
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_action_area(GTK_DIALOG(queue.dialog))),button,FALSE,FALSE,0);
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(remove_queue),NULL);

  button=gtk_button_new_from_stock(GTK_STOCK_CLOSE);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_action_area(GTK_DIALOG(queue.dialog))),button,TRUE,TRUE,2);

  g_signal_connect_swapped(G_OBJECT(button),"clicked",G_CALLBACK(gtk_widget_hide),queue.dialog);

  g_signal_connect_swapped( G_OBJECT (queue.dialog), "delete_event", G_CALLBACK( gtk_widget_hide ), queue.dialog);






  /* build the add/subtract window */

  add_sub.dialog = gtk_dialog_new();
  gtk_window_set_title(GTK_WINDOW(add_sub.dialog),"Add/Subtract");
  //  label=gtk_label_new("Add/Subtract");
  //  gtk_box_pack_start(GTK_BOX(GTK_DIALOG(add_sub.dialog)->vbox),label,FALSE,FALSE,0);

  gtk_window_set_transient_for(GTK_WINDOW(add_sub.dialog),GTK_WINDOW(panwindow));
  gtk_window_set_position(GTK_WINDOW(add_sub.dialog),GTK_WIN_POS_CENTER_ON_PARENT);


  // The buffer line:
  hbox=gtk_hbox_new_wrap(TRUE,2);
  // in gtk3, this will become - stupid!
  //  hbox=gtk_box_new(GTK_ORIENTATION_HORIZONTAL,2);
  //  gtk_box_set_homogeneous(GTK_BOX(hbox),TRUE);

  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(add_sub.dialog))),hbox,FALSE,FALSE,0);

  label=gtk_label_new("Source 1");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);
  label=gtk_label_new("Source 2");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);
  label=gtk_label_new("Destination");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);

  

  hbox=gtk_hbox_new_wrap(TRUE,2);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(add_sub.dialog))),hbox,FALSE,FALSE,0);

  label=gtk_label_new("Buffer");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);


  add_sub.s_buff1 = gtk_combo_box_text_new();
  //  gtk_combo_box_append_text(GTK_COMBO_BOX(add_sub.s_buff1),"line1");
  //  gtk_combo_box_append_text(GTK_COMBO_BOX(add_sub.s_buff1),"line2");
  g_signal_connect(G_OBJECT(add_sub.s_buff1),"changed",
		     G_CALLBACK (add_sub_changed),NULL);
  gtk_box_pack_start(GTK_BOX(hbox),add_sub.s_buff1,FALSE,FALSE,2);

  label=gtk_label_new("Buffer");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);


  add_sub.s_buff2 = gtk_combo_box_text_new();
  //  gtk_combo_box_append_text(GTK_COMBO_BOX(add_sub.s_buff2),"line1");
  //  gtk_combo_box_append_text(GTK_COMBO_BOX(add_sub.s_buff2),"line2");
  g_signal_connect(G_OBJECT(add_sub.s_buff2),"changed",
		     G_CALLBACK (add_sub_changed),NULL);
  gtk_box_pack_start(GTK_BOX(hbox),add_sub.s_buff2,FALSE,FALSE,2);


  label=gtk_label_new("Buffer");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);

  add_sub.dest_buff = gtk_combo_box_text_new();
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(add_sub.dest_buff),"New");
  g_signal_connect(G_OBJECT(add_sub.dest_buff),"changed",
		     G_CALLBACK (add_sub_changed),NULL);
  gtk_box_pack_start(GTK_BOX(hbox),add_sub.dest_buff,FALSE,FALSE,2);


  // the record line:
  hbox = gtk_hbox_new_wrap(TRUE,2);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(add_sub.dialog))),hbox,FALSE,FALSE,0);
  

  label=gtk_label_new("record");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);

  add_sub.s_record1 = gtk_combo_box_text_new();
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(add_sub.s_record1),"Each");
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(add_sub.s_record1),"Sum All");
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(add_sub.s_record1),"0");
  add_sub.s_rec_c1 = 1; 
  g_signal_connect(G_OBJECT(add_sub.s_record1),"changed",
		     G_CALLBACK (add_sub_changed),NULL);
  gtk_box_pack_start(GTK_BOX(hbox),add_sub.s_record1,FALSE,FALSE,2);


  label=gtk_label_new("record");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);

  add_sub.s_record2 = gtk_combo_box_text_new();
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(add_sub.s_record2),"Each");
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(add_sub.s_record2),"Sum All");
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(add_sub.s_record2),"0");
  add_sub.s_rec_c2 = 1; 
  g_signal_connect(G_OBJECT(add_sub.s_record2),"changed",
		     G_CALLBACK (add_sub_changed),NULL);
  gtk_box_pack_start(GTK_BOX(hbox),add_sub.s_record2,FALSE,FALSE,2);


  label=gtk_label_new("record");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);

  add_sub.dest_record = gtk_combo_box_text_new();
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(add_sub.dest_record),"Each");
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(add_sub.dest_record),"Append");
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(add_sub.dest_record),"0");
  add_sub.dest_rec_c = 1; 
  g_signal_connect(G_OBJECT(add_sub.dest_record),"changed",
		     G_CALLBACK (add_sub_changed),NULL);
  gtk_box_pack_start(GTK_BOX(hbox),add_sub.dest_record,FALSE,FALSE,2);


  // now the multiplier line:

  hbox = gtk_hbox_new_wrap(TRUE,2);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(add_sub.dialog))),hbox,FALSE,FALSE,0);

  label=gtk_label_new("multiplier");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);
  
  adjust = (GtkAdjustment *) gtk_adjustment_new(1.0,-1e6,1e6,0.1,1.,0.);
  add_sub.mult1= gtk_spin_button_new(GTK_ADJUSTMENT(adjust),0,6);
  gtk_box_pack_start(GTK_BOX(hbox),add_sub.mult1,FALSE,FALSE,2);


  label=gtk_label_new("multiplier");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);
  
  adjust = (GtkAdjustment *) gtk_adjustment_new(1.0,-1e6,1e6,0.1,1.,0.);
  add_sub.mult2= gtk_spin_button_new(GTK_ADJUSTMENT(adjust),0,6);
  gtk_box_pack_start(GTK_BOX(hbox),add_sub.mult2,FALSE,FALSE,2);

  label = gtk_label_new(" ");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);
  label = gtk_label_new(" ");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);

  add_sub.apply = gtk_button_new_from_stock(GTK_STOCK_APPLY);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_action_area(GTK_DIALOG(add_sub.dialog))),add_sub.apply,TRUE,TRUE,2);
  g_signal_connect(G_OBJECT(add_sub.apply),"clicked",G_CALLBACK(add_sub_buttons),NULL);

  add_sub.close=gtk_button_new_from_stock(GTK_STOCK_CLOSE);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_action_area(GTK_DIALOG(add_sub.dialog))),add_sub.close,TRUE,TRUE,2);
  g_signal_connect(G_OBJECT(add_sub.close),"clicked",G_CALLBACK(add_sub_buttons),NULL);

  g_signal_connect( G_OBJECT (add_sub.dialog), "delete_event", G_CALLBACK( hide_add_sub ), NULL );
  // defaults:
  add_sub.shown = 0;


  // now build the fitting window



  fit_data.dialog = gtk_dialog_new();
  gtk_window_set_resizable(GTK_WINDOW(fit_data.dialog),0);
  fit_data.add_dialog = NULL;

  gtk_window_set_transient_for(GTK_WINDOW(fit_data.dialog),GTK_WINDOW(panwindow));
  gtk_window_set_position(GTK_WINDOW(fit_data.dialog),GTK_WIN_POS_CENTER_ON_PARENT);

  gtk_window_set_title(GTK_WINDOW(fit_data.dialog),"Fitting");
  //  label = gtk_label_new("Fitting");
  //  gtk_box_pack_start(GTK_BOX(GTK_DIALOG(fit_data.dialog)->vbox),label,FALSE,FALSE,0);

  // the label line:

  hbox=gtk_hbox_new_wrap(TRUE,2);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(fit_data.dialog))),hbox,FALSE,FALSE,0);

  label=gtk_label_new("Data Source");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);

  label=gtk_label_new("Best Fit Destination");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);

  label=gtk_label_new("  \t");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);




  // buffer line:
  hbox=gtk_hbox_new_wrap(TRUE,2);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(fit_data.dialog))),hbox,FALSE,FALSE,0);

  label=gtk_label_new("Buffer");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);

  fit_data.s_buff = gtk_combo_box_text_new();
  g_signal_connect(G_OBJECT(fit_data.s_buff),"changed",
		   G_CALLBACK(fit_data_changed),NULL);
  gtk_box_pack_start(GTK_BOX(hbox),fit_data.s_buff,FALSE,FALSE,2);

  label=gtk_label_new("Buffer");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);
  
  fit_data.d_buff = gtk_combo_box_text_new();
  g_signal_connect(G_OBJECT(fit_data.d_buff),"changed",
		   G_CALLBACK(fit_data_changed),NULL);
  gtk_box_pack_start(GTK_BOX(hbox),fit_data.d_buff,FALSE,FALSE,2);

  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(fit_data.d_buff),"New");

  label=gtk_label_new("Store best fit?");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);

  fit_data.store_fit = gtk_check_button_new();
  gtk_box_pack_start(GTK_BOX(hbox),fit_data.store_fit,FALSE,FALSE,2);




  // record line
  hbox=gtk_hbox_new_wrap(TRUE,2);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(fit_data.dialog))),hbox,FALSE,FALSE,0);

  label=gtk_label_new("record");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);
  
  fit_data.s_record = gtk_combo_box_text_new();
  //  gtk_combo_box_append_text(GTK_COMBO_BOX(fit_data.s_record),"Each");
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(fit_data.s_record),"0");
  g_signal_connect(G_OBJECT(fit_data.s_record),"changed",
		   G_CALLBACK(fit_data_changed),NULL);
  gtk_box_pack_start(GTK_BOX(hbox),fit_data.s_record,FALSE,FALSE,2);


  label=gtk_label_new("record");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);

  fit_data.d_record = gtk_combo_box_text_new();
  //  gtk_combo_box_append_text(GTK_COMBO_BOX(fit_data.d_record),"Each");
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(fit_data.d_record),"Append");
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(fit_data.d_record),"0");
  g_signal_connect(G_OBJECT(fit_data.d_record),"changed",
		   G_CALLBACK(fit_data_changed),NULL);
  gtk_box_pack_start(GTK_BOX(hbox),fit_data.d_record,FALSE,FALSE,2);

  label=gtk_label_new("Include imaginary?");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,2);

  fit_data.include_imag = gtk_check_button_new();
  gtk_box_pack_start(GTK_BOX(hbox),fit_data.include_imag,FALSE,FALSE,2);




  // next line:

  hbox=gtk_hbox_new_wrap(FALSE,2);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(fit_data.dialog))),hbox,FALSE,FALSE,0);


  label = gtk_label_new("# Components:");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,0);
  


  
  adjust =  (GtkAdjustment *)gtk_adjustment_new(0,0,MAX_FIT,1,1,0);
  fit_data.components=gtk_spin_button_new(GTK_ADJUSTMENT(adjust),0,0);
  gtk_box_pack_start(GTK_BOX(hbox),fit_data.components,FALSE,FALSE,2);
  g_signal_connect(G_OBJECT(fit_data.components),"value_changed",G_CALLBACK(fit_data_changed),NULL);

  label = gtk_label_new("       Enable\nProcess Broadening");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,3);

  fit_data.enable_proc_broad = gtk_check_button_new();
  gtk_box_pack_start(GTK_BOX(hbox),fit_data.enable_proc_broad,FALSE,FALSE,0);
  


  fit_data.start_clicking = gtk_button_new_with_label("Add components");
  gtk_box_pack_start(GTK_BOX(hbox),fit_data.start_clicking,FALSE,FALSE,2);
  g_signal_connect(G_OBJECT(fit_data.start_clicking),"clicked",G_CALLBACK(fitting_buttons),NULL);

  fit_data.precalc = gtk_button_new_with_label("Precalc");
  gtk_box_pack_start(GTK_BOX(hbox),fit_data.precalc,FALSE,FALSE,2);
  g_signal_connect(G_OBJECT(fit_data.precalc),"clicked",G_CALLBACK(fitting_buttons),NULL);


  fit_data.run_fit = gtk_button_new_with_label("Run Fit\nFull Spectrum");
  gtk_box_pack_start(GTK_BOX(hbox),fit_data.run_fit,FALSE,FALSE,2);
  g_signal_connect(G_OBJECT(fit_data.run_fit),"clicked",G_CALLBACK(fitting_buttons),NULL);

  fit_data.run_fit_range = gtk_button_new_with_label("Run Fit\nDisplayed Range");
  gtk_box_pack_start(GTK_BOX(hbox),fit_data.run_fit_range,FALSE,FALSE,2);
  g_signal_connect(G_OBJECT(fit_data.run_fit_range),"clicked",G_CALLBACK(fitting_buttons),NULL);

  //  fit_data.close = gtk_button_new_with_label("Close");
  fit_data.close = gtk_button_new_from_stock(GTK_STOCK_CLOSE);
  gtk_box_pack_end(GTK_BOX(hbox),fit_data.close,FALSE,FALSE,2);
  g_signal_connect(G_OBJECT(fit_data.close),"clicked",G_CALLBACK(fitting_buttons),NULL);

  //  the labels for the start values
  vbox=gtk_vbox_new_wrap(TRUE,0);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_action_area(GTK_DIALOG(fit_data.dialog))),vbox,FALSE,FALSE,0);
  

  hbox=gtk_hbox_new_wrap(TRUE,2);
  gtk_box_pack_start(GTK_BOX(vbox),hbox,FALSE,FALSE,0);

  label=gtk_label_new("Num");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,0);

  label=gtk_label_new("center freq");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,0);

  label=gtk_label_new("amplitude");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,0);

  label=gtk_label_new("Gaussian width");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,0);

  label=gtk_label_new("Enable Gaussian");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,0);

  label=gtk_label_new("Lorentzian width");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,0);

  label=gtk_label_new("Enable Lorentzian");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,0);




  gtk_widget_show_all(GTK_WIDGET(gtk_dialog_get_content_area(GTK_DIALOG(fit_data.dialog))));
  gtk_widget_show_all(GTK_WIDGET(vbox));

  // start with zero components, let the user choose how to add them.
  fit_data.num_components = 0;
  fit_data.s_rec = 1;
  fit_data.d_rec = 1;

  // but build them all, just don't show them:
  for (i=0;i<MAX_FIT;i++){
    char stt[5];

    fit_data.hbox[i] = gtk_hbox_new_wrap(TRUE,2);
    gtk_box_pack_start(GTK_BOX(vbox),fit_data.hbox[i],FALSE,FALSE,0);
    
    sprintf(stt,"%i",i+1);
    label=gtk_label_new(stt);
    gtk_box_pack_start(GTK_BOX(fit_data.hbox[i]),label,FALSE,FALSE,0);
    gtk_widget_show(label);


  //center
    adjust = (GtkAdjustment *) gtk_adjustment_new(0.0,-1e7,1e7,100,1000,0);
    fit_data.center[i] = gtk_spin_button_new(GTK_ADJUSTMENT(adjust),0.2,1);
    gtk_box_pack_start(GTK_BOX(fit_data.hbox[i]),fit_data.center[i],FALSE,FALSE,2);
    gtk_widget_show(GTK_WIDGET(fit_data.center[i]));

    // amplitude
    adjust = (GtkAdjustment *) gtk_adjustment_new(0.0,-1e12,1e12,100,1000,0);
    fit_data.amplitude[i] = gtk_spin_button_new(GTK_ADJUSTMENT(adjust),0.2,1);
    gtk_box_pack_start(GTK_BOX(fit_data.hbox[i]),fit_data.amplitude[i],FALSE,FALSE,2);
    gtk_widget_show(GTK_WIDGET(fit_data.amplitude[i]));
    
    // gauss width
    adjust = (GtkAdjustment *) gtk_adjustment_new(0.0,0.0,1e6,10,100,0);
    fit_data.gauss_wid[i] = gtk_spin_button_new(GTK_ADJUSTMENT(adjust),0.2,1);
    gtk_box_pack_start(GTK_BOX(fit_data.hbox[i]),fit_data.gauss_wid[i],FALSE,FALSE,2);
    gtk_widget_show(GTK_WIDGET(fit_data.gauss_wid[i]));


    // enable Gauss
    fit_data.enable_gauss[i] = gtk_check_button_new();
    gtk_box_pack_start(GTK_BOX(fit_data.hbox[i]),fit_data.enable_gauss[i],FALSE,FALSE,2);
    gtk_widget_show(GTK_WIDGET(fit_data.enable_gauss[i]));


    // lorentz width
    adjust = (GtkAdjustment *) gtk_adjustment_new(0.0,0.0,1e6,10,100,0);
    fit_data.lorentz_wid[i] = gtk_spin_button_new(GTK_ADJUSTMENT(adjust),0.2,1);
    gtk_box_pack_start(GTK_BOX(fit_data.hbox[i]),fit_data.lorentz_wid[i],FALSE,FALSE,2);
    gtk_widget_show(GTK_WIDGET(fit_data.lorentz_wid[i]));


    // enable Lorentz
    fit_data.enable_lorentz[i] = gtk_check_button_new();
    gtk_box_pack_start(GTK_BOX(fit_data.hbox[i]),fit_data.enable_lorentz[i],FALSE,FALSE,2);
    gtk_widget_show(GTK_WIDGET(fit_data.enable_lorentz[i]));



  }


 
  g_signal_connect(G_OBJECT(fit_data.dialog),"delete_event",G_CALLBACK(hide_fit),NULL);
  fit_data.shown = 0;




  buffp[0]=create_buff(0);


  gtk_combo_box_set_active(GTK_COMBO_BOX(add_sub.s_buff1),0);
  gtk_combo_box_set_active(GTK_COMBO_BOX(add_sub.s_buff2),0);
  gtk_combo_box_set_active(GTK_COMBO_BOX(add_sub.dest_buff),0);
  gtk_combo_box_set_active(GTK_COMBO_BOX(add_sub.s_record1),0);
  gtk_combo_box_set_active(GTK_COMBO_BOX(add_sub.s_record2),0);
  gtk_combo_box_set_active(GTK_COMBO_BOX(add_sub.dest_record),0);

  gtk_combo_box_set_active(GTK_COMBO_BOX(fit_data.s_buff),0);
  gtk_combo_box_set_active(GTK_COMBO_BOX(fit_data.d_buff),0);
  gtk_combo_box_set_active(GTK_COMBO_BOX(fit_data.s_record),0);
  gtk_combo_box_set_active(GTK_COMBO_BOX(fit_data.d_record),0);



  //  fprintf(stderr,"did first buffer create, npts = %i\n",buffp[0]->npts);
  if (buffp[0] == NULL){
    fprintf(stderr,"first buffer creation failed\n");
    exit(0);
  }
  // now a first buffer exists, find its size.
  gtk_window_get_size(GTK_WINDOW(buffp[0]->win.window),&width,&height);


  gtk_window_move(GTK_WINDOW(panwindow),0,height+50);
  gtk_widget_show(panwindow);

  current = 0;
  last_current = -1;



  /* do some color stuff */
  
  // colourgc=gdk_gc_new(buffp[0]->win.canvas->window);

  colours[RED].red=1.;
  colours[RED].blue=0;
  colours[RED].green=0;
#if GTK_MAJOR_VERSION > 2
  colours[RED].alpha=1.; // red
#endif
  // fprintf(stderr,"pixel: %li\n",colours[RED].pixel);
  //gdk_colormap_alloc_color(gtk_widget_get_colormap(buffp[0]->win.canvas),&colours[RED],FALSE,TRUE);
  // fprintf(stderr,"pixel: %li\n",colours[RED].pixel);
  
  colours[BLUE].red=0;
  colours[BLUE].blue=1.;
  colours[BLUE].green=0;
#if GTK_MAJOR_VERSION > 2
  colours[BLUE].alpha=1.; // blue
#endif
  // fprintf(stderr,"pixel: %li\n",colours[BLUE].pixel);
  //gdk_colormap_alloc_color(gtk_widget_get_colormap(buffp[0]->win.canvas),&colours[BLUE],FALSE,TRUE);
  // fprintf(stderr,"pixel: %li\n",colours[BLUE].pixel);

  colours[GREEN].red=0;
  colours[GREEN].blue=0;
  colours[GREEN].green=1.;
#if GTK_MAJOR_VERSION > 2
  colours[GREEN].alpha = 1.;// green
#endif
  // fprintf(stderr,"pixel: %li\n",colours[GREEN].pixel);
  //gdk_colormap_alloc_color(gtk_widget_get_colormap(buffp[0]->win.canvas),&colours[GREEN],FALSE,TRUE);
  // fprintf(stderr,"pixel: %li\n",colours[GREEN].pixel);

  colours[WHITE].red=1.;
  colours[WHITE].blue=1.;
  colours[WHITE].green=1.;
#if GTK_MAJOR_VERSION > 2
  colours[WHITE].alpha=1.;
#endif
  //gdk_colormap_alloc_color(gtk_widget_get_colormap(buffp[0]->win.canvas),&colours[WHITE],FALSE,TRUE);

  colours[BLACK].red=0;
  colours[BLACK].blue=0;
  colours[BLACK].green=0;
#if GTK_MAJOR_VERSION > 2
  colours[BLACK].alpha=1;
#endif
  //  gdk_colormap_alloc_color(gtk_widget_get_colormap(buffp[0]->win.canvas),&colours[BLACK],FALSE,TRUE);


  ic=0;
  for (i=0;i<NUM_COLOURS/4;i++) {
    colours[ic].red = 1.;
    colours[ic].blue = 0.;
    colours[ic].green = i/(NUM_COLOURS/4.);
    
#if GTK_MAJOR_VERSION > 2
    colours[ic].alpha = 1.;
#endif
    //    fprintf(stderr,"r g b %i %i %i %li\n",colours[ic].red,colours[ic].green, colours[ic].blue,colours[ic].pixel);
    
    //    gdk_colormap_alloc_color(gtk_widget_get_colormap(buffp[0]->win.canvas),
    //		    &colours[ic],FALSE,TRUE);
    ic++;
  }

  for (i=0;i<NUM_COLOURS/4;i++){
    colours[ic].red = (1.-(i/(NUM_COLOURS/4.)));
    colours[ic].blue = 0.;
    colours[ic].green = 1.;


#if GTK_MAJOR_VERSION > 2
    colours[ic].alpha = 1.;
#endif
    //    fprintf(stderr,"r g b %i %i %i %li\n",colours[ic].red,colours[ic].green, colours[ic].blue,colours[ic].pixel);
    //gdk_colormap_alloc_color(gtk_widget_get_colormap(buffp[0]->win.canvas),
    //		    &colours[ic],FALSE,TRUE);
    ic++;
  }

  //extra for white to be in the middle
  ic++;
  //  fprintf(stderr,"just skipped colour: %i\n",ic-1);
  for (i=0;i<NUM_COLOURS/4;i++){
    colours[ic].red = 0.;
    colours[ic].blue = i/(NUM_COLOURS/4.);
    colours[ic].green = 1.;
    
#if GTK_MAJOR_VERSION > 2
    colours[ic].alpha = 1.;
#endif
    //    fprintf(stderr,"r g b %i %i %i %li\n",colours[ic].red,colours[ic].green, colours[ic].blue,colours[ic].pixel);
    //gdk_colormap_alloc_color(gtk_widget_get_colormap(buffp[0]->win.canvas),
    //		    &colours[ic],FALSE,TRUE);
    ic++;
  }

  for (i=0;i<NUM_COLOURS/4;i++){
    colours[ic].red = 0.;
    colours[ic].blue = 1.;
    colours[ic].green = (1.0-i/(NUM_COLOURS/4.));


#if GTK_MAJOR_VERSION > 2
    colours[ic].alpha = 1.;
#endif
    //    fprintf(stderr,"r g b %i %i %i %li\n",colours[ic].red,colours[ic].green, colours[ic].blue,colours[ic].pixel);
    //gdk_colormap_alloc_color(gtk_widget_get_colormap(buffp[0]->win.canvas),
    //		    &colours[ic],FALSE,TRUE);
    ic++;
  }

  /*
  ic=NUM_COLOURS/2; // also white ?
  colours[ic].red=256*255;
  colours[ic].green=256*255;
  colours[ic].blue=256*255;
  colours[ic].pixel= (gulong) colours[ic].green 
    +256*colours[ic].red+colours[ic].blue/256;
  
  ic=NUM_COLOURS/2-1; //another white?
  colours[ic].red=256*255;
  colours[ic].green=256*255;
  colours[ic].blue=256*255;
  colours[ic].pixel= (gulong) colours[ic].green 
    +256*colours[ic].red+colours[ic].blue/256;

white set up below  */
  



  /* build some other dialogs */

  /* first the phase dialog */
  phase_dialog = gtk_dialog_new();
  phase0_ad= (GtkAdjustment *) gtk_adjustment_new(0.0,-180.0,180.0,.1,1.,0);
  phase1_ad= (GtkAdjustment *) gtk_adjustment_new(0.0,-180.0,180.0,.1,1.,0);

  hbox=gtk_hbox_new_wrap(FALSE,1);
  button=gtk_button_new_with_label("-360");
  gtk_box_pack_start(GTK_BOX(hbox),button,FALSE,FALSE,1);
  gtk_widget_show(button);
  phase_data.mbut=button;
  g_signal_connect(G_OBJECT(button),"clicked",
		     G_CALLBACK(phase_buttons),NULL);

  button=gtk_button_new_with_label("+360");
  gtk_box_pack_end(GTK_BOX(hbox),button,FALSE,FALSE,1);
  gtk_widget_show(button);
  phase_data.pbut=button;
  g_signal_connect(G_OBJECT(button),"clicked",
		     G_CALLBACK(phase_buttons),NULL);
  gtk_widget_show(hbox);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(phase_dialog))),
		     hbox,TRUE,TRUE,1);

  snprintf(title,UTIL_LEN,"Phase 1");
  label=gtk_label_new(title);
  gtk_widget_show(label);
  gtk_box_pack_start(GTK_BOX(hbox),
		     label,TRUE,TRUE,1);
#if GTK_MAJOR_VERSION == 2
  phase_data.pscroll1=gtk_hscale_new(GTK_ADJUSTMENT(phase1_ad));
#else
  phase_data.pscroll1 = gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,phase1_ad);
#endif
  //  gtk_range_set_update_policy(GTK_RANGE(phase_data.pscroll1),
  //			      GTK_UPDATE_CONTINUOUS);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(phase_dialog))) ,
		     phase_data.pscroll1,TRUE,TRUE,1);
  g_signal_connect(G_OBJECT(phase1_ad),"value_changed",
		     G_CALLBACK(phase_changed),NULL);
  //    gtk_widget_set_size_request(phase_data.pscroll1,397,50);
  gtk_widget_show (phase_data.pscroll1);

  snprintf(title,UTIL_LEN,"Phase 0");
  label=gtk_label_new(title);
  gtk_widget_show(label);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(phase_dialog))),
		     label,TRUE,TRUE,1);
#if GTK_MAJOR_VERSION == 2
  phase_data.pscroll0=gtk_hscale_new(GTK_ADJUSTMENT(phase0_ad));
#else
  phase_data.pscroll0 = gtk_scale_new(GTK_ORIENTATION_HORIZONTAL,phase0_ad);
#endif
  //  gtk_widget_set_size_request(phase_data.pscroll0,397,50);
  //gtk_range_set_update_policy(GTK_RANGE(phase_data.pscroll0),
  //			      GTK_UPDATE_CONTINUOUS);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(phase_dialog))) ,
		     phase_data.pscroll0,TRUE,TRUE,0);
  g_signal_connect(G_OBJECT(phase0_ad),"value_changed",
		     G_CALLBACK(phase_changed),NULL);
  gtk_widget_show (phase_data.pscroll0);


  button=gtk_button_new_from_stock(GTK_STOCK_OK);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_action_area(GTK_DIALOG(phase_dialog))),
		     button,TRUE,TRUE,1);
  g_signal_connect(G_OBJECT(button),"clicked",
		     G_CALLBACK(phase_buttons),(void *) 0);
  gtk_widget_show(button);
  phase_data.ok=button;

  button=gtk_button_new_with_label("Apply all");
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_action_area(GTK_DIALOG(phase_dialog))),
		     button,TRUE,TRUE,1);
  g_signal_connect(G_OBJECT(button),"clicked",
		     G_CALLBACK(phase_buttons),(void *) 1);
  gtk_widget_show(button);
  phase_data.apply_all=button;


  button=gtk_button_new_with_label("Update Last");
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_action_area(GTK_DIALOG(phase_dialog))),
		     button,TRUE,TRUE,1);
  g_signal_connect(G_OBJECT(button),"clicked",
		     G_CALLBACK(phase_buttons),(void *) 2);
  gtk_widget_show(button);
  phase_data.update=button;

  button=gtk_button_new_from_stock(GTK_STOCK_CANCEL);
  gtk_box_pack_start(GTK_BOX(gtk_dialog_get_action_area(GTK_DIALOG(phase_dialog))),
		     button,TRUE,TRUE,1);
  g_signal_connect(G_OBJECT(button),"clicked",
		     G_CALLBACK(phase_buttons),(void *) 3);
  gtk_widget_show(button);
  phase_data.cancel=button;
  phase_data.is_open=0;
  phase0=0.;
  phase1=0.;



  g_signal_connect( G_OBJECT (phase_dialog), "delete_event", G_CALLBACK( hide_phase ), NULL );
  

  /* now build frequency pop-up */

  freq_popup=gtk_window_new(GTK_WINDOW_POPUP);
  gtk_window_set_gravity(GTK_WINDOW(freq_popup),GDK_GRAVITY_NORTH_WEST);

  vbox=gtk_vbox_new_wrap(FALSE,1);
  gtk_container_add(GTK_CONTAINER(freq_popup),vbox);
  gtk_widget_show(vbox);
  
  fplab1=gtk_label_new("position");
  fplab2=gtk_label_new("and stuff");
  fplab3=gtk_label_new("dummy");
  fplab4=gtk_label_new("freqs");
  fplab5=gtk_label_new("freqs");

  gtk_widget_show(fplab1);
  gtk_widget_show(fplab2);
  gtk_widget_show(fplab3);
  gtk_widget_show(fplab4);
  gtk_widget_show(fplab5);

  gtk_box_pack_start(GTK_BOX(vbox),fplab1,FALSE,FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),fplab2,FALSE,FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),fplab3,FALSE,FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),fplab4,FALSE,FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),fplab5,FALSE,FALSE,0);



  /* setup a busy cursor */
  cursorclock=gdk_cursor_new(GDK_CLOCK);

  if( result_shm == 1 && !no_acq ) {
    fprintf(stderr,"connecting to old shm\n");
    upload_buff = current;
    // fprintf(stderr, "initializing IPC signals and sending an idle_draw_canvas\n" );

        
    init_ipc_signals();
	
    
    path_strcpy(buffp[0]->path_for_reload,data_shm->save_data_path);
    set_window_title(buffp[0]);
    reload(NULL,NULL);
  }
  else
    draw_canvas(buffp[0]); 

  /*initialization complete */
  
  if (no_acq ==TRUE){
    signal( SIGINT,  do_destroy_all );
    signal( SIGTERM,  do_destroy_all );
#ifndef MINGW
    signal( SIGQUIT,  do_destroy_all ); // this is ordinary ^C or kill
#endif
  }

  show_active_border(); 

  last_draw();


  // add a timeout for debugging...

  //  timeout_tag = gtk_timeout_add(2000,(GSourceFunc) check_for_overrun_timeout,timeout_data);
  //  gdk_threads_enter();
  gtk_main();
  //  gdk_threads_leave();
  //  fprintf(stderr,"out of gtk_main\n");
  /* post main - clean up */
  // fprintf(stderr,"post main cleanup\n");
  /* unalloc gc */
  //  gdk_gc_unref(colourgc);

  //  gtk_timeout_remove(timeout_tag);

  return 0;

}


//void open_phase( dbuff *buff, int action, GtkWidget *widget )
void open_phase(GtkAction *action, dbuff *buff)
{
  int buffnum,i,true_complex;
  float old_low,old_up,future_p1;
  char temps[UTIL_LEN];

  CHECK_ACTIVE(buff);
  true_complex = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.true_complex));
  buffnum=buff->buffnum;
  if (phase_data.is_open==1 || 
      (buff->disp.dispstyle != SLICE_ROW && buff->disp.dispstyle !=SLICE_COL ))
      return;

  /* if its a column, has to be hyper */
  if(buff->disp.dispstyle ==SLICE_COL && !buff->is_hyper && !true_complex){
    popup_msg("can't phase a column unless its hypercomplex or true complex",TRUE);
    printf("true_complex: %i, is_hyper: %i\n",true_complex,buff->is_hyper);
    return;
  }
  if (buff->disp.dispstyle == SLICE_COL && buff->npts2 %2 == 1 && buff->is_hyper){
    popup_msg("Can't phase hypercomplex 2d on odd number of npts2", TRUE);
    return;
  }
  if (grab_canvas(buff, ID_PHASE) == 0){
    popup_msg("Can't open phase, canvas blocked",TRUE);
    return;
  }

  sprintf(temps,"Phasing for buffer %i",buff->buffnum);
  gtk_window_set_title(GTK_WINDOW (phase_dialog),temps);

  buff->win.press_pend =1;

  
  if(buff->disp.dispstyle==SLICE_ROW){
    phase_data.data=g_malloc(buff->npts*2*sizeof(float));
    phase_data.data2=g_malloc(buff->npts*2*sizeof(float));
    phase_npts = buff->npts;
    //copy the data to be phased to a safe place:

    for(i=0;i<phase_npts*2;i++) 
      phase_data.data[i]=
	buff->data[i+buff->disp.record*buff->npts*2];
  }
  else if(buff->disp.dispstyle==SLICE_COL){
    phase_data.data=g_malloc(buff->npts2*2*sizeof(float));
    phase_data.data2=g_malloc(buff->npts2*2*sizeof(float));
    if (buff->is_hyper)
      phase_npts = buff->npts2/2;
    else  if(true_complex)
      phase_npts=buff->npts2;
    
    printf("phase_npts is: %i\n",phase_npts);
    //copy the data to be phased to a safe place:

    if (buff->is_hyper){
      for(i=0;i<phase_npts;i++){
	phase_data.data[2*i]=buff->data[buff->npts*2*2*i+
					2*buff->disp.record2];
	phase_data.data[2*i+1]=buff->data[buff->npts*2*(2*i+1)
					  +2*buff->disp.record2];
      }
    }
    if (true_complex){
      for(i=0;i<phase_npts;i++){
	phase_data.data[2*i]=buff->data[buff->npts*2*i+2*buff->disp.record2];
	phase_data.data[2*i+1]=buff->data[buff->npts*2*i+2*buff->disp.record2+1];
      }
    }
  }
  /* copy the data into the first spot */

  phase_data.buffnum=buffnum;
  phase_data.pivot=0.0;


  /* save old phase values from buffer */
  if(buff->disp.dispstyle ==SLICE_ROW ){
    phase_data.ophase0=buff->phase0_app;
    phase_data.ophase1=buff->phase1_app; 
  }
  else{
    phase_data.ophase0=buff->phase20_app;
    phase_data.ophase1=buff->phase21_app;
  }

  if (((int)buff->process_data[PH].val & GLOBAL_PHASE_FLAG)!=0 ){
    if (buff->disp.dispstyle == SLICE_ROW){  //row
      gtk_adjustment_set_value(GTK_ADJUSTMENT(phase0_ad),phase0);
      future_p1=phase1;
    }
    else{   //col
      gtk_adjustment_set_value(GTK_ADJUSTMENT(phase0_ad),phase20);
      future_p1=phase21;
    }
  }
  else { /* not global */
    if(buff->disp.dispstyle == SLICE_ROW){  //row
      gtk_adjustment_set_value(GTK_ADJUSTMENT(phase0_ad),buff->phase0);
      future_p1=buff->phase1;
    }
    else{   //col
      gtk_adjustment_set_value(GTK_ADJUSTMENT(phase0_ad),buff->phase20);
      future_p1=buff->phase21;
    }
  }

  /* check what the phase 1 value is */

  old_low=gtk_adjustment_get_lower(GTK_ADJUSTMENT(phase1_ad));
  old_up=gtk_adjustment_get_upper(GTK_ADJUSTMENT(phase1_ad));
  
  gtk_adjustment_set_lower(GTK_ADJUSTMENT(phase1_ad),floor((future_p1+180.0)/360.0)*360.-180.);
  gtk_adjustment_set_upper(GTK_ADJUSTMENT(phase1_ad) ,gtk_adjustment_get_lower(GTK_ADJUSTMENT(phase1_ad))+
			   (old_up-old_low));
  
  // postpone setting phase1 till the upper and lower limits make sense.
  gtk_adjustment_changed(GTK_ADJUSTMENT(phase1_ad));
  gtk_adjustment_set_value(GTK_ADJUSTMENT(phase1_ad),future_p1);

  phase_data.last_phase1=gtk_adjustment_get_value(GTK_ADJUSTMENT(phase1_ad));

  gtk_window_set_transient_for(GTK_WINDOW(phase_dialog),GTK_WINDOW(panwindow));
  gtk_window_set_position(GTK_WINDOW(phase_dialog),GTK_WIN_POS_CENTER_ON_PARENT);

  gtk_widget_show(phase_dialog);
  phase_data.is_open=1;

  phase_changed(phase0_ad,NULL);
  
  return;
}

gint phase_buttons(GtkWidget *widget,gpointer data)
{
  int i, j,npts1,true_complex;
  float lp0,lp1,dp0,dp1;
  dbuff *buff;
  /* first check to make sure buffer still exists */
  if (buffp[phase_data.buffnum] != NULL){

    buff=buffp[phase_data.buffnum];
    npts1=buff->npts;  
    true_complex = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.true_complex));

    lp0=gtk_adjustment_get_value(GTK_ADJUSTMENT(phase0_ad));
    lp1=gtk_adjustment_get_value(GTK_ADJUSTMENT(phase1_ad));

    /* do the ok button */
    if(widget == phase_data.ok || widget ==phase_data.apply_all){
      /* new phase values: */

      lp0 = lp0-(floor((lp0+180.)/360.))*360. ;

      /* differences */
      
      dp0=lp0-phase_data.ophase0;
      dp1=lp1-phase_data.ophase1;

      if(((int)buff->process_data[PH].val & GLOBAL_PHASE_FLAG)!=0){
	if (buff->disp.dispstyle == SLICE_ROW){
	  phase0=lp0; /* set new global values */
	  phase1=lp1;
	}
	else{
	  phase20=lp0;
	  phase21=lp1;
	}
      }
      /* either way the buffer's phases get it */
      if(buff->disp.dispstyle==SLICE_ROW){
	buff->phase0=lp0;
	buff->phase1=lp1;
	buff->phase0_app=lp0;
	buff->phase1_app=lp1;
      }
      else{ //its a col
	buff->phase20=lp0;
	buff->phase21=lp1;
	buff->phase20_app=lp0;
	buff->phase21_app=lp1;
      }

      /* actually do the phase on this data */

      if(buff->disp.dispstyle == SLICE_ROW){
	if (phase_npts == buff->npts){
	  // fprintf(stderr,"in ok button, dealing with slice\n");
	  if(widget==phase_data.apply_all){
	    
	    for(j=0;j<buff->npts2;j++){
	      for(i=0;i<buff->npts*2;i++) 
		phase_data.data[i]=
		  buff->data[i+j*buff->npts*2];
	      do_phase(phase_data.data,&buff->data[npts1*2*j],
		       dp0,dp1,buff->npts);
	    }
	  }
	  else{  // not apply all 
	    do_phase(phase_data.data,&buff->data[npts1*2*buff->disp.record],
		     dp0,dp1,buff->npts);
	  }
	}// end if npts the same
	else popup_msg("npts changed, didn't apply phase",TRUE);
      }// end slice row
      else{ /* SLICE_COL */
	if ((phase_npts == buff->npts2/2 && buff->is_hyper) || (phase_npts == buff->npts2 && true_complex)){
	  if(widget==phase_data.apply_all){
	    if (buff->is_hyper){
	      for(j=0;j<buff->npts*2;j++){// does real and imag of 1d's
		for(i=0;i<buff->npts2/2;i++){
		  phase_data.data[2*i]=buff->data[j+buff->npts*2*2*i];
		  phase_data.data[2*i+1]=buff->data[j+
						    buff->npts*2*(2*i+1)];
		}
		do_phase(phase_data.data,phase_data.data2,dp0,dp1,buff->npts2/2);
		for(i=0;i<buff->npts2/2;i++){
		  buff->data[j+buff->npts*2*2*i]=phase_data.data2[2*i];
		  buff->data[j+buff->npts*2*(2*i+1)]
		    =phase_data.data2[2*i+1];
		}
	      }
	    }
	    else if (true_complex){
	      for(j=0;j<buff->npts;j++){
		for(i=0;i<buff->npts2;i++){
		  phase_data.data[2*i]=buff->data[2*j+buff->npts*2*i];
		  phase_data.data[2*i+1]=buff->data[2*j+buff->npts*2*i+1];
		}
		do_phase(phase_data.data,phase_data.data2,dp0,dp1,buff->npts2);
		for(i=0;i<buff->npts2;i++){
		  buff->data[2*j+buff->npts*2*i]=phase_data.data2[2*i];
		  buff->data[2*j+buff->npts*2*i+1]=phase_data.data2[2*i+1];
		}
	      }
	    }
	  }
	  else{ // not apply all 
	    if (buff->is_hyper){
	      for (j=2*buff->disp.record2;j<2*buff->disp.record2+2;j++){
		for(i=0;i<buff->npts2/2;i++){
		  phase_data.data[2*i]=buff->data[j+buff->npts*2*2*i];
		  phase_data.data[2*i+1]=buff->data[j+
						    buff->npts*2*(2*i+1)];
		}
		do_phase(phase_data.data,phase_data.data2,dp0,dp1,buff->npts2/2);
		for(i=0;i<buff->npts2/2;i++){
		  buff->data[j+buff->npts*2*2*i]=phase_data.data2[2*i];
		  buff->data[j+buff->npts*2*(2*i+1)]
		    =phase_data.data2[2*i+1];
		}
	      }
	    }
	    else if (true_complex){
	      j=2*buff->disp.record2;
	      for(i=0;i<buff->npts2;i++){
		phase_data.data[2*i]=buff->data[j+buff->npts*2*i];
		phase_data.data[2*i+1]=buff->data[j+1+buff->npts*2*i];
	      }
	      do_phase(phase_data.data,phase_data.data2,dp0,dp1,buff->npts2);
	      for(i=0;i<buff->npts2;i++){
		buff->data[j+buff->npts*2*i]=phase_data.data2[2*i];
		buff->data[j+buff->npts*2*i+1]=phase_data.data2[2*i+1];
	      }
	    }
	  }
	}// end npts the same
	else popup_msg("npts changed, didn't apply phase",TRUE);
      }// end slice col.


      /* close up */
      gtk_widget_hide(phase_dialog);
      phase_data.is_open=0;
      ungrab_canvas(buff, ID_PHASE);
      buff->win.press_pend=0;
      draw_canvas(buff);
      if (buff->buffnum == current){
	gdk_window_raise(gtk_widget_get_window(buff->win.window));
	// fprintf(stderr,"raised current window 4\n");
      }
      phase_npts=0;
      g_free(phase_data.data);
      g_free(phase_data.data2);
    }/* end button ok */
    else if(widget == phase_data.update){
      if(buff->disp.dispstyle ==SLICE_ROW){
	phase0=lp0;
	phase1=lp1;
	buff->phase0=lp0;
	buff->phase1=lp1;
	}
      else{
	phase20=lp0;
	phase21=lp1;
	buff->phase20=lp0;
	buff->phase21=lp1;
      }
      
    }
    else if(widget == phase_data.cancel){
      g_free(phase_data.data);
      g_free(phase_data.data2);
      phase_npts = 0;
      gtk_widget_hide(phase_dialog);
      phase_data.is_open=0;
      ungrab_canvas(buff, ID_PHASE);
      buff->win.press_pend=0;
      if (buff->buffnum == current){
	gdk_window_raise(gtk_widget_get_window(buff->win.window));
	// fprintf(stderr,"raised current window 5\n");
      }

      draw_canvas(buff);
    }
    else if (widget == phase_data.pbut){
      lp1+=360;
      gtk_adjustment_set_lower(GTK_ADJUSTMENT(phase1_ad),
			       gtk_adjustment_get_lower(GTK_ADJUSTMENT(phase1_ad))+360.0);
      gtk_adjustment_set_upper(GTK_ADJUSTMENT(phase1_ad),
			       gtk_adjustment_get_upper(GTK_ADJUSTMENT(phase1_ad))+360.0);
      gtk_adjustment_set_value(GTK_ADJUSTMENT(phase1_ad),lp1);
      phase_changed(phase1_ad,NULL);
      gtk_adjustment_changed(GTK_ADJUSTMENT(phase1_ad));
    }
    else if (widget == phase_data.mbut){
      lp1-=360;
      gtk_adjustment_set_lower(GTK_ADJUSTMENT(phase1_ad),
			       gtk_adjustment_get_lower(GTK_ADJUSTMENT(phase1_ad))-360.0);
      gtk_adjustment_set_upper(GTK_ADJUSTMENT(phase1_ad),
			       gtk_adjustment_get_upper(GTK_ADJUSTMENT(phase1_ad))-360.0);
      gtk_adjustment_set_value(GTK_ADJUSTMENT(phase1_ad),lp1);
      phase_changed(phase1_ad,NULL);
      gtk_adjustment_changed(GTK_ADJUSTMENT(phase1_ad));
    }

  }
  else{ /* if the buffer disappeared */
    g_free(phase_data.data);
    g_free(phase_data.data2);
    gtk_widget_hide(phase_dialog);
    phase_data.is_open=0;
    fprintf(stderr,"in phase, buffer is gone\n");
  }
  return 0;
}

gint do_phase(float *source,float *dest,float phase0,float phase1,int npts)
{
  unsigned int i;
  float s1;
  float s2;

  phase0 *= M_PI/180.0;
  phase1 *= M_PI/180.0;

  for(i=0;i<npts;i++){

    s1 = source[i*2];
    s2 = source[i*2+1];

    if (npts >1 ){
      dest[2*i]=s1*cos(phase0+phase1*i/npts)
	+s2*sin(phase0+phase1*i/npts);
      dest[2*i+1]=-s1*sin(phase0+phase1*i/npts)
	+s2*cos(phase0+phase1*i/npts);
    }
    else{ // ignore lp1  might need this with a 2d data set with only 1 complex point.
      dest[2*i]=s1*cos(phase0)
	+s2*sin(phase0);
      dest[2*i+1]=-s1*sin(phase0)
	+s2*cos(phase0);


    }
 }

  return 0; 
}


gint phase_changed(GtkAdjustment *widget,gpointer *data)
{
  float lp0,lp1;
  float dp0,dp1;
  dbuff *buff;
  int sizex,sizey;
  GdkRectangle rect;

  /* make sure buffer still exists and phase window is actually open */
  if(buffp[phase_data.buffnum]==NULL || phase_data.is_open==0) return -1;
  buff=buffp[phase_data.buffnum];
  lp0=gtk_adjustment_get_value(GTK_ADJUSTMENT(phase0_ad));
  lp1=gtk_adjustment_get_value(GTK_ADJUSTMENT(phase1_ad));

  if(G_OBJECT(widget) == G_OBJECT(phase1_ad)){
   /* all we do in here is make the corresponding change in phase0 */
    lp0 = lp0- (lp1-phase_data.last_phase1)*phase_data.pivot;
    lp0 = lp0-(floor ((lp0+180.)/360.))*360. ;
    phase_data.last_phase1=lp1;
    if (phase_data.pivot == 0.) phase_changed(phase0_ad,NULL);
    gtk_adjustment_set_value(GTK_ADJUSTMENT(phase0_ad),lp0);
  }
  else /* is a phase 0 adjust */
    {
      /* so what do the adjustments say:*/
      dp0=lp0-phase_data.ophase0;
      dp1=lp1-phase_data.ophase1;
      /* copy the data over */
      if(buff->disp.dispstyle==SLICE_ROW){
	/*	do_phase(&buff->data[2*buff->disp.record*buff->npts],
		phase_data.data,dp0,dp1,buff->npts); */
	do_phase(phase_data.data,
		 phase_data.data2,dp0,dp1,phase_npts);
      }
      else{ /* is column */
	//	for(i=0;i<buff->npts2/2;i++){
	  /*	  phase_data.data2[2*i]=buff->data[buff->npts*2*2*i+
					  2*buff->disp.record2];
	  phase_data.data2[2*i+1]=buff->data[buff->npts*2*(2*i+1)
	  +2*buff->disp.record2]; */
	  do_phase(phase_data.data,phase_data.data2,dp0,dp1,phase_npts);
	  //	}
      }      
      
      
      /* clear the canvas */
      sizex=buff->win.sizex;
      sizey=buff->win.sizey;
      rect.x=1;
      rect.y=1;
      rect.width=sizex;
      rect.height=sizey;

      //      printf("phase changed, drawing whole thing in white\n");
      /*      gdk_draw_rectangle(buff->win.pixmap,
			 buff->win.canvas->style->white_gc,TRUE,
			 rect.x,rect.y,rect.width,rect.height);
      */
       cairo_set_source_rgb(buff->win.cr,1.,1.,1.);
       cairo_rectangle(buff->win.cr,rect.x,rect.y,rect.width,rect.height);
       cairo_fill(buff->win.cr);
      if(buff->disp.dispstyle==SLICE_ROW){
	draw_oned(buff,0.,0.,phase_data.data2,phase_npts);
      }
      else{ //column
	draw_oned(buff,0.,0.,phase_data.data2,phase_npts);
      }

      /* and draw in the pivot line */
      if ( phase_data.pivot >0 && phase_data.pivot <1){
	draw_vertical(buff,&colours[BLUE],phase_data.pivot,-1);
      }
      gtk_widget_queue_draw_area(buff->win.darea,rect.x,rect.y,rect.width,rect.height);
      while (gtk_events_pending())
      	gtk_main_iteration();
    } /* end phase 0 adjust */
  return 0;  
}

gint pivot_set_event (GtkWidget *widget, GdkEventButton *event,dbuff *buff)
{
  int sizex,xval; //sizey;
  float new_pivot,lp0,ll1,ll2; //lp1;

  sizex=buff->win.sizex;
  //  sizey=buff->win.sizey;
  lp0=gtk_adjustment_get_value(GTK_ADJUSTMENT(phase0_ad));
  //  lp1=gtk_adjustment_get_value(GTK_ADJUSTMENT(phase1_ad));

  if(buff->win.press_pend<1){
    fprintf(stderr,"in pivot set and press_pend <1\n");
    return TRUE;
  }

  if(buff->disp.dispstyle==SLICE_ROW){
    ll1=buff->disp.xx1;
    ll2=buff->disp.xx2;
  }
  else{
    ll1=buff->disp.yy1;
    ll2=buff->disp.yy2;
  }
  // fprintf(stderr,"ll1: %f, ll2: %f\n",ll1,ll2);
  /* if there's an old pivot, erase it */

  if(phase_data.pivot>0 && phase_data.pivot<1){
    draw_vertical(buff,&colours[WHITE],phase_data.pivot,-1);
  }
  
  xval=event->x;
  if (xval == 0 ) xval=1; /* in case we clicked in the border, bring inside */
  if(xval == sizex+1) xval=sizex;

  new_pivot = (float) (xval-1) /(sizex-1) *(ll2-ll1)+ll1;

  // fprintf(stderr,"pivot set to: %f\n",new_pivot);

  /*  
  lp0 += (new_pivot-phase_data.pivot)*
  GTK_ADJUSTMENT(phase1_ad)->value;   */


  lp0 = lp0-(floor((lp0+180.)/360.))*360. ;
  phase_data.pivot=new_pivot;
  gtk_adjustment_set_value(GTK_ADJUSTMENT(phase0_ad),lp0);

  draw_vertical(buff,&colours[BLUE],new_pivot,-1);

  // fprintf(stderr,"leaving pivot set\n");
  return TRUE;
    

}


void cursor_busy(dbuff *buff)
{
  //    fprintf(stderr,"setting cursor busy\n");
  gdk_window_set_cursor(gtk_widget_get_window(buff->win.window),cursorclock);

  return;
}

void cursor_normal(dbuff *buff)
{
  //    fprintf(stderr,"setting cursor normal\n");
  gdk_window_set_cursor(gtk_widget_get_window(buff->win.window),NULL); 
   return;
}


gint destroy_all(GtkWidget *widget, gpointer data)
{
  // never use the widget...
  GtkWidget *dialog;
  int result;
  // this routine gets called if we do a file_exit, or if we kill the panel window
  //  fprintf(stderr,"in destroy_all\n");
  
  if (acq_in_progress != ACQ_STOPPED){
    // need to build a "acq is running, are you sure you want to quit? box"
    
    /*    dialog = gtk_dialog_new();
    gtk_window_set_modal( GTK_WINDOW( dialog ), TRUE );
    */

    if (queue.num_queued > 0) 
      dialog = gtk_message_dialog_new(GTK_WINDOW(buffp[current]->win.window),
				      GTK_DIALOG_DESTROY_WITH_PARENT,
				      GTK_MESSAGE_WARNING,GTK_BUTTONS_NONE,
				      "Acquisition is in progress, are you sure you want to quit?\nQueued experiments will be LOST!");
    else
      dialog = gtk_message_dialog_new(GTK_WINDOW(buffp[current]->win.window),
				      GTK_DIALOG_DESTROY_WITH_PARENT,
				      GTK_MESSAGE_WARNING,GTK_BUTTONS_NONE,
				      "Acquisition is in progress, are you sure you want to quit?");
    
    gtk_dialog_add_buttons (GTK_DIALOG(dialog),
			    "Yes (Quit and leave acq running)",GTK_RESPONSE_YES,
			    "No (Don't quit)",GTK_RESPONSE_NO,NULL);


    gtk_window_set_keep_above(GTK_WINDOW(dialog),TRUE);
    result = gtk_dialog_run(GTK_DIALOG(dialog));
    if (result == GTK_RESPONSE_YES)
      do_destroy_all();
    else{
      gtk_widget_destroy(dialog);
      return TRUE;
    }
  }
  do_destroy_all();
  printf("returned from do_destroy_all\n");
  return TRUE; // seems hardly necessary
}

void do_destroy_all()
     // this routine does not fail - always destroys ( with gtk_main_quit )
{
 /* should run through all buffers and call file_close for each */
  int i;

  // this gets called from destroy_all, and also if we get a kill signal

  //  fprintf(stderr,"in do_destroy_all\n");
  from_do_destroy_all = 1;
  for( i=0; i<MAX_BUFFERS; i++ ) {
    if( buffp[i] != NULL )
      destroy_buff( buffp[i]->win.window, NULL,buffp[i] ); 
  }

  //release_ipc_stuff will be called when the last file window is destroyed,, so we don't have to 
  // call it explicitaly
  //  fprintf(stderr, "done closing buffers\n" );

  //  gtk_main_quit();  // this is done in destroy buff.
  //  fprintf(stderr,"just called gtk_main_quit\n");
  return;
}

gint hide_phase( GtkWidget *widget, GdkEvent  *event, gpointer   data )
{
  phase_buttons( phase_data.cancel, NULL );
  // fprintf(stderr, "hiding window\n" );
  return TRUE;
}

gint popup_msg( char* msg ,char modal)
{
  GtkWidget* dialog;
  GtkWidget* label;


  if (modal == TRUE){
    dialog = gtk_message_dialog_new(GTK_WINDOW(buffp[current]->win.window),GTK_DIALOG_DESTROY_WITH_PARENT,
				    GTK_MESSAGE_INFO,GTK_BUTTONS_CLOSE,"%s",msg);
    gtk_window_set_modal( GTK_WINDOW( dialog ), TRUE );
    g_signal_connect_swapped(dialog,"response",G_CALLBACK(gtk_widget_destroy),dialog);
    gtk_widget_show_all(dialog);
    gtk_window_set_keep_above(GTK_WINDOW(dialog),TRUE);

    /*    gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);
    */
  }
  else{
    dialog = gtk_dialog_new_with_buttons("Fit Results",GTK_WINDOW(panwindow),
	      GTK_DIALOG_DESTROY_WITH_PARENT,GTK_STOCK_CLOSE,GTK_RESPONSE_NONE,NULL);
    label=gtk_label_new(msg);
    gtk_container_add(GTK_CONTAINER(gtk_dialog_get_content_area(GTK_DIALOG(dialog))),label);
    gtk_widget_show_all(dialog);
    g_signal_connect_swapped(dialog,"response",G_CALLBACK(gtk_widget_destroy),dialog);
  }
  return FALSE;
}

gint popup_msg_wrap(char *msg){
  //  gdk_threads_enter();
  popup_msg(msg,TRUE);
  //  gdk_threads_leave();
  return FALSE;
}




void draw_vertical(dbuff *buff,GdkRGBA *col, float xvalf,int xvali){


  GdkRectangle rect;
  int sizex,sizey;
  float ll1,ll2;
  // routine will use either xvalf or xvali 
  // 0 < xvalf < 1, 0 < xvalf < number of pixels !

  sizex=buff->win.sizex;
  sizey=buff->win.sizey;

  if(buff->disp.dispstyle==SLICE_ROW){
    ll1=buff->disp.xx1;
    ll2=buff->disp.xx2;
  }
  else{
    ll1=buff->disp.yy1;
    ll2=buff->disp.yy2;
  }


  if (xvali == -1){
    rect.x=(int) ((xvalf-ll1)*(sizex-1)/
      (ll2-ll1)+1.5);

    //    fprintf(stderr,"in draw_vertical with rect.x = %i from float\n",rect.x);
  }
  else{ 
    rect.x = xvali;
    //    fprintf(stderr,"in draw_vertical with rect.x = %i from int\n",rect.x);
  }

    if (rect.x < 0 || rect.x >sizex){
      fprintf(stderr,"in draw row_vert, xval is out of range\n");
    }

  
    rect.y=1;
    rect.width=1;
    rect.height=sizey;

    cairo_set_source_rgb(buff->win.cr,col->red,col->green,col->blue);
    cairo_move_to(buff->win.cr,rect.x+0.5,1);
    cairo_line_to(buff->win.cr,rect.x+0.5,rect.height);
    cairo_stroke(buff->win.cr);
    //    gdk_draw_line(buff->win.pixmap,colourgc,rect.x,1,rect.x,sizey);
    gtk_widget_queue_draw_area (buff->win.darea, rect.x,rect.y,rect.width,rect.height);
}



GtkWidget * gtk_hbox_new_wrap(gboolean homo, gint spacing){

#if GTK_MAJOR_VERSION == 2
  return gtk_hbox_new(homo,spacing);
#else
  GtkWidget* temp;
  temp = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,spacing);
  gtk_box_set_homogeneous(GTK_BOX(temp),homo);
  return temp;
#endif

}

GtkWidget * gtk_vbox_new_wrap(gboolean homo, gint spacing){

#if GTK_MAJOR_VERSION == 2
  return gtk_vbox_new(homo,spacing);
#else
  GtkWidget* temp;
  temp = gtk_box_new(GTK_ORIENTATION_VERTICAL,spacing);
  gtk_box_set_homogeneous(GTK_BOX(temp),homo);
  return temp;
#endif

}
