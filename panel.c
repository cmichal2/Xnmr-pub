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

/* panel.c
 *
 * source code for Xnmr panel
 * Part of the Xnmr software project
 *
 * UBC Physics
 * April, 2000
 * 
 * written by: Scott Nelson, Carl Michal
 */


#include "panel.h"
#include "process_f.h"
#include "param_f.h"
#include "xnmr_ipc.h"      //indirectly dependant on shm_data.h
#include "p_signals.h"
#include "shm_data.h"
#include "buff.h"
#include "xnmr.h"

#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <gtk/gtk.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
 *  Global Variables
 */

volatile extern char redraw;
volatile char acq_in_progress;
GtkWidget* start_button;
GtkWidget* start_button_nosave;
GtkWidget* repeat_button;
GtkWidget* repeat_p_button;
GtkWidget* book;
GtkWidget* acq_label;
//GtkWidget* acq_2d_label;
GtkWidget* time_remaining_label;
//GtkWidget* completion_time_label;

int upload_buff = 0;
int last_upload_buff;  // used only for removing * from buffer names

int  setup_channels(){

  dbuff *buff;

  buff = buffp[current];
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.but1a))) data_shm->ch1 = 'A';
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.but1b))) data_shm->ch1 = 'B';
#if defined MSL200 || defined DUELIME || defined DUESDR
#else
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.but1c))) data_shm->ch1 = 'C';
#endif
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.but2a))) data_shm->ch2 = 'A';
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.but2b))) data_shm->ch2 = 'B';
#if defined MSL200 || defined DUELIME || defined DUESDR
#else
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(buff->win.but2c))) data_shm->ch2 = 'C';
#endif
  //  fprintf(stderr,"setup channels: ch1 %c, ch2: %c\n",data_shm->ch1,data_shm->ch2);
  if (data_shm->ch1 == data_shm->ch2){ 
    popup_msg("Channels 1 & 2 set to same hardware!\nNot proceeding.",TRUE);
    return -1;
  }
  return 0;
}


#ifndef MINGW

gint kill_button_clicked(GtkWidget *widget, gpointer *data)
{
  int was_in_progress,i,valid,count;
  //  fprintf(stderr,"in kill_button_clicked, acq_in_progress=%i\n",acq_in_progress);

  //  fprintf(stderr,"in kill clicked, setting not green\nin progress: %i, mode: %i\n",acq_in_progress,data_shm->mode);
  gtk_widget_modify_bg(buffp[upload_buff]->win.ct_box,GTK_STATE_NORMAL,NULL);
  send_sig_acq( ACQ_KILL ); 
  
  if (acq_in_progress == ACQ_STOPPED) return 0;

  /* if acq is listening, sends pulse program a sure death */

  //pop up any buttons which might be down
  was_in_progress = acq_in_progress;
  acq_in_progress = ACQ_STOPPED;

  if (was_in_progress == ACQ_RUNNING) {
    if (data_shm->mode == NORMAL_MODE){
      //      fprintf(stderr,"toggling start_button (save)\n");
      gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( start_button ), FALSE );
    }
    else{
      //      fprintf(stderr,"toggling start_button_nosave\n");
      gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( start_button_nosave ), FALSE );
    }
  }      
  if (was_in_progress == ACQ_REPEATING)
    gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( repeat_button ), FALSE );
  if (was_in_progress == ACQ_REPEATING_AND_PROCESSING)
    gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( repeat_p_button ), FALSE );

  // if there were experiments queued, kill them too.
  
  count = 0;
  for (i=0;i<queue.num_queued;i++){
    valid =  gtk_tree_model_get_iter_first(GTK_TREE_MODEL(queue.list),&queue.iter);
    if (valid == 1){
      gtk_list_store_remove(queue.list,&queue.iter);
      count += 1;
    }
    else
      fprintf(stderr,"in kill, num_queued is messed up\n");
  }
  queue.num_queued -= count;
  //  fprintf(stderr,"killed: %i experiments from the queue\n",count);
  set_queue_label();
  
  queue.num_queued = 0;

  data_shm->mode = NO_MODE;
  return 0;
}

gint start_button_toggled( GtkWidget *widget, gpointer *data )

{

  // in here with data = 0 means acq+save,data = 1 acq nosave
  dbuff* buff;
  
  char s[PATH_LENGTH];
  static char norecur=0;

  //    fprintf(stderr,"coming into start_button, norecur is: %i\n",norecur);
  
  buff = buffp[ current ];

  if (norecur == 1){
    norecur = 0;
    //    fprintf(stderr,"start button: returning on norecur=1\n");
    return 0;
  }

  if ( gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
    /* If control reaches here, the toggle button is down */
    //  fprintf(stderr,"widget is active\n");
    if( acq_in_progress == ACQ_STOPPED ) {
      char fileN[ PATH_LENGTH];

      // button coming down and we were stopped.  This is a start.

      if (array_popup_showing == 1){
	popup_msg("Can't start Acquisition with Array window open",TRUE);
	//	fprintf(stderr,"trying to start acq with popup showing\n");
	norecur = 1;
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(widget),FALSE); 
	//	fprintf(stderr,"returning on can't start with array open\n");
	return 0;
      }
	
      if (setup_channels() == -1){
	  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(widget),FALSE); 
	  //	  fprintf(stderr,"returning after sending normal start\n");
	  return 0;
      }

      if (widget == start_button ){
	//	fprintf(stderr,"setting mode: NORMAL_MODE\n");
	data_shm->mode = NORMAL_MODE;
      }
      else {
	//	fprintf(stderr,"setting mode NOSAVE\n");
	data_shm->mode = NORMAL_MODE_NOSAVE;
      }

      acq_process_data = buffp[ current ]->process_data;

      last_upload_buff = upload_buff;
      upload_buff = current;
      set_window_title(buffp[last_upload_buff]); // remove its star

      acq_param_set = current_param_set;

      // should really do this here, I think, and in repeat button, and repeat and process.
      send_paths(); // part of this is repeated below.

      if (data_shm->mode == NORMAL_MODE_NOSAVE) { // means its a start, not start and save
	path_strcpy( s, getenv(HOMEP));
	path_strcat( s,DPATH_SEP "Xnmr" DPATH_SEP "data" DPATH_SEP "acq_temp");
	path_strcpy( data_shm->save_data_path, s); 
      }
      else  // its a start and save, check to make sure filename is valid
	if (buff->param_set.save_path[strlen(buff->param_set.save_path)-1] == PATH_SEP){
	  popup_msg("Invalid file name",TRUE);
	  norecur = 1;
	  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(widget),FALSE); 
	  //	  fprintf(stderr,"returning with invalid file name\n");
	  return 0;
	}

      
      send_acqns();
      send_params();
      data_shm->dwell= buffp[current]->param_set.dwell; // this is what the pulse program will use

      path_strcpy(buffp[current]->path_for_reload,data_shm->save_data_path);
      set_window_title(buffp[current]);

      path_strcpy( fileN, data_shm->save_data_path);


      acq_in_progress = ACQ_RUNNING;
      redraw = 0;

      if (data_shm->mode != NORMAL_MODE_NOSAVE)
	if (check_overwrite(buffp[current],fileN) == FALSE){
	  acq_in_progress = ACQ_STOPPED;
	  norecur = 1;
	  gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( start_button ), FALSE );
	  return 0;
	}

      //      fprintf(stderr,"starting!\n");
      check_buff_size();  
      send_sig_acq( ACQ_START );
      set_acqn_labels(1);
#if GTK_MAJOR_VERSION == 2
      GdkColor color;      
      gdk_color_parse("green",&color);
      gtk_widget_modify_bg(buffp[upload_buff]->win.ct_box,GTK_STATE_NORMAL,&color);
#else
      GdkRGBA color = {0.,1.,0.,1.};
      gtk_widget_override_background_color(buffp[upload_buff]->win.ct_box,GTK_STATE_NORMAL,&color);
#endif
      set_window_title(buffp[upload_buff]); // add a * to the name

      return 0;
        

    }
    // so to be in here at all means the button is coming down, but we're not stopped
    // we just dealt with coming down and ACQ_STOPPED
    else {  
      if (acq_in_progress==ACQ_RUNNING){  // so this is button coming down but already running
	/*
	//	fprintf(stderr,"coming down but already running\n");
	if ((int) data == 0 && data_shm->mode == NORMAL_MODE) {
	  //	  fprintf(stderr,"button coming down but running data == 0\n");
	  return 0;
	}
	if ((int) data == 1 && data_shm->mode == NORMAL_MODE_NOSAVE){
	  //	  fprintf(stderr,"button coming down but running data == 1\n");
	  return 0;
	} 
	data isn't used for anything anymore... */
	fprintf(stderr,"button coming down but running\n");
      
	//	fprintf(stderr,"button coming down, but wasn't stopped setting norecur, raising button\n");
	norecur=1;
	gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( widget ), FALSE );
	
      }
         //otherwise, it's an invalid press because another button is down, set the button inactive
      // ie get here if we're repeating
      else{
	//	fprintf(stderr,"invalid press because other button is down - we're repeating?\n");
	norecur = 1;
	gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( widget ), FALSE );	
      }
    }
	
  }
  else {
    //  fprintf(stderr,"widget is inactive\n");
    /* If control reaches here, the toggle button is up */
    if( acq_in_progress == ACQ_RUNNING ) {  //a genuine stop
      //      fprintf(stderr,"got genuine stop, putting button back down\n");
      norecur = 1;
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(widget),TRUE); /* and put the
		button back down, it will come back up at the end of this scan */
      send_sig_acq( ACQ_STOP );
    }
      
    else {
      //      fprintf(stderr,"setting style nocolor\n");

      gtk_widget_modify_bg(buffp[upload_buff]->win.ct_box,GTK_STATE_NORMAL,NULL);
      data_shm->mode = NO_MODE;
      //      fprintf(stderr,"inactive and not running (normal stop?\n");
      /* this is raise of the button was not triggered by the user, 
	 but by a call to gtk_toggle_button_set_active from idle_button_up */
      if (script_widgets.acquire_notify != 0)
	g_idle_add_full(G_PRIORITY_LOW, (GSourceFunc) script_notify_acq_complete,NULL,NULL);
      // make sure this is done only after all the end of acquisition work is complete.
      //script_notify_acq_complete();
    } 
  }
  //  fprintf(stderr,"got to end of start button\n");
  return 0;
}





gint repeat_button_toggled( GtkWidget *widget, gpointer *data )
{
  static char norecur=0;
  if (norecur == 1){
    norecur = 0;
    return 0;
  }


  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
    /* If control reaches here, the toggle button is coming down */

    if( acq_in_progress == ACQ_STOPPED ) {
      if (setup_channels() == -1){
	norecur = 1;
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(widget),FALSE); 
	return 0;
      }
      check_buff_size();
      data_shm->mode = REPEAT_MODE;  
      acq_process_data = buffp[ current ]->process_data;
      //fprintf(stderr, "Sending start repeat signal to ACQ\n" );      

      last_upload_buff = upload_buff;
      upload_buff = current;
      set_window_title(buffp[last_upload_buff]); // remove its star

      acq_param_set = current_param_set;
      // should really do this here, I think, and in repeat button, and repeat and process.
      send_paths();
      send_acqns();
      send_params();
      data_shm->dwell= buffp[current]->param_set.dwell; // this is what the pulse program will use

      //      fprintf(stderr,"put %f in shm dwell\n",data_shm->dwell);
      path_strcpy(buffp[current]->path_for_reload,data_shm->save_data_path);
      set_window_title(buffp[current]);

#if GTK_MAJOR_VERSION == 2
      GdkColor color;      
      gdk_color_parse("green",&color);
      gtk_widget_modify_bg(buffp[upload_buff]->win.ct_box,GTK_STATE_NORMAL,&color);
#else
      GdkRGBA color = {0.,1.,0.,1.};
      gtk_widget_override_background_color(buffp[upload_buff]->win.ct_box,GTK_STATE_NORMAL,&color);
#endif

      acq_in_progress = ACQ_REPEATING;
      set_window_title(buffp[upload_buff]);
      redraw = 0;
      //      last_draw();
      send_sig_acq( ACQ_START );
      set_acqn_labels(1);
    }

    else {     //if this is an invalid press, set the button inactive
      norecur = 1;
      gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( widget ), FALSE );
    }
	
  }
  else {
      
    //fprintf(stderr, "repeat button lifted\n" );

    /* If control reaches here, the toggle button is up */
    if( acq_in_progress == ACQ_REPEATING ) {  //a genuine stop from the user
      //fprintf(stderr, "Sending stop signal to ACQ\n" );
      // put the button back down though, it will come up on its own at end of scan
      norecur = 1;
      gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON(widget),TRUE);
      send_sig_acq( ACQ_STOP );
    }
      
    else {// it actually comes up here.
      data_shm->mode = NO_MODE;
      gtk_widget_modify_bg(buffp[upload_buff]->win.ct_box,GTK_STATE_NORMAL,NULL);

    }
  }
  return TRUE;
  
}

gint repeat_p_button_toggled( GtkWidget *widget, gpointer *data )
{

  static char norecur = 0;
  if (norecur == 1){
    norecur = 0;
    return 0;
  }

  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON (widget))) {
    /* If control reaches here, the toggle button is down */

    if( acq_in_progress == ACQ_STOPPED ) {
      //fprintf(stderr, "Sending start repeat and process signal to ACQ\n" );
      if (setup_channels() == -1) {
	norecur = 1;
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(widget),FALSE); 
	return 0;
      }
      check_buff_size();
      data_shm->mode = REPEAT_MODE;        
      acq_process_data = buffp[ current ]->process_data;


      last_upload_buff = upload_buff;
      upload_buff = current;
      set_window_title(buffp[last_upload_buff]); // remove its star

      acq_param_set = current_param_set;
      // should really do this here, I think, and in repeat button, and repeat and process.
      send_paths();
      send_acqns();
      send_params();
      data_shm->dwell= buffp[current]->param_set.dwell; // this is what the pulse program will use

      //      fprintf(stderr,"put %f in shm dwell\n",data_shm->dwell);
#if GTK_MAJOR_VERSION == 2
      GdkColor color;      
      gdk_color_parse("green",&color);
      gtk_widget_modify_bg(buffp[upload_buff]->win.ct_box,GTK_STATE_NORMAL,&color);
#else
      GdkRGBA color = {0.,1.,0.,1.};
      gtk_widget_override_background_color(buffp[upload_buff]->win.ct_box,GTK_STATE_NORMAL,&color);
#endif

      acq_in_progress = ACQ_REPEATING_AND_PROCESSING;
      set_window_title(buffp[upload_buff]);
      redraw = 0;

      //      last_draw();
      send_sig_acq( ACQ_START );
      set_acqn_labels(1);
    }

    else {     //if this is an invalid press, set the button inactive
      norecur = 1;
      gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( widget ), FALSE );
      
    }
  }

  else {
      
    /* If control reaches here, the toggle button is up */
    if( acq_in_progress == ACQ_REPEATING_AND_PROCESSING ) {  //a genuine stop
      //fprintf(stderr, "Sending stop signal to ACQ\n" );
      norecur = 1;
      gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON(widget),TRUE);
      send_sig_acq( ACQ_STOP );
    }
      
    else { // button legitimately coming up.
      data_shm->mode = NO_MODE;
      gtk_widget_modify_bg(buffp[upload_buff]->win.ct_box,GTK_STATE_NORMAL,NULL);
    }
  }
  return 0;

}

#endif

GtkWidget* create_panels()

{
  GtkWidget *panhbox,*panvbox, *page, *label, *button;

  acq_in_progress = ACQ_STOPPED;
 
  //fprintf(stderr, "creating control panel\n" );

  book = gtk_notebook_new();

  panhbox=gtk_hbox_new_wrap(FALSE,0);
  panvbox=gtk_vbox_new_wrap(FALSE,0);

  gtk_notebook_set_tab_pos(GTK_NOTEBOOK(book),GTK_POS_BOTTOM);

  gtk_box_pack_start(GTK_BOX(panhbox),book,TRUE,TRUE,0);
  gtk_box_pack_start(GTK_BOX(panhbox),panvbox,FALSE,FALSE,0);

  label=gtk_label_new( "Process" );
  //  gtk_widget_show(label);

  page = create_process_frame();

  gtk_notebook_append_page(GTK_NOTEBOOK(book),page,label);

  label=gtk_label_new("Process 2d");
  //  gtk_widget_show(label);
  page = create_process_frame_2d();
  gtk_notebook_append_page(GTK_NOTEBOOK(book),page,label);


  label=gtk_label_new("Extras");
  page = create_extras_frame();
  gtk_notebook_append_page(GTK_NOTEBOOK(book),page,label);

  
  label = gtk_label_new( "Parameters" );
  gtk_widget_show(label);
  
  page = create_parameter_frame();
  //  gtk_widget_show(book);
  
  gtk_notebook_prepend_page(GTK_NOTEBOOK(book),page,label);


  gtk_notebook_set_current_page(GTK_NOTEBOOK(book),0);


    start_button = gtk_toggle_button_new_with_label( "Acquire and Save" );



  if(no_acq == FALSE){
  
    if( data_shm->mode == NORMAL_MODE && data_shm->acq_sig_ui_meaning != ACQ_DONE) {
      gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( start_button ), TRUE );
      acq_in_progress = ACQ_RUNNING;
    }
#ifndef MINGW
    g_signal_connect(G_OBJECT(start_button),"toggled",G_CALLBACK(start_button_toggled), NULL);
#endif
  }
  else
    gtk_widget_set_sensitive(GTK_WIDGET(start_button),FALSE);
    //    g_signal_connect (G_OBJECT(start_button),"toggled",G_CALLBACK(noacq_button_press),NULL);
  gtk_box_pack_start(GTK_BOX(panvbox),start_button,TRUE,TRUE,0);



  start_button_nosave = gtk_toggle_button_new_with_label( "Acquire" );
  script_widgets.acquire_button = start_button_nosave;

  if(no_acq == FALSE){
    if( data_shm->mode == NORMAL_MODE_NOSAVE && data_shm->acq_sig_ui_meaning != ACQ_DONE) {
      gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( start_button_nosave ), TRUE );
      acq_in_progress = ACQ_RUNNING;
    }
#ifndef MINGW
    g_signal_connect(G_OBJECT(start_button_nosave),"toggled",G_CALLBACK(start_button_toggled),NULL);
#endif
  }
  else
    gtk_widget_set_sensitive(GTK_WIDGET(start_button_nosave),FALSE);
  //    g_signal_connect (G_OBJECT(start_button_nosave),"toggled",G_CALLBACK(noacq_button_press),NULL);
  gtk_box_pack_start(GTK_BOX(panvbox),start_button_nosave,TRUE,TRUE,0);
    

  repeat_button = gtk_toggle_button_new_with_label( "Repeat" );
  if(no_acq ==FALSE){
  
    if( data_shm->mode == REPEAT_MODE ) {
      gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( repeat_button ), TRUE );
      acq_in_progress = ACQ_REPEATING;
    }
#ifndef MINGW
    g_signal_connect(G_OBJECT(repeat_button),"toggled",G_CALLBACK(repeat_button_toggled), NULL);
#endif
  }
  else
    gtk_widget_set_sensitive(GTK_WIDGET(repeat_button),FALSE);
  //    g_signal_connect (G_OBJECT(repeat_button),"toggled",G_CALLBACK(noacq_button_press),NULL);

  gtk_box_pack_start(GTK_BOX(panvbox),repeat_button,TRUE,TRUE,0);

  
  button = gtk_button_new_with_label( "Process" );
  script_widgets.process_button = button;
  g_signal_connect(G_OBJECT(button),"clicked",G_CALLBACK(process_data), NULL);
  gtk_box_pack_start(GTK_BOX(panvbox),button,TRUE,TRUE,0);


  repeat_p_button = gtk_toggle_button_new_with_label( "Repeat and Process" );
  if(no_acq ==FALSE){
#ifndef MINGW
    g_signal_connect(G_OBJECT(repeat_p_button),"toggled",G_CALLBACK(repeat_p_button_toggled), NULL);
#endif
  }
  else
    gtk_widget_set_sensitive(GTK_WIDGET(repeat_p_button),FALSE);
    //    g_signal_connect (G_OBJECT(repeat_p_button),"toggled",G_CALLBACK(noacq_button_press),NULL);
  gtk_box_pack_start(GTK_BOX(panvbox),repeat_p_button,TRUE,TRUE,0);

  button = gtk_button_new_with_label("Reload");
  g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( reload ), NULL );
  gtk_box_pack_start(GTK_BOX(panvbox),button,TRUE,TRUE,0);

  
  button = gtk_button_new_with_label("Kill");
  if( no_acq == FALSE ){
#ifndef MINGW
  g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( kill_button_clicked ), NULL );
#endif
  }
  else
    gtk_widget_set_sensitive(GTK_WIDGET(button),FALSE);
  //    g_signal_connect(G_OBJECT(button),"clicked", G_CALLBACK(noacq_kill_button_press), NULL);
  //  gtk_box_pack_start(GTK_BOX(panvbox),button,TRUE,TRUE,0);


  if ( no_acq == FALSE )
    acq_label = gtk_label_new( "Acq Stopped\n " );
  else
    acq_label = gtk_label_new( "NoAcq mode" );
  gtk_box_pack_start(GTK_BOX(panvbox),acq_label,TRUE,TRUE,0);


  /*
  acq_2d_label = gtk_label_new( "" );
  gtk_table_attach_defaults( GTK_TABLE(pantable),acq_2d_label,2,3,8,9 );
  gtk_widget_show( acq_2d_label );
  */

  time_remaining_label = gtk_label_new( " \n \n " );
  gtk_box_pack_start(GTK_BOX(panvbox),time_remaining_label,TRUE,TRUE,0);

  /*  completion_time_label = gtk_label_new( "completion:\n" );
  gtk_box_pack_start(GTK_BOX(panvbox),completion_time_label,TRUE,TRUE,0);
  gtk_widget_show(completion_time_label);
  */
  gtk_box_pack_start(GTK_BOX(panvbox),button,TRUE,TRUE,0);

  gtk_widget_show_all(panhbox);


  // so theme colors work:
  gtk_widget_set_name(repeat_button,"mybutton");
  gtk_widget_set_name(start_button,"mybutton");
  gtk_widget_set_name(start_button_nosave,"mybutton");
  gtk_widget_set_name(repeat_p_button,"mybutton");

  return panhbox;
  
}





void check_buff_size(){
  /* two places this number can get reset from which are awkward:
     here, and in upload routine */

  if (buffp[current]->acq_npts > MAX_DATA_POINTS)
    buffp[current]->acq_npts=MAX_DATA_POINTS;
  buff_resize( buffp[ current ], buffp[current]->acq_npts, 1 );


  data_shm->npts=buffp[current]->acq_npts;

  /* also set the new size in the box
     don't need to make sure that current set 
     is acq set because it must be */
  update_npts(buffp[current]->npts);
}
