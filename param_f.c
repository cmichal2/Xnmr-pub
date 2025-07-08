 /* param_f.c
 *
 * Implementation of the parameter panel page in Xnmr Software
 *
 * UBC Physics
 * April, 2000
 * 
 * written by: Scott Nelson, Carl Michal
 */


#include "param_f.h"
#include "shm_data.h"
#include "xnmr_ipc.h"
#include "panel.h"
#include "param_utils.h"
#ifdef MSL200
#include "h_config-pb.h"
#elif defined DUELIME
// this is here so that we get finely spaced values, even if Lime
// clock speed changes between compiles. When things are settled, this could become
// TX_SAMPLE_RATE
#define DEFAULT_RCVR_CLK TX_SAMPLE_RATE
#elif defined DUESDR
#include "h_config-duesdr.h"
#define DEFAULT_RCVR_CLK RX_SAMPLE_RATE // so we pick up AIRSPY vs RX88 
#else
#include "h_config.h" // for default receiver clock.
#endif
#include "xnmr.h"

#include <stdlib.h>
#include <unistd.h>
#include <gtk/gtk.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
/*
 *  Global Variables
 */
extern volatile char redraw; // from xnmr_ipc.c

GtkWidget *param_frame;
GtkWidget *param_scrolled_window;
GtkWidget *param_table;
GtkWidget *prog_text_box;
GtkWidget *save_text_box;
GtkAdjustment *acqs_adj;
GtkAdjustment *acqs_2d_adj;
GtkAdjustment *dwell_adj;
GtkAdjustment *sw_adj;
GtkAdjustment *npts_adj;
GtkWidget *acqs_spin_button;
GtkWidget *acqs_2d_spin_button;
GtkWidget *dwell_spin_button;
GtkWidget *sw_spin_button;
GtkWidget *npts_spin_button;

struct parameter_button_t param_button[ MAX_PARAMETERS ];
int num_buttons = 0;
long active_button = -1;
char doing_2d_update=0;

parameter_set_t* current_param_set = NULL;
parameter_set_t* acq_param_set = NULL;

struct popup_data_t popup_data;

char no_update_open = 0;
gpointer *update_data;
gpointer *update_obj;


/*
 * Update Methods
 *
 * These methods update the visible parameters
 */


gint am_i_queued(int test_bnum){
  int bnum,valid;

  valid =  gtk_tree_model_get_iter_first(GTK_TREE_MODEL(queue.list),&queue.iter);
  while (valid){
    gtk_tree_model_get(GTK_TREE_MODEL(queue.list),&queue.iter,
		       BUFFER_COLUMN,&bnum,-1);
    //    fprintf(stderr,"am_i_queued: checking buffer: %i\n",bnum);
    if (test_bnum == bnum){
      //      fprintf(stderr,"yes, I'm queued\n");
      return TRUE;
    }
    valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(queue.list),&queue.iter);
  }

  
  return FALSE;
}

gint allowed_to_change(int test_bnum){

  // short circuit
  if (acq_in_progress == ACQ_STOPPED || from_make_active == 1) return TRUE;
  if (upload_buff == test_bnum) return FALSE; // uploading to this buff
  if (am_i_queued(test_bnum) == TRUE) return FALSE;
  // if user tries to change one that's not acquiring or queued.
  return TRUE;
}

gint allowed_to_change_repeat(int test_bnum){ 
  if (acq_in_progress == ACQ_STOPPED || from_make_active == 1) return TRUE;

  if (am_i_queued(test_bnum) == TRUE || (upload_buff == test_bnum && acq_in_progress == ACQ_RUNNING))
    return FALSE;
  return TRUE; 

  /*
  if ((am_i_queued(test_bnum) == TRUE || (upload_buff == test_bnum && acq_in_progress == ACQ_RUNNING))
      && from_make_active == 0)
    return FALSE;
    return TRUE; */
}


void update_param( GtkAdjustment* adj, parameter_t* param ) // Parameters must be loaded first!

{
  static char norecur = 0;


  /*     fprintf(stderr,"coming into update_param, value: %f\n", 
  	  adj->value); 
     fprintf(stderr,"parameter is: %s, val_f %f, val_i %i\n",param->name,param->f_val, 
     param->i_val);  */


     if (norecur == 1){ 
       norecur = 0; 
       //       fprintf(stderr,"resetting no recur\n");
       return; 
       } 
    

     // so if the value hasn't changed, get out 


    if (doing_2d_update == 0 && from_make_active == 0 ){ // ok, so we're not doing 2d update 
      if(allowed_to_change_repeat(current) == FALSE){  
        //user punched a value 
	//        fprintf(stderr,"can't change value during acquistion\n");    
        switch(param->type) 
  	{ 
  	case 'i': 
  	  norecur = 1; 
	  gtk_adjustment_set_value( GTK_ADJUSTMENT( adj ), param->i_val);  
    	  break;  
    	case 'f':  
  	  norecur = 1; 
	  gtk_adjustment_set_value( GTK_ADJUSTMENT( adj ), param->f_val);  
    	  break;  
    	case 'F':  // these should set the value to the correct thing  
  	  norecur = 1; 
  	  if( data_shm->acqn_2d < param->size)  
	    gtk_adjustment_set_value(GTK_ADJUSTMENT(adj),   
				     param->f_val_2d[ data_shm->acqn_2d]);   
  	  break;  
    	case 'I':  
  	  norecur = 1; 
    	  if( data_shm->acqn_2d < param->size)  
	    gtk_adjustment_set_value(GTK_ADJUSTMENT(adj),   
				     param->i_val_2d[ data_shm->acqn_2d]);  
    	  break;  
    	}
	
	if (no_update_open == 0)
	  popup_no_update("Can't change parameter value in acquiring or queued window");  
        return; 
      }
    }

    // so here we're either doing 2d update, or we're not currently locked 
    // just put our parameter where its supposed to go.
    switch( param->type ) 
      { 
      case 'i': 
        param->i_val = (int) gtk_adjustment_get_value(adj); 
        break; 
      
      case 'f': 
        param->f_val = gtk_adjustment_get_value(adj); 
        break; 
      
      case 'F': 
      case 'I': 
        break; 
      
      default: 
        fprintf(stderr, "invalid type for parameter\n" ); 
        break; 
      } 
    if ((acq_in_progress==ACQ_REPEATING || acq_in_progress==ACQ_REPEATING_AND_PROCESSING)  
        && upload_buff==current) 
      send_params(); 
    return;   
} 



  gint update_t_param(GtkEntry* ent , parameter_t *event, parameter_t *param) 
  { 
    //    double temp; 
    char tempstr[PARAM_T_VAL_LEN];
    parameter_t *n_param=NULL;
    
    n_param = param;
    /* Aug 28, 2007.  This isn't reliable.  Put in wrap function for other cases.
    if (param == NULL) {
      //      fprintf(stderr,"got param == null, must be a direct activate callback\n");
      n_param = event;
    }
    else {
      //      fprintf(stderr,"got a param, is a focus out\n");
      n_param = param;
    }
    */

    // focus out doesn't seem to give the right param data back...
    /*    for (i=0;i<current_param_set->num_parameters;i++){
      if (ent  == (GtkEntry *)param_button[i].ent ){
	n_param = &current_param_set->parameter[i];
	i = current_param_set->num_parameters+1;
      }
    }
    if (n_param == NULL){
      fprintf(stderr,"couldn't find the paramter!!!\n");
      return FALSE;
      }*/
    //    fprintf(stderr,"param is %0x, should be:c %0x\n",param,n_param);


    if ( strncmp(n_param->t_val,gtk_entry_get_text(ent),PARAM_T_VAL_LEN)==0){
      //      fprintf(stderr,"t_param, got same value...\n"); // happens on focus loss with no change
      return FALSE;
    }
    /*
    if (n_param != param){
      fprintf(stderr,"update_t_param: different param from ent passed in\n");
      } 
    */

    if (allowed_to_change(current) == FALSE ){ 
      //      fprintf(stderr,"Can't change params while in progress\n"); 
      //      norecur = 1; 
      gtk_entry_set_text(ent,n_param->t_val); 
      //      norecur = 0;
      if (no_update_open == 0)
	popup_no_update("Can't change parameter value in acquiring or queued window");
      return FALSE; 
    } 

    switch( n_param->type ) 
      { 
      case 't': 
	strncpy(tempstr,gtk_entry_get_text(ent),PARAM_T_VAL_LEN);
	/*	if (isdigit((int)tempstr[0])){
	  //	  fprintf(stderr,"first char is a digit, reformatting\n");
	  sscanf(tempstr,"%lf",&temp); 
	  snprintf(tempstr,UTIL_LEN,"%.7f",temp); 
	  } */
        strncpy(n_param->t_val,tempstr,PARAM_T_VAL_LEN); 
	//        norecur=1; 
        gtk_entry_set_text(ent,tempstr); 
	//	fprintf(stderr,"storing %s\n",tempstr);
	//	norecur = 0;
        break; 
      
      default: 
        fprintf(stderr, "invalid type: %c for parameter\n",param->type ); 
        break; 
      } 
   
    if (upload_buff == current &&  
        (acq_in_progress == ACQ_REPEATING || 
         acq_in_progress == ACQ_REPEATING_AND_PROCESSING)) 
      send_params(); 
   return FALSE;   
  } 
gint update_t_param_wrap(GtkEntry* ent, parameter_t *event, parameter_t *param){
  return   update_t_param(ent,param,event);


}


/*
  void update_sw_dwell(){ 
    // there is a potentially subtle problem here, that when  
     //  we open a new buffer...  

    if( current_param_set != acq_param_set ) { 
      //    fprintf(stderr,"not updating sw/dwell\n"); 
      return; 
    } 

    // fprintf(stderr, "updating dwell time\n" ); 
    if (GTK_ADJUSTMENT(sw_adj)->value !=current_param_set->sw){
      gtk_adjustment_set_value( GTK_ADJUSTMENT( sw_adj ), current_param_set->sw); 

      // only need to do one of these because they will take care of each other 
      // gtk_adjustment_set_value( GTK_ADJUSTMENT( dwell_adj ), current_param_set->dwell);  
    }
  }

*/


  void update_npts(int npts){ 
    /* this routine isn't a callback - the callback for the npts 
       box is update_acqn  */

    gtk_adjustment_set_value( GTK_ADJUSTMENT( npts_adj ), npts); 

  } 
  void update_npts2(int npts){ 
    /* this routine isn't a callback - the callback for the npts 
       box is update_acqn  */

    gtk_adjustment_set_value( GTK_ADJUSTMENT( acqs_2d_adj ), npts); 

  } 

void update_acqs(int acqs){
  /* not a callback */

  gtk_adjustment_set_value(GTK_ADJUSTMENT( acqs_adj),acqs);

}


  void update_2d_buttons()      //This methods updates the buttons when acq moves to a new value in 2nd d
  { 
    int i; 
    int rec;

    if( current_param_set != acq_param_set ) { 
      // fprintf(stderr, "not updating 2d buttons\n" ); 
      return; 
    } 
    doing_2d_update=1; 
    rec = data_shm->acqn_2d ; // I think this is correct.
    if (rec < 0) rec = 0;
    for( i=0; i<num_buttons; i++ ) { 
      switch( current_param_set->parameter[i].type ) 
        { 
        case 'i': 
        case 'f': 
        case 't': 
  	break; 

        case 'I': 
  	if( rec < current_param_set->parameter[i].size ) 
  	  gtk_adjustment_set_value( GTK_ADJUSTMENT( param_button[i].adj ),  
  				    current_param_set->parameter[i].i_val_2d[ rec ] ); 
  	break; 

        case 'F': 
  	if( rec < current_param_set->parameter[i].size ) 
  	  gtk_adjustment_set_value( GTK_ADJUSTMENT( param_button[i].adj ),  
  				    current_param_set->parameter[i].f_val_2d[ rec ] ); 
  	break; 

        default: 
  	fprintf(stderr, "Xnmr: update_2d_buttons: invalid type for parameter"); 
  	break; 
        } 
    } 
    doing_2d_update=0; 
    return; 
  } 

  void update_2d_buttons_from_buff( dbuff* buff )  //This method updates the 2d parameters as the user moves through records in the buffer. 

  { 
    int i;

    if (&buff->param_set != current_param_set){
      popup_msg("Panic:  update_2d_buttons_from_buff, got a buffer that's not current?",TRUE);
      return;
    }
    for( i=0; i<num_buttons; i++ ) { 
      switch( current_param_set->parameter[i].type ) 
        { 
        case 'i': 
        case 'f': 
        case 't': 
  	break; 

        case 'I': 
  	if( buff->disp.record < current_param_set->parameter[i].size ) 
  	  gtk_adjustment_set_value( GTK_ADJUSTMENT( param_button[i].adj ), current_param_set->parameter[i].i_val_2d[ buff->disp.record ] ); 
  	break; 

        case 'F': 
  	if( buff->disp.record < current_param_set->parameter[i].size ) 
  	  gtk_adjustment_set_value( GTK_ADJUSTMENT( param_button[i].adj ), current_param_set->parameter[i].f_val_2d[ buff->disp.record ] );
  	break; 

        default: 
  	fprintf(stderr, "Xnmr: update_2d_buttons: invalid type for parameter\n" ); 
  	break; 
        } 
    } 
    return; 
  } 



int show_parameter_frame_wrap( parameter_set_t *current_param_set )
{

  // this is used just to to show the parameter frame  if we loaded a new pulse program in update_paths.
  // need this in case we destroy a widget that has the focus.

  if (  current_param_set != &buffp[current]->param_set)
    printf("show_parameter_frame_wrap - npts may be wrong in here!!!\n");

  //  gdk_threads_enter();
  //  fprintf(stderr,"showing parameter frame in mutex wrap\n");
  show_parameter_frame(current_param_set,buffp[current]->npts);
  
  //  gdk_threads_leave();
  return FALSE;
}

gint update_paths( GtkWidget* widget, gpointer data ) 
  { 

    char s[ PATH_LENGTH ],s2[PATH_LENGTH]; 
    char old_exec[ PATH_LENGTH ], *lpath=NULL; 
    //    static char norecur=0; 
    int result;
    char last_exec[ PATH_LENGTH ] = "";
    

    //    fprintf(stderr,"in update_paths\n");
    /*    if (data != NULL){
      fprintf(stderr,"in update_paths, with data not Null, must be focus out\n");
      } */

    /* don't need norecur here.  These boxes don't get an event on a set text...  
    if (norecur==1) {
      fprintf(stderr,"update_paths: coming through with norecur = 1\n");
      norecur=0; // entries don't seem to need this...
      return FALSE; 
      } */
    

    // if its the same as what it should be, don't worry about it.
    if ( widget == prog_text_box ){
      strncpy(last_exec,current_param_set->exec_path,PATH_LENGTH);
      if( strncmp(last_exec,gtk_entry_get_text(GTK_ENTRY(widget)),PATH_LENGTH) == 0) {
	//	fprintf(stderr,"same prog as last time\n");
	return FALSE;
       }
     }
    else if (widget == save_text_box){

      path_strcpy(s, current_param_set->save_path);
      lpath = strrchr(s,PATH_SEP);
      if (lpath != NULL)
	*lpath = 0;
      else lpath = s-1; // in case our save path has no / in it?
      
      if (strncmp(lpath+1,gtk_entry_get_text(GTK_ENTRY(widget)),PATH_LENGTH) == 0){
	//	fprintf(stderr,"same save as last time\n");
	return FALSE;
      }
    }


    if (allowed_to_change(current) == FALSE){ 
      //       fprintf(stderr,"can't change value during acquistion\n"); 
       if (no_update_open == 0)
	 popup_no_update("Can't change value in acquiring or queued window");  

      // put the old name back in.

      if (widget == prog_text_box){
	//	norecur = 1;
	//	fprintf(stderr,"resetting the program\n");
	gtk_entry_set_text(GTK_ENTRY(prog_text_box),last_exec);
	//	norecur = 0;
      }
      else{ // reset old save name.
	//	norecur = 1;
	//	fprintf(stderr,"resetting the save name\n");
	gtk_entry_set_text( GTK_ENTRY( save_text_box ), lpath +1  ); 
	//	norecur = 0;
      }



      return FALSE; 
    }

    //Save the old path for comparison 


    path_strcpy( old_exec, current_param_set->exec_path); 

    //set the new path 

    if( widget == prog_text_box ) { 
      // fprintf(stderr, "Previous pprog path: %s\n", old_exec ); 
      path_strcpy( current_param_set->exec_path, gtk_entry_get_text( GTK_ENTRY( prog_text_box) )); 
      //      fprintf(stderr, "new pprog path: %s\n", current_param_set->exec_path); 
    } 
    else if( widget == save_text_box ) { 
      path_strcpy(s,gtk_entry_get_text(GTK_ENTRY ( save_text_box)));
      //      fprintf(stderr,"update paths, new path is: %s\n",s);

      
      // then look for any other / (at end)
      lpath = strrchr(s,PATH_SEP);      
      if (lpath != NULL){
	*lpath = 0;
	path_strcpy(s2,lpath+1);
	result = set_cwd(s);
	if (result != 0) {
	  popup_msg("Directory not found",TRUE);
	  lpath = strrchr(current_param_set->save_path,PATH_SEP);
	  //	  norecur = 1;
	  gtk_entry_set_text(GTK_ENTRY(save_text_box),lpath+1);
	  //	  norecur = 0;
	  return FALSE; 
	}
	path_strcpy(current_param_set->save_path,getcwd(s,PATH_LENGTH));
	path_strcat(current_param_set->save_path,DPATH_SEP);
	path_strcat(current_param_set->save_path,s2);
	//	norecur = 1;
	gtk_entry_set_text(GTK_ENTRY(save_text_box),s2);
	//	norecur = 0;

      }
      else { // just a straightforward change of filename
	lpath = strrchr(current_param_set->save_path,PATH_SEP);
	//	norecur = 1;
	gtk_entry_set_text(GTK_ENTRY(save_text_box),s);
	//	norecur = 0;
	strncpy(lpath+1,s,PATH_LENGTH - strlen(current_param_set->save_path)-1);
      }
      update_param_win_title(&buffp[current]->param_set);
      //      fprintf(stderr, "update_paths: new save path: %s\n", current_param_set->save_path); 
    }
     
    /* 
     * reload the parameters only if necesary 
     * 
     * -this comparison has to be done because this function is called whenever 
     * the path box changes.  This includes when switching buffers, and we don't 
     * want to reload the parameters if this is the case 
      */

    
    if( strcmp( old_exec, current_param_set->exec_path) ) {       
      path_strcpy ( s, current_param_set->exec_path); 
      //      fprintf(stderr,"trying to load param_file\n");
      if (load_param_file( s, current_param_set ) != -1 ){ // this if is new CM Aug 24, 2004
	//	fprintf(stderr,"back from load without -1\n");


	/*
	if (current_param_set != &buffp[current]->param_set)
	  printf("about to call show_parameter_frame_mutex_wrap - npts may be wrong!!!\n");
	  current_param_set->tnpts = buffp[current]->npts; */

	g_idle_add ((GSourceFunc) show_parameter_frame_wrap, current_param_set) ; 


	/*
      // put the focus back on the program box...  gtk seems to crash if we try to focus out to a spin 
      // button that show_parameter_frame destroyed


	widget = gtk_window_get_focus(panwindow);
	if (widget == prog_text_box ) fprintf(stderr,"get_focus says its the prog text box\n");
	fprintf(stderr,"focus: %0x\n",widget);
	fprintf(stderr,"acqs_adj: %0x\n",acqs_adj);

	gtk_widget_grab_focus(GTK_WIDGET(prog_text_box));
	// this unselects the text in the program box.
	gtk_editable_select_region(GTK_EDITABLE(prog_text_box),0,0);
	// and puts the cursor at the end.
	gtk_editable_set_position(GTK_EDITABLE(prog_text_box),-1);
	*/

      }
    }
    return FALSE; 
  }

  void update_acqn( GtkAdjustment* adj, gpointer data ) 
       /* callback routine to update the numbers of scans, and also sw and  
  	dwell, and even the npts box */
  { 
    static char doing_sw_dwell=FALSE;  
    static char norecur = 0;
    //need this to prevent infinte looping in here? 

    if (norecur == 1){
      //      fprintf(stderr,"update_acqn, got norecur\n");
      norecur = 0;
      return;
    }

    //    if (allowed_to_change(current) == FALSE && !(adj == GTK_ADJUSTMENT(npts_adj) && current == upload_buff)){  // npts should be allowed to change in the acq buff.
    if (allowed_to_change(current) == FALSE && adj != GTK_ADJUSTMENT(npts_adj)){ // npts should be allowed anytime, but not acq_npts below
      // not allowed to change.  reset
      //      fprintf(stderr,"in update_acqn, not allowing change\n");
      if (adj ==  GTK_ADJUSTMENT(acqs_adj)){
	norecur = 1;
	//	fprintf(stderr,"update acqn: num_acqs\n");
	gtk_adjustment_set_value(GTK_ADJUSTMENT(acqs_adj),current_param_set->num_acqs);
      }
      else if (adj == GTK_ADJUSTMENT(acqs_2d_adj)){
	norecur = 1;
	//	fprintf(stderr,"update aqcn: acqs_2d\n");
	gtk_adjustment_set_value(GTK_ADJUSTMENT(acqs_2d_adj),current_param_set->num_acqs_2d);
      }
      else if (adj == GTK_ADJUSTMENT(dwell_adj)){
	norecur = 1;
	//	fprintf(stderr,"update aqcn: dwell\n");
	gtk_adjustment_set_value(GTK_ADJUSTMENT(dwell_adj),current_param_set->dwell);
      }
      else if (adj == GTK_ADJUSTMENT(sw_adj)){
	norecur = 1;
	//	fprintf(stderr,"update aqcn: sw_adj\n");
	gtk_adjustment_set_value(GTK_ADJUSTMENT(sw_adj),current_param_set->sw);
      }
      else{
	fprintf(stderr,"in update_acqn with unknown widget...\n");
      }
      if (no_update_open == 0)
	popup_no_update("Can't change parameter value in acquiring or queued window");  
      return;
    }


    if( adj == GTK_ADJUSTMENT( acqs_adj ) ){ 
      current_param_set->num_acqs = gtk_adjustment_get_value(adj); 
    } 
    else if( adj == GTK_ADJUSTMENT( acqs_2d_adj ) ) { 
      current_param_set->num_acqs_2d = gtk_adjustment_get_value(adj); 

    } 
    else if (adj== GTK_ADJUSTMENT( sw_adj )){ 
      //    fprintf(stderr,"in sw adj\n"); 
      current_param_set->sw = gtk_adjustment_get_value(adj); 
      if (!doing_sw_dwell) { 
	long int decimate;
        doing_sw_dwell=TRUE; 
	// get the closest we're going to get:
	decimate = (long int) (DEFAULT_RCVR_CLK/gtk_adjustment_get_value(adj)+0.5);
	printf("setting dwell to: %f\n",1./DEFAULT_RCVR_CLK*decimate*1e6);
        gtk_adjustment_set_value( GTK_ADJUSTMENT( dwell_adj ), 1./DEFAULT_RCVR_CLK*decimate*1e6 ); 
	current_param_set->sw = (long int) DEFAULT_RCVR_CLK/decimate; 
	//	printf("stored sw as: %li\n",current_param_set->sw);
        doing_sw_dwell=FALSE; 
      } 
    } 
    else if (adj == GTK_ADJUSTMENT( dwell_adj )){ 
      //    fprintf(stderr,"in dwell adj\n"); 
      current_param_set->dwell = gtk_adjustment_get_value(adj);
      //#if defined DUELIME || defined DUESDR
      //#else
      if(!doing_sw_dwell){ 
	long int decimate;
        doing_sw_dwell=TRUE; 
	decimate = (long int) (gtk_adjustment_get_value(adj)/1e6*DEFAULT_RCVR_CLK +0.5);
	printf("in dwell, decimate is: %li, clock is %i\n",decimate,DEFAULT_RCVR_CLK);
	//	printf("setting sw to %g\n",DEFAULT_RCVR_CLK/decimate);
        gtk_adjustment_set_value(GTK_ADJUSTMENT( sw_adj ),DEFAULT_RCVR_CLK/decimate); 
	current_param_set->dwell = 1e6*decimate/DEFAULT_RCVR_CLK; 
	printf("stored dwell as: %g\n",current_param_set->dwell);
        doing_sw_dwell=FALSE; 
      } 
      //#endif
    } 
    else if (adj== GTK_ADJUSTMENT( npts_adj)){ 
      if (gtk_adjustment_get_value(adj) != buffp[current]->npts){ 
        buff_resize(buffp[current],gtk_adjustment_get_value(adj),buffp[current]->npts2); 
	if (allowed_to_change(current) == TRUE){
	  //	  fprintf(stderr,"resetting acq_npts in buff %i",current);
	  buffp[current]->acq_npts = gtk_adjustment_get_value(adj);  /* always set this here, if we're doing a 
						       zero fill, the zf routine will restore it  */
	}
        draw_canvas(buffp[current]);   
	//	fprintf(stderr," resized to %f\n",adj->value);
      } 
    } 
	   
    else { 
      fprintf(stderr, "Error occured in update_acqn\n" ); 
      return; 
    } 


    return; 
  } 

  /* 
   * Send Methods 
   * 
   * These methods update the shared memory structure if the correct conditions are met 
    */

  void send_acqns() 
  { 
    switch( acq_in_progress ) 
      { 
      case ACQ_STOPPED: 
        // fprintf(stderr, "loading acqns from current buffer\n" ); 
        if(no_acq ==FALSE){ 
  
  	data_shm->num_acqs = current_param_set->num_acqs; 
  	data_shm->num_acqs_2d = current_param_set->num_acqs_2d; 
        } 
        break; 
      case ACQ_RUNNING: 
        // fprintf(stderr, "not uploading acqn\n" ); 
        break; 
      case ACQ_REPEATING: 
      case ACQ_REPEATING_AND_PROCESSING: 
        // fprintf(stderr, "loading acqns from acq buffer\n" ); 
        data_shm->num_acqs = acq_param_set->num_acqs; 
        data_shm->num_acqs_2d = acq_param_set->num_acqs_2d; 
        break; 
      default: 
        fprintf(stderr, "send_acq: invalid mode\n" ); 
        break; 
      } 

    return; 
  } 

  void send_params() 
  { 
    parameter_set_t* p_set; 

    switch( acq_in_progress ) 
      { 
      case ACQ_STOPPED: 
        // fprintf(stderr, "uploading params from current buffer\n" ); 
        p_set = current_param_set; 
        break; 
      case ACQ_RUNNING: 
        // fprintf(stderr, "not uploading params\n" ); 
        p_set = NULL; 
        return; 
        break; 
      case ACQ_REPEATING: 
      case ACQ_REPEATING_AND_PROCESSING: 

        if( acq_param_set != current_param_set ) {     
  	/* 
  	 * No parameters could have changed, so just stop right now and save  
  	 * the effort.  This check isn't warranted in the other upload methods 
  	 * because all they do is copy a single variable.  Generating the 
  	 * parameter string is a lot of work that could be avoided 
  	  */
	  //  	fprintf(stderr, "skipping parameter upload\n" ); 
  	return; 
        } 

        // fprintf(stderr, "uploading params from acq buffer\n" ); 
        p_set = acq_param_set; 
        break; 
      default: 
        fprintf(stderr, "send_params: invalid mode\n" ); 
        return; 
        break; 
      } 
    if(no_acq ==FALSE){ 
      make_param_string( p_set, data_shm->parameters ); 
    } 
    // fprintf(stderr, "new parameter string is:\n%s", data_shm->parameters ); 
    return; 

  } 

  void send_paths() 

  { 
    switch( acq_in_progress ) 
      { 
      case ACQ_STOPPED: 
        //      fprintf(stderr, "uploading path from current buffer\n" ); 
        if(no_acq ==FALSE){ 
  	path_strcpy( data_shm->pulse_exec_path, current_param_set->exec_path); 

	path_strcpy(data_shm->save_data_path,current_param_set->save_path); 
	//	fprintf(stderr,"send paths: put %s in shm\n",data_shm->save_data_path);
        } 
        break; 
      case ACQ_REPEATING: 
      case ACQ_REPEATING_AND_PROCESSING: 
      case ACQ_RUNNING: 
        fprintf(stderr, "not uploading path\n" ); 
        break; 
        break; 
      default: 
        fprintf(stderr, "send_paths: invalid mode\n" ); 
        break; 
      } 
    return; 
  } 

  GtkWidget* create_parameter_frame( ) 

  { 

    GtkWidget *label; 
    GtkWidget *button,*vbox; 
    int i; 

    // fprintf(stderr, "creating the parameter frame\n" ); 

    for( i=0; i<MAX_PARAMETERS; i++ ) { 
      param_button[i].adj = NULL; 
      param_button[i].button = NULL; 
      param_button[i].label = NULL; 
    } 

    num_buttons = 0; 

    param_frame = gtk_frame_new("Parameters"); 

    gtk_container_set_border_width(GTK_CONTAINER (param_frame),5); 
    //     gtk_widget_set_size_request(param_frame,900,300); 

    param_scrolled_window =  gtk_scrolled_window_new( NULL, NULL);
				     
    gtk_container_add(GTK_CONTAINER(param_frame),param_scrolled_window);

    /* arguments are homogeneous and spacing  */
    param_table = gtk_table_new(3,6,FALSE); /* rows, columns, homogeneous  */
    

    //    gtk_table_set_row_spacings(GTK_TABLE(param_table),1);


    vbox=gtk_vbox_new_wrap(FALSE,0);
    gtk_box_pack_start(GTK_BOX(vbox),param_table,FALSE,FALSE,0);
    label=gtk_label_new("");
    gtk_box_pack_start(GTK_BOX(vbox),label,TRUE,FALSE,0);

    gtk_scrolled_window_add_with_viewport(GTK_SCROLLED_WINDOW(param_scrolled_window),vbox); 

    gtk_scrolled_window_set_policy ( GTK_SCROLLED_WINDOW(param_scrolled_window),
				     GTK_POLICY_NEVER, GTK_POLICY_AUTOMATIC);

  


    /* 
     *  Now start to put parameter fields into the frame 
      */

    label = gtk_label_new( "Pulse Program:" ); 

    gtk_table_attach_defaults(GTK_TABLE(param_table),label,0,1,0,1); 
    

    label = gtk_label_new( "Save Path:" ); 
    gtk_table_attach_defaults(GTK_TABLE(param_table),label,0,1,1,2); 

    label = gtk_label_new( "#Acquisitions:" ); 
    gtk_table_attach_defaults(GTK_TABLE(param_table),label,3,4,0,1); 

    label = gtk_label_new( "#Acquisitions 2d:" ); 
    gtk_table_attach_defaults(GTK_TABLE(param_table),label,3,4,1,2); 

  
    label = gtk_label_new( "dwell (us)" ); 
    gtk_table_attach_defaults(GTK_TABLE(param_table),label,2,3,2,3); 

    label = gtk_label_new( "sw (Hz)" ); 
    gtk_table_attach_defaults(GTK_TABLE(param_table),label,0,1,2,3); 
  
    label = gtk_label_new( "npts" ); 
    gtk_table_attach_defaults(GTK_TABLE(param_table),label,4,5,2,3); 




    dwell_adj = (GtkAdjustment *) gtk_adjustment_new(1,0.01,1000000,1,10,0 ); 
    sw_adj = (GtkAdjustment *) gtk_adjustment_new(1000000,1,100000000,1,10,0 ); 
    npts_adj = (GtkAdjustment *) gtk_adjustment_new(2048,1,MAX_DATA_NPTS,1,1,0); 

    dwell_spin_button = gtk_spin_button_new( GTK_ADJUSTMENT( dwell_adj ), 0.5, 3 ); 
    sw_spin_button = gtk_spin_button_new( GTK_ADJUSTMENT( sw_adj ), 0.5, 0 ); 
    npts_spin_button = gtk_spin_button_new(GTK_ADJUSTMENT(npts_adj),0.5,0); 
  
    g_signal_connect (G_OBJECT (dwell_adj), "value_changed", 
  		      G_CALLBACK (update_acqn), NULL ); 
 
    g_signal_connect (G_OBJECT (sw_adj), "value_changed", 
  		      G_CALLBACK (update_acqn), NULL ); 

    g_signal_connect (G_OBJECT (npts_adj), "value_changed", 
  		      G_CALLBACK (update_acqn), NULL ); 

    gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( dwell_spin_button ), GTK_UPDATE_IF_VALID ); 
    gtk_table_attach_defaults(GTK_TABLE(param_table),dwell_spin_button,3,4,2,3); 

    gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( sw_spin_button ), GTK_UPDATE_IF_VALID ); 
    gtk_table_attach_defaults(GTK_TABLE(param_table),sw_spin_button,1,2,2,3); 

    gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( npts_spin_button ), GTK_UPDATE_IF_VALID ); 
    gtk_table_attach_defaults(GTK_TABLE(param_table),npts_spin_button,5,6,2,3); 

    prog_text_box = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(prog_text_box), PATH_LENGTH ); 
    gtk_table_attach_defaults(GTK_TABLE(param_table),prog_text_box,1,3,0,1); 
    if(no_acq==FALSE) 
      gtk_entry_set_text( GTK_ENTRY( prog_text_box ), (gchar*)data_shm->pulse_exec_path ); 
    else 
      gtk_entry_set_text(GTK_ENTRY(prog_text_box),"~/prog"); 

    save_text_box = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(save_text_box),PATH_LENGTH ); 

    gtk_table_attach_defaults(GTK_TABLE(param_table),save_text_box,1,3,1,2); 
    if(no_acq==FALSE) 
      gtk_entry_set_text( GTK_ENTRY( save_text_box ), (gchar*)data_shm->save_data_path ); 
    else 
      gtk_entry_set_text(GTK_ENTRY(save_text_box),"~/data"); 

    g_signal_connect( G_OBJECT( prog_text_box ), "activate",  
		      G_CALLBACK( update_paths ), NULL ); 

    g_signal_connect( G_OBJECT( prog_text_box ), "focus_out_event",  
		      G_CALLBACK( update_paths ), NULL ); 


    g_signal_connect( G_OBJECT( save_text_box ), "activate",  
  		      G_CALLBACK( update_paths ), NULL ); 

    g_signal_connect( G_OBJECT( save_text_box ), "focus_out_event",  
      		      G_CALLBACK( update_paths ), NULL ); 
    //  this nearly works well, except if we lose focus during an acquisition
    // get annoying messages about not being able to change.
    // 


    acqs_adj = (GtkAdjustment *) gtk_adjustment_new(1,1,10000000,1,10,0 ); 
    acqs_2d_adj = (GtkAdjustment *) gtk_adjustment_new(1,1,65535,1,10,0 ); 
  
    acqs_spin_button = gtk_spin_button_new( GTK_ADJUSTMENT( acqs_adj ), 0.5, 0 ); 
    acqs_2d_spin_button = gtk_spin_button_new( GTK_ADJUSTMENT( acqs_2d_adj ), 0.5, 0 ); 

    g_signal_connect (G_OBJECT (acqs_adj), "value_changed", 
  		      G_CALLBACK (update_acqn), NULL ); 

    g_signal_connect (G_OBJECT (acqs_2d_adj), "value_changed", 
  		      G_CALLBACK (update_acqn), NULL ); 

    gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( acqs_spin_button ), GTK_UPDATE_IF_VALID ); 
    gtk_table_attach_defaults(GTK_TABLE(param_table),acqs_spin_button,4,5,0,1); 

    gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( acqs_2d_spin_button ), GTK_UPDATE_IF_VALID ); 
    gtk_table_attach_defaults(GTK_TABLE(param_table),acqs_2d_spin_button,4,5,1,2); 



    button = gtk_button_new_with_label( "Array" ); 
    gtk_table_attach_defaults(GTK_TABLE(param_table),button,5,6,0,1); 
    g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( show_2d_popup ), &active_button ); 




    create_2d_popup(); 
    gtk_widget_show_all(param_frame);

    return param_frame; 
  } 


  int load_param_file( char* fileN, parameter_set_t *param_set ) 
  { 
    FILE* fs; 
    char s[PATH_LENGTH]; 
    int i = 0; 
    int j,k; 
    int result; 
    char type; 
    parameter_set_t old_param_set; 
    char message_printed = 0;



    //    fprintf(stderr,"in load_param_file with %s\n",fileN);
    // first look in ~/Xnmr/prog/  then in /usr/share/Xnmr/prog/
    path_strcpy(s,getenv(HOMEP));
    path_strcat(s,DPATH_SEP "Xnmr" DPATH_SEP "prog" DPATH_SEP);
    path_strcat(s , fileN);
    
    //    fprintf(stderr,"now looking in: %s\n",s);
    fs = fopen( s, "r" ); 
    
    if( fs == NULL ) { 
      path_strcpy(s,SYS_PROG_PATH);
      path_strcat(s,fileN);
      //      fprintf(stderr,"now looking in: %s\n",s);
      fs = fopen( s, "r" ); 
      if( fs == NULL){
	if (buffp[current] == NULL) // if the buffer doesn't exist yet, delay the popup
	  g_idle_add ((GSourceFunc) popup_msg_wrap,"Can't find pulse program");
	else
	  popup_msg("Can't find pulse program",TRUE);
	//	fprintf(stderr, "Can't open parameter file %s\n", fileN ); 
	return -1; 
      } 
    }
    
    fclose(fs);
    path_strcat(s,".param");
    fs = fopen( s ,"r" );
  
    
    if (fs == NULL) {
       popup_msg("Can't find parameter file",TRUE);
      return -1;
    }
    //    fprintf(stderr,"got param file on: %s\n",s);
    
    
 

      // ok, so if we can't find the param file, send a message, but leave all the old params in memory?
    memcpy(&old_param_set,param_set,sizeof(parameter_set_t)); 

    //clear the memory; 
    for( i=0; i<MAX_PARAMETERS; i++ ) { 
      param_set->parameter[i].i_val_2d = NULL; 
      param_set->parameter[i].f_val_2d = NULL; 
      param_set->parameter[i].size = 0; 
      param_set->parameter[i].unit=1; 
    } 
 
    // fprintf(stderr, "loading %s\n", fileN ); 

    //Start parsing the file 

    i = 0; 

    while( !feof( fs ) ) { 
      s[0] = 0; // this prevents us from reading the last line twice if the file ends with a single carriage return after the last record.
      fgets( s, 200, fs ); 

      //Check to see if this line is a comment or not 

      if( s[0] != '#' ) { 
        result = sscanf( s, PARAMETER_FILE_FORMAT, param_set->parameter[i].name, &type ); 
	//	printf("line is: %s\n",s);
	//	printf("result is: %i, name is: %s\n",result,param_set->parameter[i].name);
        // in here we need to see if each new one has the same name and type as an old one, and if so, keep it 
        if( result == 2 ) {  //This indicates a valid format 
  	switch( type ) 
  	  { 
  	  case 'i': 
  	    // fprintf(stderr, "loading parameter %d of type int\n",i ); 
  	    param_set->parameter[i].type = 'i'; 
  	    sscanf( s, PARAMETER_FILE_FORMAT_INT, param_set->parameter[i].name, &param_set->parameter[i].i_val,  
  		    &param_set->parameter[i].i_min, &param_set->parameter[i].i_max, &param_set->parameter[i].i_step, &param_set->parameter[i].i_page ); 
  	    for(j=0;j<old_param_set.num_parameters;j++){ 
  	      if (strcmp(param_set->parameter[i].name,old_param_set.parameter[j].name) == 0){ 
  		param_set->parameter[i].i_val = old_param_set.parameter[j].i_val; 
  		  // must also do 2d 
  		//  fprintf(stderr,"copied old val from %s\n",old_param_set.parameter[j].name); 


  		if (old_param_set.parameter[j].type == 'I'){ 
  		  //		  fprintf(stderr,"old param was an array\n"); 
  		  param_set->parameter[i].type = 'I'; 
  		  param_set->parameter[i].size=old_param_set.parameter[j].size; 
  		  param_set->parameter[i].i_val_2d=g_malloc(param_set->parameter[i].size *sizeof(gint)); 
		  //		  fprintf(stderr,"load_param_file: malloc\n");
  		  for(k=0;k<param_set->parameter[i].size;k++)  
  		    param_set->parameter[i].i_val_2d[k]=old_param_set.parameter[j].i_val_2d[k]; 
		  
  		} 
  		  j=old_param_set.num_parameters; 

  	      } 
  	    } 
  	    i++; 
  	    break; 

  	  case 'f': 
  	    // fprintf(stderr, "loading parameter %d of type float\n",i ); 
  	    param_set->parameter[i].type = 'f'; 
  	    sscanf( s, PARAMETER_FILE_FORMAT_DOUBLE, param_set->parameter[i].name, &param_set->parameter[i].f_val,   
  		    &param_set->parameter[i].f_min,  
  		    &param_set->parameter[i].f_max, &param_set->parameter[i].f_step,  
  		    &param_set->parameter[i].f_page, &param_set->parameter[i].f_digits, 
  		    &param_set->parameter[i].unit_c); 
  	    //	    fprintf(stderr,"got unit %c\n",param_set->parameter[i].unit_c); 
  	    switch (param_set->parameter[i].unit_c){ 
  	    case 'M': 
  	      param_set->parameter[i].unit=1e6; 
	      strcpy(param_set->parameter[i].unit_s,"e6");
  	      break; 
  	    case 'k': 
  	      param_set->parameter[i].unit=1e3; 
	      strcpy(param_set->parameter[i].unit_s,"e3");
  	      break; 
  	    case '-': 
  	    case ' ': 
  	    case '1': 
	      strcpy(param_set->parameter[i].unit_s,"");
  	      param_set->parameter[i].unit=1; 
	      break;
	    case 'm': 
	      strcpy(param_set->parameter[i].unit_s,"e-3");
	      param_set->parameter[i].unit=1e-3; 
	      break; 
  	    case 'u': 
	      strcpy(param_set->parameter[i].unit_s,"e-6");
  	      param_set->parameter[i].unit=1e-6; 
  	      break; 
  	    default: 
	      strcpy(param_set->parameter[i].unit_s,"");
  	      param_set->parameter[i].unit=1; 
  	      param_set->parameter[i].unit_c='-'; 
  	    } 
  	    for(j=0;j<old_param_set.num_parameters;j++){ 
  	      if (strcmp(param_set->parameter[i].name,old_param_set.parameter[j].name) == 0 ){ 
		if ( old_param_set.parameter[j].type == 't') { // was a string, is now a double
		  sscanf(old_param_set.parameter[j].t_val,"%lf",&param_set->parameter[i].f_val);
		  param_set->parameter[j].f_val /= param_set->parameter[j].unit;
		  //		  fprintf(stderr,"was string, now double, got val %f\n",param_set->parameter[i].f_val);
		}
		else if (old_param_set.parameter[j].type == 'f' || old_param_set.parameter[j].type =='F') { 
		  param_set->parameter[i].f_val = old_param_set.parameter[j].f_val*old_param_set.parameter[j].unit/param_set->parameter[i].unit; 
		  //		fprintf(stderr,"copied old val from %s\n",old_param_set.parameter[j].name); 
  		  // must also do 2d 
		  if (old_param_set.parameter[j].type == 'F'){ 
		    //		  fprintf(stderr,"old param was an array\n"); 
		    param_set->parameter[i].type = 'F'; 
		    param_set->parameter[i].size=old_param_set.parameter[j].size; 
		    param_set->parameter[i].f_val_2d=g_malloc(param_set->parameter[i].size *sizeof(double)); 
		    //		  fprintf(stderr,"load_param_file: malloc\n");
		    for(k=0;k<param_set->parameter[i].size;k++)  
		      param_set->parameter[i].f_val_2d[k]=old_param_set.parameter[j].f_val_2d[k]; 
		 
		  }
		}
		else fprintf(stderr,"new param is float, old isn't f, F or t for param: %s\n",param_set->parameter[i].name);
  		j=old_param_set.num_parameters; 
  	      }
  	    }

  	    i++; 
  	    break; 

  	  case 't': 
  	    // fprintf(stderr, "loading parameter %d of type text\n",i ); 

  	    param_set->parameter[i].type = 't'; 
  	    sscanf( s, PARAMETER_FILE_FORMAT_TEXT, param_set->parameter[i].name, param_set->parameter[i].t_val ); 


	    // if this is a text parameter for sf, print a warning.
	    if (strstr(param_set->parameter[i].name,"sf") != NULL){
	      if (message_printed == 0){
		fprintf(stderr,"\n\ntext parameters are no longer needed for double. Change types to double in pulse progs and use GET_PARAMETER_DOUBLE\n");
		fprintf(stderr,"also change frequency lines in param file to: sf1 f 55.84 0 500 .001 .01 7 -\n\n");
		message_printed = 1;
	      }
	    }

  	    for(j=0;j<old_param_set.num_parameters;j++){ 
  	      if (strcmp(param_set->parameter[i].name,old_param_set.parameter[j].name) == 0){ 
		if (old_param_set.parameter[j].type == 'f'){
		  sprintf(param_set->parameter[i].t_val,"%.7lf",old_param_set.parameter[j].f_val*old_param_set.parameter[j].unit);
		}
		else if (old_param_set.parameter[j].type == 't'){ // assume it was also a 't'
		  strncpy(param_set->parameter[i].t_val,old_param_set.parameter[j].t_val,PARAM_T_VAL_LEN); 
		}
		else fprintf(stderr,"new parameter is t, old type unrecognized for param: %s.  old type: %c\n",param_set->parameter[i].name,old_param_set.parameter[j].type);
  		j=old_param_set.num_parameters; 
  	      } 
  	    } 

  	    i++; 
  	    break; 
	    
  	  default: 
  	    fprintf(stderr, "invalid parameter type. . . ignoring\n" ); 
  	    break; 
  	  } 

        }//if 
      }//if 
    } //while 

    param_set->num_parameters = i; 
    // now try to plug memory leaks in arrays 

    clear_param_set_2d(&old_param_set);
    /*
    for(i=0;i<old_param_set.num_parameters;i++) 
      if(old_param_set.parameter[i].type == 'F'){ 
  	g_free(old_param_set.parameter[i].f_val_2d); 
      } 
      else if(old_param_set.parameter[i].type == 'I'){ 
	g_free(old_param_set.parameter[i].i_val_2d); 
      } 
      else fprintf(stderr,"%s had size bigger than 1, but type was: %c\n",old_param_set.parameter[i].name, 
  		  old_param_set.parameter[i].type); 
    */

   
    fclose( fs ); 

    return 0; 
  } 

  gint param_spin_pressed( GtkWidget* widget, GdkEventButton *event, gpointer data ) 
  { 
    // fprintf(stderr, "new active button is %d\n", (int) data ); 
    active_button = (long) data; 
    return FALSE;  // new for gtk+-2.0
  } 


  void show_parameter_frame( parameter_set_t *param_set, int npts) 
  { 
    long i,tab_height; 
    char s[PATH_LENGTH],*lpath; 

    current_param_set = param_set; 
    active_button = -1; 

    // fprintf(stderr, "showing parameter frame\n" ); 

    //Clear out the previous parameter set 

    //    fprintf(stderr, "removing %d buttons\n", num_buttons ); 

    for( i=0; i<num_buttons; i++ ) { 

      if( param_button[i].adj != NULL ) { 
	// the adjustment gets destroyed with the spin button for reasons I don't understand.
	//	printf("count is: %i\n",G_OBJECT(param_button[i].adj)->ref_count);
        gtk_widget_hide( GTK_WIDGET( param_button[i].button ) ); 
        gtk_widget_destroy( GTK_WIDGET( param_button[i].button ) ); 
	//	printf("count is: %i\n",G_OBJECT(param_button[i].adj)->ref_count);
	//	printf("destroying an adjust:\n");
	//	gtk_widget_destroy( GTK_WIDGET(param_button[i].adj) ); 
       	//g_object_unref(G_OBJECT(param_button[i].adj));
	//	printf("done destroying an adjust:\n");
      } 
      else { 
        gtk_widget_hide( GTK_WIDGET( param_button[i].ent ) ); 
        gtk_widget_destroy( GTK_WIDGET( param_button[i].ent ) ); 
      } 
      gtk_widget_hide( GTK_WIDGET( param_button[i].label ) ); 
      gtk_widget_destroy( GTK_WIDGET( param_button[i].label ) ); 

      param_button[i].adj = NULL; 
      param_button[i].label = NULL; 
      param_button[i].button = NULL; 
      param_button[i].ent = NULL; 

    } 

    num_buttons = 0; 

    /* 
     * setting the text box leads to a call to update_paths, which will ensure  
     * that the correct parameter file is loaded before we start to create buttons, etc. 
      */
    gtk_entry_set_text( GTK_ENTRY( prog_text_box ), param_set->exec_path ); 

    path_strcpy(s, param_set->save_path);
    lpath = strrchr(s,PATH_SEP);
    if (lpath != NULL)
      *lpath = 0;
    else lpath = s-1; // in case our save path has no / in it?
    // this sets the current wd to where this buffer wants it, and sets the
    // title in the parameter window
    set_cwd(s);
    update_param_win_title(param_set);

    // this puts the actual name into the box.
    gtk_entry_set_text( GTK_ENTRY( save_text_box ), lpath +1  ); 

    gtk_adjustment_set_value( GTK_ADJUSTMENT( acqs_adj ), param_set->num_acqs); 
    gtk_adjustment_set_value( GTK_ADJUSTMENT( acqs_2d_adj ), param_set->num_acqs_2d); 
    gtk_adjustment_set_value( GTK_ADJUSTMENT( sw_adj ),  param_set->sw); 
    gtk_adjustment_set_value( GTK_ADJUSTMENT( dwell_adj ), param_set->dwell); 
    gtk_adjustment_set_value( GTK_ADJUSTMENT( npts_adj ), npts); 

    /* 
     *  Now we have to create some buttons 
      */

    // fprintf(stderr, "building %d buttons\n", param_set->num_parameters ); 
    tab_height = 3+(param_set->num_parameters+2)/3;
    if (tab_height <  3) tab_height = 3;

    gtk_table_resize(GTK_TABLE(param_table),tab_height,6);
    for( i=0; i < param_set->num_parameters; i++ ) { 

      strcpy( s, "" ); 

      //      fprintf(stderr, "trying to build a button of type %c\n", param_set->parameter[i].type ); 

      switch( param_set->parameter[i].type ) 
        { 
        case 'I': 
	  strcpy( s, "*" ); 
        case 'i': 
	  strncat( s, param_set->parameter[i].name,PARAM_NAME_LEN-1); 
	  param_button[i].adj = (GtkAdjustment *) gtk_adjustment_new( param_set->parameter[i].i_val, param_set->parameter[i].i_min, param_set->parameter[i].i_max, 
						    param_set->parameter[i].i_step, param_set->parameter[i].i_page, 0 ); 

	  param_button[i].button = gtk_spin_button_new( GTK_ADJUSTMENT( param_button[i].adj ), 0.5, 0 ); 
	  g_signal_connect (G_OBJECT (param_button[i].adj), "value_changed", G_CALLBACK (update_param), &param_set->parameter[i] ); 
	  gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( param_button[i].button ), GTK_UPDATE_IF_VALID ); 
	  param_button[i].label = gtk_label_new( s ); 
	  gtk_table_attach_defaults(GTK_TABLE(param_table),param_button[i].label,(i%3)*2, (i%3)*2+1, (i/3)+3, (i/3)+4 ); 
	  gtk_table_attach_defaults(GTK_TABLE(param_table),param_button[i].button,(i%3)*2+1, (i%3)*2+2, (i/3)+3, (i/3)+4 ); 
	  g_signal_connect( G_OBJECT( param_button[i].button), "button_press_event", G_CALLBACK( param_spin_pressed ), (gpointer) i );  
	  update_param( GTK_ADJUSTMENT( param_button[i].adj ), &param_set->parameter[i]  ); 
	  gtk_widget_show( param_button[i].button ); 
	  gtk_widget_show( param_button[i].label );
	  break; 
	  
        case 'F': 
	  strcpy( s, "*" ); 
        case 'f': 
	  strncat( s, param_set->parameter[i].name,PARAM_NAME_LEN-1 ); 
	  strcat( s, "("); 
	  strncat( s, &param_set->parameter[i].unit_c, 1 ); 
	  strcat( s, ")" ); 
	  param_button[i].adj = (GtkAdjustment *)  gtk_adjustment_new( param_set->parameter[i].f_val, param_set->parameter[i].f_min, param_set->parameter[i].f_max, 
						    param_set->parameter[i].f_step, param_set->parameter[i].f_page, 0 ); 
	  param_button[i].button = gtk_spin_button_new( GTK_ADJUSTMENT( param_button[i].adj ), 0.5, param_set->parameter[i].f_digits ); 
	  g_signal_connect (G_OBJECT (param_button[i].adj), "value_changed", G_CALLBACK (update_param), &param_set->parameter[i] ); 
	  gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( param_button[i].button ), GTK_UPDATE_IF_VALID ); 
	  param_button[i].label = gtk_label_new( s ); 
	  gtk_table_attach_defaults(GTK_TABLE(param_table),param_button[i].label,(i%3)*2, (i%3)*2+1, (i/3)+3, (i/3)+4 ); 
	  gtk_table_attach_defaults(GTK_TABLE(param_table),param_button[i].button,(i%3)*2+1, (i%3)*2+2, (i/3)+3, (i/3)+4 ); 
	  g_signal_connect( G_OBJECT( param_button[i].button), "button_press_event", G_CALLBACK( param_spin_pressed ), (gpointer) i );  
	  update_param( GTK_ADJUSTMENT( param_button[i].adj ), &param_set->parameter[i]  ); 
	  gtk_widget_show( param_button[i].button ); 
	  gtk_widget_show( param_button[i].label ); 
	  break; 

        case 't': 
	  strncat( s, param_set->parameter[i].name,PARAM_NAME_LEN-1 ); 
	  param_button[i].ent = gtk_entry_new(); 
	  gtk_entry_set_width_chars(GTK_ENTRY(param_button[i].ent),11);
	  gtk_entry_set_max_length(GTK_ENTRY(param_button[i].ent),PARAM_T_VAL_LEN);
      
	  gtk_entry_set_text( GTK_ENTRY( param_button[i].ent ), param_set->parameter[i].t_val ); 

	  g_signal_connect(G_OBJECT (param_button[i].ent), "activate", G_CALLBACK (update_t_param_wrap), &param_set->parameter[i] ); 
	  g_signal_connect(G_OBJECT (param_button[i].ent), "focus_out_event", G_CALLBACK (update_t_param), &param_set->parameter[i] ); 

	  param_button[i].label = gtk_label_new( s ); 
	  gtk_table_attach_defaults(GTK_TABLE(param_table),param_button[i].label,(i%3)*2, (i%3)*2+1, (i/3)+3, (i/3)+4 ); 
	  gtk_table_attach_defaults(GTK_TABLE(param_table),param_button[i].ent,(i%3)*2+1, (i%3)*2+2, (i/3)+3, (i/3)+4 ); 
	  g_signal_connect( G_OBJECT( param_button[i].ent), "button_press_event", G_CALLBACK( param_spin_pressed ), (gpointer) i);  
	  update_t_param( GTK_ENTRY( param_button[i].ent ),NULL,&param_set->parameter[i]); 
	  gtk_widget_show( param_button[i].ent ); 
	  gtk_widget_show( param_button[i].label ); 
	  break; 
	  
        default: 
	  fprintf(stderr, "invalid type : %c can't build button %ld\n", param_set->parameter[i].type, i ); 
	  break; 
        } 
    } 

    num_buttons = param_set->num_parameters; 

    /* this feels like a hack - but it makes the scrolled window get
       sized correctly */
    gtk_scrolled_window_set_policy ( GTK_SCROLLED_WINDOW(param_scrolled_window),
				     GTK_POLICY_NEVER, GTK_POLICY_NEVER);
    gtk_scrolled_window_set_policy ( GTK_SCROLLED_WINDOW(param_scrolled_window),
				     GTK_POLICY_NEVER, GTK_POLICY_AUTOMATIC);

    //    fprintf(stderr,"returning\n");
  }


  void resize_popup(GtkAdjustment *adj, gpointer data  ) 

  { 
    int new_size, old_size; 
    GtkWidget **new_hbox; 
    GtkWidget **new_spb; 
    GtkAdjustment **new_adj; 
    GtkWidget *label; 
    int i; 
    char s[UTIL_LEN]; 

    new_size = (int) (gtk_adjustment_get_value(adj)); 
    old_size = popup_data.size; 
    //    fprintf(stderr, "resizing popup from %d to %d\n", old_size, new_size ); 

    if( new_size == old_size ) 
      return; 

    new_hbox = g_malloc( sizeof( GtkWidget* ) * new_size ); 
    new_adj = g_malloc( sizeof( GtkWidget* ) * new_size ); 
    new_spb = g_malloc( sizeof( GtkWidget* ) * new_size ); 
    //    fprintf(stderr,"resize popup: malloc\n");

    if( new_size > old_size ) { 
      //first, copy the existing boxes 

      for( i=0; i<old_size; i++ ) { 
        new_hbox[i] = popup_data.spin_hbox[i]; 
        new_adj[i] = popup_data.adj[i]; 
        new_spb[i] = popup_data.sp_button[i]; 
      } 

      //now create the additional ones 

      for( i=old_size; i<new_size; i++ ) { 
        new_hbox[i] = gtk_hbox_new_wrap( FALSE, 0 ); 
        snprintf( s,UTIL_LEN, "%d: ", i+1 ); 
        label = gtk_label_new( s ); 
        gtk_box_pack_start( GTK_BOX( new_hbox[i] ), label, TRUE, TRUE, 0 ); //was false false
        gtk_widget_show( label ); 

        switch( popup_data.param->type ) 
  	{ 
  	case 'i': 
  	case 'I': 
  	  new_adj[i] = (GtkAdjustment *) gtk_adjustment_new(  popup_data.param->i_val, popup_data.param->i_min,  popup_data.param->i_max, popup_data.param->i_step, popup_data.param->i_page, 0 ); 
  	  new_spb[i] = gtk_spin_button_new( GTK_ADJUSTMENT( new_adj[i] ),  0.5, 0 ); 
  	  break; 
  	case 'f': 
  	case 'F':  
  	  new_adj[i] = (GtkAdjustment *) gtk_adjustment_new(  popup_data.param->f_val, popup_data.param->f_min, popup_data.param->f_max, popup_data.param->f_step, popup_data.param->f_page, 0 ); 
  	  new_spb[i] = gtk_spin_button_new( GTK_ADJUSTMENT( new_adj[i] ),  0.5, popup_data.param->f_digits ); 
  	  break; 
  	default: 
  	  fprintf(stderr, "invalid parameter type in resize popup\n" ); 
  	  break; 
  	} 
        gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( new_spb[i] ), GTK_UPDATE_IF_VALID ); 
        gtk_widget_show( new_spb[i] ); 
        gtk_box_pack_start( GTK_BOX( new_hbox[i] ), new_spb[i], TRUE, TRUE, 0 ); 
        gtk_widget_show( new_hbox[i] ); 
        gtk_box_pack_start( GTK_BOX( popup_data.button_vbox ), new_hbox[i], FALSE, FALSE, 0 ); 
      } 

    } 
    else if( new_size < old_size ) { 
      for( i=0; i<new_size; i++ ) { 
        new_hbox[i] = popup_data.spin_hbox[i]; 
        new_adj[i] = popup_data.adj[i]; 
        new_spb[i] = popup_data.sp_button[i]; 
      } 

      for( i=new_size; i<old_size; i++ ) { 
        gtk_widget_hide( popup_data.spin_hbox[i] ); 
	//	printf("count is: %i\n",G_OBJECT(popup_data.adj[i])->ref_count);
        gtk_widget_destroy( popup_data.sp_button[i] ); 
        gtk_widget_destroy( popup_data.spin_hbox[i] ); 
	// the adjustment gets destroyed with the spin button...
	//	gtk_widget_destroy( GTK_WIDGET(popup_data.adj[i] )); 
	//	printf("doing popup_data destroy\n");
	//	printf("done popup_data destroy\n");
	//	g_object_unref(G_OBJECT(popup_data.adj[i]));
      } 
    } 

    g_free( popup_data.adj ); 
    g_free( popup_data.spin_hbox ); 
    g_free( popup_data.sp_button ); 
  
    popup_data.adj = new_adj; 
    popup_data.spin_hbox = new_hbox; 
    popup_data.sp_button = new_spb; 

    popup_data.size = new_size; 
    return; 
  } 


  void create_2d_popup() 
  { 
    GtkWidget* hbox; 
    GtkWidget* button; 
    GtkWidget* label; 
    GtkWidget* vbox;
    GtkWidget* hbox1;
    GtkWidget* separator;
    
    GSList *group;



    popup_data.win = gtk_window_new( GTK_WINDOW_TOPLEVEL ); 
    gtk_window_set_resizable(GTK_WINDOW(popup_data.win),0); //user can't, wm can

    popup_data.frame = gtk_frame_new( "No label yet" ); 
    gtk_container_add( GTK_CONTAINER( popup_data.win ), popup_data.frame ); 
    gtk_container_set_border_width(GTK_CONTAINER (popup_data.frame),5); 
    gtk_widget_show( popup_data.frame ); 

    hbox = gtk_hbox_new_wrap( FALSE, 5 ); 
    gtk_container_set_border_width(GTK_CONTAINER (hbox),5); 
    gtk_container_add( GTK_CONTAINER( popup_data.frame ), hbox ); 
    gtk_widget_show( hbox ); 


    popup_data.button_vbox = gtk_vbox_new_wrap( FALSE, 5 ); 
    //    gtk_widget_set_size_request( popup_data.button_vbox, 200, 0 ); 
    gtk_box_pack_start( GTK_BOX( hbox ), popup_data.button_vbox, FALSE,FALSE, 0 ); 
    gtk_widget_show( popup_data.button_vbox ); 
    
    vbox=gtk_vbox_new_wrap(FALSE,0);
    gtk_widget_show(vbox);
    gtk_box_pack_start(GTK_BOX(hbox),vbox,FALSE,FALSE,0);


    // do the unarray, number of boxes line
    hbox1=gtk_hbox_new_wrap(TRUE,0);
    gtk_widget_show(hbox1);
    
    button = gtk_button_new_with_label( "Unarray" ); 
    g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( unarray_pressed ), NULL ); 
    gtk_box_pack_start(GTK_BOX(hbox1),button,TRUE,TRUE,0);
    gtk_widget_show( button ); 

    popup_data.num_adj = (GtkAdjustment *) gtk_adjustment_new( 0,1,1024,1,5,0 ); 
    button = gtk_spin_button_new( GTK_ADJUSTMENT( popup_data.num_adj ),  0.5, 0 ); 
    g_signal_connect( G_OBJECT( popup_data.num_adj ), "value_changed", G_CALLBACK( resize_popup ), NULL ); 
    gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON( button ), GTK_UPDATE_IF_VALID ); 
    gtk_box_pack_start(GTK_BOX(hbox1),button,TRUE,TRUE,0);
    gtk_widget_show( button ); 
    
    gtk_box_pack_start(GTK_BOX(vbox),hbox1,FALSE,FALSE,2);
#if GTK_MAJOR_VERSION == 2
    separator=gtk_hseparator_new();
#else
    separator=gtk_separator_new(GTK_ORIENTATION_HORIZONTAL);
#endif
    gtk_widget_show(separator);
    gtk_box_pack_start(GTK_BOX(vbox),separator,FALSE,FALSE,2);


    // do the cancel - ok line
    

    hbox1=gtk_hbox_new_wrap(TRUE,0);
    gtk_widget_show(hbox1);
    button = gtk_button_new_with_label( "Cancel" ); 
    g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( array_cancel_pressed ), NULL ); 
    gtk_box_pack_start(GTK_BOX(hbox1),button,TRUE,TRUE,0);
    gtk_widget_show( button ); 


    button = gtk_button_new_with_label( "OK" ); 
    g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( ok_pressed ), NULL ); 
    gtk_box_pack_start(GTK_BOX(hbox1),button,TRUE,TRUE,0);
    gtk_widget_show( button ); 

    gtk_box_pack_start(GTK_BOX(vbox),hbox1,FALSE,FALSE,2);

#if GTK_MAJOR_VERSION == 2
    separator=gtk_hseparator_new();
#else
    separator=gtk_separator_new(GTK_ORIENTATION_HORIZONTAL);
#endif
    gtk_widget_show(separator);
    gtk_box_pack_start(GTK_BOX(vbox),separator,FALSE,FALSE,2);

    // now the start line
    hbox1=gtk_hbox_new_wrap(TRUE,0);
    gtk_widget_show(hbox1);

    label = gtk_label_new( "Start" ); 
    gtk_box_pack_start(GTK_BOX(hbox1),label,TRUE,TRUE,0);
    gtk_widget_show( label ); 

    popup_data.start_adj = (GtkAdjustment *)  gtk_adjustment_new(0,0,0,0,0,0  ); 
    popup_data.start_button = gtk_spin_button_new( GTK_ADJUSTMENT( popup_data.start_adj ),  0.5, 2 ); 
    gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON(  popup_data.start_button ), GTK_UPDATE_IF_VALID ); 
    gtk_box_pack_start(GTK_BOX(hbox1),popup_data.start_button,TRUE,TRUE,0);
    gtk_widget_show( popup_data.start_button ); 

    gtk_box_pack_start(GTK_BOX(vbox),hbox1,FALSE,FALSE,2);

    // then the end line
    hbox1=gtk_hbox_new_wrap(TRUE,0);
    gtk_widget_show(hbox1);

    popup_data.end_radio = gtk_radio_button_new_with_label( NULL, "End" );
    gtk_box_pack_start(GTK_BOX(hbox1),popup_data.end_radio,FALSE,FALSE,1);
    gtk_widget_show (popup_data.end_radio);


    popup_data.end_adj = (GtkAdjustment *) gtk_adjustment_new(0,0,0,0,0,0  ); 
    popup_data.end_button = gtk_spin_button_new( GTK_ADJUSTMENT( popup_data.end_adj ),  0.5, 2 ); 
    gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON(  popup_data.end_button ), GTK_UPDATE_IF_VALID ); 
    gtk_box_pack_start(GTK_BOX(hbox1),popup_data.end_button,TRUE,TRUE,0);
    gtk_widget_show( popup_data.end_button );
    gtk_box_pack_start(GTK_BOX(vbox),hbox1,FALSE,FALSE,2);
    /////
    // then the increment line
    hbox1=gtk_hbox_new_wrap(TRUE,0);
    gtk_widget_show(hbox1);

    group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (popup_data.end_radio) );
    button = gtk_radio_button_new_with_label( group, "Increment" );
    gtk_box_pack_start(GTK_BOX(hbox1),button,FALSE,FALSE,1);

    gtk_widget_show (button);

    popup_data.inc_adj = (GtkAdjustment *) gtk_adjustment_new(0,0,0,0,0,0  ); 
    popup_data.inc_button = gtk_spin_button_new( GTK_ADJUSTMENT( popup_data.inc_adj ),  0.5, 2 ); 
    gtk_spin_button_set_update_policy( GTK_SPIN_BUTTON(  popup_data.inc_button ), GTK_UPDATE_IF_VALID ); 
    gtk_box_pack_start(GTK_BOX(hbox1),popup_data.inc_button,TRUE,TRUE,0);
    gtk_widget_show( popup_data.inc_button );
    gtk_box_pack_start(GTK_BOX(vbox),hbox1,FALSE,FALSE,2);
    /////
    // then the log/linear line
        // then the increment line

    
    hbox1=gtk_hbox_new_wrap(TRUE,0);
    gtk_widget_show(hbox1);

    popup_data.linear_radio = gtk_radio_button_new_with_label( NULL, "Linear" );
    gtk_box_pack_start(GTK_BOX(hbox1),popup_data.linear_radio,FALSE,FALSE,1);
    gtk_widget_show (popup_data.linear_radio);

    group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (popup_data.linear_radio) );
    button = gtk_radio_button_new_with_label( group, "Log" );
    gtk_box_pack_start(GTK_BOX(hbox1),button,FALSE,FALSE,1);

    gtk_widget_show (button);
    gtk_box_pack_start(GTK_BOX(vbox),hbox1,FALSE,FALSE,2);
    

    // then the update line
    hbox1=gtk_hbox_new_wrap(TRUE,0);
    gtk_widget_show(hbox1);
    gtk_box_pack_start(GTK_BOX(vbox),hbox1,FALSE,FALSE,2);

    //    label = gtk_label_new("");
    //    gtk_widget_show(label);
    //    gtk_box_pack_start(GTK_BOX(hbox1),label,TRUE,TRUE,0);

    button = gtk_button_new_with_label( "Scramble Points" ); 
    g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( scramble_points ), NULL ); 
    gtk_box_pack_start(GTK_BOX(hbox1),button,TRUE,TRUE,0);
    gtk_widget_show( button ); 

    button = gtk_button_new_with_label( "Generate Points" ); 
    g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( apply_inc_pressed ), NULL ); 
    gtk_box_pack_start(GTK_BOX(hbox1),button,TRUE,TRUE,0);
    gtk_widget_show( button ); 



    
    label=gtk_label_new("");
    gtk_box_pack_start(GTK_BOX(vbox),label,TRUE,TRUE,0);


    g_signal_connect( G_OBJECT( popup_data.win ), "delete_event", G_CALLBACK( array_cancel_pressed ), NULL ); 
    g_signal_connect( G_OBJECT( popup_data.win ), "destroy", G_CALLBACK( destroy_popup ), NULL ); 

    popup_data.size = 0; 
    return; 
  } 

  void show_2d_popup( GtkWidget *widget, int* button ) 

  { 
    int i; 


    if( *button < 0 ) 
      return; 

    if (allowed_to_change(current) == FALSE){ 
      if (no_update_open == 0)
	popup_no_update("Can't open array window for acquiring or queued window"); 
     return; 
    } 
 
    popup_data.bnum = current; 

    popup_data.param = &current_param_set->parameter[ *button ]; 
    popup_data.button_number = *button; 

    // fprintf(stderr, "showing popup, button is %d, type is %c\n", *button, popup_data.param->type ); 

    gtk_frame_set_label( GTK_FRAME( popup_data.frame ), current_param_set->parameter[ *button ].name ); 

    switch( popup_data.param->type ) 
      { 
      case 't': 
        return; 
        break; 

      case 'i': 
        gtk_adjustment_set_value( GTK_ADJUSTMENT( popup_data.num_adj ), current_param_set->num_acqs_2d ); 
	if (popup_data.size == 0){ // starts at zero when popup is first built
	  resize_popup(GTK_ADJUSTMENT(popup_data.num_adj),NULL);
	}
        for( i=0; i<popup_data.size; i++ ) { 
	  gtk_adjustment_set_upper(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->i_max); 
	  gtk_adjustment_set_lower(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->i_min); 
	  gtk_adjustment_set_step_increment(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->i_step); 
	  gtk_adjustment_set_page_increment(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->i_page); 
	  gtk_adjustment_set_value(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->i_val); 
	  //	  gtk_adjustment_changed( GTK_ADJUSTMENT( popup_data.adj[i] ) ); 
	  //	  gtk_adjustment_value_changed( GTK_ADJUSTMENT( popup_data.adj[i] ) ); 
	  gtk_spin_button_set_digits( GTK_SPIN_BUTTON( popup_data.sp_button[i] ), 0 ); 
        } 
        break; 

      case 'f': 
        gtk_adjustment_set_value( GTK_ADJUSTMENT( popup_data.num_adj ), current_param_set->num_acqs_2d ); 
	if (popup_data.size == 0 ){// starts at zero when popup is first built
	  resize_popup(GTK_ADJUSTMENT(popup_data.num_adj),NULL);
	}
        for( i=0; i<popup_data.size; i++ ) { 
	  gtk_adjustment_set_upper(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->f_max); 
	  gtk_adjustment_set_lower(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->f_min); 
	  gtk_adjustment_set_step_increment(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->f_step); 
	  gtk_adjustment_set_page_increment(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->f_page); 
	  gtk_adjustment_set_value(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->f_val); 
	  //	  gtk_adjustment_changed( GTK_ADJUSTMENT( popup_data.adj[i] ) ); 
	  //	  gtk_adjustment_value_changed( GTK_ADJUSTMENT( popup_data.adj[i] ) ); 
	  gtk_spin_button_set_digits( GTK_SPIN_BUTTON( popup_data.sp_button[i] ), popup_data.param->f_digits ); 
        } 
        break; 

      case 'I': 
        gtk_adjustment_set_value( GTK_ADJUSTMENT( popup_data.num_adj ), popup_data.param->size ); 
        for( i=0; i<popup_data.size; i++ ) { 
	  gtk_adjustment_set_upper(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->i_max); 
	  gtk_adjustment_set_lower(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->i_min); 
	  gtk_adjustment_set_step_increment(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->i_step); 
	  gtk_adjustment_set_page_increment(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->i_page); 
	  gtk_adjustment_set_value(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->i_val_2d[i]); 
	  //	  gtk_adjustment_changed( GTK_ADJUSTMENT( popup_data.adj[i] ) ); 
	  //	  gtk_adjustment_value_changed( GTK_ADJUSTMENT( popup_data.adj[i] ) ); 
	  gtk_spin_button_set_digits( GTK_SPIN_BUTTON( popup_data.sp_button[i] ), 0 ); 
        } 
        break; 

      case 'F': 
        gtk_adjustment_set_value( GTK_ADJUSTMENT( popup_data.num_adj ), popup_data.param->size ); 
        for( i=0; i<popup_data.size; i++ ) { 
	  gtk_adjustment_set_upper(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->f_max);
	  gtk_adjustment_set_lower(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->f_min);
	  gtk_adjustment_set_step_increment(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->f_step);
	  gtk_adjustment_set_page_increment(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->f_page); 
	  gtk_adjustment_set_value(GTK_ADJUSTMENT( popup_data.adj[i] ), popup_data.param->f_val_2d[i]);
	  //	  gtk_adjustment_changed( GTK_ADJUSTMENT( popup_data.adj[i] ) ); 
	  //	  gtk_adjustment_value_changed( GTK_ADJUSTMENT( popup_data.adj[i] ) ); 
	  gtk_spin_button_set_digits( GTK_SPIN_BUTTON( popup_data.sp_button[i] ), popup_data.param->f_digits ); 
        } 
        break; 

      default: 
        fprintf(stderr, "invalid parameter type in show_2d_popup\n" ); 
        break; 
      } 

    //Now we have to set the start and increment adjustments 

    switch( popup_data.param->type ) 
      { 
      case 'i': 
      case 'I': 
        gtk_spin_button_set_digits( GTK_SPIN_BUTTON( popup_data.start_button ), 0); 
	gtk_adjustment_set_upper(GTK_ADJUSTMENT( popup_data.start_adj ), popup_data.param->i_max); 
	gtk_adjustment_set_lower(GTK_ADJUSTMENT( popup_data.start_adj ), popup_data.param->i_min); 
	gtk_adjustment_set_step_increment(GTK_ADJUSTMENT( popup_data.start_adj ), popup_data.param->i_step); 
	gtk_adjustment_set_page_increment(GTK_ADJUSTMENT( popup_data.start_adj ), popup_data.param->i_page); 
	//        gtk_adjustment_set_value(GTK_ADJUSTMENT( popup_data.start_adj ), popup_data.param->i_val); 
        gtk_adjustment_set_value(GTK_ADJUSTMENT( popup_data.start_adj ), gtk_adjustment_get_value(GTK_ADJUSTMENT(popup_data.adj[0]))); 
	//	gtk_adjustment_changed( GTK_ADJUSTMENT( popup_data.start_adj ) ); 
	//	gtk_adjustment_value_changed( GTK_ADJUSTMENT( popup_data.start_adj ) ); 

	gtk_spin_button_set_digits( GTK_SPIN_BUTTON( popup_data.end_button ), 0); 
	gtk_adjustment_set_upper(GTK_ADJUSTMENT( popup_data.end_adj ), popup_data.param->i_max); 
	gtk_adjustment_set_lower(GTK_ADJUSTMENT( popup_data.end_adj ), popup_data.param->i_min); 
	gtk_adjustment_set_step_increment(GTK_ADJUSTMENT( popup_data.end_adj ), popup_data.param->i_step); 
	gtk_adjustment_set_page_increment(GTK_ADJUSTMENT( popup_data.end_adj ), popup_data.param->i_page); 
        gtk_adjustment_set_value(GTK_ADJUSTMENT( popup_data.end_adj ), gtk_adjustment_get_value(GTK_ADJUSTMENT(popup_data.adj[popup_data.size-1]))); 
	//	gtk_adjustment_changed( GTK_ADJUSTMENT( popup_data.end_adj ) ); 
	//	gtk_adjustment_value_changed( GTK_ADJUSTMENT( popup_data.end_adj ) ); 

	gtk_spin_button_set_digits( GTK_SPIN_BUTTON( popup_data.inc_button ), 3 ); 
        gtk_adjustment_set_upper(GTK_ADJUSTMENT( popup_data.inc_adj ), abs( popup_data.param->i_max ) > abs( popup_data.param->i_min ) ? abs( popup_data.param->i_max ) : abs( popup_data.param->i_min )); 
        gtk_adjustment_set_lower(GTK_ADJUSTMENT( popup_data.inc_adj ), -1 * gtk_adjustment_get_upper(GTK_ADJUSTMENT( popup_data.inc_adj ))); 
        gtk_adjustment_set_step_increment(GTK_ADJUSTMENT( popup_data.inc_adj ), popup_data.param->i_step); 
        gtk_adjustment_set_page_increment(GTK_ADJUSTMENT( popup_data.inc_adj ), popup_data.param->i_page); 
        gtk_adjustment_set_value(GTK_ADJUSTMENT( popup_data.inc_adj ), popup_data.param->i_val); 
	//        gtk_adjustment_changed( GTK_ADJUSTMENT( popup_data.inc_adj ) ); 
	//        gtk_adjustment_value_changed( GTK_ADJUSTMENT( popup_data.inc_adj ) ); 

break;
      case 'f': 
      case 'F': 
        gtk_spin_button_set_digits( GTK_SPIN_BUTTON( popup_data.start_button ), popup_data.param->f_digits );
	gtk_adjustment_set_upper(GTK_ADJUSTMENT( popup_data.start_adj ), popup_data.param->f_max); 
	gtk_adjustment_set_lower(GTK_ADJUSTMENT( popup_data.start_adj ), popup_data.param->f_min); 
	gtk_adjustment_set_step_increment(GTK_ADJUSTMENT( popup_data.start_adj ), popup_data.param->f_step); 
	gtk_adjustment_set_page_increment(GTK_ADJUSTMENT( popup_data.start_adj ), popup_data.param->f_page); 
	//        gtk_adjustment_set_value(GTK_ADJUSTMENT( popup_data.start_adj ), popup_data.param->f_val); 
        gtk_adjustment_set_value(GTK_ADJUSTMENT( popup_data.start_adj ), gtk_adjustment_get_value(GTK_ADJUSTMENT(popup_data.adj[0]))); 
	//        gtk_adjustment_changed( GTK_ADJUSTMENT( popup_data.start_adj ) ); 
	//        gtk_adjustment_value_changed( GTK_ADJUSTMENT( popup_data.start_adj ) ); 
      
        gtk_spin_button_set_digits( GTK_SPIN_BUTTON( popup_data.end_button ), popup_data.param->f_digits );
	gtk_adjustment_set_upper(GTK_ADJUSTMENT( popup_data.end_adj ), popup_data.param->f_max); 
	gtk_adjustment_set_lower(GTK_ADJUSTMENT( popup_data.end_adj ), popup_data.param->f_min); 
	gtk_adjustment_set_step_increment(GTK_ADJUSTMENT( popup_data.end_adj ), popup_data.param->f_step); 
	gtk_adjustment_set_page_increment(GTK_ADJUSTMENT( popup_data.end_adj ), popup_data.param->f_page); 
        gtk_adjustment_set_value(GTK_ADJUSTMENT( popup_data.end_adj ), gtk_adjustment_get_value(GTK_ADJUSTMENT(popup_data.adj[popup_data.size-1]))); 
	//        gtk_adjustment_changed( GTK_ADJUSTMENT( popup_data.end_adj ) ); 
	//        gtk_adjustment_value_changed( GTK_ADJUSTMENT( popup_data.end_adj ) ); 
      
	gtk_spin_button_set_digits( GTK_SPIN_BUTTON( popup_data.inc_button ), popup_data.param->f_digits+3 ); 
        gtk_adjustment_set_upper(GTK_ADJUSTMENT( popup_data.inc_adj ), fabs( popup_data.param->f_max ) > fabs( popup_data.param->f_min ) ? fabs( popup_data.param->f_max ) : fabs( popup_data.param->f_min )); 
	gtk_adjustment_set_lower(GTK_ADJUSTMENT( popup_data.inc_adj ), -1 * gtk_adjustment_get_upper(GTK_ADJUSTMENT( popup_data.inc_adj ))); 
        gtk_adjustment_set_step_increment(GTK_ADJUSTMENT( popup_data.inc_adj ), popup_data.param->f_step); 
        gtk_adjustment_set_page_increment(GTK_ADJUSTMENT( popup_data.inc_adj ), popup_data.param->f_page); 
        gtk_adjustment_set_value(GTK_ADJUSTMENT( popup_data.inc_adj ),  popup_data.param->f_val); 
	//        gtk_adjustment_changed( GTK_ADJUSTMENT( popup_data.inc_adj ) ); 
	//        gtk_adjustment_value_changed( GTK_ADJUSTMENT( popup_data.inc_adj ) ); 
      } 

    array_popup_showing = 1; 

    gtk_window_set_transient_for(GTK_WINDOW(popup_data.win),GTK_WINDOW(buffp[current]->win.window));
    //    gtk_window_set_position(GTK_WINDOW(popup_data.win),GTK_WIN_POS_CENTER_ON_PARENT);
    // this made it go offscreen if it was tall...



    gtk_widget_show( popup_data.win ); 
    //    gdk_window_raise(popup_data.win->window);
    gdk_window_raise(gtk_widget_get_window(popup_data.win));
    return; 
  } 

  void ok_pressed( GtkWidget* widget, gpointer data ) 
  { 
    int i; 
    char s[UTIL_LEN] = "*"; 

    if (popup_data.size == 1){
      unarray_pressed (widget , data); 
      return;
    }


    switch( popup_data.param->type ) 
      { 
      case 'i': 
        popup_data.param->type = 'I'; 
	popup_data.param->i_val_2d = g_malloc( sizeof( gint ) * popup_data.size );
	//	fprintf(stderr,"ok_pressed: malloc\n");

        if (popup_data.bnum == current){ 
	  gtk_adjustment_set_value( GTK_ADJUSTMENT( acqs_2d_adj ), popup_data.size ); 
	  strncat( s, popup_data.param->name,PARAM_NAME_LEN-strlen(s)); 
	  gtk_label_set_text( GTK_LABEL( param_button[ popup_data.button_number ].label ), s ); 
        } 
	else buffp[popup_data.bnum]->param_set.num_acqs_2d = popup_data.size;
        break; 

      case 'f': 
        popup_data.param->type = 'F'; 
        popup_data.param->f_val_2d = g_malloc( sizeof(double) * popup_data.size ); 
	//	fprintf(stderr,"ok_pressed: malloc\n");

        if (popup_data.bnum == current){ 
	  gtk_adjustment_set_value( GTK_ADJUSTMENT( acqs_2d_adj ), popup_data.size ); 
	  strncat( s, popup_data.param->name,PARAM_NAME_LEN-strlen(s)); 
	  strcat( s, "("); 
	  strncat( s, &popup_data.param->unit_c, 1); 
	  strcat( s, ")");
	  gtk_label_set_text( GTK_LABEL( param_button[ popup_data.button_number ].label ), s ); 
        } 
	else buffp[popup_data.bnum]->param_set.num_acqs_2d = popup_data.size;
        break; 

      case 'I': 
        if( popup_data.size != popup_data.param->size ) { 
  	g_free( popup_data.param->i_val_2d ); 
  	popup_data.param->i_val_2d = g_malloc( sizeof( gint ) * popup_data.size ); 
	//	fprintf(stderr,"ok_pressed: malloc\n");

	if (popup_data.bnum == current) 
	  gtk_adjustment_set_value( GTK_ADJUSTMENT( acqs_2d_adj ), popup_data.size ); 
	else buffp[popup_data.bnum]->param_set.num_acqs_2d = popup_data.size;
        } 
        break; 

      case 'F': 
        if( popup_data.size != popup_data.param->size ) { 
  	g_free( popup_data.param->f_val_2d ); 
  	popup_data.param->f_val_2d = g_malloc( sizeof( double ) * popup_data.size ); 
	//	fprintf(stderr,"ok_pressed: malloc\n");

  	if (popup_data.bnum == current) 
  	  gtk_adjustment_set_value( GTK_ADJUSTMENT( acqs_2d_adj ), popup_data.size ); 
	else buffp[popup_data.bnum]->param_set.num_acqs_2d = popup_data.size;
        } 
        break; 

      default: 
        fprintf(stderr, "Xnmr: apply_pressed: invalid parameter type\n" ); 
        break; 
      } 

    switch( popup_data.param->type ) 
      { 
      case 'I': 
  	for( i=0; i<popup_data.size; i++ ) 
  	  popup_data.param->i_val_2d[i] = (int) gtk_adjustment_get_value(GTK_ADJUSTMENT( popup_data.adj[i] )); 
  	break; 
      case 'F': 
  	for( i=0; i<popup_data.size; i++ ) 
  	  popup_data.param->f_val_2d[i] = gtk_adjustment_get_value(GTK_ADJUSTMENT( popup_data.adj[i] ) ); 
  	break; 
      default: 
  	fprintf(stderr, "Xnmr: apply_pressed: invalid parameter type\n" ); 
  	break; 
       
      } 
    
    
    popup_data.param->size = popup_data.size; 
    
    array_popup_showing =0; 
    gtk_widget_hide( popup_data.win ); 

    return; 
  } 


  void unarray_pressed( GtkWidget* widget, gpointer data ) 
  { 
    int max,i;
    char s[PARAM_NAME_LEN];
    switch( popup_data.param->type ) 
    { 
    case 'I': 
      popup_data.param->type = 'i'; 
      g_free( popup_data.param->i_val_2d ); 
	popup_data.param->i_val_2d = NULL;   // is this necessary? - I think so


      if (popup_data.bnum == current) 
	gtk_label_set_text( GTK_LABEL( param_button[ popup_data.button_number ].label ), popup_data.param->name ); 

      break; 
    case 'F': 
      popup_data.param->type = 'f'; 
      strncpy( s, popup_data.param->name,PARAM_NAME_LEN); 
      strcat( s, "("); 
      strncat( s, &popup_data.param->unit_c, 1); 
      strcat( s, ")"); 

      if (popup_data.bnum == current) 
	gtk_label_set_text( GTK_LABEL( param_button[ popup_data.button_number ].label ), s ); 
      // should this be  only if current?

      //      fprintf(stderr,"put name: %s in button label\n",s);
      g_free( popup_data.param->f_val_2d ); 
	popup_data.param->f_val_2d = NULL;   // is this necessary? - I think so

      break; 
    case 'i': 
    case 'f': 
      break; 
    default: 
      fprintf(stderr, "invalid parameter type in unarray_pressed\n" ); 
      break; 
    } 
    popup_data.param->size = 0;  


    // some of this code duplicated in set_sf1_press_event

    // now look through params to find biggest size to put in panel and acqs_2d
    max=1;
    for (i=0;i<buffp[popup_data.bnum]->param_set.num_parameters;i++){
      if (buffp[popup_data.bnum]->param_set.parameter[i].size > max) 
	max = buffp[popup_data.bnum]->param_set.parameter[i].size;
    }
    //    fprintf(stderr,"unarray: max size was %i\n",max);
    if (popup_data.bnum == current) 
      gtk_adjustment_set_value( GTK_ADJUSTMENT( acqs_2d_adj ), max ); 
    else buffp[popup_data.bnum]->param_set.num_acqs_2d = max;

    if (current == popup_data.bnum)
      update_param( GTK_ADJUSTMENT(param_button[popup_data.button_number].adj),
		    popup_data.param ) ; 
    //if it isn't current, we'll see what it is later anyway.  Ambiguous
    // things happened without this.

    array_cancel_pressed( NULL, NULL ); 
    return; 
  } 

void array_cancel_pressed( GtkWidget* widget, gpointer data )  // missing middle argument?
  { 
    array_popup_showing =0; 
    gtk_widget_hide( popup_data.win ); 
    return; 
  } 


void destroy_popup( GtkWidget *widget, gpointer data )  
  { 
    fprintf(stderr, "destroying popup\n" ); 
    g_free( popup_data.adj ); 
    g_free( popup_data.spin_hbox ); 
    g_free( popup_data.sp_button ); 
    return; 
  } 



  void reload( GtkWidget* widget, gpointer data ) 


  { 
    // this gets called with NULL,NULL at the end of an acquisition.
    // and if we started up running.

    char path[ PATH_LENGTH ]; 
    dbuff *buff; 


    if (upload_buff == current && acq_in_progress != ACQ_STOPPED) {
      redraw = 1;
      if (acq_in_progress == ACQ_REPEATING_AND_PROCESSING){
	upload_and_draw_canvas_with_process(buffp[current]);
      }
      else{
	upload_and_draw_canvas(buffp[current]);
      }
      /*      buffp[current]->flags &= ~FT_FLAG;
      buffp[current]->phase0_app = 0;
      buffp[current]->phase1_app = 0;
      buffp[current]->phase20_app = 0;
      buffp[current]->phase21_app = 0;  done in the upload_data */
      return; // don't do anything if we're in the uploading, just do upload 
    }

    if( widget != NULL ) { // this means user pushed button
      if (am_i_queued(current)){
	popup_msg("Can't reload into queued buffer",TRUE);
	return;
      }
      buff = buffp[ current ];  
    }
    else 
      buff = buffp[ upload_buff ]; // here we call at end of acq
    if (buff == NULL){
      popup_msg("reload panic, buff = NULL",TRUE);
      return;
    }
    path_strcpy( path, buff->path_for_reload); 
    //    fprintf(stderr,"in reload, path is: %s\n",path);
    if (strcmp(path,"") == 0 ) return;
    do_load( buff, path,1);  //always read in the fid.

    return; 
  } 

void scramble_points( GtkWidget *widget, gpointer data){
  int i,i1,i2;
  double exval1,exval2;

  for(i=0;i<popup_data.size*2;i++){
    i1 = 1.*rand()/RAND_MAX*popup_data.size;
    if (i1 >= popup_data.size) i1 -= 1;
    i2 = 1.*rand()/RAND_MAX*popup_data.size;
    if (i2 >= popup_data.size) i2 -= 1;
    exval1 = gtk_adjustment_get_value(GTK_ADJUSTMENT(popup_data.adj[i1]));
    exval2 = gtk_adjustment_get_value(GTK_ADJUSTMENT(popup_data.adj[i2]));
    gtk_adjustment_set_value(GTK_ADJUSTMENT(popup_data.adj[i1]), exval2);
    gtk_adjustment_set_value(GTK_ADJUSTMENT(popup_data.adj[i2]), exval1);
  }

}

  void apply_inc_pressed( GtkWidget* widget, gpointer data ) 
  { 
    int i;
    char is_linear, use_end;
    double start, increment,end;
    is_linear = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(popup_data.linear_radio));
    if (is_linear) printf("Got linear\n");
    else printf("Got log\n");
    use_end = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(popup_data.end_radio));
    if (use_end) printf("Using end\n");
    else printf("using increment\n");

    start = gtk_adjustment_get_value(GTK_ADJUSTMENT( popup_data.start_adj ));
    increment = gtk_adjustment_get_value(GTK_ADJUSTMENT( popup_data.inc_adj ));
    end =  gtk_adjustment_get_value(GTK_ADJUSTMENT( popup_data.end_adj ));

    if (popup_data.size > 1 && use_end){
      if (is_linear)
	increment = (end-start)/(popup_data.size-1);
      else
	if (end != 0. && start != 0. && end/start > 0.)
	  increment = pow(end/start,1./(popup_data.size-1));
	else{
	  increment = 1;
	  printf("start is: %f, end is: %f, ratio is %f\n",start,end,end/start);
	  popup_msg("Invalid limits for log\n", TRUE);
	}
    }
    printf("using increment: %f\n",increment);

    if (is_linear)
      for( i=0; i<popup_data.size; i++ ) 
	gtk_adjustment_set_value( GTK_ADJUSTMENT( popup_data.adj[i] ),  
				  start + i*increment); 
    else
      for( i=0; i<popup_data.size; i++ ) 
	gtk_adjustment_set_value( GTK_ADJUSTMENT( popup_data.adj[i] ),  
				  start*pow(increment,i)); 

    return; 
  } 


  gint popup_no_update( char* msg ) 

  { 
    GtkWidget* dialog; 


    no_update_open = 1; 

    dialog=gtk_message_dialog_new(GTK_WINDOW(panwindow),GTK_DIALOG_DESTROY_WITH_PARENT,
				  GTK_MESSAGE_ERROR,GTK_BUTTONS_CLOSE,"%s",msg);
    gtk_widget_show(dialog);
    g_signal_connect_swapped(dialog,"response",G_CALLBACK(popup_no_update_ok),dialog);
  
    return FALSE; 
  }


gint popup_no_update_ok(GtkWidget *widget,gpointer *data)
{

  //  fprintf(stderr,"in popup_no_update_ok\n");

  if (no_update_open == 1)
    no_update_open = 0;
  gtk_widget_destroy(GTK_WIDGET(widget));

  return FALSE;

}
