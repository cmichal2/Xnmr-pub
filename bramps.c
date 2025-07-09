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
#include <gtk/gtk.h>


GtkWidget *powampbutH,*powampbutX;
GtkWidget *preampbutH,*preampbutX;
GtkWidget *gainH,*gainL;
GtkWidget *enableH,*enableX,*enableT;

static gboolean button_press(GtkWidget *widget,gpointer data){
  unsigned char  val = 0,val2;
  FILE *ofile;

  // go through and set up value to write:

  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gainH))) val += 1;
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(preampbutH))) val += 2;
  if (!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(enableT))) val += 4;
  if (!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(enableH))) val += 8;
  if (!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(enableX))) val += 16;
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(powampbutH))) val += 32;

  ofile = fopen("/dev/usb/lp0","wb");
  if(ofile == NULL){ 
    printf("failed to open /dev/usb/lp0\n");
    /*
    gtk_main_quit();
    return; */
  }

  //  printf("setting value to: %i\n",val);

  if (ofile != NULL){
    fwrite(&val,1,1,ofile);
    val2 = val+128;
    fwrite(&val2,1,1,ofile);
    fwrite(&val,1,1,ofile);
    
    fclose(ofile);
  }

  return 0;
}


static  void destroy ( GtkWidget *widget, 
			      gpointer data){

  gtk_main_quit();
}

int main( int argc, char *argv[])
{

  GtkWidget *window,*vbox,*label,*hbox;
  GtkWidget *sep;
  
  gtk_init(&argc,&argv);

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

  g_signal_connect(G_OBJECT(window),"destroy",G_CALLBACK(destroy),NULL);

  vbox = gtk_vbox_new(FALSE,0);
  gtk_container_add(GTK_CONTAINER(window),vbox);

  label=gtk_label_new("Pre-amp and Power Amp control");
  gtk_box_pack_start(GTK_BOX(vbox),label,FALSE,FALSE,0);
  

  sep = gtk_hseparator_new();
  gtk_box_pack_start(GTK_BOX(vbox),sep,FALSE,FALSE,0);

  // first pair of radio buttons for power amp freq range

  hbox=gtk_hbox_new(FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),hbox,FALSE,FALSE,0);

  label = gtk_label_new("Power amp frequency range:  ");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,0);

  powampbutH = gtk_radio_button_new_with_label(NULL,"H/F");
  gtk_box_pack_end(GTK_BOX(hbox),powampbutH,FALSE,FALSE,0);

  powampbutX = gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(powampbutH),"X");
  gtk_box_pack_end(GTK_BOX(hbox),powampbutX,FALSE,FALSE,0);
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(powampbutX),TRUE);
  
  g_signal_connect(G_OBJECT(powampbutH),"toggled",G_CALLBACK(button_press),NULL);

  sep = gtk_hseparator_new();
  gtk_box_pack_start(GTK_BOX(vbox),sep,FALSE,FALSE,0);

  // second pair of buttons, for preamp selection

  hbox=gtk_hbox_new(FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),hbox,FALSE,FALSE,0);

  label = gtk_label_new("Active pre-amp:");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,0);

  preampbutH = gtk_radio_button_new_with_label(NULL,"H/F");
  gtk_box_pack_end(GTK_BOX(hbox),preampbutH,FALSE,FALSE,0);

  preampbutX = gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(preampbutH),"X");
  gtk_box_pack_end(GTK_BOX(hbox),preampbutX,FALSE,FALSE,0);
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(preampbutH),TRUE);
  
  g_signal_connect(G_OBJECT(preampbutH),"toggled",G_CALLBACK(button_press),NULL);

  // next pair of buttons, for preamp gain range

  hbox=gtk_hbox_new(FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),hbox,FALSE,FALSE,0);

  label = gtk_label_new("Pre-amp Gain:");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,0);

  gainH = gtk_radio_button_new_with_label(NULL,"High");
  gtk_box_pack_end(GTK_BOX(hbox),gainH,FALSE,FALSE,0);

  gainL = gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(gainH),"Low");
  gtk_box_pack_end(GTK_BOX(hbox),gainL,FALSE,FALSE,0);
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gainL),TRUE);
  
  g_signal_connect(G_OBJECT(gainL),"toggled",G_CALLBACK(button_press),NULL);

  sep = gtk_hseparator_new();
  gtk_box_pack_start(GTK_BOX(vbox),sep,FALSE,FALSE,0);


  // next button, for enable H preamp

  hbox=gtk_hbox_new(FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),hbox,FALSE,FALSE,0);

  label = gtk_label_new("Enable H/F preamp:");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,0);

  enableH = gtk_check_button_new();
  gtk_box_pack_end(GTK_BOX(hbox),enableH,FALSE,FALSE,0);
  g_signal_connect(G_OBJECT(enableH),"toggled",G_CALLBACK(button_press),NULL);

  // next button, for enable X preamp

  hbox=gtk_hbox_new(FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),hbox,FALSE,FALSE,0);

  label = gtk_label_new("Enable X preamp:");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,0);

  enableX = gtk_check_button_new();
  gtk_box_pack_end(GTK_BOX(hbox),enableX,FALSE,FALSE,0);
  g_signal_connect(G_OBJECT(enableX),"toggled",G_CALLBACK(button_press),NULL);

  // next button, for enable tune

  hbox=gtk_hbox_new(FALSE,0);
  gtk_box_pack_start(GTK_BOX(vbox),hbox,FALSE,FALSE,0);

  label = gtk_label_new("Enable tune");
  gtk_box_pack_start(GTK_BOX(hbox),label,FALSE,FALSE,0);

  enableT = gtk_check_button_new();
  gtk_box_pack_end(GTK_BOX(hbox),enableT,FALSE,FALSE,0);
  g_signal_connect(G_OBJECT(enableT),"toggled",G_CALLBACK(button_press),NULL);





  gtk_widget_show_all(window);



  gtk_main();

  return 0;
}
