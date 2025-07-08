#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "spinapi.h"

#define LBUFF_LEN 500
#define NAME_LEN 200

#define MAX_DEVICES 50
/*  program to preprocess a pulse sequence from a friendly human programmable form, into code suitable for the c compiler.
    December 19, 2000 Carl Michal 
    
    The compiler wants calls of the form:  event(double time, int num_events, char device,int value, ... )

    But in the human readable pulse program, I don't want to have the num_events, so this program takes a line like:

    EVENT time {device, value} {device, value} {device, value};

    and turns it into the above.  The EVENT must start a line (first non white-space characters).

    The whole EVENT needn't be on one line, but if it isn't, use a \ to indicate continuation on the next line
    Don't break a {device, value} pair across a line.

    Modified Feb 22, 2007 so that if the device is a PHASE, AMP or GRAD device, we insert a (float) before the argument
    One thing that is very confusing is that in the va_arg calls, we look for a double...

    Major revision June 4, 2007 for pulseblaster boards - now supports loops and branching
    In addition to EVENT, you can have LOOP[num_times], END_LOOP[label name of start loop], JSR[label name of routine],
    RTS, STOP, WAIT and BRANCH[label name of location to branch to]

    Nov 14, 2022. Add cast for integer devices too! (unsigned int)
    Add parentheses around all casted arguments

*/


int deal(char *lbuff, FILE *infile, FILE *outfile){

  char *p1,*eo,*tok2;
  char tok[NAME_LEN],olbuff[LBUFF_LEN];
  char label[NAME_LEN];
  int l,count,is_float,is_string;
  

  // here we need to parse the command line, figure out how many devices we're trying to control, and create the correct c
  // code.
  p1 = lbuff;
  //  printf("in deal, got lbuff: %s\n",lbuff);
  count = 0;
  olbuff[0]=0;
  do{
    if (p1[0] == ';'){
      fprintf(outfile,",%i%s);%s",count,olbuff,&p1[1]);  // takes whatever appears on the line after the ; as well
      return 0;  // there used to be a newline at the end of this string, but it shouldn't be needed
    }
      
    if (p1[0] == '\\'){
       eo = fgets(lbuff,LBUFF_LEN,infile);
       if (eo == NULL){
	 printf("\ncontinuation line found, but eof\n");
	 return 1;
       }
       if ( strstr(lbuff,"EVENT") != 0){
	 printf("\nContinuation line found, but EVENT in the next line\n");
	 return 1;
       }
       p1 = strstr(lbuff,"{");
       if (p1 == 0){
	 printf("\nNo { found after continuation line\n");
	 return 1;
       }
    }

    if (p1[0] == '{'){  // start of an event
      count +=1;
      l = strcspn(p1+1,"}");
      if (l == strlen(p1+1)){ // so p1+1+l points to the }
	printf("\nFound a { without matching } on the same line\n");
	return 1;
      }
      // ok, so p1 points to the { and p1[l] points to the }
      // check to see if we start with PHASE or AMP
      
      strncpy(tok,p1+1,l);// copy the whole thing inside the {} to tok
      p1 = p1+l+1; 

      tok[l]=0;// make sure our string is terminated
      // now tok has both our args in it.

      // make sure that the device, value pair acutally is a pair - that it has a "," in it
      tok2=strstr(tok,",");
      if (tok2 == NULL){
	printf("\nA device value pair doesn't seem to be a pair\n");
	return 1;
      }
      // now tok2 points to the ,

      tok2[0]=0;
      //      printf("first arg: %s, second arg: %s\n",tok,&tok2[1]);
      
      is_float = 0;

      if(strstr(tok,"PHASE") != NULL )
	is_float = 1;

      if ((strstr(tok,"GRAD") != NULL) && (strstr(tok,"GRAD_ON") == NULL))
	   is_float=1;
      // treat AMP separately, because _AMP devices are the actual integer devices.
      if((strstr(tok,"AMP") != NULL) && (strstr(tok,"_AMP") == NULL))
	is_float = 1;
      
      is_string = 0;
      if (strstr(tok,"LABEL") != NULL){ // its a label
	is_string = 1;
	sscanf(tok2+1,"%s",label);
      }

      // check first args for PHASE, AMP, GRAD
      if (is_float)
	snprintf(&olbuff[strlen(olbuff)],LBUFF_LEN-strlen(olbuff),",%s,(float) %s",tok,tok2+1);
      else if (is_string)
	snprintf(&olbuff[strlen(olbuff)],LBUFF_LEN-strlen(olbuff),",%s,\"%s\"",tok,label);
      else
	snprintf(&olbuff[strlen(olbuff)],LBUFF_LEN-strlen(olbuff),",%s,(unsigned int) (%s)",tok,tok2+1);
    }

    l = strcspn(p1+1,"{;\\");
    if (l == strlen(p1+1)){
      printf("\nError, line ends without ; or \\\n");
      return 1;
    }

    p1=p1+l+1;

  }while( 0 == 0); 

  fprintf(outfile,"\n");
  return 0;
}

int deal_simple(char *lbuff,FILE *infile, FILE *outfile,int opcode,char *name){

  char *p0;
  char tok[NAME_LEN];
  int l,len;

  // this handles the events without arguments: EVENT-> CONTINUE/LONG_EVENT
  // STOP, WAIT, and RTS



  // first, spit out the "event(time,"  bit
  p0 = strstr(lbuff,name); // p0 points to start of Event label

  if(p0 == NULL) return 1;
  len = strlen(name);

  // get whatever might be in front of the EVENT (like a // for a comment)
  p0[0]=0;
  fprintf(outfile,"%s",lbuff);

  // find next delimter
  l = strcspn(p0+len+1,";{\\");// p0+len+1+l points to first delimiter
  strncpy(tok,p0+len+1,l);
  tok[l]=0;

  fprintf(outfile,"event_pb((double) %s,%i,0",tok,opcode);


  if (l == strlen(p0+len+1)){
    printf("\nNo ; { or \\ found after a time\n");
    return 1;
  }
  // ok that's the time done.  Now need to figure out how many arguments.

  return (deal(p0+len+1+l,infile,outfile));

}

int deal_argument(char *lbuff,FILE *infile, FILE *outfile,int opcode,char *name){

  char *p0,*p1;
  char tok[NAME_LEN];
  int l,len;
  char label[NAME_LEN];

  // this handles the events with arguments:
  // END_LOOP, JSR, BRANCH, and LOOP



  // first, spit out the "event(time,"  bit
  p0 = strstr(lbuff,name); // p0 points to start of event name

  if(p0 == NULL) return 1;
  len = strlen(name);

  // get whatever might be in front of the EVENT (like a // for a comment)
  p0[0]=0; // kill first character of event name with a null
  fprintf(outfile,"%s",lbuff);

  l=strcspn(p0+len,";{\\["); 
  
  p0=p0+len+l; // now points to the first delimiter
  if (p0[0] != '['){
    printf("didn't find a [ immediately after event label with an argument,found: %c\n",p0[0]);
    return -1;
  }

  l = strcspn(p0,";{\\]");
  p1 = p0+l; // p1 points to the closing delimiter
  if (p1[0] != ']'){
    printf("argument for LOOP, JSR, BRANCH, or END_LOOP didn't end with ]\n");
    return -1;
  }
  
  p1[0] = 0;  // null the ]
  
 // want to put everything up to the ] in label for LOOP, END_LOOP, JSR and BRANCH
  sscanf(p0+1,"%s",label);
  printf("got argument: %s\n",label);


 

  p0 = p1+1; // should point at start of time.
  l = strcspn(p0,";{\\");
  strncpy(tok,p0,l);
  tok[l]=0;

  if (opcode == LOOP)
    fprintf(outfile,"event_pb((double) %s,%i,%s",tok,opcode,label);
  else{
    fprintf(outfile,"label_to_resolve(\"%s\");\n",label);
    fprintf(outfile,"%sevent_pb((double) %s,%i,0",lbuff,tok,opcode);
  }

  if (l == strlen(p0)){
    printf("\nNo ; { or \\ found after a time\n");
    return 1;
  }
  // ok that's the time done.  Now need to figure out how many arguments.

  return (deal(p0+l,infile,outfile));

}



int main(int argc,char *argv[]){

  char *eo;
  char lbuff[LBUFF_LEN],fname[NAME_LEN];
  FILE *infile,*outfile;
  
  
  if (argc !=2) {
    printf("%s called incorrectly\n",argv[0]);
    exit(1);
  }
  
  
  //printf("Got: %s as input\n",argv[1]);
  
  // make sure there's a .x at the end.
  
  strncpy(fname,argv[1],NAME_LEN);
  
  if (strcmp(".x",&argv[1][strlen(argv[1])-2]) != 0){
    strcat(fname,".x");
  }
  
  //  printf("working with name: %s\n",fname);
  
  // ok, try to open the source and output files
  infile = fopen(fname,"r");
  if (infile ==NULL){
    printf("couldn't open infile: %s\n",fname);
    exit(1);
  }
  strcat(fname,".c");
  outfile = fopen(fname,"w");
  if(outfile ==NULL){
    printf("couldn't open outfile: %s\n",fname);
    exit(1);
  }

 eo = fgets(lbuff,LBUFF_LEN,infile);

 do{
      // see if there's an EVENT in this line
   if ( strstr(lbuff,"EVENT") != NULL){
     if (deal_simple(lbuff,infile,outfile,CONTINUE,"EVENT") != 0){ 
       printf("Syntax error in EVENT:\n%s\n",lbuff);
       exit(-1);
     }    
   }
   else if (strstr(lbuff,"STOP") != NULL){
     if (deal_simple(lbuff,infile,outfile,STOP,"STOP") != 0){
       printf("Syntax error in STOP:\n%s\n",lbuff);
       exit(-1);
     }
   }
   else if (strstr(lbuff,"WAIT") != NULL){
     if (deal_simple(lbuff,infile,outfile,WAIT,"WAIT") != 0){
       printf("Syntax error in WAIT:\n%s\n",lbuff);
       exit(-1);
     }
   }
   else if (strstr(lbuff,"RTS") != NULL){
     if (deal_simple(lbuff,infile,outfile,RTS,"RTS") != 0){
       printf("Syntax error in RTS:\n%s\n",lbuff);
       exit(-1);
     }
   }
   else if (strstr(lbuff,"BRANCH") != NULL){
     if (deal_argument(lbuff,infile,outfile,BRANCH,"BRANCH") != 0){
       printf("Syntax error in BRANCH:\n%s\n",lbuff);
       exit(-1);
     }
   }
   else if (strstr(lbuff,"JSR") != NULL){
     if (deal_argument(lbuff,infile,outfile,JSR,"JSR") != 0){
       printf("Syntax error in JSR:\n%s\n",lbuff);
       exit(-1);
     }
   }
   else if (strstr(lbuff,"END_LOOP") != NULL){ // this has to be before LOOP, since that would find this too...
     if (deal_argument(lbuff,infile,outfile,END_LOOP,"END_LOOP") != 0){
       printf("Syntax error in END_LOOP:\n%s\n",lbuff);
       exit(-1);
     }
   }
   else if (strstr(lbuff,"LOOP") != NULL){
     if (deal_argument(lbuff,infile,outfile,LOOP,"LOOP") != 0){
       printf("Syntax error in LOOP:\n%s\n",lbuff);
       exit(-1);
     }
   }
   else fprintf(outfile,"%s",lbuff);

   eo = fgets(lbuff,LBUFF_LEN,infile);

 }while (eo != NULL);
    

return 0;

}
