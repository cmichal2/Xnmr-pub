#include <unistd.h>
#include  <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>


// program to talk to Xnmr through socket

  struct sockaddr_un my_addr,wr_addr;
  int fd;

  struct sockaddr_un my_addrs,wr_addrs;
  int fds;


int tell_xnmr(char *iline){
  char buff[800];
  strncpy(buff,iline,800);
  strncat(buff,"\n",800);
  printf("telling Xnmr: %s\n",buff);

  sendto(fd,buff,strnlen(buff,800),0,(struct sockaddr *)&wr_addr,SUN_LEN(&wr_addr));
  return 0;
}

int read_xnmr(char *oline){
  int rlen,fromlen;
  fromlen = 108;

  rlen=recvfrom(fd,oline,800,0,(struct sockaddr *)&wr_addr,&fromlen);
  oline[rlen] = 0;
  printf("received from Xnmr: %s\n",oline);
}


int tell_shims(char *message){
  printf("telling shims: %s\n",message);
  sendto(fds,message,strnlen(message,800),0,(struct sockaddr *) &wr_addrs,SUN_LEN(&wr_addrs));
  return 0;
}

int read_shims(char *oline){
  int rlen,fromlen;
  fromlen = 108;

  rlen=recvfrom(fds,oline,800,0,(struct sockaddr *)&wr_addrs,&fromlen);
  oline[rlen] = 0;
  printf("received from shims: %s\n",oline);
}

main(){

  struct stat buf;
  int i;
  char iline[800],oline[800];


  i = stat("/tmp/Xnmr_remote",&buf);
  if (i == -1){
    printf("from stat got: %i, Xnmr socket not open?\n");
    exit(0);
  }

  // target name:
  wr_addr.sun_family = AF_UNIX;
  strncpy(wr_addr.sun_path,"/tmp/Xnmr_remote",17);
  

  // my name:
  my_addr.sun_family = AF_UNIX;
  snprintf(my_addr.sun_path,108,"/tmp,Xnmr_rem%d",getpid());

  //open my socket
  fd = socket(PF_LOCAL,SOCK_DGRAM,0);
  if (fd == 0) {
    perror("socket:");
    exit(0);
  }

  //bind to my name:
  if (bind(fd,(struct sockaddr *)&my_addr,SUN_LEN(&my_addr)) != 0){
    perror("bind:");
    exit(0);
  }
  chmod(my_addr.sun_path,0666);

  printf("Xnmr init completed\n");


  i = stat("/tmp/shimsd",&buf);
  if (i == -1){
    printf("from stat got: %i, shims socket not open?\n");
    exit(0);
  }

  // target name:
  wr_addrs.sun_family = AF_UNIX;
  strncpy(wr_addrs.sun_path,"/tmp/shimsd",12);
  

  // my name:
  my_addrs.sun_family = AF_UNIX;
  snprintf(my_addrs.sun_path,108,"/tmp/shims%d",getpid());

  //open my socket
  fds = socket(PF_LOCAL,SOCK_DGRAM,0);
  if (fds == 0) {
    perror("socket:");
    exit(0);
  }

  //bind to my name:
  if (bind(fds,(struct sockaddr *)&my_addrs,SUN_LEN(&my_addrs)) != 0){
    perror("bind:");
    exit(0);
  }
  chmod(my_addrs.sun_path,0666);



  // do some testing:
  // for Xnmr:
  // after every write, read a response, except read twice after ACQUIRE
  /*
  tell_xnmr("LOAD acq_temp");
  read_xnmr(oline);

  tell_xnmr("ACQUIRE");
  read_xnmr(oline);
  read_xnmr(oline);


  tell_xnmr("SAVE ctest2");
  read_xnmr(oline);

  
  tell_xnmr("APPEND");
  read_xnmr(oline);
  */

  //  tell_xnmr("EXIT");
  // just exits Xnmr's scripting.


    // for shims, don't register or we get lots of crap back.
    // only read after a get

  tell_shims("SH:GET:Z2:");
  read_shims(oline);

  tell_shims("SH:SET:Z2:100");

  tell_shims("SH:GET:Z2:");
  read_shims(oline);

  tell_shims("SH:SET:Z2:150");

  tell_shims("SH:GET:Z2:");
  read_shims(oline);


  unlink(my_addr.sun_path);
  unlink(my_addrs.sun_path);

}

