/*
 *  ipcclean.c
 *
 *  This program just cleans up any leftover IPC structres from
 *  the improper termination of Xnmr or acq
 *
 *  Part of the Xnmr software project
 *
 *  UBC Physics,
 *  April, 2000
 *
 *  written by: Scott Nelson, Carl Michal
 */


#include <gtk/gtk.h>
#include "param_f.h"

/*
 *  Global Variables
 */


#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/errno.h>
#include <sys/wait.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/msg.h>

#include "shm_data.h"
#ifdef MSL200
#include "shm_prog-pb.h"
#elif defined DUELIME
#include "shm_prog-duelime.h"
#elseif defined DUESDR
#include "shm_prog-duesdr.h"
#else
#include "shm_prog.h"
#endif

int main()

{
  int dataid;
  int progid;
  int msgqid;

  dataid = shmget( DATA_SHM_KEY, sizeof( struct data_shm_t ), 0);
  progid = shmget( PROG_SHM_KEY, sizeof( struct prog_shm_t ), 0);
  msgqid = msgget( MSG_KEY, 0 );     

  printf("dataid: %x, progid: %x, msgqid: %x\n",dataid,progid,msgqid);
  shmctl ( progid, IPC_RMID, NULL ); 
  shmctl ( dataid, IPC_RMID, NULL );
  msgctl( msgqid, IPC_RMID, NULL );

  return 0;
}

