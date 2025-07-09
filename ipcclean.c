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

