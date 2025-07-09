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

// File ADepp.h

#ifndef _ADEPP_
#define _ADEPP_

//#include <sys/io.h>


// The EPP handles all control of the Write, Data Strobe and Wait lines
// One only needs to write the data to the correct register to have the
// handshaking performed automatically.  The registers are as follows:


#define  EPP_DATA  0x04
#define  EPP_ADDR  0x03
#define  SPP_CTRL  0x02
#define  SPP_STAT  0x01
#define  SPP_DATA  0x00


// With the AD board one must always call select_device when switching io between
// the FIFO and the DSP - the AD hardware requires this.


// Define addresses of various devices addresed by the EPP data lines.
// These will be used to set a bit mask for A7 of the interface control
// latch.  FIFO_FLIP_FLOP and DSP_MICROPORT are not used.

#define FIFO  0                 // 00000000
#define DSP  128                // 10000000


// The following constants define the bit masks for the DSP control lines

#define DSP_NCS  1        // 0000001
// Next three lines are for A0-A2 address lines of micro_port

#define DSP_NWR 16  // write flag for mode 0
#define DSP_NRD 32  // read flag for mode 0
#define DSP_NRESET  64  // 0100000

// FIFO control line bit masks 

#define FIFO_OFF  1     // Data bypasses DSP if this isn't there
// The OEA line should be toggled every other read    
#define OEA  2         // OEA on AD board = High byte of fifo data
#define STAQ_ON  4     // 0000100  starts fifo aquiring data
#define MR_OFF  8      // 0000100 reset line for fifo
// The R line is connected to RCLK (rising edge triggered)
//  and should be toggled every second time the fifo is read
#define R_ON  16           // 0001000 triggers a read from the fifo
#define LOADMODE_OFF  32   // 0010000 direct aquisition from digitizer
// Line A6 not used

void select_device(unsigned char address);
// Selects the device at "address" for reading and/or writing by writing
// "address" to EPP address register. 

int initialize_dsp_epp(int p);
// Must be called before any epp related functions


// On the AD board, fifo control is achieved by an address write
// with line A7 = 0.  The address lines correspond to the following:
// A0       A1         A2       A3        A4      A5        A6        A7
// FIFOON   OEA        STAQ     MR        R       LOADMODE  not used  0

void reset_fifo();
// Pre: None
// Post: Fifo is reset and STAQ is turned off

void reset_dsp();

void start_acquire_bypass();
// Pre: the fifo should probably be reset before starting each aquisition
// Post: fifo begins to acquire data directly from the digitizer

void start_acquire_dsp();
// Pre: the fifo should probably be reset before starting each aquisition
// Post: fifo begins to acquire data from the DSP

void start_acquire_pulse();
//  Makes fifo ready for dsp acquire, waits for pulse

int read_fifo_epp(int npts,int *data,int receiver_model);
// Pre: Fifo control lines are active
// Post: reads high and low bytes from data lines coming from fifo



void write_micro(unsigned char addr, unsigned char data);
// Pre: DSP microport data lines selected (control lines are active)
// Post: epp data lines D0-D7 are sent to an internal address location
// in the DSP specified by A0-A2

void dsp_close_port_epp();
void clear_interrupt();
void wait_interrupt();
int request_data_epp(int npts,int receiver_model);
int query_ready_epp();
#endif






