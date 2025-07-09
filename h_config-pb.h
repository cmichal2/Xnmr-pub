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


/*  h_config.h
 *
 *  This file specifies the hardware configuration of the pulse 
 *  programmer hardware and for the oscilloscope
 *
 * Part of Xnmr software project
 *
 * UBC Physics
 * April, 2000
 * 
 * written by: Scott Nelson, Carl Michal
 */

#ifndef H_CONFIG_H
#define H_CONFIG_H



/*
 *  Pulse Hardware information
 */


#define DEFAULT_IF 138281250.
#define DEFAULT_RCVR_CLK 60000000.

#define NUM_DEVICES 24
#define MAX_EVENTS 32768
#define NUM_BOARDS 2

//#define CLOCK_SPEED 20000000
//#define MAX_CLOCK_CYCLES 4294967295UL
/*
 * This is the configuration format which must be followed
 * (For use with sscanf)
 */

#define H_CONFIG_FORMAT "#define %s %u // %u %u %u %u %lg\n"
#define PARSE_START "#define H_CONFIG_PARSE\n"
#define PARSE_END "#undef H_CONFIG_PARSE\n"


/*
 * This next line signals a parsing algorithm to begin searching this file.
 * Do not add any comments where CONFIG_PARSE is defined except to specify
 * device start bit, etc. 
 *
 *
 * Format for parsing is as follows
 *
 * #define <symbol> <device number> // <start bit> <num bits> <latch> <default> <max duration>
 *
 */

// the synthesizer device names are used internally in libxnmr (pulse.c) - freq set routines.

// the amplitude and phase lines as well as RF and RF_BNLK are used internally in there as well.

// the  XTRN_TRIG and PP_OVER are all hardwired inside, and used in pulse.c
// the second board starts at 32 - our logic assumes there's 32 bits in a board...


#define H_CONFIG_PARSE

#define WR_A              0 //  0    1  0  0  0
#define ADD_A             1 //  1    6  0  0  0
#define DAT_A             2 //  7    8  0  0  0
#define UPD_A             3 // 15    1  0  0  0
#define GATE_A            4 // 16    1  0  0  0
#define GATE_B            5 // 17    1  0  0  0
#define BLNK_A            6 // 18    1  0  0  0
#define BLNK_B            7 // 19    1  0  0  0
#define BNC_0             8 // 20    1  0  0  0
#define BNC_1             9 // 21    1  0  0  0
#define BNC_2            10 // 22    1  0  0  0
#define SLAVEDRIVER      11 // 23    1  0  0  0
#define WR_B             12 // 32    1  0  0  0
#define ADD_B            13 // 33    6  0  0  0
#define DAT_B            14 // 39    8  0  0  0
#define UPD_B            15 // 47    1  0  0  0
#define BNC_3            16 // 48    1  0  0  0
#define BNC_4            17 // 49    1  0  0  0
#define BNC_5            18 // 50    1  0  0  0
#define RCVR_GATE        19 // 51    1  0  0  0
#define ACQ_TRIG         20 // 52    1  0  0  0
#define SYNC_CIC         21 // 53    1  0  0  0
#define SYNC_NCO         22 // 54    1  0  0  0
#define PP_INTERRUPT     23 // 55    1  0  0  0

#undef H_CONFIG_PARSE




// these are bogus devices that pulse.c will translate for us.
// numbers above RF_OFFSET get translated according to users toggle button channel assignment
// don't change these numbers!
// write_device wrap recognizes those and does the right thing with them

// LABEL is a pseudo device that assigns a text label to an event.
// the FREQ, AMP and PHASE devices get picked up by event_pb and
// turned into the correct synth programming events.
#define LABEL 199
#define FREQA 198
#define FREQB 197
#define AMP1 196
#define AMP2 195
#define PHASE1 194
#define PHASE2 193

#define RF_OFFSET 200
#define RF1      200
#define BLNK1 201

#define RF2      202
#define BLNK2 203


#define  AD_STAQ ACQ_TRIG





#endif







