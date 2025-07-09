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

// many of these things require a recompile to take effect!
// due pulse programmer:
#define MAX_EVENTS 12000
#define NUM_BOARDS 1
#define DUE_PP_CLOCK 50000000


// Lime SDR boards:
#define NUM_TX 2
// only declare the serial numbers inside of lime-duepp.c
// An ugly hack but I want this declared here...
#ifdef LIME_SERIAL
// order of serial numbers matters!
const char * lime_serials[] = {"0009070105C61D34","0009070105C61923"};
//const char * lime_serials[] = {"0009070105C61D34"};
#endif
#define MAX_TX_EVENTS 262144 // could be bigger!
#define TX_SAMPLE_RATE 10000000
//#define TX_SAMPLE_RATE 25000000
//#define TX_SAMPLE_RATE 50000000

// Gradient driver:
#define MAX_GRAD_EVENTS 262144
#define BGU_SCALE_FACTORS // 1.00 0.977 0.966
// scale factors for x, y, z from 2000 installation of the Bruker

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

#define NUM_DEVICES 16
#define H_CONFIG_PARSE

#define LIME_SYNC         0 //  0    1  0  0  0
#define GATE_A            1 //  1    1  0  0  0
#define GATE_B            2 //  2    1  0  0  0
#define RCVR_GATE_A       3 //  3    1  0  0  0
#define RCVR_GATE_B       4 //  4    1  0  0  0
#define BLNK_A            5 //  5    1  0  0  0
#define BLNK_B            6 //  6    1  0  0  0
#define BNC_0             7 //  7    1  0  0  0
#define BNC_1             8 //  8    1  0  0  0
#define BNC_2             9 //  9    1  0  0  0
#define BNC_3            10 // 10    1  0  0  0
#define BNC_4            11 // 11    1  0  0  0
#define BNC_5            12 // 12    1  0  0  0
#define BNC_6            13 // 13    1  0  0  0
#define BGU_TRIG         14 // 14   1  0  0  0
#define BGU_NGI          15 // 15    1  0  1  0

// NGI idles high.
/* BIG FAT WARNING

Any pins that idle high, should also be set to do so
in the due pulse programmer firmware!

*/

#undef H_CONFIG_PARSE

// these are bogus devices that pulse.c will translate for us.
// numbers above RF_OFFSET get translated according to users toggle button channel assignment
// don't change these numbers!
// write_device wrap recognizes those and does the right thing with them

// LABEL is a pseudo device that assigns a text label to an event.

#define LABEL 199
#define RF_OFFSET 200
#define BLNK1 200
#define BLNK2 201

#define RCVR_GATE1 202
#define RCVR_GATE2 203

#define GATE1 204
#define GATE2 205

#define TX1 220
#define TX2 221
#define RX1 230
#define RX2 231
// device id's only go up to 255!


#endif







