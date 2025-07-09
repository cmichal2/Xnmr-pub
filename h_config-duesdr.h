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

#define AIRSPY
// #define RX888
// XXX SOAPY TODO  h_config-duesdr should have an entry for airspy/rx888 that gets looked up, so that can
// all be done runtime instead of compile time, serial number too - should get looked up at runtime


/*
 *  Pulse Hardware information
 */

// many of these things require a recompile to take effect!
// due pulse programmer:
#define MAX_EVENTS 12000
#define NUM_BOARDS 1
#define DUE_PP_CLOCK 50000000


// SDR boards:
#define NUM_RX 1

// only declare the serial numbers inside of lime-duepp.c
// An ugly hack but I want this declared here...
//#ifdef LIME_SERIAL
// order of serial numbers matters!
//const char * lime_serials[] = {"0009070105C61D34","0009070105C61923"};
//const char * lime_serials[] = {"0009070105C61D34"};
//#endif


#define MAX_RX_EVENTS 262144 // could be bigger! This is size of ring buffer. Can have only half this in a pulse program.
#ifdef AIRSPY
#define RX_SAMPLE_RATE 10000000 // 10 MHz for Airspy, 64 MHz (? or 32? for RX888?)
#elif defined RX888
#define RX_SAMPLE_RATE 32000000 // is this right, or is it 16 MHz?
#endif

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

#define NUM_DEVICES 13
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
#define RCVR_GATE_A      11 // 23    1  0  0  0
#define RCVR_GATE_B      12 // 24    1  0  0  0


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

// keep AMP2 and PHASE2 here even if there's only one channel.
// also FREQB

// these devices are handled separately in software:
#define LABEL 199
#define FREQA 198
#define FREQB 197
#define AMP1 196
#define AMP2 195
#define PHASE1 194
#define PHASE2 193
#define RX1 192
#define RX2 191

// these get looked up and translated to be above devices.
#define RF_OFFSET 200
#define RF1      200
#define BLNK1 201
#define RCVR_GATE1 202
#define GATE1 203

#define RF2      204
#define BLNK2 205
#define RCVR_GATE2 205
#define GATE2 206

// these get handled separately in software. The ALT's, should really get built up just the way normal hardware devices are...
#define ALT1 207
#define ALT2 208

#define DACA1 300
#define DACA2 301
#define DACB1 302
#define DACB2 303
#define ALTA 304
#define ALTB 305





#endif







