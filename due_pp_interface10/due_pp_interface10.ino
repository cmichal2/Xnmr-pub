/*

   Version 10 responsds to most instructions. Trying to get
   to the bottom of some hangs.

  version 9 catches interrupts so parallel port should no longer be necessary!

  version 8 is for Electronics shop board that includes the reset line
  from the arduino, not piped through to the parallel port. For now,
  interrupts are still delivered to the parallel port. This may
  change in a future version.

  version 7 tries to deal with OEA/PPREAD a little better so that
  we don't ask both output latches to turn on at the same time.
  R idling low more of the time - changed R high to a short pulse rather than
  half the read cycle.


  This program is to interface an arduino Due to an AD6620 evaluation board, taking much of the functionality
  of adepp.c from Xnmr.

  version 3 has the data collection seemingly all worked out. Just need to test the asm
  code and see if the delays are long enough and in the right places!

  version 6 is for a more hacked up version of the board, more connections, drive R and OEA
  directly!

  routines in adepp:


  D0 or D1 - select device - simple  0 or 1 are binary 0/1 0 is FIFO, 1 is DSP DONE
  S = start_acquire_pulse - simple 0 DONE
  Cxxxx read_fifo_epp -   xxxx is 4 byte binary number of points to acquire. DONE

  R = reset_dsp - DONE
  Wad = write_micro -some work a = char binary address, d = data to write.

  Will also want:
  A = Abort (to abort a read_fifo_epp) - DONE
  Q = Query - to get version DONE

  T = hit the reset button on the synthesizers
  i = clear my interrupt flag
  I = wait till an interrupt has been received (returns immediately if it came before the call)


  Thinking currently (The C ports are likely to change):
  D.0-D.7 for port data input.
  (arduino pins are: D25, D26, D27, D28, D14, D15, D29, D11)
  D.8 (D12) for fifo Empty input.

  C.1-C.8 for data outputs (arduino pins: D33, D34,D35,D36,D37,D38,D39,D40)
  C.9 (D41) for WR
  C.12 (D51) for AS
  C.13 (D50) for DS
  C.14 (D49)  for FIFO R
  C.15 (D48)  for OEA

  On the centronics connector we need 8 data bits, 3 strobes, GND, and something for the empty line.
  data pins are 2-9, AS is 36, WR is 1, DS is 14, empty line should come on
  12 (FULL) or 13 ( DD0) or 32 (AA0)

  ///// C.14 (D49) for reset - ad6620 board ignores reset pin!

  C.1 - C.8 are outputs with open drain - pullup enabled, write them as high to read.


  pin mapping:
  centronics  Arduino
  1 (WR)       D41 (C.9)                Brown (Brn/Blk)  - signal (ground)
  2 (data0)    D25 (D.0) and D33 (C.1)  Red (Red/Blk)
  3 (data1)    D26 (D.1) and D34 (C.2)  Orange (Orange/Blk)
  4 (data2)    D27 (D.2) and D35 (C.3)  Pink (Pink/Blk)
  5 (data3)    D28 (D.3) and D36 (C.4)  Yellow (Yellow/Blk)
  6 (data4)    D14 (D.4) and D37 (C.5)  Green (Green/Blk)
  7 (data5)    D15 (D.5) and D38 (C.6)  Deep Blue (Deep Blue/Blk)
  8 (data6)    D29 (D.6) and D39 (C.7)  Purple (Purple/Blk)
  9 (data7)    D11 (D.7) and D40 (C.8)  Grey (Grey/Blk)
  14 (DS)      D50 (C.13)               Grey/Red (Grey/White)
  36 (AS)      D51 (C.12)               Light Blue (Light Blue/Blk) (careful, there is a paler blue than this.
  19-30 GND

  12 FULL     D12 (D.8) hacked on DSP to connect to EMPTY  Pink/Red (Pink/White)
  13 DD0      D49 (C.14) hacked on DSP to R line on FIFO   Yellow/Red (Yellow/White)
  32 AA0      D48 (C.15) hacked on DSP to OEA line on FIFO White/Red (Red/White)

  11 [ACK/Busy]  D30 (D.9) not used                        Orange/Red (Orange/White)

  N/C                                    Blk/(Blk/White)

  10 (interrupt) D52 (B.21) also - maintain to DB25 pin 10 White (White/Blk)
  31 (reset)   D22 (B.26)     Pale Blue (Pale Blue/Blk)

  On arduino, most of these are on the end block, except for D14, D15, D11, and D12
  20 connections on arduino + ground.
  12 parallel port + ground.

  Need to keep parallel port interrupt line intact!

  Timing-wise, this current version looks like it may work!

  0.25 us = 21 clock ticks.or 42 clock ticks for 2 bytes.

  The pulse on the DS may have to be longer. More like 4 clock cycles, 48 ns.


*/

// these are port pins in PIOC - all shifted up one bit from where you might think because
// we use C.1 - C.8 (C.0 is tied up...)
#define FIFO 0
#define DSP 0x100 // bit 7 in the parallel port.

#define DSP_NCS 0x02 // bit 0

// dsp microport control
#define DSP_AA0 0x04 // bit 1
#define DSP_AA1 0x08 // bit 2
#define DSP_AA2 0x10 // bit 3
#define DSP_NWR 0x20 // bit 4
#define DSP_NRD 0x40 // bit 5
#define DSP_NRESET 0x80 // bit 6

//lines in FIFO control:
#define FIFO_OFF 0x02 // bit 0
#define OEA 0x8000      // no longer in FIFO latch
#define STAQ_ON 0x08  // bit 2
#define MR_OFF 0x10   //bit 3

#define R_ON 0x4000 // no longer in FIFO latch
#define LOADMODE_OFF 0x40 // bit 5

#define CBITS 0x1FE  // data bits in register C

// these are port control lines, also in PIOC
#define WR 0x0200
#define AS 0x1000
#define DS 0x2000


#define EMPTYB 0x100 // where the empty bit is read in reg D.

// all these constants are set by hand in the assembly.

#include "chip.h"
#include <stdio.h>

// this should reserve r8 just for us!
//register unsigned int cread asm("r8");

// iflag is my interrupt flag - when an interrupt is received, it is set.
volatile unsigned char iflag = 0;

void catch_int() {
  iflag = 1;
}

void setup()
{
  SerialUSB.begin(29600);
  pinMode(12, INPUT); // set one of the PIOD pins as an input
  // so that the arduino pinMode functions turns on the
  // peripheral clocking, otherwise no inputs observed.
  // this pin 12 is D.8 - what we'll use for FIFO_EMPTY.
  pinMode(13, OUTPUT); // LED line
  pinMode(22, OUTPUT); //reset line on port - for synths

  // configure ports:
  // set the data bits + strobes all high: except for R_ON - idle low.
  //  PIOC->PIO_ODSR = 0x33FE; // ODSR is +56  all data bit + WR+AS+DS
  PIOC->PIO_ODSR = CBITS | WR | AS | DS | OEA ; // R_ON idle high here?
  // unmask parallel port bits for direct writes (8 data bits + 3strobes + 2 extras)
  //  PIOC->PIO_OWER = 0x33FE; // OWER is + 160 WR+AS+DS+data
  PIOC->PIO_OWER = CBITS | WR | AS | DS | R_ON | OEA; // OWER is + 160 WR+AS+DS+data

  // set  them as outputs (parallel port + 3 strobes + 2 extras)
  //  PIOC->PIO_OER = 0x33FE; // OER is +16
  PIOC->PIO_OER = CBITS | WR | AS | DS | R_ON | OEA; // OER is +16

  //pull-ups on data bits
  //  PIOC->PIO_PUER = 0x01FE; // PUER is +100
  PIOC->PIO_PUER = CBITS; // PUER is +100
  // open drain data bits
  //  PIOC->PIO_MDER = 0x01FE; // MDER is +80
  //  PIOC->PIO_MDER = CBITS; // MDER is +80

  ///// nothing, just refs for SODR, CODR, PDSR
  //PIOC->PIO_SODR = 0x10100101; // set bits is +48
  //PIOC->PIO_CODR = 0x01; // clear bits is +52
  // something = PIOC->PIO_PDSR; // PDSR is +60
  // to write to upper 16 bits, can't just MOV into register, need to LDR it.
  /////

  // the port D pins default to inputs.
  digitalWrite(13, LOW);
  // register our ISR:
  attachInterrupt(52, catch_int, RISING);
}

inline void add_write(uint32_t mask) {
  PIOC->PIO_ODSR = mask | DS | AS | OEA; // WR1 low
  asm("nop\n\tnop");
  PIOC->PIO_ODSR = mask | DS | OEA ; // WR1+AS low
  asm("nop\n\tnop");
  asm("nop\n\tnop");
  PIOC->PIO_SODR = AS; // raise AS
  asm("nop\n\tnop");
  asm("nop\n\tnop");
  PIOC->PIO_ODSR = CBITS | WR | AS | DS | OEA ;
}

inline void data_write(uint32_t mask) {
  PIOC->PIO_ODSR = mask | AS  | DS | OEA ; // WR1 low
  asm("nop\n\tnop");
  PIOC->PIO_ODSR = mask | AS  | OEA ; // WR1+DS low
  asm("nop\n\tnop");
  asm("nop\n\tnop");
  PIOC->PIO_SODR = DS; // raise DS
  asm("nop\n\tnop");
  asm("nop\n\tnop");
  PIOC->PIO_ODSR = CBITS | WR | AS | DS | OEA ;

}

void loop()
{
  uint32_t i, j, npacket, nword, mask, paddress, pdata, npts;
  unsigned char buff[16];
  //  size_t size = 512;
  uint8_t *ptr_dest;
  const uint8_t *ptr_src;
  uint32_t read1, read2;
  unsigned long inc;
  // sl = r10, fp=r11, ip=r12, sp=r13, lr=r14. r15 = pc
  // r15 and r13 are off limits. r14 may be ok. 0 -12 are ok.

  while (0 == 0) {
    while (SerialUSB.available() == 0);
    buff[0] = SerialUSB.read();
    switch (buff[0]) {
      case 'Q': // query
        SerialUSB.println("USB to 6620EVB Due adapter v10");
        break;
      case 'i': // reset my interrupt flag
        iflag = 0;
        SerialUSB.write(buff[0]);
        break;
      case 'I': // wait till an interrupt comes:
        SerialUSB.write(buff[0]);
        while (iflag == 0 && SerialUSB.available() == 0);
        if (SerialUSB.available() == 0)
          SerialUSB.write('Z'); // 'Z' is not significant - just a 'random' number.
        iflag = 0;
        break;
      case 'D': // get 0 for FIFO, 1 for DSP
        buff[0] = SerialUSB.read();
        mask = DSP_NCS | DSP_NWR | DSP_NRD | DSP_NRESET;
        if (buff[0]) mask |= DSP;
        add_write(mask);
        SerialUSB.write('D');
        break;
      case 'S': // start acquire pulse
        mask = FIFO_OFF | DSP_NRESET; // reset the fifo
        add_write(mask);

        mask = FIFO_OFF | MR_OFF | DSP_NRESET;
        add_write(mask);
        SerialUSB.write(buff[0]);
        break;
      case 'T': // reset synths
        digitalWrite(22, LOW);
        digitalWrite(22, HIGH);
        SerialUSB.write('T');
        break;
      case 'R': // reset dsp
        mask = DSP | DSP_NWR | DSP_NRD;
        add_write(mask);
        mask = DSP | DSP_NWR | DSP_NRD | DSP_NRESET;
        add_write(mask);
        SerialUSB.write('R');
        break;
      case 'W': //write some data to DSP
        buff[0] = SerialUSB.read(); // microport address to write to
        buff[1] = SerialUSB.read(); // data to write
        /*
              // yuck!
              paddress = 0; // 3 bits in address line
              if (buff[0] & 1) paddress |= DSP_AA0;
              if (buff[0] & 2) paddress |= DSP_AA1;
              if (buff[0] & 4) paddress |= DSP_AA2;
              pdata = 0;
              if (buff[1] & 1)  pdata |= DSP_NCS;
              if (buff[1] & 2)  pdata |= DSP_AA0;
              if (buff[1] & 4)  pdata |= DSP_AA1;
              if (buff[1] & 8)  pdata |= DSP_AA2;
              if (buff[1] & 16)  pdata |= DSP_NWR;
              if (buff[1] & 32)  pdata |= DSP_NRD;
              if (buff[1] & 64)  pdata |= DSP_NRESET;
              if (buff[1] & 128)  pdata |= DSP;
        */
        paddress = ((uint16_t) buff[0]) << 2;
        pdata = ((uint16_t) buff[1]) << 1;

        mask = DSP | DSP_NCS | DSP_NWR | DSP_NRD | DSP_NRESET | paddress;
        add_write(mask);
        asm("nop\n\tnop");
        asm("nop\n\tnop");
        data_write(pdata);
        asm("nop\n\tnop");
        asm("nop\n\tnop");
        mask = DSP | DSP_NRD | DSP_NRESET | paddress;
        add_write(mask);
        asm("nop\n\tnop");
        asm("nop\n\tnop");
        mask = DSP | DSP_NCS | DSP_NWR | DSP_NRD | DSP_NRESET | paddress;
        add_write(mask);
        SerialUSB.write('W');
        break;
      case 'c': // collect data, but don't look at empty flag.
        npts = SerialUSB.read();
        npts += SerialUSB.read() * 256;
        npts += SerialUSB.read() * 256 * 256;
        npts += SerialUSB.read() * 256 * 256 * 256;
        SerialUSB.write(buff[0]);
        ptr_dest = (uint8_t *) &udd_get_endpoint_fifo_access8(CDC_TX);
        PIOC->PIO_MDER = CBITS; // MDER is +80 - turn off drivers on data bits
        PIOC->PIO_ODSR = CBITS | WR | AS | DS; // data bits high, strobes high, idle R (low), idle OEA (high)

        while ( UOTGHS_DEVEPTISR_TXINI != (UOTGHS->UOTGHS_DEVEPTISR[CDC_TX] & UOTGHS_DEVEPTISR_TXINI ));

        for (j = 1; j <= npts * 2; j++)      {
          PIOC->PIO_ODSR = CBITS | WR | AS | R_ON; // data bits high, strobes high, lower OEA,
          asm("nop");
          PIOC->PIO_ODSR = CBITS | WR | AS; // data bits high, strobes high, raise R, lower OEA, DS asserted
          asm("nop\n\tnop\n\tnop");
          asm("nop\n\tnop");
          read1 = PIOD->PIO_PDSR; //read

          *ptr_dest++ = read1 & 255;

          PIOC->PIO_ODSR = CBITS | WR | AS | DS ; //  raise DS
          asm("nop\n\tnop\n\t");
          asm("nop\n\tnop\n\t");
          asm("nop\n\tnop\n\t");
          PIOC->PIO_ODSR = CBITS | WR | AS | OEA; // assert DS, raise OEA for high byte.
          asm("nop\n\tnop");
          asm("nop\n\tnop");
          read2 = PIOD->PIO_PDSR; //read
          *ptr_dest++ = read2 & 255;
          if ((j % 256) == 0) {
            UOTGHS->UOTGHS_DEVEPTICR[CDC_TX] = UOTGHS_DEVEPTICR_TXINIC;
            UOTGHS->UOTGHS_DEVEPTIDR[CDC_TX] = UOTGHS_DEVEPTIDR_FIFOCONC;
            ptr_dest = (uint8_t *) &udd_get_endpoint_fifo_access8(CDC_TX);
            while ( UOTGHS_DEVEPTISR_TXINI != (UOTGHS->UOTGHS_DEVEPTISR[CDC_TX] & UOTGHS_DEVEPTISR_TXINI ));
          }
        }
        // need an extra packet sent if we finished in middle of a packet, or at end of an odd number of packets
        npacket = npts / 128;
        if ((npacket % 2 ) == 1 || (npts % 128 != 0)) {
          UOTGHS->UOTGHS_DEVEPTICR[CDC_TX] = UOTGHS_DEVEPTICR_TXINIC;
          UOTGHS->UOTGHS_DEVEPTIDR[CDC_TX] = UOTGHS_DEVEPTIDR_FIFOCONC;
        }
        PIOC->PIO_ODSR = CBITS | OEA | WR | AS | DS ; // data bits high, strobes high, OEA and R idle, DS deasserted
        PIOC->PIO_MDDR = CBITS; // MDER is +80 - turn on drivers on data bits

        break;
      case 'C': // collect data - 4 bytes binary to specify number of points to acquire. BROKEN! (er, not updated)
        // each complex point is 4 bytes.
        npts = SerialUSB.read();
        npts += SerialUSB.read() * 256;
        npts += SerialUSB.read() * 256 * 256;
        npts += SerialUSB.read() * 256 * 256 * 256;
        if (npts == 0) break;
        SerialUSB.write(buff[0]);

        PIOC->PIO_MDER = CBITS; // MDER is +80 - turn off drivers on data bits
        PIOC->PIO_ODSR = CBITS | OEA | WR | AS ; // data bits high, strobes high, idle R (high), idle OEA (high), DS asserted
        digitalWrite(13, HIGH);
        // disable interrupts while we're working:
        cpu_irq_disable();

        // each packet is 512 bytes = 512/4 = 128.
        //      for (j = 0; j < npts/128; j++) {
        npacket = npts / 128;
        read2 = 0; // pretend empty flag was low.
        // EMPTYB in here in tst:

        for (j = 0; j < npacket; j++) {

          ptr_dest = (uint8_t *) &udd_get_endpoint_fifo_access8(CDC_TX);
          // this costs a little time.

          while ( UOTGHS_DEVEPTISR_TXINI != (UOTGHS->UOTGHS_DEVEPTISR[CDC_TX] & UOTGHS_DEVEPTISR_TXINI ));
          //TXINI bit is cleared when we can load up a new packet.
#define TEMPLATE \
  "str r4, [%1,#56]\n\t" \
  "nop\n\t"\
  "str r0, [%1,#56]\n\t" \
  "IT NE\n\t" \
  "strbne %5, [%3], #1\n\t"\
  "nop\n\tnop\n\t" \
  "nop\n\tnop\n\t"\
  "str r6, [%1,#56]\n\t" \
  "ldr r5, [%2,#60]\n\t" \
  "tst %5, #256\n\t" \
  "str r2, [%1,#56]\n\t" \
  "IT NE\n\t" \
  "strbne r5, [%3], #1\n\t"\
  "nop\n\tnop\n\tnop\n\tnop\n\t"\
  "ldr %5, [%2,#60]\n\t"\
  "IT EQ\n\t"\
  "bleq empty\n\t"
          /* commented version, which the compiler barfs on now.
            #define TEMPLATE \
            "str r4, [%1,#56]\n\t"  \// raise R, lower OEA
            "nop\n\t" \// wait - might need more?
            "str r0, [%1,#56]\n\t"  \// lower R,
            "IT NE\n\t" \// if previous was valid, write previous.
            "strbne %5, [%3], #1\n\t"\
            "nop\n\tnop\n\t" \// wait - might need more?
            "nop\n\tnop\n\t"\
            "str r6, [%1,#56]\n\t" \// deassert DS
            "ldr r5, [%2,#60]\n\t" \// read new data
            "tst %5, #256\n\t" \// is previous valid?
            "str r2, [%1,#56]\n\t" \// assert DS, raise OEA
            "IT NE\n\t" \// if valid write it.
            "strbne r5, [%3], #1\n\t"\
            "nop\n\tnop\n\tnop\n\tnop\n\t"\
            "ldr %5, [%2,#60]\n\t" \// read second byte.
            "IT EQ\n\t"\
            "bleq empty\n\t"
          */
          // if data's not valid, jump to empty (below) to check for an interrupt.
          // it returns to the beginning of this block - to repeat the block.

          // low registers (r0-r7) can be 16 bit instructions, r8-r15 require ldrb.w strb.w
          //32 bit load stores might need to be 32 bit aligned for best speed.

          asm volatile( // all high is 33FE+8000+4000 4000 is R, 8000 is OEA
            "movw r4, #0x53FE\n\t" // same as r0,  R_ON
            "movw r0, #0x13FE\n\t" // CBITS + R_ON+WR+AS+R_ON  sequence is r4 r0  then r6 r2
            "movw r6, #0xA3FE\n\t"     // CBITS+WR+AS+DS or OEA) 1000 is AS, 2000 is DS
            "movw r2, #0x93FE\n\t" // CBITS+OEA+WR+AS 200 is WR, 1FE is CBITS
            "tst %5, #256\n\t" // 256 is the bit where empty arrives EMPTYB
            ".align 2\n\t"
            "doloop1:\n\t"  //now, my template 1+255 times:

            // first loop is different from the rest since we don't have a previous byte to write.
            "str r4, [%1,#56]\n\t"  // raise R, lower OEA
            "nop\n\t"  // pad to the same number of bytes as the others
            "str r0, [%1,#56]\n\t"  // lower R, lower OEA
            "nop\n\t"  // pad to the same number of bytes as the others
            "nop\n\tnop\n\tnop\n\t"
            "nop\n\t" // wait - might need more?
            "str r6, [%1, #56]\n\t" //raise DS
            "ldr r5, [%2,#60]\n\t" // read new data
            "tst %5, #256\n\t" // is previous valid?
            "str r2, [%1,#56]\n\t" // raise OEA, lower DS again.
            "IT NE\n\t" // if valid write it.
            "strbne r5, [%3], #1\n\t"
            "nop\n\tnop\n\tnop\n\tnop\n\t"
            "nop\n\tnop\n\t" // wait - might need more?
            "ldr %5, [%2,#60]\n\t" // read second byte.
            "IT EQ\n\t"
            "bleq empty\n\t" // if data's not valid, jump to empty (below) to check for an interrupt.
            // end of first loop

            TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE

            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE

            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE

            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE
            TEMPLATE TEMPLATE TEMPLATE TEMPLATE      TEMPLATE TEMPLATE TEMPLATE TEMPLATE

            "strb %5, [%3], #1\n\t"
            "b done\n\t"

            "empty:\n\t"
            // in here we'll check to see if a usb interrupt has arrived. UOTGHS_DEVEPTISRx.RXOUTI
            "ldr r12, [%4, #312]\n\t"
            "tst r12, #2\n\t"
            // now, if bit is set, then Z is clear (neq) then there is an interupt waiting
            // if there is, we need to close - do we need to finish up our packet?
            "bne cleanup\n\t"

            // return to our loop - have to return with zero flag set
            "movs r12, #0\n\t" // the s on movs updates flags.
            "sub lr,lr,#52\n\t"
            // the 52 here is the number of bytes in each template.
            "bx lr\n\t"


            "cleanup:\n\t"
            // in here we might need to write bytes to the fifo till its full, then get out.
            // hmm - should kind of set j big enough so we get out of our loop...
            // after releasing packet, check again if there an interrupt waiting.
            // if so, get out.
            "nop\n\t"

            "done:\n\t"
            "mov %0, %5\n\t" // copy the last read value in case there are more words to read.
            : "=l" (read2) //outputs - the last data read - need the flag for later.
            //      : "l" (PIOC), "l" (PIOD), "h" (ptr_dest), "h" (UOTGHS->UOTGHS_DEVEPTISR[CDC_RX]) // inputs
            : "l" (PIOC), "l" (PIOD), "h" (ptr_dest), "h" (UOTGHS), "l" (read2) // inputs
            : "r0", "r2",  "r5", "lr" , "r12", "r6", "r4"
          );

          /*
            // put ptr_dest in high register since stores are conditional, are going to be 32bit instructions anyway.
            r0, r6 and r2 are the words I write to portC
            %5 and r5 get the data read into them
            lr =r14 is used in empty call.
            r1, r4, r3, r11 are free for compiler.
            r7  is fp in thumb mode.
            r13 is stack pointer, r15 is PC
            r12 is what we'll use to check for USB RXOUTI flag.
            %0 - we copy the high byte with EMPTY flag into this on exit.
            %1 is PIOC
            %2 is PIOD
            %3 is where we write the data to for USB
            %4 is where we check to see if there is a USB interrupt waiting
            %5 is where we read the high byte and empty flag into.

            %1, %2, %5 chosen from r1, r3, r4, r7 (?)
            %3, %4 chosen from r8, r9, r10, r11

            r8, r9, r10 compiler picks one for ptr_dest, and one for USB interrupt pointer
            %0 = low is data read

            r0-r6 and r8-r12 are available.
          */

          // send the packet:
          UOTGHS->UOTGHS_DEVEPTICR[CDC_TX] = UOTGHS_DEVEPTICR_TXINIC;
          UOTGHS->UOTGHS_DEVEPTIDR[CDC_TX] = UOTGHS_DEVEPTIDR_FIFOCONC;

          // SerialUSB.write(buff, size);
          if (UOTGHS->UOTGHS_DEVEPTISR[CDC_RX] & UOTGHS_DEVEPTISR_RXOUTI)  // incoming packet waiting!
            break; // this gets us out of the j for loop
        } // end of j loop - fast packets over.
        // now check again for incoming packet interrupt... and handle it.
        cpu_irq_enable();
        nword = npts * 2 - npacket * 256; // words to go. Anywhere from 0 to 255
        j = 0;
        if (nword > 0) {
          ptr_dest = (uint8_t *) &udd_get_endpoint_fifo_access8(CDC_TX);
          while ( UOTGHS_DEVEPTISR_TXINI != (UOTGHS->UOTGHS_DEVEPTISR[CDC_TX] & UOTGHS_DEVEPTISR_TXINI ));
        }
        while (j < nword && SerialUSB.available() == 0) {
          // there are npts*2-npacket*256 16-bit words left to read.
          // read2 holds the previously read register - has the empty flag
          PIOC->PIO_ODSR = CBITS | WR | AS | R_ON; // raise R
          asm("nop\n\t");
          asm("nop\n\t");
          PIOC->PIO_ODSR = CBITS | WR | AS; // lower R
          asm("nop\n\t");
          asm("nop\n\t");
          asm("nop\n\t");
          asm("nop\n\t");
          read1 = PIOD->PIO_PDSR; //read
          PIOC->PIO_ODSR = CBITS | WR | AS | DS; // raise DS first
          asm("nop\n\t");
          asm("nop\n\t");
          asm("nop\n\t");
          asm("nop\n\t");

          PIOC->PIO_ODSR = CBITS | WR | AS | OEA ; // raise  OEA
          if (read2 & EMPTYB)
            *ptr_dest++ = read1 & 255;

          asm("nop\n\tnop\n\t");
          inc = PIOD->PIO_PDSR; //read
          if (read2 & EMPTYB) {
            *ptr_dest++ = inc & 255;
            j += 1;
          }
          read2 = inc; // holds flag for next time.
        } // end of while loop

        if (j > 0  || npacket % 2 == 1 ) { // if we wrote any words just here, send the packet.
          // also weirdness - if we exactly fill an odd number of packets, the last won't get sent
          // till next trigger: so do an extra whether we wrote more words or not.
          UOTGHS->UOTGHS_DEVEPTICR[CDC_TX] = UOTGHS_DEVEPTICR_TXINIC;
          UOTGHS->UOTGHS_DEVEPTIDR[CDC_TX] = UOTGHS_DEVEPTIDR_FIFOCONC;
        }

        // done sending points now.

        if (SerialUSB.available() != 0) {
          buff[0] = SerialUSB.peek();
          if (buff[0] == 'A' ) buff[0] = SerialUSB.read(); // if its A for abort, remove it from the buffer to not trigger another acquisition.
          // if its not, something is messed up!
        }
        digitalWrite(13, LOW);
        PIOC->PIO_ODSR = CBITS | OEA | WR | AS | DS | R_ON; // data bits high, strobes high, OEA and R idle, DS deasserted
        PIOC->PIO_MDDR = CBITS; // MDER is +80 - turn on drivers on data bits

        //     SerialUSB.println("Done");

        break; // end of case 'C'


    } // end of switch statement
  }
}



