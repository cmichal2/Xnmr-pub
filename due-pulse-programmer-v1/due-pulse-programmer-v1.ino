/*

Copyright (c) 2025 Carl Michal

This program is free software: you can redistribute it and/or modify it under
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
/* current pin assignments: (arduino pin numbers are D11 D12, SAM3X ports are D.7, D.8, C.1 etc)
    - output latch pin is D11 = D.7
    - timer trigger input is D12 = D.8 - a rising edge starts the timer at start of sequence
    - timer trigger source will be D2 = B.25 - used to trigger D.8
    - Put MCK/10 PWM output on D53 = PWMH2 = B.14
    - B.26 = D22 will be our external sync/trigger. To stop pulse program and sync to external hardware
    - port C, all available pins (25) are the output word.
      C1-9, 12-19, 21-26 and 28-30. These are:
      C.0 =           A.0 = D69 CANTX0
      C.1 = D33       A.1 = D68 CANRX0
      C.2 = D34       A.2 = A7 = D61
      C.3 = D35       A.3 = A6 = D60
      C.4 = D36       A.4 = A5 = D59
      C.5 = D37       A.5 =
      C.6 = D38       A.6 = A4 = D58
      C.7 = D39       A.7 = D31
      C.8 = D40       A.8 = D0 (RX0 no go?)
      C.9 = D41       A.9 = D1 (TX0 no go?)
      C.10 =          A.10 = D19
      C.11 =          A.11 = D18
      C.12 = D51      A.12 = D17
      C.13 = D50      A.13 = D16
      C.14 = D49      A.14 = D23
      C.15 = D48      A.15 = D24
      C.16 = D47      A.16 = A0 = D54
      C.17 = D46      A.17 = D70
      C.18 = D45      A.18 = D71
      C.19 = D44      A.19 = D42
      C.20 =          A.20 = D43
      C.21 = D9       A.21 = D73 (not easy)
      C.22 = D8       A.22 = A3 = D57
      C.23 = D7       A.23 = A2 = D56
      C.24 = D6       A.24 = A1 = D55
      C.25 = D5       A.25 = D74 (MISO)
      C.26 - D4/D87   A.26 = D75 (MOSI)
      C.27 =          A.27 = D76 (SCK)
      C.28 = D3       A.28 = D77/D10
      C.29 = D10/D77  A.29 = D87/D4
      C.30 = D72 (not easy)
      C.31 =

      pins B.17 -> B.20 aka A8-A11 or D62-D65 are analog inputs. Can read with 'a' command.
  C has 26 pins - C.30 not usable, and C.26/C.29 shared with A = 25
  A has 29 pins - A.21 not usable,A.28 and A.29 shared with C, and A.8/A.9 are RX/TX = 24

  That leaves some port D pins as well. And a handful of B unused too.

  pins D.0 -> D.3 (= D25-D28) are ID pins so you can identify
  which due board is which in a multiple board system.

*/
/*

  // in m15 taking a new tack. Configure a timer. Use it to produce short pulses. We'll use an external latch to use the short pulse
  // to clock out a new data word.
  // in m16 - move most execution to flash.
  // 16 seems to be in good shape!
  // 17 see if we can speed things up and make minimum event
  // shorter by unrolling the flash loop, and just calling
  // in to the unrolled loop at the right spot.

    In -16 we stored the number of events in an event queue, in
  17 all code is in flash as 3 unrolled loops that end with: start_loop, end_loop, or generic branch.
  Data controls how we go.

  in 18, move calculation of jump offsets onto the arduino. Code moved into local const variable, offsets to be calculated
  here after download.

  in m19, use wfi call to stall the cpu between events.
  in m19b try some fiddling with assembler events - use ldm instead of ldr twice, play with ordering.
  in m20, move word write to just after wfi to see if we can get by without latch!
  Nearly - seems to be +/- one clock cycle of play. Bummer.
  but reverse phase of latch so it can be used with latch.

  18-20 all use the same pulse-progs18.

  m21: add subroutines, and maybe wait for external event, and wait for external event with max time.
  uses new pulse-progs21 with subroutines.

  m22 - try to move from wfi to wfe. The idea being we might be able to do clean reset over USB.
  THe plan is: set priorities of ext trig and timer below BASEPRI so that they wouldn't actually cause interrupts.
  Leave interrupts enabled. So they never trigger interrupts. Then, with SEVONPEND, they will wake from WFE.

  Then, can use USB interrupt at high priority, to actually cause an interrupt. We can hijack the USB interrupt with
  UDD_SetStack.and take over the interrupt.
  wfe is bf20, wfi was bf30

  m23 - try to improve m22. In 22 we captured the usb interrupt, fixed up our outputs
  then rebooted. Here try instead to fix up the stack so we return and continue.
  Its tricky. We'll need to fiddle with the stacked return address, return, then restore
  the stack pointer to the right place, This works!

  m24 - issue with m23 is that we count back on the stack through registers saved at the
  beginning of my_usb_isr and of the uotghs_isr, and that is dangerous - depends on the compiler.
  Instead, lets rejig my code so it doesn't use the main stack pointer for loops and subroutines-
  we'll copy the sp to r9 so we are saving onto the stack, but pointing with r9 instead.
  - add dac outputs.

  m25 - clean up play_queue in pulse_progs25,
  m26 - clear up priorities and minimize how much is done at start stop.
  m27 - work on uploading next program during last event.
  m28 - see about external clock trigger - move main timer to TC2,2 = TC8.
  m29 try to fix up wait_for_trigger's. Seems like it fails if a second trigger comes while running.
  m30 - hack the USB reception so pulse program download is much faster.
*/
/* TODO:
  - checksum downloads
  - integrate with Xnmr. Pulse programmer compiler should optimize:
   for loops!
     - do a first pass to collapse events with identical outputs and
     different times.
     - second pass: start with first pair of events, see if second pair is same.
     if so, see if third pair is the same, etc.
     then start from 2nd instruction, 3rd instruction, etc.
     Then look for three event long duplicates,
     then four-event long duplicates, up to 1/2 of length-long duplicates.

  - error checking in host program - at least partly done.

*/
// default value written to ports A and C:
// #define DEFAULT_OUTPUT_C (1<<18)  // this was needed for BGU_NGI on pin 15 = C.18
#define DEFAULT_OUTPUT_C (0x0) // all pins idle low
// on C, the BGU's NGI bit idles high
#define DEFAULT_OUTPUT_A 0x00000000

// how many bytes in each event:
#define EVSIZE 12
// how many opcodes are there in our pulse programming language:
#define NUM_OPCODES 14
// the opcodes are:
#define START_LOOP 0
#define END_LOOP 1
#define BRANCH 2
#define EXIT 3
#define SUB_START 4
#define SUB_END 5
#define EXT_TRIG 6
#define TRIG_MAX 7
#define WRITE_DACS 8
#define WRITE_ALT 9
#define SWAP_TO_ALT 10
#define SWAP_TO_DEFAULT 11
#define SWAP_TO_DACS 12
#define WRITE_DEFAULT 13
#define CALL_SUB 255 // this is a fake one - no code associated.

// preassembled code. T0 is one event, T4 is 10,000 events.
// this is good: 9 ticks minimum! Best, I think
#define T0 0xca12, 0xbf20, 0x6381, 0x61ec, 0x6a29, 0x6077,
#define T1 T0 T0 T0 T0 T0 T0 T0 T0 T0 T0
#define T2 T1 T1 T1 T1 T1 T1 T1 T1 T1 T1
#define T3 T2 T2 T2 T2 T2 T2 T2 T2 T2 T2
#define T4 T3 T3 T3 T3 T3 T3 T3 T3 T3 T3

// we need EVSIZE bytes * 12000 *3 + 16+(loop start) 20 (loop end) +4 (generic branch)
// +2 (EXIT) + 10 (sub start) + 8 (SUB END)(EVSIZE was 16, now 12?) + 50 and 58 bytes for ext trig.
// + 10 for write dacs + 10 for write_a +6 for swap to alt and 6 for swap to default +8 for swap_to_dacs and 10 for write_default

// how many bytes to get to each opcode:
int code_lengths[NUM_OPCODES + 1] = {12000 * EVSIZE, 16 + 12000 * EVSIZE, 20 + 12000 * EVSIZE,
                                     4, 2, 10, 8, 50, 58, 10, 10, 6, 6, 8, 10
                                    };
// here is our preassembled code:
const uint16_t code[(EVSIZE * 12000 * 3 + 16 + 20 + 4 + 2 + 10 + 8 + 50 + 58 + 10 + 10 + 6 + 6 + 8 + 10) / 2] = {
  T4 T3 T3 0xf849, 0x3d04, 0xf852, 0x3b04, 0xf849, 0x2d04, 0xf852, 0xfb04, // loop start
  T4 T3 T3 0x3b01, 0xd003, 0xf8d9, 0x2000, 0xf852, 0xfb04, 0xe8b9, 0x000a, 0xf852, 0xfb04, // loop end
  T4 T3 T3 0xf852, 0xfb04,// generic branch
  0x4770,// exit
  0xca12, 0xf849, 0x2d04, 0x440a, 0x4720,// subroutine start
  0xf859, 0x2b04, 0xf852, 0xfb04, // subroutine end

  0xf04f, 0x0402, 0x602c, 0xf8d8, 0x404c, 0xf44f, 0x5180, 0x6031, 0xf04f, 0x6480,
  0xf8c8, 0x4040, 0xf04f, 0x0105, 0xbf20, 0x6029, 0xf8c8, 0x4044, 0xf8d8, 0x404c, 0xf44f, 0x5180, 0x6031, 0xf852, 0xfb04, // wait for external trigger

  0xf8d8, 0x404c, 0xf44f, 0x5180, 0x6031, 0xf04f, 0x6480, 0xf8c8, 0x4040, 0xf04f, 0x0432, 0xf04f, 0x0105, 0xbf20,
  0x61ec, 0x6029, 0xf04f, 0x6480, 0xf8c8, 0x4044, 0xf8d8, 0x404c, 0x6a2c, 0xf44f, 0x5180, 0x6031, 0x6077, 0xf852, 0xfb04,  // wait for ext trig with max.
  0xca02, 0xf8ca, 0x1020, 0xf852, 0xfb04, //write dacs,
  0xca02, 0xf8cb, 0x1038, 0xf852, 0xfb04, // write_alt : write word to port a outputs,
  0x4658, 0xf852, 0xfb04, // swap to alt
  0x4660, 0xf852, 0xfb04, // swap to default
  0xf1aa, 0x0018, 0xf852, 0xfb04, //swap to dacs
  0xca02, 0xf8cc, 0x1038, 0xf852, 0xfb04 // write_default
};

/* 12 repeated bytes are:
  in various orders,
    "ldmia r2!, {r1,r4}\n\t" ca12
    "str r1,[r0,#56]\n\t" 6381
    "wfe\n\t" bf20
    // store new timer RC
    "str r4,[r5,#28]\n\t" 61ec
    // clear match flag - feels like this should be later.
    "ldr r1,[r5,#32]\n\t" 6a29
    // clear irq
    "str r7, [r6,#4]\n\t" 6077

*/

/* 16 bytes for loop start are:
    // push any previous loop counter onto stack
   801ee: f849 3d04   str.w r3, [r9, #-4]!
   // load new loop counter from data
   801f2: f852 3b04   ldr.w r3, [r2],#4
   // save data pointer on stack
   801f6: f849 2d04   str.w r2, [r9, #-4]!
   // junk
   801fa: f852 fb04   ldr.w pc, [r2], #4

  20 bytes for loop end are
   801fe: 3b01        subs  r3, #1
   80200: d003        beq.n 8020a <continue>
   80202: f8d9 2000   ldr.w r2, [r9]
   80206: f852 fb04   ldr.w pc, [r2], #4
   8020a <continue>:
   8020a: e8b9 000a   ldmia.w r9!, {r1, r3}
   8020e: f852 fb04   ldr.w pc, [r2], #4


  4 bytes at end of generic branch are
  ldr.w pc, [r2],#4 f852 fb04

  2 bytes after that are:
  bx lr 4770
*/


/* subroutine start is: 10 bytes
   8016e:       ca12            ldmia   r2!, {r1, r4}
   //80170:       b404            push    {r2}
   801f0:  f849 2d04   str.w r2, [r9, #-4]!
   80172:       440a            add     r2, r1
   80174:       4720            bx      r4

   subroutine end is:
   //80176:       bc04            pop     {r2}
                f859 2b04   ldr.w r2, [r9], #4
   80178:       f852 fb04       ldr.w   pc, [r2], #4


  "wait_max2:\n\t"
    //76 is offset from PIOB to PIOB->PIO_ISR
    "ldr r4,[r8,#76]\n\t" f8d8 404c
    // clear pending on NVIC - PIOB is interrupt 12
    "mov r1,#0x1000\n\t" f44f 5180
    // r6 points at NVIC->ICPR[0],
    "str r1,[r6]\n\t" 6031
    // and enable the interrupt in PIOB->PIO_IER
    "mov r4,#0x4000000\n\t" f04f 6480
    "str r4, [r8,#0x40]\n\t" f8c8 4040
    // get ready to set Rc to 50, gives 1us delay after wakeup.
    "mov r4,#50\n\t" f04f 0432
    "mov r1,#5\n\t" f04f 0105
    "wfe\n\t" bf20
    // set Rc to 50
    "str r4,[r5,#28]\n\t"   61ec
    // and restart the timer:
    "str r1,[r5]\n\t"  6029
    //turn off the pin interrupt: PIOB->PIO_IDR
    "mov r4,#0x4000000\n\t" f04f 6480
    "str r4, [r8,#0x44]\n\t" f8c8 4044
    // clear pin interrupt in PIO_ISR
    //76 is offset from PIOB to PIOB->PIO_ISR
    "ldr r4, [r8, #76]\n\t"  f8d8 404c
    //also clear timer interrupt:
    "ldr r4,[r5,#32]\n\t"  6a2c
    // and clear the pending flags on both:
    // r6 points at NVIC->ICPR[0]
    "mov r1,#0x1000\n\t"  f44f 5180
    "str r1, [r6]\n\t" 6031
    // clear timer irq in NVIC->ICPR[0]
    "str r7, [r6,#4]\n\t"  6077
    // ok done.
    // do the next jump
    "ldr pc, [r2], #4\n\t"  f852 fb04

    write_dacs is:
    // load the dac data word
    "ldmia r2!, {r1}\n\t" ca02
    // write it to the dac
    "str r1,[r10,#0x20]\n\t" f8ca 1020
    // and jump
    "ldr pc,[r2],#4\n\t" f852 fb04

  "wait_for_external2:\n\t"
    // stop the timer: - it can't interrupt if its not running.
    "mov r4, #2\n\t" f04f 0402
    "str r4,[r5]\n\t" 602c
    // clear the trigger pin interrupt in PIOB->PIO_ISR r8 is a pointer to PIOB
    // clears on read.
    //76 is offset from PIOB to PIOB->PIO_ISR
    "ldr r4,[r8,#76]\n\t"  f8d8 404c
    // clear pending on NVIC - PIOB is interrupt 12
    "mov r1,#0x1000\n\t" f44f 5180
    "str r1,[r6]\n\t"   6031
    // and enable the interrupt in PIO_IER
    "mov r4,#0x4000000\n\t"  f04f 6480
    "str r4, [r8,#0x40]\n\t"  f8c8 4040
    // store a 5 for later, for restarting timer
    "mov r1,#5\n\t"  f04f 0105
    "wfe\n\t"  bf20
    // start the timer:
    "str r1, [r5]\n\t"   6029
    //turn off the pin interrupt: PIO_IDR
    "str r4, [r8,#0x44]\n\t"   f8c8 4044
    // clear it in the PIO_ISR
    //76 is offset from PIOB to PIOB->PIO_ISR
    "ldr r4, [r8, #76]\n\t"   f8d8 404c
    "mov r1,#0x1000\n\t"  f44f 5180
    // and clear the pending flag.
    "str r1, [r6]\n\t"  6031
    // r6 points at NVIC->ICPR[0]
    // ok done.
    // do the next jump
    "ldr pc, [r2], #4\n\t"   f852 fb04




  write_alt is:
  // load output word for port a
  "ldmia r2!,{r1}\n\t"
  // store in port a ODSR
    "str r1,[r11,#56]\n\t"
    // jump
    "ldr pc,[r2],#4\n\t"

    swap_to_alt:
    80190:       4658          mov     r0, fp
   80192:       f852 fb04       ldr.w   pc, [r2], #4

  swap_to_default:
  80196:       4660            mov     r0, ip
   80198:       f852 fb04       ldr.w   pc, [r2], #4

  swap_to_dacs:
  // move the DACC pointer into our pointer, but since we only offset it by 32 and the pio pointer gets
  // offset by 56, subtract 24 so the 56 offset writes to the right place.
  80186:       f1aa 0018       sub.w   r0, sl, #24
  018a:       f852 fb04       ldr.w   pc, [r2], #4

  write_default:
   8018e:       ca02            ldmia   r2!, {r1}
   80190:       f8cc 1038       str.w   r1, [ip, #56]   ; 0x38
   80194:       f852 fb04       ldr.w   pc, [r2], #4

*/

extern "C" char* sbrk(int incr);
int FreeRam() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}

// hold the downloaded program here:
#define MAX_DATA_WORDS 23003
unsigned int data[MAX_DATA_WORDS]; // enough for 12000 events, an inital address, a midpoint jump, and a final jump to exit

// copy of the stack pointer position from when the pulse program starts executing.
volatile int stashed_stack;
// volatile int debug;

volatile int status = 0;
#define STATUS_STOPPED 0
#define STATUS_EXECUTING 1
#define STATUS_FINAL_EVENT 2
#define STATUS_INTERRUPTED 3
#define STATUS_FINAL_TIMEOUT 4

extern void (*gpf_isr)(void); // this is what the CMSIS library thinks the USB interrupt handler is.
void (*saved_gpf_isr)(void);
volatile unsigned int prog_bytes;

void my_receive_usb_isr() {
  int i, n;
  uint8_t *ptr_src = (uint8_t *) &udd_get_endpoint_fifo_access8(CDC_RX);
  uint8_t *ptr_dest = (uint8_t *) data;
  irqflags_t flags = cpu_irq_save();


  if (Is_udd_endpoint_interrupt(CDC_RX)) {

    ptr_dest += prog_bytes;

    // we make lots of assumptions here!
    // assume that this is a CDC_RX interrupt
    // assume that the characters we want are at the start of the fifo.
    // assume we successfully read the whole fifo.

    udd_ack_out_received(CDC_RX);

    // How many bytes in fifo:
    n = ((UOTGHS->UOTGHS_DEVEPTISR[CDC_RX] & UOTGHS_DEVEPTISR_BYCT_Msk) >> UOTGHS_DEVEPTISR_BYCT_Pos);
    do {
      // copy them into our data buffer
      for (i = 0 ; i < n; i++)
        *ptr_dest++ = *ptr_src++;

      // release the buffer, since we emptied it.
      if (n > 0) {
        UOTGHS->UOTGHS_DEVEPTICR[CDC_RX] = UOTGHS_DEVEPTICR_TXINIC;
        UOTGHS->UOTGHS_DEVEPTIDR[CDC_RX] = UOTGHS_DEVEPTIDR_FIFOCONC;
      }
      prog_bytes += n;
      n = ((UOTGHS->UOTGHS_DEVEPTISR[CDC_RX] & UOTGHS_DEVEPTISR_BYCT_Msk) >> UOTGHS_DEVEPTISR_BYCT_Pos);
    } while (n > 0);
  }
  if (Is_udd_sof()) udd_ack_sof();
  if (Is_udd_endpoint_interrupt(0) || Is_udd_reset())
    saved_gpf_isr();
  //udd_ack_fifocon(CDC_RX);
  cpu_irq_restore(flags);
  return;


}
void my_abort_usb_isr() {
  /* This interrupt gets called if a USB interrupt is triggered while the pulse program is running.
      Done if you send a character over the USB interface.
  */
  // - stop timer,
  TC2->TC_CHANNEL[2].TC_CCR = 2;
  // - set outputs to defaults
  PIOC->PIO_ODSR = DEFAULT_OUTPUT_C;

  // reconfigure the latch pin so we can drive it directly.
  PIOD->PIO_OER = 1 << 7; // enable output
  PIOD->PIO_PER = 1 << 7; // PIO drives it

  // - force latch pulse - pin is D.11 = D7
  PIOD->PIO_CODR = 1 << 7; // set output low
  asm volatile ("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
  PIOD->PIO_SODR = 1 << 7; // set output high to latch.


  // now fix up the saved return address on the stack so when we exit, we don't
  // exit back into the program, but into the right spot to get out.

  // the plan is to set the stacked return address to the stacked LR.
  // The architecture ref manual (DUI0552A pg 2-26) says interrupt entry stacks: R0-R3,R12, LR, return address, PSR
  // What I want is to take the LR (that got stacked on interrupt entry) add 4, and stick
  // that into the return address.

  // ok. This works. Potential issue with stack alignment. If my stashed_stack was
  // only 4 byte aligned and the exception entry uses 8 byte aligned then I'm off by 4. Should be able to check for this though.
  // we can look at stashed_stack to see if its 4 or 8, and can check CCR to see if 8 bit is required
  // This appears to work:
  if (SCB->CCR & 0x200)
    if ( stashed_stack % 8 > 0) stashed_stack -= 4;

  // LR is 12 bytes down from the stashed stack, the return address is 8 bytes down.
  /* used for debugging purposes.
    asm volatile (
      // from our stashed stack pointer, go down 3 words
      "sub %1, #12\n\t"

      //debug:
      // copy the original return address into debug:
      "ldr %0,[%1,#4]\n\t"
      // now copy the thing the return address points to into debug
      "ldr %0,[%0]\n\t"
      // copy the saved r2 into debug
      "ldr %0,[%1,#-12]\n\t"


      // load the link address
      "ldr r1,[%1]\n\t"
      // and save it on top of the return address.
      "str r1, [%1,#4]\n\t"
      : "=r" (debug)
      : "h" (stashed_stack)
    );
  */
  asm volatile (
    // from our stashed stack pointer, go down 3 words
    "sub %0, #12\n\t"
    // load the link address
    "ldr r1,[%0]\n\t"
    // and save it on top of the return address.
    "str r1, [%0,#4]\n\t"
    :
    : "h" (stashed_stack)
  );


  //let the usb interrupt do its thing.
  saved_gpf_isr();

  //reset latch pin for the timer to drive:
  PIOD->PIO_PDR = 1 << 7; // peripheral drives it
  PIOD->PIO_ODR = 1 << 7; // disable output

  // if we interrupted during a wait_for_trigger event, the D22 = B.26 interrupt will be enabled, shut it off:
  PIOB->PIO_IDR = 1 << 26;
  // and make sure its cleared in the PIO_ISR and on the NVIC - PIOB is interrupt #12.
  int i = PIOB->PIO_ISR;
  NVIC->ICPR[0] = 0x1000; // could be 1 << PIOB_IRQn

  status += 1;
}

void TC8_Handler() {
  /* this is only called if we exited and don't restart in time,
     Even though we trigger these interrupts all the time during execution, they are
     normally blocked by the BASEPRI.

     If this interrupt happens, the final event is about 380 ns longer than requested.
  */
  int i;
  // stop timer
  TC2->TC_CHANNEL[2].TC_CCR = 2;
  // - set outputs to defaults
  PIOC->PIO_ODSR = DEFAULT_OUTPUT_C; //

  PIOD->PIO_CODR = 1 << 7; // set output low
  // reconfigure the latch pin so we can drive it directly.
  PIOD->PIO_OER = 1 << 7; // enable output
  PIOD->PIO_PER = 1 << 7; // PIO drives it

  // - force latch pulse - pin is D2 = B.25
  asm volatile ("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
  PIOD->PIO_SODR = 1 << 7; // set output high to latch.

  PIOD->PIO_PDR = 1 << 7; // peripheral drives it
  PIOD->PIO_ODR = 1 << 7; // disable output


  // make sure no matches pending:
  i = TC2->TC_CHANNEL[2].TC_SR;



  status = STATUS_FINAL_TIMEOUT;
}


void TC1_Handler()
{
  /* This is called at the start of every pulse sequence. We execute the whole thing inside
      this interrupt handler.
  */
  int junk;
  register int *fr0 asm("r0");
  register int *fr2 asm("r2");
  register int *fr5 asm("r5");
  register int *fr6 asm("r6");
  register int *fr8 asm("r8");
  register int *fr10 asm("r10");
  register int *fr11 asm("r11");
  register int *fr12 asm("r12");

  PIOB->PIO_SODR = 0x08000000; // turn LED on
  // clear the TC1 irq flag. The timer was configured to stop on Rc match.
  junk = TC0->TC_CHANNEL[1].TC_SR;


  // do prep stuff so we can be interrupted by USB and exit cleanly

  // turn off systick interrupts. This could be done once globally, but it interferes with
  // flashing new program over native port if its not enabled.
  SysTick->CTRL &= ~2;

  // Hijack the USB interrupt, so if it occurs we catch it and abort.
  saved_gpf_isr = gpf_isr;
  //  UDD_SetStack(&my_abort_usb_isr);
  gpf_isr = &my_abort_usb_isr; // same, but without func call

  // put this stack pointer into stashed_stack so our isr can find it.
  asm volatile("mov %0,sp\n\t"
               :"=h" (stashed_stack)
               :
               : );

  fr0 = (int*) PIOC; // put PIOC in r0 active output port
  fr2 = (int *) data; // data pointed to in r2
  fr5 = (int *) &TC2->TC_CHANNEL[2].TC_CCR; // base address of the timer.
  fr6 = (int *) NVIC; // 0x180 gets added to point to clear pending reg.
  fr8 = (int *) PIOB;
  fr10 = (int *) DACC;
  fr11 = (int *) PIOA; // alternate outputs
  fr12 = (int *) PIOC; // default output port
  asm volatile(
    // set priority so we don't trigger interrupts.
    "mov r1,#0x30\n\t"
    "msr BASEPRI,r1\n\t"
    // we'll put stuff on the stack, but don't mess with the sp.
    "mov r9,sp\n\t"
    // make sure event register is set
    "SEV\n\t"
    // then make sure it is cleared
    "wfe\n\t"
    // store the value we need to clear the TC8 irq - bit 3.
    "mov r7, 0x08\n\t"
    // point at the spot in NVIC
    "add r6, r6, #0x180\n\t"
    // start the timer.
    //    "mov r4, #5\n\t"
    //    "str r4, [r5]\n\t"
    // read status to clear any match flags
    "ldr r4, [r5, #32]\n\t"

    // run the pulse program
    // load the start address from the data.
    "ldr r1, [r2], #4\n\t"
    "blx r1 \n\t"
    // blx set the lr to point here. This 4 bytes

    // If a usb abort is received during execution, we end up here too
    // write 0 to port at end?! TODO should be default value
    // the interrupt already set the outputs to default values and
    // shut off the timer.
    "get_out:\n\t" // if we were interrupted, the timer is stopped. If not, the timer is still running, last event just started.
    // set interrupt priority to 0 - allows our timer to produce interrupts.
    "mov r1,#0x0\n\t"
    "msr BASEPRI,r1\n\t"
    // read status to clear any match flags
    "ldr r4, [r5, #32]\n\t"
    :
    : "l" (fr0), "l" (fr2), "l" (fr5), "l" (fr6), "h" (fr8), "h" (fr10), "h" (fr11), "h" (fr12)
    : "r1", "r3", "r4", "r7", "r9", "lr"); // clobbered in code

  /*
    r0 points at the port (PIOC).
    r1 is a scratch for loading
    r2 is our data pointer
    r3 is the runtime loop count
    r4 scratch for loading.
    r5 points to the timer for access to TC_SR and TC_RC
    r6 holds pointer to NVIC->ICPR[0]
    r7 holds the number needed to clear our IRQ for the timer
    r8 holds PIOB, which has the input trigger pin in it.
    r9 holds the thing that I use as my stack pointer.
    r10 points to DACC to write the dac values
    r11 points to PIOA // these last two are really wasted. Could just load them from
    r12 points to PIOC // immediate addresses.
  */

  PIOB->PIO_CODR = 0x08000000; // turn LED off
  // unhijack the usb isr.
  //UDD_SetStack(saved_gpf_isr);
  gpf_isr = saved_gpf_isr; // same, but without func call
  // turn back on systick interrupts. This isn't needed here, but if not done, it intereferes with
  // flashing new program over native port.
  SysTick->CTRL |= 2;

  status += 1; // if there was a usb interrupt, it added one too.
}

void checksum_data(unsigned char *c1, unsigned char *c2, int len, unsigned int *data) {
  // calculate checksums for the data. Based on what Bruker does in SBS.
  // its really lousy for long messages - it only notices differences at the beginning and end of the data.
  // But for our purposes that seems sort-of ok.
  unsigned int i, ch1 = 0, ch2 = 0;
  unsigned char *cdata;
  /* d0 d1 d2 d3 d4 d5 ... dn
      1  2  3  4  5  6 ...  n
     n+1 n n-1 n-2 ...      2

    for the first checksum, multiply top row elements by middle row elements. Sum, and keep low 8 bits.
    For the second, use the lower row.
  */
  cdata = (unsigned char *) data;
  /*
    for (i = 0; i < len * 4; i++) {
      ch1 += (i + 1) * cdata[i];
      ch2 += (len*4 + 1 - i) * cdata[i];
    } */
  for (i = 0; i < len; i++) {
    ch1 += (i + 1) * data[i];
    ch2 += (len + 1 - i) * data[i];
  }

  *c1 = ch1 & 0xff;
  *c2 = ch2 & 0xff;
}

void setup() {

  SerialUSB.begin(38400);
  // need to set all the other port C pins - get 26 outputs altogether.
  // C1-9, 12-19, 21-26 and 28-30.

  pinMode(13, OUTPUT); // led this is B.27

  analogReadResolution(12); // analog reads use full 12 bits.

  /////////////// configure port A and C pins as outputs
  // enable PIO control
  PIOA->PIO_PER = 0x0fdffcdf;
  // allow direct writes to all pins
  PIOA->PIO_OWER = 0x0fdffcdf;
  // set the outpus
  PIOA->PIO_ODSR = DEFAULT_OUTPUT_A;
  // and enable outputs
  PIOA->PIO_OER = 0x0fdffcdf; //24 bits

  PIOC->PIO_PER = 0x37eff3fe;
  PIOC->PIO_OWER = 0x37eff3fe;
  PIOC->PIO_ODSR = DEFAULT_OUTPUT_C;
  PIOC->PIO_OER = 0x37eff3fe; //25 bits


  // force a latch pulse on the latch pin:
  // reconfigure the latch pin so we can drive it directly.
  PIOD->PIO_OER = 1 << 7; // enable output
  PIOD->PIO_PER = 1 << 7; // PIO drives it
  // - force latch pulse - pin is D.11 = D7
  PIOD->PIO_CODR = 1 << 7; // set output low
  asm volatile ("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
  PIOD->PIO_SODR = 1 << 7; // set output high to latch.
  //reset latch pin for the timer to drive:
  PIOD->PIO_PDR = 1 << 7; // peripheral drives it
  PIOD->PIO_ODR = 1 << 7; // disable output


  //////////////// set up external trig pin for pulse program use - can stop pulse program till rising pulse.
  // this is D22, B.26
  pmc_enable_periph_clk(ID_PIOB);
  PIO_Configure(PIOB, PIO_INPUT, 1 << 26, PIO_PULLUP | PIO_DEGLITCH);
  //PIO_Configure(PIOB, PIO_INPUT, 1 << 26, PIO_PULLUP );
  PIOB->PIO_AIMER = 1 << 26; // enable additional modes
  PIOB->PIO_ESR = 1 << 26; // on edges
  PIOB->PIO_REHLSR = 1 << 26; // rising edge.
  PIOB->PIO_IDR = 1 << 26; // disable interrupt on the pin. for now. Enabled on the fly.
  PIOB->PIO_IDR = 0xffffffff;
  int i =  PIOB->PIO_ISR; // clear the flag if its set
  NVIC_DisableIRQ(PIOB_IRQn); // don't need to enable the irq on NVIC. WFE/SEVONPEND doesn't need it to be enabled!

  // pin interrupts trigger two things: PIOB->PIO_ISR gets the pin bit set, and
  // and the pending flag gets set on the NVIC. Clear the bit in PIO_ISR first
  // and then the NVIC.

  // to enable the pin interrupt we should: clear the PIO_ISR, clear any pending irq, then enable the irq.
  // to disable: disable the irq, clear the PIO_ISR, and clear any pending.
  // this is irq 12 - interrupt set enable is NVIC base. Interrupt clear enable is base+ 0x80
  // clear pending is base + 0x180.


  //////////////// Configure a 10 MHz PWM output on pin 53
  //set a pwm on pin 53 = PWMH2 = B.14
  // for other pins, like D.53 = B.14 = PWMH2 need to do more work ourselves
  int chan = 2;
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  // on B.14, timer output is periph_B
  PIO_Configure(PIOB, PIO_PERIPH_B, 1 << 14, 0);
  //  PWMC_SetPeriod(PWM_INTERFACE, chan, 2); // chan = 2 // for 10 MHz with 20 MHz clka
  //  PWMC_SetDutyCycle(PWM_INTERFACE, chan, 1);

  // 10MHz/16.
  PWMC_SetPeriod(PWM_INTERFACE, chan, 2 * 16); // chan = 2 //
  PWMC_SetDutyCycle(PWM_INTERFACE, chan, 1 * 16);

  PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, 0, 0);
  PWMC_EnableChannel(PWM_INTERFACE, chan);
  PWMC_ConfigureClocks(2000000 * 10 , 0, 100000000); // ask for 20 MHz clock with 100 MHz mck


  ///////////////// Configure our main timer
  // our latch output is on pin 11 = PD7.
  // set up TC2, channel 2 for our output clock, use A8 - PD7, peripheral B
  /* turn on the timer clock in the power management controller */
  pmc_set_writeprotect(false);       // disable write protection for pmc registers
  pmc_enable_periph_clk(ID_TC8);     // enable peripheral clock TC8
  // wavesel mode, match at RC and reset
  // use clock1 = MCLK/2
  // set external events from TIOB, external events reset counter and start clock, on rising edge.
  // latch pin set high on RA match, low on RC match.
  //
  TC_Configure(/* clock */TC2,/* channel */2, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1 |
                          TC_CMR_EEVT_TIOB | TC_CMR_ENETRG | TC_CMR_EEVTEDG_RISING | TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR);

  TC_SetRC(TC2, 2, 999); // duration of event will be RC
  TC_SetRA(TC2, 2, 6); // output is turned off at RC match, set at RA match, 6 ticks later. Need at least 6 for "write after wfi" strategy. Leaves 20-40ns setup time. 7 might be better.

  // enable irqs for us:
  TC2->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;

  // enable timer interrupt - this has to be enabled so it fires after the final event is complete
  NVIC_EnableIRQ(TC8_IRQn);


  ///////////////Configure timer controlled output pin, use for latching.
  // pin for timer controlled latch ticks: This is D.7, arduino D11
  // this will force the latch pin low for now.
  PIO_Configure(PIOD, PIO_PERIPH_B, 1 << 7, 0);


  ///////////////Configure external trigger pin to start our timer, useful for chaining boards together.
  // the trigger pin is D.8 = D12, TIOB8.
  PIO_Configure(PIOD, PIO_PERIPH_B, 1 << 8, PIO_PULLUP);


  ////////////////Configure pin to act as the external trigger for the timer. Pick D2 = B.25.
  PIO_Configure(PIOB, PIO_OUTPUT_1, 1 << 25, 0);


  // configure a second timer to give us an interrupt.  it stops on Rc match
  pmc_enable_periph_clk(ID_TC1);
  TC_Configure(TC0, 1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_CPCSTOP);
  TC_SetRC(TC0, 1, 5); // duration of event will be RC
  TC0->TC_CHANNEL[1].TC_IER = TC_IER_CPCS; // enable interrupt on Rc match.
  NVIC_EnableIRQ(TC1_IRQn); // enable interrupts on the dummy timer


  // overclock:? 84MHz
  // looks like the freq is 12MHz/(2*DIVA)*(MULA+1) so should get 100 MHz with DIVA 3 and MULA 49
  // the PRES_CLK_2 divides by 2 below.
  // proba
  // standard 84 MHz:
  //#define SYS_BOARD_PLLAR (CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(13UL) | CKGR_PLLAR_PLLACOUNT(0x3fUL) | CKGR_PLLAR_DIVA(1UL))
  // 80 MHz. 12 /3 * 20 =80
  //#define SYS_BOARD_PLLAR (CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(19UL) | CKGR_PLLAR_PLLACOUNT(0x3fUL) | CKGR_PLLAR_DIVA(2UL))
  //100 MHz 12 / 3/2 * 25 = 100
#define SYS_BOARD_PLLAR (CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(49UL) | CKGR_PLLAR_PLLACOUNT(0x3fUL) | CKGR_PLLAR_DIVA(3UL))
#define SYS_BOARD_MCKR ( PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK)
  // alternate 100 MHz
  //#define SYS_BOARD_PLLAR (CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(24UL) | CKGR_PLLAR_PLLACOUNT(0x3fUL) | CKGR_PLLAR_DIVA(3UL))
  //#define SYS_BOARD_MCKR ( PMC_MCKR_PRES_CLK | PMC_MCKR_CSS_PLLA_CLK)
  //Set FWS according to SYS_BOARD_MCKR configuration
  EFC0->EEFC_FMR = EEFC_FMR_FWS(4); //4 waitstate flash access
  EFC1->EEFC_FMR = EEFC_FMR_FWS(4);
  // Initialize PLLA
  PMC->CKGR_PLLAR = SYS_BOARD_PLLAR;
  while (!(PMC->PMC_SR & PMC_SR_LOCKA));
  //PMC->PMC_MCKR = SYS_BOARD_MCKR;
  PMC->PMC_MCKR = (PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK);
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY));

  SystemCoreClockUpdate();

  /////////////////////configure DACs pins DAC0 and DAC1, B.15 and B.16.
  pmc_enable_periph_clk(DACC_INTERFACE_ID); // enable the clock
  dacc_reset(DACC_INTERFACE); // reset the dac
  dacc_set_transfer_mode(DACC_INTERFACE, 1); // 1 is full word, 0 is half word
  dacc_set_power_save(DACC_INTERFACE, 0, 0); // sleep mode, fast wakeup both off
  dacc_set_timing(DACC_INTERFACE, 0x08, 0, 0x10); // refresh time, max speed mode (off) and startup time.
  dacc_set_analog_control(DACC_INTERFACE, DACC_ACR_IBCTLCH0(0x02) | DACC_ACR_IBCTLCH1(0x02) | DACC_ACR_IBCTLDACCORE(0x01));
  dacc_enable_flexible_selection(DACC_INTERFACE); //set tag mode so a single 32 bit write will set both dacs
  dacc_enable_channel(DACC_INTERFACE, 0); // these take over the pins. No PIO_Configure needed.
  dacc_enable_channel(DACC_INTERFACE, 1);

  // write both dacs to 0:
  // we write a single word and set both dacs in one go.
  // One half word has to have bit 12 to set dac1.
  while ((DACC->DACC_ISR & DACC_ISR_TXRDY) == 0);
  // set both dacs to 0:
  DACC->DACC_CDR = 1 << 28;

  //////////////// Configure D0-D.3 as ID input pins
  pmc_enable_periph_clk(ID_PIOD);
  PIOD->PIO_PER |= 0xf; // enable pio control
  PIOD->PIO_PUER |= 0xf; // enable pullups.

  //////////////////// interrupt priority and the like
  NVIC_SetPriority(TC8_IRQn, 6);  // this is our main running timer
  NVIC_SetPriority(PIOB_IRQn, 7); // this is for the external trigger. Priority is irrelevant since its never enabled.
  NVIC_SetPriority(TC1_IRQn, 2); // this is the bogus timer we use to get into the interrupt handler
  // only other relevant isr is the usb, left its priority at 0.
  // set BASEPRIO to 3, so our interrupts don't actually cause interrupts.

  // and set SEVONPEND and DISMCYCINT - sends event on interrupt and prevents interruption of LDM/STM
  SCB->SCR |= (0x10 | 1);

  // now that leaves irqs with priority higher than 4 ( ie 0-3) will still be triggered.
  // for WFI/WFE:
  // turn off systick interrupts. Don't do it here. I think it messes with usb native flashing.
  //SysTick->CTRL &= ~2;


}

// this is how long we will wait between receiving USB packets while downloading a pulse program.
// Its measured in SysTick interrupts (1ms)
#define TIMEOUT_COUNT 1000

void loop() {
  int bytes_rec, last_prog_bytes, pin;
  unsigned int dac_val, timeout_counter;
  char prog_downloaded = 0;
  unsigned char c1, c2;
  while (1) {

    unsigned char buff[2];
    unsigned int  dlen, i;
    unsigned int base_addrs[NUM_OPCODES], elements, inst;
    // shut off cpu while we wait for a command.
    // put in a loop because the systimer will cause interrupts.
    while (SerialUSB.available() == 0)
      asm volatile ("wfi\n\t");

    buff[0] = SerialUSB.read();
    switch (buff[0]) {
      case 'Q': // query
        SerialUSB.println("Due pulse programmer v1");
        break;
      case 'A': // analog - set dac values
        dac_val = SerialUSB.read();
        dac_val |= (SerialUSB.read() << 8);
        dac_val |= (SerialUSB.read() << 16);
        dac_val |= (SerialUSB.read() << 24);
        // if (status == STATUS_FINAL_EVENT)
        // SerialUSB.println("No");
        //else {
        DACC->DACC_CDR = dac_val;
        SerialUSB.println("OK");
        break;
      //}
      case 'a': // analog read - read an adc
        {
          int val1, val2, val3, spare, i;

          pin = SerialUSB.read(); // which channel? one byte available pins are A8-A11 which are D62-D65 or B.17-B.20
          // so we should send 62, 63, 64, or 65
          if (pin < 62 || pin > 65) SerialUSB.println("0");
          else {
            val1 = analogRead(pin);
            val2 = analogRead(pin);
            val3 = analogRead(pin);
            // want to discard low and high, send middle
            for (i = 0; i < 2; i++) {
              if (val1 > val2) {
                spare = val1;
                val1 = val2;
                val2 = spare;
              }
              if (val2 > val3) {
                spare = val2;
                val2 = val3;
                val3 = spare;
              }
            }
            SerialUSB.println(val2);
          }
        }
        break;
      case 'P': // set alternate port values
        dac_val = SerialUSB.read();
        dac_val |= (SerialUSB.read() << 8);
        dac_val |= (SerialUSB.read() << 16);
        dac_val |= (SerialUSB.read() << 24);
        //  if (status == STATUS_FINAL_EVENT)
        //SerialUSB.println("No");
        //else {
        PIOA->PIO_ODSR = dac_val;
        SerialUSB.println("OK");
        //        }
        break;
      case 'D': //download program.  first 2 bytes are length of data. followed by data.
        dlen = SerialUSB.read();
        dlen += (SerialUSB.read() << 8);
        // dlen is in 32-bit words;
        if (dlen >= MAX_DATA_WORDS) {
          SerialUSB.print(dlen);
          SerialUSB.println(" too big");
          break;
        }

        // before we say good to go, hack up the usb_isr so we do it ourselves.
        prog_bytes = 0;

        // hijack the usb isr:
        saved_gpf_isr = gpf_isr;
        gpf_isr = &my_receive_usb_isr;
        SerialUSB.print(dlen);
        SerialUSB.println(" size ok");
        // do all the receive in the isr.
        do {
          timeout_counter = 0;
          last_prog_bytes = prog_bytes;
          while (last_prog_bytes == prog_bytes && timeout_counter < TIMEOUT_COUNT) {
            timeout_counter += 1;
            asm volatile("wfi\n\t"); // this gets woken by a systick interrupt and usb interrupt. - seems to be about once/ms. SO here - 1/2 second.
          }
          if (prog_bytes != dlen * 4 && timeout_counter != TIMEOUT_COUNT && (prog_bytes % 512) == 0) {
            SerialUSB.println(prog_bytes);// acknowledge each packet (except final) to our client.
          }
        } while (prog_bytes < dlen * 4 && timeout_counter < TIMEOUT_COUNT);

        // unhijack the USB isr.
        gpf_isr = saved_gpf_isr;

        /*
                //divide read in 512 byte chunks, acknowledge each with an ok.
                bytes_rec = 0;
                while (bytes_rec < ((dlen / 128) * 512)) {
                  new_bytes = SerialUSB.readBytes((char *) & (data[bytes_rec / 4]), 512);
                  bytes_rec += new_bytes;
                  if (new_bytes < 512) {// probably something is wrong. Shouldn't get just one byte...
                    break;
                  }
                  //SerialUSB.println("ok");
                  SerialUSB.println(bytes_rec);
                }
                if (bytes_rec < dlen * 4) {
                  new_bytes = SerialUSB.readBytes((char*) & (data[bytes_rec / 4]), dlen * 4 - bytes_rec);
                  bytes_rec += new_bytes;
                }
        */
        //bytes_rec = SerialUSB.readBytes((char *) data, dlen * 4);
        if (prog_bytes == dlen * 4) {
          // should checksum data.
          checksum_data(&c1, &c2, dlen, data);
          SerialUSB.print(c1);
          SerialUSB.print(" ");
          SerialUSB.print(c2);
          SerialUSB.println(" data received");
          prog_downloaded = 1;
        }
        else {
          SerialUSB.print("  ");
          SerialUSB.print(prog_bytes);
          SerialUSB.println("bytes: data incomplete");
          prog_downloaded = 0;
        }
        // We need to resolve the all the jumps.
        base_addrs[0] = (int) code + code_lengths[0];
        for (i = 1; i < NUM_OPCODES; i++)
          base_addrs[i] = base_addrs[i - 1] + code_lengths[i];

        i = 0;
        while ( i < dlen) {
          inst = data[i] >> 16;
          elements = data[i] & 0xffff;
          if (inst == CALL_SUB) {
            data[i] = base_addrs[SUB_START] + 1;
            inst = data[i + 2] >> 16;
            elements = data[i + 2] & 0xffff;
            data[i + 2] = base_addrs[inst] - EVSIZE * elements + 1;
            i += 3;
          }
          else {
            data[i] = base_addrs[inst] - EVSIZE * elements + 1; // +1 for bx.
            i += 1; // advance past the address we just resolved.
            i += 2 * elements; // advance past the events
            if (inst == START_LOOP || inst == WRITE_DACS || inst == WRITE_ALT || inst == WRITE_DEFAULT) i += 1; // for start loop or write_dacs, advance past the argument.

          }
        }/*
         // debugging write back resolved program (and base addrs).
                for (i = 0; i < NUM_OPCODES; i++)
                  SerialUSB.println(base_addrs[i],HEX);
                  SerialUSB.println();
                for (i = 0; i < dlen; i++)
                  SerialUSB.println(data[i],HEX);
*/
        break;
      case 'E': //execute the program - should check that there is one! TODO
      case 'e':
        if ( status == STATUS_FINAL_EVENT ) {
          SerialUSB.println("Use R for restart");
          break;
        }
        if (prog_downloaded == 0) {
          SerialUSB.println("no program");
          break;
        }
        PIOC->PIO_ODSR = DEFAULT_OUTPUT_C; // write them all low for now. - this will get latched on first time after power-up only.
        if (TC2->TC_CHANNEL[2].TC_CV == 2500) // ensure we don't cause a match. - gives 50 us to get started
          TC_SetRC(TC2, 2, 2499); // duration of pre-event will be RC
        else
          TC_SetRC(TC2, 2, 2500); // duration of pre-event will be RC

        SerialUSB.println("Starting");
        status = STATUS_EXECUTING;
        if (buff[0] == 'E') {
          TC2->TC_CHANNEL[2].TC_CCR = 1; // enable the timer, but don't start it. The external trigger will start it.
          // to actually start the timer we need an external trigger, provided here:
          // need to hook pin D2 (which produces the trigger pulse) up to D12, which will start the timer.
          PIOB->PIO_CODR = 1 << 25;
          PIOB->PIO_CODR = 1 << 25;
          PIOB->PIO_SODR = 1 << 25;
        }
        else if (buff[0] == 'e')
          TC2->TC_CHANNEL[2].TC_CCR = 5; // enable the timer, and start it.

        TC_Start(TC0, 1); // start the dummy timer
        while (status == STATUS_EXECUTING);

        if (status == STATUS_INTERRUPTED)
          SerialUSB.println("Was interrupted");
        /*{
          SerialUSB.print("Was interrupted: ");
          SerialUSB.print(debug);
          SerialUSB.print(" ");
          SerialUSB.println((int) data);
          }*/
        else
          SerialUSB.println("Final Event started");
        break;
      case 'R': //restart the program - assume we left a program running and will now resume.
        if (prog_downloaded == 0) {
          SerialUSB.println("no program");
          break;
        }
        if (status == STATUS_FINAL_EVENT && TC2->TC_CHANNEL[2].TC_RC - TC2->TC_CHANNEL[2].TC_CV > 2500) {
          SerialUSB.println("Restarting");
          status = STATUS_EXECUTING;
          TC_Start(TC0, 1); // start the dummy timer
          while (status == STATUS_EXECUTING);
          if (status == STATUS_INTERRUPTED)
            SerialUSB.println("Was interrupted");
          else
            SerialUSB.println("Final Event started");
        }
        else SerialUSB.println("Too late");
        break;
      case 'S': // request status
        switch (status) {
          case STATUS_STOPPED:
            SerialUSB.println("Status stopped");
            break;
          case STATUS_EXECUTING:
            SerialUSB.println("status executing - can't happen!");
            break;
          case STATUS_FINAL_EVENT:
            bytes_rec = TC2->TC_CHANNEL[2].TC_RC - TC2->TC_CHANNEL[2].TC_CV;
            SerialUSB.print("status final event: ");
            SerialUSB.print(bytes_rec);
            SerialUSB.println(" ticks remain");
            break;
          case STATUS_FINAL_TIMEOUT:
            SerialUSB.println("status final_timeout");
            status = STATUS_STOPPED;
            break;

        }
        break;
      case 'K':
        if (status == STATUS_INTERRUPTED) {
          status = STATUS_STOPPED;
        }

        else if (status == STATUS_FINAL_EVENT) {
          TC8_Handler(); // treat as though the event finished.
          status = STATUS_STOPPED;
          SerialUSB.println("Final event killed");
        }
        else
          SerialUSB.println("Got K");
        break;
      case 'I': // return id value from pins D0-D3 (D25-28)
        SerialUSB.println(PIOD->PIO_PDSR & 0xf );
        break;
      case 10: //CR and NL
      case 13:
        break;
      default:
        SerialUSB.print("Got unknown command: ");
        SerialUSB.println(buff[0]);
        break;
    }

    // there is an initial 1000 tick delay before the sequence starts. Could shorten that.
    // the last event is not waited for - unless we wait for it I guess. But seems pointless to wait?

  }
}

