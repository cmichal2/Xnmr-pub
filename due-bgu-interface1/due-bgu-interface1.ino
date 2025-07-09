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
/*
    To Compile this, you have to comment out the PIOB_Handler function in WInterupts.c in cores/arduino/

    Idea is:
    Every packet has a first byte header that is:
    S - request status
    D , then 1 byte that says how many gradient triples there are in the packet. Max is 42
    they start after the fourth byte.
    R - reset data pointers to throw away any buffered data. Also reset the zero_counter
    Q - query, respond with id string.
    Z - write zeros to the bgu and hit the NGI

    response to S is 2 bytes that specify how much data is left in the buffer, then two bytes
    that say how many times we've written zero to the bgu since last reset.

    reponse to R is OK

    response to D comes only when there is room for more data in the buffer. It is: GO.

    So on start-up, do:
    R <OK>
  loop:
    D [data]  <OK>

    can S at end

    interrupt with
    R <OK>
    Z <OK>

   That's all in 'Running mode' also have 'idle mode'
   send an E to get into running mode and K to get out.
   The only reason to have idle mode is that SYSTICK is disabled in running mode, and flashing over native usb won't work
   if systick is disabled.

    The NGI pin should be set up as open drain here, and connected to the pulse programmer through a resistor (1k?). So that we can pull
    it low after a Z command.


    current pin assignments: (arduino pin numbers are D11 D12, SAM3X ports are D.7, D.8, C.1 etc)
    Bruker pins are D00-D15, A00 - A04 etc

        C.0 =           A.0 = D69 CANTX0
  D00   C.1 = D33       A.1 = D68 CANRX0
  D01   C.2 = D34       A.2 = A7 = D61
  D02   C.3 = D35       A.3 = A6 = D60
  D03   C.4 = D36       A.4 = A5 = D59
  D04   C.5 = D37       A.5 =
  D05   C.6 = D38       A.6 = A4 = D58
  D06   C.7 = D39       A.7 = D31
  D07   C.8 = D40       A.8 = D0 (RX0 no go?)
  D08   C.9 = D41       A.9 = D1 (TX0 no go?)
        C.10 =          A.10 = D19
        C.11 =          A.11 = D18
  D09   C.12 = D51      A.12 = D17
  D10   C.13 = D50      A.13 = D16
  D11   C.14 = D49      A.14 = D23
  D12   C.15 = D48      A.15 = D24
  D13   C.16 = D47      A.16 = A0 = D54
  D14   C.17 = D46      A.17 = D70
  D15   C.18 = D45      A.18 = D71
  A00   C.19 = D44      A.19 = D42
        C.20 =          A.20 = D43
  A01   C.21 = D9       A.21 = D73 (not easy)
  A02   C.22 = D8       A.22 = A3 = D57
  A03   C.23 = D7       A.23 = A2 = D56
  NGI** C.24 = D6       A.24 = A1 = D55
  DAS   C.25 = D5       A.25 = D74 (MISO)
  GDTR  C.26 - D4/D87   A.26 = D75 (MOSI)
        C.27 =          A.27 = D76 (SCK)
  WRS   C.28 = D3       A.28 = D77/D10
        C.29 = D10/D77  A.29 = D87/D4
        C.30 = D72 (not easy)
        C.31 =

BGU connector: looking at cable, (female side) with notch up, pin 1 is upper left, pin 2 is below. Evens on lower row.

Pin 2 is NGI, pin 4: N/C pin 6 DAS, pin 8 GDTR, pins 12-18 A00 -> A03, pins 20-5 D00 -> D15.
pin 10 is WRS

C has 26 pins - C.30 not usable, and C.26/C.29 shared with A = 25
  A has 29 pins - A.21 not usable,A.28 and A.29 shared with C, and A.8/A.9 are RX/TX = 24

  That leaves some port D pins as well. And a handful of B unused too.

  For BGU use port C for data and address - 20 bits in total, and also for strobes.

  D22, B.26 is the input trigger for us to write to the bgu
*/
#define MODE_RUNNING 1
#define MODE_IDLE 0

#define RESPONSE_ID "Due BGU interface v1"

#define MAX_DATA (6144*3)
//volatile unsigned int rpoint = 0, wpoint = 0; // point to where we read and write in the program buffer.
register uint32_t rpoint asm ("r5");
register uint32_t wpoint asm ("r6");

uint32_t data[MAX_DATA]; // 6144 events with three gradients each 73728 bytes
register uint32_t *rdata asm ("r4");
volatile int mode = MODE_IDLE;

extern void (*gpf_isr)(void); // this is what the CMSIS library thinks the USB interrupt handler is.
void (*saved_gpf_isr)(void);
volatile unsigned int zero_count = 0;

volatile int need_ack = 0;

#define NGI (1<<24)
#define DAS (1<<25)
#define GDTR (1<<26)
#define WRS (1<<28)
#define A0 (1<<19)
#define A1 (1<<21)
#define A2 (1<<22)
#define A3 (1<<23)

void return_status() {
  uint16_t b1, b2;
  // returns four bytes:
  // 2 bytes for how much data is left in buffer - 4 byte words.
  // then 2 bytes for how many zero counts we've written
  if (wpoint >= rpoint)
    b1 = wpoint - rpoint ;
  else
    b1 = wpoint + MAX_DATA - rpoint;
  b2 = zero_count;

  SerialUSB.write((char *)&b1, 2);
  SerialUSB.write((char *)&b2, 2);
}

void write_zeros() {

  // ensure everybody starts high:
  PIOC->PIO_SODR = 0x37eff3fe;
  asm volatile("nop\n\tnop\n\t");
  // address 0, and data 0:
  PIOC->PIO_CODR = ( A0 | A1 | A2 | A3 | 0x7f3fe) ;
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

  // lower strobes:
  PIOC->PIO_CODR = (DAS | WRS);
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

  // raise strobes
  PIOC->PIO_SODR = (DAS | WRS);
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

  // address 1:
  PIOC->PIO_SODR = (A0); // address 1.
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

  // lower strobes:
  PIOC->PIO_CODR = (WRS | DAS);
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

  // raise strobes
  PIOC->PIO_SODR = (WRS | DAS);
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

  // address 2:
  PIOC->PIO_SODR = (A1); // address 2.
  PIOC->PIO_CODR = (A0); // clear address 1
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

  // lower strobes:
  PIOC->PIO_CODR = (WRS | DAS);
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

  // raise strobes
  PIOC->PIO_SODR = (WRS | DAS);
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

  // and B0 as well
  PIOC->PIO_SODR = (A0 | A1 | A2 | A3); // address F
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

  // lower strobes:
  PIOC->PIO_CODR = (WRS | DAS);
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

  // raise strobes
  PIOC->PIO_SODR = (WRS | DAS);
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

  // leave everything high when done.
  PIOC->PIO_SODR = 0x37eff3fe;

  // here, hit NGI!
  PIOC->PIO_CODR = NGI;
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  PIOC->PIO_SODR = NGI;
}

void PIOB_Handler() {
  // declaring our own handler should be quicker than going through the arduino handler.
  // no needing to decode which pin was set, we know already, and stack is handled in hardware
  // on interrupt entry.
  // Even faster with data, rpoint and wpoint all as register variables!
  // Its about 345 ns from interrupt trigger till our first write appears on the outputs.
  int i, room;
  // write the outputs to the BGU:

  PIOC->PIO_CODR = rdata[rpoint]; // strobes are all set high and address high in the word before we start
  asm volatile("nop\n\tnop\n\tnop\n\t");

  // lower strobes:
  PIOC->PIO_CODR = (DAS | WRS);
  //asm volatile("nop\n\tnop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");

  // raise strobes
  PIOC->PIO_SODR = (DAS | WRS);
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

  // address 1:
  //PIOC->PIO_SODR = 0x37eff3fe;
  PIOC->PIO_SODR = rdata[rpoint];
  PIOC->PIO_CODR = rdata[rpoint + 1] ; // strobes and address set in word.
  asm volatile("nop\n\tnop\n\tnop\n\t");

  // lower strobes:
  PIOC->PIO_CODR = (WRS | DAS);
  //asm volatile("nop\n\tnop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");

  // raise strobes
  PIOC->PIO_SODR = (WRS | DAS);
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

  // address 2:
  //PIOC->PIO_SODR = 0x37eff3fe;
  PIOC->PIO_SODR = rdata[rpoint + 1];
  PIOC->PIO_CODR = rdata[rpoint + 2] ; // A1 set.
  // asm volatile("nop\n\tnop\n\t");

  // lower strobes:
  PIOC->PIO_CODR = (WRS | DAS);
  asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");

  // raise strobes
  PIOC->PIO_SODR = (WRS | DAS);
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");

/*
/// TEMPORARY
// here, hit NGI!
  PIOC->PIO_CODR = NGI;
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  asm volatile("nop\n\tnop\n\t");
  PIOC->PIO_SODR = NGI;
/// TEMPORARY
*/

  // set everything high on exit.

  PIOC->PIO_SODR = rdata[rpoint + 2];
  //PIOC->PIO_SODR = 0x37eff3fe;
  if (rpoint == wpoint)
    zero_count += 1;

  rpoint += 3; // we used up 3 words
  if (rpoint == MAX_DATA) rpoint = 0;

  // if we went from not enough room for more data to enough room, tell the mothership:
  if (wpoint >= rpoint)
    room = wpoint - rpoint; // data occupied in 4 byte words
  else
    room = wpoint + MAX_DATA - rpoint;
  room = MAX_DATA - room;
  if (need_ack == 1 && (room + 3) > 126 ) { // room available for 42 triples - a full packet. Make sure its more than 42 though.
    // don't do the usb write in here - we might miss the next event. Just set need_ack to 2 and let it be done in main program
    need_ack = 2;
  }
  if (rpoint == wpoint) { //prep for zeros for next time:
    rdata[rpoint] = 0;
    rdata[rpoint + 1] = 0;
    rdata[rpoint + 2] = 0;
  }
  //    PIOB->PIO_SODR = 0x8000000; // turn on led/D13  to indicate we're sending zeros

  // clear the pin interrupt
  i = PIOB->PIO_ISR;
  // and we're done.
}

void my_receive_usb_isr() {
  int i, n;
  uint8_t *ptr_src = (uint8_t *) &udd_get_endpoint_fifo_access8(CDC_RX);
  uint32_t  room, words_to_copy;
  uint32_t *fptr_src;
  uint8_t *dat8;

  fptr_src = (uint32_t *) ptr_src;
  dat8 = (uint8_t *) rdata;

  // we want to be able to interrupted in here, so don't:
  //irqflags_t flags = cpu_irq_save();

  if (Is_udd_endpoint_interrupt(CDC_RX)) {


    // we make lots of assumptions here!
    // assume that this is a CDC_RX interrupt
    // assume that the characters we want are at the start of the fifo.
    // assume we successfully read the whole fifo.

    udd_ack_out_received(CDC_RX);

    // How many bytes in fifo:
    n = ((UOTGHS->UOTGHS_DEVEPTISR[CDC_RX] & UOTGHS_DEVEPTISR_BYCT_Msk) >> UOTGHS_DEVEPTISR_BYCT_Pos);
    while (n > 0) {
      //SerialUSB.print("found n of ");
      //SerialUSB.println(n);
      switch (*ptr_src) {
        case 'K': // get out
          gpf_isr = saved_gpf_isr;
          mode = MODE_IDLE;
          SysTick->CTRL |= 2; // turn back on systick interrupts
          SerialUSB.println("OKK");
          break;
        case 'Q':
          SerialUSB.println(RESPONSE_ID);
          break;
        case 'S':
          return_status();
          break;
        case 'E':
          SerialUSB.println("OKE");
          break;
        case 'R':
          rpoint = 0;
          wpoint = 0;
          zero_count = 0;
          need_ack = 0;
          SerialUSB.println("OKR");
          break;
        case 'Z':
          cpu_irq_disable();
          write_zeros();
          cpu_irq_enable();
          SerialUSB.println("OKZ");
          break;
        case 'D':
          // check that there's room.
          if (wpoint >= rpoint)
            room = wpoint - rpoint; // data occupied in 4 byte words
          else
            room = wpoint + MAX_DATA - rpoint;

          room = MAX_DATA - room;

          words_to_copy = *(ptr_src + 1);
          words_to_copy *= 3;
          // how about:
          //words_to_copy = (n - 4) / 4;
          if (words_to_copy >= room) {
            SerialUSB.print(wpoint);
            SerialUSB.print(" ");
            SerialUSB.print(rpoint);
            SerialUSB.println(" Overfull, packet ignored");
          }
          else {
            //SerialUSB.println("GO");

            for (i = 0; i < words_to_copy; i++) {
              //              data[wpoint] = fptr_src[i + 1]; // no - alignment doesn't work! Crazy
              dat8[4 * wpoint] = ptr_src[(i + 1) * 4];
              dat8[4 * wpoint + 1] = ptr_src[(i + 1) * 4 + 1];
              dat8[4 * wpoint + 2] = ptr_src[(i + 1) * 4 + 2];
              dat8[4 * wpoint + 3] = ptr_src[(i + 1) * 4 + 3];

              wpoint += 1;
              if (wpoint == MAX_DATA)
                wpoint = 0;
            }
            if (room - words_to_copy > 126) {
              SerialUSB.print("GO"); // tell sender we're ready for more.
              SerialUSB.println(words_to_copy);
            }
            else
              need_ack = 1;
          }
          break;
        case 'd':
          SerialUSB.println("OKd");
          break;

      } // end switch.
      // release the buffer, since we emptied it.
      if (n > 0) {
        UOTGHS->UOTGHS_DEVEPTICR[CDC_RX] = UOTGHS_DEVEPTICR_TXINIC;
        UOTGHS->UOTGHS_DEVEPTIDR[CDC_RX] = UOTGHS_DEVEPTIDR_FIFOCONC;
      }
      n = ((UOTGHS->UOTGHS_DEVEPTISR[CDC_RX] & UOTGHS_DEVEPTISR_BYCT_Msk) >> UOTGHS_DEVEPTISR_BYCT_Pos);
      //if (n>0) SerialUSB.println("going for second pass");
    }
  }


  if (Is_udd_sof()) udd_ack_sof();
  if (Is_udd_endpoint_interrupt(0) || Is_udd_reset())
    saved_gpf_isr();


  //cpu_irq_restore(flags);
  return;


}

void setup() {

  rdata = (uint32_t *) data;
  rdata[0] = 0;
  rdata[1] = 0;
  rdata[2] = 0;
  SerialUSB.begin(38400);
  // need to set all the other port C pins - get 26 outputs altogether.
  // C1-9, 12-19, 21-26 and 28-30.

  pinMode(13, OUTPUT); // led this is B.27

  /////////////// configure port C pins as outputs

  // enable PIO control
  PIOC->PIO_PER = 0x37eff3fe;
  // allow direct writes to all pins - nope
  //PIOC->PIO_OWER = 0x37eff3fe;
  // set the outputs high
  PIOC->PIO_SODR = 0x37eff3fe;
  // and enable outputs
  PIOC->PIO_OER = 0x37eff3fe; //25 bits

  // for led:
  PIOB->PIO_PER = 0x8000000;
  PIOB->PIO_OER = 0x8000000;

  // will need one pin we hook up to an interrupt that we use to trigger our write.

  //////////////// set up external trig pin for pulse programer to tell us to write:
  // this is D22, B.26
  pmc_enable_periph_clk(ID_PIOB);
  PIO_Configure(PIOB, PIO_INPUT, 1 << 26, PIO_PULLUP | PIO_DEGLITCH);
  //PIO_Configure(PIOB, PIO_INPUT, 1 << 26, PIO_PULLUP );
  PIOB->PIO_AIMER = 1 << 26; // enable additional modes
  PIOB->PIO_ESR = 1 << 26; // on edges
  PIOB->PIO_REHLSR = 1 << 26; // rising edge.
  PIOB->PIO_IDR = 0xffffffff; // disable all pin interrupts
  int i =  PIOB->PIO_ISR; // clear the flag if its set
  PIOB->PIO_IER = 1 << 26; // but enable ours.
  zero_count = 0;
  NVIC_EnableIRQ(PIOB_IRQn); // don't need to enable the irq on NVIC. WFE/SEVONPEND doesn't need it to be enabled!

  // pin interrupts trigger two things: PIOB->PIO_ISR gets the pin bit set, and
  // and the pending flag gets set on the NVIC. Clear the bit in PIO_ISR first
  // and then the NVIC.

  // to enable the pin interrupt we should: clear the PIO_ISR, clear any pending irq, then enable the irq.
  // to disable: disable the irq, clear the PIO_ISR, and clear any pending.
  // this is irq 12 - interrupt set enable is NVIC base. Interrupt clear enable is base+ 0x80
  // clear pending is base + 0x180.

  //////////////////Configure NGI pin as open drain output That's C.24
  PIOC->PIO_MDER = 1 << 24;



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

  //////////////////// interrupt priority and the like
  NVIC_SetPriority(PIOB_IRQn, 0); // this is for the external trigger. Highest priority
  NVIC_SetPriority(UOTGHS_IRQn, 2); // USB interrupt priority is lower than our write interrupt.

  // hack the usb receive isr:

  saved_gpf_isr = gpf_isr;

}


void loop() {
  // we just sleep:
  char buff[2];
  while (1) {
    while (SerialUSB.available() == 0) {
      asm volatile ("wfi\n\t"); // does this slow us down to shut off the cpu?
      if (need_ack == 2) {
        SerialUSB.println("GO");
        need_ack = 0;
      }
    }
    buff[0] = SerialUSB.read();
    switch (buff[0]) {
      case 'E':
        PIOB->PIO_IER |= 1 << 26;
        mode = MODE_RUNNING;
        SysTick->CTRL &= ~2;
        gpf_isr = &my_receive_usb_isr;
        SerialUSB.println("OK");
        break;
      case 'D':
        SerialUSB.println("NOT RUNNING");
        break;
      case 'Q':
        SerialUSB.println(RESPONSE_ID);
        break;
      case 'S':
        return_status();
        break;
      case 'R':
        rpoint = 0;
        wpoint = 0;
        need_ack = 0;
        zero_count = 0;
        SerialUSB.println("OK");
        break;
      case 'K': // already there, just say ok.
        SerialUSB.println("OK");
        break;
      case 'Z':
        cpu_irq_disable();
        write_zeros();
        cpu_irq_enable();
        SerialUSB.println("OK");
        break;

    }
  }
}

