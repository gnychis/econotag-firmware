/*
 * Copyright (c) 2010, Mariano Alvira <mar@devl.org> and other contributors
 * to the MC1322x project (http://mc1322x.devl.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of libmc1322x: see http://mc1322x.devl.org
 * for details. 
 *
 *
 */

#include <mc1322x.h>
#include <board.h>
#include <stdio.h>

#include "tests.h"
#include "config.h"

#define LED LED_GREEN
#define COUNT_MODE 1      /* use rising edge of primary source */
#define PRIME_SRC  0xf    /* Perip. clock with 128 prescale (for 24Mhz = 187500Hz)*/
#define SEC_SRC    0      /* don't need this */
#define ONCE       0      /* keep counting */
#define LEN        1      /* count until compare then reload with value in LOAD */
#define DIR        0      /* count up */
#define CO_INIT    0      /* other counters cannot force a re-initialization of this counter */
#define OUT_MODE   0      /* OFLAG is asserted while counter is active */

#define DELAY 400000

#define NEWLED (1ULL << LED_GREEN)

void maca_rx_callback(volatile packet_t *p) {
  (void)p;
  int i;
  gpio_data(NEWLED);

  for(i=0; i<DELAY; i++) { continue; }

  gpio_data(0);

  for(i=0; i<DELAY; i++) { continue; }
}

volatile int missing=0;
volatile int tick_count=0;
volatile int pcnt=0;

void tick(void) {

  if(tick_count%10==0) {
    printf("Packets-per-second: %d (power: %u) -- %d / %d\n\r",pcnt, get_power(), missing, missing+pcnt);
    pcnt=0;
    missing=0;
  }

  *TMR0_SCTRL = 0;
  tick_count++;
}

void main(void) {
  volatile packet_t *p;
  volatile unsigned int next_pktnum=0;
  volatile int init=1;

  gpio_data(0);

  gpio_pad_dir_set( 1ULL << LED );
  gpio_data_sel( 1ULL << LED);

  /* trim the reference osc. to 24MHz */
  trim_xtal();
  uart_init(INC, MOD, SAMP);
  vreg_init();
  maca_init();

  ///* Setup the timer */
  *TMR_ENBL = 0;                     /* tmrs reset to enabled */
  *TMR0_SCTRL = 0;
  *TMR0_LOAD = 0;                    /* reload to zero */
  *TMR0_COMP_UP = 18750;             /* trigger a reload at the end */
  *TMR0_CMPLD1 = 18750;              /* compare 1 triggered reload level, 10HZ maybe? */
  *TMR0_CNTR = 0;                    /* reset count register */
  *TMR0_CTRL = (COUNT_MODE<<13) | (PRIME_SRC<<9) | (SEC_SRC<<7) | (ONCE<<6) | (LEN<<5) | (DIR<<4) | (CO_INIT<<3) | (OUT_MODE);
  *TMR_ENBL = 0xf;                   /* enable all the timers --- why not? */

  set_channel(15); /* channel 11 */
  set_power(0x0f); /* 0dbm */

  gpio_pad_dir_set( 1ULL << 44 );

  while(1) {		

    if((*TMR0_SCTRL >> 15) != 0) 
      tick();

    /* call check_maca() periodically --- this works around */
    /* a few lockup conditions */
    check_maca();

    if((p = rx_packet())) {
      volatile unsigned int pktnum=0;
      pktnum = pktnum | (p->data[1] << 8*3);
      pktnum = pktnum | (p->data[2] << 8*2);
      pktnum = pktnum | (p->data[3] << 8*1);
      pktnum = pktnum | (p->data[4]);
      free_packet(p);

      // If the next expected number is -1, it's an initialization flag
      if(init) { 
        next_pktnum = pktnum;
        init=0;
      }

      if(pktnum != next_pktnum)
        missing = missing + (pktnum - next_pktnum);

      next_pktnum = pktnum+1;
      pcnt++;
    }
  }
}
