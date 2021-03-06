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

#define LED LED_RED
#define COUNT_MODE 1      /* use rising edge of primary source */
#define PRIME_SRC  0xf    /* Perip. clock with 128 prescale (for 24Mhz = 187500Hz)*/
#define SEC_SRC    0      /* don't need this */
#define ONCE       0      /* keep counting */
#define LEN        1      /* count until compare then reload with value in LOAD */
#define DIR        0      /* count up */
#define CO_INIT    0      /* other counters cannot force a re-initialization of this counter */
#define OUT_MODE   0      /* OFLAG is asserted while counter is active */

#define POWER_DELAY 200

#define FIXED_WAIT

#define NODE_A
//#define NODE_B

/*
    1369863 --> 1 second
    328752  --> 240ms
    164376  --> 120ms
    136986  --> 100ms
    82188   --> 60ms
    54794   --> 40ms
    13698   --> 10ms
    6849    --> 5ms
    2738    --> 2ms
    1369    --> 1ms
    228     --> 166us
*/
//#define RANDOM_WAIT_TIME
//#define BLOCKING_TX
#ifdef NODE_A
  #define MAX_WAIT 13698
  #define PAYLOAD_LEN 75
  #define channel 1
#else
  #define MAX_WAIT 82188
  #define PAYLOAD_LEN 45
  #define channel 15
#endif

#define ADDR 0xcc

void fill_packet(volatile packet_t *p) {
  volatile int i=0;

  p->length = PAYLOAD_LEN;
  p->offset = 0;
  p->data[0] = 0x03;
  p->data[1] = 0x08;
  p->data[2] = 0x17;
  p->data[3] = 0xff;
  p->data[4] = 0xff;
  p->data[5] = 0xff;
  p->data[6] = ADDR;
  p->data[7] = 0x07;

  /* payload */
  for(i=8; i < PAYLOAD_LEN; i++)
    p->data[i] = i & 0xff;
}

void random_wait(void) {
#ifndef FIXED_WAIT
  volatile uint32_t wait = (unsigned int)maca_random%MAX_WAIT;
#else
  volatile uint32_t wait = (unsigned int)MAX_WAIT;
#endif
  while(wait>0) {wait--;}
}

int count=0;
unsigned int pkt_cnt=0;
unsigned int cnt=0;

void tmr0_isr(void) {

  if(count%10==0) {
    printf("Packets-per-second: %d  Payload: %u  MaxWait: %u\n\r", pkt_cnt, PAYLOAD_LEN, MAX_WAIT/1369);
    pkt_cnt=0;
  }

  *TMR0_SCTRL = 0; /*clear bit 15, and all the others --- should be ok, but clearly not "the right thing to do" */
  count++;
}

volatile int transmitting=0;
void blocking_tx_packet(volatile packet_t *p) {
  transmitting=1;
  tx_packet(p);
  while(transmitting) {}
}

void maca_tx_callback(volatile packet_t *p) {
  switch(p->status) {
    case 0:
      transmitting=0;
      break;
    default:
      break;
  }
}

void main(void) {
  volatile packet_t *p;
  volatile int hold_tx=0;

#ifdef CARRIER_SENSE
  volatile int i=0;
#endif

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

  set_channel(channel); /* channel 11 */
  set_power(0x12); /* 0x12 is the highest, not documented */

  /* sets up tx_on, should be a board specific item */
  *GPIO_FUNC_SEL2 = (0x01 << ((44-16*2)*2));
  gpio_pad_dir_set( 1ULL << 44 );

  while(1) {		
		
    if(uart1_can_get()) {
			uart1_getc();
      if(hold_tx==0)
        hold_tx=1;
      else
        hold_tx=0;
		}

    if((*TMR0_SCTRL >> 15) != 0) 
      tmr0_isr();

    if(hold_tx)
      continue;

    /* call check_maca() periodically --- this works around */
    /* a few lockup conditions */
    check_maca();

    while((p = rx_packet())) {
      if(p) free_packet(p);
    }

    p = get_free_packet();

    if(p) {
      fill_packet(p);

      p->data[3] = cnt & 0xff;
      p->data[2] = (cnt >> 8*1) & 0xff;
      p->data[1] = (cnt >> 8*2) & 0xff;
      p->data[0] = (cnt >> 8*3) & 0xff;

#ifdef CARRIER_SENSE
      for(i=0; i<POWER_DELAY; i++) {continue;}
      while(get_power()>74) {}
#endif

#ifdef BLOCKING_TX
      blocking_tx_packet(p);
#else
      tx_packet(p);
#endif

      pkt_cnt++;
      cnt++;

#ifdef RANDOM_WAIT_TIME
      random_wait();
#endif
    }
  }
}
