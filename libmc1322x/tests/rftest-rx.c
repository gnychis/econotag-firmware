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

volatile uint8_t led;

#define led_init() do { gpio_pad_dir_set(LED_WHITE); gpio_data_reset(LED_WHITE); } while(0);
#define led_on() do  { led = 1; gpio_data_set(LED); } while(0);
#define led_off() do { led = 0; gpio_data_reset(LED); } while(0);

void toggle_led(void) {
	if(0 == led) {
		led_on();
		led = 1;

	} else {
		led_off();
	}
}

int count=0;

void tmr0_isr(void) {

	if(count%10==0) {
		toggle_led();
		printf("trigger\n\r");
	}
	*TMR0_SCTRL = 0;
	*TMR0_CSCTRL = 0x0040; /* clear compare flag */
	count++;
}

void main(void) {
	volatile packet_t *p;
	volatile uint8_t chan;

	gpio_data(0);
	
	gpio_pad_dir_set( 1ULL << LED );
        /* read from the data register instead of the pad */
	/* this is needed because the led clamps the voltage low */
	gpio_data_sel( 1ULL << LED);

	/* trim the reference osc. to 24MHz */
	trim_xtal();

	uart_init(INC, MOD, SAMP);

	vreg_init();

	maca_init();

        /* sets up tx_on, should be a board specific item */
	//       *GPIO_FUNC_SEL2 = (0x01 << ((44-16*2)*2));
	gpio_pad_dir_set( 1ULL << 44 );

	set_power(0x0f); /* 0dbm */
	chan = 1;
	set_channel(chan); /* channel 11 */
	
	/* pin direction */
	led_init();

	/* timer setup */
	/* CTRL */
#define COUNT_MODE 1      /* use rising edge of primary source */
#define PRIME_SRC  0xf    /* Perip. clock with 128 prescale (for 24Mhz = 187500Hz)*/
#define SEC_SRC    0      /* don't need this */
#define ONCE       0      /* keep counting */
#define LEN        1      /* count until compare then reload with value in LOAD */
#define DIR        0      /* count up */
#define CO_INIT    0      /* other counters cannot force a re-initialization of this counter */
#define OUT_MODE   0      /* OFLAG is asserted while counter is active */

	*TMR_ENBL     = 0;                    /* tmrs reset to enabled */
	*TMR0_SCTRL   = 0;
	*TMR0_CSCTRL  = 0x0040;
	*TMR0_LOAD    = 0;                    /* reload to zero */
	*TMR0_COMP_UP = 18750;                /* trigger a reload at the end */
	*TMR0_CMPLD1  = 18750;                /* compare 1 triggered reload level, 10HZ maybe? */
	*TMR0_CNTR    = 0;                    /* reset count register */
	*TMR0_CTRL    = (COUNT_MODE<<13) | (PRIME_SRC<<9) | (SEC_SRC<<7) | (ONCE<<6) | (LEN<<5) | (DIR<<4) | (CO_INIT<<3) | (OUT_MODE);
	*TMR_ENBL     = 0xf;                  /* enable all the timers --- why not? */

	led_on();

	enable_irq(TMR);

	print_welcome("rftest-rx");
	while(1) {		

		/* call check_maca() periodically --- this works around */
		/* a few lockup conditions */
		check_maca();

		if((p = rx_packet())) {
			/* print and free the packet */
			printf("rftest-rx --- ");
			print_packet(p);
			free_packet(p);
		}

		if(uart1_can_get()) {
			uart1_getc();
			chan++;
			if(chan >= 16) { chan = 0; }
			set_channel(chan);
			printf("channel: %d\n\r", chan);
		}

	}
}
