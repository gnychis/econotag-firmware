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
#include <stdint.h>

volatile char u2_tx_buf[64];
volatile uint32_t u2_head, u2_tail;

void uart2_isr(void) {
	while( *UART2_UTXCON != 0 ) {
		if (u2_head == u2_tail) {
			disable_irq(UART2);
			return;
		}
		*UART2_UDATA = u2_tx_buf[u2_tail];
		u2_tail++;		
		if (u2_tail >= sizeof(u2_tx_buf))
			u2_tail = 0;
	}
}

void uart2_putc(char c) {
	/* disable UART2 since */
	/* UART2 isr modifies u2_head and u2_tail */ 
	disable_irq(UART2);

	if( (u2_head == u2_tail) &&
	    (*UART2_UTXCON != 0)) {
		*UART2_UDATA = c;
	} else {
		u2_tx_buf[u2_head] = c;
		u2_head += 1;
		if (u2_head >= sizeof(u2_tx_buf))
			u2_head = 0;
		if (u2_head == u2_tail) { /* drop chars when no room */
			if (u2_head) { u2_head -=1; } else { u2_head = sizeof(u2_tx_buf); }
		}
		enable_irq(UART2);
	}
}

uint8_t uart2_getc(void) {
	while(uart2_can_get() == 0) { continue; }
	return *UART2_UDATA;
}
