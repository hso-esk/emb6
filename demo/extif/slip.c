/* -*- C -*- */
/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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
 * This file is part of the Contiki operating system.
 *
 */
/**
 * \file
 *         A brief description of what this file is
 */

#include <stdio.h>
#include <string.h>

#include "emb6.h"

#include "evproc.h"
#include "uip.h"

#define BUF ((struct uip_tcpip_hdr *)&uip_buf[UIP_LLH_LEN])

#include "slip.h"

#define SLIP_END     0300
#define SLIP_ESC     0333
#define SLIP_ESC_END 0334
#define SLIP_ESC_ESC 0335


uint8_t slip_active;

#if 1
#define SLIP_STATISTICS(statement)
#else
uint16_t slip_rubbish, slip_twopackets, slip_overflow, slip_ip_drop;
#define SLIP_STATISTICS(statement) statement
#endif

/* Must be at least one byte larger than UIP_BUFSIZE! */
#define RX_BUFSIZE (5*(UIP_BUFSIZE - UIP_LLH_LEN + 16))

enum {
  STATE_TWOPACKETS = 0,    /* We have 2 packets and drop incoming data. */
  STATE_OK = 1,
  STATE_ESC = 2,
  STATE_RUBBISH = 3,
};

/*
 * Variables begin and end manage the buffer space in a cyclic
 * fashion. The first used byte is at begin and end is one byte past
 * the last. I.e. [begin, end) is the actively used space.
 *
 * If begin != pkt_end we have a packet at [begin, pkt_end),
 * furthermore, if state == STATE_TWOPACKETS we have one more packet at
 * [pkt_end, end). If more bytes arrive in state STATE_TWOPACKETS
 * they are discarded.
 */

static uint8_t state = STATE_TWOPACKETS;
static uint16_t begin, next_free;
static uint8_t rxbuf[RX_BUFSIZE];
static uint16_t pkt_end;        /* SLIP_END tracker. */

static     void (* input_callback)(void) = NULL;
static     void rxbuf_init(void);
static     void slip_callback(c_event_t ev, p_data_t data);

void slip_init(void)
{
    rxbuf_init();
    evproc_regCallback(EVENT_TYPE_SLIP_POLL, slip_callback);
}

/*---------------------------------------------------------------------------*/
void
slip_set_input_callback(void (*c)(void))
{
  input_callback = c;
}
/*---------------------------------------------------------------------------*/
/* slip_send: forward (IPv4) packets with {UIP_FW_NETIF(..., slip_send)}
 * was used in slip-bridge.c
 */
//#if WITH_UIP

/*---------------------------------------------------------------------------*/
static void
rxbuf_init(void)
{
  begin = next_free = pkt_end = 0;
  state = STATE_OK;
}
/*---------------------------------------------------------------------------*/
/* Upper half does the polling. */
static uint16_t
slip_poll_handler(uint8_t *outbuf, uint16_t blen)
{
  /* This is a hack and won't work across buffer edge! */
  if(rxbuf[begin] == 'C') {
    int i;
    if(begin < next_free && (next_free - begin) >= 6
       && memcmp(&rxbuf[begin], "CLIENT", 6) == 0) {
      state = STATE_TWOPACKETS;    /* Interrupts do nothing. */
      memset(&rxbuf[begin], 0x0, 6);
      
      rxbuf_init();
      
      for(i = 0; i < 13; i++) {
        //  printf("CLIENTSERVER\300%d",i);
      }
      return 0;
    }
  }
#ifdef SLIP_CONF_ANSWER_MAC_REQUEST
  else if(rxbuf[begin] == '?') { 
    /* Used by tapslip6 to request mac for auto configure */
    int i, j;
    char* hexchar = "0123456789abcdef";
    if(begin < next_free && (next_free - begin) >= 2
       && rxbuf[begin + 1] == 'M') {
      state = STATE_TWOPACKETS; /* Interrupts do nothing. */
      rxbuf[begin] = 0;
      rxbuf[begin + 1] = 0;
      
      rxbuf_init();
      
      linkaddr_t addr = get_mac_addr();
      /* this is just a test so far... just to see if it works */
      slip_arch_writeb('!');
      slip_arch_writeb('M');
      for(j = 0; j < 8; j++) {
        slip_arch_writeb(hexchar[addr.u8[j] >> 4]);
        slip_arch_writeb(hexchar[addr.u8[j] & 15]);
      }
      slip_arch_writeb(SLIP_END);
      return 0;
    }
  }
#endif /* SLIP_CONF_ANSWER_MAC_REQUEST */

  /*
   * Interrupt can not change begin but may change pkt_end.
   * If pkt_end != begin it will not change again.
   */
  if(begin != pkt_end) {
    uint16_t len;
    uint16_t cur_next_free;
    uint16_t cur_ptr;
    int esc = 0;

    if(begin < pkt_end) {
      uint16_t i;
      len = 0;
      for(i = begin; i < pkt_end; ++i) {
        if(len > blen) {
          len = 0;
          break;
        }
        if (esc) {
          if(rxbuf[i] == SLIP_ESC_ESC) {
            outbuf[len] = SLIP_ESC;
            len++;
          } else if(rxbuf[i] == SLIP_ESC_END) {
            outbuf[len] = SLIP_END;
            len++;
          }
          esc = 0;
        } else if(rxbuf[i] == SLIP_ESC) {
          esc = 1;
        } else {
          outbuf[len] = rxbuf[i];
          len++;
        }
      }
    } else {
      uint16_t i;
      len = 0;
      for(i = begin; i < RX_BUFSIZE; ++i) {
        if(len > blen) {
          len = 0;
          break;
        }
        if (esc) {
          if(rxbuf[i] == SLIP_ESC_ESC) {
            outbuf[len] = SLIP_ESC;
            len++;
          } else if(rxbuf[i] == SLIP_ESC_END) {
            outbuf[len] = SLIP_END;
            len++;
          }
          esc = 0;
        } else if(rxbuf[i] == SLIP_ESC) {
          esc = 1;
        } else {
          outbuf[len] = rxbuf[i];
          len++;
        }
      }
      for(i = 0; i < pkt_end; ++i) {
        if(len > blen) {
          len = 0;
          break;
        }
        if (esc) {
          if(rxbuf[i] == SLIP_ESC_ESC) {
            outbuf[len] = SLIP_ESC;
            len++;
          } else if(rxbuf[i] == SLIP_ESC_END) {
            outbuf[len] = SLIP_END;
            len++;
          }
          esc = 0;
        } else if(rxbuf[i] == SLIP_ESC) {
          esc = 1;
        } else {
          outbuf[len] = rxbuf[i];
          len++;
        }
      }
    }

    /* Remove data from buffer together with the copied packet. */
    pkt_end = pkt_end + 1;
    if(pkt_end == RX_BUFSIZE) {
      pkt_end = 0;
    }
    if(pkt_end != next_free) {
      cur_next_free = next_free;
      cur_ptr = pkt_end;
      while(cur_ptr != cur_next_free) {
        if(rxbuf[cur_ptr] == SLIP_END) {
          uint16_t tmp_begin = pkt_end;
          pkt_end = cur_ptr;
          begin = tmp_begin;
          /* One more packet is buffered, need to be polled again! */
          evproc_putEvent(E_EVPROC_HEAD, EVENT_TYPE_SLIP_POLL, NULL);
          //process_poll(&slip_process);
          break;
        }
        cur_ptr++;
        if(cur_ptr == RX_BUFSIZE) {
          cur_ptr = 0;
        }
      }
      if(cur_ptr == cur_next_free) {
        /* no more pending full packet found */
        begin = pkt_end;
      }
    } else {
      begin = pkt_end;
    }
    return len;
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
void slip_callback(c_event_t ev, p_data_t data)
{
    slip_active = 1;
    /* Move packet from rxbuf to buffer provided by uIP. */
    uip_len = slip_poll_handler(&uip_buf[UIP_LLH_LEN],
                UIP_BUFSIZE - UIP_LLH_LEN);
    if(uip_len > 0) {
      if(input_callback) {
        input_callback();
      }
#ifdef SLIP_CONF_TCPIP_INPUT
      SLIP_CONF_TCPIP_INPUT();
#else
      tcpip_input();
#endif
    }
}
/*---------------------------------------------------------------------------*/
void slip_input_byte(void * chr)
{
  uint16_t cur_end;
  unsigned char c = *((unsigned char *)chr);
  switch(state) {
  case STATE_RUBBISH:
    if(c == SLIP_END) {
      state = STATE_OK;
    }
    return;

  case STATE_ESC:
    if(c != SLIP_ESC_END && c != SLIP_ESC_ESC) {
      state = STATE_RUBBISH;
      SLIP_STATISTICS(slip_rubbish++);
      next_free = pkt_end;        /* remove rubbish */
      return;
    }
    state = STATE_OK;
    break;
  }

  if(c == SLIP_ESC) {
    state = STATE_ESC;
  }

  /* add_char: */
  cur_end = next_free;
  next_free = next_free + 1;
  if(next_free == RX_BUFSIZE) {
    next_free = 0;
  }
  if(next_free == begin) {         /* rxbuf is full */
    state = STATE_RUBBISH;
    SLIP_STATISTICS(slip_overflow++);
    next_free = pkt_end;            /* remove rubbish */
    return;
  }
  rxbuf[cur_end] = c;
  /* There could be a separate poll routine for this. */
  if(c == 'T' && rxbuf[begin] == 'C') {
      evproc_putEvent(E_EVPROC_HEAD, EVENT_TYPE_SLIP_POLL, NULL);
      return;
  }

  if(c == SLIP_END) {
    /*
     * We have a new packet, possibly of zero length.
     *
     * There may already be one packet buffered.
     */
    if(cur_end != pkt_end) {    /* Non zero length. */
      if(begin == pkt_end) {    /* None buffered. */
        pkt_end = cur_end;
      } else {
        SLIP_STATISTICS(slip_twopackets++);
      }
      evproc_putEvent(E_EVPROC_HEAD, EVENT_TYPE_SLIP_POLL, NULL);
      return;
    } else {
      /* Empty packet, reset the pointer */
      next_free = cur_end;
    }
    return;
  }

  return;
}
/*---------------------------------------------------------------------------*/
