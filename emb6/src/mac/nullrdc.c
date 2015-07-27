/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 *         A null RDC implementation that uses framer for headers.
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Niclas Finne <nfi@sics.se>
 */

//#include "net/mac/mac-sequence.h"
#include "emb6.h"
#include "nullrdc.h"
#include "packetbuf.h"
#include "linkaddr.h"
#include "queuebuf.h"
#include "rimestats.h"




#if CONTIKI_TARGET_COOJA
#include "lib/simEnvChange.h"
#endif /* CONTIKI_TARGET_COOJA */

#define DEBUG DEBUG_NONE
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define ACK_LEN 3

static s_ns_t*    p_ns = NULL;

/*---------------------------------------------------------------------------*/
static int
send_one_packet(mac_callback_t sent, void *ptr)
{
  int ret;
  int last_sent_ok = MAC_TX_ERR;

  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);

  if ((p_ns != NULL) && (p_ns->frame != NULL) && (p_ns->inif != NULL)) {

      if(p_ns->frame->create_and_secure(p_ns) < 0) {
        /* Failed to allocate space for headers */
        PRINTF("nullrdc: send failed, too large header\n");
        ret = MAC_TX_ERR_FATAL;
      } else {

        switch(p_ns->inif->send(packetbuf_hdrptr(), packetbuf_totlen())) {
        case RADIO_TX_OK:
          ret = MAC_TX_OK;
          break;
        case RADIO_TX_COLLISION:
          ret = MAC_TX_COLLISION;
          break;
        case RADIO_TX_NOACK:
          ret = MAC_TX_NOACK;
          break;
        default:
          ret = MAC_TX_ERR;
          break;
        }

      }
      if(ret == MAC_TX_OK) {
        last_sent_ok = 1;
      }
      mac_call_sent_callback(sent, ptr, ret, 1);
  }
  return last_sent_ok;
}
/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
  send_one_packet(sent, ptr);
}
/*---------------------------------------------------------------------------*/
static void
send_list(mac_callback_t sent, void *ptr, struct lmac_buf_list *buf_list)
{
  while(buf_list != NULL) {
    /* We backup the next pointer, as it may be nullified by
     * mac_call_sent_callback() */
    struct lmac_buf_list *next = buf_list->next;
    int last_sent_ok;

    queuebuf_to_packetbuf(buf_list->buf);
    last_sent_ok = send_one_packet(sent, ptr);

    /* If packet transmission was not successful, we should back off and let
     * upper layers retransmit, rather than potentially sending out-of-order
     * packet fragments. */
    if(!last_sent_ok) {
      return;
    }
    buf_list = next;
  }
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
//    int original_datalen;
//    uint8_t *original_dataptr;
//
//    original_datalen = packetbuf_datalen();
//    original_dataptr = packetbuf_dataptr();

    if ((p_ns != NULL) && (p_ns->frame != NULL) && (p_ns->hmac != NULL)) {
        if(p_ns->frame->parse() < 0) {
            PRINTF("nullrdc: failed to parse %u\n", packetbuf_datalen());
        } else {
            p_ns->hmac->input();
        }
    }
}
/*---------------------------------------------------------------------------*/
static int8_t on(void)
{
    if ((p_ns != NULL) && (p_ns->inif != NULL))
        return p_ns->inif->on();
    else
        return 0;
}
/*---------------------------------------------------------------------------*/
static int8_t off(int keep_radio_on)
{
    if ((p_ns != NULL) && (p_ns->inif != NULL)) {
        if(keep_radio_on) {
            return p_ns->inif->on();
        } else {
            return p_ns->inif->off();
        }
    } else
        return 0;
}
/*---------------------------------------------------------------------------*/
static void init(s_ns_t* p_netStack)
{
    if ((p_netStack != NULL) && (p_netStack->inif != NULL)) {
        p_ns = p_netStack;
        p_ns->inif->on();
    }
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
const s_nsLowMac_t nullrdc_driver = {
  "nullrdc",
  init,
  send_packet,
  send_list,
  packet_input,
  on,
  off,
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/
