/*
 * Copyright (c) 2013, Institute for Pervasive Computing, ETH Zurich
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
 */

/**
 * \file
 *      CoAP implementation for the REST Engine.
 * \author
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 */

#ifndef ER_COAP_ENGINE_H_
#define ER_COAP_ENGINE_H_


#include "er-coap.h"
#include "er-coap-transactions.h"
#include "er-coap-observe.h"
#include "er-coap-separate.h"
#include "er-coap-observe-client.h"

#define SERVER_LISTEN_PORT  COAP_SERVER_PORT

typedef coap_packet_t rest_request_t;
typedef coap_packet_t rest_response_t;

void coap_init_engine(void);

void coap_engine_callback(c_event_t c_event, p_data_t p_data);

/*---------------------------------------------------------------------------*/
/*- Client Part -------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
typedef void (*nonblocking_response_handler)(void *response);

struct request_state_t {
  coap_transaction_t *transaction;
  coap_packet_t *response;
  uint32_t block_num;
  uint8_t more;
  uint32_t res_block;
  uint8_t block_error;
  uip_ipaddr_t *remote_ipaddr;
  uint16_t remote_port;
  coap_packet_t *request;
  nonblocking_response_handler request_callback;
};

coap_transaction_t* coap_nonblocking_request (uip_ipaddr_t *remote_ipaddr, uint16_t remote_port,
            coap_packet_t *request,
            nonblocking_response_handler request_callback);
/*---------------------------------------------------------------------------*/

#endif /* ER_COAP_ENGINE_H_ */
