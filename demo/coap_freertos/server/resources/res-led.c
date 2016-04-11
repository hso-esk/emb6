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
 *      Example resource
 * \author
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 *      modified by Peter Lehmann <peter.lehmann@hs-offenburg.de>
 *
 */



#include "emb6.h"
#include "demo_coap_srv.h"
#include "bsp.h"
#include "er-coap.h"

s_led_task_param_t ledTaskParams;

static void res_leds_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_leds_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);

/* A simple actuator example. Toggles the leds */
RESOURCE(res_leds,
        "title=\"LEDs: ?speed=slow|fast, POST/PUT mode=on|off\"rt=\"Control\"",
         res_leds_get_handler,
         res_leds_post_put_handler,
         res_leds_post_put_handler,
         NULL);

static void
res_leds_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  /* Some data that has the length up to REST_MAX_CHUNK_SIZE. For more, see the chunk resource. */
  char const *const message = "URI-Syntax: LEDs?speed=slow|fast; Outgoing: mode=on|off";
  int length = 55; /*          |<-------->| */
  memcpy(buffer, message, length);
  REST.set_header_content_type(response, REST.type.TEXT_PLAIN); /* text/plain is the default, hence this option could be omitted. */
  REST.set_header_etag(response, (uint8_t *)&length, 1);
  REST.set_response_payload(response, buffer, length);
}

static void
res_leds_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
      size_t len = 0;
      const char *speed = NULL;
      const char *mode = NULL;
      int success = 1;

      if((len = REST.get_query_variable(request, "speed", &speed))) {

        if(strncmp(speed, "slow", len) == 0) {
            ledTaskParams.delay = 1000;
        } else if(strncmp(speed, "fast", len) == 0) {
            ledTaskParams.delay = 150;
        } else {
          success = 0;
        }
      }
        if(success && (len = REST.get_post_variable(request, "mode", &mode))) {

        if(strncmp(mode, "on", len) == 0) {
            ledTaskParams.en = 1;
        } else if(strncmp(mode, "off", len) == 0) {
            ledTaskParams.en = 0;
        } else {
          success = 0;
        }
      } else {
        success = 0;
      } if(!success) {
        REST.set_response_status(response, REST.status.BAD_REQUEST);
      }
}
