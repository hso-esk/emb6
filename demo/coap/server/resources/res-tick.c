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

/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "bsp.h"
#include "er-coap.h"
#include "emb6.h"
#include "packetbuf.h"


/*
********************************************************************************
*                                    MACROS
********************************************************************************
*/
/** Update interval */
#define RES_TICK_UPDATE_INTERVAL    2
/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/

/* Handler for GET actions. For further details see the function definition */
static void res_get_handler(void *request, void *response, uint8_t *buffer,
        uint16_t preferred_size, int32_t *offset, void* user);

/* Periodic handler */
static void res_periodic_handler(void);

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
/* Resource definition of a simple periodic resource. */
PERIODIC_RESOURCE(res_tick,
        "title=\"Alarm Message\";type=\"Alarm\";obs",
        res_get_handler,
        NULL,
        NULL,
        NULL,
        RES_TICK_UPDATE_INTERVAL,
        res_periodic_handler);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
/*
 * Use local resource state that is accessed by res_get_handler()
 * and altered by res_periodic_handler().
 */
static int32_t event = 0;


/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
/**
 * \brief   Handler for getting data from the resource.
 *
 *          Returns the current counter value.
 */
static void res_get_handler(void *request, void *response, uint8_t *buffer,
        uint16_t preferred_size, int32_t *offset, void* user)
{
    /* Just return the current counter value. */
    int length = snprintf( (char*)buffer, preferred_size,"Counter: %li", event );

    REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
    REST.set_header_max_age(response, res_tick.un_handler.periodic->period );
    REST.set_response_payload(response, buffer, length);

}
/**
 * \brief   Periodic update handler.
 *
 *          Is called in well defined intervals depending on the configuration
 *          of the resource. It updates all subscribers.
 */
static void res_periodic_handler()
{
    event++;
    event++;

  /* Usually a condition is defined under with subscribers are notified, e.g., large enough delta in sensor reading. */
  if(1) {
    /* Notify the registered observers which will trigger the res_get_handler to create the response. */
    REST.notify_subscribers(&res_tick);
  }
}
