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


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"
#include "bsp.h"
#include "er-coap.h"

/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/

/* Handler for GET actions. For further details see the function definition */
static void res_get_handler(void *request, void *response, uint8_t *buffer,
        uint16_t preferred_size, int32_t *offset);

/* Handler for POST actions. For further details see the function definition */
static void res_post_handler(void *request, void *response, uint8_t *buffer,
        uint16_t preferred_size, int32_t *offset);

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
/* Resource definition of a simple actuator example. This
 * toggles one or more LEDs */
RESOURCE(res_led,
         "title=\"LED toggle\";type=\"Control\"",
         res_get_handler,
         res_post_handler,
         NULL,
         NULL);


/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/

/**
 * \brief   Handler for getting data from the resource.
 *
 *          The LED resource has no data to get However it is possible
 *          to get a description of how to control the resource..
 */
static void res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  /* Some data that has the length up to REST_MAX_CHUNK_SIZE. For more, see the chunk resource. */
  char const *const message = "Use POST to toggle the LEDs using parameters (e.g. POST: led=1)";
  int length = strlen(message);

  memcpy(buffer, message, length);
  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_header_etag(response, (uint8_t *)&length, 1);
  REST.set_response_payload(response, buffer, length);
}

/**
 * \brief   Handler for posting data to the resource.
 *
 *          The LED resource can be controlled by using a POST. The post
 *          variable "led" defines which LED to toggle as a bitmask. If no or an
 *          invalid LED mask was selected all the LEDs will be toggled.
 *          command: led=<x>.
 */
static void res_post_handler(void *request, void *response, uint8_t *buffer,
        uint16_t preferred_size, int32_t *offset)
{
    const char *ledVal = NULL;

    /* check the query variable */
    if(REST.get_post_variable(request, "led", &ledVal) == 0) {

        /* No POST variable. Toggle all LEDs */
        bsp_led(E_BSP_LED_0,E_BSP_LED_TOGGLE);
        bsp_led(E_BSP_LED_1,E_BSP_LED_TOGGLE);
        bsp_led(E_BSP_LED_2,E_BSP_LED_TOGGLE);
        bsp_led(E_BSP_LED_3,E_BSP_LED_TOGGLE);

    } else {

        int ledMsk = atoi(ledVal);

        /* LED mask is available. Check which LEDs to toggle. */
        if( ledMsk & (1<<0) )
            bsp_led(E_BSP_LED_0,E_BSP_LED_TOGGLE);
        else if( ledMsk & (1<<1) )
            bsp_led(E_BSP_LED_1,E_BSP_LED_TOGGLE);
        else if( ledMsk & (1<<2) )
            bsp_led(E_BSP_LED_2,E_BSP_LED_TOGGLE);
        else if( ledMsk & (1<<3) )
            bsp_led(E_BSP_LED_3,E_BSP_LED_TOGGLE);
        else {
            bsp_led(E_BSP_LED_0,E_BSP_LED_TOGGLE);
            bsp_led(E_BSP_LED_1,E_BSP_LED_TOGGLE);
            bsp_led(E_BSP_LED_2,E_BSP_LED_TOGGLE);
            bsp_led(E_BSP_LED_3,E_BSP_LED_TOGGLE);
        }
    }
}
