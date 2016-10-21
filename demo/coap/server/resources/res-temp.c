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
 * \authors
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 *      Peter Lehmann <peter.lehmann@hs-offenburg.de>
 *      Edgar Schmitt <edgar.schmitt@hs-offenburg.de>
 */


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"
#include "er-coap.h"

/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/

/* Handler for GET actions. For further details see the function definition */
static void res_get_handler(void *request, void *response, uint8_t *buffer,
        uint16_t preferred_size, int32_t *offset);

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
/* Resource definition of a simple sensor example. This
 * returns the temperature */
RESOURCE(res_temp,
         "title=\"Temperature sensor\";type=\"Info\"",
         res_get_handler,
         NULL,
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
 *          The TEMP resource can return the temperature in Celcius of
 *          Fahrenheit depending on the query parameters. This can be selected
 *          using the query ?unit=celcius for Celcius or ?unit=fahrenheit for Fahrenheit.
 */
static void res_get_handler(void *request, void *response, uint8_t *buffer,
        uint16_t preferred_size, int32_t *offset)
{
    size_t len;
    const char* unit = NULL;
    char* message = NULL;
    int length;

    if((len = REST.get_query_variable(request, "unit", &unit))) {

        if(strncmp(unit, "celcius", len) == 0) {
            /* Ceclius chosen as unit */
            message ="Temperature 22°C";
        }
        else if(strncmp(unit, "fahrenheit", len) == 0) {
            /* Fahrenheit chosen as unit */
            message ="Temperature 71,6°F";
        }
        else{
            /* Invalid unit */
            message ="Invalid unit: Please use ?unit=<celcius|fahrenheit>";
        }
    } else {
        /* No unit */
        message ="No unit specified: Please use ?unit=<celcius|fahrenheit>";
    }

    length = snprintf( (char*)buffer, preferred_size, message );
    REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
    REST.set_header_etag(response, (uint8_t *)&length, 1);
    REST.set_response_payload(response, buffer, length);
}
