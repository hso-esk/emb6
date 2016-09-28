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

#if DEMO_COAP_SERVER_LCD_RES_EN

#include "emb6.h"
#include "bsp.h"
#include "er-coap.h"
#include "lcd_dogm128_6.h"

/*
********************************************************************************
*                                   MACROS
********************************************************************************
*/
/** maximum LCD message size */
#define LCD_MSG_MAX						40
/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/

/** LCD enabled ? */
static uint8_t lcdEn = 0;

/** LCD message to display */
static char lcdMsg[LCD_MSG_MAX] = {0};

/** LCD buffer */
static char lcdBuf[LCD_BYTES] = {0};

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

/* Handler for PUT actions. For further details see the function definition */
static void res_put_handler(void *request, void *response, uint8_t *buffer,
        uint16_t preferred_size, int32_t *offset);

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
/* Resource definition of a simple actuator example. This
 * toggles one or more LEDs */
RESOURCE(res_lcd,
         "title=\"Control LCD display\";type=\"Control\"",
         res_get_handler,
         res_post_handler,
		 res_put_handler,
         NULL);


/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
/**
 * \brief   Handler for getting data from the resource.
 *
 *          The LCD resource has no data to get However it is possible
 *          to get a description of how to control the resource..
 */
static void res_get_handler(void *request, void *response, uint8_t *buffer,
        uint16_t preferred_size, int32_t *offset)
{
  /* Some data that has the length up to REST_MAX_CHUNK_SIZE. For more, see the chunk resource. */
  char const *const message = "Use POST to enable/disable the LCD and PUT to set a text.";
  int length = snprintf( (char*)buffer, preferred_size, message );

  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_header_etag(response, (uint8_t *)&length, 1);
  REST.set_response_payload(response, buffer, length);
}

/**
 * \brief   Handler for posting data to the resource.
 *
 *          The LCD resource can be controlled by using a POST. The post
 *          variable "enable" defines if the LCD shall be enabled or not (1/0).
 *          command: enable=<x>.
 */
static void res_post_handler(void *request, void *response, uint8_t *buffer,
        uint16_t preferred_size, int32_t *offset)
{
    const char *lcdVal = NULL;

    /* check the query variable */
    if(REST.get_post_variable(request, "enable", &lcdVal) == 0) {

        /* No POST variable. Ignore */

    } else {

        lcdEn = atoi(lcdVal);

        if( lcdEn ) {

        	/* write to LCD */
        	lcdBufferClear( lcdBuf );
        	lcdBufferPrintString( lcdBuf, lcdMsg, 0, 0 );
        	lcdSendBuffer( lcdBuf );

        } else {
        	/* clear LCD */
        	lcdBufferClear( lcdBuf );
        	lcdSendBuffer( lcdBuf );
        }
    }
}

/**
 * \brief   Handler to PUT data to the resource.
 *
 *          The LCD resource can be controlled by using a PUT. It allows
 *          to send a specific text to the LCD display.
 *          command: text=<x>.
 */
static void res_put_handler(void *request, void *response, uint8_t *buffer,
        uint16_t preferred_size, int32_t *offset)
{
    const char *lcdText = NULL;

    /* check the query variable */
    if(REST.get_post_variable(request, "text", &lcdText) == 0) {

        /* No POST variable. Ignore */

    } else {

    	/* copy the text */
    	snprintf( lcdMsg, LCD_MSG_MAX, lcdText );

        if( lcdEn ) {
        	/* write to LCD */
        	lcdBufferClear( lcdBuf );
        	lcdBufferPrintString( lcdBuf, lcdMsg, 0, 0 );
        	lcdSendBuffer( lcdBuf );

        } else {
        	/* clear LCD */
        	lcdBufferClear( lcdBuf );
        	lcdSendBuffer( lcdBuf );
        }
    }
}
#endif /* #if COAP_SERVER_LCD_RES_EN */
