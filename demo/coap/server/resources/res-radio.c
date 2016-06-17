/*
 * emb6 is licensed under the 3-clause BSD license. This license gives everyone
 * the right to use and distribute the code, either in binary or source code
 * format, as long as the copyright license is retained in the source code.
 *
 * The emb6 is derived from the Contiki OS platform with the explicit approval
 * from Adam Dunkels. However, emb6 is made independent from the OS through the
 * removal of protothreads. In addition, APIs are made more flexible to gain
 * more adaptivity during run-time.
 *
 * The license text is:
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "er-coap.h"
#include "emb6.h"

/*
********************************************************************************
*                                    MACROS
********************************************************************************
*/
#define     LOGGER_ENABLE        LOGGER_DEMO_COAP
#include    "logger.h"

/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
/* Handler for GET actions. For further details see the function definition */
static void res_get_handler(void *request, void *response,
        uint8_t *buffer, uint16_t preferred_size,
        int32_t *offset);


/* Handler for POST/PUT actions. For further details see the function definition */
static void res_postput_handler(void *request, void *response,
        uint8_t *buffer, uint16_t preferred_size,
        int32_t *offset);

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
/* Resource definition of the main RADIO resource. This
 * returns the all information about the radio settings */
RESOURCE(res_radio_info,
        "title=\"Radio Info\";type=\"Info\"",
        res_get_handler,
        NULL,
        NULL,
        NULL);

/* Resource definition the main RADIO resource. This
 * allows to change the Tx-Power of the transceiver */
RESOURCE(res_radio_ctrl,
        "title=\"Radio Control\";type=\"Control\"",
         NULL,
         res_postput_handler,
         res_postput_handler,
         NULL);
/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
/**
 * \brief   Handler for getting data from the resource.
 *
 *          The RADIO resource can return the current info and settings of the
 *          transceiver. Using the query ?info=<param> it is possible to
 *          choose between RSSI(rssi), Tx-Power(pwr) and Rx-Sensivity (rx-sens).
 */
static void res_get_handler(void *request,   void *response,
        uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
    int len = 0;
    const char* info = NULL;
    char rpc_param[64];
    int8_t success = 1;
    unsigned int i_accept = -1;

    e_nsErr_t err = NETSTK_ERR_NONE;
    uint8_t rssi;
    uint8_t txpower;
    uint8_t sensitivity;

    /* get the current stack */
    s_ns_t* ps_ns = NULL;
    ps_ns = emb6_get();


    REST.get_header_accept(request, &i_accept);

    if( (ps_ns != NULL) && (ps_ns->rf != NULL) &&
        (len = REST.get_query_variable(request, "p", &info))) {

        /* info request found. Get all the information from the transceiver */
        ps_ns->rf->ioctrl(NETSTK_CMD_RF_RSSI_GET, &rssi, &err);
        ps_ns->rf->ioctrl(NETSTK_CMD_RF_SENS_GET, &sensitivity, &err);
        ps_ns->rf->ioctrl(NETSTK_CMD_RF_TXPOWER_GET, &txpower, &err);

        if(strncmp(info, "rssi", len) == 0) {
            snprintf(rpc_param, 5, "%d", rssi);
            LOG1_INFO("GET request for RSSI: %s", rpc_param);
        } else if(strncmp(info, "pwr", len) == 0) {
            snprintf(rpc_param, 5, "%d", txpower);
            LOG1_INFO("GET request for  TX_PWR: %s", rpc_param);
        } else if(strncmp(info, "sens", len) == 0) {
            snprintf(rpc_param, 5, "%d", sensitivity);
            LOG1_INFO("GET request for  RX_SENS: %s", rpc_param);
        } else {
            snprintf(rpc_param, 64 ,"RSSI [%d]: TX_PWR [%d]: RX_SENS [%d]",
                     rssi, txpower, sensitivity);
        }
    } else {
        /* invalid operation */
        success = 0;
    }

    if(success )
    {
        if(i_accept == -1 || i_accept == REST.type.TEXT_PLAIN)
        {
          REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
          snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "%s", rpc_param);
          REST.set_response_payload(response, (uint8_t *)buffer, strlen((char *)buffer));
        } else {
          REST.set_response_status(response, REST.status.NOT_ACCEPTABLE);
          const char *msg = "Supporting content-types text/plain and application/json";
          REST.set_response_payload(response, msg, strlen(msg));
        }
    } else {
        /* return invalid request */
        REST.set_response_status(response, REST.status.BAD_REQUEST);
    }
}

/**
 * \brief   Handler for setting the radio transmission power.
 *
 *          The RADIO resource allows to set the current transmission
 *          power and receiver sensivity in dBm. This can be done by using
 *          the query variables ?pwr=<power> and ?sens=<sensivity>.
 */
static void res_postput_handler(void *request, void *response,
        uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
    const char* p = NULL;
    int len = 0;
    int8_t val = 0;
    uint8_t success = 1;

    /* get the current stack */
    e_nsErr_t err = NETSTK_ERR_NONE;
    s_ns_t* ps_ns = NULL;
    ps_ns = emb6_get();

    if( (ps_ns != NULL) && (ps_ns->rf != NULL) &&
        (len = REST.get_query_variable(request, "pwr", &p)))
    {
        int8_t tmpVal = 0;

        /* Set Power ... get value from request */
        val = (int8_t)atoi(p);

        /* try to set the Tx Power and retrieve the value to check
         * if it worked. */
        ps_ns->rf->ioctrl(NETSTK_CMD_RF_TXPOWER_SET, &val, &err);

        if( err != NETSTK_ERR_NONE ) {
            /* an error occured when changing the tx power */
            REST.set_response_status(response, INTERNAL_SERVER_ERROR_5_00);
        }

        if( success )
        {
            /* check if the correct valu ewas set */
            ps_ns->rf->ioctrl(NETSTK_CMD_RF_TXPOWER_GET, &tmpVal, &err);

            if( (val != tmpVal) || (err != NETSTK_ERR_NONE) ) {
                REST.set_response_status(response, INTERNAL_SERVER_ERROR_5_00);
            }
        }

    } else if( (ps_ns != NULL) && (ps_ns->rf != NULL) &&
            (len = REST.get_query_variable(request, "sens", &p)))
    {
        int8_t tmpVal = 0;

        /* Set Power ... get value from request */
        val = (int8_t)atoi(p);

        /* try to set the Tx Power and retrieve the value to check
         * if it worked. */
        ps_ns->rf->ioctrl(NETSTK_CMD_RF_SENS_GET, &val, &err);

        if( err != NETSTK_ERR_NONE ) {
            /* an error occured when changing the tx power */
            REST.set_response_status(response, INTERNAL_SERVER_ERROR_5_00);
        }

        if( success )
        {
            /* check if the correct valu ewas set */
            ps_ns->rf->ioctrl(NETSTK_CMD_RF_TXPOWER_GET, &tmpVal, &err);

            if( (val != tmpVal) || (err != NETSTK_ERR_NONE) ) {
                REST.set_response_status(response, INTERNAL_SERVER_ERROR_5_00);
            }
        }
    }
}

