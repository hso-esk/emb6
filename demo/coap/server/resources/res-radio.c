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
/*============================================================================*/
/**
 *      \addtogroup emb6
 *      @{
 *      \addtogroup demo_coap
 *      @{
 *      \addtogroup demo_coap_server
 *      @{
*/
/*! \file   res-rf.c

 \author Peter Lehmann
 \author Edgar Schmitt
 \author Artem Yushev

 \brief  Example resourse

 \version 0.0.1
 */
/*============================================================================*/
/*==============================================================================
 INCLUDE FILES
 =============================================================================*/

#include "er-coap.h"
#include "emb6.h"

/*==============================================================================
 MACROS
 =============================================================================*/
#define     LOGGER_ENABLE        LOGGER_DEMO_COAP
#include    "logger.h"

/*==============================================================================
 ENUMS
 =============================================================================*/

/*==============================================================================
 LOCAL CONSTANTS
 =============================================================================*/

/*==============================================================================
 LOCAL FUNCTION PROTOTYPES
 =============================================================================*/
static void     _res_get_handler(void *request, void *response,
                            uint8_t *buffer, uint16_t preferred_size,
                            int32_t *offset);
static void     _res_postput_handler(void *request, void *response,
                            uint8_t *buffer, uint16_t preferred_size,
                            int32_t *offset);
static uint8_t _res_setTxPwr(const char* rpc_data, uint8_t c_len, void* p_resp);
static uint8_t _res_setRxSens(const char* rpc_data, uint8_t c_len, void* p_resp);


/*==============================================================================
 LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
RESOURCE(res_radio_info, "title=\"RADIO: ?p=rssi|pwr|sens\";rt=\"Radio\"",
         _res_get_handler, NULL, NULL, NULL);

RESOURCE(res_radio_ctrl, "title=\"RADIO: POST/PUT pwr=<dB> sens=<dB>:\";rt=\"dB\"",
         NULL,
         _res_postput_handler,
         _res_postput_handler,
         NULL);

RESOURCE(res_radio_txpwr, "title=\"RADIO: tx power:\";rt=\"Control\"",
         NULL,
         _res_postput_handler,
         _res_postput_handler,
         NULL);

/*==============================================================================
 LOCAL FUNCTIONS
 =============================================================================*/
static void _res_get_handler(void *request,   void *response,
                            uint8_t *buffer, uint16_t preferred_size,
                            int32_t *offset)
{
    uint8_t     c_length = 0;
    const char* p = NULL;
    char        rpc_param[64];
    int8_t      c_success = 1;
    e_nsErr_t  err = NETSTK_ERR_NONE;
    uint8_t     rssi;
    uint8_t     txpower;
    uint8_t     sensitivity;


    LOG2_INFO("Enter _res_get_handler() function");

    s_ns_t* ps_ns = NULL;
    ps_ns = emb6_get();

    unsigned int i_accept = -1;
    REST.get_header_accept(request, &i_accept);

    if((c_length = REST.get_query_variable(request, "p", &p))) {
        ps_ns->rf->ioctrl(NETSTK_CMD_RF_RSSI_GET, &rssi, &err);
        ps_ns->rf->ioctrl(NETSTK_CMD_RF_SENS_GET, &sensitivity, &err);
        ps_ns->rf->ioctrl(NETSTK_CMD_RF_TXPOWER_GET, &txpower, &err);

        if(strncmp(p, "rssi", c_length) == 0) {
            snprintf(rpc_param, 5, "%d", rssi);
            LOG1_INFO("GET request for RSSI: %s", rpc_param);
        } else if(strncmp(p, "pwr", c_length) == 0) {
            snprintf(rpc_param, 5, "%d", txpower);
            LOG1_INFO("GET request for  TX_PWR: %s", rpc_param);
        } else if(strncmp(p, "sens", sensitivity) == 0) {
            snprintf(rpc_param, 5, "%d", txpower);
            LOG1_INFO("GET request for  RX_SENS: %s", rpc_param);
        } else {
            snprintf(rpc_param, 64 ,"RSSI [%]: TX_PWR [%d]: RX_SENS [%d]",
                     rssi, txpower, sensitivity);
        }
    } else {
        c_success = 0;
    }

    if (c_success && (ps_ns != NULL) && (ps_ns->rf != NULL))
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
        REST.set_response_status(response, REST.status.BAD_REQUEST);
    }

    LOG2_INFO("Leave _res_get_handler() function");
}

static uint8_t _res_setTxPwr(const char* rpc_data, uint8_t c_len, void* p_resp)
{
    uint8_t     c_ret = 1;
    int8_t      tx_pwr = 0;
    s_ns_t*     ps_ns = emb6_get();
    char        param[6];
    uint8_t     i = 0;
    e_nsErr_t  err = NETSTK_ERR_NONE;
    uint8_t     act_txpower;


    if ((ps_ns == NULL) || (ps_ns->rf == NULL))
    {
        LOG_ERR("POST method failed. Stack pointers NULL");
        REST.set_response_status(p_resp, INTERNAL_SERVER_ERROR_5_00);
        c_ret = 0;
    }

    if (c_ret) {
        for (i=0;i<6;i++) {
            param[i] = rpc_data[i];
        }
        tx_pwr = (int8_t)atoi(param);

        LOG1_INFO("Received POST request to change tx_pwr %s:[%d]",param,tx_pwr);

        if (tx_pwr > -20) {
            LOG1_INFO("Guards passed, try to change parameters");
            ps_ns->rf->off(&err);
            ps_ns->rf->ioctrl(NETSTK_CMD_RF_TXPOWER_SET, &tx_pwr, &err);
            ps_ns->rf->on(&err);

            ps_ns->rf->ioctrl(NETSTK_CMD_RF_TXPOWER_GET, &act_txpower, &err);
            if (tx_pwr != act_txpower) {
                LOG_ERR("POST method failed. Value wasn't changed");
                REST.set_response_status(p_resp, INTERNAL_SERVER_ERROR_5_00);
                c_ret = 0;
            } else {
                LOG1_OK("POST method succeed");
                REST.set_response_status(p_resp, REST.status.CHANGED);
            }
        } else {
            LOG_ERR("POST method failed. TX power too small");
            REST.set_response_status(p_resp, PRECONDITION_FAILED_4_12);
            c_ret = 0;
        }


    }
    return (c_ret);
}

static uint8_t _res_setRxSens(const char* rpc_data, uint8_t c_len, void* p_resp)
{
    uint8_t     c_ret = 1;
    int8_t      rx_sens = 0;
    char        param[6];
    uint8_t     i;
    s_ns_t*     ps_ns = emb6_get();
    e_nsErr_t  err = NETSTK_ERR_NONE;
    uint8_t     act_rx_sens;


    if ((ps_ns == NULL) || (ps_ns->rf == NULL))
    {
        LOG_ERR("POST method failed. Stack pointers NULL");
        REST.set_response_status(p_resp, INTERNAL_SERVER_ERROR_5_00);
        c_ret = 0;
    }

    if (c_ret)
    {
        for (i=0;i<6;i++) {
            param[i] = rpc_data[i];
        }
        rx_sens = (int8_t)atoi(param);

        LOG1_INFO("Received POST request to change rx_sens %s:[%d]",param,rx_sens);

        if (rx_sens < 0) {
            LOG1_INFO("Guards passed, try to change parameters");
            ps_ns->rf->off(&err);
            ps_ns->rf->ioctrl(NETSTK_CMD_RF_SENS_SET, &rx_sens, &err);
            ps_ns->rf->on(&err);


            ps_ns->rf->ioctrl(NETSTK_CMD_RF_SENS_GET, &act_rx_sens, &err);
            if (rx_sens != act_rx_sens) {
                LOG_ERR("POST method failed. Value wasn't changed");
                REST.set_response_status(p_resp, INTERNAL_SERVER_ERROR_5_00);
                c_ret = 0;
            } else {
                LOG1_OK("POST method succeed");
                REST.set_response_status(p_resp, REST.status.CHANGED);
            }
        } else {
            LOG_ERR("POST method failed. rx_sens should be less than 0");
            REST.set_response_status(p_resp, PRECONDITION_FAILED_4_12);
            c_ret = 0;
        }
    }

    return (c_ret);
}

static void _res_postput_handler(void *request,   void *response,
                                 uint8_t *buffer, uint16_t preferred_size,
                                 int32_t *offset)
{
    const char*             p = NULL;
    uint8_t                 c_ret = 0;
    uint8_t                 c_len = 0;

    LOG2_INFO("Enter _res_postput_handler() function");

    if((c_len = REST.get_query_variable(request, "pwr", &p)) > 0) {
        LOG2_INFO("POST tx power %s[%d]",p,c_len);
        c_ret = _res_setTxPwr(p,c_len,response);
    }

    if((c_len = REST.get_query_variable(request, "sens", &p)) > 0) {
        LOG2_INFO("POST rx sens %s[%d]",p,c_len);
        c_ret = _res_setRxSens(p, c_len, response);
    }

    if (!c_ret) {
        LOG_ERR("POST method failed. Error in request processing");
        REST.set_response_status(response, PRECONDITION_FAILED_4_12);
    }

    LOG2_INFO("Leave _res_postput_handler() function");
}
