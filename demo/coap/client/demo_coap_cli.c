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
 *   \addtogroup emb6
 *      @{
 *      \addtogroup demo
 *      @{
 *      \addtogroup demo_coap
 *      @{
 *      \addtogroup demo_coap_client
 *      @{
*/
/*! \file   demo_coap_cli.c

 \author Peter Lehmann, peter.lehmann@hs-offenburg.de

 \brief  CoAP Client example application

 \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/

#include "emb6.h"
#include "bsp.h"
#include "er-coap.h"
#include "er-coap-engine.h"
#include "evproc.h"
#include "demo_coap_cli.h"

/*==============================================================================
                                     MACROS
 =============================================================================*/

#define     LOGGER_ENABLE        LOGGER_DEMO_COAP
#include    "logger.h"

#define SERVER_NODE(ipaddr)   uip_ip6addr(ipaddr, 0xfe80, 0x0000, 0x0000, 0x0000, 0x0250, 0xc2ff, 0xfea8, 0xdddd)
/* #define SERVER_NODE(ipaddr)   uip_ip6addr(ipaddr, 0xbbbb, 0, 0, 0, 0, 0, 0, 0x1) */

#define LOCAL_PORT      UIP_HTONS(COAP_DEFAULT_PORT + 1)
#define REMOTE_PORT     UIP_HTONS(COAP_DEFAULT_PORT)    /* CoAP Default Port: 5683 */

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

static struct etimer et;
static coap_packet_t request[1];      /* This way the packet can be treated as pointer as usual. */
uip_ipaddr_t server_ipaddr;

/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

void _demo_coapCl_callback (c_event_t c_event, p_data_t p_data);

/*==============================================================================
                                LOCAL CONSTANTS
 =============================================================================*/

#define TOGGLE_INTERVAL 2  /* interval for client requests */

/* Example URIs that can be queried. */
#define NUMBER_OF_URLS 2

/* leading and ending slashes only for demo purposes, get cropped automatically when setting the Uri-Path */
char *service_urls[NUMBER_OF_URLS] =
{ ".well-known/core", "actuators/LED_toggle"};

/*==============================================================================
                                LOCAL FUNCTIONS
 =============================================================================*/

/* This function is will be passed to coap_nonblocking_request() to handle responses. */
void _client_chunk_handler(void *response)
{
    const uint8_t *chunk;
    if(!response) {
        LOG_INFO("%s\n\r","Restart Timer (no response)");
    } else {
        LOG_INFO("%s\n\r","response payload:");
//        int len = coap_get_payload(response, &chunk);
//        printf("%d|%s", len, (char *)chunk);
//        printf("\n\r");
//        printf("\n\r");
    }
    /* Restart Timer after response or timeout */
    etimer_restart(&et);
}
//uint8_t i=0;

void _demo_coapCl_callback (c_event_t c_event, p_data_t p_data)
{
    if(etimer_expired(&et) && (&et == p_data)) {
        LOG_INFO("%s\n\r","--Toggle timer--");

//        if (i>0) {
            /* LED_Toggle Demo request */
            coap_init_message(request, COAP_TYPE_CON, COAP_POST, 0);
            coap_set_header_uri_path(request, service_urls[1]);
            coap_nonblocking_request(&server_ipaddr, REMOTE_PORT, request, _client_chunk_handler);
//            i=0;
//        } else {
//            /* Toggle LED demo post handler request */
//            coap_init_message(request, COAP_TYPE_CON, COAP_POST, 0);
//            coap_set_header_uri_path(request, service_urls[4]);
//            coap_nonblocking_request(&server_ipaddr, REMOTE_PORT, request, _client_chunk_handler);
//            i++;
//        }
   }
}

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

/*==============================================================================
 demo_coapInit()
==============================================================================*/

int8_t demo_coapInit(void)
{
    LOG_INFO("%s\n\r","Starting CoAP Example Client");
    SERVER_NODE(&server_ipaddr);

    /* receives all CoAP messages */
    coap_init_engine();

    /* set request intervall */
    etimer_set(&et, TOGGLE_INTERVAL * bsp_get(E_BSP_GET_TRES), _demo_coapCl_callback);
    return 1;
}

/*==============================================================================
 demo_coapConf()
==============================================================================*/

uint8_t demo_coapConf(s_ns_t* pst_netStack)
{
    uint8_t c_ret = 1;

    /*
     * By default stack
     */
    if (pst_netStack != NULL) {
        if (!pst_netStack->c_configured) {
            pst_netStack->hc     = &sicslowpan_driver;
            pst_netStack->llsec   = &nullsec_driver;
            pst_netStack->hmac   = &nullmac_driver;
            pst_netStack->lmac   = &sicslowmac_driver;
            pst_netStack->frame  = &framer_802154;
            pst_netStack->c_configured = 1;
            /* Transceiver interface is defined by @ref board_conf function*/
            /*pst_netStack->inif   = $<some_transceiver>;*/
        } else {
            if ((pst_netStack->hc == &sicslowpan_driver)   &&
                (pst_netStack->llsec == &nullsec_driver)   &&
                (pst_netStack->hmac == &nullmac_driver)    &&
                (pst_netStack->lmac == &sicslowmac_driver) &&
                (pst_netStack->frame == &framer_802154)) {
                /* right configuration */
            }
            else {
                pst_netStack = NULL;
                c_ret = 0;
            }
        }
    }
    return (c_ret);
}

/** @} */
/** @} */
/** @} */
/** @} */
