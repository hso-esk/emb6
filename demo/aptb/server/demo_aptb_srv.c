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
 *      \addtogroup demo
 *      @{
 *      \addtogroup demo_aptb
 *      @{
 *      \addtogroup demo_aptb_server
 *      @{
*/
/*! \file   demo_aptb_srv.c

 \author Artem Yushev, 

 \brief  Demo profile for testcases on Automated Physical Testbed (APTB) .

 \version 0.1
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/

#include "emb6.h"
#include "bsp.h"
#include "demo_aptb.h"
#include "etimer.h"
#include "evproc.h"
#include "tcpip.h"
#include "udp-socket.h"

/*==============================================================================
                                         MACROS
 =============================================================================*/
#define     LOGGER_ENABLE        LOGGER_DEMO_APTB
#include    "logger.h"

#define     MAX_S_PAYLOAD_LEN        256
#define     MAX_C_PAYLOAD_LEN        40

/** Communication port for client (not for CoAP) */
#define     _LISTEN_PORT             4124
/** Communication port for server (not for CoAP) */
#define     _LOCAL_PORT              4123

#define     EMB6_APTB_REQUEST        0x10
#define     EMB6_APTB_RESPONSE       0x11
/*==============================================================================
                                         ENUMS
 =============================================================================*/

/*==============================================================================
                          STRUCTURES AND OTHER TYPEDEFS
 =============================================================================*/
static  struct  udp_socket          st_udp_socket;
/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
/*==============================================================================
                                    LOCAL CONSTANTS
 =============================================================================*/

/*==============================================================================
                               LOCAL FUNCTION PROTOTYPES
 =============================================================================*/
static void  _aptb_callback(struct udp_socket *c, void *ptr,
             const uip_ipaddr_t *source_addr, uint16_t source_port,
             const uip_ipaddr_t *dest_addr,   uint16_t dest_port,
             const uint8_t *data,             uint16_t datalen);
/*==============================================================================
                                    LOCAL FUNCTIONS
 =============================================================================*/


/*----------------------------------------------------------------------------*/
/** \brief  This function is called if a new packet was received. Format is the
 *          the same as @ref udp_socket_input_callback_t
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static void  _aptb_callback(struct udp_socket *c, void *ptr,
             const uip_ipaddr_t *source_addr, uint16_t source_port,
             const uip_ipaddr_t *dest_addr,   uint16_t dest_port,
             const uint8_t *data,             uint16_t datalen)
{
    char        pc_buf[MAX_S_PAYLOAD_LEN];
    uint64_t    seqID;

    if (data != NULL)
    {
        if (data[0] == EMB6_APTB_REQUEST) {
            seqID = (data[1] << 24) + (data[2] << 16) +
                    (data[3] << 8)  + data[4];
            LOG_INFO("Request sequence: [%lu]", seqID);
            seqID++;

            pc_buf[0] = EMB6_APTB_RESPONSE;
            pc_buf[1] = seqID >> (24);
            pc_buf[2] = (seqID & 0x00ff0000) >> (16);
            pc_buf[3] = (seqID & 0x0000ff00) >> (8);
            pc_buf[4] = (seqID & 0xff);

            LOG_INFO("Response from server: [%lu]",seqID);
            udp_socket_sendto(&st_udp_socket, pc_buf, 5,
                              source_addr, source_port);
        } else {
            LOG_ERR("Error in parsing: invalid packet format");
        }
    } else {
        LOG_ERR("Error in reception: data is NULL");
    }

}

/*==============================================================================
                                         API FUNCTIONS
 =============================================================================*/
/*----------------------------------------------------------------------------*/
/*    demo_aptbInit()                                                           */
/*----------------------------------------------------------------------------*/
int8_t demo_aptbInit(void)
{
    udp_socket_register(&st_udp_socket, NULL, _aptb_callback);
    udp_socket_bind(&st_udp_socket, _LOCAL_PORT);
    /* works like "listen" */
    udp_socket_connect(&st_udp_socket, NULL, _LISTEN_PORT);
    LOG_INFO("local port %u", _LOCAL_PORT);
    LOG_INFO("APTB demo initialized, waiting for connection...");
    return 1;
}/* demo_aptbInit()  */

/*----------------------------------------------------------------------------*/
/*    demo_aptbConf()                                                           */
/*----------------------------------------------------------------------------*/
uint8_t demo_aptbConf(s_ns_t* pst_netStack)
{
    uint8_t c_ret = 1;

    /*
     * By default stack
     */
    if (pst_netStack != NULL) {
        if (!pst_netStack->c_configured) {
            pst_netStack->hc     = &sicslowpan_driver;
            pst_netStack->dllsec  = &nullsec_driver;
            pst_netStack->frame  = &framer_802154;
            pst_netStack->c_configured = 1;
            /* Transceiver interface is defined by @ref board_conf function*/
            /* pst_netStack->inif   = $<some_transceiver>;*/
        } else {
            if ((pst_netStack->hc == &sicslowpan_driver)   &&
                (pst_netStack->dllsec == &nullsec_driver)   &&
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
}/* demo_aptbConf()  */
/** @} */
/** @} */
/** @} */
/** @} */
