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
 * \file    demo_udp_socket.c
 * \author  Phuong Nguyen
 * \version v0.0.2
 *
 * \brief   UDP demo application using emb6 stack
 */

/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"

#include "bsp.h"
#include "etimer.h"
#include "evproc.h"

#include "tcpip.h"
#include "uip.h"
#include "uiplib.h"
#include "uip-debug.h"
#include "uip-udp-packet.h"
#include "uip-ds6.h"
#include "rpl.h"

#include "demo_udp_socket.h"

#define     LOGGER_ENABLE       LOGGER_DEMO_UDP_SOCKET
#if         LOGGER_ENABLE   ==  TRUE
#define     LOGGER_SUBSYSTEM    "UDP Socket"
#endif
#include    "logger.h"


/*
********************************************************************************
*                               LOCAL MACROS
********************************************************************************
*/
#define DEF_UDP_MAX_PAYLOAD_LEN         40U

#ifdef  DEMO_UDP_SOCKET_ROLE_SERVER
#define DEF_UDP_DEVPORT                 4211UL
#define DEF_UDP_REMPORT                 4233UL
#endif

#ifdef  DEMO_UDP_SOCKET_ROLE_CLIENT
#define DEF_UDP_DEVPORT                 4233UL
#define DEF_UDP_REMPORT                 4211UL
#endif

#define DEF_UDP_SEND_INTERVAL           (clock_time_t)( 500u )

#define UIP_IP_BUF                      ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static struct uip_udp_conn *UDPSocket_Conn = (struct uip_udp_conn *) 0;

#ifdef  DEMO_UDP_SOCKET_ROLE_CLIENT
static uint32_t UDPSocket_CurrSeqTx;
static uint32_t UDPSocket_LastSeqRx;
static uint32_t UDPSocket_LostPktQty;
static struct etimer UDPSocket_Etimer;
#endif /* DEMO_UDP_SOCKET_ROLE_CLIENT */


/*=============================================================================
 *                              LOCAL FUNCTION PROTOTYPE
 *============================================================================*/
static void     UDPSocket_EventHandler(c_event_t c_event, p_data_t p_data);
static void     UDPSocket_Tx(uint32_t seq);
static uint32_t UDPSocket_GetSeq(uint8_t *p_data, uint16_t len);


/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/

/**
 * @brief   Achieve sequence number of an UDP message
 */
static uint32_t UDPSocket_GetSeq(uint8_t *p_data, uint16_t len)
{
    uint32_t ret = 0u;
    uint8_t is_valid = 0u;


    is_valid = (p_data != NULL) &&
               (len > 0);
    if (is_valid == 1u) {
        memcpy(&ret, p_data, sizeof(ret));
    }

    return ret;
}


/**
 * @brief   UDP socket transmission
 *
 * @param   seq     Sequence number of the UDP message to transmit
 */
static void UDPSocket_Tx(uint32_t seq)
{
    uint8_t     seq_size = sizeof(seq);
    uint8_t     payload[] = { 0, 0, 0, 0, 0x50, 0x51, 0x52, 0x53, 0x54 };
    uint16_t    payload_len = sizeof(payload);


    /*
     * create packet to send
     */
    memcpy(payload, &seq, seq_size);

#ifdef DEMO_UDP_SOCKET_ROLE_CLIENT
    uip_ds6_addr_t  *ps_src_addr;
    rpl_dag_t       *ps_dag_desc;

    /*
     * Get Server IP address
     */
    ps_src_addr = uip_ds6_get_global(ADDR_PREFERRED);
    if (ps_src_addr) {
        ps_dag_desc = rpl_get_any_dag();
        if (ps_dag_desc) {
            /*
             * Issue transmission request
             */
            uip_ipaddr_copy(&UDPSocket_Conn->ripaddr, &ps_src_addr->ipaddr);
            memcpy(&UDPSocket_Conn->ripaddr.u8[8], &ps_dag_desc->dag_id.u8[8], 8);

            uip_udp_packet_send(UDPSocket_Conn, payload, payload_len);
            uip_create_unspecified(&UDPSocket_Conn->ripaddr);
        }
    }
#endif


#ifdef DEMO_UDP_SOCKET_ROLE_SERVER
    uip_ipaddr_copy(&UDPSocket_Conn->ripaddr,
                    &UIP_IP_BUF->srcipaddr);

    uip_udp_packet_send(UDPSocket_Conn,
                        payload,
                        payload_len);

    uip_create_unspecified(&UDPSocket_Conn->ripaddr);
#endif /* DEMO_UDP_SOCKET_ROLE_SERVER */


    /*
     * Logging
     */
#if LOGGER_ENABLE
    LOG_RAW("UDP Sending...  : ");
    uint16_t len = payload_len;
    uint8_t *p_data = payload;
    while (len--) {
        LOG_RAW("%02x ", *p_data++);
    }
    LOG_RAW("\r\n=========================\r\n");
#endif
}


/**
 * @brief   UDP event handler function
 *
 * @param   c_event     Event to be processed
 * @param   p_data      Callback argument that was registered as the event was
 *                      raised
 */
static void UDPSocket_EventHandler(c_event_t c_event, p_data_t p_data)
{
    uint32_t    seq;
    uint16_t    len;
    uint8_t     has_data;
    uint8_t    *p_dataptr;


    /*
     * process input TCPIP packet
     */
    if (c_event == EVENT_TYPE_TCPIP) {
        has_data = uip_newdata();
        if (has_data != 0u) {
            p_dataptr = (uint8_t *)uip_appdata;
            len = (uint16_t)uip_datalen();

            /* Obtain sequence number of the received message */
            seq = UDPSocket_GetSeq(p_dataptr, len);

#if LOGGER_ENABLE
            /* Logging */
            LOG_RAW("UDP Receiving...  : ");
            while (len--) {
                LOG_RAW("%02x ", *p_dataptr++);
            }
            LOG_RAW("\r\n");
#endif

#ifdef DEMO_UDP_SOCKET_ROLE_SERVER
            //UDPSocket_Tx(seq);
#else
            /* TODO compare sequence number */
#endif /* DEMO_UDP_SOCKET_ROLE_SERVER */
        }
    }
#ifdef DEMO_UDP_SOCKET_ROLE_CLIENT
    else {
        int err = 0;

        err = etimer_expired(&UDPSocket_Etimer);
        if (err != 0) {
            /*
             * Restart event timer
             */
            etimer_restart(&UDPSocket_Etimer);

            /*
             * Sends a UDP message
             */
            UDPSocket_Tx(UDPSocket_CurrSeqTx);

            /*
             * Increase sequence number of UDP message
             */
            if (++UDPSocket_CurrSeqTx >= 1000) {
                UDPSocket_CurrSeqTx = 0;
            }
        }
    }
#endif
}


/*
********************************************************************************
*                           API FUNCTION DEFINITIONS
********************************************************************************
*/

/**
 * @brief   Initialize UDP Socket demo application
 * @return
 */
int8_t demo_udpSocketInit(void)
{
    /*
     * Create a new UDP connection
     */
    UDPSocket_Conn = udp_new(NULL,
                      UIP_HTONS(DEF_UDP_REMPORT),
                      NULL);

    /*
     * Bind the UDP connection to a local port
     */
    udp_bind(UDPSocket_Conn,
             UIP_HTONS(DEF_UDP_DEVPORT));

    /* set callback for event process */
    evproc_regCallback(EVENT_TYPE_TCPIP,
                       UDPSocket_EventHandler);

#ifdef DEMO_UDP_SOCKET_ROLE_CLIENT
    clock_time_t interval = 0;

    /* set UDP event timer interval */
    interval  = DEF_UDP_SEND_INTERVAL;
    interval *= bsp_get(E_BSP_GET_TRES)/1000;

    /* Set event timer for periodic data process */
    etimer_set(&UDPSocket_Etimer,
                interval,
                &UDPSocket_EventHandler);

    /* initialize statistic */
    UDPSocket_CurrSeqTx = 0u;
    UDPSocket_LastSeqRx = 0u;
    UDPSocket_LostPktQty = 0u;
#endif

    /* Always success */
    return 1;
}


/**
 * @brief   Configure UDP socket demo application
 *
 * @param   p_netstk    Pointer to net stack structure
 * @return
 */
int8_t demo_udpSocketCfg(s_ns_t *p_netstk)
{
    int8_t i_ret = -1;


    if (p_netstk != NULL) {
        if (!p_netstk->c_configured) {
            p_netstk->hc    = &sicslowpan_driver;
            p_netstk->frame = &framer_802154;
            p_netstk->dllsec = &nullsec_driver;
            i_ret = 1;
        } else {
            if ((p_netstk->hc    == &sicslowpan_driver) &&
                (p_netstk->frame == &framer_802154)     &&
                (p_netstk->dllsec == &nullsec_driver)) {
                i_ret = 1;
            }
            else {
                p_netstk = NULL;
                i_ret = 0;
            }
        }
    }
    return i_ret;
}
