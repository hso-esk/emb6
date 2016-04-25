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
 *      \addtogroup demo_aptb_client
 *      @{
*/
/*! \file   demo_aptb_cli.c

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
#include "rpl.h"

/*==============================================================================
                                         MACROS
 =============================================================================*/
#define     LOGGER_ENABLE           LOGGER_DEMO_APTB
#include    "logger.h"


#define     SEND_INTERVAL           5 * bsp_get(E_BSP_GET_TRES)
#define     MAX_S_PAYLOAD_LEN       256
#define     MAX_C_PAYLOAD_LEN       40

/** Communication port for client (not for CoAP) */
#define     _LOCAL_PORT             4124
/** Communication port for server (not for CoAP) */
#define     _REMOTE_PORT            4123

/** Define a network prefix for all addresses */
#define     NETWORK_PREFIX          0xaaaa, 0x0000, 0x0000, 0x0000
/** Mode 3 - derived from server link-local (MAC) address */
#define     SERVER_IP_ADDR80        0x0250, 0xc2ff, 0xfea8, 0x00AA
/** Server IP address consist of network prefix and 64 bit of mac address */
#define     SERVER_IP_ADDR          NETWORK_PREFIX,SERVER_IP_ADDR80


#define     EMB6_APTB_REQUEST       0x10
#define     EMB6_APTB_RESPONSE      0x11
/*==============================================================================
                                         ENUMS
 =============================================================================*/

/*==============================================================================
                          STRUCTURES AND OTHER TYPEDEFS
 =============================================================================*/
static  struct  udp_socket          st_udp_socket;

static          uip_ipaddr_t        un_server_ipaddr = {.u16={SERVER_IP_ADDR} };
        struct  etimer              s_et;
static          uint32_t            l_lastSeqId = 0;
static          uint32_t            l_expSeqID = 0;

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
/*==============================================================================
                                    LOCAL CONSTANTS
 =============================================================================*/

/*==============================================================================
                               LOCAL FUNCTION PROTOTYPES
 =============================================================================*/
static  void        _aptb_sendMsg(uint32_t l_seqID);
static  void        _send_msg_tout(c_event_t c_event, p_data_t p_data);
static  void        _aptb_callback(struct udp_socket *c, void *ptr,
                    const uip_ipaddr_t *source_addr, uint16_t source_port,
                    const uip_ipaddr_t *dest_addr,   uint16_t dest_port,
                    const uint8_t *data,             uint16_t datalen);

/*==============================================================================
                                    LOCAL FUNCTIONS
 =============================================================================*/

/*----------------------------------------------------------------------------*/
/** \brief  This function is sending message with a sequence number.
 *
 *  \param  none
 *  \param    none
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static  void        _aptb_sendMsg(uint32_t l_seqID)
{
    char         pc_buf[MAX_S_PAYLOAD_LEN];

    pc_buf[0] = EMB6_APTB_REQUEST;
    pc_buf[1] = l_seqID >> (24);
    pc_buf[2] = (l_seqID & 0x00ff0000) >> (16);
    pc_buf[3] = (l_seqID & 0x0000ff00) >> (8);
    pc_buf[4] = (l_seqID & 0xff);

    if (l_seqID != 0)
        l_expSeqID++;

    LOG_INFO("Lost packets [%lu]", l_expSeqID - l_seqID);
    LOG_INFO("Request sequence: [%lu]", l_seqID);

    udp_socket_send(&st_udp_socket, pc_buf, 5);

} /* _aptb_sendMsg */

/*----------------------------------------------------------------------------*/
/** \brief  This function is called if a timer expired.
 *
 *  \param  event     Event type
 *  \param  data      Pointer to data
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static void  _send_msg_tout(c_event_t c_event, p_data_t p_data)
{
    if (etimer_expired(&s_et)) {
        _aptb_sendMsg(l_lastSeqId);
        etimer_restart(&s_et);
    }
}

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
    if (data != NULL)
    {

    	printf("data received length : %i \n", datalen);
        if (data[0] == EMB6_APTB_RESPONSE) {
            /* Skip packet type header */
            l_lastSeqId = (data[1] << 24) +
                          (data[2] << 16) +
                          (data[3] << 8) +
                          data[4];
            LOG_INFO("Response from a server: [%lu]", l_lastSeqId);
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

	static  uip_ipaddr_t                s_destAddr;
    /* The choice of server address determines its 6LoPAN header compression.
    * (Our address will be compressed Mode 3 since it is derived from our link-local address)
    * Obviously the choice made here must also be selected in udp-server.c.
    *
    * For correct Wireshark decoding using a sniffer, add the /64 prefix to the 6LowPAN protocol preferences,
    * e.g. set Context 0 to aaaa::.  At present Wireshark copies Context/128 and then overwrites it.
    * (Setting Context 0 to aaaa::1111:2222:3333:4444 will report a 16 bit compressed address of aaaa::1111:22ff:fe33:xxxx)
    *
    * Note the IPCMV6 checksum verification depends on the correct uncompressed addresses.
    */
    /* We know destination IP address but we need to properly convert it */
    uip_ip6addr(&un_server_ipaddr,
                un_server_ipaddr.u16[0],un_server_ipaddr.u16[1],\
                un_server_ipaddr.u16[2],un_server_ipaddr.u16[3],\
                un_server_ipaddr.u16[4],un_server_ipaddr.u16[5],\
                un_server_ipaddr.u16[6],un_server_ipaddr.u16[7]);

    udp_socket_register(&st_udp_socket, NULL, _aptb_callback);
    udp_socket_bind(&st_udp_socket, _LOCAL_PORT);

    uip_ip6addr(&s_destAddr, 0xff02, 0, 0, 0, 0, 0, 0, 0x0001);
    udp_socket_connect(&st_udp_socket,&s_destAddr, _REMOTE_PORT);

    LOG_INFO("%s", "Create connection with the server ");
    LOG_IP6ADDR(&un_server_ipaddr.u8);
    LOG_RAW("\n\r");
    LOG_INFO("local/remote port %u/%u",
            UIP_HTONS(st_udp_socket.udp_conn->lport),
            UIP_HTONS(st_udp_socket.udp_conn->rport));
    //printf("Set dudp timer %p\n\r",&s_et);
    etimer_set(&s_et, SEND_INTERVAL, _send_msg_tout);
    LOG_INFO("APTB demo initialized, Connecting ...");
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
