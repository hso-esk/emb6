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
#define     __CLIENT_PORT           4124
/** Communication port for server (not for CoAP) */
#define     __SERVER_PORT           4123

/** Define a network prefix for all addresses */
#define     NETWORK_PREFIX          0x2001, 0xbbbb, 0xdddd, 0x0000
/** Mode 3 - derived from server link-local (MAC) address */
#define     SERVER_IP_ADDR80        0x0250, 0xc2ff, 0xfea8, 0xbabe
/** Server IP address consist of network prefix and 64 bit of mac address */
#define     SERVER_IP_ADDR          NETWORK_PREFIX,SERVER_IP_ADDR80


/* External global buffer for data for packet storing (why not use packetbuf.c?)*/
#define     UIP_IP_BUF              ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define     _QUOTEME(x)             #x
#define     QUOTEME(x)              _QUOTEME(x)
#define     EMB6_DEMO_APTB_CODE     0x10
/*==============================================================================
                                         ENUMS
 =============================================================================*/

/*==============================================================================
                          STRUCTURES AND OTHER TYPEDEFS
 =============================================================================*/
static  struct  udp_socket          st_udp_socket;
static  struct  udp_socket          *pst_udp_socket;

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
static  void        loc_aptb_sendMsg(uint32_t l_seqID);
//static  char*       loc_aptb_addAddr(char * pc_buf,const uip_ipaddr_t* rs_addr);
static  uint64_t    loc_aptb_str2Seq(char * pc_str);

/*==============================================================================
                                    LOCAL FUNCTIONS
 =============================================================================*/

/*static  char*       loc_aptb_addAddr(char * pc_buf,const uip_ipaddr_t* rs_addr)
{
    uint16_t      a;
    uint16_t      i;
    int16_t       f;

    char *pc_addr = pc_buf;

    for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (rs_addr->u8[i] << 8) + rs_addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) {
          pc_addr += sprintf(pc_addr, "::");
      }
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
          pc_addr += sprintf(pc_addr, ":");
      }
      pc_addr += sprintf(pc_addr, "%04x", a);
    }
    }
    return pc_addr;
}
*/
/*----------------------------------------------------------------------------*/
/** \brief  This function is sending message with a sequence number.
 *
 *  \param  none
 *  \param    none
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static  void        loc_aptb_sendMsg(uint32_t l_seqID)
{
    char         pc_buf[MAX_S_PAYLOAD_LEN];

    pc_buf[0] = EMB6_DEMO_APTB_CODE;

    sprintf(pc_buf+1, "%lu | ", l_seqID);

    if (l_seqID != 0)
        l_expSeqID++;

    LOG_INFO("Lost packets (%lu)", l_expSeqID - l_seqID);
    LOG_INFO("Send message: %s",pc_buf);

    udp_socket_sendto(pst_udp_socket,
                      pc_buf, strlen(pc_buf),
                      &un_server_ipaddr,
                      __SERVER_PORT);

} /* loc_aptb_sendMsg */

static  uint64_t    loc_aptb_str2Seq(char * str)
{
    char temp[20];
    uint8_t i = 0;
    //uint8_t j = 0;

    for (i=uip_datalen();i>0;i--) {
        if (str[i] == '|' ) {
            i--;
            memcpy(temp,str,i);
            temp[i] = '\0';
            break;
        }
    }
    return atol(temp);
} /* loc_aptb_str2Seq() */

/*----------------------------------------------------------------------------*/
/** \brief  This function is called whenever a tcpip event occurs or
 *             one of a timers are expired.
 *
 *  \param  event     Event type
 *  \param    data    Pointer to data
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static    void      loc_aptb_callback(c_event_t c_event, p_data_t p_data)
{
    char *pc_str;
    if (etimer_expired(&s_et)) {
        loc_aptb_sendMsg(l_lastSeqId);
        etimer_restart(&s_et);
    } else if (c_event == EVENT_TYPE_TCPIP) {
        if (uip_newdata()) {
            pc_str = uip_appdata;
            if (pc_str[0] == EMB6_DEMO_APTB_CODE) {
                pc_str[uip_datalen()] = '\0';
                pc_str++;
                LOG_INFO("Packet from a server: '%s'", pc_str);
                l_lastSeqId = loc_aptb_str2Seq(pc_str);
            }
        }
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

    /* set the pointer to the udp-socket */
    pst_udp_socket = &st_udp_socket;
    udp_socket_register(pst_udp_socket, NULL, NULL);
    udp_socket_bind(pst_udp_socket, UIP_HTONS(__CLIENT_PORT));

    LOG_INFO("%s", "Create connection with the server ");
    LOG_IP6ADDR(&un_server_ipaddr.u8);
    LOG_RAW("\n\r");
    LOG_INFO("local/remote port %u/%u",
            UIP_HTONS(pst_udp_socket->udp_conn->lport),
            UIP_HTONS(pst_udp_socket->udp_conn->rport));
    //printf("Set dudp timer %p\n\r",&s_et);
    etimer_set(&s_et, SEND_INTERVAL, loc_aptb_callback);
    evproc_regCallback(EVENT_TYPE_TCPIP,loc_aptb_callback);
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
            pst_netStack->llsec  = &nullsec_driver;
            pst_netStack->hmac   = &nullmac_driver;
            pst_netStack->lmac   = &sicslowmac_driver;
            pst_netStack->frame  = &framer_802154;
            pst_netStack->c_configured = 1;
            /* Transceiver interface is defined by @ref board_conf function*/
            /* pst_netStack->inif   = $<some_transceiver>;*/
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
}/* demo_aptbConf()  */
/** @} */
/** @} */
/** @} */
/** @} */
