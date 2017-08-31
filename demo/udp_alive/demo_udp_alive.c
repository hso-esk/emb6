/**
 *      \addtogroup emb6
 *      @{
 *      \addtogroup demo
 *      @{
 *      \addtogroup demo_udp_alive
 *      @{
*/
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
/*! \file   demo_udp_alive.c

 \author Peter Lehmann, peter.lehmann@hs-offenburg.de
 \author Artem Yushev, 

 \brief  UDP Client Source for DODAG visualization on Cetic 6LBR

 \version 0.2
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/

#include "emb6.h"
#include "bsp.h"
#include "demo_udp_alive.h"
#include "etimer.h"
#include "evproc.h"
#include "tcpip.h"
#include "rpl.h"
#include "udp-socket.h"

/*==============================================================================
                                         MACROS
 =============================================================================*/
#define     LOGGER_ENABLE        LOGGER_DEMO_UDPIAA
#include    "logger.h"

/** set the send interval */
#define     SEND_INTERVAL               5

/** set the payload length */
#define     MAX_PAYLOAD_LEN             40

/** set the communication port (default for 6LBR: 3000) */
#define     _PORT                       3000
/** macro for a half of ipv6 address */
#define     HALFIP6ADDR                 8
/** macro for a half of ipv6 address */
#define     IP6ADDRSIZE                 16

/*==============================================================================
                     TYPEDEF'S DECLARATION
 =============================================================================*/
typedef enum e_udpAliveStateS {
    E_UDPALIVE_UNDEF,
    E_UDPALIVE_GETDESTADDR,
    E_UDPALIVE_CONNECT,
    E_UDPALIVE_PREPMSG,
    E_UDPALIVE_SENDMSG,
    E_UDPALIVE_DONE,
}e_udpAlive_t;

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
static  struct  udp_socket          st_udp_socket;
static  struct  udp_socket          *pst_udp_socket;

static  uip_ip6addr_t               s_destAddr;

struct  etimer                      e_udpAliveTmr;

/*==============================================================================
                               LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

static  int8_t  _udpAlive_sendMsg(void);
static  uint8_t _udpAlive_addAddr(char * pc_buf, const uip_ipaddr_t* rps_addr);

/*==============================================================================
                                    LOCAL FUNCTIONS
 =============================================================================*/

/*----------------------------------------------------------------------------*/
/** \brief  This function appends human readable ipv6 address to outcome string.
 *          0x0010  = 0b0000000000010000
 *           0x0100  = 0b0000000100000000
 *           0x1000  = 0b0001000000000000
 *  \param  pc_buf      Pointer to a data buffer where final string should be
 *                      stored
 *  \param  rps_addr    Pointer to a structure where address is stored
 *
 *  \returns c_resLen   Result length of appended string
 */
/*----------------------------------------------------------------------------*/
static uint8_t _udpAlive_addAddr(char * pc_buf, const uip_ipaddr_t* rps_addr)
{
    uint16_t    i_block; /* 2 bytes in each block */
    uint32_t    i;
    int32_t     j;
    uint8_t     k;
    char*       pc_addrStr = pc_buf;
    uint8_t     c_resLen;

    for (i = 0, j = 0; i < sizeof(uip_ipaddr_t); i += 2) {
        i_block = (rps_addr->u8[i] << 8) + rps_addr->u8[i + 1];
        if (i_block == 0 && j >= 0) {
            if (j++ == 0) {
                pc_addrStr += sprintf(pc_addrStr, "::");
                c_resLen+=2;
            }
        } else {
            if (j > 0) {
                j = -1;
            } else if (i > 0) {
                pc_addrStr += sprintf(pc_addrStr, ":");
                c_resLen+=1;
            }
            pc_addrStr += sprintf(pc_addrStr, "%04x", i_block);
            for (k=1;k<5;k++)
                if (i_block < (0x10<<k)) c_resLen+=1;
        }
    }
    return c_resLen;
}/* _udpAlive_addAddr */

/*----------------------------------------------------------------------------*/
/** \brief  This function is sending message with a sequence number.
 *
 *  \param  none
 *  \param    none
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static int8_t _udpAlive_sendMsg(void)
{
    LOG2_INFO( "Enter _udpAlive_sendMsg() function" );

    static uint32_t l_seqId;
    char            ac_buf[MAX_PAYLOAD_LEN];
    uint32_t        i;
    uint8_t         c_err               = 0;
    uint8_t         c_leaveFSM          = 0;
    uint16_t        i_destPort          = _PORT;
    uip_ds6_addr_t* ps_srcAddr          = uip_ds6_get_global(ADDR_PREFERRED);
    rpl_dag_t*      ps_dagDesc          = rpl_get_any_dag();
    e_udpAlive_t    e_state             = E_UDPALIVE_GETDESTADDR;

    while (!c_leaveFSM) {
        switch (e_state) {
        case E_UDPALIVE_GETDESTADDR:
            if (ps_srcAddr != NULL)
            {
                if(ps_dagDesc) {
                    /* Insert global prefix */
                    s_destAddr = ps_srcAddr->ipaddr;
                    memcpy(&s_destAddr.u8[8], &ps_dagDesc->dag_id.u8[8], 8);
                    e_state = E_UDPALIVE_CONNECT;
                }
                else {
                    LOG_ERR("Get destination address FAILED");
                    c_err = -1;
                    e_state = E_UDPALIVE_UNDEF;
                }
            } else {
                e_state = E_UDPALIVE_DONE;
            }
            break;
        case E_UDPALIVE_CONNECT:
            if (pst_udp_socket->udp_conn == NULL)
            {
                udp_socket_register(pst_udp_socket, NULL, NULL);
                udp_socket_connect(pst_udp_socket, &s_destAddr, i_destPort);
            } else {
                if (memcmp(&pst_udp_socket->udp_conn->ripaddr, &s_destAddr, IP6ADDRSIZE))
                {
                    udp_socket_close(pst_udp_socket);
                    udp_socket_register(pst_udp_socket, NULL, NULL);
                    udp_socket_connect(pst_udp_socket, &s_destAddr, i_destPort);
                    LOG_WARN("Reconnection to server");
                }
            }

            if (pst_udp_socket->udp_conn != NULL) {
                LOG1_RAW("Using destination addr: ");
                LOG1_IP6ADDR(pst_udp_socket->udp_conn->ripaddr.u8);
                LOG1_RAW("\n\r local/remote port %u/%u\r\n",
                       UIP_HTONS(pst_udp_socket->udp_conn->lport),
                       UIP_HTONS(pst_udp_socket->udp_conn->rport));
                e_state = E_UDPALIVE_PREPMSG;
            }else {
                e_state = E_UDPALIVE_UNDEF;
                c_err = -2;
                LOG_ERR("Get destination address FAILED\n");
            }

            break;
        case E_UDPALIVE_PREPMSG:
                i = sprintf(ac_buf, "%lu | ", ++l_seqId);
                if(ps_dagDesc && ps_dagDesc->instance->def_route) {
                    _udpAlive_addAddr(ac_buf + i,
                                      &ps_dagDesc->instance->def_route->ipaddr);
                } else {
                    sprintf(ac_buf + i, "(null)");
                }
                LOG1_INFO("Payload: %s", ac_buf);
                e_state = E_UDPALIVE_SENDMSG;
            break;
        case E_UDPALIVE_SENDMSG:
            udp_socket_send(pst_udp_socket, ac_buf, strlen(ac_buf));
            e_state = E_UDPALIVE_DONE;
            break;
        case E_UDPALIVE_DONE:
        case E_UDPALIVE_UNDEF:
        default:
            c_leaveFSM = 1;
            break;
        }
    }

    LOG2_INFO( "Leave _udpAlive_sendMsg() function" );

    return (c_err);

} /* _udpAlive_sendMsg */

/*----------------------------------------------------------------------------*/
/** \brief  This function (UDP Alive) is called whenever the timer expired.
 *
 *  \param  event     Event type
 *  \param    data    Pointer to data
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static    void _udpAlive_callback(c_event_t c_event, p_data_t p_data) {
    if (etimer_expired(&e_udpAliveTmr))
    {
        _udpAlive_sendMsg();
        etimer_restart(&e_udpAliveTmr);
    }
} /* _udpAlive_callback */

/*=============================================================================
                                         API FUNCTIONS
 ============================================================================*/

/*---------------------------------------------------------------------------*/
/*  demo_udpAliveInit()                                                      */
/*---------------------------------------------------------------------------*/
int8_t demo_udpAliveConf(s_ns_t* p_netstk)
{
  int8_t ret = -1;

  /*
   * By default stack
   */
  if (p_netstk != NULL) {
    if (!p_netstk->c_configured) {
      p_netstk->hc = &hc_driver_sicslowpan;
      p_netstk->frame = &framer_802154;
      p_netstk->dllsec = &dllsec_driver_null;
      p_netstk->c_configured = 1;
      ret = 0;
    } else {
      if ((p_netstk->hc == &hc_driver_sicslowpan) &&
          (p_netstk->frame == &framer_802154) &&
          (p_netstk->dllsec == &dllsec_driver_null)) {
        ret = 0;
      } else {
        p_netstk = NULL;
        ret = -1;
      }
    }
  }

  return ret;
}/* demo_udpAliveConf */

/*---------------------------------------------------------------------------*/
/*    demo_udpAliveInit()                                                    */
/*---------------------------------------------------------------------------*/
int8_t demo_udpAliveInit(void)
{
    LOG2_INFO( "Enter demo_udpAliveInit() function" );
    memset( &st_udp_socket, 0, sizeof(st_udp_socket) );

    /* set the pointer to the udp-socket */
    pst_udp_socket = &st_udp_socket;

    /* set periodic timer */
    etimer_set( &e_udpAliveTmr,SEND_INTERVAL * bsp_getTRes(),
                _udpAlive_callback);

    LOG2_INFO( "Leave demo_udpAliveInit() function" );
    return 0;
}/* demo_udpAliveInit()  */
/** @} */
/** @} */
/** @} */
