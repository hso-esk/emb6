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

/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "demo_udp_socket_simple.h"
#include "emb6.h"
#include "udp-socket.h"
#include "etimer.h"
#include "rpl.h"


#define     LOGGER_ENABLE       LOGGER_DEMO_UDP_SOCKET_SIMPLE
#if         LOGGER_ENABLE   ==  TRUE
#define     LOGGER_SUBSYSTEM    "UDP Socket Simple"
#endif
#include    "logger.h"


/*
********************************************************************************
*                                 LOCAL MACROS
********************************************************************************
*/
/** Port to use for the demo */
#define DEMO_UDP_SIMPLE_PORT                        45287

/** interval to send data in ms */
#define DEMO_UDP_SOCKET_SIMPLE_SEND_INTERVAL        1000u

/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/

/** The UDP socket to send and receive data from */
static struct udp_socket s_soc;

#ifdef  DEMO_UDP_SOCKET_SIMPLE_ROLE_CLIENT
static uint32_t udp_socket_currSeqTx;
static uint32_t udp_socket_lastSeqRx;
static uint32_t udp_socket_lostPktQty;
static struct etimer udp_socket_etimer;
#endif /* DEMO_UDP_SOCKET_SIMPLE_ROLE_CLIENT */

/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
/* Data input callback. For further information see the function definition. */
static void _rx_cb(struct udp_socket *c, void *ptr,
    const uip_ipaddr_t *source_addr, uint16_t source_port,
    const uip_ipaddr_t *dest_addr, uint16_t dest_port,
    const uint8_t *data, uint16_t datalen);

#ifdef  DEMO_UDP_SOCKET_SIMPLE_ROLE_CLIENT
/* Event handler that is called for every TX timeout.
   For further details see the function definition */
static void _tx_eventHandler(c_event_t c_event, p_data_t p_data);
#endif /* #ifdef  DEMO_UDP_SOCKET_SIMPLE_ROLE_CLIENT */

/*
********************************************************************************
*                          LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/

/**
 * \brief   Data Input callback.
 *
 *          This function is called whenever new data was received.
 */
static void _rx_cb(struct udp_socket *c, void *ptr,
    const uip_ipaddr_t *source_addr, uint16_t source_port,
    const uip_ipaddr_t *dest_addr, uint16_t dest_port,
    const uint8_t *data, uint16_t datalen)
{
  uint8_t seqNr = 0;

  if( datalen > 0 ) {

    /* get the sequence number from the packet */
    seqNr = data[0];
    LOG1_INFO("UDP Rx Seq.-Nr.: %d", seqNr);

#ifdef  DEMO_UDP_SOCKET_SIMPLE_ROLE_SERVER
    /* reply with the same packet to the seource address and port */
    udp_socket_sendto(&s_soc, &seqNr, 1, source_addr ,source_port);
#endif /* #ifdef  DEMO_UDP_SOCKET_SIMPLE_ROLE_SERVER */
  }
}

#ifdef  DEMO_UDP_SOCKET_SIMPLE_ROLE_CLIENT
/**
 * \brief   TX timeout event handler.
 *
 *          This function is called whenever new data shall be transmitted
 *          triggered by a timeout.
 */
static void _tx_eventHandler(c_event_t c_event, p_data_t p_data)
{
  int err = 0;
  rpl_dag_t *ps_dag_desc;
  uint8_t data = (uint8_t)udp_socket_currSeqTx;

  err = etimer_expired(&udp_socket_etimer);
  if (err != 0) {
    /* restart event timer */
    etimer_restart(&udp_socket_etimer);

    /* try to get DAG */
    ps_dag_desc = rpl_get_any_dag();
    if (ps_dag_desc) {

      LOG1_INFO("UDP Tx Seq.-Nr.: %d", udp_socket_currSeqTx);
      /*Issue transmission request */
      udp_socket_sendto(&s_soc, &data, 1, &ps_dag_desc->dag_id ,DEMO_UDP_SIMPLE_PORT);
    }

    /* increase sequence number of UDP message */
    if (++udp_socket_currSeqTx >= 255) {
      udp_socket_currSeqTx = 0;
    }
  }
}
#endif /* #ifdef  DEMO_UDP_SOCKET_SIMPLE_ROLE_CLIENT */

/*
********************************************************************************
*                           API FUNCTION DEFINITIONS
********************************************************************************
*/

/**
 * \brief   Initialize UDP Socket demo application
 */
int8_t demo_udpSocketSimpleInit(void)
{
  /* register a new port */
  udp_socket_register( &s_soc, NULL, _rx_cb );
  /* bind the socket */
  udp_socket_bind( &s_soc, DEMO_UDP_SIMPLE_PORT );
  /* connect the socket */
  udp_socket_connect( &s_soc, NULL, DEMO_UDP_SIMPLE_PORT );

#ifdef DEMO_UDP_SOCKET_SIMPLE_ROLE_CLIENT
  clock_time_t interval = 0;

  /* set UDP event timer interval */
  interval  = DEMO_UDP_SOCKET_SIMPLE_SEND_INTERVAL;
  interval *= bsp_getTRes()/1000;

  /* Set event timer for periodic data process */
  etimer_set(&udp_socket_etimer, interval, &_tx_eventHandler);

  /* initialize statistic */
  udp_socket_currSeqTx = 0u;
  udp_socket_lastSeqRx = 0u;
  udp_socket_lostPktQty = 0u;
#endif /* #ifdef DEMO_UDP_SOCKET_SIMPLE_ROLE_CLIENT */

  /* Always success */
  return 1;
}


/**
 * \brief   Configure UDP socket demo application
 *
 * \param   p_netstk    Pointer to net stack structure
 */
int8_t demo_udpSocketSimpleCfg(s_ns_t *p_netstk)
{
  int8_t i_ret = -1;

  if (p_netstk != NULL) {
    if (p_netstk->c_configured == FALSE) {
      p_netstk->hc = &hc_driver_sicslowpan;
      p_netstk->frame = &framer_802154;
      p_netstk->dllsec = &dllsec_driver_null;
      i_ret = 1;
    } else {
      if ((p_netstk->hc == &hc_driver_sicslowpan) &&
          (p_netstk->frame == &framer_802154) &&
          (p_netstk->dllsec == &dllsec_driver_null)) {
        i_ret = 1;
      } else {
        p_netstk = NULL;
        i_ret = 0;
      }
    }
  }
  return i_ret;
}
