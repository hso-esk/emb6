/*
 * --- License --------------------------------------------------------------*
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

/*
 * --- Module Description ---------------------------------------------------*
 */
/**
 *  \file       demo_udp_socket.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Demo to show how to use the UDP socket interface.
 *
 *              This Demo shows how to use the UDP socket interface. Therefore
 *              the demo is divided into a server and a client application. The
 *              client transmits data periodically to the server including a
 *              fixed payload and a sequence counter. The server replies with
 *              a fixed pattern and the same sequence number. The size of the
 *              packet transmitted by the client is increased per message until
 *              a given maximum was reached.
 *              The server is defined as the DAG-Root within the network and
 *              its IP address is retrieved automatically. The Server just
 *              replies to the node it got the packet from.
 *              This demo is mainly used to show how to use the UDP socket
 *              interface and to show basic connectivity.
 */

/*
 * --- Includes -------------------------------------------------------------*
 */
#include "emb6.h"
#include "bsp.h"
#include "demo_udp_socket.h"
#include "etimer.h"
#include "evproc.h"
#include "tcpip.h"
#include "uip-udp-packet.h"
#include "rpl.h"


/*
 *  --- Macros ------------------------------------------------------------- *
 */
#define LOGGER_ENABLE           LOGGER_DEMO_UDP_SOCKET
#if LOGGER_ENABLE == TRUE
#define LOGGER_SUBSYSTEM        "UDP Socket"
#endif /* #if LOGGER_ENABLE == TRUE */
#include    "logger.h"



/** first port used by the socket demo */
#ifndef DEMO_UDP_SOCKET_PORTA
#define DEMO_UDP_SOCKET_PORTA           4211UL
#endif /* #ifndef DEMO_UDP_SOCKET_PORTA */

#ifndef DEMO_UDP_SOCKET_PORTB
/** second port used by the socket demo */
#define DEMO_UDP_SOCKET_PORTB           4233UL
#endif /* #ifndef DEMO_UDP_SOCKET_PORTB */

#if (DEMO_UDP_SOCKET_ROLE_SERVER == TRUE)
/** assign device port */
#define DEMO_UDP_DEVPORT                DEMO_UDP_SOCKET_PORTA
/** assign remote port */
#define DEMO_UDP_REMPORT                DEMO_UDP_SOCKET_PORTB
#else
/** assign device port */
#define DEMO_UDP_DEVPORT                DEMO_UDP_SOCKET_PORTB
/** assign remote port */
#define DEMO_UDP_REMPORT                DEMO_UDP_SOCKET_PORTA

#ifndef DEMO_UDP_SEND_INTERVAL
/** specifies the interval to send periodic data */
#define DEMO_UDP_SEND_INTERVAL          (clock_time_t)( 200u )
#endif /* #ifndef DEMO_UDP_SEND_INTERVAL */
#endif /* #if (DEMO_UDP_SOCKET_ROLE_SERVER == TRUE) */



/** minimum packet length is equal to size of sequence number (4 bytes) */
#define DEMO_UDP_PKT_LEN_MIN            (  4u ) // 80u to test fragmentation (threshold ~ 84 bytes )

#ifndef DEMO_UDP_PKT_LEN_MAX
/** maximum packet length */
#define DEMO_UDP_PKT_LEN_MAX            ( 40u ) // 90u to test fragmentation (threshold ~ 84 bytes )
#endif /* #ifndef DEMO_UDP_PKT_LEN_MAX */


/** pointer to the UIP buffer structure */
#define UIP_IP_BUF                      ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

/*
 *  --- Local Variables ---------------------------------------------------- *
 */

/** UDP connection */
static struct uip_udp_conn *pudp_socket_conn = (struct uip_udp_conn *) 0;

/** current sequence number counter */
static uint32_t udp_socket_currSeqTx;

#if (DEMO_UDP_SOCKET_ROLE_CLIENT == TRUE)

/* packet length */
static uint8_t packetLength;

/** Sequence number of the last received packet */
static uint32_t udp_socket_lastSeqRx;

/** Number of lost packets. */
static uint32_t udp_socket_lostPktQty;

/** Timer used for the next tx packet */
static struct etimer udp_socket_etimer;
#endif /* #if (DEMO_UDP_SOCKET_ROLE_SERVER != TRUE) */


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */

/* Get the demo sequence number from a packet.
   For further details see the function definition */
static uint32_t udp_socket_getSeq(uint8_t *p_data, uint16_t len);

/* Event handler that is called for every IP packet received.
   For further details see the function definition */
static void udp_socket_eventHandler(c_event_t c_event, p_data_t p_data);

/* transmit a sample packet. For further details see the function definition */
static void udp_socket_tx(uint32_t seq);


/*
 *  --- Local Functions ---------------------------------------------------- *
 */

/**
 * \brief   Get sequence number from an an UDP message.
 *
 *          This functions retrieves the sequence number from a
 *          received UDP message.
 *
 * \param   p_data    The UDP message to get the sequence number from.
 * \param   len       Length of the message.
 *
 * \return  The sequence number within the UDP packet.
 */
static uint32_t udp_socket_getSeq(uint8_t *p_data, uint16_t len)
{
  uint32_t ret = 0u;
  uint8_t is_valid = FALSE;

  is_valid = (p_data != NULL) && (len > 0);
  if (is_valid == TRUE)
  {
    memcpy(&ret, p_data, sizeof(ret));
  }
  return ret;
}

/**
 * \brief   UDP event handler function
 *
 * \param   c_event     Event to be processed
 * \param   p_data      Callback argument that was registered as the event was
 *                      raised
 */
static void udp_socket_eventHandler(c_event_t c_event, p_data_t p_data)
{
  uint32_t seq;
  uint16_t len;
  uint8_t has_data;
  uint8_t *p_dataptr;

  /*
   * process input TCPIP packet
   */
  if (c_event == EVENT_TYPE_TCPIP)
  {
    has_data = uip_newdata();
    if (has_data != 0u)
    {
      p_dataptr = (uint8_t *) uip_appdata;
      len = (uint16_t) uip_datalen();

      /* Obtain sequence number of the received message */
      seq = udp_socket_getSeq(p_dataptr, len);

#if LOGGER_ENABLE
      /* Logging */
      LOG_RAW("UDP Receiving...  : ");
      while (len--) {
        LOG_RAW("%02x ", *p_dataptr++);
      }
      LOG_RAW("\r\n");
#endif

#if (DEMO_UDP_SOCKET_ROLE_SERVER == TRUE)
      udp_socket_currSeqTx = seq;
      evproc_putEvent(E_EVPROC_HEAD, EVENT_TYPE_APP_TX, NULL);
#else
      udp_socket_lastSeqRx = seq;
      if (udp_socket_currSeqTx != (udp_socket_lastSeqRx + 1)) {
        TRACE_LOG_ERR("<UDP> missed response txSeq=%08x, rxSeq=%08x", udp_socket_currSeqTx, udp_socket_lastSeqRx);
        udp_socket_lostPktQty++;
      }
#endif
    }
  }
  else {
#if (DEMO_UDP_SOCKET_ROLE_SERVER == TRUE)
    if (c_event == EVENT_TYPE_APP_TX) {
      udp_socket_tx(udp_socket_currSeqTx);
    }
#else
    int err = 0;
    err = etimer_expired(&udp_socket_etimer);
    if (err != 0) {
      /* restart event timer */
      etimer_restart(&udp_socket_etimer);

      /* sends a UDP message */
      udp_socket_tx(udp_socket_currSeqTx);

      /* increase sequence number of UDP message */
      if (++udp_socket_currSeqTx >= 1000) {
        udp_socket_currSeqTx = 0;
      }
    }
#endif
  }
}


/**
 * \brief   UDP socket transmission
 *
 * \param   seq     Sequence number of the UDP message to transmit
 */
static void udp_socket_tx(uint32_t seq)
{
#if (DEMO_UDP_SOCKET_ROLE_SERVER == TRUE)
  uint8_t payload[] = { 0, 0, 0, 0, 0x50, 0x51, 0x52, 0x53, 0x54 };
  uint16_t payload_len = sizeof(payload);
  uint8_t seq_size = sizeof(seq);

  /*
   * create packet to send
   */
  memcpy(payload, &seq, seq_size);

  uip_ipaddr_copy(&pudp_socket_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
  uip_udp_packet_send(pudp_socket_conn, payload, payload_len);
  uip_create_unspecified(&pudp_socket_conn->ripaddr);
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
  LOG_RAW("\r\n");
#endif

#else
  uint16_t payload_len;
  uint8_t seq_size = sizeof(seq);
  uint8_t payload[DEMO_UDP_PKT_LEN_MAX];

  /*
   * create packet to send
   */
  memset(payload, 0, sizeof(payload));

  memcpy(payload, &seq, seq_size);
  memset(&payload[seq_size], 0xab, packetLength - seq_size);
  payload_len = packetLength;


  packetLength++;
  if (packetLength > DEMO_UDP_PKT_LEN_MAX) {
    packetLength = DEMO_UDP_PKT_LEN_MIN;
  }

  uip_ds6_addr_t *ps_src_addr;
  rpl_dag_t *ps_dag_desc;

  /*
   * Get Server IP address
   */
  ps_src_addr = uip_ds6_get_global(ADDR_PREFERRED);
  if (ps_src_addr)
  {
    ps_dag_desc = rpl_get_any_dag();
    if (ps_dag_desc)
    {
      /*
       * Issue transmission request
       */
      uip_ipaddr_copy(&pudp_socket_conn->ripaddr, &ps_src_addr->ipaddr);
      memcpy(&pudp_socket_conn->ripaddr.u8[8], &ps_dag_desc->dag_id.u8[8], 8);

      uip_udp_packet_send(pudp_socket_conn, payload, payload_len);
      uip_create_unspecified(&pudp_socket_conn->ripaddr);
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
      LOG_RAW("\r\n");
    #endif

    }
  }
#endif
}

/*
 * --- Global Function Definitions ----------------------------------------- *
 */


/*---------------------------------------------------------------------------*/
/*
* demo_udpSocketInit()
*/
int8_t demo_udpSocketInit(void)
{
  /*Create a new UDP connection */
  pudp_socket_conn = udp_new(NULL, UIP_HTONS(DEMO_UDP_REMPORT), NULL);

  /*Bind the UDP connection to a local port*/
  udp_bind(pudp_socket_conn, UIP_HTONS(DEMO_UDP_DEVPORT));

  /* set callback for event process */
  evproc_regCallback(EVENT_TYPE_TCPIP, udp_socket_eventHandler);
  evproc_regCallback(EVENT_TYPE_APP_TX, udp_socket_eventHandler);

#ifdef DEMO_UDP_SOCKET_ROLE_CLIENT
  clock_time_t interval = 0;
  packetLength = DEMO_UDP_PKT_LEN_MIN;

  /* set UDP event timer interval */
  interval  = DEMO_UDP_SEND_INTERVAL;
  interval *= bsp_getTRes()/1000;

  /* Set event timer for periodic data process */
  etimer_set(&udp_socket_etimer, interval, &udp_socket_eventHandler);

  /* initialize statistic */
  udp_socket_currSeqTx = 0u;
  udp_socket_lastSeqRx = 0u;
  udp_socket_lostPktQty = 0u;
#endif

  /* Always success */
  return 0;
} /* demo_udpSocketInit() */


/*---------------------------------------------------------------------------*/
/*
* demo_udpSocketCfg()
*/
int8_t demo_udpSocketConf(s_ns_t *p_netstk)
{
  int8_t ret = -1;

  if (p_netstk != NULL) {
    if (p_netstk->c_configured == FALSE) {
      p_netstk->hc = &hc_driver_sicslowpan;
      p_netstk->frame = &framer_802154;
#if LLSEC802154_ENABLED
      p_netstk->dllsec = &dllsec_driver_802154;
#else
      p_netstk->dllsec = &dllsec_driver_null;
#endif /* #if LLSEC802154_ENABLED */
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
} /* demo_udpSocketCfg() */
