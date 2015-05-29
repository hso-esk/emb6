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
 * 	 \addtogroup embetter6
 * 	 @{
 * 	 \addtogroup demo
 * 	 @{
 * 	 \addtogroup demo_udp
 * 	 @{
 * 	 \addtogroup demo_udp_server
 * 	 @{
*/
/*! \file   demo_udp_srv.c

 \author Artem Yushev, artem.yushev@hs-offenburg.de

 \brief  Demonstration of the UDP protocol functionalities .

 \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/

#include "emb6.h"
#include "bsp.h"
#include "demo_udp.h"
#include "etimer.h"
#include "evproc.h"
#include "tcpip.h"
#include "uip.h"
#include "uiplib.h"
#include "uip-debug.h"
#include "uip-udp-packet.h"

/*==============================================================================
 	 	 	 	 	 	 	 	 MACROS
 =============================================================================*/
#define 	LOGGER_ENABLE		LOGGER_DEMO_EXUDP
#if			LOGGER_ENABLE 	== 	TRUE
#define		LOGGER_SUBSYSTEM	"dUDP"
#endif
#include	"logger.h"


#define 	SEND_INTERVAL			5 * bsp_get(E_BSP_GET_TRES)
#define 	MAX_S_PAYLOAD_LEN 		256
#define 	MAX_C_PAYLOAD_LEN		40

/** Communication port for client (not for CoAP) */
#define 	__CLIENT_PORT 						4124
/** Communication port for server (not for CoAP) */
#define 	__SERVER_PORT 						4123

///** Define a network prefix for all addresses */
//#define NETWORK_PREFIX						0x2001, 0xbbbb, 0xdddd, 0x0000
///** Mode 3 - derived from server link-local (MAC) address */
//#define SERVER_IP_ADDR_8_0					0x02ff, 0xffff,	0xffff, 0xffff
///** Server IP address consist of network prefix and 64 bit of mac address */
//#define SERVER_IP_ADDR						NETWORK_PREFIX,SERVER_IP_ADDR_8_0

/* External global buffer for data for packet storing (why not use packetbuf.c?)*/
#define 	UIP_IP_BUF   			((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define 	_QUOTEME(x) 			#x
#define 	QUOTEME(x) 				_QUOTEME(x)
/*==============================================================================
 	 	 	 	 	 	 	 	 ENUMS
 =============================================================================*/

/*==============================================================================
 	 	 	 	 	 STRUCTURES AND OTHER TYPEDEFS
 =============================================================================*/

static struct 	uip_udp_conn 	*pst_conn;
/*==============================================================================
 	 	 	 	 	 LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
/*==============================================================================
 	 	 	 	 	 	 	 LOCAL CONSTANTS
 =============================================================================*/

/*==============================================================================
 	 	 	 	 	 	 LOCAL FUNCTION PROTOTYPES
 =============================================================================*/
static		void	 	_demo_udp_callback(c_event_t c_event, void * p_data);
static		uint64_t 	_demo_extractSeq(char * str);
/*==============================================================================
 	 	 	 	 	 	 	 LOCAL FUNCTIONS
 =============================================================================*/

/*----------------------------------------------------------------------------*/
/** \brief  This function is sending message with a sequence number.
 *
 *  \param  none
 *  \param	none
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static void _demo_udp_sendMsg(uint64_t seqID)
{
	char 		pc_buf[MAX_S_PAYLOAD_LEN];
	sprintf(pc_buf, "%d | OK", seqID);
	LOG_INFO("Respond with message: %s\n\r",pc_buf);
	uip_ipaddr_copy(&pst_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
	uip_udp_packet_send(pst_conn, pc_buf, strlen(pc_buf));
	uip_create_unspecified(&pst_conn->ripaddr);
} /* _demo_udp_sendMsg */

uint64_t _demo_extractSeq(char * str)
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
}

/*----------------------------------------------------------------------------*/
/** \brief  This function is called whenever a tcpip event occurs or
 * 			one of a timers are expired.
 *
 *  \param  event 	Event type
 *  \param	data	Pointer to data
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static	void _demo_udp_callback(c_event_t c_event, p_data_t p_data) {
	char *pc_str;
	if(c_event == EVENT_TYPE_TCPIP) {
		if(uip_newdata()) {
			pc_str = uip_appdata;
			pc_str[uip_datalen()] = '\0';
			LOG_INFO("Packet from a client: '%s'\n\r", pc_str);
			_demo_udp_sendMsg(_demo_extractSeq(pc_str) + 1);
		}
	}
}

/*==============================================================================
 	 	 	 	 	 	 	 	 API FUNCTIONS
 =============================================================================*/
/*----------------------------------------------------------------------------*/
/*	demo_udp_init() 														  */
/*----------------------------------------------------------------------------*/
uint8_t demo_udp_socket_init(void) {
	pst_conn = udp_new(NULL, UIP_HTONS(__CLIENT_PORT), NULL);
	udp_bind(pst_conn, UIP_HTONS(__SERVER_PORT));
	LOG_INFO("local/remote port %u/%u\n\r",
			UIP_HTONS(pst_conn->lport),
			UIP_HTONS(pst_conn->rport));
	LOG_INFO("%s\n\r", "UDP server started, waiting for connection...");
	evproc_regCallback(EVENT_TYPE_TCPIP,_demo_udp_callback);
	return 1;
}/* demo_udp_init()  */
/** @} */
/** @} */
/** @} */
/** @} */
