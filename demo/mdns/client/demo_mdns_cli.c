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
 *      \addtogroup demo_mdns
 *      @{
 *      \addtogroup demo_mdns_node
 *      @{
 */
/*! \file   demo_mdns_cli.c

 \author Fesseha Tsegaye

 \brief  mDNS example client application

 \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/

#include "emb6.h"
#include "bsp.h"
#include "tcpip.h"
#include "udp-socket.h"
#include "uip-ds6.h"
#include "uiplib.h"
#include "evproc.h"
#include "resolv.h"
#include "demo_mdns_cli.h"
//#include "uip-debug.h"

/*==============================================================================
                                     MACROS
 =============================================================================*/

#define     LOGGER_ENABLE        LOGGER_DEMO_MDNS
#include    "logger.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]", (lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3], (lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif
/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

static struct etimer et;

static struct udp_socket   st_client_udp_socket;
struct udp_socket*  pst_client_udp_socket;

/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

void demo_mdns_callback (c_event_t c_event, p_data_t p_data);
static resolv_status_t set_connection_address(uip_ipaddr_t *ipaddr);

/*==============================================================================
                                LOCAL CONSTANTS
 =============================================================================*/

#define SEND_INTERVAL		15 * bsp_get(E_BSP_GET_TRES)
#define MAX_PAYLOAD_LEN		40

static char buf[MAX_PAYLOAD_LEN];
static int seq_id;
static resolv_status_t status = RESOLV_STATUS_UNCACHED;

/*==============================================================================
                                LOCAL FUNCTIONS
 =============================================================================*/

void demo_mdns_callback (c_event_t c_event, p_data_t p_data)
{
	switch(status)
	{
//		case RESOLV_STATUS_UNCACHED:
//		case RESOLV_STATUS_RESOLVING:
//			break;
		case RESOLV_STATUS_CACHED:
		{
			if(etimer_expired(&et) && (&et == p_data)) {
				/* */
				printf("Client sending to: ");
				PRINT6ADDR(&pst_client_udp_socket->udp_conn->ripaddr);
				sprintf(buf, "Hello %d from the client", ++seq_id);
				printf(" (msg: %s)\n", buf);
		#if SEND_TOO_LARGE_PACKET_TO_TEST_FRAGMENTATION
				udp_socket_send(pst_client_udp_socket, buf, UIP_APPDATA_SIZE);
		#else /* SEND_TOO_LARGE_PACKET_TO_TEST_FRAGMENTATION */
				udp_socket_send(pst_client_udp_socket, buf, strlen(buf));
		#endif /* SEND_TOO_LARGE_PACKET_TO_TEST_FRAGMENTATION */
				etimer_restart(&et);
			} /* if() */
		} /* RESOLV_STATUS_CACHED */
		break;
		default:
			if (etimer_expired(&et) && (&et == p_data)) {
				status = set_connection_address(&pst_client_udp_socket->udp_conn->ripaddr);
				etimer_restart(&et);
			}
	}
}

static void
print_local_addresses(void)
{
	int i;
	uint8_t state;

	PRINTF("Client IPv6 addresses: ");
	for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if(uip_ds6_if.addr_list[i].isused &&
				(state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
			PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
			PRINTF("\n");
		}
	}
}

resolv_status_t set_connection_address(uip_ipaddr_t *ipaddr)
{
#ifndef UDP_CONNECTION_ADDR
#if RESOLV_CONF_SUPPORTS_MDNS
#define UDP_CONNECTION_ADDR       contiki-udp-server.local
#elif UIP_CONF_ROUTER
#define UDP_CONNECTION_ADDR       aaaa:0:0:0:0212:7404:0004:0404
#else
#define UDP_CONNECTION_ADDR       fe80:0:0:0:6466:6666:6666:6666
#endif
#endif /* !UDP_CONNECTION_ADDR */

#define _QUOTEME(x) #x
#define QUOTEME(x) _QUOTEME(x)

	resolv_status_t status = RESOLV_STATUS_ERROR;

	if(uiplib_ipaddrconv(QUOTEME(UDP_CONNECTION_ADDR), ipaddr) == 0) {
		uip_ipaddr_t *resolved_addr = NULL;
		status = resolv_lookup(QUOTEME(UDP_CONNECTION_ADDR),&resolved_addr);
		if(status == RESOLV_STATUS_UNCACHED || status == RESOLV_STATUS_EXPIRED) {
			PRINTF("Attempting to look up %s\n",QUOTEME(UDP_CONNECTION_ADDR));
			resolv_query(QUOTEME(UDP_CONNECTION_ADDR));
			status = RESOLV_STATUS_RESOLVING;
		} else if(status == RESOLV_STATUS_CACHED && resolved_addr != NULL) {
			PRINTF("Lookup of \"%s\" succeeded!\n",QUOTEME(UDP_CONNECTION_ADDR));
		} else if(status == RESOLV_STATUS_RESOLVING) {
			resolv_query(QUOTEME(UDP_CONNECTION_ADDR));
			PRINTF("Still looking up \"%s\"...\n",QUOTEME(UDP_CONNECTION_ADDR));
		} else {
			PRINTF("Lookup of \"%s\" failed. status = %d\n",QUOTEME(UDP_CONNECTION_ADDR),status);
		}
		if(resolved_addr)
			uip_ipaddr_copy(ipaddr, resolved_addr);
	} else {
		status = RESOLV_STATUS_CACHED;
	}

	return status;
}
/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

/*==============================================================================
 demo_coapInit()
==============================================================================*/

int8_t demo_mdnsInit(void)
{
	LOG_INFO("%s\n\r","Starting MDNS Example Client");
	/**
	 * start probing - init()
	 * on the callback to the probing process:
	 * 	if conflict occurs - probing failed
	 * 	if successful - proceed with announcing
	 *
	 */
	init();

	pst_client_udp_socket = &st_client_udp_socket;

	PRINTF("UDP client process started\n");
	print_local_addresses();

	/* new connection with remote host */
	udp_socket_register(pst_client_udp_socket, NULL, NULL);
	pst_client_udp_socket->udp_conn->rport = UIP_HTONS(3000);
	udp_socket_bind(pst_client_udp_socket, 3001);

//	status = set_connection_address(&pst_client_udp_socket->udp_conn->ripaddr);

	PRINTF("Created a connection with the server ");
	PRINT6ADDR(&pst_client_udp_socket->udp_conn->ripaddr);
	PRINTF(" local/remote port %u/%u\n",
			UIP_HTONS(pst_client_udp_socket->udp_conn->lport), UIP_HTONS(pst_client_udp_socket->udp_conn->rport));

	etimer_set(&et, SEND_INTERVAL, demo_mdns_callback);
	return 1;
}

/*==============================================================================
 demo_coapConf()
==============================================================================*/

uint8_t demo_mdnsConf(s_ns_t* pst_netStack)
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
