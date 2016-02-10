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
 *      \addtogroup demo_mdns
 *      @{
 *      \addtogroup demo_mdns_server
 *      @{
*/
/*! \file   demo_mdns_srv.c

 \author Fesseha Tsegaye

 \brief  mDNS example server application

 \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/

#include "emb6.h"
#include "tcpip.h"
#include "udp-socket.h"
#include "uip.h"
#include "resolv.h"
#include "demo_mdns_srv.h"

/*==============================================================================
                                     MACROS
 =============================================================================*/

#define     LOGGER_ENABLE        LOGGER_DEMO_MDNS
#include    "logger.h"

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
#define MAX_PAYLOAD_LEN 120

static struct udp_socket   st_server_udp_socket;
struct udp_socket*  pst_server_udp_socket;
static int seq_id;

static void mdns_callback(struct udp_socket *c,
		void *ptr,
		const uip_ipaddr_t *source_addr,
		uint16_t source_port,
		const uip_ipaddr_t *dest_addr,
		uint16_t dest_port,
		const uint8_t *data,
		uint16_t datalen)
{
	char buf[MAX_PAYLOAD_LEN];
	memset(buf, 0, MAX_PAYLOAD_LEN);

	if(data) {
		((char *)data)[datalen] = 0;
		LOG_INFO("Server received: '%s' from ", (char *)data);
		LOG_IP6ADDR(source_addr->u8);

		uip_ipaddr_copy(&pst_server_udp_socket->udp_conn->ripaddr, source_addr);
		LOG_INFO("Responding with message: ");
		sprintf(buf, "Hello from the server! (%d)", ++seq_id);
		LOG_INFO("%s\n", buf);

		udp_socket_send(pst_server_udp_socket, buf, strlen(buf));
		/* Restore server connection to allow data from any node */
		memset(&pst_server_udp_socket->udp_conn->ripaddr, 0, sizeof(uip_ipaddr_t));
	}
}
/*==============================================================================
                                         API FUNCTIONS
 =============================================================================*/
/*------------------------------------------------------------------------------
demo_mdnsInit()
------------------------------------------------------------------------------*/

int8_t demo_mdnsInit(void)
{
	init();
#if RESOLV_CONF_SUPPORTS_MDNS
	resolv_set_hostname("demo-gateway");
#endif

	pst_server_udp_socket = &st_server_udp_socket;

	udp_socket_register(pst_server_udp_socket, NULL, mdns_callback);
	pst_server_udp_socket->udp_conn->rport = UIP_HTONS(3001);
	udp_socket_bind(pst_server_udp_socket, 3000);

	return 1;
}

/*------------------------------------------------------------------------------
demo_mdnsConf()
------------------------------------------------------------------------------*/

uint8_t demo_mdnsConf(s_ns_t* pst_netStack)
{
	uint8_t c_ret = 1;

	/*
	 * By default stack
	 */
    if (pst_netStack != NULL) {
        if (!pst_netStack->c_configured) {
            pst_netStack->hc    = &sicslowpan_driver;
            pst_netStack->frame = &framer_802154;
            pst_netStack->dllsec = &nullsec_driver;
            c_ret = 1;
        } else {
            if ((pst_netStack->hc    == &sicslowpan_driver) &&
                (pst_netStack->frame == &framer_802154)     &&
                (pst_netStack->dllsec == &nullsec_driver)) {
                c_ret = 1;
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
