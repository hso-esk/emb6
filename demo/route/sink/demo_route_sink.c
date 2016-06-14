/*
 * demo_mcast_sink.c
 *
 *  Created on: 4 Apr 2016
 *      Author: osboxes
 */

#include "emb6.h"
#include "bsp.h"
#include "evproc.h"
#include "tcpip.h"
#include "uip.h"
#include "rpl.h"
#include "udp-socket.h"

#include "demo_route_sink.h"

#include "thread_conf.h"

#include "thrd-route.h"
#include "thrd-leader-db.h"
#include "thrd-addr-query.h"

#define		DEBUG		DEBUG_PRINT

#include "uip-debug.h"	// For debugging terminal output.


/*==============================================================================
                                         MACROS
 =============================================================================*/

#define		UIP_CONF_ROUTER			TRUE

#define     LOGGER_ENABLE       	LOGGER_DEMO_MCAST
#include    "logger.h"

#define		MCAST_SINK_UDP_PORT		3001 /* Host byte order */

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

static 	struct 		uip_udp_conn 		*sink_conn;

#if !NETSTACK_CONF_WITH_IPV6 || !UIP_CONF_ROUTER || !UIP_CONF_IPV6_MULTICAST || !UIP_CONF_IPV6_RPL
#error "This example can not work with the current contiki configuration"
#error "Check the values of: NETSTACK_CONF_WITH_IPV6, UIP_CONF_ROUTER, UIP_CONF_IPV6_RPL"
#endif

/*==============================================================================
                                    LOCAL FUNCTIONS
 =============================================================================*/

/*----------------------------------------------------------------------------*/
/** \brief  This function (MCAST) is called whenever an UDP datagram was received.
 *
 *  \param  event     Event type
 *  \param    data    Pointer to data
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static void _printMessage(c_event_t c_event, p_data_t p_data) {

	uint16_t len;
	uint8_t has_data;
	uint8_t *p_dataptr;

	if (c_event == EVENT_TYPE_TCPIP) {
		has_data = uip_newdata();
		if (has_data != 0u) {
			p_dataptr = (uint8_t *) uip_appdata;
			len =(uint16_t) uip_datalen();

			PRINTF("Message received!\n");

			/* Logging */
			LOG_RAW("UDP Receiving...  : ");
			while (len--) {
				LOG_RAW("%02x ", *p_dataptr++);
			}
			LOG_RAW("\r\n");
		} else {
			PRINTF("Error!\n");
		}
	}
}

/*----------------------------------------------------------------------------*/
/** \brief  This function (MCAST) is joining a multicast group.
 *
 *  \param  none
 *
 *  \returns address
 */
/*----------------------------------------------------------------------------*/
static uip_ds6_maddr_t * join_mcast_group(void)
{
	uip_ipaddr_t addr;
	uip_ds6_maddr_t *rv;

	/* First, set our v6 global */
	uip_ip6addr(&addr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
	uip_ds6_set_addr_iid(&addr, &uip_lladdr);
	uip_ds6_addr_add(&addr, 0, ADDR_AUTOCONF);

	/*
	 * IPHC will use stateless multicast compression for this destination
	 * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD)
	 */
	// uip_ip6addr(&addr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);	// Global-Scope Multicast Address.
	REALM_LOCAL_ALL_ROUTERS_ADDR(&addr);

	// uip_ip6addr(&addr, 0xFF12,0,0,0,0,0,0,0xaa);

	rv = uip_ds6_maddr_add(&addr);

	if(rv) {
		PRINTF("Joined multicast group: ");
		PRINT6ADDR(&uip_ds6_maddr_lookup(&addr)->ipaddr);
		PRINTF("\n");
	}
	return rv;
}


/*=============================================================================
                                         API FUNCTIONS
 ============================================================================*/

/*---------------------------------------------------------------------------*/
/*  demo_routeSinkConf()                                                      */
/*---------------------------------------------------------------------------*/
uint8_t demo_routeSinkConf(s_ns_t* pst_netStack)
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
}/* demo_mcastSinkConf */

/*---------------------------------------------------------------------------*/
/*    demo_routeSinkInit()                                                    */
/*---------------------------------------------------------------------------*/
int8_t demo_routeSinkInit(void)
{
	// Initialize routing database.
	thrd_rdb_init();

	// Initialize leader database.
	thrd_ldb_init();

	thrd_dev.Router_ID = 2;

	thrd_eid_rloc_db_init();

	if(join_mcast_group() == NULL) {
		PRINTF("Failed to join multicast group.\n");
	}

	/*
	// Create a UDP 'connection' with IP 0:0:0:0:0:0 and port 0 as remote host.
	// This means the stack will accept UDP datagrams from any node and any source-port.
	sink_conn = udp_new(NULL, UIP_HTONS(0), NULL);

	// Bind the UDP 'connection' to the port 3001. That's the port we're listening on.
	udp_bind(sink_conn, UIP_HTONS(MCAST_SINK_UDP_PORT));

	// tcpip_poll_udp(sink_conn);

	PRINTF("Listening on UDP port %u\n", UIP_HTONS(sink_conn->lport));

	evproc_regCallback(EVENT_TYPE_TCPIP, _printMessage);

	PRINTF("Multicast Sink demo initialized, waiting for connection...");

	*/

	return 1;
}/* demo_mcastInit()  */

