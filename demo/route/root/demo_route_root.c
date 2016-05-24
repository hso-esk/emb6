/*
 * demo_mcast_root.c
 *
 *  Created on: 4 Apr 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 */

#include "emb6.h"
#include "bsp.h"
#include "etimer.h"
#include "evproc.h"
#include "tcpip.h"
#include "rpl.h"
#include "udp-socket.h"
#include "uip-udp-packet.h"

#include "demo_route_root.h"

#include "thrd-adv.h"
#include "thrd-route.h"
#include "thrd-leader-db.h"

#include "thrd-partition.h"

#include "thrd-send-adv.h"

#include "tlv.h"

#define		DEBUG		DEBUG_PRINT

#include "uip-debug.h"	// For debugging terminal output.

/*==============================================================================
                                         MACROS
 =============================================================================*/
#define     LOGGER_ENABLE        LOGGER_DEMO_MCAST
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

// LZ.

#define 	MCAST_SINK_UDP_PORT 		3001 /* Host byte order */
#define 	ITERATIONS 					100 /* messages */

#if !NETSTACK_CONF_WITH_IPV6 || !UIP_CONF_ROUTER || !UIP_CONF_IPV6_MULTICAST || !UIP_CONF_IPV6_RPL
#error "This example can not work with the current contiki configuration"
#error "Check the values of: NETSTACK_CONF_WITH_IPV6, UIP_CONF_ROUTER, UIP_CONF_IPV6_RPL"
#endif


/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
static  struct  udp_socket          st_udp_socket;
static  struct  udp_socket          *pst_udp_socket;

// static  uip_ip6addr_t               s_destAddr;

struct  etimer                      e_mcastTmr;

// LZ.
static struct uip_udp_conn * mcast_conn;
static char buf[MAX_PAYLOAD_LEN];
static uint32_t seq_id;


/*==============================================================================
                               LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

static int8_t _mcast_sendMsg(void);
static void _mcast_prepare(void);
static void _mcast_set_own_addresses(void);


/*==============================================================================
                                    LOCAL FUNCTIONS
 =============================================================================*/

/*----------------------------------------------------------------------------*/
/** \brief  This function (MCAST) is called whenever the timer expired.
 *
 *  \param  event     Event type
 *  \param    data    Pointer to data
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static void _mcast_callback(c_event_t c_event, p_data_t p_data) {
	if (etimer_expired(&e_mcastTmr))
	{
		_mcast_sendMsg();
		etimer_restart(&e_mcastTmr);
	}
} /* _mcast_callback */


/*----------------------------------------------------------------------------*/
/** \brief  This function is sending message with a sequence number.
 *
 *  \param  none
 *  \param    none
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/

uint8_t cnt = 0;

uint8_t l_m = 5;

// TLV value buffers.
static uint8_t source_tlv[4];
static uint8_t route64_tlv[MAX_ROUTE64_TLV_DATA_SIZE];
static uint8_t leader_tlv[10];

static int8_t _mcast_sendMsg(void)
{

	uint32_t id;

	id = uip_htonl(seq_id);
	memset(buf, 0, MAX_PAYLOAD_LEN);
	memcpy(buf, &id, sizeof(seq_id));

	/*
	PRINTF("Send to: ");
	PRINT6ADDR(&mcast_conn->ripaddr);
	PRINTF(" Remote Port %u,", uip_ntohs(mcast_conn->rport));
	PRINTF(" (msg=0x%08lx)", (unsigned long)uip_ntohl(*((uint32_t *)buf)));
	PRINTF(" %lu bytes\n", (unsigned long)sizeof(id));
	 */

	uip_udp_packet_send(mcast_conn, buf, sizeof(id));

	// --- NEW

	tlv_t *tlv_source;
	tlv_t *tlv_route64;
	tlv_t *tlv_leader;

	// Routing Database.
	switch (cnt) {
	case 0:

		// thrd_trickle_init();

		thrd_partition_start();

		// thrd_rdb_rid_empty();



		// Build Source TLV.
		source_tlv[0] = TLV_SOURCE_ADDRESS;
		source_tlv[1] = 2;
		source_tlv[2] = 0x04;
		source_tlv[3] = 0x00;
		tlv_init(&tlv_source, source_tlv);
		tlv_print(tlv_source);

		// Build Route64 TLV.
		route64_tlv[0] = TLV_ROUTE64;
		route64_tlv[2] = 1;
		uint32_t router_id_mask = 0xD5000000;
		route64_tlv[7] = 0x64;	// RID = 0.
		route64_tlv[8] = 0xB7;	// RID = 1.
		route64_tlv[9] = 0xC5;	// RID = 3.
		route64_tlv[10] = 0x9A;	// RID = 5.
		route64_tlv[11] = 0x71;	// RID = 7.
		route64_tlv[6] = (uint8_t) router_id_mask;
		for ( uint8_t i = 5; i > 2; i-- ) {
			router_id_mask >>= 8;
			route64_tlv[i] = (uint8_t) router_id_mask;
		}
		route64_tlv[1] = 10;
		tlv_init(&tlv_route64, route64_tlv);
		tlv_print(tlv_route64);

		// Build Leader TLV.
		leader_tlv[0] = TLV_LEADER_DATA;
		leader_tlv[1] = 8;
		leader_tlv[2] = 0x00;
		leader_tlv[3] = 0x00;
		leader_tlv[4] = 0x00;
		leader_tlv[5] = 0x01;
		leader_tlv[6] = 64;
		leader_tlv[7] = 7;
		leader_tlv[8] = 5;
		leader_tlv[9] = 3;
		tlv_init(&tlv_leader, leader_tlv);
		tlv_print(tlv_leader);

		thrd_process_adv(tlv_source, tlv_route64, tlv_leader);



		/*

		thrd_rdb_rid_add(2);
		thrd_rdb_link_update(2, 15, 2, 250);

		thrd_generate_route64();

		*/

		// printf("[%d] | link hysteresis: %d\n", cnt, thrd_rdb_link_hysteresis(5, 10));
		printf("------------------------------------------\n\n");
		break;
	case 1:

		// thrd_trickle_reset();

		/*
		thrd_rdb_rid_add(3);
		thrd_rdb_route_update(2, 3, 6);

		thrd_generate_route64();
		*/
		// printf("[%d] | link hysteresis: %d\n", cnt, thrd_rdb_link_hysteresis(5, 12));
		printf("------------------------------------------\n\n");
		break;
	case 2:

		// tlv = thrd_generate_route64();

		// thrd_rdb_rid_empty();

		// thrd_process_route64(2, tlv);

		/*
		thrd_rdb_rid_add(4);
		thrd_rdb_route_update(2, 4, 7);

		thrd_generate_route64();
		*/

		// printf("[%d] | link hysteresis: %d\n", cnt, thrd_rdb_link_hysteresis(17, 25));
		printf("------------------------------------------\n\n");
		break;
	case 3:
		thrd_rdb_route_update(2, 4, 2);

		// printf("[%d] | link hysteresis: %d\n", cnt, thrd_rdb_link_hysteresis(0, 5));
		printf("------------------------------------------\n\n");
		break;
	case 4:
		thrd_rdb_rid_add(5);
		thrd_rdb_link_update(5, 25, 3, 250);

		thrd_generate_route64();

		// printf("[%d] | link hysteresis: %d\n", cnt, thrd_rdb_link_hysteresis(2, 25));
		printf("------------------------------------------\n\n");
		break;
	case 5:
		thrd_rdb_route_update(5, 4, 6);

		thrd_generate_route64();

		// printf("[%d] | link hysteresis: %d\n", cnt, thrd_rdb_link_hysteresis(0, 25));
		printf("------------------------------------------\n\n");
		break;
	case 6:
		thrd_rdb_route_update(5, 3, 7);

		thrd_generate_route64();

		// printf("[%d] | link hysteresis: %d\n", cnt, thrd_rdb_link_hysteresis(25, 15));
		printf("------------------------------------------\n\n");
		break;
	case 7:
		thrd_rdb_route_update(5, 2, 1);

		thrd_generate_route64();

		// printf("[%d] | link hysteresis: %d\n", cnt, thrd_rdb_link_hysteresis(5, 0));
		printf("------------------------------------------\n\n");
		break;
	case 8:
		thrd_rdb_route_update(5, 3, 4);

		thrd_generate_route64();

		// printf("[%d] | link hysteresis: %d\n", cnt, thrd_rdb_link_hysteresis(15, 0));
		printf("------------------------------------------\n\n");
		break;
	default:
		exit(0);
		break;
	}

	if ( cnt <= 8 ) {
		// Print all data of the routing database.
		// thrd_rdb_print_routing_database();
	}


	/*
	// Leader Database.
	switch (cnt) {
	case 0:
		thrd_ldb_ida_add(1, 2, 3);
		break;
	case 1:
		thrd_ldb_ida_add(2, 2, 3);
		break;
	case 2:
		thrd_ldb_ida_add(3, 2, 3);
		break;
	case 3:
		thrd_ldb_ida_rm(thrd_ldb_ida_lookup(1));
		break;
	case 4:
		thrd_ldb_ida_rm(thrd_ldb_ida_lookup(2));
		break;
	case 5:
		thrd_ldb_ida_rm(thrd_ldb_ida_lookup(3));
		break;
	case 6:
		thrd_ldb_ida_rm(thrd_ldb_ida_lookup(4));
		break;
	case 7:
		thrd_ldb_ida_add(1, 2, 3);
		break;
	case 8:
		break;
	default:
		exit(0);
		break;
	}

	if ( cnt <= 8 ) {
		// Print all data of the routing database.
		thrd_ldb_print_leader_database();
	}
	*/

	cnt++;

	return 1;

} /* _mcast_prepare */

/*----------------------------------------------------------------------------*/
/** \brief  This function is preparing message with a sequence number.
 *
 *  \param  none
 *  \param    none
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static void _mcast_prepare(void)
{
	uip_ipaddr_t ipaddr;

	/*
	 * IPHC will use stateless multicast compression for this destination
	 * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD)
	 */
	uip_ip6addr(&ipaddr, 0xFF1E,0,0,0,0,0,0x89,0xABCD);		// Global-Scope Multicast Address.
	mcast_conn = udp_new(&ipaddr, UIP_HTONS(MCAST_SINK_UDP_PORT), NULL);
}

/*----------------------------------------------------------------------------*/
/** \brief  This function is setting own addresses.
 *
 *  \param  none
 *  \param    none
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static void _mcast_set_own_addresses(void)
{
	int i;
	uint8_t state;
	rpl_dag_t *dag;
	uip_ipaddr_t ipaddr;

	uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
	uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
	uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

	PRINTF("Our IPv6 addresses:\n");
	for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if(uip_ds6_if.addr_list[i].isused && (state == ADDR_TENTATIVE || state
				== ADDR_PREFERRED)) {
			PRINTF("  ");
			PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
			PRINTF("\n");
			if(state == ADDR_TENTATIVE) {
				uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
			}
		}
	}

	/* Become root of a new DODAG with ID our global v6 address */
	dag = rpl_set_root(RPL_DEFAULT_INSTANCE, &ipaddr);
	if(dag != NULL) {
		rpl_set_prefix(dag, &ipaddr, 64);
		PRINTF("Created a new RPL dag with ID: ");
		PRINT6ADDR(&dag->dag_id);
		PRINTF("\n");
	}
}


/*=============================================================================
                                         API FUNCTIONS
 ============================================================================*/

/*---------------------------------------------------------------------------*/
/*  demo_routeRootInit()                                                      */
/*---------------------------------------------------------------------------*/
uint8_t demo_routeRootConf(s_ns_t* pst_netStack)
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
}/* demo_mcastRootConf */

/*---------------------------------------------------------------------------*/
/*    demo_routeRootInit()                                                    */
/*---------------------------------------------------------------------------*/
int8_t demo_routeRootInit(void)
{

	// rt_db_init();

	// Initialize routing database.
	thrd_rdb_init();

	// Initialize leader database.
	thrd_ldb_init();

	/* set the pointer to the udp-socket */
	pst_udp_socket = &st_udp_socket;

	_mcast_set_own_addresses();

	_mcast_prepare();

	/* set periodic timer */
	etimer_set( &e_mcastTmr, SEND_INTERVAL * bsp_get(E_BSP_GET_TRES),
			_mcast_callback);

	return 1;
}/* demo_mcastRootInit()  */

