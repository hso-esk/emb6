/*
 * demo_mcast_root.c
 *
 *  Created on: 4 Apr 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 */

#include "emb6.h"
#include "stdlib.h"
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

#include "thrd-addr-query.h"

#include "thrd-eid-rloc.h"

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
static uint8_t route64_data[MAX_ROUTE64_TLV_DATA_SIZE];
static uint8_t leader_data[8];

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

	tlv_t *tlv_route64;
	tlv_t *tlv_leader;

	tlv_route64_t *rt64_tlv;
	tlv_leader_t *ld_tlv;

	uip_ipaddr_t test_eid;
	uip_ipaddr_t test_eid2;

	net_tlv_target_eid_t *target_eid_tlv;
	net_tlv_rloc16_t *rloc16_tlv;
	net_tlv_ml_eid_t *ml_eid_tlv;

	uint16_t test_rloc16 = 0x1010;

	uint8_t test_buf[16];

	// Routing Database.
	switch (cnt) {
	case 0:

		/*
		uip_ip6addr(&test_eid, 0xfe80, 0x0000, 0x0000, 0x0000, 0x0250, 0xc2ff, 0xfea8, 0xdddd);

		memcpy(test_buf, &test_eid, 16);

		for( uint8_t i = 0; i < 16; i++ ) {
			printf("test_buf[%d] = %02x\n", i, test_buf[i]);
		}

		uip_ip6addr_u8(&test_eid2, test_buf[0], test_buf[1], test_buf[2], test_buf[3],
				test_buf[4], test_buf[5], test_buf[6], test_buf[7], test_buf[8],
				test_buf[9], test_buf[10], test_buf[11], test_buf[12], test_buf[13],
				test_buf[14], test_buf[15]);

		PRINT6ADDR(&test_eid2);
		 */

		// ---

		/*
		uip_ip6addr(&test_eid, 0xfe80, 0x0000, 0x0000, 0x0000, 0x0250, 0xc2ff, 0xfea8, 0xdddd);
		tlv_target_eid_init(&target_eid_tlv, test_eid.u8);

		uint8_t rloc16[2] = { 0x10, 0x10 };

		tlv_rloc16_init(&rloc16_tlv, rloc16);
		uint8_t ml_eid[8] = { 0xF0, 0x00,
				0x00, 0x00,
				0x00, 0x00,
				0x00, 0x0F };
		tlv_ml_eid_init(&ml_eid_tlv, ml_eid);

		thrd_addr_ntf_response(target_eid_tlv, rloc16_tlv, ml_eid_tlv);
		*/

		uip_ip6addr(&test_eid, 0xfe80, 0x0000, 0x0000, 0x0000, 0x0250, 0xc2ff, 0xfea8, 0xdddd);

		uint16_t rloc16 = 0x1010;
		uint8_t ml_eid[8] = { 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F };
		clock_time_t time = 0x80000008;

		thrd_addr_ntf_response(&test_eid, &rloc16, ml_eid, NULL);

		// ---

		/*
		thrd_eid_rloc_cache_init();

		uip_ipaddr_t eid;
		uip_ipaddr_t rloc;

		uip_ip6addr(&eid, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
		uip_ip6addr(&rloc, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
		thrd_eid_rloc_cache_update(eid, rloc);

		uip_ip6addr(&eid, 0xbbbb, 0, 0, 0, 0, 0, 0, 0);
		uip_ip6addr(&rloc, 0xbbbb, 0, 0, 0, 0, 0, 0, 1);
		thrd_eid_rloc_cache_update(eid, rloc);

		uip_ip6addr(&eid, 0xcccc, 0, 0, 0, 0, 0, 0, 0);
		uip_ip6addr(&rloc, 0xcccc, 0, 0, 0, 0, 0, 0, 1);
		thrd_eid_rloc_cache_update(eid, rloc);

		uip_ip6addr(&eid, 0xdddd, 0, 0, 0, 0, 0, 0, 0);
		uip_ip6addr(&rloc, 0xdddd, 0, 0, 0, 0, 0, 0, 1);
		thrd_eid_rloc_cache_update(eid, rloc);

		thrd_eid_rloc_cache_print();

		uip_ip6addr(&eid, 0xbbbb, 0, 0, 0, 0, 0, 0, 0);
		thrd_eid_rloc_cache_rm(thrd_eid_rloc_cache_lookup(eid));

		thrd_eid_rloc_cache_print();

		thrd_eid_rloc_cache_empty();

		thrd_eid_rloc_cache_print();
		*/

		/*

		// thrd_trickle_init();

		// thrd_partition_start();

		// thrd_rdb_rid_empty();


		// Build Route64 TLV.
		route64_data[0] = TLV_ROUTE64;
		route64_data[2] = 1;
		uint64_t router_id_mask = uip_htonll( 0xD500000000000000 ); //-> 1101 0101 0000 0000 ....
													  //-> OUT IN ROUTE
		route64_data[11] = 0x64;	// RID = 0. 		-> 01  10 0100
		route64_data[12] = 0xB7;	// RID = 1. 		-> 10  11 0111
		route64_data[13] = 0xC5;	// RID = 3. 		-> 11  00 0101
		route64_data[14] = 0x9A;	// RID = 5. 		-> 10  01 1010
		route64_data[15] = 0x71;	// RID = 7. 		-> 01  11 0001
		route64_data[10] = (uint8_t) router_id_mask;
		for ( uint8_t i = 9; i > 2; i-- ) {
			router_id_mask >>= 8;
			route64_data[i] = (uint8_t) router_id_mask;
		}
		route64_data[1] = 14;
		tlv_init(&tlv_route64, route64_data);
		tlv_print(tlv_route64);

		tlv_route64_init(&rt64_tlv, tlv_route64->value);

		// Build Leader TLV.
		leader_data[0] = TLV_LEADER_DATA;
		leader_data[1] = 8;
		leader_data[2] = 0x00;
		leader_data[3] = 0x00;
		leader_data[4] = 0x00;
		leader_data[5] = 0x01;
		leader_data[6] = 64;
		leader_data[7] = 7;
		leader_data[8] = 5;
		leader_data[9] = 3;
		tlv_init(&tlv_leader, leader_data);
		tlv_print(tlv_leader);

		tlv_leader_init(&ld_tlv, tlv_leader->value);
		thrd_process_adv(0x0400, rt64_tlv, ld_tlv);

		*/

		printf("------------------------------------------\n\n");
		break;
	case 1:


		rt64_tlv = thrd_generate_route64();

		PRINTF("rt64_tlv->id_sequence_number = %02x\n", rt64_tlv->id_sequence_number);
		PRINTF("rt64_tlv->router_id_mask = %16x\n", rt64_tlv->router_id_mask);
		PRINTF("rt64_tlv->lq[0] = %02x\n", rt64_tlv->lq_rd[0]);
		PRINTF("rt64_tlv->lq[1] = %02x\n", rt64_tlv->lq_rd[1]);
		PRINTF("rt64_tlv->lq[2] = %02x\n", rt64_tlv->lq_rd[2]);
		PRINTF("rt64_tlv->lq[3] = %02x\n", rt64_tlv->lq_rd[3]);
		PRINTF("rt64_tlv->lq[4] = %02x\n", rt64_tlv->lq_rd[4]);

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
		thrd_rdb_print_routing_database();
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

	thrd_dev.Router_ID = 1;

	/* set the pointer to the udp-socket */
	pst_udp_socket = &st_udp_socket;

	_mcast_set_own_addresses();

	_mcast_prepare();

	/* set periodic timer */
	etimer_set( &e_mcastTmr, SEND_INTERVAL * bsp_get(E_BSP_GET_TRES),
			_mcast_callback);

	return 1;
}/* demo_mcastRootInit()  */

