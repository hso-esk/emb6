/**
 * \file thrd-router-id.c
 * \author Institute for reliable Embedded Systems and Communication Electronics
 * \date 2016/05/23
 * \version 1.0
 *
 * \brief Router ID management / router ID assignment
 */

#include "emb6.h"
#include "bsp.h"
#include "thread_conf.h"
#include "ctimer.h"
#include "thrd-dev.h"
#include "er-coap.h"
#include "er-coap-engine.h"
#include "rest-engine.h"
#include "mle_management.h"
#include "thrd-router-id.h"
#include "thrd-leader-db.h"
#include "thrd-partition.h"
#include "thrd-iface.h"
#include "thrd-addr.h"
#include "thrd-send-adv.h"
#include "thrd-route.h"

#define     LOGGER_ENABLE                 LOGGER_THRD_NET
#include    "logger.h"

/*=============================================================================
                               Router ID Management
===============================================================================*/

static char *service_urls[2] =
{ "a/as", "a/ar"};

static struct ctimer leader_ct;
// static struct ctimer router_ct;

/**
 * Address Query Payload Buffer (MUST be at least 38 octets).
 */
static uint8_t addr_solicit_buf[38] = { 0 };	// 24.

static coap_packet_t packet[1]; /* This way the packet can be treated as pointer as usual. */

static size_t len = 0;						// CoAP payload length.
static tlv_t *tlv;
static net_tlv_status_t *status_tlv;
static net_tlv_rloc16_t *rloc16_tlv;
static net_tlv_router_mask_t *router_mask_tlv;

static uip_ipaddr_t leader_addr;

/*
 * CoAP Resources to be activated need to be imported through the extern keyword.
 */
extern resource_t
	thrd_res_a_as;

/* --------------------------------------------------------------------------- */

/**
 * Initialize CoAP resources.
 */
static void coap_init();

static void handle_increment_id_seq_num_timer(void *ptr);

static void handle_id_reuse_delay_timeout(void *ptr);

static size_t create_addr_solicit_req_payload(uint8_t *buf, uint8_t *ml_eid,
		uint16_t *rloc16);

static uint8_t get_unassigned_router_id(void);

/* --------------------------------------------------------------------------- */

/**
 * The Leader maintains an ID_Assignment_Set containing the IEEE 802.15.4
 * Extended Address of each device that has an assigned ID and, for currently
 * unassigned IDs, the time at which the ID may be reassigned.
 */

void
thrd_leader_init(void)
{
	// Starting a new Partition as the Leader.
	thrd_dev.net_type = THRD_DEV_NETTYPE_LEADER;

	// Initialize device's interface addresses.
	// thrd_iface_init();

	thrd_ldb_init();
	thrd_ldb_ida_empty();	// Empty ID Assignment Set.

	// Get a Router ID.
	uint8_t desired_rid = 0;
	thrd_ldb_ida_t *ida = thrd_leader_assign_rid(&desired_rid, mac_phy_config.mac_address);	// TODO Add my MAC Extended Address here (owner).
	if ( ida != NULL ) {
		LOG_RAW("Leader Init: Set Leader Router ID = %d\n", ida->ID_id);
		thrd_iface_set_router_id(ida->ID_id);
		thrd_partition_set_leader_router_id(ida->ID_id);
		thrd_partition.leader_router_id = ida->ID_id;

		// Increment ID sequence number.
		thrd_partition.ID_sequence_number++;

		// Update router ID set.
		thrd_rdb_id_t *rid;
		rid = thrd_rdb_rid_add(ida->ID_id);
		if ( rid == NULL ) {
			// This should not happen.
			return;
		}
	}
	// Set RLOC16 address and create corresponding IPv6 addresses.
	thrd_iface_set_rloc(THRD_CREATE_RLOC16(thrd_iface_get_router_id(), 0));

	coap_init();
	// Start timer for incrementing ID_sequence_number.
	ctimer_set(&leader_ct, ID_SEQUENCE_PERIOD * bsp_get(E_BSP_GET_TRES), handle_increment_id_seq_num_timer, NULL);
}

/* --------------------------------------------------------------------------- */

static void
coap_init()
{
	LOG_RAW("Router ID Assignment: Registering CoAP resources.\n\r");

	// Bind the resources to their Uri-Path.
	rest_activate_resource(&thrd_res_a_as, "a/as");
}

/* --------------------------------------------------------------------------- */

static void
handle_increment_id_seq_num_timer(void *ptr)
{
	thrd_partition.ID_sequence_number++;
	thrd_print_partition_data();
	ctimer_restart(&leader_ct);
}

/* --------------------------------------------------------------------------- */

thrd_ldb_ida_t*
thrd_leader_assign_rid(uint8_t *router_id, uint8_t *id_owner)
{
	LOG_RAW("Assigning new Router ID.\n\r");
	if ( thrd_ldb_num_ida() < 32 ) {
		thrd_ldb_ida_t *ida;

		if ( router_id != NULL ) {

			// Check whether desired Router ID is available.
			ida = thrd_ldb_ida_lookup(*router_id);

			if ( ida == NULL ) {
				ida = thrd_ldb_ida_add(*router_id, id_owner, 0);
				LOG_RAW("Router ID Assignment: Assigned Router ID = %d\n", ida->ID_id);
				return ida;
			} else {
				// If the desired Router ID currently is in use --> Looking for free Router ID.

				uint8_t r_id = get_unassigned_router_id();
				/*
				// Router ID is not available --> Looking for an unassigned Router ID.
				uint8_t id_cnt = 0;
				ida = thrd_ldb_ida_lookup(id_cnt);
				// Looking for an unassigned Router ID.
				while ( ida != NULL && id_cnt < 63 ) {
					id_cnt++;
					ida = thrd_ldb_ida_lookup(id_cnt);
				}
				*/
				// Create Router ID if available.
				if ( r_id < 63 ) {
					ida = thrd_ldb_ida_add(r_id, id_owner, 0);
					LOG_RAW("Router ID Assignment: Assigned Router ID = %d\n", ida->ID_id);
					thrd_trickle_reset();	// Reset trickle algorithm (MLE advertisement sending rate).
					return ida;
				} else {
					LOG_RAW("Router ID Assignment: No more Router IDs available.\n");
					return NULL;
				}
			}
		} else {
			uint8_t r_id = get_unassigned_router_id();
			// Create Router ID if available.
			if ( r_id < 63 ) {
				ida = thrd_ldb_ida_add(r_id, id_owner, 0);
				LOG_RAW("Router ID Assignment: Assigned Router ID = %d\n", ida->ID_id);
				thrd_trickle_reset();	// Reset trickle algorithm (MLE advertisement sending rate).
				return ida;
			} else {
				LOG_RAW("Router ID Assignment: No more Router IDs available.\n");
				return NULL;
			}
		}
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

void
thrd_leader_unassign_rid(uint8_t router_id)
{
	thrd_rdb_id_t *rid;
	thrd_ldb_ida_t *ida;
	rid = thrd_rdb_rid_lookup(router_id);
	ida = thrd_ldb_ida_lookup(router_id);

	if ( rid != NULL && ida != NULL ) {
		// Removing the router id from the Router ID Set.
		thrd_rdb_rid_rm(rid);
		// Set the router id's reuse time.
		ctimer_set(&ida->ID_reuse_time, ID_REUSE_DELAY * bsp_get(E_BSP_GET_TRES), handle_id_reuse_delay_timeout, ida);
		thrd_trickle_reset(); // Reset trickle algorithm (MLE advertisement sending rate).
	}
}

static void
handle_id_reuse_delay_timeout(void *ptr)
{
	thrd_ldb_ida_t *ida = (thrd_ldb_ida_t*) ptr;

	LOG_RAW("ID Assignment Set: Timer expired");
	if ( ida != NULL ) {
		LOG_RAW(" for ID Assignment Set Entry with Router ID = &d\n\r", ida->ID_id);
		thrd_ldb_ida_rm(ida);
	}
	return;
}

static uint8_t
get_unassigned_router_id(void)
{
	// Router ID is not available --> Looking for an unassigned Router ID.
	uint8_t id_cnt = 0;
	thrd_ldb_ida_t *ida;
	ida = thrd_ldb_ida_lookup(id_cnt);
	// Looking for an unassigned Router ID.
	while ( ida != NULL && id_cnt < 63 ) {
		id_cnt++;
		ida = thrd_ldb_ida_lookup(id_cnt);
	}
	return id_cnt;
}

/* --------------------------------------------------------------------------- */

void
thrd_request_router_id(uint8_t *router_id)
{
	LOG_RAW("Sending Router ID Request.\n");
	// uint16_t rloc16 = THRD_CREATE_RLOC16(*router_id, 0); // TODO Create RLOC16 (not here! --> After Router ID).
	len = create_addr_solicit_req_payload(addr_solicit_buf, &thrd_iface.ml_eid.u8[8], &thrd_iface.rloc16);

	thrd_create_meshlocal_prefix(&leader_addr);
	thrd_create_rloc_iid(&leader_addr, THRD_CREATE_RLOC16(thrd_partition.leader_router_id, 0));


	// THRD_LINK_LOCAL_ALL_NODES_ADDR(&leader_addr);

	coap_init_message(packet, COAP_TYPE_CON, COAP_POST, 0);
	coap_set_header_uri_path(packet, service_urls[0]);
	coap_set_payload(packet, addr_solicit_buf, len);


	coap_nonblocking_request(&leader_addr, UIP_HTONS(COAP_DEFAULT_PORT), packet, thrd_addr_solicit_chunk_handler); // TODO Changing CoAP Port.
	LOG_RAW("Sending Router ID Request to Partition Leader [");

	LOG_IP6ADDR(&leader_addr);

	LOG_RAW("]\n\r");
}

/* --------------------------------------------------------------------------- */

static size_t
create_addr_solicit_req_payload(uint8_t *buf, uint8_t *ml_eid, uint16_t *rloc16)
{
	if ( buf != NULL && ml_eid != NULL ) {
		// Create ML-EID TLV.
		buf[0] = NET_TLV_ML_EID;
		buf[1] = 8;
		memcpy(&buf[2], ml_eid, 8);
		if ( rloc16 != NULL ) {
			// Create RLOC16 TLV.
			buf[10] = NET_TLV_RLOC16;
			buf[11] = 2;
			memcpy(&buf[12], rloc16, 2);
			return 14;
		}
		return 10;
	}
	return 0;
}

/* --------------------------------------------------------------------------- */

/* This function is will be passed to coap_nonblocking_request() to handle responses. */
void
thrd_addr_solicit_chunk_handler(void *response)
{
	LOG_RAW("thrd_addr_solicit_chunk_handler: Received response!\n\r");
    const uint8_t *chunk;
    if ( !response ) {

    } else {
    	int payload_len = coap_get_payload(response, &chunk);
    	if ( payload_len >= 3 ) {
    		// TODO Process payload -> Receipt of Address Solicit Response.
    		tlv = (tlv_t*) &chunk[0];
    		if ( tlv->type == NET_TLV_STATUS && tlv->length == 1 ) {
    			status_tlv = (net_tlv_status_t*) tlv->value;
    			LOG_RAW("Status = %d\n", status_tlv->status);

    			if ( status_tlv->status == 0 && payload_len >= 16 ) {
    				// Success.
    				tlv = (tlv_t*) &chunk[3];
    				if ( tlv->type == NET_TLV_RLOC16 && tlv->length == 2 ) {
    					rloc16_tlv = (net_tlv_rloc16_t*) tlv->value;
    					// Set the interface's RLOC16 and update ML-RLOC and LL-RLOC addresses.
    					thrd_iface_set_rloc(rloc16_tlv->rloc16);
    					thrd_set_dev_net_type(THRD_DEV_NETTYPE_ROUTER);
    					LOG_RAW("RLOC16 = %04x\n", rloc16_tlv->rloc16);
    				}
    				tlv = (tlv_t*) &chunk[7];
    				if ( tlv->type == NET_TLV_ROUTER_MASK && tlv->length == 9 ) {
    					router_mask_tlv = (net_tlv_router_mask_t*) tlv->value;
    					thrd_partition_set_id_seq_number(router_mask_tlv->id_sequence_number);
    					// LOG_RAW("ID Sequence Number = %d\n", router_mask_tlv->id_sequence_number);
    					// LOG_RAW("Router ID Mask = %16x\n", router_mask_tlv->router_id_mask);
    				}
    			}
    		}
    	}
    	thrd_dev_print_dev_info();
    }
    reply_for_mle_childID_request(&thrd_iface.router_id);
}

/* --------------------------------------------------------------------------- */

uint64_t
thrd_create_router_id_mask()
{
	thrd_rdb_id_t *rid;
	// Router ID Mask and Link Quality and Router Data.
	uint64_t router_id_mask = 0x0000000000000000;
	for ( rid = thrd_rdb_rid_head(); rid != NULL; rid = thrd_rdb_rid_next(rid) ) {
		router_id_mask |= (0x8000000000000000 >> rid->router_id);
	}
	return router_id_mask;
}

/* --------------------------------------------------------------------------- */

