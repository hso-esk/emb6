/*
 * thrd-addr-query.c
 *
 *  Created on: 6 Jun 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  Thread Address Query.
 */

#include "emb6.h"
#include "thread_conf.h"
#include "stdlib.h"
#include "clist.h"
#include "memb.h"
#include "rip.h"
#include "net_tlv.h"
#include "uip.h"
#include "uip-ds6.h"

#include "ctimer.h"

#include "er-coap.h"
#include "er-coap-engine.h"
#include "rest-engine.h"

#include "thrd-addr-query.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"

/*
 ********************************************************************************
 *                          LOCAL FUNCTION DECLARATIONS
 ********************************************************************************
 */

/**
 * Initialize CoAP Engine and related resources.
 */
static void coap_init();

/**
* Create Address Notification Payload.
* @param buf A buffer of (at least) size 38 (octets).
* @param target_eid The Target EID.
* @param rloc16 The Corresponding RLOC16.
* @param ml_eid The Corresponding ML-EID.
* @param last_trans_time The last transaction time (optional). NULL if not used.
 * @return The payload length.
 */
static size_t create_addr_ntf_resp_payload(uint8_t *buf, uip_ipaddr_t *target_eid,
		uint16_t *rloc16, uint8_t *ml_eid, clock_time_t *last_trans_time);

/* --------------------------------------------------------------------------- */

/**
 * Create Address Query Request Payload.
 * @param buf A buffer of (at least) size 18 (octets).
 * @param target_eid The Target EID.
 * @return The payload length.
 */
static size_t create_addr_qry_req_payload(uint8_t *buf, uip_ipaddr_t *target_eid);

/* --------------------------------------------------------------------------- */

/**
 * Create Address Error Notification Payload.
 * @param buf A buffer of (at least) size 28 (octets).
 * @param target_eid The Target EID.
 * @param ml_eid The corresponding ML-EID.
 * @return
 */
static uint8_t create_addr_err_ntf_payload(uint8_t *buf, uip_ipaddr_t *target_eid,
		uint8_t *ml_eid);

/**
 * Get the RFD Child IPv6 Address Prefix Type.
 * @param ip_addr An IPv6 address.
 * @return The address type as thrd_rfd_child_prefix_type_t.
 */
static thrd_rfd_child_prefix_type_t get_rfd_child_addr_prefix_type(uip_ipaddr_t *ip_addr);

/*
 ********************************************************************************
 *                               LOCAL VARIABLES
 ********************************************************************************
 */

uip_ipaddr_t ipaddr;

static struct ctimer aq_timer;		// Timer for AQ_Timeout.

static char *service_urls[NUMBER_OF_URLS] =
{ "a/aq", "a/an", "a/ae"};

/**
 * Realm-Local-All-Routers address.
 */
uip_ipaddr_t rlar_ipaddr;

/**
 * Local Address Set
 */
LIST(localAddrSet_list);
MEMB(localAddrSet_memb, thrd_local_addr_t, THRD_MAX_LOCAL_ADDRESSES);

/**
 * RFD Child Address Set.
 * Link-Local child addresses.
 */
LIST(rfdChildLLAddrSet_list);
MEMB(rfdChildLLAddrSet_memb, thrd_rfd_child_addr_t, THRD_MAX_RFD_CHILD_ADDRESSES_LL);

/**
 * RFD Child Address Set.
 * Mesh-Local child addresses.
 */
LIST(rfdChildMLAddrSet_list);
MEMB(rfdChildMLAddrSet_memb, thrd_rfd_child_addr_t, THRD_MAX_RFD_CHILD_ADDRESSES_ML);

/**
 * RFD Child Address Set.
 * Other-Scope child addresses.
 */
LIST(rfdChildOSAddrSet_list);
MEMB(rfdChildOSAddrSet_memb, thrd_rfd_child_addr_t, THRD_MAX_RFD_CHILD_ADDRESSES_OS);

/**
 * Address Query Set.
 */
LIST(addrQuerySet_list);
MEMB(addrQuerySet_memb, thrd_addr_qry_t, THRD_MAX_ADDRESS_QUERIES);

/* Number of currently stored EIDs in the Local Address Set. */
static uint8_t num_eids = 0;
static uint8_t num_rfd_ll_addr = 0;
static uint8_t num_rfd_ml_addr = 0;
static uint8_t num_rfd_os_addr = 0;
static uint8_t num_addr_qry = 0;

static uip_ipaddr_t prefix_ll, prefix_ml;

/*
 * CoAP Resources to be activated need to be imported through the extern keyword.
 */
extern resource_t
	thrd_res_a_aq,
	thrd_res_a_an,
	thrd_res_a_ae;

/*
 ********************************************************************************
 *                               GLOBAL VARIABLES
 ********************************************************************************
 */

/*
 ********************************************************************************
 *                           LOCAL FUNCTION DEFINITIONS
 ********************************************************************************
 */

void
thrd_eid_rloc_db_init(void)
{
	// Initialize Local Address Set.
	memb_init(&localAddrSet_memb);
	list_init(localAddrSet_list);

	// Initialize RFD Child Address Set.
	memb_init(&rfdChildLLAddrSet_memb);		// Link-Local child addresses.
	list_init(rfdChildLLAddrSet_list);

	memb_init(&rfdChildMLAddrSet_memb);		// Mesh-Local child addresses.
	list_init(rfdChildMLAddrSet_list);

	memb_init(&rfdChildOSAddrSet_memb);		// Other-Scope child addresses.
	list_init(rfdChildOSAddrSet_list);

	// Initialize Address Query Set.
	memb_init(&addrQuerySet_memb);
	list_init(addrQuerySet_list);

	uip_ip6addr(&prefix_ll, 0xfe80, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000);
	uip_ip6addr(&prefix_ml, 0xfe80, 0x0000, 0x0000, 0x0001, 0x0000, 0x0000, 0x0000, 0x0000);

	// Subscribe to Router and REED addresses.
	// Link-Local-Add-Routers Address.
	THRD_LINK_LOCAL_ALL_ROUTERS_ADDR(&ipaddr);
	uip_ds6_maddr_add(&ipaddr);
	// Realm-Local-All-Routers Address.
	THRD_REALM_LOCAL_ALL_ROUTERS_ADDR(&ipaddr);
	uip_ds6_maddr_add(&ipaddr);

	coap_init();
}

/* --------------------------------------------------------------------------- */

static void
coap_init()
{
	PRINTF("thrd_eid_rloc_coap_init: Starting EID-to-RLOC Mapping (CoAP).\n\r");
	THRD_REALM_LOCAL_ALL_ROUTERS_ADDR(&rlar_ipaddr);
	/* Receives all CoAP messages */
	coap_init_engine();

	/* Initialize the REST engine. */
	rest_init_engine();

	// Bind the resources to their Uri-Path.
	rest_activate_resource(&thrd_res_a_aq, "a/aq");
	rest_activate_resource(&thrd_res_a_an, "a/an");
	rest_activate_resource(&thrd_res_a_ae, "a/ae");
}

/* --------------------------------------------------------------------------- */

static thrd_rfd_child_prefix_type_t
get_rfd_child_addr_prefix_type(uip_ipaddr_t *ip_addr)
{
	if ( uip_ipaddr_prefixcmp(&prefix_ll, ip_addr, 64) ) {
		return THRD_RFD_CHILD_PREFIX_LL;
	} else if ( uip_ipaddr_prefixcmp(&prefix_ml, ip_addr, 64) ) {
		return THRD_RFD_CHILD_PREFIX_ML;
	} else {
		return THRD_RFD_CHILD_PREFIX_OS;
	}
}

/* --------------------------------------------------------------------------- */
/* --------------------------- Local Address Set ----------------------------- */
/* --------------------------------------------------------------------------- */

uint8_t
thrd_local_addr_num()
{
	return num_eids;
}

/* --------------------------------------------------------------------------- */

thrd_local_addr_t
*thrd_local_addr_head(void)
{
	return list_head(localAddrSet_list);
}

/* --------------------------------------------------------------------------- */

thrd_local_addr_t
*thrd_local_addr_next(thrd_local_addr_t *i)
{
	if ( i != NULL ) {
		thrd_local_addr_t *n = list_item_next(i);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

thrd_local_addr_t
*thrd_local_addr_lookup(uip_ipaddr_t eid)
{
	thrd_local_addr_t *localAddr;
	thrd_local_addr_t *found_localAddr;

	PRINTF("thrd_local_addr_lookup: Looking up EID: ");
	PRINT6ADDR(&eid);
	PRINTF("\n\r");

	found_localAddr = NULL;
	for ( localAddr = thrd_local_addr_head(); localAddr != NULL; localAddr = thrd_local_addr_next(localAddr) ) {
		if ( uip_ipaddr_cmp(&eid, &(localAddr->EID)) ) {
			found_localAddr = localAddr;
			break;
		}
	}

	if ( found_localAddr != NULL ) {
		PRINTF("thrd_local_addr_lookup: Found EID: ");
		PRINT6ADDR(&eid);
		PRINTF("\n\r");
	} else {
		PRINTF("thrd_local_addr_lookup: No EID found.\n\r");
	}
	return found_localAddr;
}

/* --------------------------------------------------------------------------- */

thrd_local_addr_t
*thrd_local_addr_add(uip_ipaddr_t eid)
{
	thrd_local_addr_t *localAddr;

	/* Find the corresponding Router ID entry (Router ID Set). */

	localAddr = thrd_local_addr_lookup(eid);

	/* Check whether the given router id already has an entry in the Router ID Set. */
	if ( localAddr == NULL ) {
		PRINTF("thrd_local_addr_add: Unknown EID: ");
		PRINT6ADDR(&eid);
		PRINTF("\n\r");

		/* If there is no router id entry, create one. We first need to
		 * check if we have room for this router id. If not, we remove the
		 * least recently used one we have. */

		if ( thrd_local_addr_num() == THRD_MAX_LOCAL_ADDRESSES ) {
			/* Removing the oldest router id entry from the Router ID Set. The
			 * least recently used link is the first router id on the set. */
			thrd_local_addr_t *oldest;

			oldest = list_tail(localAddrSet_list);
			thrd_local_addr_rm(oldest);
		}

		/* Allocate a router id entry and populate it. */
		localAddr = memb_alloc(&localAddrSet_memb);

		if ( localAddr == NULL ) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			PRINTF("thrd_local_addr_add: Could not allocate EID.\n");
			return NULL;
		}

		localAddr->EID = eid;

		/* Add new router id first - assuming that there is a reason to add this
		 * and that there is a packet coming soon. */
		list_push(localAddrSet_list, localAddr);

		PRINTF("thrd_local_addr_add: Added EID: ");
		PRINT6ADDR(&eid);
		PRINTF("\n\r");

		num_eids++;

		PRINTF("thrd_local_addr_add: num_eids %d\n\r", num_eids);

	} else {

		PRINTF(ANSI_COLOR_RED "thrd_local_addr_add: EID is already known for ");
		PRINTF("%d\n", eid);
		PRINTF(ANSI_COLOR_RESET "\n\r");

		PRINTF("thrd_local_addr_add: num_eids %d\n\r", num_eids);
		PRINTF("-----------------------------------------------------\n\r");

		return NULL;
	}

	PRINTF("-----------------------------------------------------\n\r");

	return localAddr;
}

/* --------------------------------------------------------------------------- */

void
thrd_local_addr_rm(thrd_local_addr_t *localAddr)
{
	if ( localAddr != NULL ) {
		PRINTF("thrd_local_addr_rm: Removing EID from 'Local Address Set' with EID: ");
		PRINT6ADDR(&localAddr->EID);
		PRINTF("\n\r");

		/* Remove the router id from the Router ID Set. */
		list_remove(localAddrSet_list, localAddr);
		memb_free(&localAddrSet_memb, localAddr);

		num_eids--;

		PRINTF("thrd_local_addr_rm: num_eids %d\n\r", num_eids);
	}
}

/* --------------------------------------------------------------------------- */

void
thrd_local_addr_empty()
{
	thrd_local_addr_t *localAddr;
	thrd_local_addr_t *localAddr_nxt;
	PRINTF("thrd_local_addr_empty: Removing all (%d) assigned EIDs from 'Local Address Set'.", num_eids);
	PRINTF("\n\r");
	localAddr = thrd_local_addr_head();
	localAddr_nxt = localAddr;
	while ( localAddr_nxt != NULL ) {
		localAddr = localAddr_nxt;
		localAddr_nxt = localAddr->next;
		thrd_local_addr_rm(localAddr);
	}
}

/* --------------------------------------------------------------------------- */
/* ------------------------- RFD Child Address Set --------------------------- */
/* --------------------------------------------------------------------------- */

size_t
thrd_rfd_child_ll_addr_num()
{
	return num_rfd_ll_addr;
}

/* --------------------------------------------------------------------------- */

size_t
thrd_rfd_child_ml_addr_num()
{
	return num_rfd_ml_addr;
}

/* --------------------------------------------------------------------------- */

size_t
thrd_rfd_child_os_addr_num()
{
	return num_rfd_os_addr;
}

/* --------------------------------------------------------------------------- */

thrd_rfd_child_addr_t
*thrd_rfd_child_addr_head(thrd_rfd_child_prefix_type_t addr_type)
{
	switch (addr_type) {
	case THRD_RFD_CHILD_PREFIX_LL:
		return list_head(rfdChildLLAddrSet_list);
		break;
	case THRD_RFD_CHILD_PREFIX_ML:
		return list_head(rfdChildMLAddrSet_list);
		break;
	case THRD_RFD_CHILD_PREFIX_OS:
		return list_head(rfdChildOSAddrSet_list);
		break;
	default:
		return list_head(rfdChildOSAddrSet_list);
	}
}

/* --------------------------------------------------------------------------- */

thrd_rfd_child_addr_t
*thrd_rfd_child_addr_next(thrd_rfd_child_addr_t *i)
{
	if ( i != NULL ) {
		thrd_rfd_child_addr_t *n = list_item_next(i);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

thrd_rfd_child_addr_t
*thrd_rfd_child_addr_lookup(uip_ipaddr_t child_addr)
{
	thrd_rfd_child_addr_t *childAddr;
	thrd_rfd_child_addr_t *found_childAddr;

	PRINTF("thrd_rfd_child_addr_lookup: Looking up EID: ");
	PRINT6ADDR(&child_addr);
	PRINTF("\n\r");

	found_childAddr = NULL;
	for ( childAddr = thrd_rfd_child_addr_head(get_rfd_child_addr_prefix_type(&child_addr));
			childAddr != NULL; childAddr = thrd_rfd_child_addr_next(childAddr) ) {
		if ( uip_ipaddr_cmp(&child_addr, &(childAddr->childAddr)) ) {
			found_childAddr = childAddr;
			break;
		}
	}

	if ( found_childAddr != NULL ) {
		PRINTF("thrd_rfd_child_addr_lookup: Found RFD Child Address: ");
		PRINT6ADDR(&child_addr);
		PRINTF("\n\r");
	} else {
		PRINTF("thrd_rfd_child_addr_lookup: No RFD Child Address found.\n\r");
	}
	return found_childAddr;
}

/* --------------------------------------------------------------------------- */

thrd_rfd_child_addr_t
*thrd_rfd_child_addr_add(uip_ipaddr_t child_addr)
{
	thrd_rfd_child_addr_t *childAddr;
	thrd_rfd_child_prefix_type_t prefix_type = get_rfd_child_addr_prefix_type(&child_addr);

	/* Find the corresponding Router ID entry (Router ID Set). */

	childAddr = thrd_rfd_child_addr_lookup(child_addr);

	/* Check whether the given router id already has an entry in the Router ID Set. */
	if ( childAddr == NULL ) {
		PRINTF("thrd_rfd_child_addr_add: Unknown RFD Child Address: ");
		PRINT6ADDR(&child_addr);
		PRINTF("\n\r");

		/* If there is no router id entry, create one. We first need to
		 * check if we have room for this router id. If not, we remove the
		 * least recently used one we have. */

		thrd_rfd_child_addr_t *oldest;

		switch (prefix_type) {
		case THRD_RFD_CHILD_PREFIX_LL:
			if ( thrd_rfd_child_ll_addr_num() == THRD_MAX_RFD_CHILD_ADDRESSES_LL ) {
				oldest = list_tail(rfdChildLLAddrSet_list);
				thrd_rfd_child_addr_rm(oldest);
			}
			/* Allocate a router id entry and populate it. */
			childAddr = memb_alloc(&rfdChildLLAddrSet_memb);

			if ( childAddr == NULL ) {
				/* This should not happen, as we explicitly deallocated one
				 * link set entry above. */
				PRINTF("thrd_rfd_child_addr_add: Could not allocate RFD Child Address.\n");
				return NULL;
			}

			childAddr->childAddr = child_addr;

			/* Add new router id first - assuming that there is a reason to add this
			 * and that there is a packet coming soon. */
			list_push(rfdChildLLAddrSet_list, childAddr);

			PRINTF("thrd_rfd_child_addr_add: Added Link-Local RFD Child Address ");
			PRINT6ADDR(&child_addr);
			PRINTF("\n\r");

			num_rfd_ll_addr++;
			PRINTF("thrd_rfd_child_addr_add: num_rfd_ll_addr %d\n\r", num_rfd_ll_addr);
			break;
		case THRD_RFD_CHILD_PREFIX_ML:
			if ( thrd_rfd_child_ml_addr_num() == THRD_MAX_RFD_CHILD_ADDRESSES_ML ) {
				oldest = list_tail(rfdChildMLAddrSet_list);
				thrd_rfd_child_addr_rm(oldest);
			}
			/* Allocate a router id entry and populate it. */
			childAddr = memb_alloc(&rfdChildMLAddrSet_memb);

			if ( childAddr == NULL ) {
				/* This should not happen, as we explicitly deallocated one
				 * link set entry above. */
				PRINTF("thrd_rfd_child_addr_add: Could not allocate RFD Child Address.\n");
				return NULL;
			}

			childAddr->childAddr = child_addr;

			/* Add new router id first - assuming that there is a reason to add this
			 * and that there is a packet coming soon. */
			list_push(rfdChildMLAddrSet_list, childAddr);

			PRINTF("thrd_rfd_child_addr_add: Added Mesh-Local RFD Child Address ");
			PRINT6ADDR(&child_addr);
			PRINTF("\n\r");

			num_rfd_ml_addr++;
			PRINTF("thrd_rfd_child_addr_add: num_rfd_ml_addr %d\n\r", num_rfd_ml_addr);
			break;
		case THRD_RFD_CHILD_PREFIX_OS:
			if ( thrd_rfd_child_os_addr_num() == THRD_MAX_RFD_CHILD_ADDRESSES_OS ) {
				oldest = list_tail(rfdChildOSAddrSet_list);
				thrd_rfd_child_addr_rm(oldest);
			}
			/* Allocate a router id entry and populate it. */
			childAddr = memb_alloc(&rfdChildOSAddrSet_memb);

			if ( childAddr == NULL ) {
				/* This should not happen, as we explicitly deallocated one
				 * link set entry above. */
				PRINTF("thrd_rfd_child_addr_add: Could not allocate RFD Child Address.\n");
				return NULL;
			}

			childAddr->childAddr = child_addr;

			/* Add new router id first - assuming that there is a reason to add this
			 * and that there is a packet coming soon. */
			list_push(rfdChildOSAddrSet_list, childAddr);

			PRINTF("thrd_rfd_child_addr_add: Added Other-Scope RFD Child Address ");
			PRINT6ADDR(&child_addr);
			PRINTF("\n\r");

			num_rfd_os_addr++;
			PRINTF("thrd_rfd_child_addr_add: num_rfd_os_addr %d\n\r", num_rfd_os_addr);
			break;
		default:
			if ( thrd_rfd_child_os_addr_num() == THRD_MAX_RFD_CHILD_ADDRESSES_OS ) {
				oldest = list_tail(rfdChildOSAddrSet_list);
				thrd_rfd_child_addr_rm(oldest);
			}
			/* Allocate a router id entry and populate it. */
			childAddr = memb_alloc(&rfdChildOSAddrSet_memb);

			if ( childAddr == NULL ) {
				/* This should not happen, as we explicitly deallocated one
				 * link set entry above. */
				PRINTF("thrd_rfd_child_addr_add: Could not allocate RFD Child Address.\n");
				return NULL;
			}

			childAddr->childAddr = child_addr;

			/* Add new router id first - assuming that there is a reason to add this
			 * and that there is a packet coming soon. */
			list_push(rfdChildOSAddrSet_list, childAddr);

			PRINTF("thrd_rfd_child_addr_add: Added Other-Scope RFD Child Address ");
			PRINT6ADDR(&child_addr);
			PRINTF("\n\r");

			num_rfd_os_addr++;
			PRINTF("thrd_rfd_child_addr_add: num_rfd_os_addr %d\n\r", num_rfd_os_addr);
			break;
		}

	} else {

		PRINTF(ANSI_COLOR_RED "thrd_rfd_child_addr_add: RFD Child Address is already known for ");
		PRINT6ADDR(&child_addr);
		PRINTF(ANSI_COLOR_RESET "\n\r");
		PRINTF("-----------------------------------------------------\n\r");

		return NULL;
	}

	PRINTF("-----------------------------------------------------\n\r");

	return childAddr;
}

/* --------------------------------------------------------------------------- */

void
thrd_rfd_child_addr_rm(thrd_rfd_child_addr_t *child_addr)
{
	if ( child_addr != NULL ) {
		thrd_rfd_child_prefix_type_t prefix_type = get_rfd_child_addr_prefix_type(&child_addr->childAddr);
		PRINTF("thrd_rfd_child_addr_rm: Removing RFD Child Address from 'RFD Child Address Set' with EID: ");
		PRINT6ADDR(&child_addr->childAddr);
		PRINTF("\n\r");

		switch ( prefix_type ) {
		case THRD_RFD_CHILD_PREFIX_LL:
			/* Remove the router id from the Router ID Set. */
			list_remove(rfdChildLLAddrSet_list, child_addr);
			memb_free(&rfdChildLLAddrSet_memb, child_addr);
			num_rfd_ll_addr--;
			PRINTF("thrd_rfd_child_addr_rm: num_rfd_ll_addr %d\n\r", num_rfd_ll_addr);
			break;
		case THRD_RFD_CHILD_PREFIX_ML:
			/* Remove the router id from the Router ID Set. */
			list_remove(rfdChildMLAddrSet_list, child_addr);
			memb_free(&rfdChildMLAddrSet_memb, child_addr);
			num_rfd_ml_addr--;
			PRINTF("thrd_rfd_child_addr_rm: num_rfd_ml_addr %d\n\r", num_rfd_ml_addr);
			break;
		case THRD_RFD_CHILD_PREFIX_OS:
			/* Remove the router id from the Router ID Set. */
			list_remove(rfdChildOSAddrSet_list, child_addr);
			memb_free(&rfdChildOSAddrSet_memb, child_addr);
			num_rfd_os_addr--;
			PRINTF("thrd_rfd_child_addr_rm: num_rfd_os_addr %d\n\r", num_rfd_os_addr);
			break;
		default:
			/* Remove the router id from the Router ID Set. */
			list_remove(rfdChildOSAddrSet_list, child_addr);
			memb_free(&rfdChildOSAddrSet_memb, child_addr);
			num_rfd_os_addr--;
			PRINTF("thrd_rfd_child_addr_rm: num_rfd_os_addr %d\n\r", num_rfd_os_addr);
			break;
		}
	}
}

/* --------------------------------------------------------------------------- */

void
thrd_rfd_child_addr_empty()
{
	thrd_rfd_child_addr_t *childAddr;
	thrd_rfd_child_addr_t *childAddr_nxt;
	PRINTF("thrd_rfd_child_addr_empty: Removing all (%d) assigned EIDs from 'RFD Child Address Set'.", num_rfd_ll_addr + num_rfd_ml_addr + num_rfd_os_addr);
	PRINTF("\n\r");
	// Empty Link-Local RFD Child Address Set.
	childAddr = thrd_rfd_child_addr_head(THRD_RFD_CHILD_PREFIX_LL);
	childAddr_nxt = childAddr;
	while ( childAddr_nxt != NULL ) {
		childAddr = childAddr_nxt;
		childAddr_nxt = childAddr->next;
		thrd_rfd_child_addr_rm(childAddr);
	}
	// Empty Mesh-Local RFD Child Address Set.
	childAddr = thrd_rfd_child_addr_head(THRD_RFD_CHILD_PREFIX_ML);
	childAddr_nxt = childAddr;
	while ( childAddr_nxt != NULL ) {
		childAddr = childAddr_nxt;
		childAddr_nxt = childAddr->next;
		thrd_rfd_child_addr_rm(childAddr);
	}
	// Empty Other-Scope RFD Child Address Set.
	childAddr = thrd_rfd_child_addr_head(THRD_RFD_CHILD_PREFIX_OS);
	childAddr_nxt = childAddr;
	while ( childAddr_nxt != NULL ) {
		childAddr = childAddr_nxt;
		childAddr_nxt = childAddr->next;
		thrd_rfd_child_addr_rm(childAddr);
	}
}

/* --------------------------------------------------------------------------- */
/* --------------------------- Address Query Set ----------------------------- */
/* --------------------------------------------------------------------------- */

uint8_t
thrd_addr_qry_num()
{
	return num_addr_qry;
}

/* --------------------------------------------------------------------------- */

thrd_addr_qry_t
*thrd_addr_qry_head(void)
{
	return list_head(addrQuerySet_list);
}

/* --------------------------------------------------------------------------- */

thrd_addr_qry_t
*thrd_addr_qry_next(thrd_addr_qry_t *i)
{
	if ( i != NULL ) {
		thrd_addr_qry_t *n = list_item_next(i);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

thrd_addr_qry_t
*thrd_addr_qry_lookup(uip_ipaddr_t eid)
{
	thrd_addr_qry_t *addrQr;
	thrd_addr_qry_t *found_addrQr;

	PRINTF("thrd_addr_qr_lookup: Looking up Address Query for EID: ");
	PRINT6ADDR(&eid);
	PRINTF("\n\r");

	found_addrQr = NULL;
	for ( addrQr = thrd_addr_qry_head(); addrQr != NULL; addrQr = thrd_addr_qry_next(addrQr) ) {
		if ( uip_ipaddr_cmp(&eid, &(addrQr->EID)) ) {
			found_addrQr = addrQr;
			break;
		}
	}

	if ( found_addrQr != NULL ) {
		PRINTF("thrd_addr_qr_lookup: Found Address Query for EID: ");
		PRINT6ADDR(&eid);
		PRINTF("\n\r");
	} else {
		PRINTF("thrd_addr_qr_lookup: No Address Query for EID found.\n\r");
	}
	return found_addrQr;
}

/* --------------------------------------------------------------------------- */

thrd_addr_qry_t
*thrd_addr_qry_add(uip_ipaddr_t eid, clock_time_t timeout, clock_time_t retry_delay)
{
	thrd_addr_qry_t *addrQr;

	/* Find the corresponding Router ID entry (Router ID Set). */

	addrQr = thrd_addr_qry_lookup(eid);

	/* Check whether the given router id already has an entry in the Router ID Set. */
	if ( addrQr == NULL ) {
		PRINTF("thrd_addr_qr_add: Unknown Address Query for EID: ");
		PRINT6ADDR(&eid);
		PRINTF("\n\r");

		/* If there is no router id entry, create one. We first need to
		 * check if we have room for this router id. If not, we remove the
		 * least recently used one we have. */

		if ( thrd_addr_qry_num() == THRD_MAX_ADDRESS_QUERIES ) {
			/* Removing the oldest router id entry from the Router ID Set. The
			 * least recently used link is the first router id on the set. */
			thrd_addr_qry_t *oldest;

			oldest = list_tail(addrQuerySet_list);
			thrd_addr_qry_rm(oldest);
		}

		/* Allocate a router id entry and populate it. */
		addrQr = memb_alloc(&addrQuerySet_memb);

		if ( addrQr == NULL ) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			PRINTF("thrd_addr_qr_add: Could not allocate Address Query.\n");
			return NULL;
		}

		addrQr->EID = eid;
		addrQr->AQ_Timeout = timeout;
		addrQr->AQ_Failures = 0;
		addrQr->AQ_Retry_Delay = retry_delay;

		/* Add new router id first - assuming that there is a reason to add this
		 * and that there is a packet coming soon. */
		list_push(addrQuerySet_list, addrQr);

		PRINTF("thrd_addr_qr_add: Added Address Query for EID: ");
		PRINT6ADDR(&eid);
		PRINTF("\n\r");

		num_addr_qry++;

		PRINTF("thrd_addr_qr_add: num_addr_qry %d\n\r", num_addr_qry);

	} else {

		PRINTF(ANSI_COLOR_RED "thrd_addr_qr_add: Address Query is already known for EID ");
		PRINTF("%d\n", eid);
		PRINTF(ANSI_COLOR_RESET "\n\r");

		PRINTF("thrd_addr_qr_add: num_addr_qry %d\n\r", num_addr_qry);
		PRINTF("-----------------------------------------------------\n\r");

		return NULL;
	}

	PRINTF("-----------------------------------------------------\n\r");

	return addrQr;
}

/* --------------------------------------------------------------------------- */

void
thrd_addr_qry_rm(thrd_addr_qry_t *addrQr)
{
	if ( addrQr != NULL ) {
		PRINTF("thrd_local_addr_rm: Removing Address Query from 'Address Query Set' with EID: ");
		PRINT6ADDR(&addrQr->EID);
		PRINTF("\n\r");

		/* Remove the router id from the Router ID Set. */
		list_remove(addrQuerySet_list, addrQr);
		memb_free(&addrQuerySet_memb, addrQr);

		num_addr_qry--;

		PRINTF("thrd_addr_qr_rm: num_addr_qry %d\n\r", num_addr_qry);
	}
}

/* --------------------------------------------------------------------------- */

void
thrd_addr_qry_empty()
{
	thrd_addr_qry_t *addrQr;
	thrd_addr_qry_t *addrQr_nxt;
	PRINTF("thrd_addr_qr_empty: Removing all (%d) assigned Address Queries from 'Address Query Set'.", num_addr_qry);
	PRINTF("\n\r");
	addrQr = thrd_addr_qry_head();
	addrQr_nxt = addrQr;
	while ( addrQr_nxt != NULL ) {
		addrQr = addrQr_nxt;
		addrQr_nxt = addrQr->next;
		thrd_addr_qry_rm(addrQr);
	}
}

/* --------------------------------------------------------------------------- */
/* ----------------------------- Address Query ------------------------------- */
/* --------------------------------------------------------------------------- */

/**
 * Address Query Payload Buffer (MUST be at least 38 octets).
 */
static uint8_t addr_qry_buf[38] = { 0 };

static coap_packet_t packet[1]; /* This way the packet can be treated as pointer as usual. */

/* --------------------------------------------------------------------------- */

/* This function is will be passed to coap_nonblocking_request() to handle responses. */
void
thrd_addr_ntf_chunk_handler(void *response)
{
	PRINTF("thrd_addr_ntf_chunk_handler: Got called!");
    const uint8_t *chunk;
    if ( !response ) {

    } else {
    	int payload_len = coap_get_payload(response, &chunk);
    	if ( payload_len == 32 ) {
    		// TODO Process payload -> Receipt of Address Query Messages.
    	} else if ( payload_len == 38 ) {
    		// TODO Process payload -> Receipt of Address Query Messages.
    	}
    }
}

/*---------------------------------------------------------------------------*/
/*
 * Called after Address Query Timer has expired.
 */
static void
thrd_handle_timeout(void *ptr)
{
	thrd_addr_qry_t *addr_qry = (thrd_addr_qry_t*) ptr;

	PRINTF("Address Query Timer: Timer expired for Address Query Entry with EID = ");
	PRINT6ADDR(&addr_qry->EID);
	PRINTF("\n\r");

	addr_qry->AQ_Failures++;
	addr_qry->AQ_Retry_Delay = THRD_AQ_INITIAL_RETRY_DELAY << addr_qry->AQ_Failures;

	// TODO

	return;
}

/* --------------------------------------------------------------------------- */

void
thrd_addr_qry_request(uip_ipaddr_t *target_eid)
{

	thrd_addr_qry_t *addr_qry;
	addr_qry = thrd_addr_qry_lookup(*target_eid);
	if (addr_qry == NULL  ) {
		// Add a new entry to the Address Query Set.
		addr_qry = thrd_addr_qry_add(*target_eid,
				THRD_AQ_TIMEOUT * bsp_get(E_BSP_GET_TRES),
				THRD_AQ_INITIAL_RETRY_DELAY * bsp_get(E_BSP_GET_TRES));
		ctimer_set(&aq_timer, THRD_AQ_TIMEOUT * bsp_get(E_BSP_GET_TRES), thrd_handle_timeout, addr_qry);
	} else {
		if ( addr_qry->AQ_Timeout != 0 ) {
			// TODO
		}
		if ( addr_qry->AQ_Timeout == 0 && addr_qry->AQ_Retry_Delay == 0 ) {
			// TODO
		}
		if ( addr_qry->AQ_Timeout == 0 && addr_qry->AQ_Retry_Delay != 0 ) {
			// Drop the packet.
			return;
		}
	}
	size_t len = create_addr_qry_req_payload(&addr_qry_buf[0], target_eid);

	PRINTF("thrd_addr_qry_request: payload_len = %d\n", len);

	if ( len > 0 ) {
		coap_init_message(packet, COAP_TYPE_NON, COAP_POST, 0);
		coap_set_header_uri_path(packet, service_urls[0]);
		coap_set_payload(packet, addr_qry_buf, len);
		coap_nonblocking_request(&rlar_ipaddr, UIP_HTONS(COAP_DEFAULT_PORT), packet, NULL); // TODO Changing CoAP Port.
	}
}

/* --------------------------------------------------------------------------- */

static size_t
create_addr_qry_req_payload(uint8_t *buf, uip_ipaddr_t *target_eid)
{
	if ( buf != NULL && target_eid != NULL ) {
		// Create Target EID TLV.
		buf[0] = NET_TLV_TARGET_EID;
		buf[1] = 16;
		memcpy(&buf[2], target_eid, 16);
		return 18;
	}
	return 0;
}

/* --------------------------------------------------------------------------- */

void
thrd_addr_ntf_response(uip_ipaddr_t *dest_addr, uip_ipaddr_t *target_eid, uint16_t *rloc16,
		uint8_t *ml_eid, clock_time_t *last_trans_time)
{
	uint8_t payload_len = create_addr_ntf_resp_payload(addr_qry_buf, target_eid, rloc16, ml_eid, last_trans_time);

	coap_init_message(packet, COAP_TYPE_CON, COAP_POST, 0);
	coap_set_header_uri_path(packet, service_urls[1]);
	coap_set_payload(packet, addr_qry_buf, payload_len);
	coap_nonblocking_request(dest_addr, UIP_HTONS(COAP_DEFAULT_PORT), packet, thrd_addr_ntf_chunk_handler); // TODO Changing CoAP Port.
}

/* --------------------------------------------------------------------------- */

static size_t
create_addr_ntf_resp_payload(uint8_t *buf, uip_ipaddr_t *target_eid, uint16_t *rloc16,
		uint8_t *ml_eid, clock_time_t *last_trans_time)
{
	if ( buf != NULL && target_eid != NULL && rloc16 != NULL && ml_eid != NULL ) {
		// Create Target EID TLV.
		buf[0] = NET_TLV_TARGET_EID;
		buf[1] = 16;
		memcpy(&buf[2], target_eid, 16);
		// Create RLOC16 TLV.
		buf[18] = NET_TLV_RLOC16;
		buf[19] = 2;
		memcpy(&buf[20], rloc16, 2);
		buf[22] = NET_TLV_ML_EID;
		buf[23] = 8;
		memcpy(&buf[24], ml_eid, 8);
		if ( last_trans_time != NULL ) {
			buf[32] = NET_TLV_LAST_TRANSACTION_TIME;
			buf[33] = 4;
			memcpy(&buf[34], last_trans_time, 4);
			return 38;
		}
		return 32;
	}
	return 0;
}

/* --------------------------------------------------------------------------- */

void
thrd_addr_err_ntf_send(uip_ipaddr_t *dest_addr, uip_ipaddr_t *target_eid, uint8_t *ml_eid)
{
	uint8_t payload_len = create_addr_err_ntf_payload(addr_qry_buf, target_eid, ml_eid);

	coap_init_message(packet, COAP_TYPE_NON, COAP_POST, 0);
	coap_set_header_uri_path(packet, service_urls[2]);
	coap_set_payload(packet, addr_qry_buf, payload_len);
	coap_nonblocking_request(dest_addr, UIP_HTONS(COAP_DEFAULT_PORT), packet, NULL); // TODO Changing CoAP Port.
}

/* --------------------------------------------------------------------------- */

static uint8_t
create_addr_err_ntf_payload(uint8_t *buf, uip_ipaddr_t *target_eid, uint8_t *ml_eid)
{
	if ( buf != NULL && target_eid && ml_eid != NULL ) {
		// Create Target EID TLV.
		buf[0] = NET_TLV_TARGET_EID;
		buf[1] = 16;
		memcpy(&buf[2], target_eid, 16);
		buf[18] = NET_TLV_ML_EID;
		buf[19] = 8;
		memcpy(&buf[20], ml_eid, 8);
		return 28;
	}
	return 0;
}

/* --------------------------------------------------------------------------- */
/* --------------------------------- DEBUG ----------------------------------- */
/* --------------------------------------------------------------------------- */

#if RIP_DEBUG
void
thrd_local_addr_set_print(void)
{
	thrd_local_addr_t *i;
	printf(ANSI_COLOR_RED
			"|============================= LOCAL ADDRESS SET ===============================|"
			ANSI_COLOR_RESET "\n\r");
	printf("-------------------------\n");
	printf("|          EID          |\n");
	printf("-------------------------\n\r");
	for (i = thrd_local_addr_head(); i != NULL; i = thrd_local_addr_next(i)) {
		printf("| " ANSI_COLOR_YELLOW);
		PRINT6ADDR(&i->EID);
		printf(ANSI_COLOR_RESET " |\n");
	}
	printf("-------------------------\n\r");
	printf(ANSI_COLOR_RED
			"|===============================================================================|"
			ANSI_COLOR_RESET "\n\r");
}

void
thrd_rfd_child_addr_set_print()
{
	thrd_rfd_child_addr_t *i;
	printf(ANSI_COLOR_RED
			"|=========================== RFD Child Address Set =============================|"
			ANSI_COLOR_RESET "\n\r");
	printf("-------------------------\n");
	printf("|          EID          |\n");
	printf("-------------------------\n\r");
	for (i = thrd_rfd_child_addr_head(THRD_RFD_CHILD_PREFIX_LL); i != NULL; i = thrd_rfd_child_addr_next(i)) {
		printf("| " ANSI_COLOR_YELLOW);
		PRINT6ADDR(&i->childAddr);
		printf(ANSI_COLOR_RESET " |\n");
	}
	for (i = thrd_rfd_child_addr_head(THRD_RFD_CHILD_PREFIX_ML); i != NULL; i = thrd_rfd_child_addr_next(i)) {
		printf("| " ANSI_COLOR_YELLOW);
		PRINT6ADDR(&i->childAddr);
		printf(ANSI_COLOR_RESET " |\n");
	}
	for (i = thrd_rfd_child_addr_head(THRD_RFD_CHILD_PREFIX_OS); i != NULL; i = thrd_rfd_child_addr_next(i)) {
		printf("| " ANSI_COLOR_YELLOW);
		PRINT6ADDR(&i->childAddr);
		printf(ANSI_COLOR_RESET " |\n");
	}
	printf("-------------------------\n\r");
	printf(ANSI_COLOR_RED
			"|===============================================================================|"
			ANSI_COLOR_RESET "\n\r");
}

void
thrd_addr_qr_set_print()
{
	thrd_addr_qry_t *i;
	printf(ANSI_COLOR_RED
			"|============================= ADDRESS QUERY SET ===============================|"
			ANSI_COLOR_RESET "\n\r");
	printf("---------------------------------------------------------------------------------\n");
	printf("|      EID      | AQ_Timeout | AQ_Failures | AQ_Retry_Delay |\n");
	printf("---------------------------------------------------------------------------------\n\r");
	for (i = thrd_addr_qry_head(); i != NULL; i = thrd_addr_qry_next(i)) {
		printf("| ");
		PRINT6ADDR(&i->EID);
		printf( " | " ANSI_COLOR_YELLOW "%10d" ANSI_COLOR_RESET
				" | " ANSI_COLOR_YELLOW "%11d" ANSI_COLOR_RESET
				" | " ANSI_COLOR_YELLOW "%14d" ANSI_COLOR_RESET
				" |\n", i->AQ_Timeout, i->AQ_Failures, i->AQ_Retry_Delay);
	}
	printf("---------------------------------------------------------------------------------\n\r");
	printf(ANSI_COLOR_RED
			"|===============================================================================|"
			ANSI_COLOR_RESET "\n\r");
}
#endif /* RIP_DEBUG */

/*
 ********************************************************************************
 *                               END OF FILE
 ********************************************************************************
 */
