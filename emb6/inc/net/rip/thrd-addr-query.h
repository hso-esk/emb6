/*
 * thrd-addr-query.h
 *
 *  Created on: 6 Jun 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  Thread Address Query.
 */

#ifndef EMB6_INC_NET_RIP_THRD_ADDR_QUERY_H_
#define EMB6_INC_NET_RIP_THRD_ADDR_QUERY_H_

#include "emb6.h"
#include "clist.h"
#include "uip.h"
#include "net_tlv.h"

// #define THRD_AQ_TIMEOUT
// #define THRD_AQ_INITIAL_RETRY_DELAY

/* Number of URIs that can be queried. */
#define NUMBER_OF_URLS 2

extern void thrd_eid_rloc_db_init(void);

void thrd_eid_rloc_coap_init();

/**
 * Local Address Set.
 */
typedef struct thrd_local_addr {
	struct thrd_local_addr *next;

	uip_ipaddr_t EID;
} thrd_local_addr_t;

/**
 * AddressSet - A set of IPv6 addresses assigned to the RFD Child's
 * Thread interface.
 */
typedef struct thrd_rfd_addr {
	struct thrd_rfd_addr *next;

	uip_ipaddr_t childAddr;
} thrd_rfd_addr_t;

/**
 * RFD Child Address Set.
 */
typedef struct thrd_rfd_child_set {
	thrd_rfd_addr_t *AddressSet;
} thrd_rfd_child_set_t;

/**
 * Address Query Set.
 */
typedef struct thrd_addr_qr {
	struct thrd_addr_qr *next;

	uip_ipaddr_t EID;
	clock_time_t AQ_Timeout;
	uint8_t AQ_Failures;
	clock_time_t AQ_Retry_Delay;
} thrd_addr_qr_t;

/* --------------------------------------------------------------------------- */
/* --------------------------- Local Address Set ----------------------------- */
/* --------------------------------------------------------------------------- */

/**
 * Get the number of the currently stored EIDs in the Local Address Set.
 * @return The number of currently stored EIDs.
 */
uint8_t thrd_local_addr_num();

/**
 * Get a pointer to the first element of the Local Address Set.
 * @return A pointer to the first element.
 */
thrd_local_addr_t *thrd_local_addr_head(void);

/**
 * Get the subsequent element of the given entry in the Local Address Set.
 * @param i A thrd_local_addr_t element.
 * @return A pointer to the subsequent element.
 */
thrd_local_addr_t *thrd_local_addr_next(thrd_local_addr_t *i);

/**
 * Look up a given EID in the Local Address Set.
 * @param eid An Endpoint Identifier.
 * @return The corresponding thrd_local_addr_t entry (if exist).
 */
thrd_local_addr_t *thrd_local_addr_lookup(uip_ipaddr_t eid);

/**
 * Add a given EID to the Local Address Set.
 * @param eid An Endpoint Identifier.
 * @return A pointer to the currently inserted thrd_local_addr_t entry.
 */
thrd_local_addr_t *thrd_local_addr_add(uip_ipaddr_t eid);

/**
 * Remove a given EID element of the Local Address Set.
 * @param localAddr An EID element.
 */
void thrd_local_addr_rm(thrd_local_addr_t *localAddr);

/**
 * Empty Local Address Set.
 */
void thrd_local_addr_empty();

/* --------------------------------------------------------------------------- */
/* ------------------------- RFD Child Address Set --------------------------- */
/* --------------------------------------------------------------------------- */



uint8_t thrd_rfd_child_addr_num();

thrd_rfd_addr_t *thrd_rfd_child_addr_head(void);

thrd_rfd_addr_t *thrd_rfd_child_addr_next(thrd_rfd_addr_t *i);

thrd_rfd_addr_t *thrd_rfd_child_addr_lookup(uip_ipaddr_t child_addr);

thrd_rfd_addr_t *thrd_rfd_child_addr_add(uip_ipaddr_t child_addr);

void thrd_rfd_child_addr_rm(thrd_rfd_addr_t *child_addr);

void thrd_rfd_child_addr_empty();

/* --------------------------------------------------------------------------- */
/* --------------------------- Address Query Set ----------------------------- */
/* --------------------------------------------------------------------------- */

uint8_t thrd_addr_qr_num();

thrd_addr_qr_t *thrd_addr_qr_head(void);

thrd_addr_qr_t *thrd_addr_qr_next(thrd_addr_qr_t *i);

thrd_addr_qr_t *thrd_addr_qr_lookup(uip_ipaddr_t eid);

thrd_addr_qr_t *thrd_addr_qr_add(uip_ipaddr_t eid, clock_time_t timeout, clock_time_t retry_delay);

void thrd_addr_qr_rm(thrd_addr_qr_t *addrQr);

void thrd_addr_qr_empty();

/* --------------------------------------------------------------------------- */
/* ----------------------------- Address Query ------------------------------- */
/* --------------------------------------------------------------------------- */

void thrd_addr_qry_request(uip_ipaddr_t *target_eid);

/**
 * Send an Address Notification Response message.
 * @param target_eid The Target EID.
 * @param rloc16 The corresponding RLOC16.
 * @param ml_eid_tlv The corresonding ML-EID of the target.
 * @param last_trans_time The Last Transaction Time (Optional). NULL if not used.
 */
void thrd_addr_ntf_response(uip_ipaddr_t *target_eid, uint16_t *rloc16,
		uint8_t *ml_eid_tlv, clock_time_t *last_trans_time);

/*
void thrd_addr_ntf_response(net_tlv_target_eid_t *target_tlv,
		net_tlv_rloc16_t *rloc16_tlv,
		net_tlv_ml_eid_t *ml_eid_tlv);
*/

/* --------------------------------------------------------------------------- */
/* --------------------------------- DEBUG ----------------------------------- */
/* --------------------------------------------------------------------------- */

#if RIP_DEBUG
void thrd_local_addr_set_print(void);
void thrd_rfd_child_addr_set_print();
void thrd_addr_qr_set_print();
#endif /* RIP_DEBUG */


#endif /* EMB6_INC_NET_RIP_THRD_ADDR_QUERY_H_ */

