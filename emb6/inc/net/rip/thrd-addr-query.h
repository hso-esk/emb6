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

// #define THRD_AQ_TIMEOUT
// #define THRD_AQ_INITIAL_RETRY_DELAY

extern void thrd_eid_rloc_db_init(void);

/**
 * Local Address Set.
 */
typedef struct thrd_local_addr {
	struct thrd_local_addr *next;

	uip_ipaddr_t eid;
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
typedef struct thrd_addr_qr_set {
	struct thrd_addr_qr_set *next;

	uip_ipaddr_t EID;
	clock_time_t AQ_Timeout;
	uint8_t AQ_Failures;
	clock_time_t AQ_Retry_Delay;
} thrd_addr_qr_set_t;

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
/* --------------------------------- DEBUG ----------------------------------- */
/* --------------------------------------------------------------------------- */

#if RIP_DEBUG
void thrd_local_addr_set_print(void);
void thrd_rfd_child_addr_set_print();
#endif /* RIP_DEBUG */


#endif /* EMB6_INC_NET_RIP_THRD_ADDR_QUERY_H_ */

