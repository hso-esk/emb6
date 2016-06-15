/*
 * thrd-router-id.h
 *
 *  Created on: 23 May 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  Router ID Management / Router ID Assignment.
 */

#ifndef EMB6_INC_NET_RIP_THRD_ROUTER_ID_H_
#define EMB6_INC_NET_RIP_THRD_ROUTER_ID_H_

#include "emb6.h"
#include "thrd-leader-db.h"

/*=============================================================================
                               Router ID Management
===============================================================================*/

extern void thrd_leader_init(void);

clock_time_t thrd_next_period(uint8_t sec);

/**
 * Assign a new Router ID.
 * @param router_id The desired Router ID (optional).
 * @return A pointer to the created ID Assignment Set entry (NULL if no free
 * Router ID is available..
 */
thrd_ldb_ida_t *thrd_leader_assign_rid(uint8_t *router_id, uint64_t id_owner);

/**
 * Deallocate a Router ID.
 * @param router_id The Router ID to be deallocated.
 */
void thrd_leader_dealloc_rid(uint8_t router_id);

/*=============================================================================
                               Router ID Assignment
===============================================================================*/

/**
 * Send an Address Solicit Request message to obtain a router id from the
 * leader.
 * @param leader_addr The Leader's IPv6 address.
 * @param ml_eid The ML-EID.
 * @param router_id The desired Router ID (optional).
 */
// extern void thrd_request_router_id(uip_ipaddr_t *leader_addr, uint8_t *ml_eid,
// 		uint8_t *router_id);

/**
 * Send an Address Solicit Request message to obtain a Router ID from the leader.
 * @param router_id The desired Router ID (optional).
 */
extern void thrd_request_router_id(uint8_t *router_id);

/**
 * Response handler for Address Solicit Request.
 * @param response A pointer to the response.
 */
extern void thrd_addr_solicit_chunk_handler(void *response);

/**
 * Create Assigned Router ID Mask based on the ID Assignment Set.
 * @return The Assigned Router ID Mask.
 */
uint64_t thrd_create_router_id_mask();

#endif /* EMB6_INC_NET_RIP_THRD_ROUTER_ID_H_ */
