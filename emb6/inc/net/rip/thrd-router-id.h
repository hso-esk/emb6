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

/* Number of URIs that can be queried. */
#define NUMBER_OF_URLS 2

/*=============================================================================
                               Router ID Management
===============================================================================*/

extern void thrd_leader_init(void);

clock_time_t thrd_next_period(uint8_t sec);

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
extern void thrd_request_router_id(uip_ipaddr_t *leader_addr, uint8_t *ml_eid,
		uint8_t *router_id);

/**
 * Response handler for Address Solicit Request.
 * @param response A pointer to the response.
 */
extern void thrd_addr_solicit_chunk_handler(void *response);

#endif /* EMB6_INC_NET_RIP_THRD_ROUTER_ID_H_ */
