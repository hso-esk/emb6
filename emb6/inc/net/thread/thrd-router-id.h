/**
 * \file thrd-router-id.h
 * \author Institute for reliable Embedded Systems and Communication Electronics
 * \date 2016/05/23
 * \version 1.0
 *
 * \brief Router ID management / router ID assignment
 */

#ifndef EMB6_INC_NET_RIP_THRD_ROUTER_ID_H_
#define EMB6_INC_NET_RIP_THRD_ROUTER_ID_H_

#include "emb6.h"
#include "thrd-leader-db.h"

/**
 * Address Solicit Response Status.
 */
typedef enum {
	THRD_ADDR_SOL_STATUS_SUCCESS = 0,//!< THRD_ADDR_SOL_STATUS_SUCCESS
	THRD_ADDR_SOL_STATUS_FAIL = 1,   //!< THRD_ADDR_SOL_STATUS_FAIL
} thrd_addr_sol_status_t;

/*=============================================================================
                               Router ID Management
===============================================================================*/

extern void thrd_leader_init(void);

/**
 * Assign a new Router ID.
 * @param router_id The desired Router ID (optional).
 * @return A pointer to the created ID Assignment Set entry (NULL if no free
 * Router ID is available..
 */
thrd_ldb_ida_t *thrd_leader_assign_rid(uint8_t *router_id, uint8_t *id_owner);

/**
 * Unassign a Router ID.
 * @param router_id The Router ID to be unassigned.
 */
void thrd_leader_unassign_rid(uint8_t router_id);

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
