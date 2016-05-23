/*
 * thrd-router-id.c
 *
 *  Created on: 23 May 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  Router ID Management / Router ID Assignment.
 */

#include "thread_conf.h"

#include "thrd-router-id.h"

#include "thrd-leader-db.h"

/*=============================================================================
                               Router ID Management
===============================================================================*/

/**
 * The Leader maintains an ID_Assignment_Set containing the IEEE 802.15.4
 * Extended Address of each device that has an assigned ID and, for currently
 * unassigned IDs, the time at which the ID may be reassigned.
 */

void
thrd_leader_init(void)
{
	// Starting a new Partition as the Leader.
	thrd_dev.type = THRD_DEV_TYPE_LEADER;

	thrd_ldb_ida_empty();	// Empty ID Assignment Set.
}

/*=============================================================================
                               Router ID Assignment
===============================================================================*/

/**
 * Send an Address Solicit Request message to obtain a router id from the
 * leader.
 * @param router_id The desired router id [0..62].
 */
void
thrd_request_router_id(uint8_t router_id)
{

}
