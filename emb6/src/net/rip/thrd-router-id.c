/*
 * thrd-router-id.c
 *
 *  Created on: 23 May 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  Router ID Management / Router ID Assignment.
 */

#include "emb6.h"
#include "bsp.h"
#include "thread_conf.h"
#include "ctimer.h"

#include "thrd-router-id.h"

#include "thrd-leader-db.h"

#include "thrd-partition.h"

#include "thrd-route.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"	// For debugging terminal output.

/*=============================================================================
                               Router ID Management
===============================================================================*/

static struct ctimer leader_ct;
// static struct ctimer router_ct;

static void
handle_timer(void *ptr)
{
	ID_sequence_number++;
	thrd_print_partition_data();
	ctimer_set(&leader_ct, (clock_time_t)thrd_next_period(ID_SEQUENCE_PERIOD), handle_timer, NULL);
}

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
	ctimer_set(&leader_ct, (clock_time_t)thrd_next_period(ID_SEQUENCE_PERIOD), handle_timer, NULL);
}

clock_time_t
*thrd_next_period(uint8_t sec)
{
	return (clock_time_t)(sec * bsp_get(E_BSP_GET_TRES));
}

void
thrd_leader_dealloc_rid(uint8_t router_id)
{
	thrd_rdb_id_t *rid;
	thrd_ldb_ida_t *ida;
	rid = thrd_rdb_rid_lookup(router_id);
	ida = thrd_ldb_ida_lookup(router_id);

	if ( rid != NULL && ida != NULL ) {
		// Removing the router id from the Router ID Set.
		thrd_rdb_rid_rm(rid);
		// Set the router id's reuse time.
		ida->ID_reuse_time = thrd_next_period(ID_REUSE_DELAY);
	}
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
