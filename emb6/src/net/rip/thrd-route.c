/*
 * thrd-route.c
 *
 *  Created on: 18 Apr 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 *  Routing database manipulation.
 */

/*
 ********************************************************************************
 *                                   INCLUDES
 ********************************************************************************
 */

#include "emb6.h"
#include "stdlib.h"
#include "string.h"
#include "bsp.h"
#include "ctimer.h"
#include "clist.h"
#include "memb.h"
#include "rip.h"
#include "thread_conf.h"

#include "thrd-iface.h"
#include "thrd-partition.h"

#include "thrd-route.h"

// #define DEBUG DEBUG_PRINT
#include "uip-debug.h"

#define     LOGGER_ENABLE                 LOGGER_THRD_NET
#include    "logger.h"

/*
 ********************************************************************************
 *                          LOCAL FUNCTION DECLARATIONS
 ********************************************************************************
 */

static void handle_max_neighbor_age_timeout(void *ptr);

/*
 ********************************************************************************
 *                               LOCAL VARIABLES
 ********************************************************************************
 */

/* Each 'Router ID' is represented by a thrd_rdb_id_t structure and
 memory for each route is allocated from the routerid_memb memory
 block. These routes are maintained on the routerid_list. */
LIST(routerid_list);
MEMB(routerid_memb, thrd_rdb_id_t, MAX_ROUTERS); // TODO Check if MAX_ROUTERS is correct?

/* The 'Router ID Set' is represented by a thrd_rdb_router_id_t structure and
 memory is allocated from the routerid_memb memory
 block. These routes are maintained on the routerid_list. */
// MEMB(routeridset_memb, thrd_rdb_router_id_t, 1);

/* Each 'Link' is represented by a thrd_rdb_link_t structure and
 memory for each route is allocated from the link_memb memory
 block. These routes are maintained on the link_list. */
LIST(link_list);
MEMB(link_memb, thrd_rdb_link_t, MAX_ROUTERS); // TODO Check if MAX_ROUTERS is correct?

/* Each 'Route' is represented by a thrd_rdb_route_t structure and
 memory for each route is allocated from the route_memb memory
 block. These routes are maintained on the route_list. */
LIST(route_list);
MEMB(route_memb, thrd_rdb_route_t, THRD_CONF_MAX_ROUTES);

/* --------------------------------------------------------------------------- */

/* Number of currently stored router ids in the Router ID Set. */
static uint8_t num_rids = 0;
/* Number of currently stored links in the Link Set. */
static uint8_t num_links = 0;
/* Number of currently stored routes in the Route Set. */
static uint8_t num_routes = 0;

/* --------------------------------------------------------------------------- */

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

/* --------------------------------------------------------------------------- */

void
thrd_rdb_init(void)
{
	memb_init(&routerid_memb);
	list_init(routerid_list);

	// thrd_partition.ID_sequence_number = 0;

	memb_init(&link_memb);
	list_init(link_list);

	memb_init(&route_memb);
	list_init(route_list);
}

/* --------------------------------------------------------------------------- */

uint8_t
thrd_rdb_rid_num_rids(void)
{
	return num_rids;
}

/* --------------------------------------------------------------------------- */

uint8_t
thrd_rdb_link_num_links(void)
{
	return num_links;
}

/* --------------------------------------------------------------------------- */

uint8_t
thrd_rdb_route_num_routes(void)
{
	return num_routes;
}

/* --------------------------------------------------------------------------- */

thrd_rdb_id_t
*thrd_rdb_rid_head(void)
{
	return list_head(routerid_list);
}

/* --------------------------------------------------------------------------- */

thrd_rdb_link_t
*thrd_rdb_link_head(void)
{
	return list_head(link_list);
}

/* --------------------------------------------------------------------------- */

thrd_rdb_route_t
*thrd_rdb_route_head(void)
{
	return list_head(route_list);
}

/* --------------------------------------------------------------------------- */

thrd_rdb_id_t
*thrd_rdb_rid_next(thrd_rdb_id_t *r)
{
	if (r != NULL) {
		thrd_rdb_id_t *n = list_item_next(r);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

thrd_rdb_link_t
*thrd_rdb_link_next(thrd_rdb_link_t *r)
{
	if (r != NULL) {
		thrd_rdb_link_t *n = list_item_next(r);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */
/**
 * Get the next route, which directly follows the given route entry in the
 * Route Set.
 * @param r		Pointer to a route.
 * @return
 */
thrd_rdb_route_t
*thrd_rdb_route_next(thrd_rdb_route_t *r)
{
	if (r != NULL) {
		thrd_rdb_route_t *n = list_item_next(r);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */
/**
 * Get the next hop entry of a route.
 * @param route
 * @return
 */
uint8_t
*thrd_rdb_route_nexthop(thrd_rdb_route_t *route)
{
	if (route != NULL) {
		return &route->R_next_hop;
	} else {
		return NULL;
	}
}

/* --------------------------------------------------------------------------- */

/**
 * Calculate the incoming quality based on the measured link margin.
 * @param link_margin	The measured link margin in dB (shifted by
 * 						THRD_EXP_WEIGHT_MOV_AVG to avoid rounding errors).
 * @return				The corresponding incoming quality.
 */
uint8_t
thrd_rdb_link_calc_incoming_quality(uint16_t link_margin)
{
	if ( link_margin > (20 << THRD_EXP_WEIGHT_MOV_AVG) )
		return 3;
	else if ( (link_margin > (10 << THRD_EXP_WEIGHT_MOV_AVG))
			&& (link_margin <= (20 << THRD_EXP_WEIGHT_MOV_AVG)) )
		return 2;
	else if ( (link_margin > (2 << THRD_EXP_WEIGHT_MOV_AVG))
			&& (link_margin <= (10 << THRD_EXP_WEIGHT_MOV_AVG)) )
		return 1;
	else
		return 0;
}

/* --------------------------------------------------------------------------- */

/**
 * Calculate the link cost based on the given incoming quality.
 * @param incoming_quality	The incoming quality (a number between 0 and 3).
 * @return					The corresponding link cost.
 */
uint8_t
thrd_rdb_calc_link_cost(uint8_t incoming_quality)
{
	switch (incoming_quality) {
	case 3:
		return THRD_LINK_COST_1;
	case 2:
		return THRD_LINK_COST_2;
	case 1:
		return THRD_LINK_COST_6;
	default:
		return THRD_LINK_COST_INFINTE;
	}
}

/* --------------------------------------------------------------------------- */

/**
 * Determine the link margin using exponentially weighted moving average
 * calculation. Round the result using the first decimal place.
 * Formula: x(t)* = a * x(t) + (1 - a) * x(t - 1).
 * <=> (1 / a) * x(t)* = x(t) + ((1 / a) - 1) * x(t - 1).
 * @param link_margin	The measured link margin for messages received from the
 * 						neighbor.
 * @return				The calculated link margin, based on the previous value.
 */
uint16_t
thrd_rdb_link_margin_average(uint16_t old_link_margin, uint16_t new_link_margin)
{
#if (THRD_EXP_WEIGHT_MOV_AVG == EXP_WEIGHT_MOV_AVG_1_8)
	uint16_t link_margin = new_link_margin + (7 * old_link_margin);
#else
	uint16_t link_margin = new_link_margin + (15 * old_link_margin);
#endif
	link_margin /= (0x0001 << THRD_EXP_WEIGHT_MOV_AVG);
	return link_margin;
}

/* --------------------------------------------------------------------------- */

/**
 * Determine the link incoming quality using hysteresis, based on the link margin.
 * @param old_link_margin	The current link margin.
 * @param new_link_margin	The new (measured) link margin.
 * @return					The incoming quality of the link after applying
 * 							hysteresis.
 */
uint8_t
thrd_rdb_link_hysteresis(uint16_t old_link_margin, uint16_t new_link_margin)
{
	uint8_t old_iq = thrd_rdb_link_calc_incoming_quality(old_link_margin);
	uint8_t new_iq = thrd_rdb_link_calc_incoming_quality(new_link_margin);

	/* Check whether the link margin changed in such a way that the incoming
	 * quality for this link should change. To avoid frequent changes of the
	 * quality of a link, a hysteresis of 2dB is applied.
	 */
	if ( old_iq != new_iq ) {
		// Check whether the incoming quality should be increased.
		if ( old_iq < new_iq ) {
			if ( (new_iq - old_iq) > 1 ) {
				LOG_RAW("\nIncreasing incoming quality from %d to %d\n", old_iq, new_iq);
				return thrd_rdb_link_calc_incoming_quality(new_link_margin);
			}
			// Check hysteresis boundary.
			uint16_t diff_lm = new_link_margin - thrd_rdb_link_margin_get_lower_bound(new_link_margin);
			if ( diff_lm >= (2 << THRD_EXP_WEIGHT_MOV_AVG) ) {
				LOG_RAW("\nIncreasing incoming quality from %d to %d\n", old_iq, new_iq);
				return thrd_rdb_link_calc_incoming_quality(new_link_margin);
			}
		} else {
			if ( (old_iq - new_iq) > 1 ) {
				LOG_RAW("\nDecreasing incoming quality from %d to %d\n", old_iq, new_iq);
				return thrd_rdb_link_calc_incoming_quality(new_link_margin);
			}
			// The incoming quality should be decreased.
			uint16_t diff_lm = thrd_rdb_link_margin_get_lower_bound(old_link_margin) - new_link_margin;
			if ( diff_lm >= (2 << THRD_EXP_WEIGHT_MOV_AVG) ) {
				LOG_RAW("\nDecreasing incoming quality from %d to %d\n", old_iq, new_iq);
				return thrd_rdb_link_calc_incoming_quality(new_link_margin);
			}
		}
	}
	return thrd_rdb_link_calc_incoming_quality(old_link_margin);
}

/* --------------------------------------------------------------------------- */

/**
 * Get the lower bound of the link margin <--> quality conversion.
 * Example: link_margin = 15 --> return 10.
 * @param link_margin
 * @return
 */
uint16_t
thrd_rdb_link_margin_get_lower_bound(uint16_t link_margin)
{
	uint8_t quality = thrd_rdb_link_calc_incoming_quality(link_margin);
	uint16_t lower_bound = 0;

	switch (quality) {
	case 3:
		lower_bound = (20 << THRD_EXP_WEIGHT_MOV_AVG);
		break;
	case 2:
		lower_bound = (10 << THRD_EXP_WEIGHT_MOV_AVG);
		break;
	case 1:
		lower_bound = (2 << THRD_EXP_WEIGHT_MOV_AVG);
		break;
	default:
		lower_bound = 0;
		break;
	}
	return lower_bound;
}

/* --------------------------------------------------------------------------- */

thrd_rdb_id_t*
thrd_rdb_rid_lookup(uint8_t router_id)
{
	thrd_rdb_id_t *rid;
	thrd_rdb_id_t *found_rid;

	LOG_RAW("thrd_rdb_rid_lookup: Looking up Router ID Set for router id ");
	LOG_RAW("%d\n", router_id);
	LOG_RAW("\n\r");

	found_rid = NULL;
	for (rid = thrd_rdb_rid_head(); rid != NULL; rid = thrd_rdb_rid_next(rid)) {
		LOG_RAW("%d\n", rid->router_id);
		LOG_RAW("\n\r");
		if (rid->router_id == router_id) {
			found_rid = rid;
			break;
		}
	}

	if (found_rid != NULL) {
		LOG_RAW("thrd_rdb_rid_lookup: Found router id: ");
		LOG_RAW("%d\n", router_id);
		LOG_RAW("\n\r");
	} else {
		LOG_RAW("thrd_rdb_rid_lookup: No router id found\n\r");
	}

	return found_rid;
}

/* --------------------------------------------------------------------------- */

thrd_rdb_link_t*
thrd_rdb_link_lookup(uint8_t router_id)
{
	thrd_rdb_link_t *link;
	thrd_rdb_link_t *found_link;

	LOG_RAW("thrd_rdb_link_lookup: Looking up Link Set for router id ");
	LOG_RAW("%d\n", router_id);
	LOG_RAW("\n\r");

	found_link = NULL;
	for (link = thrd_rdb_link_head(); link != NULL;
			link = thrd_rdb_link_next(link)) {
		LOG_RAW("%d\n", link->L_router_id);
		LOG_RAW("\n\r");
		if (link->L_router_id == router_id) {
			found_link = link;
			break;
		}
	}

	if (found_link != NULL) {
		LOG_RAW("thrd_rdb_link_lookup: Found link for router id: ");
		LOG_RAW("%d\n", router_id);
		LOG_RAW("\n\r");
	} else {
		LOG_RAW("thrd_rdb_link_lookup: No link for router id found\n\r");
	}

	if (found_link != NULL && found_link != list_head(link_list)) {
		/* If we found the link with the router id, we put it at the start of the link_list
		 list. The list is ordered by how recently we looked them up:
		 the least recently used link will be at the end of the
		 list - for fast lookups (assuming multiple packets to the same node). */
		list_remove(link_list, found_link);
		list_push(link_list, found_link);
	}

	return found_link;
}

/* --------------------------------------------------------------------------- */

thrd_rdb_route_t*
thrd_rdb_route_lookup(uint8_t router_id)
{
	thrd_rdb_route_t *r;
	thrd_rdb_route_t *found_route;

	LOG_RAW("thrd_rdb_route_lookup: Looking up route for router id ");
	LOG_RAW("%d\n", router_id);
	LOG_RAW("\n\r");

	found_route = NULL;
	for (r = thrd_rdb_route_head(); r != NULL; r = thrd_rdb_route_next(r)) {
		// LOG_RAW("%d\n", r->R_destination);
		// LOG_RAW("\n\r");
		if (r->R_destination == router_id) {
			found_route = r;
			break;
		}
	}

	if (found_route != NULL) {
		LOG_RAW("thrd_rdb_route_lookup: Found route: ");
		LOG_RAW("%d", router_id);
		LOG_RAW(" via ");
		LOG_RAW("%d\n", thrd_rdb_route_nexthop(found_route));	// TODO
		LOG_RAW("\n\r");
	} else {
		LOG_RAW("thrd_rdb_route_lookup: No route found\n\r");
	}

	if (found_route != NULL && found_route != list_head(route_list)) {
		/* If we found a route, we put it at the start of the route_list
		 list. The list is ordered by how recently we looked them up:
		 the least recently used route will be at the end of the
		 list - for fast lookups (assuming multiple packets to the same node). */
		list_remove(route_list, found_route);
		list_push(route_list, found_route);
	}

	return found_route;
}

/* --------------------------------------------------------------------------- */

thrd_rdb_id_t*
thrd_rdb_rid_add(uint8_t router_id)
{
	thrd_rdb_id_t *rid;

	/* Find the corresponding Router ID entry (Router ID Set). */

	rid = thrd_rdb_rid_lookup(router_id);

	/* Check whether the given router id already has an entry in the Router ID Set. */
	if (rid == NULL) {
		LOG_RAW("thrd_rdb_rid_add: found unassigned router id %d\n\r", router_id);
		// LOG_RAW("%d\n", router_id);
		// LOG_RAW("\n\r");

		/* If there is no router id entry, create one. We first need to
		 * check if we have room for this router id. If not, we remove the
		 * least recently used one we have. */

		if (thrd_rdb_rid_num_rids() == MAX_ROUTER_ID) {
			/* Removing the last router id entry from the Router ID Set. */
			thrd_rdb_id_t *last;

			last = list_tail(routerid_list);
			thrd_rdb_rid_rm(last);
		}

		/* Allocate a router id entry and populate it. */
		rid = memb_alloc(&routerid_memb);

		if (rid == NULL) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			LOG_RAW("thrd_rdb_rid_add: could not allocate router id\n");
			return NULL;
		}

		rid->router_id = router_id;

		/* Sort the router ids in ascending order (this will facilitate Route64 TLV creation). */
		thrd_rdb_id_t *rid_prev;
		thrd_rdb_id_t *rid_cur;
		rid_prev = NULL;

		for ( rid_cur = thrd_rdb_rid_head(); rid_cur != NULL; rid_cur = thrd_rdb_rid_next(rid_cur) ) {
			if ( router_id < rid_cur->router_id )
				break;
			rid_prev = rid_cur;
		}

		if ( rid_prev == NULL )		// First element.
			list_push(routerid_list, rid);
		else
			list_insert(routerid_list, rid_prev, rid);

		LOG_RAW("thrd_rdb_rid_add: Added router id %d\n\r", router_id);

		num_rids++;

		LOG_RAW("thrd_rdb_rid_add: num_rids %d\n\r", num_rids);

	} else {

		LOG_RAW(
				ANSI_COLOR_RED "thrd_rdb_rid_add: router id is already known for ");
		LOG_RAW("%d\n", router_id);
		LOG_RAW(ANSI_COLOR_RESET "\n\r");

		LOG_RAW("thrd_rdb_rid_add: num_rids %d\n\r", num_rids);
		LOG_RAW("-----------------------------------------------------\n\r");

		return NULL;
	}

	LOG_RAW("-----------------------------------------------------\n\r");

	return rid;
}

/* --------------------------------------------------------------------------- */

thrd_rdb_route_t*
thrd_rdb_route_add(uint8_t destination, uint8_t next_hop, uint8_t route_cost)
{
	thrd_rdb_route_t *r;

	// Get the valid router id set.
	thrd_rdb_id_t *rid;

	LOG_RAW("thrd_rdb_route_add: search route set entry for destination id ");
	LOG_RAW("%d\n", destination);
	LOG_RAW("\n\r");

	r = thrd_rdb_route_lookup(destination);

	if (r == NULL) {

		/* If there is no routing entry, check if the given router id
		 * is valid (Router ID Set). If valid, create one. We first need to
		 * check if we have room for this route. If not, we remove the
		 * least recently used one we have. */

		/* We first check to see if the destination router is in our
		 * Router ID Set (set of valid Router IDs). */
		rid = thrd_rdb_rid_lookup(destination);

		if (rid == NULL) {
			/* If the router did not have an entry in our Router ID Set,
			 * return NULL. */

			LOG_RAW(
					ANSI_COLOR_RED "thrd_rdb_route_add: Destination router with router id ");
			LOG_RAW("%d is invalid.", destination);
			LOG_RAW(ANSI_COLOR_RESET "\n\r");

			LOG_RAW("-----------------------------------------------------\n\r");
			return NULL;
		}

		if (thrd_rdb_route_num_routes() == THRD_CONF_MAX_ROUTES) {
			/* Removing the oldest route entry from the route table. The
			 least recently used route is the first route on the set. */
			thrd_rdb_route_t *oldest;

			oldest = list_tail(route_list);
			thrd_rdb_route_rm(oldest);
		}

		/* Allocate a routing entry and populate it. */
		r = memb_alloc(&route_memb);

		if (r == NULL) {
			/* This should not happen, as we explicitly deallocated one
			 * route set entry above. */
			LOG_RAW(
					ANSI_COLOR_RED "thrd_rdb_route_add: could not allocate route with router id ");
			LOG_RAW("%d\n", destination);
			LOG_RAW(ANSI_COLOR_RESET "\n\r");
			return NULL;
		}

		/* add new routes first - assuming that there is a reason to add this
		 and that there is a packet coming soon. */
		list_push(route_list, r);

		num_routes++;

		LOG_RAW("thrd_rdb_route_add: num_routes %d\n\r", num_routes);

		r->R_destination = destination;
		r->R_next_hop = next_hop;
		r->R_route_cost = route_cost;

		LOG_RAW("uip_ds6_route_add: adding route: ");
		LOG_RAW("%d", r->R_destination);
		LOG_RAW(" via ");
		LOG_RAW("%d", r->R_next_hop);
		LOG_RAW("\n\r");

		LOG_RAW("-----------------------------------------------------\n\r");

	} else {

		LOG_RAW(ANSI_COLOR_RED "thrd_rdb_route_add: Route already known." ANSI_COLOR_RESET "\n\r");

		LOG_RAW("thrd_rdb_route_add: num_routes %d\n\r", num_routes);
		LOG_RAW("-----------------------------------------------------\n\r");

		/* Return NULL, because the Route Set already contains a route for the given destination. */
		return NULL;
	}

	return r;
}

/* --------------------------------------------------------------------------- */

thrd_error_t
thrd_rdb_rid_rm(thrd_rdb_id_t *rid)
{
	if (rid != NULL) {
		LOG_RAW("thrd_rdb_rid_rm: removing router id from 'Router ID Set' with router id: ");
		LOG_RAW("%d\n", rid->router_id);
		LOG_RAW("\n\r");

		/* Remove the router id from the Router ID Set. */
		list_remove(routerid_list, rid);
		memb_free(&routerid_memb, rid);
		num_rids--;

		LOG_RAW("thrd_rdb_rid_rm: num_rids %d\n\r", num_rids);
		return THRD_ERROR_NONE;
	}
	return THRD_ERROR_INVALID_ARGS;
}

/* --------------------------------------------------------------------------- */

void
thrd_rdb_rid_empty()
{
	thrd_rdb_id_t *rid;
	thrd_rdb_id_t *rid_nxt;
	LOG_RAW("thrd_rdb_rid_empty: removing all (%d) router ids from 'Router ID Set'.", num_rids);
	LOG_RAW("\n\r");
	rid = thrd_rdb_rid_head();
	rid_nxt = rid;
	while ( rid_nxt != NULL ) {
		rid = rid_nxt;
		rid_nxt = rid->next;
		thrd_rdb_rid_rm(rid);
	}
}

/* --------------------------------------------------------------------------- */

thrd_error_t
thrd_rdb_link_rm(thrd_rdb_link_t *link)
{
	if ( link != NULL ) {
		LOG_RAW("thrd_rdb_link_rm: removing link with router id: ");
		LOG_RAW("%d\n", link->L_router_id);
		LOG_RAW("\n\r");

		/* Remove the link from the Link Set. */
		list_remove(link_list, link);
		memb_free(&link_memb, link);
		num_links--;

		LOG_RAW("thrd_rdb_link_rm: num_links %d\n\r", num_links);
		return THRD_ERROR_NONE;
	}
	return THRD_ERROR_INVALID_ARGS;
}

/* --------------------------------------------------------------------------- */

thrd_error_t
thrd_rdb_route_rm(thrd_rdb_route_t *route)
{
	if (route != NULL) {

		LOG_RAW("thrd_rdb_route_rm: removing route with router id: ");
		LOG_RAW("%d\n", route->R_destination);
		LOG_RAW("\n\r");

		/* Remove the route from the Route Set. */
		list_remove(route_list, route);
		memb_free(&route_memb, route);
		num_routes--;

		LOG_RAW("thrd_rdb_route_rm: num_routes %d\n\r", num_routes);
		return THRD_ERROR_NONE;
	}
	return THRD_ERROR_INVALID_ARGS;
}

/*
 ********************************************************************************
 *                                 API FUNCTIONS
 ********************************************************************************
 */

/**
 * Update the Link Set.
 * @param router_id			The router id.
 * @param link_margin		The measured link margin for messages received from
 * 							the neighbor.
 * @param outgoing_quality	The incoming link quality metric reported by the
 * 							neighbor for messages arriving from this Router.
 * @param age				The elapsed time since an advertisement was received
 * 							from the neighbor.
 * @return					The new/updated link entry.
 */
thrd_rdb_link_t*
thrd_rdb_link_update(uint8_t router_id, uint8_t link_margin,
		uint8_t outgoing_quality, clock_time_t age)
{
	thrd_rdb_link_t *l;
	thrd_rdb_id_t *rid;
	uint16_t link_margin_shifted = (link_margin << THRD_EXP_WEIGHT_MOV_AVG);

	/* Find the corresponding Router ID entry (Router ID Set). */
	l = thrd_rdb_link_lookup(router_id);

	/* Check whether the given router id already has an entry in the Link Set. */
	if ( l == NULL ) {
		LOG_RAW("thrd_rdb_link_update: router id %d unknown\n\r", router_id);
		// LOG_RAW("%d\n", router_id);
		// LOG_RAW("\n\r");

		/* If there is no link entry, check if the given router id
		 * is valid (Router ID Set). If valid, create one. We first need to
		 * check if we have room for this link. If not, we remove the
		 * least recently used one we have. */

		/* We first check to see if the destination router is in our
		 * Router ID Set (set of valid Router IDs). */
		rid = thrd_rdb_rid_lookup(router_id);

		if (rid == NULL) {
			/* If the router did not have an entry in our Router ID Set,
			 * return NULL. */

			LOG_RAW(ANSI_COLOR_RED "thrd_rdb_link_update: Router with router id ");
			LOG_RAW("%d is invalid.", router_id);
			LOG_RAW(ANSI_COLOR_RESET "\n\r");

			LOG_RAW("-----------------------------------------------------\n\r");
			return NULL;
		}

		/* If there is no link entry, create one. We first need to
		 * check if we have room for this link. If not, we remove the
		 * least recently used one we have. */
		if ( thrd_rdb_link_num_links() == MAX_ROUTERS ) {
			/* Removing the oldest link entry from the Link Set. The
			 * least recently used link is the first link on the set. */
			thrd_rdb_link_t *oldest;

			oldest = list_tail(link_list);
			thrd_rdb_link_rm(oldest);
		}

		/* Allocate a link entry and populate it. */
		l = memb_alloc(&link_memb);

		if (l == NULL) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			LOG_RAW("thrd_rdb_link_update: could not allocate link\n");
			return NULL;
		}

		l->L_router_id = router_id;
		l->L_link_margin = link_margin_shifted;
		l->L_incoming_quality = thrd_rdb_link_calc_incoming_quality(link_margin_shifted);
		l->L_outgoing_quality = outgoing_quality;
		ctimer_set(&l->L_age, MAX_NEIGHBOR_AGE * bsp_get(E_BSP_GET_TRES), handle_max_neighbor_age_timeout, l);

		/* Add new link first - assuming that there is a reason to add this
		 * and that there is a packet coming soon. */
		list_push(link_list, l);

		num_links++;

		LOG_RAW("thrd_rdb_link_update: num_links %d\n\r", num_links);

	} else {

		LOG_RAW(ANSI_COLOR_RED "thrd_rdb_link_update: router id is already known for ");
		LOG_RAW("%d\n", router_id);
		LOG_RAW(ANSI_COLOR_RESET "\n\r");

		LOG_RAW("thrd_rdb_link_update: num_links %d\n\r", num_links);
		LOG_RAW("-----------------------------------------------------\n\r");

		/* Calculate the new link margin using exponential weighted moving
		 * averaging. */
		uint8_t new_lm = thrd_rdb_link_margin_average(l->L_link_margin, link_margin_shifted);

		/* Check whether the incoming quality has changed. */
		uint8_t new_iq = thrd_rdb_link_hysteresis(l->L_link_margin, new_lm);

		l->L_link_margin = new_lm;
		l->L_outgoing_quality = outgoing_quality;
		ctimer_reset(&l->L_age);

		if ( l->L_incoming_quality != new_iq ) {
			l->L_incoming_quality = new_iq;
			return l;
		}
		// return NULL;
	}
	LOG_RAW("-----------------------------------------------------\n\r");
	return l;
}

/*---------------------------------------------------------------------------*/

bool
thrd_rdb_is_neighbor_router(uint8_t router_id)
{
	if ( thrd_rdb_link_lookup(router_id) != NULL ) {
		return TRUE;
	} else {
		return FALSE;
	}
}

/*---------------------------------------------------------------------------*/
/*
 * Called after max_age_timer of a Link Set Entry has expired.
 */
static void
handle_max_neighbor_age_timeout(void *ptr)
{
	thrd_rdb_link_t *l = (thrd_rdb_link_t*) ptr;

	LOG_RAW("Link Set: Timer expired");
	if ( l != NULL ) {
		LOG_RAW(" for Link Set Entry with Router ID = &d\n\r", l->L_router_id);
		thrd_rdb_link_rm(l);
	}
	return;
}

/* --------------------------------------------------------------------------- */

/**
 * Update the Route Set. This function is part of 'Processing Route TLVs'.
 * @param router_id		The router id of the sender (of the advertisement).
 * @param destination	The router id of the destination router.
 * @param cost_reported	The routing cost reported by the sender.
 * @return				The new/updated route entry.
 * 						NULL, else (if nothing has changed, or an error occurred).
 */
thrd_rdb_route_t*
thrd_rdb_route_update(uint8_t router_id, uint8_t destination, uint8_t cost_reported)
{
	thrd_rdb_route_t *r;
	thrd_rdb_link_t *l;
	uint8_t link_cost = 0;		// The current link cost to the sender.
	uint8_t nxt_link_cost = 0;	// The current next hop's link cost of the route.

	/* Check whether the route already exists in the current Route Set. */
	r = thrd_rdb_route_lookup(destination);

	if ( cost_reported > 0 ) {

		if ( r == NULL ) {
			/* The route doesn't exist so far. */
			LOG_RAW("thrd_rdb_route_update: Received new route from advertisement "
					"(sender rid %d) ", router_id);
			LOG_RAW("to destination router id %d", destination);
			LOG_RAW("\n\r");

			/* Check whether the destination router id is a neighbor (Link Set).
			 * If it is a neighbor, check whether the new (multihop) route is
			 * better than the (link-local) route.
			 */
			l = thrd_rdb_link_lookup(destination);

			if ( l != NULL ) {
				link_cost = thrd_rdb_calc_link_cost(thrd_rdb_link_lookup(router_id)->L_incoming_quality);

				/* If the destination router id is a neighbor and the new route is not
				 * better, return.
				 */
				if ( (link_cost + cost_reported) >= thrd_rdb_calc_link_cost(l->L_incoming_quality) ) {
					LOG_RAW("thrd_rdb_route_update: Not a better route to destination router id %d", destination);
					LOG_RAW("\n\r");

					return NULL;
				}
			}

			LOG_RAW("thrd_rdb_route_update: Adding new route from advertisement "
					"(sender rid %d) ", router_id);
			LOG_RAW("to destination router id %d", destination);
			LOG_RAW("\n\r");
			return thrd_rdb_route_add(destination, router_id,
					((cost_reported < MAX_ROUTE_COST) ? cost_reported : MAX_ROUTE_COST));

		} else {
			/* The route already exists. */
			LOG_RAW("thrd_rdb_route_update: Received existing route from advertisement "
					"(sender rid %d) ", router_id);
			LOG_RAW("to destination router id %d", destination);
			LOG_RAW("\n\r");

			l = thrd_rdb_link_lookup(router_id);

			/* This should not happen unless there is an inconsistency. */
			if ( l == NULL ) {
				LOG_RAW(ANSI_COLOR_RED "thrd_rdb_route_update: ERROR! Inconsistency found!" ANSI_COLOR_RESET);
				return NULL;
			}
			link_cost = thrd_rdb_calc_link_cost(l->L_incoming_quality);
			nxt_link_cost = thrd_rdb_calc_link_cost(thrd_rdb_link_lookup(r->R_next_hop)->L_incoming_quality);

			/* Check whether the new route is better. */
			if ( (link_cost + cost_reported) < (nxt_link_cost + r->R_route_cost) ) {

				LOG_RAW("thrd_rdb_route_update: Found a better route to destination router id: ");
				LOG_RAW("%d ", destination);
				LOG_RAW("over router with id %d", router_id);
				LOG_RAW("\n\r");

				r->R_next_hop = router_id;
				r->R_route_cost = cost_reported;

				return r;
			}
		}
	} else {
		/* Remove route. */
		if ( r != NULL ) {
			if ( router_id == r->R_next_hop ) {
				LOG_RAW("thrd_rdb_route_update: Removing route with destination router id: ");
				LOG_RAW("%d", destination);
				LOG_RAW("\n\r");
				thrd_rdb_route_rm(r);
			}
		}
	}
	return NULL;
}

/*
 ********************************************************************************
 *                                DEBUG FUNCTIONS
 ********************************************************************************
 */

#if RIP_DEBUG

void
thrd_rdb_print_rid_set(void)
{
	thrd_rdb_id_t *rid;

	LOG_RAW("ROUTER ID SET\n");
	LOG_RAW("| ROUTER ID |\n");
	LOG_RAW("-------------\n\r");
	for (rid = thrd_rdb_rid_head(); rid != NULL; rid = thrd_rdb_rid_next(rid)) {
		LOG_RAW("| "  "%9d"  " |\n",
				rid->router_id);
	}
	LOG_RAW("-------------\n\r");
}

/* --------------------------------------------------------------------------- */

void
thrd_rdb_print_link_set(void)
{
	thrd_rdb_link_t *l;

	LOG_RAW(
			"---------------------------------- LINK SET -------------------------------------\n");
	LOG_RAW(
			"| L_router_id | L_link_margin | L_incoming_quality | L_outgoing_quality | L_age |\n");
	LOG_RAW(
			"---------------------------------------------------------------------------------\n\r");
	for (l = thrd_rdb_link_head(); l != NULL; l = thrd_rdb_link_next(l)) {
		LOG_RAW(  "| "  "%11d"
				" | "  "%13d"
				" | "  "%18d"
				" | "  "%18d"
				" | "  "%5d"
				" |\n", l->L_router_id, l->L_link_margin, l->L_incoming_quality,
				l->L_outgoing_quality, l->L_age);
	}
	LOG_RAW(
			"---------------------------------------------------------------------------------\n\r");
}

/* --------------------------------------------------------------------------- */

void
thrd_rdb_print_route_set(void)
{
	thrd_rdb_route_t *r;

	LOG_RAW("----------------- ROUTE SET -----------------\n");
	LOG_RAW("| R_destination | R_next_hop | R_route_cost |\n");
	LOG_RAW("---------------------------------------------\n\r");
	for (r = thrd_rdb_route_head(); r != NULL; r = thrd_rdb_route_next(r)) {
		LOG_RAW(  "| "  "%13d"
				" | " "%10d"
				" | " "%12d"
				" |\n", r->R_destination, r->R_next_hop, r->R_route_cost);
	}
	LOG_RAW("---------------------------------------------\n\r");
}

/* --------------------------------------------------------------------------- */

void
thrd_rdb_print_routing_database(void)
{
	LOG_RAW("|============================== ROUTING DATABASE ===============================|\n\r");
	thrd_rdb_print_rid_set();
	thrd_rdb_print_link_set();
	thrd_rdb_print_route_set();
	LOG_RAW("|===============================================================================|\n\r");
}

#endif /* RIP_DEBUG */

/*
 ********************************************************************************
 *                               END OF FILE
 ********************************************************************************
 */
