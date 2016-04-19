/*
 * thrd-route.c
 *
 *  Created on: 18 Apr 2016
 *      Author: osboxes
 */

#include "emb6.h"
#include "uip.h"
#include "stdlib.h"
#include "string.h"
#include "clist.h"
#include "memb.h"
#include "rip.h"

#include "thrd-route.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"

/* --------------------------------------------------------------------------- */

/* Each 'Router ID' is represented by a thrd_rdb_id_t structure and
   memory for each route is allocated from the routerid_memb memory
   block. These routes are maintained on the routerid_list. */
LIST(routerid_list);
MEMB(routerid_memb, thrd_rdb_id_t, MAX_ROUTERS);		// TODO Check whether MAX_ROUTERS is correct?

/* The 'Router ID Set' is represented by a thrd_rdb_router_id_t structure and
   memory is allocated from the routerid_memb memory
   block. These routes are maintained on the routerid_list. */
MEMB(routeridset_memb, thrd_rdb_router_id_t, 1);

/* Each 'Link' is represented by a thrd_rdb_link_t structure and
   memory for each route is allocated from the link_memb memory
   block. These routes are maintained on the link_list. */
LIST(link_list);
MEMB(link_memb, thrd_rdb_link_t, MAX_ROUTERS);				// TODO Check whether MAX_ROUTERS is correct?

/* Each 'Route' is represented by a thrd_rdb_route_t structure and
   memory for each route is allocated from the route_memb memory
   block. These routes are maintained on the route_list. */
LIST(route_list);
MEMB(route_memb, thrd_rdb_route_t, THRD_CONF_MAX_ROUTES);

/* --------------------------------------------------------------------------- */

static int num_routes = 0;

/* --------------------------------------------------------------------------- */
void
thrd_rdb_init(void)
{
	memb_init(&routerid_memb);
	list_init(routerid_list);

	memb_init(&routeridset_memb);

	memb_init(&link_memb);
	list_init(link_list);

	memb_init(&route_memb);
	list_init(route_list);
}

/* --------------------------------------------------------------------------- */

thrd_rdb_route_t
*thrd_rdb_route_head(void)
{
	return list_head(route_list);
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

thrd_rdb_id_t
*thrd_rdb_rid_next(thrd_rdb_id_t *r)
{
	if(r != NULL) {
		thrd_rdb_id_t *n = list_item_next(r);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

thrd_rdb_link_t
*thrd_rdb_link_next(thrd_rdb_link_t *r)
{
	if(r != NULL) {
		thrd_rdb_link_t *n = list_item_next(r);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

thrd_rdb_route_t
*thrd_rdb_route_next(thrd_rdb_route_t *r)
{
	if(r != NULL) {
		thrd_rdb_route_t *n = list_item_next(r);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

uint8_t
*thrd_rdb_route_nexthop(thrd_rdb_route_t *route)
{
	if ( route != NULL ) {
		// TODO
		// return uip_ds6_nbr_ipaddr_from_lladdr(uip_ds6_route_nexthop_lladdr(route));

		// PRINTF("6.2\n");

		return (uint8_t*) route->R_next_hop;	// LZ.
	} else {
		return NULL;
	}
}

/* --------------------------------------------------------------------------- */

int
thrd_rdb_route_num_routes(void)
{
	return num_routes;
}

/* --------------------------------------------------------------------------- */

thrd_rdb_id_t
*thrd_rdb_rid_lookup(uint8_t destination)
{
	thrd_rdb_id_t *rid;
	thrd_rdb_id_t *found_rid;

	PRINTF("thrd_rdb_rid_lookup: Looking up Router ID Set for router id ");
	PRINTF("%d\n", destination);
	PRINTF("\n\r");

	found_rid = NULL;
	for ( rid = thrd_rdb_rid_head();
			rid != NULL;
			rid = thrd_rdb_rid_next(rid) ) {
		PRINTF("%d\n", rid->router_id);
		PRINTF("\n\r");
		if ( rid->router_id == destination ) {
			found_rid = rid;
			break;
		}
	}

	if ( found_rid != NULL ) {
		PRINTF("thrd_rdb_rid_lookup: Found router id: ");
		PRINTF("%d\n", destination);
		PRINTF("\n\r");
	} else {
		PRINTF("thrd_rdb_rid_lookup: No router id found\n\r");
	}

	if ( found_rid != NULL && found_rid != list_head(routerid_list) ) {
		/* If we found the router id, we put it at the start of the routerid_list
		         list. The list is ordered by how recently we looked them up:
		         the least recently used router id will be at the end of the
		         list - for fast lookups (assuming multiple packets to the same node). */
		list_remove(routerid_list, found_rid);
		list_push(routerid_list, found_rid);
	}

	return found_rid;
}

/* --------------------------------------------------------------------------- */

thrd_rdb_link_t
*thrd_rdb_link_lookup(uint8_t destination)
{
	thrd_rdb_link_t *link;
	thrd_rdb_link_t *found_link;

	PRINTF("thrd_rdb_link_lookup: Looking up Link Set for router id ");
	PRINTF("%d\n", destination);
	PRINTF("\n\r");

	found_link = NULL;
	for ( link = thrd_rdb_link_head();
			link != NULL;
			link = thrd_rdb_link_next(link) ) {
		PRINTF("%d\n", link->L_router_id);
		PRINTF("\n\r");
		if ( link->L_router_id == destination ) {
			found_link = link;
			break;
		}
	}

	if ( found_link != NULL ) {
		PRINTF("thrd_rdb_link_lookup: Found link for router id: ");
		PRINTF("%d\n", destination);
		PRINTF("\n\r");
	} else {
		PRINTF("thrd_rdb_link_lookup: No link for router id found\n\r");
	}

	if ( found_link != NULL && found_link != list_head(link_list) ) {
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

thrd_rdb_route_t
*thrd_rdb_route_lookup(uint8_t destination)
{
	thrd_rdb_route_t *r;
	thrd_rdb_route_t *found_route;

	PRINTF("thrd_rdb_route_lookup: Looking up route for router id ");
	PRINTF("%d\n", destination);
	PRINTF("\n\r");

	found_route = NULL;
	for ( r = thrd_rdb_route_head();
			r != NULL;
			r = thrd_rdb_route_next(r) ) {
		// PRINTF("%d\n", r->R_destination);
		// PRINTF("\n\r");
		if ( r->R_destination == destination ) {
			found_route = r;
			break;
		}
	}

	if ( found_route != NULL ) {
		PRINTF("thrd_rdb_route_lookup: Found route: ");
		PRINTF("%d", destination);
		PRINTF(" via ");
		PRINTF("%d\n", thrd_rdb_route_nexthop(found_route));		// TODO
		PRINTF("\n\r");
	} else {
		PRINTF("thrd_rdb_route_lookup: No route found\n\r");
	}

	if ( found_route != NULL && found_route != list_head(route_list) ) {
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

thrd_rdb_route_t
*thrd_rdb_route_add(uint8_t destination, uint8_t next_hop, uint8_t route_cost)
{
	thrd_rdb_route_t *r;

	// Get the valid router id set.
	thrd_rdb_id_t *router_id_set;
	// Get the neighbor Link Set.
	thrd_rdb_link_t *neighbor_link_set;

	/* Get next hop, make sure it is in neighbor table (Link Set). */

	/* Find the corresponding Router ID entry (Router ID Set). */
	for ( router_id_set = list_head( routerid_list );
			(router_id_set != NULL) && (router_id_set->router_id != destination);
			router_id_set = list_item_next( router_id_set ));

	/* Find the corresponding neighbor link entry (Link Set). */
	for ( neighbor_link_set = list_head( link_list );
			(neighbor_link_set != NULL) && (neighbor_link_set->L_router_id != destination);
			neighbor_link_set = list_item_next( neighbor_link_set ));

	if ( router_id_set == NULL ) {
		PRINTF("thrd_rdb_route_add: router id unknown for ");
		PRINTF("%d\n", destination);
		PRINTF("\n\r");
		// return NULL;
	}

	if ( router_id_set == NULL ) {
		PRINTF("thrd_rdb_route_add: neighbor link unknown for ");
		PRINTF("%d\n", destination);
		PRINTF("\n\r");
	}

	PRINTF("thrd_rdb_route_add: search neighbor router destination id ");
	PRINTF("%d\n", destination);
	PRINTF("\n\r");

	r = thrd_rdb_route_lookup(destination);

	if ( (r != NULL) && (!(next_hop == (uint8_t*) thrd_rdb_route_nexthop(r)) || route_cost < r->R_route_cost) ) {
		// PRINTF("6.5\n");

		if ( route_cost < r->R_route_cost ) {
			/* The new route is better, remove the current one. */
			PRINTF(ANSI_COLOR_RED "thrd_rdb_route_add: Found better route (metric). Removing old route." ANSI_COLOR_RESET "\n\r");
		}

		thrd_rdb_route_rm(r);
	}

	/* First make sure that we don't add a route twice. If we find an
	     existing route for our destination, we'll delete the old
	     one first. */
	r = thrd_rdb_route_lookup(destination);

	if ( r != NULL ) {
		uint8_t current_nexthop;
		current_nexthop = (uint8_t*) thrd_rdb_route_nexthop(r);

		if ( next_hop == current_nexthop) {
			/* At least check whether the metric has changed. */
			if ( route_cost < r->R_route_cost ) {
				/* The new route is better, remove the current one. */
				PRINTF("thrd_rdb_route_add: Found better route (metric). Removing old route.\n\r");
				thrd_rdb_route_rm(r);
			} else {
				/* No changes. The route already exists. */
				PRINTF("thrd_rdb_route_add: Route is already known. Return without any changes.\n\r");
				PRINTF("-----------------------------------------------------\n\r");
				return r;
			}
		}
	}
	{
		/* If there is no routing entry, create one. We first need to
          check if we have room for this route. If not, we remove the
          least recently used one we have. */

		if ( thrd_rdb_route_num_routes() == THRD_CONF_MAX_ROUTES ) {
			/* Removing the oldest route entry from the route table. The
             least recently used route is the first route on the list. */
			thrd_rdb_route_t *oldest;

			oldest = list_tail(route_list);
			thrd_rdb_route_rm(oldest);
		}

		/* We first check to see if we already have this router in our
		 * router_id_set.
		 */
		if ( router_id_set == NULL) {
			/* If the router did not have an entry in our router_id_set,
			 * we create one.
			 */

			router_id_set = memb_alloc(&routerid_memb);

			router_id_set->router_id = destination;

			/* add new router id first - assuming that there is a reason to add this
	           and that there is a packet coming soon. */
			list_push(routerid_list, router_id_set);
		}

		if ( neighbor_link_set == NULL ) {
			// TODO Only add neighbor link set, if the router with the given router id is a neighbor.

			neighbor_link_set = memb_alloc(&link_memb);

			neighbor_link_set->L_router_id = destination;

			/* add new link first - assuming that there is a reason to add this
	           and that there is a packet coming soon. */
			list_push(link_list, neighbor_link_set);
		}

		/* Allocate a routing entry and populate it. */
		r = memb_alloc(&route_memb);

		if ( r == NULL ) {
			/* This should not happen, as we explicitly deallocated one
			 * route set entry above. */
			PRINTF("thrd_rdb_route_add: could not allocate route\n");
			return NULL;
		}

		/* add new routes first - assuming that there is a reason to add this
           and that there is a packet coming soon. */
		list_push(route_list, r);

		num_routes++;

		PRINTF("thrd_rdb_route_add num %d\n\r", num_routes);
	}

	r->R_destination = destination;
	r->R_next_hop = next_hop;
	r->R_route_cost = route_cost;

	PRINTF("uip_ds6_route_add: adding route: ");
	PRINTF("%d", r->R_destination);
	PRINTF(" via ");
	PRINTF("%d", r->R_next_hop);
	PRINTF("\n\r");
	// ANNOTATE("#L %u 1;blue\n\r", nexthop->u8[sizeof(uip_ipaddr_t) - 1]);

	PRINTF("-----------------------------------------------------\n\r");

	return r;
}

void
thrd_rdb_route_rm(thrd_rdb_route_t *route)
{
	// Get the valid router id set.
	thrd_rdb_id_t *router_id_set;
	// Get the neighbor Link Set.
	thrd_rdb_link_t *neighbor_link_set;


	/*
	struct uip_ds6_route_neighbor_route *neighbor_route;
#if DEBUG != DEBUG_NONE
	assert_nbr_routes_list_sane();
#endif*/ /* DEBUG != DEBUG_NONE */

	if ( route != NULL ) {

		PRINTF("thrd_rdb_route_rm: removing route with router id: ");
		PRINTF("%d\n", route->R_destination);
		PRINTF("\n\r");

		/* Remove the route from the route list */
		list_remove(route_list, route);

		/* Find the corresponding Router ID entry and remove it from the Router ID Set. */
		for ( router_id_set = list_head( routerid_list );
				router_id_set != NULL && router_id_set->router_id != route->R_destination;
				router_id_set = list_item_next( router_id_set ));

		/* Find the corresponding neighbor link entry and remove it from the Link Set. */
		for ( neighbor_link_set = list_head( link_list );
				neighbor_link_set != NULL && neighbor_link_set->L_router_id != route->R_destination;
				neighbor_link_set = list_item_next( neighbor_link_set ));

		if ( router_id_set == NULL ) {
			PRINTF("thrd_rdb_route_rm: router_id_set was NULL for ");
			PRINTF("%d\n", route->R_destination);	// vorher: &route->...
			PRINTF("\n");
		}

		if ( neighbor_link_set == NULL ) {
			PRINTF("thrd_rdb_route_rm: neighbor_link_set was NULL for ");
			PRINTF("%d\n", route->R_destination);	// vorher: &route->...
			PRINTF("\n");
		}

		list_remove(routerid_list, router_id_set);
		list_remove(link_list, neighbor_link_set);

		memb_free(&routerid_memb, router_id_set);
		memb_free(&link_memb, neighbor_link_set);
		memb_free(&route_memb, route);

		num_routes--;

		PRINTF("thrd_rdb_route_rm num %d\n\r", num_routes);

		/*
#if UIP_DS6_NOTIFICATIONS
		call_route_callback(UIP_DS6_NOTIFICATION_ROUTE_RM,
				&route->ipaddr, uip_ds6_route_nexthop(route));
#endif
#if 0 //(DEBUG & DEBUG_ANNOTATE) == DEBUG_ANNOTATE
		// we need to check if this was the last route towards "nexthop"
		// if so - remove that link (annotation)
		uip_ds6_route_t *r;
		for(r = uip_ds6_route_head();
				r != NULL;
				r = uip_ds6_route_next(r)) {
			uip_ipaddr_t *nextr, *nextroute;
			nextr = uip_ds6_route_nexthop(r);
			nextroute = uip_ds6_route_nexthop(route);
			if(nextr != NULL &&
					nextroute != NULL &&
					uip_ipaddr_cmp(nextr, nextroute)) {
				// we found another link using the specific nexthop, so keep the #L
				return;
			}
		}
		ANNOTATE("#L %u 0\n\r", uip_ds6_route_nexthop(route)->u8[sizeof(uip_ipaddr_t) - 1]);
#endif*/
	}
	/*
#if DEBUG != DEBUG_NONE
	assert_nbr_routes_list_sane();
#endif // DEBUG != DEBUG_NONE
	 */
	return;
}

/* --------------------------------------------------------------------------- */

/*
#ifdef RIP_DEBUG
void rt_print_iface_list() {

	struct rt_iface *ptr;

	ptr = rt_db->iface;									// Pointer to the first element of the set.

	PRINTF("\n");
	PRINTF("------------Iface List------------\n");

	while ( ptr != NULL ) {
		PRINTF("[%s]\n", ptr->iface_config->name);
		// PRINTF("Print Pointer (iface): %p\n", ptr);
		// PRINTF("Print Pointer (iface_config): %p\n", (void*)(ptr->iface_config));
		ptr = ptr->next;
	}
	PRINTF("----------------------------------\n");
	PRINTF("\n");
}
#endif
 */



