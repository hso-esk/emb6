/*
 * thrd-route.h
 *
 *  Created on: 18 Apr 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 *  Routing database manipulation.
 */

#ifndef EMB6_INC_NET_RIP_THRD_ROUTE_H_
#define EMB6_INC_NET_RIP_THRD_ROUTE_H_

#include "emb6.h"
#include "stimer.h"
#include "clist.h"
#include "rip.h"

void thrd_rdb_init(void);

/**
 * A Router maintains a routing database that records information for each separate interface
 * that uses Thread distance-vector routing.
 *
 */

/** \brief The ID Sequence Number. */
uint8_t ID_sequence_number;

/** \brief A 'ID Set' entry in the 'Router ID Set' field in the routing database (rdb). */
typedef struct thrd_rdb_id {
	struct thrd_rdb_id *next;

	uint8_t router_id;
} thrd_rdb_id_t;

/** \brief A 'Link Set entry in the routing database (rdb). */
typedef struct thrd_rdb_link {
	struct thrd_rdb_link *next;

	uint8_t L_router_id;
	uint16_t L_link_margin;
	uint8_t L_incoming_quality;
	uint8_t L_outgoing_quality;
	uint8_t L_age;
} thrd_rdb_link_t;

/** \brief A 'Route Set' entry in the routing database (rdb). */
typedef struct thrd_rdb_route {
	struct thrd_rdb_route *next;

	uint8_t R_destination;
	uint8_t R_next_hop;
	uint8_t R_route_cost;
} thrd_rdb_route_t;

/** \brief Link costs. */
enum thrd_link_cost_t {
	THRD_LINK_COST_1 = 1,
	THRD_LINK_COST_2 = 2,
	THRD_LINK_COST_6 = 6,
	THRD_LINK_COST_INFINTE = MAX_ROUTE_COST,	// Infinite cost.
};

/* --------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------- */

/** ------------------------------- Functions -------------------------------- */

/** \name Routing Database basic routines */
/** @{ */

int thrd_rdb_rid_num_rids(void);

int thrd_rdb_link_num_links(void);

int thrd_rdb_route_num_routes(void);

thrd_rdb_id_t *thrd_rdb_rid_head(void);

thrd_rdb_link_t *thrd_rdb_link_head(void);

thrd_rdb_route_t *thrd_rdb_route_head(void);

thrd_rdb_id_t *thrd_rdb_rid_next(thrd_rdb_id_t *r);

thrd_rdb_link_t *thrd_rdb_link_next(thrd_rdb_link_t *r);

thrd_rdb_route_t *thrd_rdb_route_next(thrd_rdb_route_t *r);

uint8_t *thrd_rdb_route_nexthop(thrd_rdb_route_t *route);

uint8_t thrd_rdb_link_calc_incoming_quality(uint16_t link_margin);

uint8_t thrd_rdb_calc_link_cost(uint8_t incoming_quality);

uint16_t thrd_rdb_link_margin_average(uint16_t old_link_margin, uint16_t new_link_margin);

uint8_t thrd_rdb_link_hysteresis(uint16_t old_link_margin, uint16_t new_link_margin);

uint16_t thrd_rdb_link_margin_get_lower_bound(uint16_t link_margin);

thrd_rdb_id_t *thrd_rdb_rid_lookup(uint8_t destination);

thrd_rdb_link_t *thrd_rdb_link_lookup(uint8_t router_id);

thrd_rdb_route_t *thrd_rdb_route_lookup(uint8_t destination);

thrd_rdb_id_t *thrd_rdb_rid_add(uint8_t router_id);

thrd_rdb_route_t *thrd_rdb_route_add(uint8_t destination, uint8_t next_hop,
		uint8_t route_cost);

void thrd_rdb_rid_rm(thrd_rdb_id_t *rid);

void thrd_rdb_rid_empty();

void thrd_rdb_link_rm(thrd_rdb_link_t *link);

void thrd_rdb_route_rm(thrd_rdb_route_t *route);

/*
 ********************************************************************************
 *                                 API FUNCTIONS
 ********************************************************************************
 */

thrd_rdb_link_t *thrd_rdb_link_update(uint8_t router_id, uint8_t link_margin,
		uint8_t outgoing_quality, uint8_t age);

thrd_rdb_route_t *thrd_rdb_route_update(uint8_t router_id, uint8_t destination,
		uint8_t cost_reported);

/*
 ********************************************************************************
 *                                DEBUG FUNCTIONS
 ********************************************************************************
 */

#if RIP_DEBUG
void thrd_rdb_print_rid_set(void);
void thrd_rdb_print_link_set(void);
void thrd_rdb_print_route_set(void);
void thrd_rdb_print_routing_database(void);
#endif /* RIP_DEBUG */

#endif /* EMB6_INC_NET_RIP_THRD_ROUTE_H_ */
