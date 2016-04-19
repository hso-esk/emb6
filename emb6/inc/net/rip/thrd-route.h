/*
 * thrd-route.h
 *
 *  Created on: 18 Apr 2016
 *      Author: osboxes
 */

#ifndef EMB6_INC_NET_RIP_THRD_ROUTE_H_
#define EMB6_INC_NET_RIP_THRD_ROUTE_H_

#include "stimer.h"
#include "clist.h"

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_RESET   "\x1b[0m"

void thrd_rdb_init(void);

/**
 * A Router maintains a routing database that records information for each separate interface
 * that uses Thread distance-vector routing.
 *
 */

/** \brief A 'ID Set' entry in the 'Router ID Set' field in the routing database (rdb). */
typedef struct thrd_rdb_id {
	struct thrd_rdb_id *next;

	uint8_t router_id;
} thrd_rdb_id_t;

/** \brief A 'Router ID Set' entry in the routing database (rdb). */
typedef struct thrd_rdb_router_id {

  thrd_rdb_id_t *ID_set;
  uint8_t ID_sequence_number;
} thrd_rdb_router_id_t;

/** \brief A 'Link Set entry in the routing database (rdb). */
typedef struct thrd_rdb_link {
  struct thrd_rdb_link *next;

  uint8_t L_router_id;
  uint8_t L_link_margin;
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

/* --------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------- */

/** ------------------------------- Functions -------------------------------- */

/** \name Routing Database basic routines */
/** @{ */

/** Look up a given router id in the Router ID Set. */
thrd_rdb_id_t *thrd_rdb_rid_lookup(uint8_t destination);

/** Look up a given router id in the Link Set. */
thrd_rdb_link_t *thrd_rdb_link_lookup(uint8_t destination);

int thrd_rdb_route_num_routes(void);

void thrd_rdb_route_rm(thrd_rdb_route_t *route);

uint8_t *thrd_rdb_route_nexthop(thrd_rdb_route_t *route);

thrd_rdb_id_t *thrd_rdb_rid_next(thrd_rdb_id_t *r);

thrd_rdb_link_t *thrd_rdb_link_next(thrd_rdb_link_t *r);

thrd_rdb_route_t *thrd_rdb_route_next(thrd_rdb_route_t *r);

/** Get the first element of the Router ID Set. */
thrd_rdb_id_t *thrd_rdb_rid_head(void);

/** Get the first element of the Link Set (neighbor table). */
thrd_rdb_link_t *thrd_rdb_link_head(void);

/** Get the first element of the Route Set. */
thrd_rdb_route_t *thrd_rdb_route_head(void);

thrd_rdb_route_t *thrd_rdb_route_lookup(uint8_t destination);

thrd_rdb_route_t *thrd_rdb_route_add(uint8_t destination, uint8_t next_hop,
                                   uint8_t route_cost);

#ifdef RIP_DEBUG
/** Print all interfaces */
void rt_print_iface_list();
#endif

#endif /* EMB6_INC_NET_RIP_THRD_ROUTE_H_ */
