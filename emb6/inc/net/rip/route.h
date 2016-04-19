/*
 * route.h
 *
 *  Created on: 12 Apr 2016
 *      Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 */

#ifndef EMB6_INC_NET_RIP_ROUTE_H_
#define EMB6_INC_NET_RIP_ROUTE_H_

/**
 * A Router maintains a routing database that records information for each separate interface
 * that uses Thread distance-vector routing.
 *
 */

/* --------------------------------------------------------------------------- */

struct rt_db_instance *rt_db;

/** Routing Database Instance */
struct rt_db_instance
{
	// TODO
	struct rt_iface *iface;				// List of interfaces.
	uint8_t iface_id;
};

/** Interface.
 * Read Thread Specification, section 3.4 'Multiple Interfaces' for more information.
 * */
struct rt_iface
{
	// struct rt_id_set *id_set;			// The Router ID Set.
	// struct rt_link_set *link_set;		// The Link Set.
	// struct rt_route_set *route_set;		// The Route Set.

	struct rt_iface_config *iface_config;				// The interface configuration.

	struct rt_iface *next;
};

/** Interface configuration */
struct rt_iface_config
{
	char name[16];							// The name of the interface.
	// uip_ipaddr_t addr;				// IP Address.
	// uint16_t port;					// Source and destination port.
};

/** Router ID Set */
struct rt_id_set
{
	uint8_t id;						// A valid Router ID.

	struct rt_id_set *next;
};

/** Link Set */
struct rt_link_set
{
	uint8_t L_router_id;			// The Router ID assigned to that neighbor.
	uint8_t L_link_margin;			// The measured link margin for messages received from the neighbor.
	uint8_t L_incoming_quality;		// The incoming link quality metric as calculated from L_link_margin.
	uint8_t L_outgoing_quality;		// The incoming link quality metric reported by the neighbor for messages arriving from this Router.
	uint8_t L_age;					// The elapsed time since an advertisement was received from the neighbor.

	struct rt_link_set *next;
};

/** Route Set */
struct rt_route_set
{
	uint8_t R_destination;			// The Router ID assigned to the destination of this route.
	uint8_t R_next_hop;				// The Router ID assigned to the next hop along the route.
	uint8_t R_route_cost;			// The routing cost reported by R_next_hop for R_destination.

	struct rt_route_set *next;
};

/* --------------------------------------------------------------------------- */

/** ------------------------------------ Functions ------------------------------------ */


#ifdef RIP_DEBUG
/** Print all interfaces */
void rt_print_iface_list();
#endif


#endif /* EMB6_INC_NET_RIP_ROUTE_H_ */
