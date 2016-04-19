/*
 * leader.h
 *
 *  Created on: 13 Apr 2016
 *      Author: osboxes
 */

#ifndef EMB6_INC_NET_RIP_LEADER_H_
#define EMB6_INC_NET_RIP_LEADER_H_

/**
 * Routers acting as Leaders maintain an additional database for tracking Router ID assignments.
 * Similar to the Routing Database, a Leader maintains a separate Leader Database for each interface
 * on which a device is acting as a Leader.
 *
 */

/** Leader Database Instance */
struct ld_db_instance
{
	// TODO
	ld_iface iface;
};

/** Interface. */
struct ld_iface
{
	ld_id_assign_set id_assign_set;

	struct ld_iface *next;
};


/** ID Assignment Set */
struct ld_id_assign_set
{
	uint8_t ID_id;					// A Router ID.
	uint8_t ID_owner;				// The IEEE 802.15.4 Extended Address of the Router which this Router ID is currently assigned, or, if not assigned, the IEEE 802.15.4 Extended Address of the Router to which it was most recently assigned.
	uint8_t ID_reuse_time;			// The time at which this Router ID MAY be reassigned, if it is not currently assigned.

	struct ld_assign_set *next;
};


#endif /* EMB6_INC_NET_RIP_LEADER_H_ */
