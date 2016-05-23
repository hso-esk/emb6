/*
 * thrd-leader-db.h
 *
 *  Created on: 26 Apr 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 *  Leader database manipulation.
 */

#ifndef EMB6_INC_NET_RIP_THRD_LEADER_DB_H_
#define EMB6_INC_NET_RIP_THRD_LEADER_DB_H_

#include "clist.h"
#include "rip.h"

void thrd_ldb_init(void);

/**
 * Routers acting as Leaders maintain a routing database for tracking Router ID
 * assignments.
 */

/** \brief An 'ID Assignment Set' records the state of each Router ID.
 * The leader database (ldb) is a set of thrd_ldb_id_assign_t structs. */
typedef struct thrd_ldb_ida {
	struct thrd_ldb_ida *next;

	uint8_t ID_id;			// A Router ID.
	uint8_t ID_owner;		// The IEEE 802.15.4 Extended Address of the Router.
	uint8_t ID_reuse_time;	// The time at which this Router ID MAY be reassigned.
} thrd_ldb_ida_t;

/** \name Routing Database basic routines */
/** @{ */

int thrd_ldb_num_ida(void);

thrd_ldb_ida_t *thrd_ldb_ida_head(void);

thrd_ldb_ida_t *thrd_ldb_ida_next(thrd_ldb_ida_t *i);

thrd_ldb_ida_t *thrd_ldb_ida_lookup(uint8_t router_id);

thrd_ldb_ida_t *thrd_ldb_ida_add(uint8_t router_id, uint8_t owner, uint8_t reuse_time);

void thrd_ldb_ida_rm(thrd_ldb_ida_t *ida);

void thrd_ldb_ida_empty();

#if RIP_DEBUG
void thrd_ldb_print_leader_database(void);
#endif /* RIP_DEBUG */

#endif /* EMB6_INC_NET_RIP_THRD_LEADER_DB_H_ */
