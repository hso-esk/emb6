/*
 * thrd-route64.c
 *
 *  Created on: 10 May 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  Route64 TLV Processing.
 */

#include "emb6.h"
#include "net_tlv.h"
#include "thrd-route.h"
#include "tlv.h"

#include "thrd-route64.h"

// Incoming: MLE calls method to process incoming Route64 TLV.

// thrd_process_route64(tlv_t * tlv).

// Outgoing: Trickle timer expired.

uint8_t route64_data[MAX_ROUTE64_TLV_DATA_SIZE];          /**< tlv buffer */

// thrd_generate_route64().

tlv_t *
thrd_generate_route64()
{
	tlv_t *tlv;								// TLV structure.
	thrd_rdb_router_id_t *router_id_set;	// Router ID Set.
	thrd_rdb_id_t *router_id;				// Router IDs.
	thrd_rdb_link_t *link;
	thrd_rdb_route_t *route;				// Routing entries.

	uint8_t rids[thrd_rdb_rid_num_rids()];

	uint8_t rt_cnt = 0;

	if ( tlv_init(&tlv, route64_data) == 0 )
		return NULL;

	// ID Sequence number.
	route64_data[0] = router_id_set->ID_sequence_number;

	router_id = thrd_rdb_rid_head();

	// Router ID Mask and Link Quality and Router Data.
	uint32_t router_id_mask = 0;
	while ( router_id != NULL ) {

		uint8_t lq_rd = 0;	// Link Quality and Route Data.

		router_id_mask |= (0x80000000 >> router_id->router_id);
		// Link Set entry.
		link = thrd_rdb_link_lookup(router_id->router_id);
		lq_rd |= (link->L_outgoing_quality << 6);
		lq_rd |= (link->L_incoming_quality << 4);
		// Routing entry.
		route = thrd_rdb_route_lookup(router_id->router_id);
		lq_rd |= (route->R_route_cost);

		router_id = thrd_rdb_rid_next(router_id);
	}

	return NULL;
}

// sentMleAdv(tlv_t * tlv)
