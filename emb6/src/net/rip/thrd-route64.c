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

static uint8_t route64_data[MAX_ROUTE64_TLV_DATA_SIZE];

// thrd_generate_route64().

tlv_t *
thrd_generate_route64()
{
	tlv_t *tlv;								// TLV structure.
	thrd_rdb_router_id_t *router_id_set;	// Router ID Set.
	thrd_rdb_id_t *rid;				// Router IDs.
	thrd_rdb_link_t *link;
	thrd_rdb_route_t *route;				// Routing entries.

	if ( tlv_init(&tlv, route64_data) == 0 )
		return NULL;

	// ID Sequence number.
	route64_data[0] = 5; // router_id_set->ID_sequence_number;

	rid = thrd_rdb_rid_head();

	uint8_t tlv_len = 5;		// Static TLV length.

	// Router ID Mask and Link Quality and Router Data.
	uint32_t router_id_mask = 0;
	for ( rid = thrd_rdb_rid_head(); rid != NULL; rid = thrd_rdb_rid_next(rid) ) {

		uint8_t lq_rd = 0x00;	// Link Quality and Route Data.

		router_id_mask |= (0x80000000 >> rid->router_id);
		// Link Set entry.
		link = thrd_rdb_link_lookup(rid->router_id);
		if ( link != NULL ) {
			lq_rd |= (link->L_outgoing_quality << 6);
			lq_rd |= (link->L_incoming_quality << 4);
		}
		// Routing entry.
		route = thrd_rdb_route_lookup(rid->router_id);
		if ( route != NULL ) {
			lq_rd |= (route->R_route_cost);
		}
		route64_data[tlv_len] = lq_rd;
		tlv_len++;
	}

	route64_data[4] = (uint8_t) router_id_mask;
	for ( uint8_t i = 3; i > 0; i-- ) {
		router_id_mask >>= 8;
		route64_data[i] = (uint8_t) router_id_mask;
	}

	// DEBUG. ---
	for ( int i = 0; i < tlv_len; i++ )
		printf("%02x ", route64_data[i]);

	printf("\n");
	// ---

	tlv_write(tlv, TLV_ROUTE64, tlv_len, route64_data);

	tlv_print(tlv);

	return tlv;
}


// sentMleAdv(tlv_t * tlv)
