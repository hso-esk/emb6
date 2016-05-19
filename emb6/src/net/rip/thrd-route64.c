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

void thrd_process_route64(tlv_t *tlv) {

	// thrd_rdb_id_t *rid;						// Router IDs.
	// thrd_rdb_link_t *link;
	// thrd_rdb_route_t *route;				// Routing entries.

	if ( tlv->type == TLV_ROUTE64 && tlv->length >= 5 ) {

		uint8_t id_seq_num = tlv->value[0];

		if ( id_seq_num > ID_sequence_number ) {

			// Empty the Router ID Set.
			thrd_rdb_rid_empty();

			ID_sequence_number = id_seq_num;

			uint32_t router_id_mask = 0x00000000;
			router_id_mask |= ( ((uint32_t) tlv->value[1] ) << 24 )
					| ( ((uint32_t) tlv->value[2] ) << 16 )
					| ( ((uint32_t) tlv->value[3] ) << 8 )
					| ( ((uint32_t) tlv->value[4] ) );

			// printf("router_id_mask = %02x\n", router_id_mask);

			uint32_t bit_mask = 0x80000000;

			// Replace the ID Set.
			for ( uint8_t id_cnt = 0; id_cnt < 32; id_cnt++) {
				if ( (router_id_mask & bit_mask) > 0 ) {
					// printf("(router_id_mask & %02x) = (%02x & 0x80000000) = %d\n", bit_mask, router_id_mask, bit_mask, (router_id_mask && 0x80000000));
					thrd_rdb_rid_add(id_cnt);
				}
				// TODO Process Link Quality and Route Data.

				bit_mask >>= 1;
			}
		} else {
		}
	} else {
	}
}

// Outgoing: Trickle timer expired.

static uint8_t route64_tlv[MAX_ROUTE64_TLV_DATA_SIZE];

// thrd_generate_route64().

tlv_t *
thrd_generate_route64()
{
	tlv_t *tlv;								// TLV structure.
	thrd_rdb_id_t *rid;						// Router IDs.
	thrd_rdb_link_t *link;
	thrd_rdb_route_t *route;				// Routing entries.
	uint8_t tlv_len = 5;					// TLV length (value).

	// TLV type.
	route64_tlv[0] = TLV_ROUTE64;

	printf("1\n");
	printf("ID_sequence_number = %d\n", ID_sequence_number);
	printf("2\n");

	// ID Sequence number.
	route64_tlv[2] = 5; //router_id_set->ID_sequence_number;

	rid = thrd_rdb_rid_head();

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
		route64_tlv[tlv_len + 2] = lq_rd;
		tlv_len++;
	}

	route64_tlv[6] = (uint8_t) router_id_mask;
	for ( uint8_t i = 5; i > 2; i-- ) {
		router_id_mask >>= 8;
		route64_tlv[i] = (uint8_t) router_id_mask;
	}

	// Add length.
	route64_tlv[1] = tlv_len;

	if ( tlv_init(&tlv, route64_tlv) == 0 )
		return NULL;

	/*
	// DEBUG. ---
	for ( int i = 0; i < (tlv_len + 2); i++ )
		printf("%02x ", route64_tlv[i]);

	printf("\n");
	// ---
	*/

	// tlv_print(tlv);

	return tlv;
}


// sentMleAdv(tlv_t * tlv)
