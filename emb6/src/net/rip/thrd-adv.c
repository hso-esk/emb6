/*
 * thrd-adv.c
 *
 *  Created on: 10 May 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  MLE Advertisement Processing / Route64 TLV Generation.
 */

#include "thrd-adv.h"

#include "emb6.h"
#include "net_tlv.h"
#include "thrd-route.h"
#include "tlv.h"

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

static uint8_t route64_tlv[MAX_ROUTE64_TLV_DATA_SIZE];

/*==============================================================================
                               LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

static uint16_t thrd_process_source(tlv_t *source_tlv);
static void thrd_process_route64(uint8_t rid_sender, tlv_t *route64_tlv);
static void thrd_process_leader(tlv_t *leader_tlv);

/*==============================================================================
                                    LOCAL FUNCTIONS
 =============================================================================*/

/**
 * Get the router id of the given Source TLV.
 * @param source_tlv
 * @return The router id.
 */
static uint16_t
thrd_process_source(tlv_t *source_tlv)
{
	if ( (source_tlv->type == TLV_SOURCE_ADDRESS) && (source_tlv->length == 2) ) {
		uint16_t rloc16 = (source_tlv->value[0] << 8)
					| source_tlv->value[1];
		return rloc16;
	}
	return 0xFFFF;	// Return invalid value to signal a failure.
}

/* -------------------------------------------------------------------------- */

static void
thrd_process_route64(uint8_t rid_sender, tlv_t *route64_tlv)
{

	thrd_rdb_link_t *link;

	if ( route64_tlv->type == TLV_ROUTE64 && route64_tlv->length >= 5 ) {

		uint8_t id_seq_num = route64_tlv->value[0];

		if ( id_seq_num > ID_sequence_number ) {

			// Empty the Router ID Set.
			thrd_rdb_rid_empty();

			ID_sequence_number = id_seq_num;

			uint32_t router_id_mask = 0x00000000;
			router_id_mask |= ( ((uint32_t) route64_tlv->value[1] ) << 24 )
					| ( ((uint32_t) route64_tlv->value[2] ) << 16 )
					| ( ((uint32_t) route64_tlv->value[3] ) << 8 )
					| ( ((uint32_t) route64_tlv->value[4] ) );

			// printf("router_id_mask = %02x\n", router_id_mask);

			uint32_t bit_mask = 0x80000000;
			uint8_t data_cnt = 0;

			// Replace the ID Set.
			for ( uint8_t id_cnt = 0; id_cnt < 32; id_cnt++) {
				if ( (router_id_mask & bit_mask) > 0 ) {
					// printf("(router_id_mask & %02x) = (%02x & 0x80000000) = %d\n", bit_mask, router_id_mask, bit_mask, (router_id_mask && 0x80000000));
					thrd_rdb_rid_add(id_cnt);

					// Process Link Quality and Route Data.

					// Incoming quality.
					uint8_t lq_rd_data = (route64_tlv->value[5 + data_cnt] & 0x30) >> 6;
					if ( lq_rd_data != 0 ) {
						link = thrd_rdb_link_lookup(id_cnt);
						link->L_outgoing_quality = lq_rd_data;
					}
					// Route data.
					lq_rd_data = (route64_tlv->value[5 + data_cnt] & 0x0F);
					// TODO Check whether the destination differs to the current router id. (Otherwise, we would create a loop).
					thrd_rdb_route_update(rid_sender, id_cnt, lq_rd_data);

					// printf("update route: %d | %d | %d\n", id_cnt, rid_sender, lq_rd_data);

					data_cnt++;
				}
				bit_mask >>= 1;
			}
		} else {
		}
	} else {
	}
}

/* -------------------------------------------------------------------------- */

static void
thrd_process_leader(tlv_t *leader_tlv)
{
	if ( (leader_tlv->type == TLV_LEADER_DATA) && (leader_tlv->length == 8) ) {
		uint32_t partition_id = (leader_tlv->value[0] << 24)
					| (leader_tlv->value[1] << 16)
					| (leader_tlv->value[2] << 8)
					| (leader_tlv->value[3]);
		uint8_t weight = leader_tlv->value[4];
		uint8_t data_version = leader_tlv->value[5];
		uint8_t stable_data_version = leader_tlv->value[6];
		uint8_t leader_router_id = leader_tlv->value[7];

		PRINTF("partition_id: %lu, weight: %d, data_version: %d, stable_data_version = %d, leader_router_id= %d\n",
				partition_id, weight, data_version, stable_data_version, leader_router_id);

		// TODO I think this is the place where partitioning should be handled.
	}
}

/*==============================================================================
                                     API FUNCTIONS
 =============================================================================*/

void
thrd_process_adv(tlv_t *source_tlv, tlv_t *route64_tlv, tlv_t *leader_tlv)
{
	// Obtain the router id of the sender.
	uint8_t sender_rid = (thrd_process_source(source_tlv) & 0xFC00) >> 10;
	thrd_process_route64(sender_rid, route64_tlv);
	thrd_process_leader(leader_tlv);
}

/* -------------------------------------------------------------------------- */

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

	// ID Sequence number.
	route64_tlv[2] = ID_sequence_number;

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

	return tlv;
}

