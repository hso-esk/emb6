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
#include "thrd-partition.h"

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

static uint8_t route64_data[MAX_ROUTE64_TLV_DATA_SIZE];

/*==============================================================================
                               LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

static void thrd_process_route64(uint8_t rid_sender, tlv_route64_t *route64_tlv);
static uint8_t thrd_get_id_seq_number(tlv_route64_t *route64_tlv);

/*==============================================================================
                                    LOCAL FUNCTIONS
 =============================================================================*/

/* -------------------------------------------------------------------------- */

static void
thrd_process_route64(uint8_t rid_sender, tlv_route64_t *route64_tlv)
{
	thrd_rdb_link_t *link;

	if ( route64_tlv != NULL ) {

		if ( route64_tlv->id_sequence_number > ID_sequence_number ) {

			// Empty the Router ID Set.
			thrd_rdb_rid_empty();

			ID_sequence_number = route64_tlv->id_sequence_number;

			// printf("router_id_mask = %02x\n", router_id_mask);

			uint64_t bit_mask = 0x8000000000000000;
			uint8_t data_cnt = 0;

			PRINTF("thrd_process_route64: route64_tlv->router_id_msk = %lu\n", route64_tlv->router_id_mask);

			// Replace the ID Set.
			for ( uint8_t id_cnt = 0; id_cnt < 64; id_cnt++) {
				if ( (route64_tlv->router_id_mask & bit_mask) > 0 ) {

					thrd_rdb_rid_add(id_cnt);

					// Process Link Quality and Route Data.

					// Incoming quality.
					uint8_t lq_rd_data = (route64_tlv->lq_rd[data_cnt] & 0x30) >> 6;
					if ( lq_rd_data != 0 ) {
						link = thrd_rdb_link_lookup(id_cnt);
						link->L_outgoing_quality = lq_rd_data;
					}
					// Route data.
					lq_rd_data = (route64_tlv->lq_rd[data_cnt] & 0x0F);
					// TODO Check whether the destination differs to the current router id. (Otherwise, we would create a loop).
					thrd_rdb_route_update(rid_sender, id_cnt, lq_rd_data);

					// printf("update route: %d | %d | %d\n", id_cnt, rid_sender, lq_rd_data);

					data_cnt++;
				}
				bit_mask >>= 1;
			}
		}
	}
}

/* -------------------------------------------------------------------------- */

static uint8_t
thrd_get_id_seq_number(tlv_route64_t  *route64_tlv)
{
	return route64_tlv->id_sequence_number;
}

/*==============================================================================
                                     API FUNCTIONS
 =============================================================================*/

void
thrd_process_adv(uint16_t source_addr, tlv_route64_t *route64_tlv, tlv_leader_t *leader_tlv)
{
	thrd_partition_process(thrd_get_id_seq_number(route64_tlv), leader_tlv);
	thrd_process_route64((source_addr & 0xFC00) >> 10, route64_tlv);

	/* Correct solution:
	if ( thrd_partition_process(thrd_get_id_seq_number(route64_tlv), leader_tlv) ) {
		thrd_process_route64((source_addr & 0xFC00) >> 10, route64_tlv);
	}
	*/
}

/* -------------------------------------------------------------------------- */

size_t
thrd_generate_route64(tlv_route64_t *route64_tlv)
{
	size_t len = 9;
	// tlv_route64_t *route64_tlv;				// Route64 TLV structure.
	thrd_rdb_id_t *rid;						// Router IDs.
	thrd_rdb_link_t *link;
	thrd_rdb_route_t *route;				// Routing entries.
	uint8_t lq_rq_pos = 9;					// Position of the first link quality and route data byte.

	route64_data[0] = ID_sequence_number;

	rid = thrd_rdb_rid_head();

	// Router ID Mask and Link Quality and Router Data.
	uint64_t router_id_mask = 0x0000000000000000;
	for ( rid = thrd_rdb_rid_head(); rid != NULL; rid = thrd_rdb_rid_next(rid) ) {

		uint8_t lq_rd = 0x00;	// Link Quality and Route Data.

		router_id_mask |= (0x8000000000000000 >> rid->router_id);

		if ( rid->router_id != thrd_dev.Router_ID ) {

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
		} else {
			lq_rd = 0x01;
		}

		route64_data[lq_rq_pos] = lq_rd;
		lq_rq_pos++;
		len++;
	}

	route64_data[8] = (uint8_t) router_id_mask;
	for ( uint8_t i = 7; i > 0; i-- ) {
		router_id_mask >>= 8;
		route64_data[i] = (uint8_t) router_id_mask;
	}

	if ( tlv_route64_init(&route64_tlv, route64_data) == 0 )
		return 0;

	return len;
}

/* --------------------------------------------------------------------------- */

tlv_leader_t*
thrd_generate_leader_data_tlv(void)
{
	uint8_t tlv_buf[8] = { 0 };
	tlv_leader_t *ld_tlv;

	memcpy(&tlv_buf[0], &Partition_ID, 4);
	tlv_buf[4] = Partition_weight;
	tlv_buf[5] = VN_version;
	tlv_buf[6] = VN_stable_version;
	tlv_buf[7] = leader_router_id;
	tlv_leader_init(&ld_tlv, &tlv_buf[0]);
	return ld_tlv;
}

/* --------------------------------------------------------------------------- */

void
print_leader_data_tlv(tlv_leader_t * leader_tlv)
{
	printf(ANSI_COLOR_RED
			"|============================== LEADER DATA TLV ================================|"
			ANSI_COLOR_RESET "\n\r");
	printf("| " ANSI_COLOR_YELLOW);
	printf("Partition ID = %d\n", leader_tlv->partition_id);
	printf("Weight = %d\n", leader_tlv->weight);
	printf("Data Version = %d\n", leader_tlv->data_version);
	printf("Stable Data Version = %d\n", leader_tlv->stable_data_version);
	printf("Leader Router ID = %d\n", leader_tlv->leader_router_id);
	printf(ANSI_COLOR_RESET " |\n");
	printf(ANSI_COLOR_RED
			"|===============================================================================|"
			ANSI_COLOR_RESET "\n\r");
}

