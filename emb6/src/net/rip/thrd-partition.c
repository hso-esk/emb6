/*
 * thrd-partition.c
 *
 *  Created on: 23 May 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 *  Thread Network Partitions.
 */

#include "emb6.h"
#include "thread_conf.h"
#include "bsp.h"
#include "thrd-partition.h"
#include "rip.h"
#include "tlv.h"

#include "thrd-dev.h"
#include "thrd-eid-rloc.h"
#include "thrd-route.h"
#include "thrd-addr-query.h"
#include "thrd-router-id.h"
#include "thrd-iface.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"	// For debugging terminal output.

#define     LOGGER_ENABLE                 LOGGER_THRD_NET
#include    "logger.h"

/**
 *  Thread Network Partition.
 */
thrd_partition_t thrd_partition = {
	.leader_router_id = 63,
	.Partition_ID = 0,
	.VN_version = 0,
	.VN_stable_version = 0,
	.ID_sequence_number = 0,
	.Partition_weight = 0
};

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

/*==============================================================================
                               LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

/*==============================================================================
                                    LOCAL FUNCTIONS
 =============================================================================*/

/* --------------------------------------------------------------------------- */

/*==============================================================================
                                     API FUNCTIONS
 =============================================================================*/

/**
 * Starting a new Partition.
 */
void
thrd_partition_start(void)
{
	if ( (thrd_dev.net_type == THRD_DEV_NETTYPE_ROUTER) || (thrd_dev.net_type == THRD_DEV_NETTYPE_REED) ) {

		LOG_RAW("thrd_partition_start: Starting new Thread Partition.\n");
		// Resetting Network Partition Data.
		thrd_eid_rloc_db_init();		// Initialize EID-to-RLOC Mapping.
		thrd_eid_rloc_cache_init();		// Initialize EID-to-RLOC Map Cache.
		thrd_rdb_init();				// Initialize Routing Database.
		thrd_leader_init();				// Initialize itself as the Leader.

		thrd_partition.leader_router_id = thrd_iface.router_id;
		thrd_partition.Partition_ID = (bsp_getrand(0) << 16) | (bsp_getrand(0));
		thrd_partition.VN_version = (uint8_t) bsp_getrand(0);
		thrd_partition.VN_stable_version = (uint8_t) bsp_getrand(0);
		thrd_partition.ID_sequence_number = (uint8_t) bsp_getrand(0);
		thrd_partition.Partition_weight = 64;

		thrd_print_partition_data();
		thrd_ldb_print_leader_database();
	}
}

/* --------------------------------------------------------------------------- */

thrd_error_t
thrd_partition_process(uint8_t id_sequence_number, tlv_leader_t *leader_tlv)
{
	if ( id_sequence_number > thrd_partition.ID_sequence_number ) {

		thrd_partition.ID_sequence_number = id_sequence_number;

		if ( leader_tlv != NULL ) {
			// Compare partition values.
			if ( leader_tlv->weight > thrd_partition.Partition_weight ) {
				attach:
				// TODO Try to attach to other partition.
				LOG_RAW("thrd_partition_process: Attaching to existing Thread Partition!\n");
				// TODO Call this functions after successful attachment process.
				thrd_partition_set_pid(leader_tlv->partition_id);
				thrd_partition_set_weight(leader_tlv->weight);
				thrd_partition_set_vn_version(leader_tlv->data_version);
				thrd_partition_set_vn_stable_version(leader_tlv->stable_data_version);
				thrd_partition_set_leader_router_id(leader_tlv->leader_router_id);

			} else if ( leader_tlv->weight == thrd_partition.Partition_weight ) {
				if ( leader_tlv->partition_id > thrd_partition.Partition_ID ) {
					goto attach;
				} else if ( leader_tlv->partition_id == thrd_partition.Partition_ID ) {
					if ( (leader_tlv->data_version > thrd_partition.VN_version)
							&& (leader_tlv->stable_data_version > thrd_partition.VN_stable_version)
							&& (id_sequence_number > thrd_partition.ID_sequence_number)) {
						// Inconsistency detected -> Start a new Partition.
						thrd_partition_start();
					}
				}
			}
			thrd_dev_print_dev_info();
			return THRD_ERROR_NONE;
		}
		LOG_RAW("thrd_partition_process: Invalid Leader Data TLV.\n");
		return THRD_ERROR_INVALID_ARGS;
	}
	return THRD_ERROR_DROP;
}

/* --------------------------------------------------------------------------- */

void
thrd_partition_empty(void)
{
	// TODO
}

/* --------------------------------------------------------------------------- */

void
thrd_partition_set_id_seq_number(uint8_t id_seq_number)
{
	thrd_partition.ID_sequence_number = id_seq_number;
}

/* --------------------------------------------------------------------------- */

uint8_t
thrd_partition_get_id_seq_number()
{
	return thrd_partition.ID_sequence_number;
}

/* --------------------------------------------------------------------------- */

void
thrd_partition_set_leader_router_id(uint8_t leader_router_id)
{
	thrd_partition.leader_router_id = leader_router_id;
}

/* --------------------------------------------------------------------------- */

uint8_t
thrd_partition_get_leader_router_id()
{
	return thrd_partition.leader_router_id;
}

/* --------------------------------------------------------------------------- */

void
thrd_partition_set_pid(uint32_t partition_id)
{
	thrd_partition.Partition_ID = partition_id;
}

/* --------------------------------------------------------------------------- */

uint32_t
thrd_partition_get_pid()
{
	return thrd_partition.Partition_ID;
}

/* --------------------------------------------------------------------------- */

void
thrd_partition_set_weight(uint8_t weight)
{
	thrd_partition.Partition_weight = weight;
}

/* --------------------------------------------------------------------------- */

uint8_t
thrd_partition_get_weight()
{
	return thrd_partition.Partition_weight;
}

/* --------------------------------------------------------------------------- */

void
thrd_partition_set_vn_version(uint8_t vn_version)
{
	thrd_partition.VN_version = vn_version;
}

/* --------------------------------------------------------------------------- */

uint8_t
thrd_partition_get_vn_version()
{
	return thrd_partition.VN_version;
}

/* --------------------------------------------------------------------------- */

void
thrd_partition_set_vn_stable_version(uint8_t vn_stable_version)
{
	thrd_partition.VN_stable_version = vn_stable_version;
}

/* --------------------------------------------------------------------------- */

uint8_t
thrd_partition_get_vn_stable_version()
{
	return thrd_partition.VN_stable_version;
}

/* --------------------------------------------------------------------------- */
uint8_t
thrd_partition_get_leader_cost()
{
	thrd_rdb_route_t *route;
	route = thrd_rdb_route_lookup(thrd_partition.leader_router_id);
	return (route != NULL) ? route->R_route_cost : 63;
}

/*
 ********************************************************************************
 *                                DEBUG FUNCTIONS
 ********************************************************************************
 */

void
thrd_print_partition_data()
{
	LOG_RAW("|================================== THREAD PARTITION ===================================|\n\r");
	LOG_RAW("| Leader Router ID = %2d                                                                 |\n\r");
	LOG_RAW("| ------------------------------------------------------------------------------------- |\n\r");
	LOG_RAW("| Partition_ID | VN_version | VN_stable_version | ID_sequence_number | Partition_weight |\n");
	LOG_RAW("|---------------------------------------------------------------------------------------|\n\r");
	LOG_RAW( "| "  "%12lu"
			" | "  "%10d"
			" | "  "%17d"
			" | "  "%18d"
			" | "  "%16d"
			" |\n", thrd_partition.Partition_ID, thrd_partition.VN_version, thrd_partition.VN_stable_version, thrd_partition.ID_sequence_number, thrd_partition.Partition_weight);
	LOG_RAW("|=======================================================================================|\n\r");
}

/* --------------------------------------------------------------------------- */
