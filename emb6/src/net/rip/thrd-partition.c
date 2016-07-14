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
	.leader_router_id = 0,
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
	if ( (thrd_dev.type == THRD_DEV_TYPE_ROUTER) || (thrd_dev.type == THRD_DEV_TYPE_REED) ) {

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
	if ( leader_tlv != NULL ) {

		// Compare partition values.
		if ( leader_tlv->weight > thrd_partition.Partition_weight ) {
			attach:
			// TODO Try to attach to other partition.
			LOG_RAW("thrd_partition_process: Try to attach to other partition!\n");
			thrd_partition_set_leader_router_id(leader_tlv->leader_router_id);	// TODO Call this function after successful attachment process.
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
		return THRD_ERROR_NONE;
	}
	LOG_RAW("thrd_partition_process: Invalid Leader Data TLV.\n");
	return THRD_ERROR_INVALID_ARGS;
}

/* --------------------------------------------------------------------------- */

void
thrd_partition_empty(void)
{
	// TODO
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

uint32_t
thrd_partition_get_pid()
{
	return thrd_partition.Partition_ID;
}

/* --------------------------------------------------------------------------- */

uint8_t
thrd_partition_get_weight()
{
	return thrd_partition.Partition_weight;
}

/* --------------------------------------------------------------------------- */

uint8_t
thrd_partition_get_vn_version()
{
	return thrd_partition.VN_version;
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
	LOG_RAW("| Partition_ID | VN_version | VN_stable_version | ID_sequence_number | Partition_weight |\n");
	LOG_RAW("-----------------------------------------------------------------------------------------\n\r");
	LOG_RAW( "| "  "%12lu"
			" | "  "%10d"
			" | "  "%17d"
			" | "  "%18d"
			" | "  "%16d"
			" |\n", thrd_partition.Partition_ID, thrd_partition.VN_version, thrd_partition.VN_stable_version, thrd_partition.ID_sequence_number, thrd_partition.Partition_weight);
	LOG_RAW("=========================================================================================\n\r");
}

/* --------------------------------------------------------------------------- */
