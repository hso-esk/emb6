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

#include "thrd-router-id.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"	// For debugging terminal output.

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

		PRINTF("thrd_partition_start: Starting new Thread Partition.\n");
		thrd_leader_init();		// Initialize itself as the leader.

		Partition_ID = (bsp_getrand(0) << 16) | (bsp_getrand(0));
		VN_version = (uint8_t) bsp_getrand(0);
		VN_stable_version = (uint8_t) bsp_getrand(0);
		ID_sequence_number = (uint8_t) bsp_getrand(0);
		Partition_weight = 64;

		thrd_print_partition_data();
	}
}

/* --------------------------------------------------------------------------- */

/**
 * Process a given Leader Data TLV and compare its content with the available
 * leader data.
 * @param id_sequence_number The corresponding ID Sequence Number (Route64 TLV).
 * @param leader_tlv The Leader Data TLV.
 */
void
thrd_partition_process(uint8_t id_sequence_number, tlv_t *leader_tlv)
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

		PRINTF("TEST!\n");

		// Compare partition values.
		if ( weight > Partition_weight ) {
			attach:
			// TODO Try to attach to other partition.
			PRINTF("thrd_partition_process: Try to attach to other partition!\n");
		} else if ( weight == Partition_weight ) {
			if ( partition_id > Partition_ID ) {
				goto attach;
			} else if ( partition_id == Partition_ID ) {
				if ( (data_version > VN_version)
						&& (stable_data_version > VN_stable_version)
						&& (id_sequence_number > ID_sequence_number)) {
					// Inconsistency detected -> Start a new Partition.
					thrd_partition_start();
				}
			}
		}
		return;
	}
	PRINTF("thrd_partition_process: Invalid Leader Data TLV.\n");
}

/* --------------------------------------------------------------------------- */

/**
 * Remove all information related to the previous partition.
 * Thread Spec.: Resetting Network Partition Data.
 */
void
thrd_partition_empty(void)
{
	// TODO
}

/*
 ********************************************************************************
 *                                DEBUG FUNCTIONS
 ********************************************************************************
 */

void
thrd_print_partition_data()
{
	PRINTF(ANSI_COLOR_CYAN "|================================== THREAD PARTITION ===================================|" ANSI_COLOR_RESET "\n\r");;
	PRINTF("| Partition_ID | VN_version | VN_stable_version | ID_sequence_number | Partition_weight |\n");
	PRINTF("-----------------------------------------------------------------------------------------\n\r");
	PRINTF("| " ANSI_COLOR_YELLOW "%12lu" ANSI_COLOR_RESET
			" | " ANSI_COLOR_YELLOW "%10d" ANSI_COLOR_RESET
			" | " ANSI_COLOR_YELLOW "%17d" ANSI_COLOR_RESET
			" | " ANSI_COLOR_YELLOW "%18d" ANSI_COLOR_RESET
			" | " ANSI_COLOR_YELLOW "%16d" ANSI_COLOR_RESET
			" |\n", Partition_ID, VN_version, VN_stable_version, ID_sequence_number, Partition_weight);
	PRINTF(ANSI_COLOR_CYAN "=========================================================================================" ANSI_COLOR_RESET "\n\r");
}

/* --------------------------------------------------------------------------- */
