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

#include "thrd-router-id.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"	// For debugging terminal output.

/* --------------------------------------------------------------------------- */

/**
 * Starting a new Partition.
 */
void
thrd_partition_start(void)
{
	if ( (thrd_dev.type == THRD_DEV_TYPE_ROUTER) || (thrd_dev.type == THRD_DEV_TYPE_REED) ) {

		PRINTF("thrd_partition_start: Starting new Thread Partition.\n");
		thrd_leader_init();		// Initialize itself as the leader.

		Partition_ID = (bsp_getrand(0) << 24) //
				| (bsp_getrand(0) << 16)
				| (bsp_getrand(0) << 8)
				| (bsp_getrand(0));
		VN_version = (uint8_t) bsp_getrand(0);
		VN_stable_version = (uint8_t) bsp_getrand(0);
		ID_sequence_number = (uint8_t) bsp_getrand(0);
		Partition_weight = 64;

		thrd_print_partition_data();
	}
}

/* --------------------------------------------------------------------------- */

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
