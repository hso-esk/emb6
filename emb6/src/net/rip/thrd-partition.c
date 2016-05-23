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
#include "thrd-partition.h"
#include "rip.h"

#include "thrd-router-id.h"

#include "random.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"	// For debugging terminal output.

/* --------------------------------------------------------------------------- */

/**
 * Starting a new Partition.
 */
void
thrd_partition_init(void)
{
	if ( (thrd_dev.type == THRD_DEV_TYPE_ROUTER) || (thrd_dev.type == THRD_DEV_TYPE_REED) ) {

		Partition_ID = random_rand();
		VN_version = (uint8_t) random_rand();
		VN_stable_version = (uint8_t) random_rand();
		ID_sequence_number = (uint8_t) random_rand();
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
	PRINTF("| " ANSI_COLOR_YELLOW "%12d" ANSI_COLOR_RESET
			" | " ANSI_COLOR_YELLOW "%10d" ANSI_COLOR_RESET
			" | " ANSI_COLOR_YELLOW "%17d" ANSI_COLOR_RESET
			" | " ANSI_COLOR_YELLOW "%18d" ANSI_COLOR_RESET
			" | " ANSI_COLOR_YELLOW "%16d" ANSI_COLOR_RESET
			" |\n", Partition_ID, VN_version, VN_stable_version, ID_sequence_number, Partition_weight);
	PRINTF(ANSI_COLOR_CYAN "=========================================================================================" ANSI_COLOR_RESET "\n\r");
}

/* --------------------------------------------------------------------------- */
