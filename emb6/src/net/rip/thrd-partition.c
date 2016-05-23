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

		PRINTF("THRD_PARTITION: Initialized new Thread partition:\n"
				"Partition_ID = %lu\n"
				"VN_version = %d\n"
				"VN_stable_version = %d\n"
				"ID_sequence_number = %d\n"
				"Partition_weight = %d\n",
				Partition_ID, VN_version, VN_stable_version, ID_sequence_number,
				Partition_weight);
	}
}

/* --------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------- */
