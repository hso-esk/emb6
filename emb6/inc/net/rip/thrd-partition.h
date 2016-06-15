/*
 * thrd-partition.h
 *
 *  Created on: 23 May 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 *  Thread Network Partitions.
 */

#ifndef EMB6_INC_NET_RIP_THRD_PARTITION_H_
#define EMB6_INC_NET_RIP_THRD_PARTITION_H_

#include "tlv.h"

/**
 * Thread Network Partition.
 */
typedef struct {
	uint8_t leader_router_id;
	uint32_t Partition_ID;
	uint8_t VN_version;
	uint8_t VN_stable_version;
	uint8_t ID_sequence_number;
	uint8_t Partition_weight;
} thrd_partition_t;

/*! Thread Network Partition. */
extern thrd_partition_t thrd_partition;

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

/*==============================================================================
                                     API FUNCTIONS
 =============================================================================*/

void thrd_partition_start(void);

/**
 * Process a given Leader Data TLV and compare its content with the available
 * leader data.
 * @param id_sequence_number The corresponding ID Sequence Number (Route64 TLV).
 * @param leader_tlv The Leader Data TLV.
 * @return 0, if the partition topology changes (the Leader Data TLV is invalid).
 *         1, if the partition data is valid (this should trigger the procession
 *         of a Route64 TLV).
 */
uint8_t thrd_partition_process(uint8_t id_sequence_number, tlv_leader_t *leader_tlv);

/**
 * Remove all information related to the previous partition.
 * Thread Spec.: Resetting Network Partition Data.
 */
void thrd_partition_empty(void);

/**
 * Set the Leader Router ID.
 * @param leader_router_id The Leader Router ID.
 */
void thrd_set_leader_router_id(uint8_t leader_router_id);

/**
 * Get the Leader Router ID.
 * @return The Leader Router ID.
 */
uint8_t thrd_get_leader_router_id();

/*
 ********************************************************************************
 *                                DEBUG FUNCTIONS
 ********************************************************************************
 */

void thrd_print_partition_data();

#endif /* EMB6_INC_NET_RIP_THRD_PARTITION_H_ */
