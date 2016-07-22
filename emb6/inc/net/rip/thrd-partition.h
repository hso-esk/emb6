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

/**
 * Start a new Thread Partition.
 */
extern void thrd_partition_start(void);

/**
 * Process a given Leader Data TLV and compare its content with the available
 * leader data.
 * @param id_sequence_number The corresponding ID Sequence Number (Route64 TLV).
 * @param leader_tlv The Leader Data TLV.
 * @retval THRD_ERROR_NONE			If the Leader Data TLV is valid.
 * @retval THRD_ERROR_INVALID_ARGS	If the Leader Data TLV is invalid.
 * @retval THRD_ERROR_DROP			If the ID Sequence Number is smaller than the current one.
 */
extern thrd_error_t thrd_partition_process(uint8_t id_sequence_number, tlv_leader_t *leader_tlv);

/**
 * Remove all information related to the previous partition.
 * Thread Spec.: Resetting Network Partition Data.
 */
extern void thrd_partition_empty(void);

/**
 * Set the ID Sequence Number.
 * @param id_seq_number The ID Sequence Number.
 */
extern void thrd_partition_set_id_seq_number(uint8_t id_seq_number);

/**
 * Get the ID Sequence Number.
 * @return The ID Sequence Number.
 */
extern uint8_t thrd_partition_get_id_seq_number();

/**
 * Set the Leader Router ID.
 * @param leader_router_id The Leader Router ID.
 */
extern void thrd_partition_set_leader_router_id(uint8_t leader_router_id);

/**
 * Get the Leader Router ID.
 * @return The Leader Router ID.
 */
extern uint8_t thrd_partition_get_leader_router_id();

/**
 * Set the Partition ID.
 * @param partition_id The Partition ID.
 */
extern void thrd_partition_set_pid(uint32_t partition_id);

/**
 * Get the Partition ID.
 * @return The Partition ID.
 */
extern uint32_t thrd_partition_get_pid();

/**
 * Set the partition weight.
 * @param weight The partition weight.
 */
extern void thrd_partition_set_weight(uint8_t weight);

/**
 * Get the partition weight.
 * @return The partition weight.
 */
extern uint8_t thrd_partition_get_weight();

/**
 * Set the VN Version.
 * @param vn_version The VN Version.
 */
extern void thrd_partition_set_vn_version(uint8_t vn_version);

/**
 * Get the VN Version.
 * @return The VN Version.
 */
extern uint8_t thrd_partition_get_vn_version();

/**
 * Set the VN Stable Version.
 * @param vn_stable_version The VN Stable Version.
 */
extern void thrd_partition_set_vn_stable_version(uint8_t vn_stable_version);

/**
 * Get the VN Stable Version.
 * @return The VN Stable Version.
 */
extern uint8_t thrd_partition_get_vn_stable_version();

/**
 * Get the routing cost to reach the leader.
 * @return The routing cost for the leader.
 */
extern uint8_t thrd_partition_get_leader_cost();

/*
 ********************************************************************************
 *                                DEBUG FUNCTIONS
 ********************************************************************************
 */

void thrd_print_partition_data();

#endif /* EMB6_INC_NET_RIP_THRD_PARTITION_H_ */
