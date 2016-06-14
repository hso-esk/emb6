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

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

uint8_t leader_router_id;
uint32_t Partition_ID;
uint8_t VN_version;
uint8_t VN_stable_version;
uint8_t ID_sequence_number;

uint8_t Partition_weight;

// uint8_t Leader_router_id; // ???

/*==============================================================================
                                     API FUNCTIONS
 =============================================================================*/

void thrd_partition_start(void);

uint8_t thrd_partition_process(uint8_t id_sequence_number, tlv_leader_t *leader_tlv);

void thrd_partition_empty(void);

/*
 ********************************************************************************
 *                                DEBUG FUNCTIONS
 ********************************************************************************
 */

void thrd_print_partition_data();

#endif /* EMB6_INC_NET_RIP_THRD_PARTITION_H_ */
