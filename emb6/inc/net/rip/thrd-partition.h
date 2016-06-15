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
