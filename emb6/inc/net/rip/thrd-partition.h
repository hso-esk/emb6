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

uint32_t Partition_ID;
uint8_t VN_version;
uint8_t VN_stable_version;
uint8_t ID_sequence_number;

uint8_t Partition_weight;

void thrd_partition_init(void);

#endif /* EMB6_INC_NET_RIP_THRD_PARTITION_H_ */
