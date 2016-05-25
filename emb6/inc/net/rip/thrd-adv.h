/*
 * thrd-adv.h
 *
 *  Created on: 10 May 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  MLE Advertisement Processing / Route64 TLV Generation.
 */

#ifndef EMB6_INC_NET_RIP_THRD_ADV_H_
#define EMB6_INC_NET_RIP_THRD_ADV_H_

#include "tlv.h"

// TODO Check!
#define MAX_ROUTE64_TLV_DATA_SIZE		41		// One plus ceiling (MAX_ROUTER_ID/8) plus the number of assigned router IDs.

/*==============================================================================
                                     API FUNCTIONS
 =============================================================================*/

void thrd_process_adv(uint16_t source_addr, tlv_route64_t *route64_tlv, tlv_leader_t *leader_tlv);

tlv_t *thrd_generate_route64();

#endif /* EMB6_INC_NET_RIP_THRD_ADV_H_ */
