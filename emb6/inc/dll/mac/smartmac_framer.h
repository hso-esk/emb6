/*
 * smartmac_framer.h
 *
 *  Created on: Sep 10, 2015
 *      Author: phuongnguyen
 */

#ifndef SMARTMAC_FRAMER_PRESENT
#define SMARTMAC_FRAMER_PRESENT

#include "mac_ule.h"

/*
 * Note(s): COUNT_MAX configuration
 *
 * (1)  The smaller maximum value of timestamp count, the more reliable
 *      communication and the more accurate the waking-up prediction
 *
 * (2)  When broadcast strobes are transmitted, receiver nodes shall stay in RX
 *      mode a bit longer to minimize offsets due to sleep while waiting for
 *      broadcast payload
 */
#define SMARTMAC_CFG_COUNT_MAX                          (uint8_t ) ( 0u )


#endif /* SMARTMAC_FRAMER_PRESENT */
