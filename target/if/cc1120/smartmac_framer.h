/*
 * smartmac_framer.h
 *
 *  Created on: Sep 10, 2015
 *      Author: phuongnguyen
 */

#ifndef SMARTMAC_FRAMER_PRESENT
#define SMARTMAC_FRAMER_PRESENT

#define SMARTMAC_COUNT_MAX                              (uint8_t) ( 5u )
#define SMARTMAC_STROBE_LEN                             (uint8_t) ( 4u )
#define SMARTMAC_SACK_LEN                               (uint8_t) ( 3u )

#define SMARTMAC_TX_RX_TURNAROUND_IN_MS                 (LIB_TMR_TICK) ( 0u )
#define SMARTMAC_STROBE_TX_TIME_IN_MS                   (LIB_TMR_TICK) ( 10u )
#define SMARTMAC_STROBE_TX_TIME_GAP_IN_MS               (LIB_TMR_TICK) APSS_TMR_WFA_TIMEOUT

#endif /* SMARTMAC_FRAMER_PRESENT */
