/*
 * apss_framer_xmac.h
 *
 *  Created on: Sep 9, 2015
 *      Author: phuongnguyen
 */

#ifndef XMAC_FRAMER_PRESENT
#define XMAC_FRAMER_PRESENT



#define XMAC_FRAME_STROBE_LEN                           (uint8_t) ( 3u )
#define XMAC_FRAME_SACK_LEN                             (uint8_t) ( 3u )
#define XMAC_FRAME_LOOSE_SYNC_EN                        ( 1u )

#if XMAC_FRAME_LOOSE_SYNC_EN
#define XMAC_WAKEUP_TABLE_SIZE                          (uint8_t ) ( 3u )
#define XMAC_WAKEUP_EARLY_IN_TICKS                      (LIB_TMR_TICK) ( 45u )

typedef struct xmac_wakeup_table    XMAC_WAKEUP_TABLE;

struct xmac_wakeup_table
{
    uint16_t        DestId;
    LIB_TMR_TICK    LastWakeup;
    LIB_TMR_TICK    Delay;
};
#endif

#endif /* XMAC_FRAMER_PRESENT */
