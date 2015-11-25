/**
 * @file    smartmac_framer.c
 * @author  PN
 * @brief   Smart-MAC framer for APSS
 */

/*
********************************************************************************
*                                  INCLUDES
********************************************************************************
*/
#include "emb6.h"


#include "lib_tmr.h"
#include "smartmac_framer.h"

/*
********************************************************************************
*                               LOCAL DEFINES
********************************************************************************
*/
#define SMARTMAC_FRAME_LEN                              (uint8_t)( 5U )
#define SMARTMAC_CFG_REFACTOR_EN                        ( 0u )

/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static uint8_t SmartMAC_Strobe [SMARTMAC_FRAME_LEN];
static uint8_t SmartMAC_SACK   [SMARTMAC_FRAME_LEN];
static uint8_t SmartMAC_StrobeCnt;
static uint8_t SmartMAC_BroadcastCnt;
static uint8_t SmartMAC_RxBroadcast;
static LIB_TMR_TICK SmartMAC_SACKDelay;

/*
********************************************************************************
*                       LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void SmartMAC_Init (e_nsErr_t *p_err);
static void SmartMAC_Deinit (e_nsErr_t *p_err);
static uint8_t* SmartMAC_CreateStrobe (uint16_t *p_len, LIB_TMR_TICK *p_delay, e_nsErr_t *p_err);
static uint8_t* SmartMAC_CreateACK (uint16_t *p_len, LIB_TMR_TICK *p_delay, e_nsErr_t *p_err);
static uint8_t* SmartMAC_CreateBroadcast (uint16_t *p_len, LIB_TMR_TICK *p_delay, e_nsErr_t *p_err);
static uint8_t* SmartMAC_Create (uint8_t frame_type, uint16_t *p_len, uint32_t *p_delay, e_nsErr_t *p_err);
static void SmartMAC_Parse (uint8_t *p_frame_type, uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err);

static void SmartMAC_ParseStrobe (uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err);
static void SmartMAC_ParseSACK (uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err);
static void SmartMAC_ParseBroadcast (uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err);

static void SmartMAC_CalcSACKDelay(uint8_t ts_count);
static void SmartMAC_CalcStrobeDelay(s_nsMacUlePwrOnTblEntry_t *p_dev, uint32_t *p_delay);


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
s_nsMacUleFramerDrv_t SmartMACFramer = {
    "SmartMAC Framer",
     SmartMAC_Init,
     SmartMAC_Deinit,
     SmartMAC_Create,
     SmartMAC_Parse,
};



/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
/**
 * @brief   Calculate a time elapse before starting waking-up process
 *
 * @param   p_dev   Pointer to a structure holding power-on information of a
 *                  device
 */
static void SmartMAC_CalcStrobeDelay(s_nsMacUlePwrOnTblEntry_t *p_dev, uint32_t *p_delay)
{
    uint8_t n, i;
    uint32_t t0, t1;
    uint32_t tx_period;
    uint32_t tx_advance;
    uint32_t min_delay;

    if (p_dev->StrobeSentQty == 0) {                                    /* 1st Strobe in strobe stream              */
        t0 = p_dev->LastWakeup;                                         /* Last power-on time interval              */

        /*
         * Perform prediction on waking-up interval of the intended receiver
         */
        i = 1;
        do {
            n  = (TmrCurTick - t0) / MAC_ULE_CFG_POWERUP_INTERVAL_IN_MS;        /* Number of power-on interval in between   */
            t1 = t0 + (n + i) * MAC_ULE_CFG_POWERUP_INTERVAL_IN_MS;             /* Predicted next power-on time interval    */
            *p_delay = (t1 - TmrCurTick);

            /*
             * Note(s):
             *
             * (1)  Minimum possible delay shall be calculated as following:
             *      min_delay = T_off_to_on + T_on_to_off + T_scan + T_tx_smartpreamble
             */
            tx_period = MAC_ULE_PORT_STROBE_TX_TIME_IN_MS +
                        MAC_ULE_PORT_STROBE_TX_GAP_TIME_IN_MS;

            tx_advance = MAC_ULE_CFG_QTY_STROBE_SENT_IN_ADVANCE * tx_period;

            min_delay = MAC_ULE_PORT_OFF_TO_ON_TIME_IN_MS +
                        MAC_ULE_PORT_ON_TO_OFF_TIME_IN_MS +
                        MAC_ULE_PORT_SCAN_DURATION_IN_MS;

            if (*p_delay < min_delay) {
                *p_delay = 0;
            } else {
                *p_delay -= min_delay;
                if (*p_delay > tx_advance) {
                    *p_delay -= tx_advance;
                }
            }
            i++;
        } while (*p_delay == 0);

        /* set strobe counter to default */
        SmartMAC_StrobeCnt = MAC_ULE_CFG_STROBE_TX_MAX;
    } else {
        *p_delay = 0;
    }
}


/**
 * @brief   Calculate a time elapse before replying with an ACK
 *
 * @param   ts_count    Time stamp count value
 */
static void SmartMAC_CalcSACKDelay(uint8_t ts_count)
{
    uint32_t    min_delay;


    if (ts_count) {
        /*
         * Note:
         *
         * If calculated waiting time before sending an ACK is smaller than
         * or equal to minimum offset in the receiver caused by off-to-on
         * and on-to-off transition time, there is no need to go back to
         * sleep. Instead the receive should replay with an ACK immediately
         * as in SmartMAC framer.
         */
        SmartMAC_SACKDelay = ts_count * MAC_ULE_PORT_STROBE_TX_INTERVAL_IN_MS;
        min_delay = MAC_ULE_PORT_ON_TO_OFF_TIME_IN_MS +
                    MAC_ULE_PORT_OFF_TO_ON_TIME_IN_MS;
        if (SmartMAC_SACKDelay <= min_delay) {
            SmartMAC_SACKDelay = 0;
        } else {
            SmartMAC_SACKDelay -= min_delay;
        }
    } else {
        SmartMAC_SACKDelay = 0;
    }
}


/**
 * @brief   Parse a waking-up strobe payload
 *
 * @param   p_pkt   Pointer to buffer holding strobe payload
 * @param   len     Length of the strobe payload
 * @param   p_err   Pointer to variable that stores the return error code
 */
static void SmartMAC_ParseStrobe (uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_pkt == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }
#endif


    NETSTK_DEV_ID  dev_id;
    uint8_t     ts_count;


    dev_id = (NETSTK_DEV_ID)(p_pkt[3]     ) |
             (NETSTK_DEV_ID)(p_pkt[4] << 8);
    if (dev_id == NetstkSrcId) {
        *p_err = NETSTK_ERR_NONE;
        ts_count = p_pkt[2];
        SmartMAC_CalcSACKDelay(ts_count);

        /* set sequence number of the corresponding ACK equal to that in the
         * received strobe. This value is then used to improve the prediction
         * mechanism */
        SmartMAC_SACK[2] = ts_count;
        return;
    }

    /*
     * If the received strobe is not destined for us...
     */
    if (MAC_ULE_IS_PENDING_TX() == 0) {
        *p_err = NETSTK_ERR_LPR_INVALID_ADDR;
    } else {
        if (dev_id == NetstkDstId) {
            *p_err = NETSTK_ERR_LPR_TX_COLLISION_SAME_DEST;
        } else {
            *p_err = NETSTK_ERR_LPR_TX_COLLISION_DIFF_DEST;
        }
    }
}


/**
 * @brief   Parse a strobe ACK
 *
 * @param   p_pkt   Pointer to buffer holding ACK payload
 * @param   len     Length of the ACK payload
 * @param   p_err   Pointer to variable that stores the return error code
 */
static void SmartMAC_ParseSACK (uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err)
{
    NETSTK_DEV_ID  src_id;
    uint8_t is_valid_ack;


    src_id = (NETSTK_DEV_ID)(p_pkt[3]     ) |
             (NETSTK_DEV_ID)(p_pkt[4] << 8);

    /* valid ACK shall match device ID as well as counter value */
    is_valid_ack = (src_id   == NetstkDstId) &&
                   (p_pkt[2] == (SmartMAC_StrobeCnt + 1));
    if (is_valid_ack) {
        *p_err = NETSTK_ERR_NONE;

#if MAC_ULE_CFG_LOOSE_SYNC_EN
        uint8_t ix;

        for (ix = 0; ix < MAC_ULE_CFG_PWRON_TBL_SIZE; ix++) {
            if (MacUlePwrOnTbl[ix].DestId == NetstkDstId) {
                MacUlePwrOnTbl[ix].LastWakeup = TmrCurTick;
                MacUlePwrOnTbl[ix].StrobeSentQty = 0; /* Reset sent strobe quantity */
                break;
            }
        }
    } else {
        *p_err = NETSTK_ERR_LPR_INVALID_ACK;
    }
#endif
}


/**
 * @brief   Parse a waking-up broadcast strobe
 *
 * @param   p_pkt   Pointer to buffer holding strobe payload
 * @param   len     Length of the strobe payload
 * @param   p_err   Pointer to variable that stores the return error code
 */
static void SmartMAC_ParseBroadcast (uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_pkt == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }
#endif


    uint8_t     ts_count;


    ts_count = p_pkt[2];
    SmartMAC_CalcSACKDelay(ts_count);
    *p_err = NETSTK_ERR_NONE;
}


/*
********************************************************************************
*                           API FUNCTION DEFINITIONS
********************************************************************************
*/

/**
 * @brief   Initialize Smart-MAC framer module
 *
 * @param   p_err   Pointer to a variable that stores a return error code
 */
static void SmartMAC_Init (e_nsErr_t *p_err)
{
    SmartMAC_SACK[0] = MAC_ULE_FRAME_TYPE_SACK;
    SmartMAC_SACK[1] = 0;
    SmartMAC_SACK[2] = 0;
    SmartMAC_SACK[3] = NetstkSrcId;
    SmartMAC_SACK[4] = NetstkSrcId >> 8;

    SmartMAC_Strobe[0] = 0;
    SmartMAC_Strobe[1] = 0;
    SmartMAC_Strobe[2] = 0;
    SmartMAC_Strobe[3] = 0;
    SmartMAC_Strobe[3] = 0;

    SmartMAC_SACKDelay = 0;
    SmartMAC_StrobeCnt = MAC_ULE_CFG_STROBE_TX_MAX;
    SmartMAC_RxBroadcast = 0;
    SmartMAC_BroadcastCnt = MAC_ULE_CFG_BROADCAST_TX_MAX;
    *p_err = NETSTK_ERR_NONE;
}


/**
 * @brief   De-Initialize Smart-MAC framer module
 *
 * @param   p_err   Pointer to a variable that stores a return error code
 */
static void SmartMAC_Deinit (e_nsErr_t *p_err)
{
    *p_err = NETSTK_ERR_NONE;
}


/**
 * @brief   Create a waking-up strobe
 * @param   p_len
 * @param   p_delay
 * @param   p_err
 * @return
 */
static uint8_t* SmartMAC_CreateStrobe (uint16_t *p_len, LIB_TMR_TICK *p_delay, e_nsErr_t *p_err)
{
    uint8_t *p_pkt;

    SmartMAC_Strobe[0] = MAC_ULE_FRAME_TYPE_STROBE;
    SmartMAC_Strobe[1] = 0;
    SmartMAC_Strobe[2] = SmartMAC_StrobeCnt;
    SmartMAC_Strobe[3] = NetstkDstId;
    SmartMAC_Strobe[4] = NetstkDstId >> 8;

    *p_delay = 0u;
    *p_err   = NETSTK_ERR_NONE;
    *p_len   = sizeof(SmartMAC_Strobe);
     p_pkt   = SmartMAC_Strobe;

    if (SmartMAC_StrobeCnt) {
        SmartMAC_StrobeCnt--;
    } else {
        SmartMAC_StrobeCnt = MAC_ULE_CFG_STROBE_TX_MAX;
    }


#if MAC_ULE_CFG_LOOSE_SYNC_EN
    uint8_t ix;
    uint8_t is_found = 0;
    uint8_t ix_free_slot = 0xFF;

    for (ix = 0; ix < MAC_ULE_CFG_PWRON_TBL_SIZE; ix++) {
        if ((MacUlePwrOnTbl[ix].DestId     == NetstkDstId) &&
            (MacUlePwrOnTbl[ix].LastWakeup != 0)) {
            is_found = 1;
            SmartMAC_CalcStrobeDelay(&MacUlePwrOnTbl[ix], p_delay);
            break;
        }

        if ((MacUlePwrOnTbl[ix].DestId == 0) &&
            (ix_free_slot == 0xFF)) {
            ix_free_slot = ix;
        }
    }

    if (is_found == 0) {
        MacUlePwrOnTbl[ix_free_slot].DestId = NetstkDstId;
    }

    if (MacUlePwrOnTbl[ix].StrobeSentQty > MAC_ULE_CFG_STROBE_TX_MAX) {
        MacUlePwrOnTbl[ix].StrobeSentQty = 0;
    } else {
        MacUlePwrOnTbl[ix].StrobeSentQty++;
    }
#endif

    return p_pkt;
}


/**
 * @brief   Create an ACK to a waking-up strobe. This frame is always the same
 *          regardless of originator of the waking-up strobe
 *
 * @param   p_len
 * @param   p_delay
 * @param   p_err
 * @return
 */
static uint8_t *SmartMAC_CreateACK (uint16_t *p_len, LIB_TMR_TICK *p_delay, e_nsErr_t *p_err)
{
    uint8_t *p_pkt;


    if (SmartMAC_RxBroadcast == 1) {
        SmartMAC_RxBroadcast = 0;
        p_pkt  = NULL;
        *p_err = NETSTK_ERR_LPR_BROADCAST_NOACK;
    } else {
        p_pkt  = SmartMAC_SACK;
        *p_len = sizeof (SmartMAC_SACK);
        *p_err = NETSTK_ERR_NONE;
    }

    *p_delay = SmartMAC_SACKDelay;
    SmartMAC_SACKDelay = 0;             /* Reset SACK TX Delay */

    return p_pkt;
}


/**
 * @brief   Create a broadcast waking-up strobe
 * @param   p_len
 * @param   p_delay
 * @param   p_err
 * @return
 */
static uint8_t* SmartMAC_CreateBroadcast (uint16_t *p_len, LIB_TMR_TICK *p_delay, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_len == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return NULL;
    }
#endif


    uint8_t *p_pkt;

    SmartMAC_Strobe[0] = MAC_ULE_FRAME_TYPE_BROADCAST;
    SmartMAC_Strobe[1] = 0;
    SmartMAC_Strobe[2] = SmartMAC_BroadcastCnt;
    SmartMAC_Strobe[3] = 0xFF;
    SmartMAC_Strobe[4] = 0xFF;
    *p_delay = 0u;
    *p_err   = NETSTK_ERR_NONE;
    *p_len   = sizeof(SmartMAC_Strobe);
    p_pkt    = SmartMAC_Strobe;


    if (SmartMAC_BroadcastCnt) {
        SmartMAC_BroadcastCnt--;
    } else {
        SmartMAC_BroadcastCnt = MAC_ULE_CFG_BROADCAST_TX_MAX;
        *p_err = NETSTK_ERR_LPR_BROADCAST_LAST_STROBE;
    }
    return p_pkt;
}


/**
 * @brief   Create a frame according to a given frame type
 *
 * @param   frame_type  Type of frame to create. This parameter is set to one of
 *                      following value:
 *                      @ref LPR_FRAME_TYPE_STROBE if a waking-up strobe is desired
 *                      @ref LPR_FRAME_TYPE_SACK if an ACK to waking-up strobe is desired
 *                      @ref LPR_FRAME_TYPE_BROADCAST if a broadcasting waking-up strobe is desired
 *
 * @param   p_len       Point to variable specifying length of frame to create
 * @param   p_delay     Point to variable specifying transmission delay
 * @param   p_err       Point to returned error code which is one of following value:
 *
 * @return  Pointer to buffer holding the created frame. If a NULL value is returned
 *          and type of frame is set to @ref LPR_FRAME_TYPE_BROADCAST, then
 *          the waking-up procedure is over and the module caller should start
 *          sending to broadcast frame(s)
 */
static uint8_t *SmartMAC_Create (uint8_t frame_type, uint16_t *p_len, uint32_t *p_delay, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_len == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return NULL;
    }
#endif


    uint8_t *p_pkt;


    switch (frame_type) {
        case MAC_ULE_FRAME_TYPE_STROBE:
            p_pkt = SmartMAC_CreateStrobe(p_len, p_delay, p_err);
            break;

        case MAC_ULE_FRAME_TYPE_SACK:
            p_pkt = SmartMAC_CreateACK(p_len, p_delay, p_err);
            break;

        case MAC_ULE_FRAME_TYPE_BROADCAST:
            p_pkt = SmartMAC_CreateBroadcast(p_len, p_delay, p_err);
            break;

        default:
            p_pkt = NULL;
            break;
    }
    return p_pkt;
}


/**
 * @brief   Parse a frame
 *
 * @param   p_pkt   Point to the frame to parse
 * @param   len     Length of the frame to parse
 * @param   p_err   Point to returned error code which is one following values:
 *                  @ref NETSTK_ERR_LPR_UNSUPPORTED_FRAME if the frame is not of APSS type
 *                  @ref NETSTK_ERR_NULL_POINTER if one of parameters is null
 *                  @ref NETSTK_ERR_NONE if the frame is well-formated.
 */
static void SmartMAC_Parse (uint8_t *p_frame_type, uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_pkt == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }
#endif

    *p_frame_type = *p_pkt;
    switch (*p_frame_type) {
        case MAC_ULE_FRAME_TYPE_STROBE:
            SmartMAC_ParseStrobe(p_pkt, len, p_err);
            break;

        case MAC_ULE_FRAME_TYPE_SACK:
            SmartMAC_ParseSACK(p_pkt, len, p_err);
            break;

        case MAC_ULE_FRAME_TYPE_BROADCAST:
            SmartMAC_RxBroadcast = 1;
            SmartMAC_ParseBroadcast(p_pkt, len, p_err);
            break;

        default:
            *p_err = NETSTK_ERR_LPR_UNSUPPORTED_FRAME;
            break;
    }
}


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
