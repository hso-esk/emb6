/**
 * @file    xmac_framer.c
 * @author  PN
 * @brief   XMAC framer for APSS
 */

/*
********************************************************************************
*                                  INCLUDES
********************************************************************************
*/
#include "emb6_conf.h"

#if (APSS_XMAC_EN == TRUE)
#include "lib_tmr.h"
#include "xmac_framer.h"


/*
********************************************************************************
*                               LOCAL DEFINES
********************************************************************************
*/
#define XMAC_STROBE_LEN                             (uint8_t) ( 3u )
#define XMAC_SACK_LEN                               (uint8_t) ( 3u )

/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static uint8_t XMAC_Strobe [XMAC_STROBE_LEN];
static uint8_t XMAC_SACK   [XMAC_SACK_LEN  ];

/*
********************************************************************************
*                       LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void XMAC_Init (STK_ERR *p_err);
static void XMAC_Deinit (STK_ERR *p_err);
static uint8_t* XMAC_CreateStrobe (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err);
static uint8_t* XMAC_CreateACK (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err);
static uint8_t* XMAC_CreateBroadcast (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err);
static uint8_t* XMAC_Create (uint8_t frame_type, uint16_t *p_len, uint32_t *p_delay, STK_ERR *p_err);
static void XMAC_Parse (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err);

static void XMAC_ParseStrobe (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err);
static void XMAC_ParseSACK (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err);

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
APSS_FRAMER_DRV XMACFramer = {
    "APSS XMAC",
     XMAC_Init,
     XMAC_Deinit,
     XMAC_Create,
     XMAC_Parse,
};


/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
static void XMAC_ParseStrobe (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err)
{
#if STKCFG_ARG_CHK_EN
    if (p_pkt == NULL) {
        *p_err = STK_ERR_NULL_POINTER;
        return;
    }
#endif


    STK_DEV_ID  dev_id;


    dev_id = (STK_DEV_ID) (p_pkt[0]     ) |
             (STK_DEV_ID) (p_pkt[1] << 8);
    if (dev_id == APSSSrcId) {
        *p_err = STK_ERR_NONE;
    } else {
        *p_err = STK_ERR_APSS_FRAME_INVALID;
    }
}


static void XMAC_ParseSACK (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err)
{
    STK_DEV_ID  src_id;


    src_id = (STK_DEV_ID) (p_pkt[0]     ) |
             (STK_DEV_ID) (p_pkt[1] << 8);
    if (src_id == APSSDstId) {
        *p_err = STK_ERR_NONE;

#if APSS_CFG_LOOSE_SYNC_EN
        uint8_t ix;

        for (ix = 0; ix < APSS_CFG_PWRON_TBL_SIZE; ix++) {
            if (APSSPwrOnTbl[ix].DestId == APSSDstId) {
                APSSPwrOnTbl[ix].LastWakeup    = TmrCurTick;
                APSSPwrOnTbl[ix].StrobeSentQty = 0; /* Reset sent strobe quantity */
                break;
            }
        }
#endif
    } else {
        *p_err = STK_ERR_APSS_ACK_INVALID;
    }
}



/*
********************************************************************************
*                           API FUNCTION DEFINITIONS
********************************************************************************
*/
static void XMAC_Init (STK_ERR *p_err)
{
    XMAC_SACK[0]    = APSS_FRAME_TYPE_SACK;
    XMAC_SACK[1]    = APSSSrcId;
    XMAC_SACK[2]    = APSSSrcId >> 8;
    XMAC_Strobe[0]  = APSS_FRAME_TYPE_STROBE;
    XMAC_Strobe[1]  = 0;
    XMAC_Strobe[2]  = 0;
    *p_err = STK_ERR_NONE;
}


static void XMAC_Deinit (STK_ERR *p_err)
{
    *p_err = STK_ERR_NONE;
}


/**
 * @brief   Create a waking-up strobe
 * @param   p_len
 * @param   p_delay
 * @param   p_err
 * @return
 */
static uint8_t* XMAC_CreateStrobe (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err)
{
    uint8_t *p_pkt;


#if STKCFG_ARG_CHK_EN
    if (p_len == NULL) {
        *p_err = STK_ERR_NULL_POINTER;
        return;
    }
#endif


    XMAC_Strobe[1] = APSSDstId;
    XMAC_Strobe[2] = APSSDstId >> 8;
    *p_delay = 0u;
    *p_err   = STK_ERR_NONE;
    *p_len   = sizeof (XMAC_Strobe);
     p_pkt   = XMAC_Strobe;


#if APSS_CFG_LOOSE_SYNC_EN
    uint8_t ix;
    uint8_t is_found = 0;
    uint8_t ix_free_slot = 0xFF;
    uint8_t n;
    uint32_t t0, t1;
    uint32_t tx_period;
    uint32_t tx_advance;
    uint32_t min_offset;

    for (ix = 0; ix < APSS_CFG_PWRON_TBL_SIZE; ix++) {
        if ((APSSPwrOnTbl[ix].DestId     == APSSDstId) &&
            (APSSPwrOnTbl[ix].LastWakeup != 0)) {
            is_found = 1;

            if (APSSPwrOnTbl[ix].StrobeSentQty == 0) {                          /* 1st Strobe in strobe stream              */
                t0 = APSSPwrOnTbl[ix].LastWakeup;                               /* Last power-on time interval              */
                n  = (TmrCurTick - t0) / APSS_CFG_POWERUP_INTERVAL_IN_MS;       /* Number of power-on interval in between   */
                t1 = t0 + (n + 1) * APSS_CFG_POWERUP_INTERVAL_IN_MS;            /* Predicted next power-on time interval    */

                *p_delay = (t1 - TmrCurTick);

                /*
                 * Note(s):
                 *
                 * Prediction on waking-up interval of the receiver should take
                 * into account minimum sum of Off-to-On and On-to-Off transition
                 * time.
                 */
                min_offset = APSS_PORT_ON_TO_OFF_TIME_IN_MS +
                             APSS_PORT_OFF_TO_ON_TIME_IN_MS;

                tx_advance = APSS_CFG_QTY_STROBE_SENT_IN_ADVANCE * APSS_PORT_STROBE_TX_PERIOD_IN_MS;

                if (*p_delay >= min_offset) {
                    *p_delay -= min_offset;

                    if (*p_delay > tx_advance) {
                        *p_delay -= tx_advance;
                    }
                } else {
                    *p_delay = 0;
                }
            }
            break;
        }

        if ((APSSPwrOnTbl[ix].DestId == 0) &&
            (ix_free_slot == 0xFF)) {
            ix_free_slot = ix;
        }
    }

    if (is_found == 0) {
        APSSPwrOnTbl[ix_free_slot].DestId = APSSDstId;
    }

    if (APSSPwrOnTbl[ix].StrobeSentQty++ > APSS_CFG_STROBE_TX_MAX) {
        APSSPwrOnTbl[ix].StrobeSentQty = 0;
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
static uint8_t* XMAC_CreateACK (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err)
{
    uint8_t *p_pkt;


#if STKCFG_ARG_CHK_EN
    if (p_len == NULL) {
        *p_err = STK_ERR_NULL_POINTER;
        return;
    }
#endif


    *p_delay = 0u;
    *p_err   = STK_ERR_NONE;
    *p_len   = sizeof (XMAC_SACK);
     p_pkt   = XMAC_SACK;
    return p_pkt;
}


/**
 * @brief   Create a broadcast waking-up strobe
 * @param   p_len
 * @param   p_delay
 * @param   p_err
 * @return
 */
static uint8_t* XMAC_CreateBroadcast (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err)
{
    uint8_t *p_pkt;


#if STKCFG_ARG_CHK_EN
    if (p_len == NULL) {
        *p_err = STK_ERR_NULL_POINTER;
        return;
    }
#endif


    XMAC_Strobe[0] = APSS_FRAME_TYPE_BROADCAST;
    XMAC_Strobe[1] = 0;
    XMAC_Strobe[2] = 0;
    *p_delay = 0u;
    *p_err   = STK_ERR_NONE;
    *p_len   = sizeof (XMAC_Strobe);
     p_pkt   = XMAC_Strobe;

     return p_pkt;
}

/**
 * @brief   Create a frame according to a given frame type
 *
 * @param   frame_type  Type of frame to create. This parameter is set to one of
 *                      following value:
 *                      @ref APSS_FRAME_TYPE_STROBE if a waking-up strobe is desired
 *                      @ref APSS_FRAME_TYPE_SACK if an ACK to waking-up strobe is desired
 *                      @ref APSS_FRAME_TYPE_BROADCAST if a broadcasting waking-up strobe is desired
 *
 * @param   p_len       Point to variable specifying length of frame to create
 * @param   p_delay     Point to variable specifying transmission delay
 * @param   p_err       Point to returned error code which is one of following value:
 *
 * @return  Pointer to buffer holding the created frame. If a NULL value is returned
 *          and type of frame is set to @ref APSS_FRAME_TYPE_BROADCAST, then
 *          the waking-up procedure is over and the module caller should start
 *          sending to broadcast frame(s)
 */
static uint8_t *XMAC_Create (uint8_t     frame_type,
                             uint16_t   *p_len,
                             uint32_t   *p_delay,
                             STK_ERR    *p_err)
{
#if STKCFG_ARG_CHK_EN
    if (p_len == NULL) {
        *p_err = STK_ERR_NULL_POINTER;
        return;
    }
#endif

    uint8_t *p_pkt = NULL;

    switch (frame_type) {
        case APSS_FRAME_TYPE_STROBE:
            p_pkt = XMAC_CreateStrobe(p_len, p_delay, p_err);
            break;

        case APSS_FRAME_TYPE_SACK:
            p_pkt = XMAC_CreateACK(p_len, p_delay, p_err);
            break;

        case APSS_FRAME_TYPE_BROADCAST:
            p_pkt = XMAC_CreateBroadcast(p_len, p_delay, p_err);
            break;

        default:
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
 *                  @ref STK_ERR_APSS_FRAME_INVALID if the frame is not of APSS type
 *                  @ref STK_ERR_NULL_POINTER if one of parameters is null
 *                  @ref STK_ERR_NONE if the frame is well-formated.
 */
static void XMAC_Parse (uint8_t     *p_pkt,
                        uint16_t     len,
                        STK_ERR     *p_err)
{
#if STKCFG_ARG_CHK_EN
    if (p_pkt == NULL) {
        *p_err = STK_ERR_NULL_POINTER;
        return;
    }
#endif

    switch (*p_pkt) {
        case APSS_FRAME_TYPE_STROBE:
            XMAC_ParseStrobe (++p_pkt, len, p_err);
            break;

        case APSS_FRAME_TYPE_SACK:
            XMAC_ParseSACK (++p_pkt, len, p_err);
            break;

        default:
            *p_err = STK_ERR_APSS_FRAME_INVALID;
            break;
    }
}
#endif /* #if (APSS_XMAC_EN == TRUE) */
