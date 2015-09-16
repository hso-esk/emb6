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
#include <stdint.h>
#include <stddef.h>

#include "lib_tmr.h"
#include "include.h"
#include "smartmac_framer.h"


/*
********************************************************************************
*                               LOCAL DEFINES
********************************************************************************
*/
#define SmartMAC_FRAME_STROBE_LEN                           (uint8_t) ( 4u )
#define SmartMAC_FRAME_SACK_LEN                             (uint8_t) ( 3u )


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static uint8_t SmartMAC_Strobe [SmartMAC_FRAME_STROBE_LEN];
static uint8_t SmartMAC_SACK   [SmartMAC_FRAME_SACK_LEN  ];

static uint8_t SmartMAC_StrobeCount;
static LIB_TMR_TICK SmartMAC_SACKDelay;

/*
********************************************************************************
*                       LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void SmartMAC_Init (STK_ERR *p_err);
static void SmartMAC_Deinit (STK_ERR *p_err);
static uint8_t* SmartMAC_CreateWakeup (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err);
static uint8_t* SmartMAC_CreateACK (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err);
static void SmartMAC_Parse (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err);

static void SmartMAC_ParseStrobe (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err);
static void SmartMAC_ParseSACK (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err);

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
APSS_FRAMER_API SmartMACFramer = {
    "APSS SmartMAC Framer",
     SmartMAC_Init,
     SmartMAC_Deinit,
     SmartMAC_CreateWakeup,
     SmartMAC_CreateACK,
     SmartMAC_Parse,
};




/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
static void SmartMAC_ParseStrobe (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err)
{
#if STKCFG_ARG_CHK_EN
    if (p_pkt == NULL) {
        *p_err = STK_ERR_NULL_POINTER;
        return;
    }
#endif


    STK_DEV_ID  dev_id;
    uint8_t     ts_count;

    dev_id = (STK_DEV_ID) (p_pkt[0]     ) |
             (STK_DEV_ID) (p_pkt[1] << 8);
    if (dev_id == APSSDevId) {
        ts_count = p_pkt[2];
        if (ts_count) {
            SmartMAC_SACKDelay  = SMARTMAC_STROBE_TX_TIME_IN_MS +
                                  SMARTMAC_STROBE_TX_TIME_GAP_IN_MS;
            SmartMAC_SACKDelay *= ts_count;
            SmartMAC_SACKDelay += SMARTMAC_TX_RX_TURNAROUND_IN_MS;
        } else {
            SmartMAC_SACKDelay  = 0;
        }
        *p_err = STK_ERR_NONE;
    } else {
        *p_err = STK_ERR_APSS_FRAME_INVALID;
    }
}


static void SmartMAC_ParseSACK (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err)
{
    STK_DEV_ID  src_id;


    src_id = (STK_DEV_ID) (p_pkt[0]     ) |
             (STK_DEV_ID) (p_pkt[1] << 8);
    if (src_id == APSSDestId) {
        *p_err = STK_ERR_NONE;
    } else {
        *p_err = STK_ERR_APSS_ACK_INVALID;
    }
}



/*
********************************************************************************
*                           API FUNCTION DEFINITIONS
********************************************************************************
*/
static void SmartMAC_Init (STK_ERR *p_err)
{
    SmartMAC_SACK[0]     = APSS_FRAME_TYPE_SACK;
    SmartMAC_SACK[1]     = APSSDevId;
    SmartMAC_SACK[2]     = APSSDevId >> 8;
    SmartMAC_Strobe[0]   = APSS_FRAME_TYPE_STROBE;
    SmartMAC_Strobe[1]   = 0;
    SmartMAC_Strobe[2]   = 0;
    SmartMAC_Strobe[3]   = 0;
    SmartMAC_StrobeCount = SMARTMAC_COUNT_MAX;
    SmartMAC_SACKDelay   = 0;

    *p_err = STK_ERR_NONE;
}


static void SmartMAC_Deinit (STK_ERR *p_err)
{
    *p_err = STK_ERR_NONE;
}


static uint8_t* SmartMAC_CreateWakeup (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err)
{
    uint8_t *p_pkt;


    SmartMAC_Strobe[1] = APSSDestId;
    SmartMAC_Strobe[2] = APSSDestId >> 8;
    SmartMAC_Strobe[3] = SmartMAC_StrobeCount;
    *p_delay = 0u;
    *p_err   = STK_ERR_NONE;
    *p_len   = sizeof (SmartMAC_Strobe);
     p_pkt   = SmartMAC_Strobe;


    if (SmartMAC_StrobeCount) {
        SmartMAC_StrobeCount--;
    } else {
        SmartMAC_StrobeCount = SMARTMAC_COUNT_MAX;
    }

    return p_pkt;
}



static uint8_t* SmartMAC_CreateACK (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err)
{
    uint8_t *p_pkt;


    p_pkt    = SmartMAC_SACK;
    *p_len   = sizeof (SmartMAC_SACK);
    *p_delay = SmartMAC_SACKDelay;
    *p_err   = STK_ERR_NONE;

    SmartMAC_SACKDelay = 0;     /* Reset SACK TX Delay */

    return p_pkt;
}


static void SmartMAC_Parse (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err)
{
#if STKCFG_ARG_CHK_EN
    if (p_pkt == NULL) {
        *p_err = STK_ERR_NULL_POINTER;
        return;
    }
#endif

    switch (*p_pkt) {
        case APSS_FRAME_TYPE_STROBE:
            SmartMAC_ParseStrobe (++p_pkt, len, p_err);
            break;

        case APSS_FRAME_TYPE_SACK:
            SmartMAC_ParseSACK (++p_pkt, len, p_err);
            break;

        default:
            *p_err = STK_ERR_APSS_FRAME_INVALID;
            break;
    }
}
