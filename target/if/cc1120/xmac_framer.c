/*
********************************************************************************
*                                  INCLUDES
********************************************************************************
*/
#include <stdint.h>
#include <stddef.h>

#include "lib_tmr.h"

#include "include.h"
#include "xmac_framer.h"


/*
********************************************************************************
*                               LOCAL DEFINES
********************************************************************************
*/



/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static uint8_t XMAC_Strobe [XMAC_FRAME_STROBE_LEN];
static uint8_t XMAC_SACK   [XMAC_FRAME_SACK_LEN  ];

#if XMAC_FRAME_LOOSE_SYNC_EN
static XMAC_WAKEUP_TABLE    XMAC_WakeupTable[XMAC_WAKEUP_TABLE_SIZE];
#endif

/*
********************************************************************************
*                       LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void XMAC_Init (STK_ERR *p_err);
static void XMAC_Deinit (STK_ERR *p_err);
static uint8_t* XMAC_CreateWakeup (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err);
static uint8_t* XMAC_CreateACK (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err);
static void XMAC_Parse (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err);

static void XMAC_ParseStrobe (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err);
static void XMAC_ParseSACK (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err);

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
APSS_FRAMER_API XMACFramer = {
    "APSS XMAC Framer",
     XMAC_Init,
     XMAC_Deinit,
     XMAC_CreateWakeup,
     XMAC_CreateACK,
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
    if (dev_id == APSSDevId) {
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
    if (src_id == APSSDestId) {
        *p_err = STK_ERR_NONE;

#if XMAC_FRAME_LOOSE_SYNC_EN
        uint8_t ix;

        for (ix = 0; ix < XMAC_WAKEUP_TABLE_SIZE; ix++) {
            if (XMAC_WakeupTable[ix].DestId == APSSDestId) {
                XMAC_WakeupTable[ix].Delay = 0;
                XMAC_WakeupTable[ix].LastWakeup = TmrCurTick;
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
    XMAC_SACK[1]    = APSSDevId;
    XMAC_SACK[2]    = APSSDevId >> 8;
    XMAC_Strobe[0]  = APSS_FRAME_TYPE_STROBE;
    XMAC_Strobe[1]  = 0;
    XMAC_Strobe[2]  = 0;

#if XMAC_FRAME_LOOSE_SYNC_EN
    memset(XMAC_WakeupTable, 0, sizeof(XMAC_WakeupTable));
#endif

    *p_err = STK_ERR_NONE;
}


static void XMAC_Deinit (STK_ERR *p_err)
{
    *p_err = STK_ERR_NONE;
}


static uint8_t* XMAC_CreateWakeup (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err)
{
    uint8_t *p_pkt;


    XMAC_Strobe[1] = APSSDestId;
    XMAC_Strobe[2] = APSSDestId >> 8;
    *p_delay = 0u;
    *p_err   = STK_ERR_NONE;
    *p_len   = sizeof (XMAC_Strobe);
     p_pkt   = XMAC_Strobe;


#if XMAC_FRAME_LOOSE_SYNC_EN
    uint8_t ix;
    uint8_t is_found = 0;
    uint8_t ix_free_slot = 0xFF;

    for (ix = 0; ix < XMAC_WAKEUP_TABLE_SIZE; ix++) {
        if ((XMAC_WakeupTable[ix].DestId     == APSSDestId) &&
            (XMAC_WakeupTable[ix].LastWakeup != 0)) {
            is_found = 1;

            if (XMAC_WakeupTable[ix].Delay)  {
                XMAC_WakeupTable[ix].Delay = 0;
            } else {
                uint8_t n;
                LIB_TMR_TICK t0, t1;

                t0 = XMAC_WakeupTable[ix].LastWakeup;
                n  = (TmrCurTick - t0) / APSS_TMR_POWERUP_INTERVAL;
                t1 = t0 + (n + 1) * APSS_TMR_POWERUP_INTERVAL;
                XMAC_WakeupTable[ix].Delay = (t1 - TmrCurTick) - XMAC_WAKEUP_EARLY_IN_TICKS;
            }
            break;
        }

        if ((XMAC_WakeupTable[ix].DestId == 0) &&
            (ix_free_slot == 0xFF)) {
            ix_free_slot = ix;
        }
    }

    if (is_found == 0) {
        XMAC_WakeupTable[ix_free_slot].DestId = APSSDestId;
    } else {
        *p_delay = XMAC_WakeupTable[ix].Delay;
    }
#endif

    return p_pkt;
}


static uint8_t* XMAC_CreateACK (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err)
{
    uint8_t *p_pkt;


    *p_delay = 0u;
    *p_err   = STK_ERR_NONE;
    *p_len   = sizeof (XMAC_SACK);
     p_pkt   = XMAC_SACK;
    return p_pkt;
}


static void XMAC_Parse (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err)
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
