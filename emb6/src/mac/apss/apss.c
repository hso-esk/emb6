/**
 * @file    apss.c
 * @author  PN
 * @brief   Asynchronous Power Saving Scheme module
 */

#include "emb6_conf.h"

#if EMB6_CFG_APSS_EN == TRUE
#include "evproc.h"
#include "packetbuf.h"
#include "bsp.h"

#include "lib_tmr.h"
#include "apss.h"

#if 1   // only used for displaying LCD
#include "demo-apss.h"
#endif

#ifdef DEMO_APSS_PRESENT
#define LCD_UPDATE()                App_LCDUpdate()
#else
#define LCD_UPDATE()
#endif

/*
********************************************************************************
*                               LOCAL DEFINES
********************************************************************************
*/
#define APSS_CFG_PKT_RX_QTY_IN_IDLE             (uint8_t)( 0u )

#define APSS_WFSE_COUNT_MAX                     (uint8_t)( 2u )
#define APSS_WFAE_COUNT_MAX                     (uint8_t)( 2u )
#define APSS_WFPE_COUNT_MAX                     (uint8_t)( 3u )

#define APSS_TASK_EVENT                         (c_event_t) ( 1u )

#if APSS_CFG_CCA_BASED_SCAN_EN
#define APSS_CCA_COUNT_MAX              (uint8_t) ( 3u )
#endif


/**
 * @addtogroup  APSS_STATES     APSS states
 * @{
 */
typedef enum
{
    /*
     * Power-related states
     */
    APSS_STATE_NONE             =  0u,      /*!< Initial state                                                  */
    APSS_STATE_POWER_DOWN       =  1u,      /*!< Power-Down state                                               */
    APSS_STATE_IDLE             =  2u,      /*!< Idle state                                                     */
    APSS_STATE_SLEEP            =  3u,      /*!< Sleep state                                                    */

    /*
     * Scan procedure related states
     */
    APSS_STATE_SCAN_STARTED     = 10u,      /*!< Scan process has started                                       */
#if APSS_CFG_CCA_BASED_SCAN_EN
    APSS_STATE_SCAN_CCA         = 11u,      /*!< Performing CCA                                                 */
    APSS_STATE_SCAN_CCA_SLEEP   = 12u,      /*!< Sleep between successive CCAs                                  */
#endif
    APSS_STATE_SCAN_WFS         = 13u,      /*!< Wait For waking-up strobe                                      */
    APSS_STATE_SCAN_WFSE        = 14u,      /*!< Wait-For-Strobe Extension                                      */
    APSS_STATE_SCAN_DONE        = 15u,      /*!< Scan process has finished                                      */

    /*
     * Transmission-related states
     */
    APSS_STATE_TX_STARTED       = 20u,      /*!< Waking-up-Strobe Transmission                                  */
    APSS_STATE_TX_S             = 21u,      /*!< Waking-up-Strobe Transmission                                  */
    APSS_STATE_TX_SD            = 22u,      /*!< Waking-up Strobe Transmission Delayed                          */
    APSS_STATE_TX_SWFA          = 23u,      /*!< Wait For Strobe ACK                                            */
    APSS_STATE_TX_SWFAE         = 24u,      /*!< Wait For strobe ACK Extension                                  */
    APSS_STATE_TX_P             = 25u,      /*!< Data Payload Transmission                                      */
    APSS_STATE_TX_PWFA          = 26u,      /*!< Wait For Payload ACK                                           */
    APSS_STATE_TX_DONE          = 27u,      /*!< Data transmission process has finished                         */

#if APSS_CFG_REFACTOR_EN
    APSS_STATE_TX_LBT           = 50u,      /*!< Listen-Before-Talk     */
    APSS_STATE_TX_LBT_WFA       = 51u,      /*!< Wait-For-ACK, after a waking-up strobe with same destination ID was received           */
    APSS_STATE_TX_LBT_WFI       = 52u,      /*!< Wait-For-Idle, until the destination node has completed data exchange with other nodes */
#endif


    /*
     * Reception-related states
     */
    APSS_STATE_TX_SACK          = 30u,      /*!< Received a valid Strobe and reply with an ACK immediately      */
    APSS_STATE_TX_SACKD         = 31u,      /*!< Received a valid Strobe and reply with an ACK after a Delay    */
    APSS_STATE_RX_WFP           = 32u,      /*!< Wait-For-Payload                                               */
    APSS_STATE_RX_WFPE          = 33u,      /*!< Wait-For-Payload-Extension                                     */
    APSS_STATE_RX_P             = 34u,      /*!< Receiving-Payload                                              */


    /*
     * Broadcast-related states
     */
    APSS_STATE_TX_BS            = 40u,      /*!< Broadcasting Waking-up-Strobe Transmission                     */
    APSS_STATE_TX_B             = 41u,      /*!< Broadcast Transmission                                         */
    APSS_STATE_RX_BSD           = 42u,      /*!< Broadcast Reception with delay                                 */
    APSS_STATE_RX_B             = 43u,      /*!< Broadcast Reception with delay                                 */

}APSS_STATE;
/**
 * @}
 */


typedef struct stk_apss STK_APSS;

struct stk_apss
{
    LIB_TMR             TmrPowerup;
    LIB_TMR             Tmr1W;
    LIB_TMR             Tmr1WBS;
    LIB_TMR             Tmr1Scan;
    LIB_TMR             Tmr1TxStrobe;
    LIB_TMR             Tmr1Idle;

    uint8_t             WFSEQty;
    uint8_t             WFAEQty;
    uint8_t             WFPEQty;
    uint8_t             IdleQty;

    s_ns_t             *Netstack;
    APSS_FRAMER_DRV    *Framer;
    APSS_STATE          State;
    APSS_STATE          BroadcastState;
    STK_ERR             LastErrTx;

    mac_callback_t      TxCbFnct;
    void               *TxCbArg;
    uint8_t            *TxPktPtr;
    uint16_t            TxPktLen;

#if APSS_CFG_LOOSE_SYNC_EN
    uint8_t             IsTxDelayed;
#endif

#if APSS_CFG_CCA_BASED_SCAN_EN
    uint8_t             CCA_Qty;
#endif
};


/*
********************************************************************************
*                               LOCAL MACROS
********************************************************************************
*/
#define APSS_SEM_POST(_event_)          evproc_putEvent(E_EVPROC_HEAD, _event_, &APSS_Ctx)
#define APSS_SEM_WAIT(_event_)          evproc_regCallback(_event_, APSS_Task)


/*
********************************************************************************
*                       LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void APSS_Init(s_ns_t *p_netstack);
static void APSS_Send(mac_callback_t cbsent, void *p_arg);
static void APSS_SendList(mac_callback_t cbsent, void *p_arg, s_nsLmacBufList_t *p_list);
static void APSS_Input(void);
static int8_t APSS_On(void);
static int8_t APSS_Off(int keep_radio_on);
static unsigned short APSS_ChanChkInterval(void);

static void APSS_Task(c_event_t c_event, p_data_t p_data);
static void APSS_TmrIsrPowerUp(void *p_arg);
static void APSS_TmrIsr1Scan(void *p_arg);
static void APSS_TmrIsr1WFP(void *p_arg);
static void APSS_TmrIsr1TxStrobe(void *p_arg);
static void APSS_TmrIsr1TxSACKD(void *p_arg);
static void APSS_TmrIsr1WFA(void *p_arg);
static void APSS_Tmr1Start(LIB_TMR *p_tmr, LIB_TMR_TICK timeout, FNCT_VOID isr_handler);

static uint8_t APSS_CCA(STK_APSS *p_apss);
static uint8_t APSS_CSMA(STK_APSS *p_apss);

static void APSS_GotoIdle(STK_APSS *p_apss);
static void APSS_GotoPowerDown(STK_APSS *p_apss);
static void APSS_PrepareIdleListening(STK_APSS *p_apss);
static void APSS_TxSACK(STK_APSS *p_apss);
static void APSS_TxStrobe(STK_APSS *p_apss);
static void APSS_TxPayload(STK_APSS *p_apss);

static void APSS_RxStrobeExtended(STK_APSS *p_apss);
static void APSS_RxSACK(STK_APSS *p_apss);
static void APSS_RxPayload(STK_APSS *p_apss);

static void APSS_PrepareRxBroadcast(STK_APSS *p_apss);
static void APSS_PrepareStrobe(STK_APSS *p_apss);
static void APSS_PrepareSACK (STK_APSS *p_apss);

/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static STK_APSS     APSS_Ctx;
static uint8_t      APSS_TxBuf[128];
static uint16_t     APSS_TxBufLen;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
STK_DEV_ID      APSSSrcId;
STK_DEV_ID      APSSDstId;


#if APSS_CFG_LOOSE_SYNC_EN
APSS_PWRON_TBL_ENTRY  APSSPwrOnTbl[APSS_CFG_PWRON_TBL_SIZE];
#endif

s_nsLowMac_t    apss_driver =
{
    "APSS",
    APSS_Init,
    APSS_Send,
    APSS_SendList,
    APSS_Input,
    APSS_On,
    APSS_Off,
    APSS_ChanChkInterval,
};



/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
/**
 * @brief   Initialize APSS module
 * @param   p_netstack  Point to netstack structure
 */
static void APSS_Init (s_ns_t *p_netstack)
{
    STK_ERR   err;
    STK_APSS *p_apss = &APSS_Ctx;


    if (p_netstack == (s_ns_t*)0) {
        return;
    }


    /*
     * Initialize APSS framer
     */
    APSSDstId = APSS_INVALID_DEV_ID;
    memcpy(&APSSSrcId, mac_phy_config.mac_address, 2);
    p_apss->Framer = &APSS_FRAMER;
    p_apss->Framer->Init(&err);


    /*
     * Initialize APSS local variables
     */
    p_apss->State = APSS_STATE_POWER_DOWN;
    p_apss->Netstack = p_netstack;
    p_apss->BroadcastState = APSS_STATE_NONE;
    p_apss->WFAEQty = 0;
    p_apss->WFPEQty = 0;
    p_apss->WFSEQty = 0;
    p_apss->IdleQty = 0;

#if APSS_CFG_LOOSE_SYNC_EN
    p_apss->IsTxDelayed = 0;
    memset(&APSSPwrOnTbl, 0, sizeof(APSSPwrOnTbl));
#endif

    memset(APSS_TxBuf, 0, sizeof(APSS_TxBuf));
    p_apss->TxCbArg = 0;
    p_apss->TxCbFnct = 0;
    p_apss->TxPktPtr = NULL;
    p_apss->TxPktLen = 0;

    Tmr_Create(&p_apss->TmrPowerup,
                LIB_TMR_TYPE_PERIODIC,
                APSS_CFG_POWERUP_INTERVAL_IN_MS,
                APSS_TmrIsrPowerUp,
                p_apss);
    Tmr_Start(&p_apss->TmrPowerup);


    /*
     * Note(s):
     *
     * APSS module state machine is handled by function APSS_Task() which is
     * called directly by registered callback function APSS_CbEvproc().
     * The reason is that both function have different interface, and therefore
     * APSS_Task cannot be registered for event-processing module.
     */
    APSS_SEM_WAIT(APSS_TASK_EVENT);
}


/**
 * @brief   Frame request handler
 * @param   cbsent
 * @param   p_arg
 */
static void APSS_Send (mac_callback_t cbsent, void *p_arg)
{
    RADIO_ERR   radio_err;
    STK_APSS *p_apss = &APSS_Ctx;
    const linkaddr_t *p_dstaddr;

    /*
     * Note(s):
     *
     * (1)  APSS first wakes the destination node up by transmission of strobes.
     *      Upon complete reception of ACK in correspond to the strobe, data packet
     *      is immediately sent.
     *
     * (2)  As APSS is now running on a mission, it should take over other tasks by
     *      raising an event at head of event-processing list
     *
     * (3)  If frame to send is of broadcast type (i.e. it is known by reading
     *      function packetbuf_holds_broadcast), then APSS should first wake up
     *      all surrounding nodes by sending broadcasting strobe for a time
     *      period equal to sleep time. During transmission process of
     *      broadcasting strobes, APSS shall discard all received frames.
     *      Upon expiration of transmission timeout of broadcasting strobes,
     *      APSS shall start sending broadcast message issued by the next higher
     *      layer.
     */
    if ((p_apss->State != APSS_STATE_IDLE) &&
        (p_apss->State != APSS_STATE_POWER_DOWN)) {
        *((STK_ERR *)p_arg) = STK_ERR_BUSY;
        return;
    }

    if (p_apss->State == APSS_STATE_POWER_DOWN) {
        p_apss->Netstack->radio->On(&radio_err);
        if (radio_err != RADIO_ERR_NONE) {
            *((STK_ERR *)p_arg) = STK_ERR_BUSY;
            return;
        }

#if APSS_CFG_REFACTOR_EN
        p_apss->State = APSS_STATE_TX_STARTED;
#else
        p_apss->State = APSS_STATE_TX_STARTED;
#endif

        if (packetbuf_holds_broadcast()) {
            APSSDstId = 0;
            p_apss->BroadcastState = APSS_STATE_TX_BS;
        } else {
            /*
             * Note(s):
             *
             * (1)  A destination ID of 0xFFFF is not allowed in unicast
             *      transmission and and must be checked
             */
            p_dstaddr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
            memcpy(&APSSDstId, p_dstaddr->u8, 2);
            if (APSSDstId == APSS_INVALID_DEV_ID) {
                *((STK_ERR *)p_arg) = STK_ERR_INVALID_ARGUMENT;

                p_apss->State = APSS_STATE_POWER_DOWN;
                APSS_SEM_POST(APSS_TASK_EVENT);
                return;
            }

            APSS_Tmr1Start(&p_apss->Tmr1TxStrobe,
                            APSS_CFG_STROBE_TX_INTERVAL_IN_MS,
                            APSS_TmrIsr1TxStrobe);
        }
    } else {
        p_apss->State = APSS_STATE_TX_P;
    }


    /*
     * Store data to send to TX Buffer of APSS, as Strobe ACK frame to receive
     * is to be written into the common packet buffer by radio driver.
     */
    memcpy(APSS_TxBuf, packetbuf_hdrptr(), packetbuf_totlen());
    APSS_TxBufLen = packetbuf_totlen();
    p_apss->TxCbFnct = cbsent;
    p_apss->TxCbArg = p_arg;

    
    /*
     * Trigger APSS task processing
     */
    APSS_SEM_POST(APSS_TASK_EVENT);
    *((STK_ERR *)p_arg) = STK_ERR_NONE;
}


static void APSS_SendList (mac_callback_t cbsent, void *p_arg, s_nsLmacBufList_t *p_list)
{

}


/**
 * @brief   Frame reception handler
 */
static void  APSS_Input (void)
{
    uint8_t *p_data;
    uint16_t dlen;
    STK_ERR  err;
    STK_APSS *p_apss = &APSS_Ctx;


    /*
     * All received frames shall be discarded as APSS is processing a broadcast
     * waking-up strobe
     */
    if ((p_apss->BroadcastState == APSS_STATE_TX_BS) ||
        (p_apss->BroadcastState == APSS_STATE_TX_B)) {
        return;
    }


    /*
     * The received frame is passed to the APSS framer for parsing process.
     */
    dlen   = (uint16_t )packetbuf_datalen();
    p_data = (uint8_t *)packetbuf_dataptr();
    p_apss->Framer->Parse(p_data, dlen, &err);
    switch (p_apss->State) {
        case APSS_STATE_SCAN_WFS:
        case APSS_STATE_SCAN_WFSE:
            if (err == STK_ERR_NONE) {
                p_apss->State = APSS_STATE_TX_SACK;
            }
            break;

        case APSS_STATE_TX_SWFA:
        case APSS_STATE_TX_SWFAE:
            if (err == STK_ERR_NONE) {
                p_apss->State = APSS_STATE_TX_P;
            }
            break;

        case APSS_STATE_RX_WFP:
        case APSS_STATE_RX_WFPE:
            /*
             * Note(s):
             *
             * (1)  A returned value of STK_ERR_APSS_FRAME_INVALID means that
             *      the received frame is supported by the APSS framer.
             *      Therefore it is likely a data payload frame and then passed
             *      to upper layer.
             * (2)  If a broadcast waking-up strobe has arrived while APSS is
             *      waiting for broadcast data payload, it shall be discarded.
             * (3)  If a strobe has arrived while APSS is waiting for data
             *      payload, then APSS shall terminate scan process.
             */
            if (err == STK_ERR_APSS_UNSUPPORTED_FRAME) {
                p_apss->State = APSS_STATE_RX_P;
            } else {
                p_apss->State = APSS_STATE_SCAN_DONE;
            }
            break;

        case APSS_STATE_IDLE:
            if (err == STK_ERR_APSS_UNSUPPORTED_FRAME) {
                p_apss->State = APSS_STATE_RX_P;
            }
            break;

        default:
            break;
    }


    /*
     * Upon complete reception of a frame, the APSS Task shall be called for
     * further processes if needed.
     */
    APSS_SEM_POST(APSS_TASK_EVENT);
}


/**
 * @brief   Turn APSS on
 * @return
 */
static int8_t APSS_On (void)
{
    RADIO_ERR err;
    APSS_Ctx.Netstack->radio->On(&err);
    return 0;
}


/**
 * @brief   Turn APSS off
 * @return
 */
static int8_t APSS_Off (int keep_radio_on)
{
    RADIO_ERR err;
    APSS_Ctx.Netstack->radio->Off(&err);
    return 0;
}


/**
 * @brief   Obtain channel check interval
 * @return
 */
static unsigned short APSS_ChanChkInterval(void)
{
    return 0;
}


/**
 * @brief   APSS tasks handler
 *
 * @param   c_event
 * @param   p_data
 */
static void APSS_Task (c_event_t c_event, p_data_t p_data)
{
    STK_APSS    *p_apss = &APSS_Ctx;
    uint8_t      is_done = 0;
    RADIO_ERR    err;



    switch (p_apss->State) {
        case APSS_STATE_NONE:
            is_done = 1;
            break;
        case APSS_STATE_POWER_DOWN:
            /*
             * APSS is in power down, waiting for packet transmission requests
             * from the next higher layer or periodic idle listening events.
             * In this state, the radio transceiver must be switched off
             */
            is_done = 1;
            break;
        case APSS_STATE_IDLE:
            break;
        case APSS_STATE_SLEEP:
            APSS_GotoPowerDown(p_apss);
            break;


        /*
         * Idle listening handling
         */
        case APSS_STATE_SCAN_STARTED:
            APSS_PrepareIdleListening(p_apss);
            LED_SCAN_ON();
            break;
#if APSS_CFG_CCA_BASED_SCAN_EN
        case APSS_STATE_SCAN_CCA:
            APSS_CCA();
            break;
        case APSS_STATE_SCAN_CCA_SLEEP:
            break;
#endif
        case APSS_STATE_SCAN_WFS:
            break;
        case APSS_STATE_SCAN_WFSE:
            APSS_RxStrobeExtended(p_apss);
            break;
        case APSS_STATE_SCAN_DONE:
#if APSS_CFG_REFACTOR_EN
            /*
             * Note(s):
             *
             * (1)  It should be possible to check if the scan was due to
             *      periodic idle listening or transmission request
             *
             */
            LED_SCAN_OFF();
            if (APSS_IS_PENDING_TX()) {
                p_apss->State = APSS_STATE_TX_S;
            } else {
                APSS_GotoPowerDown(p_apss);
            }
#else
            APSS_GotoPowerDown(p_apss);
#endif
            break;
        case APSS_STATE_TX_SACK:
            APSS_TxSACK(p_apss);
            break;
        case APSS_STATE_TX_SACKD:
            /*
             * Note(s):
             *
             * APSS waits for a certain amount of time before sending Strobe ACK.
             * However as timeout expires, APSS should put/raise event at the
             * head of of event-processing list to continue processing its tasks
             */
            is_done = 1;
            break;
        case APSS_STATE_RX_BSD:
            break;
        case APSS_STATE_RX_B:
            APSS_PrepareRxBroadcast(p_apss);
            break;
        case APSS_STATE_RX_WFP:
            break;
        case APSS_STATE_RX_WFPE:
            APSS_RxPayload(p_apss);
            break;
        case APSS_STATE_RX_P:
            p_apss->Netstack->hmac->input();
            APSS_GotoIdle(p_apss);
            break;


        /*
         * Data request transmission handling
         */
        case APSS_STATE_TX_STARTED:
            APSS_PrepareStrobe(p_apss);
            break;
        case APSS_STATE_TX_S:
            APSS_TxStrobe(p_apss);
            break;
        case APSS_STATE_TX_SD:
            /*
             * Note(s):
             *
             * APSS waits for a certain amount of time before sending Strobe ACK.
             * However as timeout expires, APSS should put/raise event at the
             * head of of event-processing list to continue processing its tasks
             */
            is_done = 1;
            break;
        case APSS_STATE_TX_SWFA:
            break;
        case APSS_STATE_TX_SWFAE:
            APSS_RxSACK(p_apss);
            break;
        case APSS_STATE_TX_P:
            APSS_TxPayload(p_apss);
            break;
        case APSS_STATE_TX_DONE:
            p_apss->TxCbFnct(p_apss->TxCbArg, p_apss->LastErrTx, 0);
            APSS_GotoIdle(p_apss);
            break;


        default:
            break;
    }


    /*
     * If APSS is not done with its task, it should take over other tasks
     */
    if (is_done == 0) {
        p_apss->Netstack->radio->Task(&err);
        APSS_SEM_POST(APSS_TASK_EVENT);
    }
}


/**
 * @brief   This function implements a APSS-specific CCA
 * @note    CCA shall be performs at maximum 2 times with time period between
 *          successive CCAs equal to gap time of waking-up strobe transmission.
 *          As such APSS should declare a channel busy if either CCA indicates
 *          channel busy.
 * @return  1: if channel is free,
 *          0: if channel is busy
 */
static uint8_t APSS_CCA(STK_APSS *p_apss)
{
    uint8_t chan_idle;
    uint8_t csma_qty;
    RADIO_ERR err;
    RADIO_IOC_VAL is_idle;


    chan_idle = 1;
    csma_qty = 2;
    while (csma_qty--) {
        p_apss->Netstack->radio->Ioctrl(RADIO_IOC_CMD_CCA_GET, &is_idle, &err);
        if (is_idle == 0) {
            chan_idle = 0;
            break;
        }

        if (csma_qty == 1) {    /* time gap is inserted only once */
                                /* during this time period the radio transceiver
                                 * should be put on sleep mode              */
            Tmr_Delay(APSS_PORT_STROBE_TX_GAP_TIME_IN_MS);
        }
    }
    return chan_idle;
}


/**
 * @brief   This function performs CSMA-CA mechanism.
 * @param   p_val
 * @param   p_err
 */
static uint8_t APSS_CSMA(STK_APSS *p_apss)
{
    uint8_t is_chan_idle;
    uint32_t unit_backoff;
    uint32_t nb = 0;
    uint32_t be = 3;
    uint32_t min_be = 3;
    uint32_t max_backoff = 3;
    uint32_t delay = 0;
    uint32_t max_random;


    is_chan_idle = 0;                                           /* Failure by default           */
    unit_backoff = APSS_PORT_STROBE_TX_PERIOD_IN_MS;            /* Unit backoff period          */
    while (nb <= max_backoff) {                                 /* NB > MaxBackoff: Failure     */
        /*
         * Delay for random (2^BE - 1) unit backoff periods
         */
        max_random = (1 << be) - 1;
        delay = bsp_getrand(max_random);
        delay *= unit_backoff;
        Tmr_Delay(delay);

        /*
         * Perform CCA
         */
        is_chan_idle = APSS_CCA(p_apss);
        if (is_chan_idle == 1) {                                /* Channel idle ?               */
            break;                                              /*   Success                    */
        } else {                                                /* Else                         */
            nb++;                                               /*   NB = NB + 1                */
            be = ((be + 1) < min_be) ? (be + 1) : (min_be);     /*   BE = min(BE + 1, MinBE)    */
        }
    }
    return is_chan_idle;
}


/**
 * @brief Commence an one-shot timer
 * @param timeout
 * @param isr_handler
 */
static void APSS_Tmr1Start (LIB_TMR *p_tmr, LIB_TMR_TICK timeout, FNCT_VOID isr_handler)
{
    Tmr_Stop(p_tmr);
    Tmr_Create(p_tmr,
               LIB_TMR_TYPE_ONE_SHOT,
               timeout,
               isr_handler,
               &APSS_Ctx);
    Tmr_Start(p_tmr);
}


/**
 * @brief Timer interrupt service routine handler
 * @param p_arg
 */
static void APSS_TmrIsrPowerUp (void *p_arg)
{
    STK_APSS *p_apss = (STK_APSS *)p_arg;


    if (p_apss->State == APSS_STATE_POWER_DOWN) {
        p_apss->State = APSS_STATE_SCAN_STARTED;
        APSS_SEM_POST(APSS_TASK_EVENT);
    }
}


/**
 * @brief Timer interrupt service routine handler
 * @param p_arg
 */
static void APSS_TmrIsr1Scan (void *p_arg)
{
    RADIO_ERR err;
    RADIO_IOC_VAL state;
    STK_APSS *p_apss = (STK_APSS *)p_arg;


    if (p_apss->State == APSS_STATE_SCAN_WFS) {
        if (p_apss->WFSEQty > APSS_WFSE_COUNT_MAX) {
            p_apss->WFSEQty = 0;
            p_apss->State = APSS_STATE_SCAN_DONE;
            APSS_SEM_POST(APSS_TASK_EVENT);
            return;
        }

        p_apss->Netstack->radio->Ioctrl(RADIO_IOC_CMD_STATE_GET, &state, &err);
        if (state != RADIO_IOC_VAL_STATE_RX) {
            p_apss->State = APSS_STATE_SCAN_DONE;
            APSS_SEM_POST(APSS_TASK_EVENT);
        } else {
            p_apss->WFSEQty++;
            p_apss->State = APSS_STATE_SCAN_WFSE;
        }
    }
#if APSS_CFG_CCA_BASED_SCAN_EN
    if (p_apss->State == APSS_STATE_SCAN_CCA_SLEEP) {
        if (p_apss->CCAQty > APSS_CCA_COUNT_MAX) {
            p_apss->CCAQty = 0;
            p_apss->State = APSS_STATE_SCAN_DONE;
            return;
        }

        p_apss->State = APSS_STATE_SCAN_CCA;
    }
#endif
}


static void APSS_TmrIsr1WFP (void *p_arg)
{
    RADIO_ERR err;
    RADIO_IOC_VAL state;
    STK_APSS *p_apss = (STK_APSS *)p_arg;


    if (p_apss->State == APSS_STATE_RX_WFP) {
        if (p_apss->WFPEQty > APSS_WFPE_COUNT_MAX) {
            p_apss->WFPEQty = 0;
            p_apss->State = APSS_STATE_SCAN_DONE;
            APSS_SEM_POST(APSS_TASK_EVENT);
            return;
        }

        p_apss->Netstack->radio->Ioctrl(RADIO_IOC_CMD_STATE_GET, &state, &err);
        if (state != RADIO_IOC_VAL_STATE_RX) {
            p_apss->State = APSS_STATE_SCAN_DONE;
            APSS_SEM_POST(APSS_TASK_EVENT);
        } else {
            p_apss->WFPEQty++;
            p_apss->State = APSS_STATE_RX_WFPE;
        }
    }
}


/**
 * @brief   Waking-up strobe transmission timeout interrupt handler
 * @param   p_arg
 */
static void APSS_TmrIsr1TxStrobe (void *p_arg)
{
    STK_APSS *p_apss = (STK_APSS *)p_arg;


    p_apss->State = APSS_STATE_TX_DONE;
    p_apss->LastErrTx = STK_ERR_TX_TIMEOUT;
    APSS_SEM_POST(APSS_TASK_EVENT);
}


/**
 * @brief   Delayed strobe ACK transmission timeout interrupt handler
 * @param   p_arg
 */
static void APSS_TmrIsr1TxSACKD (void *p_arg)
{
    STK_APSS *p_apss = (STK_APSS *)p_arg;


    if (p_apss->State == APSS_STATE_TX_SACKD) {
        p_apss->State  = APSS_STATE_TX_SACK;
        APSS_SEM_POST(APSS_TASK_EVENT);
    }
}


/**
 * @brief   Delayed broadcast reception timeout interrupt handler
 * @param   p_arg
 */
static void APSS_TmrIsr1RxBSD (void *p_arg)
{
    STK_APSS *p_apss = (STK_APSS *)p_arg;


    if (p_apss->State == APSS_STATE_RX_BSD) {
        p_apss->State  = APSS_STATE_RX_B;
        APSS_SEM_POST(APSS_TASK_EVENT);
    }
}


/**
 * @brief   Waiting-for-ACK timeout interrupt handler
 * @param   p_arg
 */
static void APSS_TmrIsr1WFA (void *p_arg)
{
    RADIO_ERR err;
    RADIO_IOC_VAL state;
    STK_APSS *p_apss = (STK_APSS *)p_arg;


    if (p_apss->BroadcastState == APSS_STATE_TX_B) {
        p_apss->State = APSS_STATE_TX_P;
        APSS_SEM_POST(APSS_TASK_EVENT);
        return;
    }

    if (p_apss->State == APSS_STATE_TX_SWFA) {
        if (p_apss->WFAEQty > APSS_WFAE_COUNT_MAX) {
            p_apss->WFAEQty = 0;
            p_apss->State = APSS_STATE_TX_S;
            APSS_SEM_POST(APSS_TASK_EVENT);
            return;
        }

        p_apss->Netstack->radio->Ioctrl(RADIO_IOC_CMD_STATE_GET, &state, &err);
        if (state != RADIO_IOC_VAL_STATE_RX) {
            p_apss->State = APSS_STATE_TX_S;
            APSS_SEM_POST(APSS_TASK_EVENT);
        } else {
            p_apss->WFAEQty++;
            p_apss->State = APSS_STATE_TX_SWFAE;
        }
        return;
    }

    if (p_apss->State == APSS_STATE_TX_PWFA) {
        p_apss->State = APSS_STATE_TX_DONE;
        p_apss->LastErrTx = STK_ERR_TX_NOPACK;
        APSS_SEM_POST(APSS_TASK_EVENT);
    }
}


/**
 * @brief   A time elapse after receiving Strobe TX request, now APSS starts
 *          transmitting waking-up Strobe
 * @param   p_arg
 */
static void APSS_TmrIsr1TxSD (void *p_arg)
{
    STK_APSS *p_apss = (STK_APSS *)p_arg;


#if APSS_CFG_REFACTOR_EN
    if (p_apss->State == APSS_STATE_TX_SD) {
        p_apss->State  = APSS_STATE_SCAN_STARTED;
        APSS_SEM_POST(APSS_TASK_EVENT);
    }
#else
    if (p_apss->State == APSS_STATE_TX_SD) {
        p_apss->State  = APSS_STATE_TX_S;
        APSS_SEM_POST(APSS_TASK_EVENT);
    }
#endif
}


/**
 * @brief   Idle timeout interrupt handler
 * @param   p_arg
 */
static void APSS_TmrIsr1Idle(void *p_arg)
{
    STK_APSS *p_apss = (STK_APSS *)p_arg;
    RADIO_ERR err;
    RADIO_IOC_VAL state;


    if (p_apss->State == APSS_STATE_IDLE) {
        if (p_apss->WFPEQty > APSS_WFPE_COUNT_MAX) {
            p_apss->WFPEQty = 0;
            p_apss->State = APSS_STATE_SLEEP;
            APSS_SEM_POST(APSS_TASK_EVENT);
            return;
        }

        p_apss->Netstack->radio->Ioctrl(RADIO_IOC_CMD_STATE_GET, &state, &err);
        if (state != RADIO_IOC_VAL_STATE_RX) {
            p_apss->State = APSS_STATE_SLEEP;
            APSS_SEM_POST(APSS_TASK_EVENT);
        } else {
            p_apss->WFPEQty++;
            p_apss->State = APSS_STATE_RX_WFPE;
        }
    }
}


/**
 * @brief   APSS clean up before going back to state sleep, waiting for the next
 *          activity.
 */
static void APSS_GotoPowerDown (STK_APSS *p_apss)
{
    RADIO_ERR       radio_err;


    LED_SCAN_OFF();
    p_apss->Netstack->radio->Off(&radio_err);

    Tmr_Stop(&p_apss->Tmr1W);
    Tmr_Stop(&p_apss->Tmr1Scan);
    Tmr_Stop(&p_apss->Tmr1TxStrobe);

    memset(APSS_TxBuf, 0, sizeof(APSS_TxBuf));
    APSS_TxBufLen = 0;
    p_apss->TxPktPtr = NULL;
    p_apss->TxPktLen = 0;

    p_apss->WFAEQty = 0;
    p_apss->WFPEQty = 0;
    p_apss->WFSEQty = 0;
    p_apss->IdleQty  = 0;
    APSSDstId = APSS_INVALID_DEV_ID;

    p_apss->BroadcastState = APSS_STATE_NONE;
    p_apss->State = APSS_STATE_POWER_DOWN;
    LCD_UPDATE();
}


static void APSS_GotoIdle(STK_APSS *p_apss)
{
    /*
     * Note(s):
     *
     * (1)  APSS is allowed to enter Idle state maximum APSS_CFG_PKT_RX_QTY_IN_IDLE
     *      times in a row, meaning that APSS can receive/transmit maximum
     *      APSS_CFG_PKT_RX_QTY_IN_IDLE packets in Idle states. This is meant
     *      to avoid DoS attack that prevents APSS from going to power down.
     */
    if (++p_apss->IdleQty > APSS_CFG_PKT_RX_QTY_IN_IDLE) {
        APSS_GotoPowerDown(p_apss);
        return;
    }

    LED_SCAN_ON();
    p_apss->State = APSS_STATE_IDLE;
    p_apss->WFPEQty = 0;
    APSS_Tmr1Start(&p_apss->Tmr1Idle,
                    APSS_CFG_IDLE_DURATION_IN_MS,
                    APSS_TmrIsr1Idle);
}


/**
 * @brief   Prepare for idle listening process.
 */
static void APSS_PrepareIdleListening (STK_APSS *p_apss)
{
    RADIO_ERR       radio_err;


    /*
     * Note(s): Prepare for an idle listening
     *
     * (1)  turn RF on
     * (2)  start scan timer only when the RF is ready, and then APSS
     *      goes to state waiting for strobe APSS_STATE_SCAN_WFS.
     *      If RF is not ready, the idle listening attempt shall be
     *      terminated and APSS goes straightforward to state
     *      APSS_STATE_SCAN_DONE
     */
#if APSS_CFG_CCA_BASED_SCAN_EN
    if (p_apss->State == APSS_STATE_SCAN_STARTED) {
        APSS_CCA ();
    } else {
        p_apss->State = APSS_STATE_SCAN_DONE;
    }
#else
    p_apss->Netstack->radio->On(&radio_err);
    if (radio_err == RADIO_ERR_NONE) {
        if (p_apss->State == APSS_STATE_SCAN_STARTED) {
            p_apss->State = APSS_STATE_SCAN_WFS;
            APSS_Tmr1Start(&p_apss->Tmr1Scan,
                            APSS_PORT_SCAN_DURATION_IN_MS,
                            APSS_TmrIsr1Scan);
        }
    } else {
        p_apss->State = APSS_STATE_SCAN_DONE;
    }
#endif
}


/**
 * @brief   Transmit an ACK in response to a received waking-up strobe
 */
static void APSS_TxSACK (STK_APSS *p_apss)
{
    RADIO_ERR rf_err;


    /*
     * Prepare Strobe ACK
     */
    APSS_PrepareSACK(p_apss);

    /*
     * Turn RF on when required
     */
    if (p_apss->IsTxDelayed) {
        p_apss->Netstack->radio->On(&rf_err);
        p_apss->IsTxDelayed = 0;
    }

    /*
     * Attempt to transmit Strobe ACK
     */
    p_apss->Netstack->radio->Send(p_apss->TxPktPtr,
                                  p_apss->TxPktLen,
                                  &rf_err);
    if (rf_err == RADIO_ERR_NONE) {
        p_apss->State = APSS_STATE_RX_WFP;
        APSS_Tmr1Start(&p_apss->Tmr1W,
                        APSS_PORT_WFP_TIMEOUT_IN_MS,
                        APSS_TmrIsr1WFP);
    } else {
        /*
         * APSS has failed to reply with an ACK, then shall terminate scan
         * process
         */
        p_apss->State = APSS_STATE_SCAN_DONE;
    }
}


/**
 * @brief   Transmit waking-up strobe
 */
static void APSS_TxStrobe (STK_APSS *p_apss)
{
    RADIO_ERR   radio_err;
    RADIO_IOC_VAL state;


    /*
     * Turn radio on if required and reset Strobe transmission timer
     */
    if (p_apss->IsTxDelayed) {
        p_apss->Netstack->radio->On(&radio_err);
        APSS_Tmr1Start (&p_apss->Tmr1TxStrobe,
                         APSS_CFG_STROBE_TX_INTERVAL_IN_MS,
                         APSS_TmrIsr1TxStrobe);
        p_apss->IsTxDelayed = 0;
    }


    p_apss->Netstack->radio->Send(p_apss->TxPktPtr,
                                  p_apss->TxPktLen,
                                  &radio_err);
    if (radio_err != RADIO_ERR_NONE) {
        /*
         * Note(s): Strobe has failed to be transmitted
         *
         * (1)  After created, the waking-up strobe is sent to the destination node.
         *      Sometime a packet has arrived while APSS is about to transmit a
         *      strobe. This is because RF is set to RX mode when powered on, during
         *      which time an incoming strobe is detected.
         *      Following successful transmission of the strobe, APSS goes to a state
         *      where it waits for a corresponding ACK to the sent strobe for a
         *      predefined period of time.
         */
        p_apss->Netstack->radio->Ioctrl(RADIO_IOC_CMD_STATE_GET,
                                        &state,
                                        &radio_err);
        if (state != RADIO_IOC_VAL_STATE_RX) {
            p_apss->State = APSS_STATE_TX_DONE;
            p_apss->LastErrTx = STK_ERR_TX_RADIO_SEND;
        } else {
            p_apss->State = APSS_STATE_TX_SWFAE;
        }
    } else {
        /*
         * Strobe was successfully transmitted
         */
        if (p_apss->State == APSS_STATE_TX_S) {
            p_apss->State = APSS_STATE_TX_SWFA;
            APSS_Tmr1Start (&p_apss->Tmr1W,
                             APSS_PORT_WFA_TIMEOUT_IN_MS,
                             APSS_TmrIsr1WFA);
        }
        APSS_PrepareStrobe(p_apss);
    }
}


/**
 * @brief   Transmit data payload
 */
static void APSS_TxPayload (STK_APSS *p_apss)
{
    RADIO_ERR       err;
    RADIO_IOC_VAL   sync;


    sync = RADIO_IOC_VAL_SYNC_DATA;
    p_apss->Netstack->radio->Ioctrl (RADIO_IOC_CMD_SYNC_SET, &sync, &err);
    if (err == RADIO_ERR_NONE) {
        p_apss->Netstack->radio->Send (APSS_TxBuf, APSS_TxBufLen, &err);
        if (err != RADIO_ERR_NONE) {
            p_apss->LastErrTx = STK_ERR_TX_RADIO_SEND;
        } else {
            p_apss->LastErrTx = STK_ERR_NONE;
        }
    }
    p_apss->State = APSS_STATE_TX_DONE;
}


/**
 * @brief   Strobe reception extended preparation
 */
static void APSS_RxStrobeExtended(STK_APSS *p_apss)
{
    if (p_apss->State == APSS_STATE_SCAN_WFSE) {
        p_apss->State = APSS_STATE_SCAN_WFS;
        APSS_Tmr1Start(&p_apss->Tmr1Scan,
                        APSS_PORT_SCAN_DURATION_IN_MS,
                        APSS_TmrIsr1Scan);
    }
}


/**
 * @brief   Strobe ACK reception preparation
 * @param   p_arg
 */
static void APSS_RxSACK(STK_APSS *p_apss)
{
    if (p_apss->State == APSS_STATE_TX_SWFAE) {
        p_apss->State = APSS_STATE_TX_SWFA;
        APSS_Tmr1Start(&p_apss->Tmr1W,
                        APSS_PORT_WFA_TIMEOUT_IN_MS,
                        APSS_TmrIsr1WFA);
    }
}


/**
 * @brief   Data payload reception preparation
 * @param   p_arg
 */
static void APSS_RxPayload(STK_APSS *p_apss)
{
    if (p_apss->State == APSS_STATE_RX_WFPE) {
        p_apss->State = APSS_STATE_RX_WFP;
        APSS_Tmr1Start(&p_apss->Tmr1W,
                        APSS_PORT_WFP_TIMEOUT_IN_MS,
                        APSS_TmrIsr1WFP);
    }
}


/**
 * @brief   Prepare for broadcast reception
 * @param   p_arg
 */
static void APSS_PrepareRxBroadcast (STK_APSS *p_apss)
{
    RADIO_ERR err;


    if (p_apss->State == APSS_STATE_RX_B) {
        p_apss->Netstack->radio->On(&err);
        p_apss->State = APSS_STATE_RX_WFP;
        APSS_Tmr1Start(&p_apss->Tmr1W,
                        APSS_PORT_SCAN_DURATION_IN_MS * 3,
                        APSS_TmrIsr1WFP);
    }
}


/**
 * @brief   Prepare Strobe
 * @param   p_arg
 */
static void APSS_PrepareStrobe (STK_APSS *p_apss)
{
    STK_ERR         stk_err;
    RADIO_ERR       radio_err;
    LIB_TMR_TICK    delay;


    /*
     * Note(s):
     *
     * (1)  Issue APSS frame to create a waking-up strobe. A delay can be added to
     *      the first strobe when Loose-Sync feature is enabled.
     *
     * (2)  If APSS is required to transmit a broadcast message, then broadcast
     *      strobes are needed to send first. Upon complete transmission of the
     *      last broadcast strobe, APSS shall transmit broadcast message issued
     *      by the next higher layer.
     *      The last broadcast strobe is detected by a return error code of
     *      @ref STK_ERR_APSS_BROADCAST_LAST_STROBE
     */
    if (p_apss->BroadcastState == APSS_STATE_TX_BS) {
        p_apss->TxPktPtr = p_apss->Framer->Create(APSS_FRAME_TYPE_BROADCAST,
                                                  &p_apss->TxPktLen,
                                                  &delay,
                                                  &stk_err);
        if (stk_err == STK_ERR_APSS_BROADCAST_LAST_STROBE) {
            p_apss->BroadcastState = APSS_STATE_TX_B;
        }
    } else {
        p_apss->TxPktPtr = p_apss->Framer->Create(APSS_FRAME_TYPE_STROBE,
                                                  &p_apss->TxPktLen,
                                                  &delay,
                                                  &stk_err);
    }

#if APSS_CFG_LOOSE_SYNC_EN
    /*
     * Delay is only applied to the 1st strobe
     */
#if APSS_CFG_REFACTOR_EN
    if (p_apss->State == APSS_STATE_TX_STARTED) {
        /*
         * Perform a channel scan before strobes transmission attempt
         */
        if (delay) {
            p_apss->State = APSS_STATE_TX_SD;
            APSS_Tmr1Start(&p_apss->Tmr1W,
                            delay,
                            APSS_TmrIsr1TxSD);
            p_apss->Netstack->radio->Off(&radio_err);
            p_apss->IsTxDelayed = 1;
        } else {
            p_apss->State = APSS_STATE_SCAN_STARTED;
        }
    }
#else
    if (p_apss->State == APSS_STATE_TX_STARTED) {
        /*
         * Perform a channel scan before strobes transmission attempt
         */
        if (delay) {
            p_apss->State = APSS_STATE_TX_SD;
            APSS_Tmr1Start(&p_apss->Tmr1W,
                            delay,
                            APSS_TmrIsr1TxSD);
            p_apss->Netstack->radio->Off(&radio_err);
            p_apss->IsTxDelayed = 1;
            return;
        } else {
            p_apss->State = APSS_STATE_TX_S;
        }
    }
#endif  /* APSS_CFG_REFACTOR_EN     */

#endif  /* APSS_CFG_LOOSE_SYNC_EN   */
}


/**
 * @brief   Prepare Strobe ACK
 * @param   p_apss
 */
static void APSS_PrepareSACK (STK_APSS *p_apss)
{
    STK_ERR         stk_err;
    RADIO_ERR       rf_err;
    LIB_TMR_TICK    delay;


    /*
     * Issue Framer to create a Strobe ACK in response to the received strobe
     */
    p_apss->TxPktPtr = p_apss->Framer->Create(APSS_FRAME_TYPE_SACK,
                                              &p_apss->TxPktLen,
                                              &delay,
                                              &stk_err);
    if ((p_apss->TxPktPtr == NULL) &&
        (stk_err == STK_ERR_APSS_BROADCAST_NOACK)) {
        /*
         * ACK is not required when a broadcast strobe is received
         */
        p_apss->State = APSS_STATE_RX_BSD;
        p_apss->BroadcastState = APSS_STATE_RX_B;
        if (delay) {
            APSS_Tmr1Start(&p_apss->Tmr1WBS,
                            delay,
                            APSS_TmrIsr1RxBSD);
            p_apss->Netstack->radio->Off(&rf_err);
        }
    } else {
        /*
         * Note(s):
         *
         * (1)  If the acknowledgment is required to be sent after a time period,
         *      APSS shall go to state transmitting Strobe ACK with delay and turn
         *      the radio off.
         */
        if (delay) {
            p_apss->State = APSS_STATE_TX_SACKD;
            APSS_Tmr1Start(&p_apss->Tmr1W,
                            delay,
                            APSS_TmrIsr1TxSACKD);
            p_apss->Netstack->radio->Off(&rf_err);
            p_apss->IsTxDelayed = 1;
        }
    }
}


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
#endif /* EMB6_CFG_APSS_EN */
