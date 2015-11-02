/**
 * @file    lpr_apss.c
 * @author  PN
 * @brief   Asynchronous Power Saving Scheme module
 */

/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"


#if NETSTK_CFG_LPR_APSS_EN
#include "evproc.h"
#include "packetbuf.h"
#include "bsp.h"

#include "lib_tmr.h"
#include "lpr.h"


/*
********************************************************************************
*                               LOCAL DEFINES
********************************************************************************
*/
#define LPR_CFG_PKT_RX_QTY_IN_IDLE              (uint8_t)( 1u )


#define LPR_WFSE_COUNT_MAX                      (uint8_t)( 2u )
#define LPR_WFAE_COUNT_MAX                      (uint8_t)( 2u )
#define LPR_WFPE_COUNT_MAX                      (uint8_t)( 3u )


#if LPR_CFG_CCA_BASED_SCAN_EN
#define LPR_CCA_COUNT_MAX                       (uint8_t) ( 3u )
#endif


/**
 * @addtogroup  LPR_STATES     APSS states
 * @{
 */
typedef enum
{
    /*
     * Power-related states
     */
    LPR_STATE_NONE             =  0u,      /*!< Initial state                                                  */
    LPR_STATE_POWER_DOWN       =  1u,      /*!< Power-Down state                                               */
    LPR_STATE_IDLE             =  2u,      /*!< Idle state                                                     */
    LPR_STATE_SLEEP            =  3u,      /*!< Sleep state                                                    */

    /*
     * Scan procedure related states
     */
    LPR_STATE_SCAN_STARTED     = 10u,      /*!< Scan process has started                                       */
#if LPR_CFG_CCA_BASED_SCAN_EN
    LPR_STATE_SCAN_CCA         = 11u,      /*!< Performing CCA                                                 */
    LPR_STATE_SCAN_CCA_SLEEP   = 12u,      /*!< Sleep between successive CCAs                                  */
#endif
    LPR_STATE_SCAN_WFS         = 13u,      /*!< Wait For waking-up strobe                                      */
    LPR_STATE_SCAN_WFSE        = 14u,      /*!< Wait-For-Strobe Extension                                      */
    LPR_STATE_SCAN_DONE        = 15u,      /*!< Scan process has finished                                      */

    /*
     * Transmission-related states
     */
    LPR_STATE_TX_STARTED       = 20u,      /*!< Waking-up-Strobe Transmission                                  */
    LPR_STATE_TX_S             = 21u,      /*!< Waking-up-Strobe Transmission                                  */
    LPR_STATE_TX_SD            = 22u,      /*!< Waking-up Strobe Transmission Delayed                          */
    LPR_STATE_TX_SWFA          = 23u,      /*!< Wait For Strobe ACK                                            */
    LPR_STATE_TX_SWFAE         = 24u,      /*!< Wait For strobe ACK Extension                                  */
    LPR_STATE_TX_P             = 25u,      /*!< Data Payload Transmission                                      */
    LPR_STATE_TX_PWFA          = 26u,      /*!< Wait For Payload ACK                                           */
    LPR_STATE_TX_DONE          = 27u,      /*!< Data transmission process has finished                         */

    LPR_STATE_TX_LBT           = 50u,      /*!< Listen-Before-Talk     */
    LPR_STATE_TX_LBT_WFA       = 51u,      /*!< Wait-For-ACK, after a waking-up strobe with same destination ID was received           */
    LPR_STATE_TX_LBT_WFI       = 52u,      /*!< Wait-For-Idle, until the destination node has completed data exchange with other nodes */


    /*
     * Reception-related states
     */
    LPR_STATE_TX_SACK          = 30u,      /*!< Received a valid Strobe and reply with an ACK immediately      */
    LPR_STATE_TX_SACKD         = 31u,      /*!< Received a valid Strobe and reply with an ACK after a Delay    */
    LPR_STATE_RX_WFP           = 32u,      /*!< Wait-For-Payload                                               */
    LPR_STATE_RX_WFPE          = 33u,      /*!< Wait-For-Payload-Extension                                     */
    LPR_STATE_RX_P             = 34u,      /*!< Receiving-Payload                                              */


    /*
     * Broadcast-related states
     */
    LPR_STATE_TX_BS            = 40u,      /*!< Broadcasting Waking-up-Strobe Transmission                     */
    LPR_STATE_TX_B             = 41u,      /*!< Broadcast Transmission                                         */
    LPR_STATE_RX_BSD           = 42u,      /*!< Broadcast Reception with delay                                 */
    LPR_STATE_RX_B             = 43u,      /*!< Broadcast Reception with delay                                 */

}LPR_STATE;
/**
 * @}
 */


typedef struct netstk_apss NETSTK_APSS;

struct netstk_apss
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
    LPR_FRAMER_DRV     *Framer;
    LPR_STATE           State;
    LPR_STATE           BroadcastState;
    e_nsErr_t          LastErrTx;

    nsTxCbFnct_t       TxCbFnct;
    void               *TxCbArg;
    uint8_t            *TxPktPtr;
    uint16_t            TxPktLen;

#if LPR_CFG_LOOSE_SYNC_EN
    uint8_t             IsTxDelayed;
#endif

#if LPR_CFG_CCA_BASED_SCAN_EN
    uint8_t             CCA_Qty;
#endif
};


/*
********************************************************************************
*                               LOCAL MACROS
********************************************************************************
*/
#define LPR_EVENT_PEND(_event_)          evproc_regCallback(_event_, LPR_Task)
#define LPR_EVENT_POST(_event_)          evproc_putEvent(E_EVPROC_HEAD, _event_, &LPR_Ctx)


/*
********************************************************************************
*                       LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void LPR_Init (void *p_netstk, e_nsErr_t *p_err);
static void LPR_On(e_nsErr_t *p_err);
static void LPR_Off(e_nsErr_t *p_err);
static void LPR_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void LPR_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void LPR_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);

static void LPR_Task(c_event_t c_event, p_data_t p_data);
static void LPR_TmrIsrPowerUp(void *p_arg);
static void LPR_TmrIsr1Scan(void *p_arg);
static void LPR_TmrIsr1WFP(void *p_arg);
static void LPR_TmrIsr1TxStrobe(void *p_arg);
static void LPR_TmrIsr1TxSACKD(void *p_arg);
static void LPR_TmrIsr1WFA(void *p_arg);
static void LPR_Tmr1Start(LIB_TMR *p_tmr, LIB_TMR_TICK timeout, FNCT_VOID isr_handler);

static uint8_t LPR_CCA(NETSTK_APSS *p_apss);
static uint8_t LPR_CSMA(NETSTK_APSS *p_apss);

static void LPR_GotoIdle(NETSTK_APSS *p_apss);
static void LPR_GotoPowerDown(NETSTK_APSS *p_apss);
static void LPR_PrepareIdleListening(NETSTK_APSS *p_apss);
static void LPR_TxSACK(NETSTK_APSS *p_apss);
static void LPR_TxStrobe(NETSTK_APSS *p_apss);
static void LPR_TxPayload(NETSTK_APSS *p_apss);

static void LPR_RxStrobeExtended(NETSTK_APSS *p_apss);
static void LPR_RxSACK(NETSTK_APSS *p_apss);
static void LPR_RxPayload(NETSTK_APSS *p_apss);

static void LPR_PrepareRxBroadcast(NETSTK_APSS *p_apss);
static void LPR_PrepareStrobe(NETSTK_APSS *p_apss);
static void LPR_PrepareSACK (NETSTK_APSS *p_apss);

/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static NETSTK_APSS      LPR_Ctx;
static uint8_t          LPR_TxBuf[128];
static uint16_t         LPR_TxBufLen;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
NETSTK_DEV_ID      LPRSrcId;
NETSTK_DEV_ID      LPRDstId;


#if LPR_CFG_LOOSE_SYNC_EN
LPR_PWRON_TBL_ENTRY  LPRPwrOnTbl[LPR_CFG_PWRON_TBL_SIZE];
#endif

const s_nsLPR_t LPRDrvAPSS =
{
   "LPR APSS",
    LPR_Init,
    LPR_On,
    LPR_Off,
    LPR_Send,
    LPR_Recv,
    LPR_IOCtrl,
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
static void LPR_Init (void *p_netstk, e_nsErr_t *p_err)
{
    NETSTK_APSS *p_apss = &LPR_Ctx;


    if (p_netstk == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }


    /*
     * Initialize APSS framer
     */
    LPRDstId = LPR_INVALID_DEV_ID;
    memcpy(&LPRSrcId, mac_phy_config.mac_address, 2);
    p_apss->Framer = &LPR_FRAMER;
    p_apss->Framer->Init(p_err);


    /*
     * Initialize APSS local variables
     */
    p_apss->State = LPR_STATE_POWER_DOWN;
    p_apss->Netstack = (s_ns_t *)p_netstk;
    p_apss->BroadcastState = LPR_STATE_NONE;
    p_apss->WFAEQty = 0;
    p_apss->WFPEQty = 0;
    p_apss->WFSEQty = 0;
    p_apss->IdleQty = 0;

    /*
     * Configure transmission-related attributes
     */
#if LPR_CFG_LOOSE_SYNC_EN
    p_apss->IsTxDelayed = 0;
    memset(&LPRPwrOnTbl, 0, sizeof(LPRPwrOnTbl));
#endif
    memset(LPR_TxBuf, 0, sizeof(LPR_TxBuf));
    p_apss->TxCbArg = 0;
    p_apss->TxCbFnct = 0;
    p_apss->TxPktPtr = NULL;
    p_apss->TxPktLen = 0;

    /*
     * Configure power-up timer
     */
    Tmr_Create(&p_apss->TmrPowerup,
                LIB_TMR_TYPE_PERIODIC,
                LPR_CFG_POWERUP_INTERVAL_IN_MS,
                LPR_TmrIsrPowerUp,
                p_apss);
    Tmr_Start(&p_apss->TmrPowerup);


    /*
     * Note(s):
     *
     * APSS module state machine is handled by function LPR_Task() which is
     * called directly by registered callback function LPR_CbEvproc().
     * The reason is that both function have different interface, and therefore
     * LPR_Task cannot be registered for event-processing module.
     */
    LPR_EVENT_PEND(NETSTK_LPR_EVENT);
}


/**
 * @brief   Frame request handler
 * @param   cbsent
 * @param   p_arg
 */
static void LPR_Send (uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
    NETSTK_APSS *p_apss = &LPR_Ctx;
    const linkaddr_t *p_dstaddr;



    LED_TX_ON();
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
    if ((p_apss->State != LPR_STATE_IDLE) &&
        (p_apss->State != LPR_STATE_POWER_DOWN)) {
        *p_err = NETSTK_ERR_BUSY;
        return;
    }

    if (p_apss->State == LPR_STATE_POWER_DOWN) {
        p_apss->State = LPR_STATE_TX_STARTED;

        /*
         * Turn RF on
         */
        p_apss->Netstack->rf->on(p_err);
        if (*p_err != NETSTK_ERR_NONE) {
            p_apss->State = LPR_STATE_POWER_DOWN;
            return;
        }

        /*
         * Category frame to send
         */
        if (packetbuf_holds_broadcast()) {
            LPRDstId = 0;
            p_apss->BroadcastState = LPR_STATE_TX_BS;
        } else {
            /*
             * Note(s):
             *
             * (1)  A destination ID of 0xFFFF is not allowed in unicast
             *      transmission and and must be checked
             */
            p_dstaddr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
            memcpy(&LPRDstId, p_dstaddr->u8, 2);
            if (LPRDstId == LPR_INVALID_DEV_ID) {
                p_apss->Netstack->rf->off(p_err);
                p_apss->State = LPR_STATE_POWER_DOWN;
                *p_err = NETSTK_ERR_INVALID_ARGUMENT;
                return;
            }

            /*
             * Start transmission timer
             */
            LPR_Tmr1Start(&p_apss->Tmr1TxStrobe,
                            LPR_CFG_STROBE_TX_INTERVAL_IN_MS,
                            LPR_TmrIsr1TxStrobe);
        }
    } else {
        /*
         * If APSS is in state IDLE, frame is transmitted immediately
         */
        p_apss->State = LPR_STATE_TX_P;
    }


    /*
     * Store data to send to TX Buffer of APSS, as Strobe ACK frame to receive
     * is to be written into the common packet buffer by radio driver.
     */
    memcpy(LPR_TxBuf, p_data, len);
    LPR_TxBufLen = len;

    
    /*
     * Trigger APSS task processing
     */
    *p_err = NETSTK_ERR_NONE;
    LPR_EVENT_POST(NETSTK_LPR_EVENT);
}



/**
 * @brief   Frame reception handler
 */
static void  LPR_Recv (uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
    NETSTK_APSS *p_apss = &LPR_Ctx;


    /*
     * All received frames shall be discarded as APSS is processing a broadcast
     * waking-up strobe
     */
    if ((p_apss->BroadcastState == LPR_STATE_TX_BS) ||
        (p_apss->BroadcastState == LPR_STATE_TX_B)) {
        return;
    }

    /*
     * The received frame is passed to the APSS framer for parsing process.
     */
    *p_err = NETSTK_ERR_NONE;
    p_apss->Framer->Parse(p_data, len, p_err);
    switch (p_apss->State) {
        case LPR_STATE_SCAN_WFS:
        case LPR_STATE_SCAN_WFSE:
            switch (*p_err) {
                case NETSTK_ERR_NONE:
                    p_apss->State = LPR_STATE_TX_SACK;
                    break;

                case NETSTK_ERR_LPR_INVALID_ADDR:
                    p_apss->State = LPR_STATE_SCAN_DONE;
                    break;

                case NETSTK_ERR_LPR_TX_COLLISION_SAME_DEST:
                    p_apss->State = LPR_STATE_TX_LBT_WFA;
                    break;

                case NETSTK_ERR_LPR_TX_COLLISION_DIFF_DEST:
                    p_apss->State = LPR_STATE_TX_DONE;
                    break;

                default:
                    break;
            }
            LPR_EVENT_POST(NETSTK_LPR_EVENT);
            break;


        case LPR_STATE_TX_LBT_WFA:
            /*
             * Any incoming strobes are discarded. Only SACK from destination
             * node is of interest.
             * If a Strobe ACK isn't arrived before strobe transmission timeout
             * expires, then APSS shall declare failed transmission attempt
             */
            if (*p_err == NETSTK_ERR_NONE) {
                p_apss->State = LPR_STATE_TX_LBT_WFI;
            }
            break;

        case LPR_STATE_TX_SWFA:
        case LPR_STATE_TX_SWFAE:
            if (*p_err == NETSTK_ERR_NONE) {
                p_apss->State = LPR_STATE_TX_P;
                LPR_EVENT_POST(NETSTK_LPR_EVENT);
            }
            break;


        case LPR_STATE_RX_WFP:
        case LPR_STATE_RX_WFPE:
            /*
             * Note(s):
             *
             * (1)  A returned value of STK_ERR_LPR_FRAME_INVALID means that
             *      the received frame is supported by the APSS framer.
             *      Therefore it is likely a data payload frame and then passed
             *      to upper layer.
             * (2)  If a broadcast waking-up strobe has arrived while APSS is
             *      waiting for broadcast data payload, it shall be discarded.
             * (3)  If a strobe has arrived while APSS is waiting for data
             *      payload, then APSS shall terminate scan process.
             */
            if (*p_err == NETSTK_ERR_LPR_UNSUPPORTED_FRAME) {
                packetbuf_clear();
                packetbuf_set_datalen(len);
                memcpy(packetbuf_dataptr(),
                       p_data,
                       len);
                p_apss->State = LPR_STATE_RX_P;
            } else {
                p_apss->State = LPR_STATE_SCAN_DONE;
            }
            LPR_EVENT_POST(NETSTK_LPR_EVENT);
            break;

        case LPR_STATE_IDLE:
            if (*p_err == NETSTK_ERR_LPR_UNSUPPORTED_FRAME) {
                packetbuf_clear();
                packetbuf_set_datalen(len);
                memcpy(packetbuf_dataptr(),
                       p_data,
                       len);
                p_apss->State = LPR_STATE_RX_P;
            }
            LPR_EVENT_POST(NETSTK_LPR_EVENT);
            break;

        default:
            break;
    }
}


static void LPR_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
{
    *p_err = NETSTK_ERR_NONE;
    switch (cmd) {
        case NETSTK_CMD_TX_CBFNCT_SET:
            LPR_Ctx.TxCbFnct = (nsTxCbFnct_t)p_val;
            break;

        case NETSTK_CMD_TX_CBARG_SET:
            LPR_Ctx.TxCbArg = p_val;
            break;

        default:
            LPR_Ctx.Netstack->rf->ioctrl(cmd, p_val, p_err);
            break;
    }
}

/**
 * @brief   Turn APSS on
 * @return
 */
static void LPR_On (e_nsErr_t *p_err)
{
    LPR_Ctx.Netstack->rf->on(p_err);
}


/**
 * @brief   Turn APSS off
 * @return
 */
static void LPR_Off (e_nsErr_t *p_err)
{
    LPR_Ctx.Netstack->rf->off(p_err);
}


/**
 * @brief   APSS tasks handler
 *
 * @param   c_event
 * @param   p_data
 */
static void LPR_Task(c_event_t c_event, p_data_t p_data)
{
    NETSTK_APSS  *p_apss = &LPR_Ctx;
    e_nsErr_t    err = NETSTK_ERR_NONE;


    switch (p_apss->State) {
        case LPR_STATE_NONE:
            break;
        case LPR_STATE_POWER_DOWN:
            /*
             * APSS is in power down, waiting for packet transmission requests
             * from the next higher layer or periodic idle listening events.
             * In this state, the radio transceiver must be switched off
             */
            break;
        case LPR_STATE_IDLE:
            break;
        case LPR_STATE_SLEEP:
            LPR_GotoPowerDown(p_apss);
            break;


        /*
         * Idle listening handling
         */
        case LPR_STATE_SCAN_STARTED:
            LPR_PrepareIdleListening(p_apss);
            break;
#if LPR_CFG_CCA_BASED_SCAN_EN
        case LPR_STATE_SCAN_CCA:
            LPR_CCA();
            break;
        case LPR_STATE_SCAN_CCA_SLEEP:
            break;
#endif
        case LPR_STATE_SCAN_WFS:
            break;
        case LPR_STATE_SCAN_WFSE:
            LPR_RxStrobeExtended(p_apss);
            break;
        case LPR_STATE_SCAN_DONE:
            LED_SCAN_OFF();
            if (LPR_IS_PENDING_TX()) {
                p_apss->State = LPR_STATE_TX_S;
                LPR_EVENT_POST(NETSTK_LPR_EVENT);
            } else {
                LPR_GotoPowerDown(p_apss);
            }
            break;
        case LPR_STATE_TX_SACK:
            LPR_TxSACK(p_apss);
            break;
        case LPR_STATE_TX_SACKD:
            /*
             * Note(s):
             *
             * APSS waits for a certain amount of time before sending Strobe ACK.
             * However as timeout expires, APSS should put/raise event at the
             * head of of event-processing list to continue processing its tasks
             */
            break;
        case LPR_STATE_RX_BSD:
            break;
        case LPR_STATE_RX_B:
            LPR_PrepareRxBroadcast(p_apss);
            break;
        case LPR_STATE_RX_WFP:
            break;
        case LPR_STATE_RX_WFPE:
            LPR_RxPayload(p_apss);
            break;
        case LPR_STATE_RX_P:
            p_apss->Netstack->phy->recv(packetbuf_dataptr(),
                                        packetbuf_datalen(),
                                        &err);
            LPR_GotoIdle(p_apss);
            break;


        /*
         * Data request transmission handling
         */
        case LPR_STATE_TX_STARTED:
            LPR_PrepareStrobe(p_apss);
            break;

        case LPR_STATE_TX_LBT_WFA:
            /*
             * Any incoming strobes are discarded.
             */
            break;

        case LPR_STATE_TX_LBT_WFI:
            /*
             * Note(s):
             *
             * (1)  Most probably the destination node is on now, CSMA-CA
             *      should be performed before data payload transmission
             *      attempts.
             *
             * (2)  LPR_STATE_TX_P shall behave in same manner.
             */
            break;

        case LPR_STATE_TX_S:
            LPR_TxStrobe(p_apss);
            break;
        case LPR_STATE_TX_SD:
            /*
             * Note(s):
             *
             * APSS waits for a certain amount of time before sending Strobe ACK.
             * However as timeout expires, APSS should put/raise event at the
             * head of of event-processing list to continue processing its tasks
             */
            break;
        case LPR_STATE_TX_SWFA:
            break;
        case LPR_STATE_TX_SWFAE:
            LPR_RxSACK(p_apss);
            break;
        case LPR_STATE_TX_P:
            LPR_TxPayload(p_apss);
            break;
        case LPR_STATE_TX_DONE:
            if (p_apss->TxCbFnct) {
                p_apss->TxCbFnct(p_apss->TxCbArg, &p_apss->LastErrTx);
            }
            LPR_GotoIdle(p_apss);
            break;

        default:
            break;
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
static uint8_t LPR_CCA(NETSTK_APSS *p_apss)
{
    uint8_t chan_idle;
    uint8_t csma_qty;
    uint8_t is_idle;
    e_nsErr_t err = NETSTK_ERR_NONE;
    

    chan_idle = 1;
    csma_qty = 2;
    while (csma_qty--) {
        p_apss->Netstack->rf->ioctrl(NETSTK_CMD_RF_CCA_GET,
                                     &is_idle,
                                     &err);
        if (is_idle == 0) {
            chan_idle = 0;
            break;
        }

        if (csma_qty == 1) {    /* time gap is inserted only once */
                                /* during this time period the radio transceiver
                                 * should be put on sleep mode              */
            Tmr_Delay(LPR_PORT_STROBE_TX_GAP_TIME_IN_MS);
        }
    }
    return chan_idle;
}


/**
 * @brief   This function performs CSMA-CA mechanism.
 * @param   p_val
 * @param   p_err
 */
static uint8_t LPR_CSMA(NETSTK_APSS *p_apss)
{
    uint32_t unit_backoff;
    uint32_t nb;
    uint32_t be;
    uint32_t min_be = 3;
    uint32_t max_backoff = 3;
    uint32_t delay = 0;
    uint32_t max_random;
    e_nsErr_t err = NETSTK_ERR_NONE;
    uint8_t is_idle;


    nb = 0;
    be = min_be;
    unit_backoff = 1; //LPR_PORT_STROBE_TX_PERIOD_IN_MS;           /* Unit backoff period          */
    while (nb <= max_backoff) {                                     /* NB > MaxBackoff: Failure     */
        /*
         * Delay for random (2^BE - 1) unit backoff periods
         */
        max_random = (1 << be) - 1;
        delay = bsp_getrand(max_random);
        delay *= unit_backoff / 50; // symbol period is 20us @50kbps
        Tmr_Delay(delay);

        /*
         * Perform CCA
         */
        p_apss->Netstack->rf->ioctrl(NETSTK_CMD_RF_CCA_GET,
                                     &is_idle,
                                     &err);
        if (is_idle == 1) {                                         /* Channel idle ?               */
            break;                                                  /*   Success                    */
        } else {                                                    /* Else                         */
            nb++;                                                   /*   NB = NB + 1                */
            be = ((be + 1) < min_be) ? (be + 1) : (min_be);         /*   BE = min(BE + 1, MinBE)    */
        }
    }
    return is_idle;
}


/**
 * @brief Commence an one-shot timer
 * @param timeout
 * @param isr_handler
 */
static void LPR_Tmr1Start (LIB_TMR *p_tmr, LIB_TMR_TICK timeout, FNCT_VOID isr_handler)
{
    Tmr_Stop(p_tmr);
    Tmr_Create(p_tmr,
               LIB_TMR_TYPE_ONE_SHOT,
               timeout,
               isr_handler,
               &LPR_Ctx);
    Tmr_Start(p_tmr);
}


/**
 * @brief Timer interrupt service routine handler
 * @param p_arg
 */
static void LPR_TmrIsrPowerUp (void *p_arg)
{
    NETSTK_APSS *p_apss = (NETSTK_APSS *)p_arg;


    if (p_apss->State == LPR_STATE_POWER_DOWN) {
        p_apss->State = LPR_STATE_SCAN_STARTED;
        LPR_EVENT_POST(NETSTK_LPR_EVENT);
        LED_SCAN_ON();
    }
}


/**
 * @brief Timer interrupt service routine handler
 * @param p_arg
 */
static void LPR_TmrIsr1Scan (void *p_arg)
{
    e_nsErr_t err = NETSTK_ERR_NONE;
    uint8_t is_rx_busy;
    NETSTK_APSS *p_apss = (NETSTK_APSS *)p_arg;


    if (p_apss->State == LPR_STATE_SCAN_WFS) {
        if (p_apss->WFSEQty > LPR_WFSE_COUNT_MAX) {
            p_apss->WFSEQty = 0;
            p_apss->State = LPR_STATE_SCAN_DONE;
            LPR_EVENT_POST(NETSTK_LPR_EVENT);
            return;
        }

        p_apss->Netstack->rf->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY,
                                     &is_rx_busy,
                                     &err);
        if (is_rx_busy != 1) {
            p_apss->State = LPR_STATE_SCAN_DONE;
            LPR_EVENT_POST(NETSTK_LPR_EVENT);
        } else {
            p_apss->WFSEQty++;
            p_apss->State = LPR_STATE_SCAN_WFSE;
        }
    }
#if LPR_CFG_CCA_BASED_SCAN_EN
    if (p_apss->State == LPR_STATE_SCAN_CCA_SLEEP) {
        if (p_apss->CCAQty > LPR_CCA_COUNT_MAX) {
            p_apss->CCAQty = 0;
            p_apss->State = LPR_STATE_SCAN_DONE;
            return;
        }

        p_apss->State = LPR_STATE_SCAN_CCA;
    }
#endif
}


static void LPR_TmrIsr1WFP (void *p_arg)
{
    e_nsErr_t err = NETSTK_ERR_NONE;
    uint8_t is_rx_busy;
    NETSTK_APSS *p_apss = (NETSTK_APSS *)p_arg;


    if (p_apss->State == LPR_STATE_RX_WFP) {
        if (p_apss->WFPEQty > LPR_WFPE_COUNT_MAX) {
            p_apss->WFPEQty = 0;
            p_apss->State = LPR_STATE_SCAN_DONE;
            LPR_EVENT_POST(NETSTK_LPR_EVENT);
            return;
        }

        p_apss->Netstack->rf->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY,
                                     &is_rx_busy,
                                     &err);
        if (is_rx_busy != 1) {
            p_apss->State = LPR_STATE_SCAN_DONE;
            LPR_EVENT_POST(NETSTK_LPR_EVENT);
        } else {
            p_apss->WFPEQty++;
            p_apss->State = LPR_STATE_RX_WFPE;
        }
    }
}


/**
 * @brief   Waking-up strobe transmission timeout interrupt handler
 * @param   p_arg
 */
static void LPR_TmrIsr1TxStrobe (void *p_arg)
{
    NETSTK_APSS *p_apss = (NETSTK_APSS *)p_arg;


    p_apss->State = LPR_STATE_TX_DONE;
    p_apss->LastErrTx = NETSTK_ERR_TX_TIMEOUT;
    LPR_EVENT_POST(NETSTK_LPR_EVENT);
}


/**
 * @brief   Delayed strobe ACK transmission timeout interrupt handler
 * @param   p_arg
 */
static void LPR_TmrIsr1TxSACKD (void *p_arg)
{
    NETSTK_APSS *p_apss = (NETSTK_APSS *)p_arg;


    if (p_apss->State == LPR_STATE_TX_SACKD) {
        p_apss->State  = LPR_STATE_TX_SACK;
        LPR_EVENT_POST(NETSTK_LPR_EVENT);
    }
}


/**
 * @brief   Delayed broadcast reception timeout interrupt handler
 * @param   p_arg
 */
static void LPR_TmrIsr1RxBSD (void *p_arg)
{
    NETSTK_APSS *p_apss = (NETSTK_APSS *)p_arg;


    if (p_apss->State == LPR_STATE_RX_BSD) {
        p_apss->State  = LPR_STATE_RX_B;
        LPR_EVENT_POST(NETSTK_LPR_EVENT);
    }
}


/**
 * @brief   Waiting-for-ACK timeout interrupt handler
 * @param   p_arg
 */
static void LPR_TmrIsr1WFA (void *p_arg)
{
    e_nsErr_t err = NETSTK_ERR_NONE;
    uint8_t is_rx_busy;
    NETSTK_APSS *p_apss = (NETSTK_APSS *)p_arg;


    if (p_apss->BroadcastState == LPR_STATE_TX_B) {
        p_apss->State = LPR_STATE_TX_P;
        LPR_EVENT_POST(NETSTK_LPR_EVENT);
        return;
    }

    if (p_apss->State == LPR_STATE_TX_SWFA) {
        if (p_apss->WFAEQty > LPR_WFAE_COUNT_MAX) {
            p_apss->WFAEQty = 0;
            p_apss->State = LPR_STATE_TX_S;
            LPR_EVENT_POST(NETSTK_LPR_EVENT);
            return;
        }

        p_apss->Netstack->rf->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY,
                                     &is_rx_busy,
                                     &err);
        if (is_rx_busy != 1) {
            p_apss->State = LPR_STATE_TX_S;
            LPR_EVENT_POST(NETSTK_LPR_EVENT);
        } else {
            p_apss->WFAEQty++;
            p_apss->State = LPR_STATE_TX_SWFAE;
        }
        return;
    }

    if (p_apss->State == LPR_STATE_TX_PWFA) {
        p_apss->State = LPR_STATE_TX_DONE;
        p_apss->LastErrTx = NETSTK_ERR_TX_NOACK;
        LPR_EVENT_POST(NETSTK_LPR_EVENT);
    }
}


/**
 * @brief   A time elapse after receiving Strobe TX request, now APSS starts
 *          transmitting waking-up Strobe
 * @param   p_arg
 */
static void LPR_TmrIsr1TxSD (void *p_arg)
{
    NETSTK_APSS *p_apss = (NETSTK_APSS *)p_arg;


    /*
     * Perform a channel scan before strobes transmission attempt
     */
    if (p_apss->State == LPR_STATE_TX_SD) {
        p_apss->State  = LPR_STATE_SCAN_STARTED;
        LPR_EVENT_POST(NETSTK_LPR_EVENT);
    }
}


/**
 * @brief   Idle timeout interrupt handler
 * @param   p_arg
 */
static void LPR_TmrIsr1Idle(void *p_arg)
{
    NETSTK_APSS *p_apss = (NETSTK_APSS *)p_arg;
    e_nsErr_t err = NETSTK_ERR_NONE;
    uint8_t is_rx_busy;


    if (p_apss->State == LPR_STATE_IDLE) {
        if (p_apss->WFPEQty > LPR_WFPE_COUNT_MAX) {
            p_apss->WFPEQty = 0;
            p_apss->State = LPR_STATE_SLEEP;
            LPR_EVENT_POST(NETSTK_LPR_EVENT);
            return;
        }

        p_apss->Netstack->rf->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY,
                                     &is_rx_busy,
                                     &err);
        if (is_rx_busy != 1) {
            p_apss->State = LPR_STATE_SLEEP;
            LPR_EVENT_POST(NETSTK_LPR_EVENT);
        } else {
            p_apss->WFPEQty++;
            p_apss->State = LPR_STATE_RX_WFPE;
        }
    }
}


/**
 * @brief   APSS clean up before going back to state sleep, waiting for the next
 *          activity.
 */
static void LPR_GotoPowerDown (NETSTK_APSS *p_apss)
{
    e_nsErr_t err = NETSTK_ERR_NONE;


    LED_SCAN_OFF();

    p_apss->Netstack->rf->off(&err);
    Tmr_Stop(&p_apss->Tmr1W);
    Tmr_Stop(&p_apss->Tmr1Scan);
    Tmr_Stop(&p_apss->Tmr1TxStrobe);

    memset(LPR_TxBuf, 0, sizeof(LPR_TxBuf));
    LPR_TxBufLen = 0;
    p_apss->TxPktPtr = NULL;
    p_apss->TxPktLen = 0;
    p_apss->IsTxDelayed = 0;

    p_apss->WFAEQty = 0;
    p_apss->WFPEQty = 0;
    p_apss->WFSEQty = 0;
    p_apss->IdleQty  = 0;
    LPRDstId = LPR_INVALID_DEV_ID;

    p_apss->BroadcastState = LPR_STATE_NONE;
    p_apss->State = LPR_STATE_POWER_DOWN;
}


static void LPR_GotoIdle(NETSTK_APSS *p_apss)
{
    /*
     * Note(s):
     *
     * (1)  APSS is allowed to enter Idle state maximum LPR_CFG_PKT_RX_QTY_IN_IDLE
     *      times in a row, meaning that APSS can receive/transmit maximum
     *      LPR_CFG_PKT_RX_QTY_IN_IDLE packets in Idle states. This is meant
     *      to avoid DoS attack that prevents APSS from going to power down.
     */
    if (++p_apss->IdleQty > LPR_CFG_PKT_RX_QTY_IN_IDLE) {
        LPR_GotoPowerDown(p_apss);
        return;
    }

    LED_SCAN_ON();
    p_apss->State = LPR_STATE_IDLE;
    p_apss->WFPEQty = 0;
    LPR_Tmr1Start(&p_apss->Tmr1Idle,
                    LPR_CFG_IDLE_DURATION_IN_MS,
                    LPR_TmrIsr1Idle);
}


/**
 * @brief   Prepare for idle listening process.
 */
static void LPR_PrepareIdleListening (NETSTK_APSS *p_apss)
{
    e_nsErr_t err = NETSTK_ERR_NONE;


    /*
     * Note(s): Prepare for an idle listening
     *
     * (1)  turn RF on
     * (2)  start scan timer only when the RF is ready, and then APSS
     *      goes to state waiting for strobe LPR_STATE_SCAN_WFS.
     *      If RF is not ready, the idle listening attempt shall be
     *      terminated and APSS goes straightforward to state
     *      LPR_STATE_SCAN_DONE
     */
#if LPR_CFG_CCA_BASED_SCAN_EN
    if (p_apss->State == LPR_STATE_SCAN_STARTED) {
        LPR_CCA ();
    } else {
        p_apss->State = LPR_STATE_SCAN_DONE;
    }
#else
    p_apss->Netstack->rf->on(&err);
    if (err == NETSTK_ERR_NONE) {
        if (p_apss->State == LPR_STATE_SCAN_STARTED) {
            p_apss->State = LPR_STATE_SCAN_WFS;
            LPR_Tmr1Start(&p_apss->Tmr1Scan,
                            LPR_PORT_SCAN_DURATION_IN_MS,
                            LPR_TmrIsr1Scan);
        }
    } else {
        p_apss->State = LPR_STATE_SCAN_DONE;
        LPR_EVENT_POST(NETSTK_LPR_EVENT);
    }
#endif
}


/**
 * @brief   Transmit an ACK in response to a received waking-up strobe
 */
static void LPR_TxSACK (NETSTK_APSS *p_apss)
{
    e_nsErr_t err = NETSTK_ERR_NONE;


    /*
     * Prepare Strobe ACK
     */
    LPR_PrepareSACK(p_apss);
    if (p_apss->State == LPR_STATE_TX_SACK) {
        /*
         * Turn RF on when required
         */
        if (p_apss->IsTxDelayed) {
            p_apss->Netstack->rf->on(&err);
            p_apss->IsTxDelayed = 0;
        }

        /*
         * Attempt to transmit Strobe ACK
         */
        p_apss->Netstack->rf->send(p_apss->TxPktPtr,
                                   p_apss->TxPktLen,
                                   &err);
        if (err == NETSTK_ERR_NONE) {
            p_apss->State = LPR_STATE_RX_WFP;
            LPR_Tmr1Start(&p_apss->Tmr1W,
                            LPR_PORT_WFP_TIMEOUT_IN_MS,
                            LPR_TmrIsr1WFP);
        } else {
            /*
             * APSS has failed to reply with an ACK, then shall terminate scan
             * process
             */
            p_apss->State = LPR_STATE_SCAN_DONE;
        }
    }
}


/**
 * @brief   Transmit waking-up strobe
 */
static void LPR_TxStrobe (NETSTK_APSS *p_apss)
{
    e_nsErr_t err = NETSTK_ERR_NONE;
    uint8_t is_rx_busy;


    /*
     * Turn radio on if required and reset Strobe transmission timer
     */
    if (p_apss->IsTxDelayed) {
        p_apss->Netstack->rf->on(&err);
        LPR_Tmr1Start (&p_apss->Tmr1TxStrobe,
                         LPR_CFG_STROBE_TX_INTERVAL_IN_MS,
                         LPR_TmrIsr1TxStrobe);
        p_apss->IsTxDelayed = 0;
    }


    p_apss->Netstack->rf->send(p_apss->TxPktPtr,
                               p_apss->TxPktLen,
                               &err);
    if (err != NETSTK_ERR_NONE) {
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
        p_apss->Netstack->rf->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY,
                                     &is_rx_busy,
                                     &err);
        if (is_rx_busy != 1) {
            p_apss->State = LPR_STATE_TX_DONE;
            p_apss->LastErrTx = NETSTK_ERR_RF_SEND;
            LPR_EVENT_POST(NETSTK_LPR_EVENT);
        } else {
            p_apss->State = LPR_STATE_TX_SWFAE;
        }
    } else {
        /*
         * Strobe was successfully transmitted
         */
        if (p_apss->State == LPR_STATE_TX_S) {
            p_apss->State = LPR_STATE_TX_SWFA;
            LPR_Tmr1Start (&p_apss->Tmr1W,
                             LPR_PORT_WFA_TIMEOUT_IN_MS,
                             LPR_TmrIsr1WFA);
        }
        LPR_PrepareStrobe(p_apss);
    }
}


/**
 * @brief   Transmit data payload
 */
static void LPR_TxPayload (NETSTK_APSS *p_apss)
{
#if 0 // draft
    uint8_t is_chan_idle;


    is_chan_idle = LPR_CSMA(p_apss);
    if (is_chan_idle) {
        p_apss->Netstack->radio->Send (LPR_TxBuf, LPR_TxBufLen, &err);
        if (err != RADIO_ERR_NONE) {
            p_apss->LastErrTx = STK_ERR_TX_RADIO_SEND;
        } else {
            p_apss->LastErrTx = STK_ERR_NONE;
        }
    } else {
        p_apss->LastErrTx = STK_ERR_LPR_CHANNEL_ACESS_FAILURE;
    }

#else
    p_apss->Netstack->rf->send(LPR_TxBuf,
                               LPR_TxBufLen,
                               &p_apss->LastErrTx);
#endif
    p_apss->State = LPR_STATE_TX_DONE;
    LPR_EVENT_POST(NETSTK_LPR_EVENT);
}


/**
 * @brief   Strobe reception extended preparation
 */
static void LPR_RxStrobeExtended(NETSTK_APSS *p_apss)
{
    if (p_apss->State == LPR_STATE_SCAN_WFSE) {
        p_apss->State = LPR_STATE_SCAN_WFS;
        LPR_Tmr1Start(&p_apss->Tmr1Scan,
                        LPR_PORT_SCAN_DURATION_IN_MS,
                        LPR_TmrIsr1Scan);
    }
}


/**
 * @brief   Strobe ACK reception preparation
 * @param   p_arg
 */
static void LPR_RxSACK(NETSTK_APSS *p_apss)
{
    if (p_apss->State == LPR_STATE_TX_SWFAE) {
        p_apss->State = LPR_STATE_TX_SWFA;
        LPR_Tmr1Start(&p_apss->Tmr1W,
                        LPR_PORT_WFA_TIMEOUT_IN_MS,
                        LPR_TmrIsr1WFA);
    }
}


/**
 * @brief   Data payload reception preparation
 * @param   p_arg
 */
static void LPR_RxPayload(NETSTK_APSS *p_apss)
{
    if (p_apss->State == LPR_STATE_RX_WFPE) {
        p_apss->State = LPR_STATE_RX_WFP;
        LPR_Tmr1Start(&p_apss->Tmr1W,
                        LPR_PORT_WFP_TIMEOUT_IN_MS,
                        LPR_TmrIsr1WFP);
    }
}


/**
 * @brief   Prepare for broadcast reception
 * @param   p_arg
 */
static void LPR_PrepareRxBroadcast (NETSTK_APSS *p_apss)
{
    e_nsErr_t err = NETSTK_ERR_NONE;


    if (p_apss->State == LPR_STATE_RX_B) {
        p_apss->Netstack->rf->on(&err);
        p_apss->State = LPR_STATE_RX_WFP;
        LPR_Tmr1Start(&p_apss->Tmr1W,
                        LPR_PORT_SCAN_DURATION_IN_MS * 3,
                        LPR_TmrIsr1WFP);
    }
}


/**
 * @brief   Prepare Strobe
 * @param   p_arg
 */
static void LPR_PrepareStrobe (NETSTK_APSS *p_apss)
{
    e_nsErr_t     err = NETSTK_ERR_NONE;
    LIB_TMR_TICK   delay = 0;


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
     *      @ref STK_ERR_LPR_BROADCAST_LAST_STROBE
     */
    if (p_apss->BroadcastState == LPR_STATE_TX_BS) {
        p_apss->TxPktPtr = p_apss->Framer->Create(LPR_FRAME_TYPE_BROADCAST,
                                                  &p_apss->TxPktLen,
                                                  &delay,
                                                  &err);
        if (err == NETSTK_ERR_LPR_BROADCAST_LAST_STROBE) {
            p_apss->BroadcastState = LPR_STATE_TX_B;
        }
    } else {
        p_apss->TxPktPtr = p_apss->Framer->Create(LPR_FRAME_TYPE_STROBE,
                                                  &p_apss->TxPktLen,
                                                  &delay,
                                                  &err);
    }

#if LPR_CFG_LOOSE_SYNC_EN
    /*
     * Delay is only applied to the 1st strobe
     */
    if (p_apss->State == LPR_STATE_TX_STARTED) {
        /*
         * Perform a channel scan before strobes transmission attempt
         */
        if (delay) {
            p_apss->State = LPR_STATE_TX_SD;
            LPR_Tmr1Start(&p_apss->Tmr1W,
                            delay,
                            LPR_TmrIsr1TxSD);
            p_apss->Netstack->rf->off(&err);
            p_apss->IsTxDelayed = 1;
        } else {
            p_apss->State = LPR_STATE_SCAN_STARTED;
            LPR_EVENT_POST(NETSTK_LPR_EVENT);
        }
    }
#endif  /* LPR_CFG_LOOSE_SYNC_EN   */
}


/**
 * @brief   Prepare Strobe ACK
 * @param   p_apss
 */
static void LPR_PrepareSACK (NETSTK_APSS *p_apss)
{
    e_nsErr_t      err = NETSTK_ERR_NONE;
    LIB_TMR_TICK    delay;


    /*
     * Issue Framer to create a Strobe ACK in response to the received strobe
     */
    p_apss->TxPktPtr = p_apss->Framer->Create(LPR_FRAME_TYPE_SACK,
                                              &p_apss->TxPktLen,
                                              &delay,
                                              &err);
    if ((p_apss->TxPktPtr == NULL) &&
        (err == NETSTK_ERR_LPR_BROADCAST_NOACK)) {
        /*
         * ACK is not required when a broadcast strobe is received
         */
        p_apss->State = LPR_STATE_RX_BSD;
        p_apss->BroadcastState = LPR_STATE_RX_B;
        if (delay) {
            LPR_Tmr1Start(&p_apss->Tmr1WBS,
                           delay,
                           LPR_TmrIsr1RxBSD);
            p_apss->Netstack->rf->off(&err);
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
            p_apss->State = LPR_STATE_TX_SACKD;
            LPR_Tmr1Start(&p_apss->Tmr1W,
                            delay,
                            LPR_TmrIsr1TxSACKD);
            p_apss->Netstack->rf->off(&err);
            p_apss->IsTxDelayed = 1;
        }
    }
}


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
#endif /* NETSTK_CFG_LPR_APSS_EN */
