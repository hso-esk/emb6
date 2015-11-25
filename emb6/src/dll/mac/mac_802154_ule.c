/**
 * @file    mac_802154_ule.c
 * @author  PN
 * @brief   IEEE802.15.4 Ultra-Low Energy MAC
 */

/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"

#include "evproc.h"
#include "packetbuf.h"
#include "bsp.h"

#include "lib_tmr.h"
#include "framer_802154.h"
#include "mac_ule.h"

/*
********************************************************************************
*                               LOCAL DEFINES
********************************************************************************
*/
#define MAC_ULE_CFG_REFACTOR_EN             ( 1u )


/*
********************************************************************************
*                               LOCAL ENUMS
********************************************************************************
*/
typedef enum
{
    MAC_ULE_CMD_NONE = 0U,
    MAC_ULE_CMD_INIT,
    MAC_ULE_CMD_SLEEP,
    MAC_ULE_CMD_SCAN,
    MAC_ULE_CMD_TX,
    MAC_ULE_CMD_RX,
    MAC_ULE_CMD_IDLE,
}MAC_ULE_CMD;


typedef enum
{
    MAC_ULE_STATE_NON_INIT = 0U,
    MAC_ULE_STATE_INIT,
    MAC_ULE_STATE_SLEEP,
    MAC_ULE_STATE_IDLE,

    /* SCAN Submachine states */
    MAC_ULE_STATE_SCAN_STARTED,
    MAC_ULE_STATE_SCAN_BUSY,
    MAC_ULE_STATE_SCAN_FINISHED,

    /* RX Submachine states */
    MAC_ULE_STATE_RX_STARTED,
    MAC_ULE_STATE_RX_PARSING,
    MAC_ULE_STATE_RX_DELAYED,
    MAC_ULE_STATE_RX_WFP,
    MAC_ULE_STATE_RX_FINISHED,

    /* TX Submachine states */
    MAC_ULE_STATE_TX_STARTED,
    MAC_ULE_STATE_TX_CSMA,
    MAC_ULE_STATE_TX_DELAYED,
    MAC_ULE_STATE_TX_STROBE,
    MAC_ULE_STATE_TX_WFSA,
    MAC_ULE_STATE_TX_DATA,
    MAC_ULE_STATE_TX_WFA,
    MAC_ULE_STATE_TX_FINISHED
}MAC_ULE_STATE;


/*
********************************************************************************
*                               LOCAL MACROS
********************************************************************************
*/
#define MAC_ULE_EVENT_PEND(_event_)         evproc_regCallback(_event_, MAC_ULE_ProcessCmd)
#define MAC_ULE_EVENT_POST(_event_)         evproc_putEvent(E_EVPROC_HEAD, _event_, NULL)

#define MAC_ULE_IS_SLEEP()                  (MAC_ULE_State==MAC_ULE_STATE_SLEEP)

#define MAC_ULE_IS_FREE()                   ((MAC_ULE_State==MAC_ULE_STATE_SLEEP) ||   \
                                             (MAC_ULE_State==MAC_ULE_STATE_IDLE))


/*
********************************************************************************
*                       LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void MAC_ULE_Init (void *p_netstk, e_nsErr_t *p_err);
static void MAC_ULE_On(e_nsErr_t *p_err);
static void MAC_ULE_Off(e_nsErr_t *p_err);
static void MAC_ULE_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void MAC_ULE_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void MAC_ULE_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);

static void MAC_ULE_ProcessCmd(c_event_t c_event, p_data_t p_data);
static void MAC_ULE_ProcessIdle(e_nsErr_t *p_err);
static void MAC_ULE_ProcessSleep(e_nsErr_t *p_err);
static void MAC_ULE_ProcessScan(e_nsErr_t *p_err);
static void MAC_ULE_ProcessTx(e_nsErr_t *p_err);
static void MAC_ULE_ProcessRx(e_nsErr_t *p_err);

static void MAC_ULE_Tmr1Start(LIB_TMR *p_tmr, LIB_TMR_TICK timeout, FNCT_VOID isr_handler);
static void MAC_ULE_TmrIsrPowerUp(void *p_arg);
static void MAC_ULE_TmrIsr1WFA(void *p_arg);
static void MAC_ULE_TmrIsr1WFS(void *p_arg);
static void MAC_ULE_TmrIsr1Delay(void *p_arg);
static void MAC_ULE_TmrIsr1Idle(void *p_arg);

static void MAC_ULE_RxStrobe(e_nsErr_t *p_err);
static void MAC_ULE_RxPayload(e_nsErr_t *p_err);
static void MAC_ULE_TxStrobe(e_nsErr_t *p_err);
static void MAC_ULE_TxPayload(e_nsErr_t *p_err);
static void MAC_ULE_TxACK(e_nsErr_t *p_err, uint8_t seq);
static void MAC_ULE_CSMA(e_nsErr_t *p_err);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static  s_ns_t         *MAC_ULE_Netstk;
static  void           *MAC_ULE_TxCbArg;
static  nsTxCbFnct_t    MAC_ULE_TxCbFnct;
static  uint8_t        *MAC_ULE_TxPktPtr;
static  uint16_t        MAC_ULE_TxPktLen;
static  uint8_t        *MAC_ULE_RxPktPtr;
static  uint16_t        MAC_ULE_RxPktLen;

static  MAC_ULE_CMD     MAC_ULE_Cmd;
static  MAC_ULE_STATE   MAC_ULE_State;

static  LIB_TMR         MAC_ULE_TmrPowerUp;
static  LIB_TMR         MAC_ULE_Tmr1Scan;
static  LIB_TMR         MAC_ULE_Tmr1W;
static  LIB_TMR         MAC_ULE_Tmr1Tx;
static  LIB_TMR         MAC_ULE_Tmr1Delay;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
NETSTK_DEV_ID      NetstkSrcId;
NETSTK_DEV_ID      NetstkDstId;


#if MAC_ULE_CFG_LOOSE_SYNC_EN
s_nsMacUlePwrOnTblEntry_t  MacUlePwrOnTbl[MAC_ULE_CFG_PWRON_TBL_SIZE];
#endif

const s_nsMAC_t MACDrv802154ULE =
{
   "MAC IEEE802.15.4 Ultra-Low Energy",
    MAC_ULE_Init,
    MAC_ULE_On,
    MAC_ULE_Off,
    MAC_ULE_Send,
    MAC_ULE_Recv,
    MAC_ULE_IOCtrl,
};


/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
static void MAC_ULE_Init (void *p_netstk, e_nsErr_t *p_err)
{
    MAC_ULE_Cmd = MAC_ULE_CMD_NONE;
    MAC_ULE_State = MAC_ULE_STATE_NON_INIT;


#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }

    if (p_netstk == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }
#endif


    /* set returned error code to default */
    *p_err = NETSTK_ERR_NONE;

    /*
     * Initialize internal attributes
     */
    MAC_ULE_Netstk = (s_ns_t *)p_netstk;
    MAC_ULE_TxCbArg = 0;
    MAC_ULE_TxCbFnct = 0;
    MAC_ULE_TxPktPtr = NULL;
    MAC_ULE_TxPktLen = 0;
    MAC_ULE_RxPktPtr = NULL;
    MAC_ULE_RxPktLen = 0;

    /*
     * Initialize APSS framer
     */
    NetstkDstId = MAC_ULE_ID_INVALID;
    memcpy(&NetstkSrcId, mac_phy_config.mac_address, 2);
    SmartMACFramer.Init(p_err);

    /*
     * Configure timers
     */
    Tmr_Create(&MAC_ULE_TmrPowerUp,
                LIB_TMR_TYPE_PERIODIC,
                MAC_ULE_CFG_POWERUP_INTERVAL_IN_MS,
                MAC_ULE_TmrIsrPowerUp,
                NULL);

    Tmr_Create(&MAC_ULE_Tmr1Scan,
               LIB_TMR_TYPE_ONE_SHOT,
               MAC_ULE_PORT_SCAN_DURATION_IN_MS,
               0,
               NULL);

    /* Register event handler */
    MAC_ULE_EVENT_PEND(NETSTK_MAC_ULE_EVENT);

    /* State transitions */
    MAC_ULE_Cmd = MAC_ULE_CMD_SLEEP;
    MAC_ULE_State = MAC_ULE_STATE_SLEEP;

    /* start power-up intervals */
    Tmr_Start(&MAC_ULE_TmrPowerUp);
}


static void MAC_ULE_On(e_nsErr_t *p_err)
{
    MAC_ULE_Netstk->phy->on(p_err);
}


static void MAC_ULE_Off(e_nsErr_t *p_err)
{
    MAC_ULE_Netstk->phy->off(p_err);
}


/**
 * @brief   Transmission request handling
 *
 * @note    (1) A transmission request is accepted only if one of following
 *          conditions is met
 *            (a) MAC_ULE_State == MAC_ULE_STATE_IDLE, or
 *            (b) MAC_ULE_State == MAC_ULE_STATE_SLEEP
 *
 *          (2) In case MAC ULE in state MAC_ULE_STATE_IDLE, if the destination
 *          node is not the node with which the device is exchanging data (i.e.
 *          some local variables could be used to temporarily store those info),
 *          then the MAC ULE shall start the low-power transmission procedure
 *          over.
 *
 *          (3) While MAC ULE stay in sleep mode, waiting for the 'right' time
 *          from when it starts sending wake-up frames, it shall neither enable
 *          the transceiver nor perform channel scan. This allows simplicity in
 *          implementation.
 *          The simplicity, nevertheless, comes at costs of losing advantage of
 *          the loosely-synchronization feature in congested network. The
 *          problem occurs when the device is waiting until the moment it thinks
 *          the node of interest wakes up. Simultaneously another node desires
 *          to communicate with the device, and therefore it misses the 'right'
 *          time as the device is not able to receive any frames during this
 *          time.
 *          This problem could be solved via an appropriate retransmission
 *          operation.
 *
 */
static void MAC_ULE_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
    const linkaddr_t *p_dstaddr;


#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }

    if ((len == 0) ||
        (p_data == NULL)) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }
#endif

    if (MAC_ULE_IS_SLEEP() == FALSE) {
        *p_err = NETSTK_ERR_BUSY;
        return;
    }


    /* set the returned error code to default */
    *p_err = NETSTK_ERR_NONE;

    /* obtain destination addressing information */
    if (packetbuf_holds_broadcast()) {
        NetstkDstId = MAC_ULE_DEV_ID_BROADCAST;
    } else {
        p_dstaddr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
        memcpy(&NetstkDstId, p_dstaddr->u8, 2);
        if (NetstkDstId == MAC_ULE_ID_INVALID) {
            *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        }
    }

    /* Issue transmission command */
    if (*p_err == NETSTK_ERR_NONE) {
        MAC_ULE_Cmd = MAC_ULE_CMD_TX;
        MAC_ULE_State = MAC_ULE_STATE_TX_STARTED;
        MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
    }
}


static void MAC_ULE_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
    /* store information regarding the received packet */
    MAC_ULE_RxPktLen = len;
    MAC_ULE_RxPktPtr = p_data;


    if (MAC_ULE_RxPktLen == 0) {
        MAC_ULE_Cmd = MAC_ULE_CMD_SLEEP;
    } else {

    }
    MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
}


static void MAC_ULE_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
{
    *p_err = NETSTK_ERR_NONE;
    switch (cmd) {
        case NETSTK_CMD_TX_CBFNCT_SET:
            MAC_ULE_TxCbFnct = (nsTxCbFnct_t)p_val;
            break;

        case NETSTK_CMD_TX_CBARG_SET:
            MAC_ULE_TxCbArg = p_val;
            break;

        default:
            MAC_ULE_Netstk->phy->ioctrl(cmd, p_val, p_err);
            break;
    }
}


/**
 * @brief   Command-processing handling
 */
static void MAC_ULE_ProcessCmd(c_event_t c_event, p_data_t p_data)
{
    /* Set the returned error code to default */
    e_nsErr_t err = NETSTK_ERR_NONE;


    switch (MAC_ULE_Cmd) {
        case MAC_ULE_CMD_NONE:
            /* Do nothing */
            break;

        case MAC_ULE_CMD_SLEEP:
            MAC_ULE_ProcessSleep(&err);
            break;

        case MAC_ULE_CMD_IDLE:
            MAC_ULE_ProcessIdle(&err);
            break;

        case MAC_ULE_CMD_SCAN:
            MAC_ULE_ProcessScan(&err);
            break;

        case MAC_ULE_CMD_TX:
            MAC_ULE_ProcessTx(&err);
            break;

        case MAC_ULE_CMD_RX:
            MAC_ULE_ProcessRx(&err);
            break;

        default:
            break;
    }
}


/*
********************************************************************************
*                               SUBMACHINES
********************************************************************************
*/
/**
 * @brief   Low-Power sleep handling
 */
static void MAC_ULE_ProcessSleep(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif

    MAC_ULE_Off(p_err);
}


/**
 * @brief   Low-Power idle handling
 */
static void MAC_ULE_ProcessIdle(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif


    MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1W,
                       MAC_ULE_CFG_IDLE_DURATION_IN_MS,
                       MAC_ULE_TmrIsr1Idle);
}


/**
 * @brief   Channel scan handling
 */
static void MAC_ULE_ProcessScan(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif

    uint8_t is_done = 0;
    LIB_TMR_STATE tmr_state;


    /* set returned error code to default */
    *p_err = NETSTK_ERR_NONE;

    /* #1 turn RF on */
    MAC_ULE_On(p_err);

    /* #2 start scan timer */
    MAC_ULE_State = MAC_ULE_STATE_SCAN_BUSY;
    Tmr_Start(&MAC_ULE_Tmr1Scan);
    LED_SCAN_ON();

    /* #3 check scan termination conditions */
    do {
        *p_err = NETSTK_ERR_NONE;
        tmr_state = Tmr_StateGet(&MAC_ULE_Tmr1Scan);
        MAC_ULE_Netstk->phy->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY,
                                    NULL,
                                    p_err);

        is_done = (*p_err != NETSTK_ERR_NONE) ||
                  (tmr_state != LIB_TMR_STATE_RUNNING);
    } while (!is_done);
    LED_SCAN_OFF();

    /* #4 Instruct LPMAC according to result of the channel scan */
    if (*p_err == NETSTK_ERR_BUSY) {
        *p_err = NETSTK_ERR_NONE;

        Tmr_Stop(&MAC_ULE_Tmr1Scan);
        MAC_ULE_Cmd = MAC_ULE_CMD_RX;
        MAC_ULE_State = MAC_ULE_STATE_RX_PARSING;
        MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1W,
                           MAC_ULE_PORT_WFP_TIMEOUT_IN_MS,
                           MAC_ULE_TmrIsr1WFS);
    } else {
        MAC_ULE_Off(p_err);
        MAC_ULE_Cmd = MAC_ULE_CMD_SLEEP;
        MAC_ULE_State = MAC_ULE_STATE_SLEEP;
        MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
    }
}


/**
 * @brief   Packet reception handling in Low-Power MAC
 */
static void MAC_ULE_ProcessRx(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif

    /* #0 Set returned error code to default */
    *p_err = NETSTK_ERR_NONE;

    /*
     * Reception submachine handling
     */
    switch (MAC_ULE_State) {
        case MAC_ULE_STATE_RX_PARSING:
            MAC_ULE_RxStrobe(p_err);
            break;

        case MAC_ULE_STATE_RX_DELAYED:
            break;

        case MAC_ULE_STATE_RX_WFP:
            MAC_ULE_RxPayload(p_err);

            if (*p_err == NETSTK_ERR_NONE) {
                /* goto state idle */
                MAC_ULE_Cmd = MAC_ULE_CMD_IDLE;
                MAC_ULE_State = MAC_ULE_STATE_IDLE;
                MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
            }
            break;

        case MAC_ULE_STATE_RX_FINISHED:
            break;

        default:
            break;
    }

    /* immediate error handling */
    if (*p_err != NETSTK_ERR_NONE) {
        MAC_ULE_Off(p_err);
        MAC_ULE_Cmd = MAC_ULE_CMD_SLEEP;
        MAC_ULE_State = MAC_ULE_STATE_SLEEP;
        MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
    }
}


/**
 * @brief   Packet transmission handling in Low-Power MAC
 */
static void MAC_ULE_ProcessTx(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif
   
    uint8_t frame_type;  
    LIB_TMR_TICK tx_delay;
    uint8_t has_tx_finished;


    /* #0 Set returned error code to default */
    *p_err = NETSTK_ERR_NONE;
    has_tx_finished = 0;

    /*
     * Transmission submachine handling
     */
    switch (MAC_ULE_State) {
        case MAC_ULE_STATE_TX_STARTED:
            /* #1 Create a Strobe */
            MAC_ULE_TxPktPtr = SmartMACFramer.Create(MAC_ULE_FRAME_TYPE_STROBE,
                                                     &MAC_ULE_TxPktLen,
                                                     &tx_delay,
                                                     p_err);
            if (tx_delay) {
                MAC_ULE_State = MAC_ULE_STATE_TX_DELAYED;
                MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1Delay,
                                   tx_delay,
                                   MAC_ULE_TmrIsr1Delay);
            } else {
                MAC_ULE_State = MAC_ULE_STATE_TX_CSMA;
                MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
            }
            break;

        case MAC_ULE_STATE_TX_DELAYED:
            break;

        case MAC_ULE_STATE_TX_CSMA:
            LED_TX_ON();
            MAC_ULE_CSMA(p_err);
            if (*p_err != NETSTK_ERR_NONE) {
                /*
                 * MAC_ULE should be switched to SCAN mode in order to check if
                 * there is any pending frames destined to the device.
                 */
            } else {
                MAC_ULE_State = MAC_ULE_STATE_TX_STROBE;
                MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
            }
            break;

        case MAC_ULE_STATE_TX_STROBE:
            MAC_ULE_TxStrobe(p_err);
            if (*p_err != NETSTK_ERR_NONE) {
                has_tx_finished = 1;
            }
            break;


        case MAC_ULE_STATE_TX_WFSA:
            /* Parse the received frame */
            SmartMACFramer.Parse(&frame_type,
                                  MAC_ULE_RxPktPtr,
                                  MAC_ULE_RxPktLen,
                                  p_err);
            if ((*p_err != NETSTK_ERR_NONE) ||
                (frame_type != MAC_ULE_FRAME_TYPE_SACK)) {
                has_tx_finished = 1;
            } else {
                Tmr_Stop(&MAC_ULE_Tmr1W);
                MAC_ULE_State = MAC_ULE_STATE_TX_DATA;
                MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
            }
            break;


        case MAC_ULE_STATE_TX_DATA:
            MAC_ULE_TxPayload(p_err);
            if (*p_err != NETSTK_ERR_NONE) {
                has_tx_finished = 1;
            }
            break;

        case MAC_ULE_STATE_TX_WFA:
            Tmr_Stop(&MAC_ULE_Tmr1W);
            MAC_ULE_RxPayload(p_err);
            if (*p_err != NETSTK_ERR_NONE) {
                MAC_ULE_State = MAC_ULE_STATE_TX_DATA;
                MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
            } else {
                has_tx_finished = 1;
            }
            break;


        case MAC_ULE_STATE_TX_FINISHED:
            has_tx_finished = 1;
            break;

        default:
            break;
    }


    if (has_tx_finished) {
        MAC_ULE_Off(p_err);

        if (MAC_ULE_TxCbFnct) {
            MAC_ULE_TxCbFnct(MAC_ULE_TxCbArg, p_err);
        }

        MAC_ULE_Cmd = MAC_ULE_CMD_SLEEP;
        MAC_ULE_State = MAC_ULE_STATE_SLEEP;
        MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
    }
}


/*
********************************************************************************
*                           INTERRUPT HANDLERS
********************************************************************************
*/
/**
 * @brief Commence an one-shot timer
 * @param timeout
 * @param isr_handler
 */
static void MAC_ULE_Tmr1Start (LIB_TMR      *p_tmr,
                               LIB_TMR_TICK  timeout,
                               FNCT_VOID     isr_handler)
{
    Tmr_Stop(p_tmr);
    Tmr_Create(p_tmr,
               LIB_TMR_TYPE_ONE_SHOT,
               timeout,
               isr_handler,
               NULL);
    Tmr_Start(p_tmr);
}


/**
 * @brief   Power-Up timer interrupt handler
 */
static void MAC_ULE_TmrIsrPowerUp(void *p_arg)
{
    if (MAC_ULE_IS_SLEEP()) {
        MAC_ULE_Cmd = MAC_ULE_CMD_SCAN;
        MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
    }
}


/**
 * @brief   Wait-For-Strobe timer interrupt handler
 */
static void MAC_ULE_TmrIsr1WFS(void *p_arg)
{
    /*
     * Used during channel scan and packet reception processes
     */
    if ((MAC_ULE_IS_SLEEP() == 0) &&
        (MAC_ULE_RxPktLen   == 0)) {
        MAC_ULE_Cmd = MAC_ULE_CMD_SLEEP;
        MAC_ULE_State = MAC_ULE_STATE_SLEEP;
        MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
    }
}


/**
 * @brief   Wait-For-ACK timer interrupt handler
 */
static void MAC_ULE_TmrIsr1WFA(void *p_arg)
{
    if (MAC_ULE_State == MAC_ULE_STATE_TX_WFSA) {
        MAC_ULE_State = MAC_ULE_STATE_TX_STROBE;
        MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
    }

    if (MAC_ULE_State == MAC_ULE_STATE_TX_WFA) {
        MAC_ULE_State = MAC_ULE_STATE_TX_FINISHED;
        MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
    }
}

/**
 * @brief   TX delay timer interrupt handler
 */
static void MAC_ULE_TmrIsr1Delay(void *p_arg)
{
    if (MAC_ULE_State == MAC_ULE_STATE_TX_DELAYED) {
        MAC_ULE_State = MAC_ULE_STATE_TX_CSMA;
        MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
    }
}

/**
 * @brief   Idle timer interrupt handler
 */
static void MAC_ULE_TmrIsr1Idle(void *p_arg)
{
    if (MAC_ULE_State == MAC_ULE_STATE_IDLE) {
        MAC_ULE_Cmd = MAC_ULE_CMD_SLEEP;
        MAC_ULE_State = MAC_ULE_STATE_SLEEP;
        MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
    }
}


/*
********************************************************************************
*                               MISCELLANEOUS
********************************************************************************
*/
static void MAC_ULE_CSMA(e_nsErr_t *p_err)
{
    uint8_t is_done;
    uint8_t cca_attempt;
    e_nsErr_t err;
    LIB_TMR_STATE tmr_state;


    /* #1 set returned error code to default */
    *p_err = NETSTK_ERR_NONE;
    is_done = 0;
    cca_attempt = 0;

    /* #2 perform CSMA */
    MAC_ULE_On(p_err);
    do {
        MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1W,
                           8,
                           0);

        *p_err = NETSTK_ERR_NONE;
        tmr_state = Tmr_StateGet(&MAC_ULE_Tmr1W);
        MAC_ULE_Netstk->phy->ioctrl(NETSTK_CMD_RF_CCA_GET,
                                    NULL,
                                    p_err);

        if (*p_err == NETSTK_ERR_NONE) {
            cca_attempt++;
            while (tmr_state != LIB_TMR_STATE_RUNNING) {
                /* wait for a certain period of time before performing the next
                 * CCA */
            }
        }

        /* check CSMA termination conditions */
        is_done = (cca_attempt > 3) ||
                  (*p_err != NETSTK_ERR_NONE);
    } while (!is_done);

    if (*p_err != NETSTK_ERR_NONE) {
        /* switch to SCAN mode */
        MAC_ULE_Off(&err);
        MAC_ULE_Cmd = MAC_ULE_CMD_SCAN;
        MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);

        if (MAC_ULE_TxCbFnct) {
            MAC_ULE_TxCbFnct(MAC_ULE_TxCbArg, p_err);
        }
    }
}


/**
 * @brief   Strobe reception handling
 */
static void MAC_ULE_RxStrobe(e_nsErr_t *p_err)
{
    uint8_t is_done;
    uint8_t frame_type;
    LIB_TMR_TICK tx_delay;
    LIB_TMR_STATE tmr_state;


    /* set returned error code to default */
    *p_err = NETSTK_ERR_NONE;

    /* #1 Parse the received packet */
     SmartMACFramer.Parse(&frame_type,
                           MAC_ULE_RxPktPtr,
                           MAC_ULE_RxPktLen,
                           p_err);
     if ((*p_err != NETSTK_ERR_NONE) ||
         (frame_type != MAC_ULE_FRAME_TYPE_STROBE)) {
         *p_err = NETSTK_ERR_LPR_NO_STROBE;
     } else {
         /* #2 Create a Strobe ACK */
         MAC_ULE_TxPktPtr = SmartMACFramer.Create(MAC_ULE_FRAME_TYPE_SACK,
                                                  &MAC_ULE_TxPktLen,
                                                  &tx_delay,
                                                  p_err);
         /* #3 reply with an ACK */
         MAC_ULE_Netstk->phy->send(MAC_ULE_TxPktPtr,
                                   MAC_ULE_TxPktLen,
                                   p_err);
         if (*p_err == NETSTK_ERR_NONE) {
             /* #4 wait for incoming data packet */
             MAC_ULE_RxPktLen = 0;
             MAC_ULE_RxPktPtr = NULL;
             MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1W,
                            MAC_ULE_PORT_WFP_TIMEOUT_IN_MS * 2,
                            MAC_ULE_TmrIsr1WFS);

             do {
                 *p_err = NETSTK_ERR_NONE;
                 tmr_state = Tmr_StateGet(&MAC_ULE_Tmr1W);
                 MAC_ULE_Netstk->phy->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY,
                                             NULL,
                                             p_err);

                 is_done = (*p_err != NETSTK_ERR_NONE) ||
                           (tmr_state != LIB_TMR_STATE_RUNNING);
             } while (!is_done);

             if (*p_err == NETSTK_ERR_BUSY) {
                 *p_err = NETSTK_ERR_NONE;

                 /* wait for a time period until the packet is totally
                  * received */
                 MAC_ULE_Cmd = MAC_ULE_CMD_RX;
                 MAC_ULE_State = MAC_ULE_STATE_RX_WFP;
                 MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1W,
                                    MAC_ULE_PORT_WFP_TIMEOUT_IN_MS * 2,
                                    MAC_ULE_TmrIsr1WFS);

             } else {
                 *p_err = NETSTK_ERR_LPR_NO_STROBE;
             }
         }
     }
}


/**
 * @brief   Strobe transmission handling
 */
static void MAC_ULE_TxStrobe(e_nsErr_t *p_err)
{
    int is_broadcast;
    uint8_t is_done;
    uint8_t is_last_strobe;
    LIB_TMR_TICK tx_delay;
    LIB_TMR_STATE tmr_wfa_state;
    LIB_TMR_STATE tmr_tx_state;


    /* set returned error code to default */
    *p_err = NETSTK_ERR_NONE;
    MAC_ULE_RxPktLen = 0;
    MAC_ULE_RxPktPtr = NULL;

    is_last_strobe = FALSE;
    is_broadcast = packetbuf_holds_broadcast();
    if (is_broadcast == FALSE) {
        /* Start strobe transmission timer only for unicasts */
        MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1Tx,
                           MAC_ULE_CFG_STROBE_TX_INTERVAL_IN_MS,
                           0);
    }

    /*
     * Send a series of wake-up preamble frames
     */
    do {
        /* start strobe transmission interval timer */
        MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1W,
                           MAC_ULE_PORT_STROBE_TX_INTERVAL_IN_MS,
                           0);

        /* transmit strobes */
        MAC_ULE_Netstk->phy->send(MAC_ULE_TxPktPtr,
                                  MAC_ULE_TxPktLen,
                                  p_err);
        if (*p_err == NETSTK_ERR_NONE) {
            do {
                /* check conditions for strobe transmission termination */
                *p_err = NETSTK_ERR_NONE;
                tmr_wfa_state = Tmr_StateGet(&MAC_ULE_Tmr1W);
                if (is_broadcast == FALSE) {
                    MAC_ULE_Netstk->phy->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY,
                                                NULL,
                                                p_err);
                    is_done = (*p_err != NETSTK_ERR_NONE) ||
                              (tmr_wfa_state != LIB_TMR_STATE_RUNNING );
                } else {
                    is_done = (tmr_wfa_state != LIB_TMR_STATE_RUNNING );
                }
            } while (!is_done);
        }

        /*
         * Strobe ACK waiting timeout.
         * It should be noted that the variable is_rx_busy is always set to
         * FALSE in case of broadcasting
         */
        if (*p_err == NETSTK_ERR_BUSY) {
            *p_err = NETSTK_ERR_NONE;

            /* wait for a time period until the packet is totally received */
            MAC_ULE_State = MAC_ULE_STATE_TX_WFSA;
            MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1W,
                               MAC_ULE_PORT_WFA_TIMEOUT_IN_MS,
                               MAC_ULE_TmrIsr1WFA);
        } else {
            /* Create a Strobe */
            MAC_ULE_TxPktPtr = SmartMACFramer.Create(MAC_ULE_FRAME_TYPE_STROBE,
                                                     &MAC_ULE_TxPktLen,
                                                     &tx_delay,
                                                     p_err);
        }

        /*
         * check conditions for strobe transmission termination
         */
        if (is_broadcast == FALSE) {
            tmr_tx_state = Tmr_StateGet(&MAC_ULE_Tmr1Tx);
            is_done = (*p_err        != NETSTK_ERR_NONE)        ||
                      (tmr_tx_state  != LIB_TMR_STATE_RUNNING)  ||
                      (MAC_ULE_State == MAC_ULE_STATE_TX_WFSA);
        } else {
            is_done = (is_last_strobe == TRUE);
        }
    } while (!is_done);


    if (is_broadcast == FALSE) {
        /*
         * if the strobe transmission was terminated due to strobe transmission
         * timeout, then MAC ULE shall conclude the transmission process has failed
         */
        if (tmr_tx_state != LIB_TMR_STATE_RUNNING) {
            *p_err = NETSTK_ERR_TX_TIMEOUT;
        }
    }
}


/**
 * @brief   Data packet reception handling
 */
static void MAC_ULE_RxPayload(e_nsErr_t *p_err)
{
    int hdrlen;
    uint8_t last_dsn;
    frame802154_t frame;
    packetbuf_attr_t is_ack_req;


    /* #0 set returned error code to default */
    *p_err = NETSTK_ERR_NONE;

    /* #1 Parse the received data packet */
    hdrlen = frame802154_parse(MAC_ULE_RxPktPtr,
                               MAC_ULE_RxPktLen,
                               &frame);
    if (hdrlen == 0) {
        *p_err = NETSTK_ERR_INVALID_FRAME;
        return;
    }

    /* #2 Process received frames */
    switch (frame.fcf.frame_type) {
        case FRAME802154_DATAFRAME:
        case FRAME802154_CMDFRAME:
            /* Auto ACK */
            if (frame.fcf.ack_required) {
                MAC_ULE_TxACK(p_err, frame.seq);
            }

            /* Store the received frame into the common packet buffer */
            packetbuf_clear();
            packetbuf_set_datalen(MAC_ULE_RxPktLen);
            memcpy(packetbuf_dataptr(),
                   MAC_ULE_RxPktPtr,
                   MAC_ULE_RxPktLen);

            /* signal the next higher layer of the received data packet */
            MAC_ULE_Netstk->llc->recv(MAC_ULE_RxPktPtr,
                                      MAC_ULE_RxPktLen,
                                      p_err);
            break;

        case FRAME802154_ACKFRAME:
            last_dsn = frame802154_getDSN();
            is_ack_req = packetbuf_attr(PACKETBUF_ATTR_RELIABLE);

            if ((is_ack_req == TRUE) &&
                (frame.seq  != last_dsn)) {
                *p_err = NETSTK_ERR_TX_NOACK;
            }
            break;

        default:
            *p_err = NETSTK_ERR_INVALID_FRAME;
            break;
    }
}


/**
 * @brief   IEEE802154 ACK transmission handling
 */
static void MAC_ULE_TxACK(e_nsErr_t *p_err, uint8_t seq)
{
    uint8_t ack[3] = {0};
    frame802154_t params;


    /* Initialize local variables */
    memset(&params, 0, sizeof(params));
    *p_err = NETSTK_ERR_NONE;

    /* Build the FCF. */
    params.fcf.frame_type = FRAME802154_ACKFRAME;
    params.fcf.security_enabled = 0;
    params.fcf.frame_pending = 0;
    params.fcf.ack_required = 0;
    params.fcf.panid_compression = 0;

    /* Insert IEEE 802.15.4 (2003) version bits. */
    params.fcf.frame_version = FRAME802154_IEEE802154_2003;

    /* set the sequence number. */
    params.seq = seq;

    /* Complete the addressing fields. */
    params.fcf.src_addr_mode = FRAME802154_NOADDR;
    params.fcf.dest_addr_mode = FRAME802154_NOADDR;

    /* Create frame to send */
    frame802154_create(&params, ack);

    /* Issue next lower layer to transmit ACK */
    MAC_ULE_Netstk->phy->send(ack, sizeof(ack), p_err);
}


/**
 * @brief
 */
static void MAC_ULE_TxPayload(e_nsErr_t *p_err)
{
    uint8_t is_tx_done;
    uint8_t is_wfa_done;
    uint8_t tx_retries;
    packetbuf_attr_t is_ack_req;
    LIB_TMR_STATE tmr_wfa_state;


    /* preparation */
    MAC_ULE_TxPktPtr = packetbuf_hdrptr();
    MAC_ULE_TxPktLen = packetbuf_totlen();
    is_ack_req = packetbuf_attr(PACKETBUF_ATTR_RELIABLE);
    tx_retries = 0;


    /* data packet transmission handling */
    do {
        /* issue data packet transmission request */
        MAC_ULE_Netstk->phy->send(MAC_ULE_TxPktPtr,
                                  MAC_ULE_TxPktLen,
                                  p_err);
        if (*p_err == NETSTK_ERR_NONE) {
            if (is_ack_req) {
                /*
                 * Wait for ACK in response to the sent data packet
                 */
                MAC_ULE_RxPktLen = 0;
                MAC_ULE_RxPktPtr = NULL;
                MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1W,
                                   MAC_ULE_PORT_WFA_TIMEOUT_IN_MS * 2,
                                   0);
                do {
                    *p_err = NETSTK_ERR_NONE;
                    tmr_wfa_state = Tmr_StateGet(&MAC_ULE_Tmr1W);
                    MAC_ULE_Netstk->phy->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY,
                                                NULL,
                                                p_err);

                    is_wfa_done = (*p_err != NETSTK_ERR_NONE) ||
                                  (tmr_wfa_state != LIB_TMR_STATE_RUNNING);
                } while (!is_wfa_done);

                /*
                 * Strobe ACK waiting timeout
                 */
                if (*p_err == NETSTK_ERR_BUSY) {
                    *p_err = NETSTK_ERR_NONE;

                    /* wait for a time period until the packet is totally
                     * received */
                    MAC_ULE_State = MAC_ULE_STATE_TX_WFA;
                    MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1W,
                                       MAC_ULE_PORT_WFA_TIMEOUT_IN_MS,
                                       MAC_ULE_TmrIsr1WFA);
                } else {
                    *p_err = NETSTK_ERR_TX_NOACK;
                }
            }
        }

        /* increase transmission attempt by one */
        tx_retries++;

        /* check retransmission conditions */
        is_tx_done = (*p_err == NETSTK_ERR_NONE) ||
                     (tx_retries > 3);
    } while (!is_tx_done);
}


/*
********************************************************************************
*                               STATE TRANSITIONS
********************************************************************************
*/



/*
********************************************************************************
*                                   END OF FILE
********************************************************************************
*/
