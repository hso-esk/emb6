/**
 * @file    xmac.c
 * @author  PN
 * @brief   X-MAC module
 */

#include <stdio.h>
#include <string.h>
#include "include.h"
#include "lib_tmr.h"


/*
********************************************************************************
*                               LOCAL DEFINES
********************************************************************************
*/
typedef uint8_t XMAC_TMR_EVENT;

#define XMAC_TMR_EVENT_NONE             (XMAC_TMR_EVENT) (0u);
#define XMAC_TMR_EVENT_SLEEP_TIMEOUT    (XMAC_TMR_EVENT) (1u);
#define XMAC_TMR_EVENT_WFLT_TIMEOUT     (XMAC_TMR_EVENT) (2u);
#define XMAC_TMR_EVENT_WFP_TIMEOUT      (XMAC_TMR_EVENT) (3u);
#define XMAC_TMR_EVENT_WFA_TIMEOUT      (XMAC_TMR_EVENT) (4u);
#define XMAC_FRAME_ACK_LEN              (uint8_t) ( 3u )
#define XMAC_FRAME_SP_LEN               (uint8_t) ( 4u )

/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static LIB_TMR          Xmac_TmrPowerup;
static LIB_TMR          Xmac_Tmr1W;
static LIB_TMR          Xmac_Tmr1Scan;
static LIB_TMR          Xmac_Tmr1TxSP;
static STK_ERR          Xmac_LastErrTx;
static XMAC_TMR_EVENT   Xmac_TmrEvent;
static uint8_t          Xmac_Ack[XMAC_FRAME_ACK_LEN];
static uint8_t          Xmac_SP [XMAC_FRAME_SP_LEN ];


/*
********************************************************************************
*                       LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void XmacInit (STK_ERR *p_err);
static void XmacOn (STK_ERR *p_err);
static void XmacOff (STK_ERR *p_err);
static void XmacSend (STK_FNCT_VOID fnct, void *arg, STK_ERR *p_err);
static void XmacRecv (void *p_buf, uint8_t len, STK_ERR *p_err);
static void XmacIsrRx (uint8_t *p_data, uint8_t len, STK_ERR *p_err);
static void XmacIoctl (STK_IOC_CMD cmd, STK_IOC_VAL *p_val, STK_ERR *p_err);
static void XmacTask (void *p_arg);

static void Xmac_TmrIsrPowerup (void *p_arg);
static void Xmac_TmrIsr1Scan (void *p_arg);
static void Xmac_TmrIsr1WFP (void *p_arg);
static void Xmac_TmrIsr1TxSP (void *p_arg);
static void Xmac_TmrIsr1WFA (void *p_arg);
static void Xmac_Tmr1Start (LIB_TMR *p_tmr, LIB_TMR_TICK timeout, FNCT_VOID isr_handler);
static void Xmac_TimestampCreate (void);
static void Xmac_TimestampProcess (uint8_t *p_data, uint8_t len);

static void Xmac_SendPayload (void);

#if 0
static void Xmac_TaskRx (void);
static void Xmac_TaskSniff (void);
#endif

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
STK_DEV_ID  XmacDestId;
STK_DEV_ID  XmacDevId;
XMAC_STATE  XmacState = XMAC_STATE_NONE;

XMAC_DRV_API XmacDrv = {
    "XMAC",
    XmacInit,
    XmacOn,
    XmacOff,
    XmacSend,
    XmacRecv,
    XmacIsrRx,
    XmacIoctl,
    XmacTask,
};



/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/

/**
 * @brief Commence an one-shot timer
 * @param timeout
 * @param isr_handler
 */
static void Xmac_Tmr1Start (LIB_TMR *p_tmr, LIB_TMR_TICK timeout, FNCT_VOID isr_handler)
{
    Tmr_Stop(p_tmr);
    Tmr_Create (p_tmr,
                LIB_TMR_TYPE_ONE_SHOT,
                timeout,
                isr_handler,
                p_tmr);
    Tmr_Start (p_tmr);
}

/**
 * @brief Timer interrupt service routine handler
 * @param p_arg
 */
static void Xmac_TmrIsrPowerup (void *p_arg)
{
    (void)&p_arg;


    if (XmacState == XMAC_STATE_SLEEP) {
        XmacState = XMAC_STATE_SCAN_STARTED;
    }
}

/**
 * @brief Timer interrupt service routine handler
 * @param p_arg
 */
static void Xmac_TmrIsr1Scan (void *p_arg)
{
    (void)&p_arg;


    if (XmacState == XMAC_STATE_SCAN_WFSP) {
        XmacState = XMAC_STATE_SCAN_DONE;
    }
}

/**
 * @brief Wait-For-Payload timeout interrupt handler
 * @param p_arg
 */
static void Xmac_TmrIsr1WFP (void *p_arg)
{
    (void)&p_arg;


    /*
     * As Wait-For-Payload timeout expires, XMAC shall terminate the idle
     * listening process.
     */
    if (XmacState == XMAC_STATE_RX_WFP) {
        XmacState = XMAC_STATE_SCAN_DONE;
    }
}

/**
 * @brief Wait-For-Ack timeout interrupt handler
 * @param p_arg
 */
static void Xmac_TmrIsr1WFA (void *p_arg)
{
    (void)&p_arg;


    if (XmacState == XMAC_STATE_TX_SPWFA) {
        XmacState = XMAC_STATE_TX_SP;
    }
    if (XmacState == XMAC_STATE_TX_PWFA) {
        XmacState = XMAC_STATE_SLEEP;
    }
}

/**
 * @brief Smart preamble transmission timeout interrupt handler
 * @param p_arg
 */
static void Xmac_TmrIsr1TxSP (void *p_arg)
{
    (void)&p_arg;


    XmacState = XMAC_STATE_TX_DONE;
    Xmac_LastErrTx = STK_ERR_TX_TIMEOUT;
}

/**
 * @brief Processing a received timestamp frame
 * @param p_data
 * @param len
 */
static void Xmac_TimestampProcess (uint8_t *p_data, uint8_t len)
{
    STK_DEV_ID      dev_id;

    /*
     * If the received timestamp is destined to us, XMAC goes to state
     * XMAC_STATE_RX_SP. Afterwards it replies the packet originator with an ACK.
     * Upon complete transmission of the ACK, the XMAC goes to state
     * XMAC_STATE_RX_WFP for a time period of XMAC_TMR_WFP_TIMEOUT
     *
     * Otherwise, the received packet shall be discarded. XMAC terminates the
     * idle listening process, going back to sleep mode.
     */

    if (XmacState == XMAC_STATE_SCAN_WFSP) {
        dev_id = (STK_DEV_ID) (p_data[0]     ) |
                 (STK_DEV_ID) (p_data[1] << 8);
        if (dev_id == XmacDevId) {
            if (p_data[2]) {
                /* TODO calculate sleeping time for the radio before
                 * turning it off */
                printf ("TODO: calculate sleeping time");
            } else {
                XmacState = XMAC_STATE_RX_SP;
            }
        } else {
            XmacState = XMAC_STATE_SCAN_DONE;
        }
    }
}

/**
 * @brief Create a timestamp frame and write it into XMAC internal buffer
 */
static void Xmac_TimestampCreate (void)
{
    Xmac_SP[1] = XmacDestId;
    Xmac_SP[2] = XmacDestId >> 8;
    Xmac_SP[3] = 0;
}


/**
 * @brief   XMAC goes to sleep state.
 */
static void Xmac_GotoSleep (void)
{
    RADIO_ERR       radio_err;
    RADIO_IOC_VAL   sync_word;


    Tmr_Stop (&Xmac_Tmr1W);
    Tmr_Stop (&Xmac_Tmr1Scan);
    Tmr_Stop (&Xmac_Tmr1TxSP);

    sync_word = RADIO_IOC_VAL_SYNC_SP;
    RadioDrv->Ioctl (RADIO_IOC_CMD_SYNC_SET, &sync_word, &radio_err);
    RadioDrv->Off(&radio_err);
    XmacState = XMAC_STATE_SLEEP;
}


/**
 * @brief   Prepare for an idle listening process
 */
static void Xmac_PrepareIdleListening (void)
{
    RADIO_ERR       radio_err;


    /*
     * Prepare for an idle listening:
     * (s1) turn RF on
     * (s2) start scan timer only when the RF is ready, and then XMAC
     *      goes to state XMAC_STATE_SCAN_WFSP.
     *      If RF is not ready, the idle listening attempt shall be
     *      terminated and XMAC goes straightforward to state
     *      XMAC_STATE_SCAN_DONE
     *
     */
    RadioDrv->On (&radio_err);
    if (radio_err == RADIO_ERR_NONE) {
        XmacState = XMAC_STATE_SCAN_WFSP;
        Xmac_Tmr1Start(&Xmac_Tmr1Scan,
                        XMAC_TMR_SCAN_DURATION,
                        Xmac_TmrIsr1Scan);
    } else {
        XmacState = XMAC_STATE_SCAN_DONE;
    }
}

/**
 * @brief   Reply the timestamp originator with an ACK
 */
static void Xmac_SendSPAck (void)
{
    RADIO_ERR       err;
    RADIO_IOC_VAL   sync_word;


    /*
     * XMAC shall issue the radio transceiver synchronization words upon successful
     * transmission of the ACK. Scan timer should be terminated here also even
     * if it probably expires before/while the smart preamble reception.
     * If the radio fails to change synchronization words, XMAC should terminate
     * the idle listening process.
     */
    RadioDrv->Send (Xmac_Ack, XMAC_FRAME_ACK_LEN, &err);
    if (err == RADIO_ERR_NONE) {
        sync_word = RADIO_IOC_VAL_SYNC_DATA;
        RadioDrv->Ioctl (RADIO_IOC_CMD_SYNC_SET, &sync_word, &err);
        if (err == RADIO_ERR_NONE) {
            XmacState = XMAC_STATE_RX_WFP;
            Xmac_Tmr1Start(&Xmac_Tmr1W,
                            XMAC_TMR_WFP_TIMEOUT,
                            Xmac_TmrIsr1WFP);
        } else {
            XmacState = XMAC_STATE_SCAN_WFSP;
        }
    } else {
        XmacState = XMAC_STATE_SCAN_WFSP;
    }
}

#if 0
/**
 * @brief Packet reception handler
 */
static void Xmac_TaskRx (void)
{

}

/**
 * @brief Periodic sniffing handler
 */
static void Xmac_TaskSniff (void)
{
    XmacState = XMAC_STATE_SCAN_WFSP;
}
#endif

/**
 * @brief Smart preamble transmission handler
 */
static void Xmac_SendSmartPreamble (void)
{
    RADIO_ERR   radio_err;


    RadioDrv->Send (Xmac_SP, XMAC_FRAME_SP_LEN, &radio_err);
    if (radio_err != RADIO_ERR_NONE) {
        XmacState = XMAC_STATE_TX_DONE;
        Xmac_LastErrTx = STK_ERR_TX_RADIO_SEND;
    } else {
        XmacState = XMAC_STATE_TX_SPWFA;
        Xmac_Tmr1Start (&Xmac_Tmr1W,
                         XMAC_TMR_WFA_TIMEOUT,
                         Xmac_TmrIsr1WFA);
    }
}


/**
 * @brief Packet transmission handler
 */
static void Xmac_SendPayload (void)
{
    RADIO_ERR       radio_err;
    RADIO_IOC_VAL   sync;


    sync = RADIO_IOC_VAL_SYNC_DATA;
    RadioDrv->Ioctl (RADIO_IOC_CMD_SYNC_SET, &sync, &radio_err);
    if (radio_err == RADIO_ERR_NONE) {
        RadioDrv->Send (StkBuf, StkBufLen, &radio_err);
        if (radio_err != RADIO_ERR_NONE) {
            Xmac_LastErrTx = STK_ERR_TX_RADIO_SEND;
        } else {
            Xmac_LastErrTx = STK_ERR_NONE;
        }
    }
    XmacState = XMAC_STATE_TX_DONE;
}


/*
********************************************************************************
*                           API FUNCTION DEFINITIONS
********************************************************************************
*/

/**
 * @brief XMAC state machine handler
 */
static void XmacTask (void *p_arg)
{
    uint8_t     is_done = 1;


    (void)&p_arg;


    RadioDrv->Task(NULL);

    /*
     * Create TaskTx, TaskRx, TaskSniff
     *
     * TaskTx pends on a send_primitive semaphore object which is then
     * posted in Send function. The pending option is Blocking.
     *
     * TaskRx pends on a packet_indication semaphore object which is then
     * posted in Input function. The pending option is Blocking.
     *
     * TaskSniff pends on a periodic timer. The pending option is Blocking.
     *
     * Using event process management module with callback function set to
     * corresponding Task function.
     * Before being able to use event process module, one must register
     * the event to pend and corresponding event handler, i.e
     * evproc_regCallback(EVENT_TYPE_PCK_LL,_rf230_callback);
     *
     * The question now is: how fast does event process module perform?
     *
     **/

    switch (XmacState) {
        case XMAC_STATE_NONE:
            /*
             * XMAC hasn't been initialized yet
             */
            break;

        case XMAC_STATE_SLEEP:
            /*
             * XMAC is sleeping, waiting for packet transmission requests from
             * the next higher layer or periodic idle listening events.
             * In this state, the radio transceiver must be switched off
             */
            break;

        case XMAC_STATE_IDLE:
            break;


        case XMAC_STATE_SCAN_STARTED:
            Xmac_PrepareIdleListening();
            LED_RX_ON();
            break;
        case XMAC_STATE_SCAN_WFSP:
            break;
        case XMAC_STATE_SCAN_DONE:
            Xmac_GotoSleep();
            LED_RX_OFF();
            break;


        case XMAC_STATE_RX_SP:
            Xmac_SendSPAck();
            break;
        case XMAC_STATE_RX_WFP:
            break;
        case XMAC_STATE_RX_P:
            HighMacDrv->IsrRx (StkBuf, StkBufLen, NULL);
            Xmac_GotoSleep();
            break;


        case XMAC_STATE_TX_SP:
            LED_TX_ON();
            Xmac_SendSmartPreamble ();
            break;
        case XMAC_STATE_TX_SPWFA:
            break;
        case XMAC_STATE_TX_P:
            Xmac_SendPayload ();
            break;
        case XMAC_STATE_TX_DONE:
            LED_TX_OFF();
            HighMacDrv->CbTx(Xmac_LastErrTx);
            Xmac_GotoSleep();
            break;


        default:
            break;
    }

    if (is_done == 0) {
        /* TODO generate events to get next turn immediately when needed */
    }
}

/**
 * @brief Initialization
 * @param p_err
 */
static void XmacInit (STK_ERR *p_err)
{
#if STKCFG_ARG_CHK_EN
    if (p_err == (STK_ERR *)0) {
        return;
    }
#endif


    *p_err = STK_ERR_NONE;

    XmacState = XMAC_STATE_SLEEP;
    Xmac_LastErrTx = STK_ERR_NONE;
    Xmac_Ack[0] = XMAC_FRAME_TYPE_ACK;
    Xmac_Ack[1] = XmacDevId;
    Xmac_Ack[2] = XmacDevId >> 8;
    Xmac_SP[0]  = XMAC_FRAME_TYPE_TIMESTAMP;
    Xmac_SP[1]  = 0;
    Xmac_SP[2]  = 0;
    Xmac_SP[3]  = 0;

    memset(&Xmac_Tmr1TxSP, 0, sizeof(Xmac_Tmr1TxSP));
    memset(&Xmac_Tmr1Scan, 0, sizeof(Xmac_Tmr1Scan));
    memset(&Xmac_Tmr1W, 0, sizeof(Xmac_Tmr1W));
    memset(&Xmac_TmrPowerup, 0, sizeof(Xmac_TmrPowerup));

#ifdef XMAC_RX
    Tmr_Create (&Xmac_TmrPowerup,
                 LIB_TMR_TYPE_PERIODIC,
                 XMAC_TMR_POWERUP_INTERVAL,
                 Xmac_TmrIsrPowerup,
                &Xmac_TmrEvent);
    Tmr_Start (&Xmac_TmrPowerup);
#endif
}

/**
 * @brief Sending a packet
 * @param p_payload
 * @param len
 * @param p_err
 */
static void XmacSend (STK_FNCT_VOID fnct, void *arg, STK_ERR *p_err)
{
    RADIO_ERR   radio_err;


#if STKCFG_ARG_CHK_EN
    if (p_err == (STK_ERR *)0) {
        return;
    }
#endif


    /* 
     * Transmit smart preamble. Upon complete reception of ACK in
     * correspond to the smart preamble, data packet is immediately sent 
     */
    if (XmacState != XMAC_STATE_SLEEP) {
        *p_err = STK_ERR_BUSY;
        return;
    }

    RadioDrv->On(&radio_err);
    if (radio_err != RADIO_ERR_NONE) {
        *p_err = STK_ERR_BUSY;
    }

    XmacState = XMAC_STATE_TX_SP;
    Xmac_Tmr1Start (&Xmac_Tmr1TxSP,
                     XMAC_TMR_TXSP_TIMEOUT,
                     Xmac_TmrIsr1TxSP);
    Xmac_TimestampCreate ();
    Xmac_LastErrTx = STK_ERR_NONE;
    *p_err = STK_ERR_NONE;
}

/**
 * @brief Receive a packet from XMAC
 * @param p_buf
 * @param len
 * @param p_err
 */
static void XmacRecv (void *p_buf, uint8_t len, STK_ERR *p_err)
{

}

/**
 * @brief Issue a XMAC command
 * @param cmd
 * @param p_val
 * @param p_err
 */
static void XmacIoctl (STK_IOC_CMD cmd, STK_IOC_VAL *p_val, STK_ERR *p_err)
{

}


/**
 * @brief   Frame reception handler
 */
static void XmacIsrRx (uint8_t *p_data, uint8_t len, STK_ERR *p_err)
{
    switch (*p_data) {
        case XMAC_FRAME_TYPE_TIMESTAMP:
            Xmac_TimestampProcess (++p_data, len);
            break;

        case XMAC_FRAME_TYPE_ACK:
            if (XmacState == XMAC_STATE_TX_SPWFA) {
                XmacState = XMAC_STATE_TX_P;
                Tmr_Stop(&Xmac_Tmr1TxSP);
                Tmr_Stop(&Xmac_Tmr1W);
            }
            break;

        default:
            if (XmacState == XMAC_STATE_RX_WFP) {
                XmacState = XMAC_STATE_RX_P;
                StkBufLen = len;
                memcpy(StkBuf, p_data, len);
            }

            break;
    }
}

/**
 * @brief Turn XMAC on
 * @param p_err
 */
static void XmacOn (STK_ERR *p_err)
{
    //RADIO_DRV->Off (NULL);
}

/**
 * @brief Turn XMAC off
 * @param p_err
 */
static void XmacOff (STK_ERR *p_err)
{
    //RADIO_DRV->On (NULL);
}
