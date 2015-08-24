/**
 * @file    xmac.c
 * @author  PN
 * @brief   X-MAC module
 */

#include <stdio.h>
#include "include.h"
#include "tmr.h"


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
static UTIL_TMR         XMAC_TmrPowerup;
static UTIL_TMR         XMAC_TmrRx1;
static UTIL_TMR         XMAC_TmrTx1;
static XMAC_TMR_EVENT   XMAC_TmrEvent;
static uint8_t          XMAC_Ack[XMAC_FRAME_ACK_LEN];
static uint8_t          XMAC_SP [XMAC_FRAME_SP_LEN ];


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
static void Xmac_Tmr1Start (UTIL_TMR *p_tmr, UTIL_TMR_TICK timeout, FNCT_VOID isr_handler);
static void Xmac_TimestampCreate (void);
static void Xmac_TimestampProcess (uint8_t *p_data, uint8_t len);

static void Xmac_TaskTx (void);

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
XMAC_STATE  XmacState = XMAC_STATE_CREATED;

const XMAC_DRV_API XmacDrv = {
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
static void Xmac_Tmr1Start (UTIL_TMR *p_tmr, UTIL_TMR_TICK timeout, FNCT_VOID isr_handler)
{
    Tmr_Create (p_tmr,
                UTIL_TMR_TYPE_ONE_SHORT,
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
    RADIO_ERR   radio_err;


    (void)&p_arg;


    if (XmacState == XMAC_STATE_SLEEP) {
        XmacState  = XMAC_STATE_SCAN;

        RadioDrv->On (&radio_err);
        Xmac_Tmr1Start (&XMAC_TmrRx1,
                         XMAC_TMR_SCAN_DURATION,
                         Xmac_TmrIsr1Scan);
    }
}

/**
 * @brief Timer interrupt service routine handler
 * @param p_arg
 */
static void Xmac_TmrIsr1Scan (void *p_arg)
{
    RADIO_ERR   radio_err;


    /* Scan timeout expires */
    RadioDrv->Off (&radio_err);
    XmacState  = XMAC_STATE_SLEEP;
}

/**
 * @brief Wait-For-Payload timeout interrupt handler
 * @param p_arg
 */
static void Xmac_TmrIsr1WFP (void *p_arg)
{
    RADIO_IOC_VAL sync;


    if (XmacState == XMAC_STATE_RX) {
        XmacState = XMAC_STATE_SLEEP;
        sync = RADIO_IOC_VAL_SYNC_SP;
        RadioDrv->Ioctl (RADIO_IOC_CMD_SYNC_SET, &sync, NULL);

        /* TODO set MCU to sleep mode */
    }
}

/**
 * @brief Wait-For-Ack timeout interrupt handler
 * @param p_arg
 */
static void Xmac_TmrIsr1WFA (void *p_arg)
{
    if (XmacState == XMAC_STATE_TXSPWFA) {
        XmacState = XMAC_STATE_TXSP;
    }

    if (XmacState == XMAC_STATE_TXWFA) {
        XmacState = XMAC_STATE_SLEEP;
    }
}

/**
 * @brief Smart preamble transmission timeout interrupt handler
 * @param p_arg
 */
static void Xmac_TmrIsr1TxSP (void *p_arg)
{
    if (XmacState == XMAC_STATE_TXSP) {
        XmacState  = XMAC_STATE_SLEEP;
        RadioDrv->Off (NULL);
    }
}

/**
 * @brief Processing a received timestamp frame
 * @param p_data
 * @param len
 */
static void Xmac_TimestampProcess (uint8_t *p_data, uint8_t len)
{
    STK_DEV_ID      dev_id;
    RADIO_IOC_VAL   sync;

    if (XmacState == XMAC_STATE_SCAN) {
        dev_id = (STK_DEV_ID) (p_data[0]     ) |
                 (STK_DEV_ID) (p_data[1] << 8);
        if (dev_id == XmacDevId) {
            if (p_data[2]) {
                /* TODO calculate sleeping time for the radio before
                 * turning it off */
                printf ("TODO: calculate sleeping time");
            } else {
                /* TODO reply with an ACK before going back to sleep
                 * until payload is completely received or RX timeout
                 * expires */
                RadioDrv->Send (XMAC_Ack, XMAC_FRAME_ACK_LEN, NULL);

                XmacState = XMAC_STATE_RX;
                sync = RADIO_IOC_VAL_SYNC_DATA;
                RadioDrv->Ioctl (RADIO_IOC_CMD_SYNC_SET, &sync, NULL);

                Xmac_Tmr1Start (&XMAC_TmrRx1,
                                 XMAC_TMR_WFP_TIMEOUT,
                                 Xmac_TmrIsr1WFP);
            }

        } else {
            printf ("TODO: set MCU to sleep mode");
        }
    }
}

/**
 * @brief Create a timestamp frame and write it into XMAC internal buffer
 */
static void Xmac_TimestampCreate (void)
{
    XMAC_SP[1] = XmacDestId;
    XMAC_SP[2] = XmacDestId >> 8;
    XMAC_SP[3] = 0;
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
static void Xmac_TaskTxSP (void)
{
    RADIO_ERR   radio_err;


    RadioDrv->Send (XMAC_SP, XMAC_FRAME_SP_LEN, &radio_err);
    if (radio_err != RADIO_ERR_NONE) {
        /* TODO inform upper layer of failed transmission attempt */
        XmacState = XMAC_STATE_SLEEP;
    } else {
        XmacState = XMAC_STATE_TXSPWFA;
        Xmac_Tmr1Start (&XMAC_TmrRx1,
                         XMAC_TMR_WFA_TIMEOUT,
                         Xmac_TmrIsr1WFA);
    }
}


/**
 * @brief Packet transmission handler
 */
static void Xmac_TaskTx (void)
{
    RADIO_ERR       radio_err;
    RADIO_IOC_VAL   sync;


    sync = RADIO_IOC_VAL_SYNC_DATA;
    RadioDrv->Ioctl (RADIO_IOC_CMD_SYNC_SET, &sync, &radio_err);

    RadioDrv->Send (StkBuf, StkBufLen, &radio_err);
    if (radio_err != RADIO_ERR_NONE) {
        /* TODO inform upper layer of failed transmission attempt */
        XmacState = XMAC_STATE_SLEEP;
    } else {
        XmacState = XMAC_STATE_IDLE;
    }
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
        case XMAC_STATE_CREATED:
            /* TODO XMAC is just created */
            break;

        case XMAC_STATE_SLEEP:
            /* TODO turn RF transceiver off */
            break;

        case XMAC_STATE_IDLE:
            break;

                                                    /* Idle listening           */
        case XMAC_STATE_SCAN:
            break;

        case XMAC_STATE_SCAN_WFSP:
            break;

        case XMAC_STATE_RX:
            /* TODO receive smart preamble or data payload */
            break;

                                                    /* Packet transmission      */
        case XMAC_STATE_TXSP:
            Xmac_TaskTxSP ();
            break;

        case XMAC_STATE_TXSPWFA:
            break;

        case XMAC_STATE_TX:
            Xmac_TaskTx ();
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
    XmacState = XMAC_STATE_SLEEP;

    XMAC_Ack[0] = XMAC_FRAME_TYPE_ACK;
    XMAC_Ack[1] = XmacDevId;
    XMAC_Ack[2] = XmacDevId >> 8;

    XMAC_SP[0]  = XMAC_FRAME_TYPE_TIMESTAMP;
    XMAC_SP[1]  = 0;
    XMAC_SP[2]  = 0;
    XMAC_SP[3]  = 0;

    Tmr_Create (&XMAC_TmrPowerup,
                 UTIL_TMR_TYPE_PERIODIC,
                 XMAC_TMR_POWERUP_INTERVAL,
                 Xmac_TmrIsrPowerup,
                &XMAC_TmrEvent);

    Tmr_Start (&XMAC_TmrPowerup);

    if (p_err) {
        *p_err = STK_ERR_NONE;
    }
}

/**
 * @brief Sending a packet
 * @param p_payload
 * @param len
 * @param p_err
 */
static void XmacSend (STK_FNCT_VOID fnct, void *arg, STK_ERR *p_err)
{
    if (XmacState == XMAC_STATE_SLEEP) {
        XmacState  = XMAC_STATE_TXSP;
        /* Transmit smart preamble. Upon complete reception of ACK in
         * correspond to the smart preamble, data packet is immediately sent */
        Xmac_Tmr1Start (&XMAC_TmrTx1,
                         XMAC_TMR_TXSP_TIMEOUT,
                         Xmac_TmrIsr1TxSP);
        Xmac_TimestampCreate ();

    } else {
        if (p_err) {
            *p_err = STK_ERR_BUSY;
        }
    }
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
    RADIO_IOC_VAL sync;


    switch (*p_data) {
        case XMAC_FRAME_TYPE_TIMESTAMP:
            Xmac_TimestampProcess (++p_data, len);
            break;

        case XMAC_FRAME_TYPE_ACK:
            if (XmacState == XMAC_STATE_TXSPWFA) {
                XmacState  = XMAC_STATE_TX;
            }
            break;

        default:
            /* A data packet has arrived */

            if (XmacState == XMAC_STATE_RX) {
                XmacState = XMAC_STATE_SLEEP;
                Tmr_Stop (&XMAC_TmrRx1);

                sync = RADIO_IOC_VAL_SYNC_SP;
                RadioDrv->Ioctl (RADIO_IOC_CMD_SYNC_SET, &sync, NULL);
                RadioDrv->Off (NULL);

                HighMacDrv->IsrRx (p_data, len, NULL);
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
