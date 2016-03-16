/*
 * emb6 is licensed under the 3-clause BSD license. This license gives everyone
 * the right to use and distribute the code, either in binary or source code
 * format, as long as the copyright license is retained in the source code.
 *
 * The emb6 is derived from the Contiki OS platform with the explicit approval
 * from Adam Dunkels. However, emb6 is made independent from the OS through the
 * removal of protothreads. In addition, APIs are made more flexible to gain
 * more adaptivity during run-time.
 *
 * The license text is:
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*============================================================================*/

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


/*
********************************************************************************
*                               LOCAL ENUMS
********************************************************************************
*/
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
    MAC_ULE_STATE_RX_DELAYED,
    MAC_ULE_STATE_RX_WFP,
    MAC_ULE_STATE_RX_FINISHED,

    /* TX Submachine states */
    MAC_ULE_STATE_TX_STARTED,
    MAC_ULE_STATE_TX_DELAYED,
    MAC_ULE_STATE_TX_STROBE,
    MAC_ULE_STATE_TX_FINISHED
}MAC_ULE_STATE;


/*
********************************************************************************
*                               LOCAL MACROS
********************************************************************************
*/
#define MAC_ULE_EVENT_POST(_e)         evproc_putEvent(E_EVPROC_HEAD, _e, NULL)


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

static void MAC_ULE_CSMA(e_nsErr_t *p_err);
static void MAC_ULE_TxBroadcast(e_nsErr_t *p_err);
static void MAC_ULE_TxUnicast(e_nsErr_t *p_err);
static void MAC_ULE_TxPayload(e_nsErr_t *p_err);

static void MAC_ULE_IdleListening(c_event_t c_event, p_data_t p_data);
static void MAC_ULE_ChannelScan(e_nsErr_t *p_err);
static void MAC_ULE_RxPayload(e_nsErr_t *p_err);
static void MAC_ULE_RxBroadcast(e_nsErr_t *p_err);
static void MAC_ULE_RxUnicast(e_nsErr_t *p_err);
static void MAC_ULE_TxAck(uint8_t seq, e_nsErr_t *p_err);

static void MAC_ULE_Tmr1Start(LIB_TMR *p_tmr, LIB_TMR_TICK timeout, FNCT_VOID isr_handler);
static void MAC_ULE_TmrIsrPowerUp(void *p_arg);
static void MAC_ULE_RxBufRead(e_nsErr_t *p_err);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static  s_ns_t         *MAC_ULE_Netstk;
static  MAC_ULE_STATE   MAC_ULE_State;

static  void           *MAC_ULE_TxCbArg;
static  nsTxCbFnct_t    MAC_ULE_TxCbFnct;

static  uint8_t        *MAC_ULE_RxPktPtr;
static  uint16_t        MAC_ULE_RxPktLen;

/*
 * Timers
 */
static  LIB_TMR         MAC_ULE_TmrPowerUp;
static  LIB_TMR         MAC_ULE_Tmr1Scan;
static  LIB_TMR         MAC_ULE_Tmr1Csma;
static  LIB_TMR         MAC_ULE_Tmr1Delay;
static  LIB_TMR         MAC_ULE_Tmr1Wait;


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

extern uip_lladdr_t uip_lladdr;


/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
/**
 * @brief   Initialization of MAC ULE
 * @param   p_netstk    pointer to variable holding netstack structure
 * @param   p_err   point to variable holding returned error code
 */
static void MAC_ULE_Init (void *p_netstk, e_nsErr_t *p_err)
{
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
    MAC_ULE_TxCbArg = NULL;
    MAC_ULE_TxCbFnct = 0;
    MAC_ULE_RxPktPtr = NULL;
    MAC_ULE_RxPktLen = 0;
    MAC_ULE_State = MAC_ULE_STATE_SLEEP;

    /*
     * Initialize APSS framer
     */
    NetstkDstId = MAC_ULE_ID_INVALID;
    memcpy(&NetstkSrcId, mac_phy_config.mac_address, 2);
    SmartMACFramer.Init(p_err);

    /*
     * Configure stack address
     */
    memcpy(&uip_lladdr.addr, &mac_phy_config.mac_address, 8);
    linkaddr_set_node_addr((linkaddr_t *)mac_phy_config.mac_address);

    /* Register event */
    evproc_regCallback(NETSTK_MAC_ULE_EVENT, MAC_ULE_IdleListening);

    /*
     * Configure timers
     */
    Tmr_Create(&MAC_ULE_TmrPowerUp,
                LIB_TMR_TYPE_PERIODIC,
                MAC_ULE_CFG_POWERUP_INTERVAL_IN_MS,
                MAC_ULE_TmrIsrPowerUp,
                NULL);
    Tmr_Start(&MAC_ULE_TmrPowerUp);
}


/**
 * @brief   Turn MAC ULE on
 * @param   p_err   point to variable holding returned error code
 */
static void MAC_ULE_On(e_nsErr_t *p_err)
{
    MAC_ULE_Netstk->phy->on(p_err);
}


/**
 * @brief   Turn MAC ULE off
 * @param   p_err   point to variable holding returned error code
 */
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


    int is_broadcast;

    if ((MAC_ULE_State != MAC_ULE_STATE_SLEEP)) {
        *p_err = NETSTK_ERR_BUSY;
        return;
    }

    /* set returned error to default */
    *p_err = NETSTK_ERR_NONE;
    MAC_ULE_State = MAC_ULE_STATE_TX_STARTED;
    MAC_ULE_On(p_err);

    /*
     * This function shall be operating in blocking manner.
     * The reason is that the sicslowpan module expects to learn the result of
     * transmission process right after the function is executed.
     */
    is_broadcast = packetbuf_holds_broadcast();
    if (is_broadcast == 1) {
        MAC_ULE_TxBroadcast(p_err);
    } else {
        MAC_ULE_TxUnicast(p_err);
    }
    MAC_ULE_State = MAC_ULE_STATE_TX_FINISHED;

    /*
     * Finalize the transmission process
     */
    MAC_ULE_Off(p_err);
    MAC_ULE_State = MAC_ULE_STATE_SLEEP;

    /* signal the caller of result of the transmission attempt */
    if (MAC_ULE_TxCbFnct) {
        MAC_ULE_TxCbFnct(MAC_ULE_TxCbArg, p_err);
    }
}


/**
 * @brief   MAC ULE frame reception handling
 * @param   p_data  point to buffer holding the received frame
 * @param   len     length of the received frame
 * @param   p_err   point to variable holding the received error code
 */
static void MAC_ULE_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
    /* store information regarding the received packet */
    MAC_ULE_RxPktLen = len;
    MAC_ULE_RxPktPtr = p_data;
}


/**
 * @brief   MAC ULE Input/Output Control
 * @param   cmd     indicates control command
 * @param   p_val   point to variable to be used by the command
 * @param   p_err   point to variable holding returned error code
 */
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
 * @brief   Handles idle listening operation
 */
static void MAC_ULE_IdleListening(c_event_t c_event, p_data_t p_data)
{
    e_nsErr_t err;

    /*
     * (S1) Preparation
     */
    LED_SCAN_ON();
    MAC_ULE_On(&err);
    MAC_ULE_State = MAC_ULE_STATE_SCAN_BUSY;

    /*
     * (S2) Perform channel scan
     */
    err = NETSTK_ERR_NONE;
    MAC_ULE_ChannelScan(&err);
    if (err == NETSTK_ERR_NONE) {
        MAC_ULE_RxPayload(&err);
    }

    /*
     * (S3) Finalize channel scan process
     */
    MAC_ULE_Off(&err);
    MAC_ULE_State = MAC_ULE_STATE_SLEEP;
    LED_SCAN_OFF();
}


/**
 * @brief   Channel scan handling
 * @param   p_err Point to variable holding returned error code
 */
static void MAC_ULE_ChannelScan(e_nsErr_t *p_err)
{
    uint8_t frame_type;
    uint8_t is_scan_done;
    uint8_t rx_valid_frame;
    LIB_TMR_STATE tmr_scan_state;

    /*
     * (S1)     Channel scan handling
     *
     *          A channel scan attempt is declared as finished only if one of
     *          following conditions is met:
     *          (a)     A valid strobe has arrived
     *          (b)     Scan timeout is over
     */
    is_scan_done = FALSE;
    rx_valid_frame = FALSE;
    MAC_ULE_RxPktLen = 0;
    MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1Scan, MAC_ULE_PORT_SCAN_DURATION_IN_MS, 0);
    do {
        /* get answer to condition (S1a) */
        MAC_ULE_RxBufRead(p_err);
        if (MAC_ULE_RxPktLen > 0) {
            /* a packet has arrived then parse it */
            SmartMACFramer.Parse(&frame_type,
                                  MAC_ULE_RxPktPtr,
                                  MAC_ULE_RxPktLen,
                                  p_err);
            /* only strobe frame is of interest */
            rx_valid_frame = (frame_type == MAC_ULE_FRAME_TYPE_STROBE) &&
                             (*p_err == NETSTK_ERR_NONE);
        }

        /* get answer to condition (S1b) */
        tmr_scan_state = Tmr_StateGet(&MAC_ULE_Tmr1Scan);

        /* check for scan termination */
        is_scan_done = (rx_valid_frame == TRUE) ||
                       (tmr_scan_state != LIB_TMR_STATE_RUNNING);
    } while (is_scan_done == FALSE);
    Tmr_Stop(&MAC_ULE_Tmr1Scan);

    /*
     * (S2)     Set returned error code
     */
    if (rx_valid_frame == TRUE) {
        *p_err = NETSTK_ERR_NONE;
    } else {
        *p_err = NETSTK_ERR_MAC_ULE_NO_STROBE;
    }
}


/**
 * @brief   Handling of actual data payload reception
 * @param   p_err   Point to variable holding returned error code
 */
static void MAC_ULE_RxPayload(e_nsErr_t *p_err)
{
    uint8_t *p_pkt;
    uint16_t pkt_len;
    LIB_TMR_TICK delay;
    LIB_TMR_STATE tmr_rx_state;


    /*
     * (S1)     Create ACK in correspond to the received strobe
     */
    p_pkt = SmartMACFramer.Create(MAC_ULE_FRAME_TYPE_SACK, &pkt_len, &delay, p_err);

    /*
     * (S2)     Actual data packet reception
     */
    if (*p_err == NETSTK_ERR_MAC_ULE_BROADCAST_NOACK) {
        /* reception delay handling */
        if (delay) {
            /* turn RF off */
            LED_SCAN_OFF();
            MAC_ULE_Off(p_err);
            MAC_ULE_State = MAC_ULE_STATE_RX_DELAYED;

            /* stay in SLEEP until estimated timeout of the actual packet
             * arrival is over */
            MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1Delay, delay, 0);
            do {
                tmr_rx_state = Tmr_StateGet(&MAC_ULE_Tmr1Delay);
            } while (tmr_rx_state == LIB_TMR_STATE_RUNNING );

            /* timeout is over, turn RF on again to receive the packet */
            MAC_ULE_On(p_err);
            LED_SCAN_ON();
        }

        /* broadcast packet reception */
        MAC_ULE_State = MAC_ULE_STATE_RX_WFP;
        MAC_ULE_RxBroadcast(p_err);
    } else {
        /* unicast packet reception */
        MAC_ULE_Netstk->phy->send(p_pkt, pkt_len, p_err);   /* reply with an ACK */
        MAC_ULE_State = MAC_ULE_STATE_RX_WFP;
        MAC_ULE_RxUnicast(p_err);
    }

    /*
     * (S3)     Signal upper layer only when the reception was successful
     */
    if (*p_err == NETSTK_ERR_NONE) {
        /* Store the received frame into the common packet buffer */
        packetbuf_clear();
        packetbuf_set_datalen(MAC_ULE_RxPktLen);
        memcpy(packetbuf_dataptr(),
               MAC_ULE_RxPktPtr,
               MAC_ULE_RxPktLen);

        /* signal the next higher layer of the received data packet */
        MAC_ULE_Netstk->dllc->recv(MAC_ULE_RxPktPtr,
                                   MAC_ULE_RxPktLen,
                                   p_err);
    }
}


/**
 * @brief   Broadcast reception handling
 * @param   p_err   Point to variable holding returned error code
 */
static void MAC_ULE_RxBroadcast(e_nsErr_t *p_err)
{
    uint8_t has_frame;
    uint8_t iteration;
    uint8_t frame_type;
    uint8_t is_rx_done;
    uint8_t rx_valid_frame;
    LIB_TMR_STATE tmr_rx_state;


    /*
     * The waiting for payload process shall be declared as done if one of
     * following conditions is met:
     * (a)  A valid data frame has arrived
     * (b)  Waiting timeout is over
     * (c)  After having received 3 frames and none of them is of interest
     */
    iteration = 3;
    while (iteration > 0) {
        /* the device shall wait for incoming packets for a limited amount of time */
        has_frame = FALSE;
        is_rx_done = FALSE;
        MAC_ULE_RxPktLen = 0;
        MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1Wait, MAC_ULE_PORT_WFP_TIMEOUT_IN_MS, 0);
        do {
            /* find out the answer for condition (a) */
            MAC_ULE_RxBufRead(p_err);
            if (MAC_ULE_RxPktLen > 0) {
                /* a packet has arrived then parse it */
                has_frame = TRUE;
                SmartMACFramer.Parse(&frame_type,
                                      MAC_ULE_RxPktPtr,
                                      MAC_ULE_RxPktLen,
                                      p_err);
                rx_valid_frame = (*p_err == NETSTK_ERR_MAC_ULE_UNSUPPORTED_FRAME);
            }

            /* find out the answer for condition (b) */
            tmr_rx_state = Tmr_StateGet(&MAC_ULE_Tmr1Wait);

            /* check for process termination */
            is_rx_done = (rx_valid_frame == TRUE) ||
                         (tmr_rx_state != LIB_TMR_STATE_RUNNING);
        } while (is_rx_done == FALSE);
        Tmr_Stop(&MAC_ULE_Tmr1Wait);

        /*
         * the waiting process is declared as done if one of following
         * conditions is met:
         * (a)  No frame has arrived during wait period
         * (b)  A valid frame has arrived
         */
        if ((has_frame == FALSE) || (rx_valid_frame == TRUE)) {
            break;
        } else {
            iteration--;
        }
    }

    /* Set returned error code */
    if (rx_valid_frame == TRUE) {
        *p_err = NETSTK_ERR_NONE;
    } else {
        *p_err = NETSTK_ERR_FATAL;
    }
}


/**
 * @brief   Unicast reception handling
 * @param   p_err   Point to variable holding returned error code
 */
static void MAC_ULE_RxUnicast(e_nsErr_t *p_err)
{
    uint8_t frame_type;
    uint8_t is_rx_done;
    uint8_t rx_valid_frame;
    LIB_TMR_STATE tmr_rx_state;
    int hdrlen;
    frame802154_t frame;


    /*
     * The waiting for payload process shall be declared as done if one of
     * following conditions is met:
     * (a)  A valid data frame has arrived
     * (b)  Waiting timeout is over
     */
    is_rx_done = FALSE;
    rx_valid_frame = FALSE;
    MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1Wait, MAC_ULE_PORT_WFP_TIMEOUT_IN_MS, 0);
    do {
        /* find out the answer for condition (a) */
        MAC_ULE_RxPktLen = 0;
        MAC_ULE_Netstk->phy->ioctrl(NETSTK_CMD_RX_BUF_READ, NULL, p_err);
        if (MAC_ULE_RxPktLen) {
            /* a packet has arrived then parse it */
            SmartMACFramer.Parse(&frame_type,
                                  MAC_ULE_RxPktPtr,
                                  MAC_ULE_RxPktLen,
                                  p_err);
            rx_valid_frame = (*p_err == NETSTK_ERR_MAC_ULE_UNSUPPORTED_FRAME);
        }

        /* find out the answer for condition (b) */
        tmr_rx_state = Tmr_StateGet(&MAC_ULE_Tmr1Wait);

        /* check for waiting process termination */
        is_rx_done = (rx_valid_frame == TRUE) ||
                     (tmr_rx_state != LIB_TMR_STATE_RUNNING);
    } while (is_rx_done == FALSE);
    Tmr_Stop(&MAC_ULE_Tmr1Wait);

    /* Set returned error code */
    if (rx_valid_frame == TRUE) {
        /* parse the received data packet */
        hdrlen = frame802154_parse(MAC_ULE_RxPktPtr, MAC_ULE_RxPktLen, &frame);
        if (hdrlen == 0) {
            *p_err = NETSTK_ERR_INVALID_FRAME;
            return;
        } else {
            /*
             * Auto-ACK
             */
            if ((frame.fcf.frame_type == FRAME802154_DATAFRAME) ||
                (frame.fcf.frame_type == FRAME802154_CMDFRAME)) {
                if (frame.fcf.ack_required) {
                    MAC_ULE_TxAck(frame.seq, p_err);
                }
            }
            *p_err = NETSTK_ERR_NONE;
        }
    } else {
        *p_err = NETSTK_ERR_FATAL;
    }
}


/**
 * @brief   ACK frame transmission
 * @param   seq     sequence number in the ACK frame
 * @param   p_err   point to variable holding returned error code
 */
static void MAC_ULE_TxAck(uint8_t seq, e_nsErr_t *p_err)
{
    frame802154_t frame;
    uint8_t hdr_len, alloc_ok;


    /* initialize */
    *p_err = NETSTK_ERR_NONE;
    memset(&frame, 0, sizeof(frame));
    packetbuf_clear();

    /* Build the FCF. */
    frame.fcf.frame_type = FRAME802154_ACKFRAME;
    frame.fcf.security_enabled = 0;
    frame.fcf.frame_pending = 0;
    frame.fcf.ack_required = 0;
    frame.fcf.panid_compression = 0;

    /* Insert IEEE 802.15.4 (2003) version bits. */
    frame.fcf.frame_version = FRAME802154_IEEE802154_2003;

    /* Increment and set the data sequence number. */
    frame.seq = seq;

    /* Complete the addressing fields. */
    frame.fcf.src_addr_mode = FRAME802154_NOADDR;
    frame.fcf.dest_addr_mode = FRAME802154_NOADDR;

    /* allocate buffer for MAC header */
    hdr_len = frame802154_hdrlen(&frame);
    alloc_ok = packetbuf_hdralloc(hdr_len);
    if (alloc_ok == 0) {
        *p_err = NETSTK_ERR_BUF_OVERFLOW;
        return;
    }

    /* write the header */
    frame802154_create(&frame, packetbuf_hdrptr());

    /* Issue next lower layer to transmit ACK */
    MAC_ULE_Netstk->phy->send(packetbuf_hdrptr(),
                              packetbuf_totlen(),
                              p_err);
}


/**
 * @brief   Broadcast transmission request handling
 * @param   p_err   point to variable holding returned error code
 */
static void MAC_ULE_TxBroadcast(e_nsErr_t *p_err)
{
    uint8_t is_last_pkt;
    uint8_t is_tx_done;
    uint8_t *p_pkt;
    uint16_t pkt_len;
    LIB_TMR_TICK delay;
    LIB_TMR_STATE tmr_state;

    /* set destination ID for outgoing broadcast strobes */
    NetstkDstId = MAC_ULE_DEV_ID_BROADCAST;

    /*
     * (S1)     Perform CSMA
     */
    MAC_ULE_CSMA(p_err);
    if (*p_err == NETSTK_ERR_NONE) {
        /*
         * (S2)     Transmit strobes
         */
        is_tx_done = FALSE;
        do {
            p_pkt = SmartMACFramer.Create(MAC_ULE_FRAME_TYPE_STROBE,
                                          &pkt_len,
                                          &delay,
                                          p_err);
            is_last_pkt = (*p_err == NETSTK_ERR_MAC_ULE_LAST_STROBE);

            /* start strobe transmission interval timer */
            MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1Wait,
                               MAC_ULE_PORT_STROBE_TX_INTERVAL_IN_MS,
                               0);

            /* transmit strobes */
            MAC_ULE_Netstk->phy->send(p_pkt, pkt_len, p_err);
            if (*p_err == NETSTK_ERR_NONE) {
                /* wait until strobe transmission interval passes */
                do {
                    tmr_state = Tmr_StateGet(&MAC_ULE_Tmr1Wait);
                } while (tmr_state == LIB_TMR_STATE_RUNNING);
            }

            /*
             * Strobe transmission process is declared as finished if one
             * of following conditions is met:
             * (1)  Last packet is transmitted
             * (2)  A strobe has failed to transmit
             */
            is_tx_done = (is_last_pkt == TRUE) ||
                         (*p_err != NETSTK_ERR_NONE);
        } while (is_tx_done == FALSE);


        /*
         * (S3)     Transmit the actual broadcast packet after last strobe
         *          was successfully transmitted
         */
        if ((is_last_pkt == TRUE) &&
            (*p_err == NETSTK_ERR_NONE)) {
            MAC_ULE_Netstk->phy->send(packetbuf_hdrptr(),
                                      packetbuf_totlen(),
                                      p_err);
        }
    }
}


/**
 * @brief   Unicast transmission request handling
 * @param   p_err   point to variable holding returned error code
 */
static void MAC_ULE_TxUnicast(e_nsErr_t *p_err)
{
    uint8_t frame_type;
    uint8_t rx_valid_frame;
    uint8_t is_rx_done;
    uint8_t is_last_pkt;
    uint8_t is_tx_done;
    uint8_t *p_pkt;
    uint16_t pkt_len;
    LIB_TMR_TICK delay;
    LIB_TMR_STATE tmr_rx_state;
    const linkaddr_t *p_dstaddr;


    /* set destination ID for outgoing broadcast strobes */
    p_dstaddr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
    memcpy(&NetstkDstId, p_dstaddr->u8, 2);
    if (NetstkDstId == MAC_ULE_ID_INVALID) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }

    /*
     * (S1)     Perform CSMA
     */
    MAC_ULE_CSMA(p_err);
    if (*p_err == NETSTK_ERR_NONE) {
        /*
         * (S2)     Transmit strobes
         */
        delay = 0;
        rx_valid_frame = FALSE;
        is_tx_done = FALSE;
        do {
            /* create wake-up strobe */
            p_pkt = SmartMACFramer.Create(MAC_ULE_FRAME_TYPE_STROBE,
                                          &pkt_len,
                                          &delay,
                                          p_err);
            is_last_pkt = (*p_err == NETSTK_ERR_MAC_ULE_LAST_STROBE);

#if (MAC_ULE_CFG_LOOSE_SYNC_EN == TRUE)
            /*
             * Transmission delay handling
             */
            if (delay > 0) {
                /* turn RF off */
                LED_SCAN_OFF();
                MAC_ULE_Off(p_err);
                MAC_ULE_State = MAC_ULE_STATE_TX_DELAYED;

                /* stay in SLEEP until estimated timeout of the actual packet
                 * arrival is over */
                MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1Delay, delay, 0);
                do {
                    tmr_rx_state = Tmr_StateGet(&MAC_ULE_Tmr1Delay);
                } while (tmr_rx_state == LIB_TMR_STATE_RUNNING );

                /* timeout is over, turn RF on again to receive the packet */
                MAC_ULE_State = MAC_ULE_STATE_TX_STROBE;
                MAC_ULE_On(p_err);
                LED_SCAN_ON();
            }
#endif

            /* start strobe TX interval timer */
            MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1Wait, MAC_ULE_PORT_STROBE_TX_INTERVAL_IN_MS, 0);

            /* transmit strobes */
            MAC_ULE_Netstk->phy->send(p_pkt, pkt_len, p_err);
            if (*p_err == NETSTK_ERR_NONE) {
                /* wait for strobe ACK until strobe TX interval timeout is expired */
                is_rx_done = FALSE;
                rx_valid_frame = FALSE;
                MAC_ULE_RxPktLen = 0;
                do {
                    /* check if a valid frame has arrived */
                    MAC_ULE_RxBufRead(p_err);
                    if (MAC_ULE_RxPktLen > 0) {
                        /* a packet has arrived then parse it */
                        SmartMACFramer.Parse(&frame_type,
                                              MAC_ULE_RxPktPtr,
                                              MAC_ULE_RxPktLen,
                                              p_err);
                        rx_valid_frame = (*p_err == NETSTK_ERR_NONE) &&
                                         (frame_type == MAC_ULE_FRAME_TYPE_SACK );
                    }

                    /* check if strobe TX interval timeout is over */
                    tmr_rx_state = Tmr_StateGet(&MAC_ULE_Tmr1Wait);

                    /* check if Wait-For-ACK process is complete */
                    is_rx_done = (rx_valid_frame == TRUE)   ||
                                 (tmr_rx_state != LIB_TMR_STATE_RUNNING );
                } while (is_rx_done == FALSE);

                /*
                 * Strobe transmission process is declared as finished if one
                 * of following conditions is met:
                 * (1)  Last packet is transmitted
                 * (2)  An ACK in correspond to previously sent strobe was received
                 * (3)  A strobe has failed to transmit
                 */
                is_tx_done = (is_last_pkt == TRUE)  ||
                             (rx_valid_frame == TRUE);
            } else {
                is_tx_done = TRUE;
            }
            Tmr_Stop(&MAC_ULE_Tmr1Wait);
        } while (is_tx_done == FALSE);

        /*
         * (S3)     Transmit the actual data packet
         */
        if (rx_valid_frame == TRUE) {
            MAC_ULE_TxPayload(p_err);
        }
    }
}


/**
 * @brief   Actual data packet handling
 * @param   p_err   point to variable holding returned error code
 */
static void MAC_ULE_TxPayload(e_nsErr_t *p_err)
{
    int hdrlen;
    uint8_t last_dsn;
    uint8_t is_acked;
    uint8_t is_rx_done;
    frame802154_t frame;
    packetbuf_attr_t is_ack_req;
    LIB_TMR_STATE tmr_rx_state;


    /* send data packet stored in packetbuf module */
    MAC_ULE_Netstk->phy->send(packetbuf_hdrptr(), packetbuf_totlen(), p_err);

    /* wait for data ACK if needed */
    is_ack_req = packetbuf_attr(PACKETBUF_ATTR_RELIABLE);
    if (is_ack_req == 1) {
        /*
         * Auto-ACK is declared as done if one of following conditions
         * is met:
         * (a)  A corresponding ACK has arrived
         * (b)  Wait-for-ACK timeout is expired
         */
        is_acked = FALSE;
        is_rx_done = FALSE;
        last_dsn = packetbuf_attr(PACKETBUF_ATTR_PACKET_ID);

        MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1Wait, 15, 0);
        do {
            /* condition (a) */
            MAC_ULE_RxPktLen = 0;
            MAC_ULE_Netstk->phy->ioctrl(NETSTK_CMD_RX_BUF_READ, NULL, p_err);
            if (MAC_ULE_RxPktLen) {
                /* parse the received frame */
                hdrlen = frame802154_parse(MAC_ULE_RxPktPtr, MAC_ULE_RxPktLen, &frame);
                if (hdrlen > 0) {
                    is_acked = (frame.fcf.frame_type == FRAME802154_ACKFRAME) &&
                               (frame.seq == last_dsn);
                }
            }
            /* condition (b) */
            tmr_rx_state = Tmr_StateGet(&MAC_ULE_Tmr1Wait);

            /* check if Auto-ACK has finished */
            is_rx_done = (is_acked == TRUE) ||
                         (tmr_rx_state != LIB_TMR_STATE_RUNNING );
        } while (is_rx_done == FALSE);
        Tmr_Stop(&MAC_ULE_Tmr1Wait);

        /* set returned error code in accordance with the result of transmission
         * process */
        if (is_acked == FALSE) {
            *p_err = NETSTK_ERR_TX_NOACK;
        } else {
            *p_err = NETSTK_ERR_NONE;
        }
    }
}


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
    if (MAC_ULE_State == MAC_ULE_STATE_SLEEP) {
        MAC_ULE_State = MAC_ULE_STATE_SCAN_STARTED;
        MAC_ULE_EVENT_POST(NETSTK_MAC_ULE_EVENT);
    }
}


/**
 * @brief   MAC ULE CSMA handling
 * @param   p_err   point to variable holding returned error code
 */
static void MAC_ULE_CSMA(e_nsErr_t *p_err)
{
    uint8_t is_done;
    uint8_t attempt;
    LIB_TMR_STATE tmr_cca_state;


    /* CSMA handling */
    is_done = FALSE;
    attempt = 0;
    do {
        /* start CCA operation interval timer */
        MAC_ULE_Tmr1Start(&MAC_ULE_Tmr1Csma, 3, 0);

        /* perform CCA */
        *p_err = NETSTK_ERR_NONE;
        MAC_ULE_Netstk->phy->ioctrl(NETSTK_CMD_RF_CCA_GET, NULL, p_err);
        if (*p_err == NETSTK_ERR_NONE) {
            attempt++;
            /* wait for a certain period of time before performing the next CCA */
            do {
                tmr_cca_state = Tmr_StateGet(&MAC_ULE_Tmr1Csma);
            } while (tmr_cca_state == LIB_TMR_STATE_RUNNING);
        }

        /* check CSMA termination conditions */
        is_done = (attempt > 3) ||
                  (*p_err != NETSTK_ERR_NONE);
    } while (is_done == FALSE);
}


/**
 * @brief   Poll lower layer for incoming frames
 * @param   p_err   point to variable holding returned error code
 */
static void MAC_ULE_RxBufRead(e_nsErr_t *p_err)
{
    MAC_ULE_Netstk->phy->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY, NULL, p_err);
    if (*p_err == NETSTK_ERR_BUSY) {
        /* wait until RF has completed reception process */
        do {
            MAC_ULE_Netstk->phy->ioctrl(NETSTK_CMD_RX_BUF_READ, NULL, p_err);
        } while (MAC_ULE_RxPktLen == 0);
    }
}


/*
********************************************************************************
*                                   END OF FILE
********************************************************************************
*/
