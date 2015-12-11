/**
 * @file    mac_802154.c
 * @date    19.10.2015
 * @author  PN
 */


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"

#include "evproc.h"
#include "framer_802154.h"
#include "packetbuf.h"
#include "random.h"

#include "lib_tmr.h"

#define     LOGGER_ENABLE        LOGGER_MAC
#include    "logger.h"


/*
********************************************************************************
*                               LOCAL MACROS
********************************************************************************
*/
#define MAC_CFG_REFACTOR_EN                 ( 0u )

#define MAC_CFG_TX_RETRY_MAX                (uint8_t  )( 3u )
#define MAC_CFG_TMR_WFA_IN_MS               (uint32_t )( 40 )

#define MAC_EVENT_PEND(_event_)             evproc_regCallback(_event_, MAC_EventHandler)
#define MAC_EVENT_POST(_event_)             evproc_putEvent(E_EVPROC_HEAD, _event_, NULL)


/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
static void MAC_Init(void *p_netstk, e_nsErr_t *p_err);
static void MAC_On(e_nsErr_t *p_err);
static void MAC_Off(e_nsErr_t *p_err);
static void MAC_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void MAC_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void MAC_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);

static uint8_t MAC_IsAcked(frame802154_t *p_frame);
static void MAC_TxACK(uint8_t seq, e_nsErr_t *p_err);
static void MAC_EventHandler(c_event_t c_event, p_data_t p_data);
static void MAC_IsrTmrACK(void *p_arg);
static void MAC_CSMA(e_nsErr_t *p_err);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
/**  \brief The sequence number (0x00 - 0xff) added to the transmitted
 *   data or MAC command frame. The default is a random value within
 *   the range.
 */
static uint8_t          MAC_IsAckReq;
static uint8_t          MAC_TxRetries;
static uint8_t          MAC_LastSeq;
static LIB_TMR          MAC_Tmr1ACK;
static s_ns_t          *MAC_Netstk;
static void            *MAC_CbTxArg;
static nsTxCbFnct_t     MAC_CbTxFnct;
static e_nsErr_t        MAC_LastErr;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsMAC_t MACDrv802154 =
{
   "MAC 802154",
    MAC_Init,
    MAC_On,
    MAC_Off,
    MAC_Send,
    MAC_Recv,
    MAC_IOCtrl,
};

extern uip_lladdr_t uip_lladdr;


/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/

/**
 * @brief   Initialize driver
 *
 * @param   p_netstk    Pointer to netstack structure
 * @param   p_err       Pointer to a variable storing returned error code
 */
void MAC_Init(void *p_netstk, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }

    if (p_netstk == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }
#endif


    MAC_Netstk = (s_ns_t *)p_netstk;
    MAC_CbTxFnct = 0;
    MAC_CbTxArg = NULL;
    MAC_IsAckReq = 0;
    MAC_LastErr = NETSTK_ERR_NONE;
    *p_err = NETSTK_ERR_NONE;

    Tmr_Stop(&MAC_Tmr1ACK);
    Tmr_Create(&MAC_Tmr1ACK,
               LIB_TMR_TYPE_ONE_SHOT,
               MAC_CFG_TMR_WFA_IN_MS,
               MAC_IsrTmrACK,
               NULL);

    /*
     * Configure stack address
     */
    memcpy(&uip_lladdr.addr, &mac_phy_config.mac_address, 8);
    linkaddr_set_node_addr((linkaddr_t *)mac_phy_config.mac_address);

    /*
     * Register events
     */
    MAC_EVENT_PEND(NETSTK_MAC_EVENT_RX);
    MAC_EVENT_PEND(NETSTK_MAC_EVENT_TX_DONE);
    MAC_EVENT_PEND(NETSTK_MAC_EVENT_ACK_TIMEOUT);
}


/**
 * @brief   Turn driver on
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
void MAC_On(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif


    MAC_Netstk->phy->on(p_err);
}


/**
 * @brief   Turn driver off
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
void MAC_Off(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif


    MAC_Netstk->phy->off(p_err);
}


/**
 * @brief   Frame transmission handler
 *
 * @param   p_data      Pointer to buffer holding frame to send
 * @param   len         Length of frame to send
 * @param   p_err       Pointer to a variable storing returned error code
 */
void MAC_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
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


#if LOGGER_ENABLE
    /*
     * Logging
     */
    uint16_t data_len = len;
    uint8_t *p_dataptr = p_data;
    LOG_RAW("MAC_TX: ");
    while (data_len--) {
        LOG_RAW("%02x", *p_dataptr++);
    }
    LOG_RAW("\r\n");
#endif


    packetbuf_attr_t is_ack_req;
    int is_broadcast;

    /*
     * Collect information required for Auto-ACK as well as Auto-Retransmission
     * mechanisms.
     */
    is_broadcast = packetbuf_holds_broadcast();
    is_ack_req = packetbuf_attr(PACKETBUF_ATTR_RELIABLE);
    if (is_ack_req && !is_broadcast) {
        MAC_IsAckReq = 1;
    }
    MAC_TxRetries = 0;
    MAC_LastErr = NETSTK_ERR_NONE;
    MAC_LastSeq = frame802154_getDSN();

    /*
     * Issue next lower layer to transmit the prepared frame
     */
    MAC_CSMA(&MAC_LastErr);
    if (*p_err != NETSTK_ERR_CHANNEL_ACESS_FAILURE) {
        MAC_Netstk->phy->send(p_data, len, &MAC_LastErr);
    }

    if (MAC_LastErr != NETSTK_ERR_NONE) {
        MAC_EVENT_POST(NETSTK_MAC_EVENT_TX_DONE);
    } else {
        /* check if ACK is required */
        if (MAC_IsAckReq) {
            Tmr_Stop(&MAC_Tmr1ACK);
            Tmr_Start(&MAC_Tmr1ACK);
        } else {
            MAC_EVENT_POST(NETSTK_MAC_EVENT_TX_DONE);
        }
    }
}


/**
 * @brief   Frame reception handler
 *
 * @param   p_data      Pointer to buffer holding frame to receive
 * @param   len         Length of frame to receive
 * @param   p_err       Pointer to a variable storing returned error code
 */
void MAC_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
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


#if LOGGER_ENABLE
    /*
     * Logging
     */
    uint16_t data_len = len;
    uint8_t *p_dataptr = p_data;
    LOG_RAW("MAC_RX: ");
    while (data_len--) {
        LOG_RAW("%02x", *p_dataptr++);
    }
    LOG_RAW("\r\n");
#endif


    int hdrlen;
    uint8_t is_acked;
    frame802154_t frame;

    /* set returned error code to default */
    *p_err = NETSTK_ERR_NONE;

    /* Parsing but not reducing header as that will be then handled by DLLC */
    hdrlen = frame802154_parse(p_data, len, &frame);
    if (hdrlen == 0) {
        *p_err = NETSTK_ERR_INVALID_FRAME;
        return;
    }

    switch (frame.fcf.frame_type) {
        case FRAME802154_DATAFRAME:
        case FRAME802154_CMDFRAME:
            if (frame.fcf.ack_required) {
                MAC_TxACK(frame.seq, p_err);
                if (*p_err != NETSTK_ERR_NONE) {
                    emb6_errorHandler(p_err);
                }
            }

            /* store the received packet into the common packet buffer */
            packetbuf_clear();
            packetbuf_set_datalen(len);
            memcpy(packetbuf_dataptr(),
                   p_data,
                   len);
            MAC_EVENT_POST(NETSTK_MAC_EVENT_RX);
            break;

        case FRAME802154_ACKFRAME:
            if (MAC_IsAckReq) {
                is_acked = MAC_IsAcked(&frame);
                if (is_acked) {
                    Tmr_Stop(&MAC_Tmr1ACK);
                    MAC_LastErr = NETSTK_ERR_NONE;
                    MAC_EVENT_POST(NETSTK_MAC_EVENT_TX_DONE);
                }
            }
            break;

        default:
            *p_err = NETSTK_ERR_INVALID_FRAME;
            //emb6_errorHandler(p_err);
            break;
    }
}


/**
 * @brief    Miscellaneous commands handler
 *
 * @param   cmd         Command to be issued
 * @param   p_val       Pointer to a variable related to the command
 * @param   p_err       Pointer to a variable storing returned error code
 */
void MAC_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif


    *p_err = NETSTK_ERR_NONE;
    switch (cmd) {
        case NETSTK_CMD_TX_CBFNCT_SET:
            if (p_val == NULL) {
                *p_err = NETSTK_ERR_INVALID_ARGUMENT;
            } else {
                MAC_CbTxFnct = (nsTxCbFnct_t)p_val;
            }
            break;

        case NETSTK_CMD_TX_CBARG_SET:
            MAC_CbTxArg = p_val;
            break;

        default:
            MAC_Netstk->phy->ioctrl(cmd, p_val, p_err);
            break;
    }
}


/**
 * @brief   Verifying received frame for ACK
 *
 * @param   p_frame     Pointer to structure holding information of the received
 *                      frame
 */
static uint8_t MAC_IsAcked(frame802154_t *p_frame)
{
    uint8_t is_acked = 0;

    if ((p_frame->fcf.frame_type == FRAME802154_ACKFRAME) &&
        (p_frame->seq == MAC_LastSeq)) {
        is_acked = 1;
    }

    return is_acked;
}


/**
 * @brief   ACK transmission
 *
 * @param   seq     Frame sequence number of the outgoing ACK
 */
static void MAC_TxACK(uint8_t seq, e_nsErr_t *p_err)
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
    Tmr_Delay(1);
    MAC_Netstk->phy->send(packetbuf_hdrptr(),
                          packetbuf_totlen(),
                          p_err);
}


/**
 * @brief   Driver's internal events handler
 *
 * @param   c_event     Event to handle
 * @param   p_data      Pointer to the registered data in combination with the
 *                      event to handle
 */
static void MAC_EventHandler(c_event_t c_event, p_data_t p_data)
{
    e_nsErr_t err = NETSTK_ERR_NONE;


    switch (c_event) {
        case NETSTK_MAC_EVENT_RX:
            /* Inform the next higher layer of the received packet */
            MAC_Netstk->dllc->recv(packetbuf_dataptr(),
                                   packetbuf_datalen(),
                                   &err);
            break;

        case NETSTK_MAC_EVENT_TX_DONE:
            MAC_IsAckReq = 0;
            MAC_TxRetries = 0;
            MAC_CbTxFnct(MAC_CbTxArg, &MAC_LastErr);
            MAC_LastErr = NETSTK_ERR_NONE;
            break;

        case NETSTK_MAC_EVENT_ACK_TIMEOUT:
            if (++MAC_TxRetries > MAC_CFG_TX_RETRY_MAX) {
                /*
                 * Transmission attempts has finished
                 */
                MAC_LastErr = NETSTK_ERR_TX_NOACK;
                MAC_EVENT_POST(NETSTK_MAC_EVENT_TX_DONE);
            } else {
                /*
                 * retransmission
                 */
                MAC_CSMA(&MAC_LastErr);
                if (MAC_LastErr != NETSTK_ERR_CHANNEL_ACESS_FAILURE) {
                    /* Issue PHY to transmit the last packet */
                    MAC_Netstk->phy->ioctrl(NETSTK_CMD_PHY_LAST_PKT_TX, NULL, &err);
                }

                if (MAC_LastErr != NETSTK_ERR_NONE) {
                    MAC_EVENT_POST(NETSTK_MAC_EVENT_TX_DONE);
                } else {
                    if (MAC_IsAckReq) {
                        Tmr_Stop(&MAC_Tmr1ACK);
                        Tmr_Start(&MAC_Tmr1ACK);
                    } else {
                        MAC_EVENT_POST(NETSTK_MAC_EVENT_TX_DONE);
                    }
                }
            }
            break;
    }
}


/**
 * @brief   Wait-For-ACK timeout interrupt handler
 *
 * @param   p_arg   Pointer to the registered callback argument
 */
static void MAC_IsrTmrACK(void *p_arg)
{
    if (MAC_IsAckReq) {
        MAC_EVENT_POST(NETSTK_MAC_EVENT_ACK_TIMEOUT);
    }
}


/**
 * @brief   This function performs CSMA-CA mechanism.
 *
 * @param   p_err   Pointer to a variable storing returned error code
 */
static void MAC_CSMA(e_nsErr_t *p_err)
{
    uint32_t unit_backoff;
    uint32_t nb;
    uint32_t be;
    uint32_t min_be = 3;
    uint32_t max_backoff = 3;
    uint32_t delay = 0;
    uint32_t max_random;


    nb = 0;
    be = min_be;
    unit_backoff = 20 * 20; /* Unit backoff period = 20 * symbol periods [us] */
    *p_err = NETSTK_ERR_NONE;

    while (nb <= max_backoff) {                                     /* NB > MaxBackoff: Failure     */
        /*
         * Delay for random (2^BE - 1) unit backoff periods
         */
        max_random = (1 << be) - 1;
        delay = bsp_getrand(max_random);
        delay *= unit_backoff; // us, symbol period is 20us @50kbps
        bsp_delay_us(delay);


        /*
         * Perform CCA
         */
        MAC_Netstk->phy->ioctrl(NETSTK_CMD_RF_CCA_GET, 0, p_err);
        if (*p_err == NETSTK_ERR_NONE) {                            /* Channel idle ?               */
            break;                                                  /*   Success                    */
        } else {                                                    /* Else                         */
            nb++;                                                   /*   NB = NB + 1                */
            be = ((be + 1) < min_be) ? (be + 1) : (min_be);         /*   BE = min(BE + 1, MinBE)    */
        }
    }
}


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
