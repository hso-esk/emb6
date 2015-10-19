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
#include "emb6_conf.h"


#if NETSTK_CFG_MAC_802154_EN
#include "evproc.h"
#include "frame802154.h"
#include "packetbuf.h"
#include "random.h"

#include "lib_tmr.h"
#define MAC_CFG_REFACTOR_EN                 ( 0u )

#define MAC_EVENT_TX_ACK                    ( 2u )  /* ACK transmission     */
#define MAC_EVENT_RX_ACK_TIMEOUT            ( 3u )  /* Wait-For-ACK timeout */

#define MAC_SEM_POST(_event_)               evproc_putEvent(E_EVPROC_HEAD, _event_, NULL)
#define MAC_SEM_WAIT(_event_, _fcnt_)       evproc_regCallback(_event_, _fcnt_)



/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
void MAC_Init(void *p_netstk, NETSTK_ERR *p_err);
void MAC_On(NETSTK_ERR *p_err);
void MAC_Off(NETSTK_ERR *p_err);
void MAC_Send(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err);
void MAC_Recv(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err);
void MAC_IOCtrl(NETSTK_IOC_CMD cmd, void *p_val, NETSTK_ERR *p_err);


static void MAC_CbTx(void *p_arg, NETSTK_ERR *p_err);
static uint8_t MAC_IsACKed(frame802154_t *p_frame);
static void MAC_TxACK(uint8_t seq);
static void MAC_TaskAutoACK(c_event_t c_event, p_data_t p_data);
static void MAC_IsrTmrACK(void *p_arg);

/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
/**  \brief The sequence number (0x00 - 0xff) added to the transmitted
 *   data or MAC command frame. The default is a random value within
 *   the range.
 */
static uint8_t          MAC_ACKReq;
static uint8_t          MAC_LastSeq;
static LIB_TMR          MAC_Tmr1ACK;
static NETSTK_CBFNCT    MAC_CbTxFnct;
static void            *MAC_CbTxArg;
static s_ns_t          *MAC_Netstk;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
NETSTK_MODULE_DRV MACDrv802154 =
{
   "MAC 802154",
    MAC_Init,
    MAC_On,
    MAC_Off,
    MAC_Send,
    MAC_Recv,
    MAC_IOCtrl,
};


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
void MAC_Init(void *p_netstk, NETSTK_ERR *p_err)
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
    *p_err = NETSTK_ERR_NONE;

    Tmr_Stop(&MAC_Tmr1ACK);
    Tmr_Create(&MAC_Tmr1ACK,
               LIB_TMR_TYPE_ONE_SHOT,
               20,
               MAC_IsrTmrACK,
               NULL);
    MAC_SEM_WAIT(MAC_EVENT_TX_ACK, MAC_TaskAutoACK);
}


/**
 * @brief   Turn driver on
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
void MAC_On(NETSTK_ERR *p_err)
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
void MAC_Off(NETSTK_ERR *p_err)
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
void MAC_Send(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err)
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


    packetbuf_attr_t is_ack_req;
    is_ack_req = packetbuf_attr(PACKETBUF_ATTR_RELIABLE);
    if (is_ack_req) {
        MAC_ACKReq = 1;
    }


    /*
     * set TX callback function and argument
     */
    MAC_Netstk->phy->ioctrl(NETSTK_CMD_TX_CBFNCT_SET,
                            (void *)MAC_CbTx,
                            p_err);

    MAC_Netstk->phy->ioctrl(NETSTK_CMD_TX_CBARG_SET,
                            NULL,
                            p_err);

    /*
     * Issue next lower layer to transmit the prepared frame
     */
    MAC_Netstk->phy->send(p_data, len, p_err);
}


/**
 * @brief   Frame reception handler
 *
 * @param   p_data      Pointer to buffer holding frame to receive
 * @param   len         Length of frame to receive
 * @param   p_err       Pointer to a variable storing returned error code
 */
void MAC_Recv(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err)
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


    frame802154_t frame;
    int hdrlen;
    uint8_t is_acked;


    hdrlen = frame802154_parse(p_data, len, &frame);
    if (hdrlen == 0) {
        *p_err = NETSTK_ERR_INVALID_FRAME;
        return;
    }

    switch (frame.fcf.frame_type) {
        case FRAME802154_DATAFRAME:
        case FRAME802154_CMDFRAME:
            if (frame.fcf.ack_required) {
                MAC_ACKReq = 1;
                MAC_LastSeq = frame.seq;
                MAC_SEM_POST(MAC_EVENT_TX_ACK);
            }

            /*
             * Signal the higher layer of the received frame
             */
            MAC_Netstk->llc->recv(p_data, len, p_err);
            break;

        case FRAME802154_ACKFRAME:
            if (MAC_ACKReq) {
                MAC_ACKReq = 0;
                is_acked = MAC_IsACKed(&frame);
                if (is_acked) {
                    *p_err = NETSTK_ERR_NONE;
                } else {
                    *p_err = NETSTK_ERR_TX_NOACK;
                }

                /*
                 * Signal the higher layer of the transmitted frame
                 */
                MAC_CbTxFnct(MAC_CbTxArg, p_err);
            }
            break;

        default:
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
void MAC_IOCtrl(NETSTK_IOC_CMD cmd, void *p_val, NETSTK_ERR *p_err)
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
                MAC_CbTxFnct = (NETSTK_CBFNCT)p_val;
            }
            break;

        case NETSTK_CMD_TX_CBARG_SET:
            MAC_CbTxArg = p_val;
            break;

        case NETSTK_CMD_MAC_DSN_SET:
            MAC_LastSeq = *((uint8_t *)p_val);
            break;

        default:
            MAC_Netstk->phy->ioctrl(cmd, p_val, p_err);
            break;
    }
}


/**
 *
 * @param ptr
 * @param status
 * @param transmissions
 */
static void MAC_CbTx(void *p_arg, NETSTK_ERR *p_err)
{
    if (MAC_ACKReq) {
        Tmr_Stop(&MAC_Tmr1ACK);
        Tmr_Start(&MAC_Tmr1ACK);
    }
}


/**
 * @brief   Verifying received frame for ACK
 */
static uint8_t MAC_IsACKed(frame802154_t *p_frame)
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
 */
static void MAC_TxACK(uint8_t seq)
{
    NETSTK_ERR      err;
    uint8_t         ack[3];
    frame802154_t   params;

    /* init to zeros */
    memset(&params, 0, sizeof(params));

    /* Build the FCF. */
    params.fcf.frame_type = FRAME802154_ACKFRAME;
    params.fcf.security_enabled = 0;
    params.fcf.frame_pending = 0;
    params.fcf.ack_required = 0;
    params.fcf.panid_compression = 0;

    /* Insert IEEE 802.15.4 (2003) version bits. */
    params.fcf.frame_version = FRAME802154_IEEE802154_2003;

    /* Increment and set the data sequence number. */
    params.seq = seq;

    /* Complete the addressing fields. */
    /**
     \todo For phase 1 the addresses are all long. We'll need a mechanism
     in the rime attributes to tell the MAC to use long or short for phase 2.
     */
    params.fcf.src_addr_mode = FRAME802154_NOADDR;
    params.fcf.dest_addr_mode = FRAME802154_NOADDR;

    /*
     * Create frame to send
     */
    frame802154_create(&params, ack);

    /*
     * Issue next lower layer to transmit ACK
     */
    MAC_Netstk->phy->send(ack, sizeof(ack), &err);
}


static void MAC_TaskAutoACK(c_event_t c_event, p_data_t p_data)
{
    if ((c_event == MAC_EVENT_TX_ACK) &&
        (MAC_ACKReq)) {
        MAC_TxACK(MAC_LastSeq);
    }

    if (c_event == MAC_EVENT_RX_ACK_TIMEOUT) {
        NETSTK_ERR err = NETSTK_ERR_TX_NOACK;
        MAC_CbTxFnct(MAC_CbTxArg, &err);
    }
}


static void MAC_IsrTmrACK(void *p_arg)
{
    if (MAC_ACKReq) {
        MAC_ACKReq = 0;
        MAC_SEM_POST(MAC_EVENT_RX_ACK_TIMEOUT);
    }
}


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
#endif /* #if NETSTK_MAC_802154_EN */
