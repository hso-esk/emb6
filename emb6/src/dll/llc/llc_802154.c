/**
 * @file    llc_802154.c
 * @date    19.10.2015
 * @author  PN
 */


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6_conf.h"


#if NETSTK_CFG_LLC_802154_EN
#include "framer_802154.h"
#include "packetbuf.h"
#include "random.h"

/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
void LLC_Init(void *p_netstk, NETSTK_ERR *p_err);
void LLC_On(NETSTK_ERR *p_err);
void LLC_Off(NETSTK_ERR *p_err);
void LLC_Send(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err);
void LLC_Recv(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err);
void LLC_IOCtrl(NETSTK_IOC_CMD cmd, void *p_val, NETSTK_ERR *p_err);


static void LLC_CbTx(void *p_arg, NETSTK_ERR *p_err);
static int LLC_IsBroadcastAddr(uint8_t mode, uint8_t *p_addr);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
/**  \brief The sequence number (0x00 - 0xff) added to the transmitted
 *   data or MAC command frame. The default is a random value within
 *   the range.
 */
static uint8_t          LLC_DSN;
static uint8_t          LLC_TxBusy;
static NETSTK_CBFNCT    LLC_CbTxFnct;
static void            *LLC_CbTxArg;
static NETSTK_CBRXFNCT  LLC_CbRxFnct;
static s_ns_t          *LLC_Netstk;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
NETSTK_MODULE_DRV LLCDrv802154 =
{
   "LLC 802154",
    LLC_Init,
    LLC_On,
    LLC_Off,
    LLC_Send,
    LLC_Recv,
    LLC_IOCtrl,
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
void LLC_Init(void *p_netstk, NETSTK_ERR *p_err)
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


    LLC_Netstk = (s_ns_t *)p_netstk;
    LLC_TxBusy = 0;
    LLC_CbRxFnct = 0;
    LLC_CbTxFnct = 0;
    LLC_CbTxArg = NULL;
    LLC_DSN = random_rand() % 256;
    *p_err = NETSTK_ERR_NONE;
}


/**
 * @brief   Turn driver on
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
void LLC_On(NETSTK_ERR *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif


    LLC_Netstk->phy->on(p_err);
}


/**
 * @brief   Turn driver off
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
void LLC_Off(NETSTK_ERR *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif


    LLC_Netstk->phy->off(p_err);
}


/**
 * @brief   Frame transmission handler
 *
 * @param   p_data      Pointer to buffer holding frame to send
 * @param   len         Length of frame to send
 * @param   p_err       Pointer to a variable storing returned error code
 */
void LLC_Send(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err)
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


    int             hdralloc;
    uint8_t         hdr_len;
    frame802154_t   params;


    /* init to zeros */
    memset(&params, 0, sizeof(params));

    /* Build the FCF. */
    params.fcf.frame_type = FRAME802154_DATAFRAME;
    params.fcf.security_enabled = 0;
    params.fcf.frame_pending = 0;
    params.fcf.ack_required = packetbuf_attr(PACKETBUF_ATTR_RELIABLE);
    params.fcf.panid_compression = 0;

    /* Insert IEEE 802.15.4 (2003) version bits. */
    params.fcf.frame_version = FRAME802154_IEEE802154_2003;

    /* Increment and set the data sequence number. */
    params.seq = LLC_DSN++;

    /* Complete the addressing fields. */
    /**
     \todo For phase 1 the addresses are all long. We'll need a mechanism
     in the rime attributes to tell the MAC to use long or short for phase 2.
     */
    params.fcf.src_addr_mode = FRAME802154_LONGADDRMODE;
    params.dest_pid = mac_phy_config.pan_id;

    if (packetbuf_holds_broadcast()) {
        /* Broadcast requires short address mode. */
        params.fcf.dest_addr_mode = FRAME802154_SHORTADDRMODE;
        params.dest_addr[0] = 0xFF;
        params.dest_addr[1] = 0xFF;
        params.fcf.ack_required = 0;
    } else {
        linkaddr_copy((linkaddr_t *) &params.dest_addr,
                      packetbuf_addr(PACKETBUF_ADDR_RECEIVER));
        params.fcf.dest_addr_mode = FRAME802154_LONGADDRMODE;
    }

    /* Set the source PAN ID to the global variable. */
    params.src_pid = mac_phy_config.pan_id;

    /*
     * Set up the source address using only the long address mode for
     * phase 1.
     */
#if NETSTACK_CONF_BRIDGE_MODE
    linkaddr_copy((linkaddr_t *)&params.src_addr,packetbuf_addr(PACKETBUF_ADDR_SENDER));
#else
    linkaddr_copy((linkaddr_t *)&params.src_addr, &linkaddr_node_addr);
#endif

    params.payload = packetbuf_dataptr();
    params.payload_len = packetbuf_datalen();
    hdr_len = frame802154_hdrlen(&params);
    hdralloc = packetbuf_hdralloc(hdr_len);
    if (hdralloc == 0) {
        *p_err = NETSTK_ERR_BUF_OVERFLOW;
        LLC_TxBusy = 0;
        return;
    }


    /*
     * Create frame to send
     */
    frame802154_create(&params,
                        packetbuf_hdrptr());


    /*
     * set TX callback function and argument
     */
    LLC_Netstk->mac->ioctrl(NETSTK_CMD_TX_CBFNCT_SET,
                            (void *)LLC_CbTx,
                            p_err);

    LLC_Netstk->mac->ioctrl(NETSTK_CMD_TX_CBARG_SET,
                            NULL,
                            p_err);

    LLC_Netstk->mac->ioctrl(NETSTK_CMD_MAC_DSN_SET,
                            &params.seq,
                            p_err);



    /*
     * Issue next lower layer to transmit the prepared frame
     */
    LLC_Netstk->mac->send(packetbuf_hdrptr(),
                          packetbuf_totlen(),
                          p_err);
}


/**
 * @brief   Frame reception handler
 *
 * @param   p_data      Pointer to buffer holding frame to receive
 * @param   len         Length of frame to receive
 * @param   p_err       Pointer to a variable storing returned error code
 */
void LLC_Recv(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err)
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
    int hdrlen, ret;


    hdrlen = frame802154_parse(p_data, len, &frame);
    if (hdrlen == 0) {
        *p_err = NETSTK_ERR_INVALID_FRAME;
        return;
    }

    ret = packetbuf_hdrreduce(len - frame.payload_len);
    if (ret == 0) {
        *p_err = NETSTK_ERR_FATAL;
        return;
    }

    /*
     * Check frame addresses
     */
    if (frame.fcf.dest_addr_mode) {
        if ((frame.dest_pid != mac_phy_config.pan_id) &&
            (frame.dest_pid != FRAME802154_BROADCASTPANDID)) {
            *p_err = NETSTK_ERR_FATAL;
            return;
        }

        ret = LLC_IsBroadcastAddr(frame.fcf.dest_addr_mode, frame.dest_addr);
        if (ret == 0) {
            packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER,
                               (linkaddr_t *)&frame.dest_addr);

#if !NETSTACK_CONF_BRIDGE_MODE
            ret = linkaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                               &linkaddr_node_addr);
            if (ret == 0) {
                *p_err = NETSTK_ERR_FATAL;
                return;     /* Not for this node */
            }
#endif
        }
    }
    packetbuf_set_addr(PACKETBUF_ADDR_SENDER,
                       (linkaddr_t *)&frame.src_addr);


    /*
     * Signal next higher layer of the valid received frame
     */
#if 1
    LLC_CbRxFnct(packetbuf_dataptr(),
                 packetbuf_datalen(),
                 p_err);
#else
    LLC_Netstk->llc->recv(packetbuf_dataptr(),
                          packetbuf_datalen(),
                          p_err);
#endif
}


/**
 * @brief    Miscellaneous commands handler
 *
 * @param   cmd         Command to be issued
 * @param   p_val       Pointer to a variable related to the command
 * @param   p_err       Pointer to a variable storing returned error code
 */
void LLC_IOCtrl(NETSTK_IOC_CMD cmd, void *p_val, NETSTK_ERR *p_err)
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
                LLC_CbTxFnct = (NETSTK_CBFNCT)p_val;
            }
            break;

        case NETSTK_CMD_TX_CBARG_SET:
            LLC_CbTxArg = p_val;
            break;

        case NETSTK_CMD_RX_CBFNT_SET:
            if (p_val == NULL) {
                *p_err = NETSTK_ERR_INVALID_ARGUMENT;
            } else {
                LLC_CbRxFnct = (NETSTK_CBRXFNCT)p_val;
            }
            break;

        case NETSTK_CMD_LLC_XXX:
            break;

        default:
            LLC_Netstk->phy->ioctrl(cmd, p_val, p_err);
            break;
    }
}


/**
 *
 * @param ptr
 * @param status
 * @param transmissions
 */
static void LLC_CbTx(void *p_arg, NETSTK_ERR *p_err)
{
    LLC_TxBusy = 0;
    if (LLC_CbTxFnct) {
        LLC_CbTxFnct(LLC_CbTxArg, p_err);
    }
}


/**
 *
 * @param mode
 * @param addr
 * @return
 */
static int LLC_IsBroadcastAddr(uint8_t mode, uint8_t *p_addr)
{
    int i = mode == FRAME802154_SHORTADDRMODE ? 2 : 8;
    while (i-- > 0) {
        if (p_addr[i] != 0xff) {
            return 0;
        }
    }
    return 1;
}


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
#endif /* NETSTK_CFG_LLC_802154_EN */
