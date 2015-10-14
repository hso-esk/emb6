/*
 * simple_mac.c
 *
 *  Created on: 01.10.2015
 *      Author: nphuong
 */

#include "emb6_conf.h"

#include "sicslowmac.h"
#include "frame802154.h"
#include "packetbuf.h"
#include "random.h"

/*
********************************************************************************
*                       LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void SimpleMAC_Init(s_ns_t *p_netstack);
static void SimpleMAC_Send(mac_callback_t cbsent, void *p_arg);
static void SimpleMAC_Input(void);
static int8_t SimpleMAC_On(void);
static int8_t SimpleMAC_Off(int keep_radio_on);
static unsigned short SimpleMAC_ChanChkInterval(void);

static void SimpleMAC_CbTx(void *p_cbarg, int status, int transmissions);
static int SimpleMAC_IsBroadcastAddr(uint8_t mode, uint8_t *p_addr);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
/**  \brief The sequence number (0x00 - 0xff) added to the transmitted
 *   data or MAC command frame. The default is a random value within
 *   the range.
 */
static uint8_t          SimpleMAC_DSN;
static uint8_t          SimpleMAC_TxBusy;
static mac_callback_t   SimpleMAC_CbFnct;
static void            *SimpleMAC_CbArg;
static s_ns_t          *SimpleMAC_NetStack = NULL;


/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
/**
 *
 * @param ptr
 * @param status
 * @param transmissions
 */
static void SimpleMAC_CbTx(void *p_cbarg, int status, int transmissions)
{
    if (status == STK_ERR_NONE) {
        status = MAC_TX_OK;
    }

    if (SimpleMAC_CbFnct) {
        SimpleMAC_CbFnct(p_cbarg, status, transmissions);
    }
}


/**
 *
 * @param mode
 * @param addr
 * @return
 */
static int SimpleMAC_IsBroadcastAddr(uint8_t mode, uint8_t *p_addr)
{
    int i = mode == FRAME802154_SHORTADDRMODE ? 2 : 8;
    while (i-- > 0) {
        if (p_addr[i] != 0xff) {
            return 0;
        }
    }
    return 1;
}


/**
 *
 * @param sent
 * @param ptr
 */
static void SimpleMAC_Send(mac_callback_t cbsent, void *p_arg)
{
    uint8_t         hdr_len;
    STK_ERR         err;
    frame802154_t   params;


    if (cbsent == 0) {
        *((STK_ERR *)p_arg) = STK_ERR_INVALID_ARGUMENT;
        return;
    }

    if (SimpleMAC_TxBusy) {
        *((STK_ERR *)p_arg) = STK_ERR_BUSY;
        return;
    }


    SimpleMAC_CbFnct = cbsent;
    SimpleMAC_CbArg = p_arg;

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
    params.seq = SimpleMAC_DSN++;

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

    int hdralloc = 0;
    hdralloc = packetbuf_hdralloc(hdr_len);
    if (hdralloc == 0) {
        *((STK_ERR *)p_arg) = STK_ERR_BUSY;
        return;
    }

    frame802154_create(&params, packetbuf_hdrptr());
    SimpleMAC_NetStack->lmac->send(SimpleMAC_CbTx, &err);
    if (err != STK_ERR_NONE) {
        *((STK_ERR *)p_arg) = STK_ERR_BUSY;
    } else {
        *((STK_ERR *)p_arg) = STK_ERR_NONE;
    }
}


/**
 *
 */
static void SimpleMAC_Input(void)
{
    frame802154_t frame;
    int hdrlen, ret;
    int len;
    uint8_t *p_data;


    len = packetbuf_datalen();;
    p_data = packetbuf_dataptr();

    hdrlen = frame802154_parse(p_data, len, &frame);
    if (hdrlen == 0) {
        return;
    }

    ret = packetbuf_hdrreduce(len - frame.payload_len);
    if (ret == 0) {
        return;
    }

    if (frame.fcf.dest_addr_mode) {
        if ((frame.dest_pid != mac_phy_config.pan_id) &&
            (frame.dest_pid != FRAME802154_BROADCASTPANDID)) {
            return;
        }

        ret = SimpleMAC_IsBroadcastAddr(frame.fcf.dest_addr_mode, frame.dest_addr);
        if (ret == 0) {
            packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER,
                               (linkaddr_t *)&frame.dest_addr);

#if !NETSTACK_CONF_BRIDGE_MODE
            ret = linkaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                               &linkaddr_node_addr);
            if (ret == 0) {
                return;     /* Not for this node */
            }
#endif
        }
    }

    packetbuf_set_addr(PACKETBUF_ADDR_SENDER,
                       (linkaddr_t *)&frame.src_addr);
    SimpleMAC_NetStack->llsec->input();
}

/**
 *
 * @return
 */
static int8_t SimpleMAC_On(void)
{
    return SimpleMAC_NetStack->lmac->on();
}


/**
 *
 * @param keep_radio_on
 * @return
 */
static int8_t SimpleMAC_Off(int keep_radio_on)
{
    return SimpleMAC_NetStack->lmac->off(keep_radio_on);
}

/**
 *
 * @param p_netStack
 */
static void SimpleMAC_Init(s_ns_t *p_netstack)
{
    SimpleMAC_DSN = random_rand() % 256;
    SimpleMAC_TxBusy = 0;
    SimpleMAC_CbFnct = 0;
    SimpleMAC_CbArg = NULL;


    if (p_netstack != NULL) {
        SimpleMAC_NetStack = p_netstack;
    }
}

/**
 *
 */
static unsigned short SimpleMAC_ChanChkInterval(void)
{
    return 0;
}

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
s_nsHighMac_t simplemac_driver =
{
    "Simple High MAC",
    SimpleMAC_Init,
    SimpleMAC_Send,
    SimpleMAC_Input,
    SimpleMAC_On,
    SimpleMAC_Off,
    SimpleMAC_ChanChkInterval
};
