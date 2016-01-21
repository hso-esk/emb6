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
 * @file    dllc_802154.c
 * @date    19.10.2015
 * @author  PN
 */


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"

#include "framer_802154.h"
#include "packetbuf.h"
#include "random.h"

#include "lib_port.h"

#define     LOGGER_ENABLE        LOGGER_LLC
#include    "logger.h"

/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
static void DLLC_Init(void *p_netstk, e_nsErr_t *p_err);
static void DLLC_On(e_nsErr_t *p_err);
static void DLLC_Off(e_nsErr_t *p_err);
static void DLLC_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void DLLC_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void DLLC_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);


static void DLLC_CbTx(void *p_arg, e_nsErr_t *p_err);
static void DLLC_VerifyAddr(frame802154_t *p_frame, e_nsErr_t *p_err);
static uint8_t DLLC_IsBroadcastAddr(uint8_t mode, uint8_t *p_addr);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
/**  \brief The sequence number (0x00 - 0xff) added to the transmitted
 *   data or MAC command frame. The default is a random value within
 *   the range.
 */
static uint8_t          DLLC_DSN;
static void            *DLLC_CbTxArg;
static nsTxCbFnct_t     DLLC_CbTxFnct;
static nsRxCbFnct_t     DLLC_CbRxFnct;
static s_ns_t          *DLLC_Netstk;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsDLLC_t DLLCDrv802154 =
{
   "DLLC 802154",
    DLLC_Init,
    DLLC_On,
    DLLC_Off,
    DLLC_Send,
    DLLC_Recv,
    DLLC_IOCtrl,
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
static void DLLC_Init(void *p_netstk, e_nsErr_t *p_err)
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


    DLLC_Netstk = (s_ns_t *)p_netstk;
    DLLC_CbRxFnct = 0;
    DLLC_CbTxFnct = 0;
    DLLC_CbTxArg = NULL;
    DLLC_DSN = random_rand() % 256;
    *p_err = NETSTK_ERR_NONE;
}


/**
 * @brief   Turn driver on
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void DLLC_On(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif


    DLLC_Netstk->mac->on(p_err);
}


/**
 * @brief   Turn driver off
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void DLLC_Off(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif


    DLLC_Netstk->mac->off(p_err);
}


/**
 * @brief   Frame transmission handler
 *
 * @param   p_data      Pointer to buffer holding frame to send
 * @param   len         Length of frame to send
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void DLLC_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
    int             alloc;
    uint8_t         hdr_len;
    frame802154_t   params;


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
    params.seq = DLLC_DSN;

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

#if NETSTK_CFG_IEEE_802154_IGNACK
    params.fcf.ack_required = 0;
#endif /* #if NETSTK_CFG_IEEE_802154_IGNACK */

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

    frame802154_setDSN(DLLC_DSN);
    params.payload = packetbuf_dataptr();
    params.payload_len = packetbuf_datalen();

    /* allocate buffer for MAC header */
    hdr_len = frame802154_hdrlen(&params);
    alloc = packetbuf_hdralloc(hdr_len);
    if (alloc == 0) {
        *p_err = NETSTK_ERR_BUF_OVERFLOW;
        return;
    }

    /* write the header */
    frame802154_create(&params, packetbuf_hdrptr());


#if LOGGER_ENABLE
    /*
     * Logging
     */
    uint16_t data_len = packetbuf_totlen();
    uint8_t *p_dataptr = packetbuf_hdrptr();
    LOG_RAW("\r\n====================\r\n");
    LOG_RAW("DLLC_TX: ");
    while (data_len--) {
        LOG_RAW("%02x", *p_dataptr++);
    }
    LOG_RAW("\r\n");
#endif


    /*
     * set TX callback function and argument
     */
    DLLC_Netstk->mac->ioctrl(NETSTK_CMD_TX_CBFNCT_SET,
                            (void *)DLLC_CbTx,
                            p_err);

    DLLC_Netstk->mac->ioctrl(NETSTK_CMD_TX_CBARG_SET,
                            NULL,
                            p_err);

    /*
     * Issue next lower layer to transmit the prepared frame
     */
    DLLC_Netstk->mac->send(packetbuf_hdrptr(),
                           packetbuf_totlen(),
                           p_err);
    if (*p_err == NETSTK_ERR_NONE) {
        DLLC_DSN++;
    }
#if NETSTK_CFG_IEEE_802154_IGNACK
    else if(*p_err == NETSTK_ERR_MAC_ACKOFF )
    {
        DLLC_DSN++;
        *p_err = NETSTK_ERR_NONE;
    }
#endif /* NETSTK_CFG_IEEE_802154_IGNACK */
}


/**
 * @brief   Frame reception handler
 *
 * @param   p_data      Pointer to buffer holding frame to receive
 * @param   len         Length of frame to receive
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void DLLC_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
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
    DLLC_VerifyAddr(&frame, p_err);
    if (*p_err != NETSTK_ERR_NONE) {
        return;
    }

    /*
     * Signal next higher layer of the valid received frame
     */
    if (DLLC_CbRxFnct) {
#if LOGGER_ENABLE
        /*
         * Logging
         */
        uint16_t data_len = packetbuf_datalen();
        uint8_t *p_dataptr = packetbuf_dataptr();
        LOG_RAW("DLLC_RX: ");
        while (data_len--) {
            LOG_RAW("%02x", *p_dataptr++);
        }
        LOG_RAW("\r\n====================\r\n");
#endif

        /*
         * Inform the next higher layer
         */
        DLLC_CbRxFnct(packetbuf_dataptr(),
                     packetbuf_datalen(),
                     p_err);
    }
}


/**
 * @brief    Miscellaneous commands handler
 *
 * @param   cmd         Command to be issued
 * @param   p_val       Pointer to a variable related to the command
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void DLLC_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
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
                DLLC_CbTxFnct = (nsTxCbFnct_t)p_val;
            }
            break;

        case NETSTK_CMD_TX_CBARG_SET:
            DLLC_CbTxArg = p_val;
            break;

        case NETSTK_CMD_RX_CBFNT_SET:
            if (p_val == NULL) {
                *p_err = NETSTK_ERR_INVALID_ARGUMENT;
            } else {
                DLLC_CbRxFnct = (nsRxCbFnct_t)p_val;
            }
            break;

        case NETSTK_CMD_DLLC_RSVD:
            break;

        default:
            DLLC_Netstk->mac->ioctrl(cmd, p_val, p_err);
            break;
    }
}


/**
 *
 * @param ptr
 * @param status
 * @param transmissions
 */
static void DLLC_CbTx(void *p_arg, e_nsErr_t *p_err)
{
    if (DLLC_CbTxFnct) {
        DLLC_CbTxFnct(DLLC_CbTxArg, p_err);
    }
}


/**
 *
 * @param mode
 * @param addr
 * @return
 */
static uint8_t DLLC_IsBroadcastAddr(uint8_t mode, uint8_t *p_addr)
{
    uint8_t i = mode == FRAME802154_SHORTADDRMODE ? 2 : 8;
    while (i-- > 0) {
        if (p_addr[i] != 0xff) {
            return 0;
        }
    }
    return 1;
}

/**
 * @brief   Verify destination addresses of the received frame
 */
static void DLLC_VerifyAddr(frame802154_t *p_frame, e_nsErr_t *p_err)
{
    int     is_addr_matched;
    uint8_t is_broadcast;


    /*
     * Verify destination address
     */
    if (p_frame->fcf.dest_addr_mode) {
        if ((p_frame->dest_pid != mac_phy_config.pan_id) &&
            (p_frame->dest_pid != FRAME802154_BROADCASTPANDID)) {
            *p_err = NETSTK_ERR_FATAL;
            return;
        }

        /*
         * Check for broadcast frame
         */
        is_broadcast = DLLC_IsBroadcastAddr(p_frame->fcf.dest_addr_mode,
                                            p_frame->dest_addr);
        if (is_broadcast == 0) {
            /*
             * If not a broadcast frame, then store destination address
             */
            packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER,
                               (linkaddr_t *)&p_frame->dest_addr);
#if !NETSTACK_CONF_BRIDGE_MODE
            is_addr_matched = linkaddr_cmp((linkaddr_t *)&p_frame->dest_addr,
                                           &linkaddr_node_addr);
            if (is_addr_matched == 0) {
                /*
                 * Not for this node
                 */
                *p_err = NETSTK_ERR_FATAL;
                return;
            }
#endif


        }
    }

    /*
     * If destination address is valid, then store source address
     */
    packetbuf_set_addr(PACKETBUF_ADDR_SENDER,
                       (linkaddr_t *)&p_frame->src_addr);
}


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
