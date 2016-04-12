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
#include "rt_tmr.h"


#define     LOGGER_ENABLE        LOGGER_MAC
#include    "logger.h"


/*
********************************************************************************
*                               LOCAL MACROS
********************************************************************************
*/
#if (NETSTK_CFG_WOR_EN == TRUE)
#define MAC_CFG_TMR_WFA_IN_MS               (uint32_t )( 9 )
#else
#define MAC_CFG_TMR_WFA_IN_MS               (uint32_t )( 5 )
#endif

/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
static void mac_init(void *p_netstk, e_nsErr_t *p_err);
static void mac_on(e_nsErr_t *p_err);
static void mac_off(e_nsErr_t *p_err);
static void mac_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void mac_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void mac_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);
static void mac_txAck(uint8_t seq, e_nsErr_t *p_err);
static void mac_csma(e_nsErr_t *p_err);

/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
/**  \brief The sequence number (0x00 - 0xff) added to the transmitted
 *   data or MAC command frame. The default is a random value within
 *   the range.
 */
static uint8_t          mac_isAckReq;
static s_rt_tmr_t       mac_tmrWfa;
static s_ns_t          *pmac_netstk;
static void            *pmac_cbTxArg;
static nsTxCbFnct_t     mac_cbTxFnct;
static e_nsErr_t        mac_txErr;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsMAC_t mac_driver_802154 =
{
 "MAC 802154",
  mac_init,
  mac_on,
  mac_off,
  mac_send,
  mac_recv,
  mac_ioctl,
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
void mac_init(void *p_netstk, e_nsErr_t *p_err)
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

  /* initialize local variables */
  pmac_netstk = (s_ns_t *) p_netstk;
  mac_cbTxFnct = 0;
  pmac_cbTxArg = NULL;
  mac_isAckReq = 0;
  mac_txErr = NETSTK_ERR_NONE;

  rt_tmr_create(&mac_tmrWfa, E_RT_TMR_TYPE_ONE_SHOT, MAC_CFG_TMR_WFA_IN_MS, 0, NULL);

  /*
   * Configure stack address
   */
  memcpy(&uip_lladdr.addr, &mac_phy_config.mac_address, 8);
  linkaddr_set_node_addr((linkaddr_t *) mac_phy_config.mac_address);

  /* set returned error */
  *p_err = NETSTK_ERR_NONE;
}


/**
 * @brief   Turn driver on
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
void mac_on(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  pmac_netstk->phy->on(p_err);
}


/**
 * @brief   Turn driver off
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
void mac_off(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  pmac_netstk->phy->off(p_err);
}


/**
 * @brief   Frame transmission handler
 *
 * @param   p_data      Pointer to buffer holding frame to send
 * @param   len         Length of frame to send
 * @param   p_err       Pointer to a variable storing returned error code
 */
void mac_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }

  if ((len == 0) || (p_data == NULL)) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

  LOG_INFO("MAC_TX: Transmit %d bytes.", len);

  int is_broadcast;
  uint8_t is_tx_done;
  uint8_t is_wfa_done;
  uint8_t tx_retries;
  uint8_t tx_retriesMax;
  packetbuf_attr_t is_ack_req;
  e_rt_tmr_state_t wfa_tmr_state;

  /* find out if ACK is required */
  is_broadcast = packetbuf_holds_broadcast();
  is_ack_req = packetbuf_attr(PACKETBUF_ATTR_RELIABLE);
  mac_isAckReq = (is_ack_req == 1) &&
                 (is_broadcast == 0);

  /*
   * Transmission handling
   */
  tx_retriesMax = packetbuf_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS);
  tx_retries = 0;
  is_tx_done = FALSE;
  is_wfa_done = FALSE;
  do {
    /* transmission attempt begins with CSMA operation */
    mac_csma(&mac_txErr);

    tx_retries++;
    LOG_INFO("MAC_TX: Attempt %d.", tx_retries);

    if (mac_txErr != NETSTK_ERR_CHANNEL_ACESS_FAILURE) {
      /* when channel is clear, attempt to transmit packet */
      pmac_netstk->phy->send(p_data, len, &mac_txErr);

      if (mac_isAckReq == FALSE) {
        is_tx_done = TRUE;
      } else {
        /* ACK is required then start Wait-For-ACK timer */
        rt_tmr_stop(&mac_tmrWfa);
        rt_tmr_start(&mac_tmrWfa);

        /* wait for ACK */
        mac_txErr = NETSTK_ERR_TX_NOACK;
        do {
          /* poll for state of WFA timer */
          wfa_tmr_state = rt_tmr_getState(&mac_tmrWfa);
          pmac_netstk->phy->ioctrl(NETSTK_CMD_RX_BUF_READ, NULL, p_err);
          /*
           * Wait-For-ACK is declared as done if one of following
           * conditions is met:
           * (1)  WFA timeout is over
           * (2)  MAC_TxErr is set to NETSTK_ERR_NONE, which is expected
           *      to happen in MAC_Recv()
           */
          is_wfa_done = (wfa_tmr_state != E_RT_TMR_STATE_RUNNING) ||
                        (mac_txErr == NETSTK_ERR_NONE);
        } while (is_wfa_done == FALSE);

        /* stop Wait-For-ACK timer */
        rt_tmr_stop(&mac_tmrWfa);
      }
    } else {
      LOG_INFO("MAC_TX: CH-ERR.");
    }

    /*
     * Transmission attempt is declared as finished when one of following
     * conditions is met.
     * (1)  Transmission is succeeded
     * (2)  CSMA declares channel busy while attempting to transmit
     * (3)  Transmission retry exceeds maximum
     */
    is_tx_done = ((mac_txErr  == NETSTK_ERR_NONE) ||
                  (tx_retries >= tx_retriesMax));
  } while (is_tx_done == FALSE);
  LOG_INFO("MAC_TX: --> Done - TX Status %d (%d/%d retries).", mac_txErr, tx_retries, tx_retriesMax);

  /* signal upper layer */
  mac_isAckReq = 0;
  mac_cbTxFnct(pmac_cbTxArg, &mac_txErr);
  mac_txErr = NETSTK_ERR_NONE;
  *p_err = mac_txErr;
}


/**
 * @brief   Frame reception handler
 *
 * @param   p_data      Pointer to buffer holding frame to receive
 * @param   len         Length of frame to receive
 * @param   p_err       Pointer to a variable storing returned error code
 */
void mac_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }

  if ((len == 0) || (p_data == NULL)) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

  int hdrlen;
  uint8_t is_acked;
  uint8_t exp_pkt_seq;
  frame802154_t frame;

  /* set returned error code to default */
  *p_err = NETSTK_ERR_NONE;

  /* frames with length greater than supported by the packetbuf shall be
   * discarded. Otherwise it leads to buffer overflow when using memcpy to
   * store frame into the packet buffer */
  if (len > PACKETBUF_SIZE) {
    *p_err = NETSTK_ERR_INVALID_FRAME;
    return;
  }

  /* Parsing but not reducing header as that will be then handled by DLLC */
  hdrlen = frame802154_parse(p_data, len, &frame);
  if (hdrlen == 0) {
    *p_err = NETSTK_ERR_INVALID_FRAME;
    return;
  }

  if (mac_isAckReq) {
    /*
     * When ACK is required, frames other than ACK shall be discarded
     */
    /* check if this is expected ACK */
    exp_pkt_seq = (uint8_t) packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);
    is_acked = ((frame.seq == exp_pkt_seq) &&
                (frame.fcf.frame_type == FRAME802154_ACKFRAME));
    if (is_acked) {
      mac_txErr = NETSTK_ERR_NONE;
    }
  } else {
    switch (frame.fcf.frame_type) {
      case FRAME802154_DATAFRAME:
      case FRAME802154_CMDFRAME:
        if ((frame.fcf.ack_required == 1) &&
            (frame802154_broadcast(&frame) == 0) &&
            (linkaddr_cmp((linkaddr_t *) frame.dest_addr, &linkaddr_node_addr)) == 1) {
          mac_txAck(frame.seq, p_err);
        }

        /* signal upper layer of the received packet */
        pmac_netstk->dllc->recv(p_data, len, p_err);
        break;

      default:
        *p_err = NETSTK_ERR_INVALID_FRAME;
        break;
    }
  }
  LOG_INFO("MAC_RX: Received %d bytes.", len);
}


/**
 * @brief    Miscellaneous commands handler
 *
 * @param   cmd         Command to be issued
 * @param   p_val       Pointer to a variable related to the command
 * @param   p_err       Pointer to a variable storing returned error code
 */
void mac_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
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
        mac_cbTxFnct = (nsTxCbFnct_t) p_val;
      }
      break;

    case NETSTK_CMD_TX_CBARG_SET:
      pmac_cbTxArg = p_val;
      break;

    default:
      pmac_netstk->phy->ioctrl(cmd, p_val, p_err);
      break;
  }
}


/**
 * @brief   ACK transmission
 *
 * @param   seq     Frame sequence number of the outgoing ACK
 */
static void mac_txAck(uint8_t seq, e_nsErr_t *p_err)
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
  pmac_netstk->phy->send(packetbuf_hdrptr(), packetbuf_totlen(), p_err);
  LOG_INFO("MAC_TX: ACK %d.", frame.seq);
}


/**
 * @brief   This function performs CSMA-CA mechanism.
 *
 * @param   p_err   Pointer to a variable storing returned error code
 */
static void mac_csma(e_nsErr_t *p_err)
{
  uint32_t unit_backoff;
  uint32_t delay = 0;
  uint32_t max_random;
  uint8_t nb;
  uint8_t be;
  uint8_t min_be = 3;
  uint8_t max_be = 5;
  uint8_t max_backoff = 4;

  nb = 0;
  be = min_be;
  unit_backoff = 20 * 20; /* unit backoff period = 20 * symbol periods [us] */
  *p_err = NETSTK_ERR_NONE;

  /* perform CCA maximum MaxBackoff time */
  while (nb <= max_backoff) {
    /* delay for random (2^BE - 1) unit backoff periods */
    max_random = (1 << be) - 1;
    delay = bsp_getrand(max_random);
    delay *= unit_backoff; // us, symbol period is 20us @50kbps
    bsp_delay_us(delay);

    /*
     * Perform CCA
     */
    pmac_netstk->phy->ioctrl(NETSTK_CMD_RF_CCA_GET, 0, p_err);
    if (*p_err == NETSTK_ERR_NONE) {
      /* channel idle is detected, then terminate CSMA */
      break;
    } else {
      /* channel busy is detected */
      nb++;
      be = ((be + 1) < max_be) ? (be + 1) : (max_be);
    }
  }
  LOG_INFO("MAC_TX: NB %d.", nb);
}

/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
