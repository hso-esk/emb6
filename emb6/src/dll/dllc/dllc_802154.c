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
#include "board_conf.h"
#include "framer_802154.h"
#include "packetbuf.h"
#include "random.h"

#define     LOGGER_ENABLE        LOGGER_LLC
#include    "logger.h"

#if !defined(NETSTK_SUPPORT_HW_CRC)
#include "crc.h"
#endif /* #if !defined(NETSTK_SUPPORT_HW_CRC) */

/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
static void dllc_init(void *p_netstk, e_nsErr_t *p_err);
static void dllc_on(e_nsErr_t *p_err);
static void dllc_off(e_nsErr_t *p_err);
static void dllc_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void dllc_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void dllc_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);
static void dllc_cbtx(void *p_arg, e_nsErr_t *p_err);
static void dllc_verifyAddr(frame802154_t *p_frame, e_nsErr_t *p_err);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
/**  \brief The sequence number (0x00 - 0xff) added to the transmitted
 *   data or MAC command frame. The default is a random value within
 *   the range.
 */
static uint8_t          dllc_dsn;
static void            *pdllc_cbtxarg;
static nsTxCbFnct_t     dllc_cbTxFnct;
static nsRxCbFnct_t     dllc_cbRxFnct;
static s_ns_t          *pdllc_netstk;

#if (NETSTK_CFG_AUTO_ONOFF_EN == TRUE)
static uint8_t       dllc_isOn;
#endif

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsDLLC_t dllc_driver_802154 =
{
 "DLLC 802154",
  dllc_init,
  dllc_on,
  dllc_off,
  dllc_send,
  dllc_recv,
  dllc_ioctl,
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
static void dllc_init(void *p_netstk, e_nsErr_t *p_err)
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

  pdllc_netstk = (s_ns_t *) p_netstk;
  dllc_cbRxFnct = 0;
  dllc_cbTxFnct = 0;
  pdllc_cbtxarg = NULL;
  dllc_dsn = random_rand() & 0xFF;
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_PAN_ID, mac_phy_config.pan_id);
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_FCS_LEN, mac_phy_config.fcs_len);

#if (NETSTK_CFG_AUTO_ONOFF_EN == TRUE)
  /* initial transition to OFF state */
  dllc_off(p_err);
#endif

  *p_err = NETSTK_ERR_NONE;
}


/**
 * @brief   Turn driver on
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void dllc_on(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  pdllc_netstk->mac->on(p_err);
#if (NETSTK_CFG_AUTO_ONOFF_EN == TRUE)
  dllc_isOn = TRUE;
#endif
}


/**
 * @brief   Turn driver off
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void dllc_off(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  pdllc_netstk->mac->off(p_err);
#if (NETSTK_CFG_AUTO_ONOFF_EN == TRUE)
  dllc_isOn = FALSE;
#endif
}


/**
 * @brief   Frame transmission handler
 *
 * @param   p_data      Pointer to buffer holding frame to send
 * @param   len         Length of frame to send
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void dllc_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
  int alloc;
  int is_broadcast;
  uint8_t hdr_len;
  frame802154_t params;

#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }

  if ((len == 0) || (p_data == NULL)) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

#if (NETSTK_CFG_AUTO_ONOFF_EN == TRUE)
  if (dllc_isOn == FALSE) {
    pdllc_netstk->mac->on(p_err);
  }
#endif

  /* init to zeros */
  memset(&params, 0, sizeof(params));

  /* build the FCF. */
  params.fcf.frame_type = packetbuf_attr(PACKETBUF_ATTR_FRAME_TYPE);
  params.fcf.frame_pending = packetbuf_attr(PACKETBUF_ATTR_PENDING);

  /* ACK-required bit */
  is_broadcast = packetbuf_holds_broadcast();
  if (is_broadcast == 1) {
    params.fcf.ack_required = 0;
  } else {
    params.fcf.ack_required = packetbuf_attr(PACKETBUF_ATTR_RELIABLE);
  }

  /* set MAC ACK required attribute accordingly */
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, params.fcf.ack_required);

  /* PAN ID compression */
  params.fcf.panid_compression = 0;

  /* Insert IEEE 802.15.4 (2006) version bits. */
  params.fcf.frame_version = FRAME802154_IEEE802154_2006;

  /* sequence number */
  params.seq = (uint8_t) packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);
  if (params.seq == 0) {
    params.seq = dllc_dsn++;
    packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, params.seq);
  }

  /* addressing fields */
  params.dest_pid = packetbuf_attr(PACKETBUF_ATTR_MAC_PAN_ID);
  if (is_broadcast == 1) {
    /* Broadcast requires short address mode. */
    params.fcf.dest_addr_mode = FRAME802154_SHORTADDRMODE;
    params.dest_addr[0] = 0xFF;
    params.dest_addr[1] = 0xFF;
  } else {
    linkaddr_copy((linkaddr_t *)&params.dest_addr, packetbuf_addr(PACKETBUF_ADDR_RECEIVER));
    params.fcf.dest_addr_mode = FRAME802154_LONGADDRMODE;
  }

  params.src_pid = packetbuf_attr(PACKETBUF_ATTR_MAC_PAN_ID);
  if (LINKADDR_SIZE == 2UL) {
    params.fcf.src_addr_mode = FRAME802154_SHORTADDRMODE;
  } else {
    params.fcf.src_addr_mode = FRAME802154_LONGADDRMODE;
  }
  linkaddr_copy((linkaddr_t *)&params.src_addr, &linkaddr_node_addr);

  /* auxiliary security */
#if LLSEC802154_SECURITY_LEVEL
  if(packetbuf_attr(PACKETBUF_ATTR_SECURITY_LEVEL)) {
    params.fcf.security_enabled = 1;
  }
  /* Setting security-related attributes */
  params.aux_hdr.security_control.security_level = packetbuf_attr(PACKETBUF_ATTR_SECURITY_LEVEL);
  params.aux_hdr.frame_counter.u16[0] = packetbuf_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_0_1);
  params.aux_hdr.frame_counter.u16[1] = packetbuf_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_2_3);
#if LLSEC802154_USES_EXPLICIT_KEYS
  params.aux_hdr.security_control.key_id_mode = packetbuf_attr(PACKETBUF_ATTR_KEY_ID_MODE);
  params.aux_hdr.key_index = packetbuf_attr(PACKETBUF_ATTR_KEY_INDEX);
  params.aux_hdr.key_source.u16[0] = packetbuf_attr(PACKETBUF_ATTR_KEY_SOURCE_BYTES_0_1);
#endif /* LLSEC802154_USES_EXPLICIT_KEYS */
#endif /* LLSEC802154_SECURITY_LEVEL */

  /* configure packet payload */
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

#if !defined(NETSTK_SUPPORT_HW_CRC)
  uint16_t checksum_data_len;
  uint8_t *p_mhr;
  uint8_t *p_mfr;
  uint32_t fcs;
  packetbuf_attr_t fcs_len;

  /* compute checksum data: MHR + MAC_Payload */
  p_mhr = (uint8_t *)packetbuf_hdrptr();
  checksum_data_len = packetbuf_totlen();

  /* allocate buffer for MAC footer (checksum) */
  fcs_len = packetbuf_attr(PACKETBUF_ATTR_MAC_FCS_LEN);
  alloc = packetbuf_ftralloc(fcs_len);
  if (alloc == 0) {
    *p_err = NETSTK_ERR_BUF_OVERFLOW;
    return;
  }

  /* write footer */
  p_mfr = (uint8_t *)packetbuf_ftrptr();
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  if (fcs_len == 4) {
    /* 32-bit CRC */
    fcs = crc_32_calc(p_mhr, checksum_data_len);
    p_mfr[0] = (fcs & 0xFF000000u) >> 24;
    p_mfr[1] = (fcs & 0x00FF0000u) >> 16;
    p_mfr[2] = (fcs & 0x0000FF00u) >> 8;
    p_mfr[3] = (fcs & 0x000000FFu);
  } else {
#endif
    /* 16-bit CRC */
    fcs = crc_16_calc(p_mhr, checksum_data_len);
    p_mfr[0] = (fcs & 0xFF00u) >> 8;
    p_mfr[1] = (fcs & 0x00FFu);
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  }
#endif
#endif /* #if !defined(NETSTK_SUPPORT_HW_CRC) */

#if LOGGER_ENABLE
  /*
   * Logging
   */
  uint16_t data_len = packetbuf_totlen();
  uint8_t *p_dataptr = packetbuf_hdrptr();
  LOG_RAW("DLLC_TX: ");
  while (data_len--) {
    LOG_RAW("%02x", *p_dataptr++);
  }
  LOG_RAW("\r\n");
#endif

  /*
   * set TX callback function and argument
   */
  pdllc_netstk->mac->ioctrl(NETSTK_CMD_TX_CBFNCT_SET, (void *) dllc_cbtx, p_err);
  pdllc_netstk->mac->ioctrl(NETSTK_CMD_TX_CBARG_SET, NULL, p_err);

  /* Issue next lower layer to transmit the prepared frame */
  pdllc_netstk->mac->send(packetbuf_hdrptr(), packetbuf_totlen(), p_err);

#if (NETSTK_CFG_AUTO_ONOFF_EN == TRUE)
  if (dllc_isOn == FALSE) {
    pdllc_netstk->mac->off(p_err);
  }
#endif
}


/**
 * @brief   Frame reception handler
 *
 * @param   p_data      Pointer to buffer holding frame to receive
 * @param   len         Length of frame to receive
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void dllc_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
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
  int8_t rssi;

  /* store the received packet into internal packet buffer */
  packetbuf_clear();
  packetbuf_set_datalen(len);
  memcpy(packetbuf_dataptr(), p_data, len);

  /* parse the received packet */
  hdrlen = frame802154_parse(p_data, len, &frame);
  if (hdrlen == 0) {
    *p_err = NETSTK_ERR_INVALID_FRAME;
    return;
  }

  /* strip MAC header off */
  ret = packetbuf_hdrreduce(len - frame.payload_len);
  if (ret == 0) {
    *p_err = NETSTK_ERR_FATAL;
    return;
  }

  /* verify frame addresses */
  dllc_verifyAddr(&frame, p_err);
  if (*p_err != NETSTK_ERR_NONE) {
    return;
  }

  /* set packet buffer miscellaneous attributes */
  pdllc_netstk->mac->ioctrl(NETSTK_CMD_RF_RSSI_GET, &rssi, p_err);
  packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rssi);

  /* signal next higher layer of the valid received frame */
  if (dllc_cbRxFnct) {
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

    /* Inform the next higher layer */
    dllc_cbRxFnct(packetbuf_dataptr(), packetbuf_datalen(), p_err);
  }
}


/**
 * @brief    Miscellaneous commands handler
 *
 * @param   cmd         Command to be issued
 * @param   p_val       Pointer to a variable related to the command
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void dllc_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
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
        dllc_cbTxFnct = (nsTxCbFnct_t) p_val;
      }
      break;

    case NETSTK_CMD_TX_CBARG_SET:
      pdllc_cbtxarg = p_val;
      break;

    case NETSTK_CMD_RX_CBFNT_SET:
      if (p_val == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
      } else {
        dllc_cbRxFnct = (nsRxCbFnct_t) p_val;
      }
      break;

    case NETSTK_CMD_DLLC_RSVD:
      break;

    default:
      pdllc_netstk->mac->ioctrl(cmd, p_val, p_err);
      break;
  }
}


/**
 *
 * @param ptr
 * @param status
 * @param transmissions
 */
static void dllc_cbtx(void *p_arg, e_nsErr_t *p_err)
{
  if (dllc_cbTxFnct) {
    dllc_cbTxFnct(pdllc_cbtxarg, p_err);
  }
}


/**
 * @brief   Verify destination addresses of the received frame
 */
static void dllc_verifyAddr(frame802154_t *p_frame, e_nsErr_t *p_err)
{
  int is_addr_matched;
  uint8_t is_broadcast;
  packetbuf_attr_t dev_pan_id;


  /*
   * Verify destination address
   */
  dev_pan_id = packetbuf_attr(PACKETBUF_ATTR_MAC_PAN_ID);
  if (p_frame->fcf.dest_addr_mode) {
    if ((p_frame->dest_pid != dev_pan_id) &&
        (p_frame->dest_pid != FRAME802154_BROADCASTPANDID)) {
      *p_err = NETSTK_ERR_FATAL;
      return;
    }

    /*
     * Check for broadcast frame
     */
    is_broadcast = frame802154_broadcast(p_frame);
    if (is_broadcast == 0) {
      /*
       * If not a broadcast frame, then store destination address
       */
      packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, (linkaddr_t *) &p_frame->dest_addr);

#if !NETSTACK_CONF_BRIDGE_MODE
      is_addr_matched = linkaddr_cmp((linkaddr_t *)&p_frame->dest_addr, &linkaddr_node_addr);
      if (is_addr_matched == 0) {
        /* the received packet is not destined not for this node, then discarded */
        *p_err = NETSTK_ERR_FATAL;
        return;
      }
#endif
    }
  }

  /*
   * If destination address is valid, then store source address
   */
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, (linkaddr_t *) &p_frame->src_addr);
}


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
