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
 * @file    dllc_null.c
 * @date    16.10.2015
 * @author  PN
 */


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"
#include "board_conf.h"
#include "packetbuf.h"
#include "logger.h"

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

static void dllc_cbTx(void *p_arg, e_nsErr_t *p_err);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/

static void         *pdllc_cbTxArg;
static s_ns_t       *pdllc_netstk;
static nsTxCbFnct_t  dllc_cbTxFnct;
static nsRxCbFnct_t  dllc_cbRxFnct;

#if (NETSTK_CFG_AUTO_ONOFF_EN == TRUE)
static uint8_t       dllc_isOn;
#endif

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsDLLC_t dllc_driver_null =
{
 "DLLC NULL",
  dllc_init,
  dllc_on,
  dllc_off,
  dllc_send,
  dllc_recv,
  dllc_ioctl
};


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
static void dllc_cbTx(void *p_arg, e_nsErr_t *p_err)
{
  if (dllc_cbTxFnct) {
    dllc_cbTxFnct(pdllc_cbTxArg, p_err);
  }
}


static void dllc_init(void *p_netstk, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_netstk == NULL) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

  pdllc_netstk = p_netstk;
  dllc_cbTxFnct = 0;
  pdllc_cbTxArg = NULL;
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_PAN_ID, mac_phy_config.pan_id);
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_FCS_LEN, mac_phy_config.fcs_len);

#if (NETSTK_CFG_AUTO_ONOFF_EN == TRUE)
  /* initial transition to OFF state */
  dllc_off(p_err);
#endif

  *p_err = NETSTK_ERR_NONE;
}


static void dllc_on(e_nsErr_t *p_err)
{
  pdllc_netstk->mac->on(p_err);
#if (NETSTK_CFG_AUTO_ONOFF_EN == TRUE)
  dllc_isOn = TRUE;
#endif
}


static void dllc_off(e_nsErr_t *p_err)
{
  pdllc_netstk->mac->off(p_err);
#if (NETSTK_CFG_AUTO_ONOFF_EN == TRUE)
  dllc_isOn = FALSE;
#endif
}


static void dllc_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
#if (NETSTK_CFG_AUTO_ONOFF_EN == TRUE)
  if (dllc_isOn == FALSE) {
    pdllc_netstk->mac->on(p_err);
  }
#endif

  uint8_t is_ack_required = 0;
  if (packetbuf_holds_broadcast() == 0) {
    is_ack_required = packetbuf_attr(PACKETBUF_ATTR_RELIABLE);
  }
  /* set MAC ACK required attribute accordingly */
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, is_ack_required);

  pdllc_netstk->mac->ioctrl(NETSTK_CMD_TX_CBFNCT_SET, (void *)dllc_cbTx, p_err);
  pdllc_netstk->mac->ioctrl(NETSTK_CMD_TX_CBARG_SET, NULL, p_err);

#if !defined(NETSTK_SUPPORT_HW_CRC)
  uint8_t *p_mfr;
  uint32_t fcs;
  packetbuf_attr_t fcs_len;

  /* write footer */
  p_mfr = p_data + len;
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  fcs_len = packetbuf_attr(PACKETBUF_ATTR_MAC_FCS_LEN);
  if (fcs_len == 4) {
    /* 32-bit CRC */
    fcs = crc_32_calc(p_data, len);
    p_mfr[0] = (fcs & 0xFF000000u) >> 24;
    p_mfr[1] = (fcs & 0x00FF0000u) >> 16;
    p_mfr[2] = (fcs & 0x0000FF00u) >> 8;
    p_mfr[3] = (fcs & 0x000000FFu);
  } else {
#endif
    /* 16-bit CRC */
    fcs = crc_16_calc(p_data, len);
    p_mfr[0] = (fcs & 0xFF00u) >> 8;
    p_mfr[1] = (fcs & 0x00FFu);
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  }
#endif
  len += fcs_len;
#endif /* #if !defined(NETSTK_SUPPORT_HW_CRC) */

  /* issue transmission request */
  pdllc_netstk->mac->send(p_data, len, p_err);

#if (NETSTK_CFG_AUTO_ONOFF_EN == TRUE)
  if (dllc_isOn == FALSE) {
    pdllc_netstk->mac->off(p_err);
  }
#endif
}


static void dllc_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
  if (dllc_cbRxFnct) {
    /* set return error code */
    *p_err = NETSTK_ERR_NONE;
    int8_t rssi;

    /* store the received frame into common packet buffer */
    packetbuf_clear();
    packetbuf_set_datalen(len);
    memcpy(packetbuf_dataptr(), p_data, len);

    /* set packet buffer miscellaneous attributes */
    pdllc_netstk->mac->ioctrl(NETSTK_CMD_RF_RSSI_GET, &rssi, p_err);
    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rssi);

    /* Inform the next higher layer */
    dllc_cbRxFnct(packetbuf_dataptr(), packetbuf_datalen(), p_err);
  }
}


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
      pdllc_cbTxArg = p_val;
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



/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
