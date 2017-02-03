/*
 * Copyright (c) 2013, Hasso-Plattner-Institut.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Insecure link layer security driver.
 * \author
 *         Konrad Krentz <konrad.krentz@gmail.com>
 */

/**
 * \addtogroup dllsec_null
 * @{
 */

#include "emb6.h"
#include "dllsec_null.h"
#include "framer_802154.h"
#include "packetbuf.h"
#include "logger.h"

static s_ns_t *pdllsec_netstk;
static mac_callback_t dllsec_txCbFnct;

/**
 * @brief   Transmission callback function handler
 *
 * @param   p_arg
 * @param   p_err
 */
static void dllsec_cbTx(void *p_arg, e_nsErr_t *p_err)
{
  int status;
  int retx = 0;

  switch (*p_err) {
    case NETSTK_ERR_NONE:
      status = MAC_TX_OK;
      retx = 1;
      break;

    case NETSTK_ERR_CHANNEL_ACESS_FAILURE:
      status = MAC_TX_COLLISION;
      retx = 0;
      break;

    case NETSTK_ERR_TX_NOACK:
      status = MAC_TX_NOACK;
      retx = 1;
      break;

    case NETSTK_ERR_BUSY:
      status = MAC_TX_DEFERRED;
      retx = 0;
      break;

    default:
      status = MAC_TX_ERR_FATAL;
      retx = 0;
      break;
  }

  if (dllsec_txCbFnct != NULL) {
    dllsec_txCbFnct(p_arg, status, retx);
    dllsec_txCbFnct = NULL;
  }
}

/*---------------------------------------------------------------------------*/
static void dllsec_send(mac_callback_t sent, void *p_arg)
{
  e_nsErr_t err = NETSTK_ERR_NONE;

  dllsec_txCbFnct = sent;
  packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_DATAFRAME);

  /*
   * Issue next lower layer to transmit the prepared packet
   */
  pdllsec_netstk->dllc->send( packetbuf_hdrptr(), packetbuf_totlen(), &err );
  if (err != NETSTK_ERR_NONE) {
    TRACE_LOG_ERR("<DLLS> e=-%d", err);
  }

  /* inform upper layer of the TX status */
  dllsec_cbTx(p_arg, &err);
}

/*---------------------------------------------------------------------------*/
static int dllsec_onFrameCreated(void)
{
  return 1;
}

/*---------------------------------------------------------------------------*/
static void dllsec_input(void)
{
  pdllsec_netstk->hc->input();
}

/*---------------------------------------------------------------------------*/
static uint8_t dllsec_getOverhead(void)
{
  return 0;
}

/*---------------------------------------------------------------------------*/
static void dllsec_init(s_ns_t *p_netstk)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_netstk == NULL) {
    return;
  }
#endif

  e_nsErr_t err = NETSTK_ERR_NONE;

  pdllsec_netstk = p_netstk;
  pdllsec_netstk->dllc->ioctrl(NETSTK_CMD_RX_CBFNT_SET, (void *) dllsec_input, &err);
}

/*---------------------------------------------------------------------------*/
const s_nsDllsec_t dllsec_driver_null =
{
 "LLSEC NULL",
  dllsec_init,
  dllsec_send,
  dllsec_onFrameCreated,
  dllsec_input,
  dllsec_getOverhead
};
/*---------------------------------------------------------------------------*/

/** @} */
