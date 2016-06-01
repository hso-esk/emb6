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
 * @file    mac_null.c
 * @date    16.10.2015
 * @author  PN
 */


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"
#include "packetbuf.h"
#include "linkaddr.h"

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


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static s_ns_t       *pmac_netstk;
static void         *pmac_cbTxArg;
static nsTxCbFnct_t  mac_cbTxFnct;
static e_nsErr_t     mac_txErr;

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsMAC_t mac_driver_null =
{
   "MAC NULL",
    mac_init,
    mac_on,
    mac_off,
    mac_send,
    mac_recv,
    mac_ioctl
};

extern uip_lladdr_t uip_lladdr;

/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
static void mac_init(void *p_netstk, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_netstk == NULL) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

  /* store pointer to netstack structure */
  pmac_netstk = p_netstk;

  /* configure stack MAC address */
  memcpy(&uip_lladdr.addr, &mac_phy_config.mac_address, 8);
  linkaddr_set_node_addr((linkaddr_t *) mac_phy_config.mac_address);

  /* set returned error code */
  *p_err = NETSTK_ERR_NONE;
}


static void mac_on(e_nsErr_t *p_err)
{
  pmac_netstk->phy->on(p_err);
}


static void mac_off(e_nsErr_t *p_err)
{
  pmac_netstk->phy->off(p_err);
}


static void mac_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
  pmac_netstk->phy->send(p_data, len, p_err);

  mac_txErr = NETSTK_ERR_NONE;
  if (mac_cbTxFnct != NULL) {
    mac_cbTxFnct(pmac_cbTxArg, &mac_txErr);
  }
}


static void mac_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
  packetbuf_clear();
  packetbuf_set_datalen(len);
  memcpy(packetbuf_dataptr(), p_data, len);

  pmac_netstk->dllc->recv(p_data, len, p_err);
}


static void mac_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
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
    case NETSTK_CMD_MAC_RSVD:
      break;

    default:
      pmac_netstk->phy->ioctrl(cmd, p_val, p_err);
      break;
  }
}



/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
