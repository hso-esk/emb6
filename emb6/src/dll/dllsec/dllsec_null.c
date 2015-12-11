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


static s_ns_t          *DLLSec_Netstk;
static mac_callback_t   DLLSec_TxCbFnct;

/**
 * @brief   Transmission callback function handler
 *
 * @param   p_arg
 * @param   p_err
 */
static void DLLSec_CbTx(void *p_arg, e_nsErr_t *p_err)
{
    int status;
    int retx = 0;

    switch (*p_err) {
        case NETSTK_ERR_NONE:
            status = MAC_TX_OK;
            retx = 0;
            break;

        case NETSTK_ERR_CHANNEL_ACESS_FAILURE:
            status = MAC_TX_COLLISION;
            retx = 3;
            break;

        case NETSTK_ERR_TX_NOACK:
            status = MAC_TX_NOACK;
            retx = 3;
            break;

        case NETSTK_ERR_BUSY:
            status = MAC_TX_DEFERRED;
            retx = 3;
            break;

        default:
            status = MAC_TX_ERR_FATAL;
            retx = 3;
            break;
    }

    DLLSec_TxCbFnct(p_arg, status, retx);
}


/*---------------------------------------------------------------------------*/
static void DLLSec_Send(mac_callback_t sent, void *p_arg)
{
    e_nsErr_t err = NETSTK_ERR_NONE;


    DLLSec_TxCbFnct = sent;
    packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_DATAFRAME);


    /*
     * set TX callback function and argument
     */
    DLLSec_Netstk->dllc->ioctrl(NETSTK_CMD_TX_CBFNCT_SET,
                                (void *)DLLSec_CbTx,
                                &err);

    DLLSec_Netstk->dllc->ioctrl(NETSTK_CMD_TX_CBARG_SET,
                                p_arg,
                                &err);

    /*
     * Issue next lower layer to transmit the prepared packet
     */
    DLLSec_Netstk->dllc->send(packetbuf_hdrptr(),
                              packetbuf_totlen(),
                              &err);
}

/*---------------------------------------------------------------------------*/
static int DLLSec_OnFrameCreated(void)
{
    return 1;
}

/*---------------------------------------------------------------------------*/
static void DLLSec_Input(void)
{
    DLLSec_Netstk->hc->input();
}

/*---------------------------------------------------------------------------*/
static uint8_t DLLSec_GetOverhead(void)
{
    return 0;
}

/*---------------------------------------------------------------------------*/
static void DLLSec_Init(s_ns_t *p_netstk)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_netstk == NULL) {
        return;
    }
#endif

    e_nsErr_t err = NETSTK_ERR_NONE;

    DLLSec_Netstk = p_netstk;
    DLLSec_Netstk->dllc->ioctrl(NETSTK_CMD_RX_CBFNT_SET,
                              (void *)DLLSec_Input,
                              &err);
}

/*---------------------------------------------------------------------------*/
const s_nsdllsec_t nullsec_driver =
{
       "LLSEC NULL",
        DLLSec_Init,
        DLLSec_Send,
        DLLSec_OnFrameCreated,
        DLLSec_Input,
        DLLSec_GetOverhead
};
/*---------------------------------------------------------------------------*/

/** @} */
