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


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/

static void             *DLLC_CbTxArg;
static nsTxCbFnct_t     DLLC_CbTxFnct;
static nsRxCbFnct_t     DLLC_CbRxFnct;
static s_ns_t           *DLLC_Netstk;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsDLLC_t DLLCDrvNull =
{
   "DLLC NULL",
    DLLC_Init,
    DLLC_On,
    DLLC_Off,
    DLLC_Send,
    DLLC_Recv,
    DLLC_IOCtrl
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
static void DLLC_CbTx(void *p_arg, e_nsErr_t *p_err)
{
    if (DLLC_CbTxFnct) {
        DLLC_CbTxFnct(DLLC_CbTxArg, p_err);
    }
}

static void DLLC_Init(void *p_netstk, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_netstk == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }
#endif


    DLLC_Netstk = p_netstk;
    DLLC_CbTxFnct = 0;
    DLLC_CbTxArg = NULL;
    *p_err = NETSTK_ERR_NONE;
}


static void DLLC_On(e_nsErr_t *p_err)
{
    DLLC_Netstk->mac->on(p_err);
}


static void DLLC_Off(e_nsErr_t *p_err)
{
    DLLC_Netstk->mac->off(p_err);
}


static void DLLC_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
    /*
     * set TX callback function and argument
     */
    DLLC_Netstk->mac->ioctrl(NETSTK_CMD_TX_CBFNCT_SET,
                            (void *)DLLC_CbTx,
                            p_err);

    DLLC_Netstk->mac->ioctrl(NETSTK_CMD_TX_CBARG_SET,
                            NULL,
                            p_err);

    DLLC_Netstk->mac->send(p_data, len, p_err);
}


static void DLLC_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
    if (DLLC_CbRxFnct) {
        /*
         * Inform the next higher layer
         */
        DLLC_CbRxFnct(p_data,
                     len,
                     p_err);
    }
}


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



/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
