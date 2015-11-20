/**
 * @file    llc_null.c
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
static void LLC_Init(void *p_netstk, e_nsErr_t *p_err);
static void LLC_On(e_nsErr_t *p_err);
static void LLC_Off(e_nsErr_t *p_err);
static void LLC_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void LLC_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void LLC_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static s_ns_t           *LLC_Netstk;
static nsRxCbFnct_t      LLC_CbRxFnct;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsLLC_t LLCDrvNull =
{
   "LLC NULL",
    LLC_Init,
    LLC_On,
    LLC_Off,
    LLC_Send,
    LLC_Recv,
    LLC_IOCtrl
};


/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
static void LLC_Init(void *p_netstk, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_netstk == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }
#endif


    LLC_Netstk = p_netstk;
    *p_err = NETSTK_ERR_NONE;
}


static void LLC_On(e_nsErr_t *p_err)
{
    LLC_Netstk->mac->on(p_err);
}


static void LLC_Off(e_nsErr_t *p_err)
{
    LLC_Netstk->mac->off(p_err);
}


static void LLC_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
    LLC_Netstk->mac->send(p_data, len, p_err);
}


static void LLC_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
    if (LLC_CbRxFnct) {
        /*
         * Inform the next higher layer
         */
        LLC_CbRxFnct(p_data,
                     len,
                     p_err);
    }
}


static void LLC_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif

    *p_err = NETSTK_ERR_NONE;
    switch (cmd) {
        case NETSTK_CMD_LLC_RSVD:
            break;

        default:
            LLC_Netstk->mac->ioctrl(cmd, p_val, p_err);
            break;
    }
}



/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
