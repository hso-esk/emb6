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


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static s_ns_t           *DLLC_Netstk;
static nsRxCbFnct_t      DLLC_CbRxFnct;


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
static void DLLC_Init(void *p_netstk, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_netstk == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }
#endif


    DLLC_Netstk = p_netstk;
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
