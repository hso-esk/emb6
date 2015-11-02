/**
 * @file    lpr_null.c
 * @date    16.10.2015
 * @author  PN
 */


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"


#if NETSTK_CFG_LPR_NULL_EN
/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
static void LPR_Init(void *p_netstk, e_nsErr_t *p_err);
static void LPR_On(e_nsErr_t *p_err);
static void LPR_Off(e_nsErr_t *p_err);
static void LPR_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void LPR_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void LPR_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static s_ns_t       *LPR_Netstk;
static void         *LPR_CbTxArg;
static nsTxCbFnct_t LPR_CbTxFnct;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsLPR_t LPRDrvNull =
{
   "LPR NULL",
    LPR_Init,
    LPR_On,
    LPR_Off,
    LPR_Send,
    LPR_Recv,
    LPR_IOCtrl
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
void LPR_Init(void *p_netstk, e_nsErr_t *p_err)
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


    LPR_Netstk = p_netstk;
    LPR_CbTxArg = NULL;
    LPR_CbTxFnct = 0;
    *p_err = NETSTK_ERR_NONE;
}


/**
 * @brief   Turn driver on
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
void LPR_On(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif


    LPR_Netstk->rf->on(p_err);
}


/**
 * @brief   Turn driver off
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
void LPR_Off(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif


    LPR_Netstk->rf->off(p_err);
}


/**
 * @brief   Frame transmission handler
 *
 * @param   p_data      Pointer to buffer holding frame to send
 * @param   len         Length of frame to send
 * @param   p_err       Pointer to a variable storing returned error code
 */
void LPR_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
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


    LPR_Netstk->rf->send(p_data, len, p_err);
    LPR_CbTxFnct(LPR_CbTxArg, p_err);
}


/**
 * @brief   Frame reception handler
 *
 * @param   p_data      Pointer to buffer holding frame to receive
 * @param   len         Length of frame to receive
 * @param   p_err       Pointer to a variable storing returned error code
 */
void LPR_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
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


    LPR_Netstk->phy->recv(p_data, len, p_err);
}


/**
 * @brief    Miscellaneous commands handler
 *
 * @param   cmd         Command to be issued
 * @param   p_val       Pointer to a variable related to the command
 * @param   p_err       Pointer to a variable storing returned error code
 */
void LPR_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
{
    *p_err = NETSTK_ERR_NONE;
    switch (cmd) {
        case NETSTK_CMD_TX_CBFNCT_SET:
            if (p_val == NULL) {
                *p_err = NETSTK_ERR_INVALID_ARGUMENT;
            } else {
                LPR_CbTxFnct = (nsTxCbFnct_t)p_val;
            }
            break;

        case NETSTK_CMD_TX_CBARG_SET:
            LPR_CbTxArg = p_val;
            break;

        case NETSTK_CMD_PHY_RSVD:
            break;

        case NETSTK_CMD_LPR_RSVD:
            *p_err = NETSTK_ERR_NONE;
            break;

        default:
            LPR_Netstk->rf->ioctrl(cmd, p_val, p_err);
            break;
    }
}


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
#endif /* NETSTK_CFG_LPR_NULL_EN */
