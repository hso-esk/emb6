/**
 * @file    phy_null.c
 * @date    16.10.2015
 * @author  PN
 */


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"


#if NETSTK_CFG_PHY_NULL_EN
#include "lib_tmr.h"

#define     LOGGER_ENABLE        LOGGER_PHY
#include    "logger.h"

/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
static void PHY_Init(void *p_netstk, e_nsErr_t *p_err);
static void PHY_On(e_nsErr_t *p_err);
static void PHY_Off(e_nsErr_t *p_err);
static void PHY_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void PHY_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void PHY_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);
static void PHY_CbTx(void *p_arg, e_nsErr_t *p_err);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static s_ns_t *PHY_Netstk;
static void *PHY_CbTxArg;
static nsTxCbFnct_t PHY_CbTxFnct;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsPHY_t PHYDrvNull =
{
   "PHY NULL",
    PHY_Init,
    PHY_On,
    PHY_Off,
    PHY_Send,
    PHY_Recv,
    PHY_IOCtrl
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
static void PHY_Init(void *p_netstk, e_nsErr_t *p_err)
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


    PHY_Netstk = (s_ns_t *)p_netstk;
    PHY_CbTxFnct = 0;
    PHY_CbTxArg = NULL;
    *p_err = NETSTK_ERR_NONE;
}


/**
 * @brief   Turn driver on
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void PHY_On(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif


    PHY_Netstk->lpr->on(p_err);
}


/**
 * @brief   Turn driver off
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void PHY_Off(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif


    PHY_Netstk->lpr->off(p_err);
}


/**
 * @brief   Frame transmission handler
 *
 * @param   p_data      Pointer to buffer holding frame to send
 * @param   len         Length of frame to send
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void PHY_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
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


#if LOGGER_ENABLE
    /*
     * Logging
     */
    uint16_t data_len = len;
    uint8_t *p_dataptr = p_data;
    LOG_RAW("PHY_TX: ");
    while (data_len--) {
        LOG_RAW("%02x", *p_dataptr++);
    }
    LOG_RAW("\r\n====================\r\n");
#endif


    /*
     * set TX callback function and argument
     */
    PHY_Netstk->lpr->ioctrl(NETSTK_CMD_TX_CBFNCT_SET,
                            (void *)PHY_CbTx,
                            p_err);

    PHY_Netstk->lpr->ioctrl(NETSTK_CMD_TX_CBARG_SET,
                            NULL,
                            p_err);

    /*
     * Issue next lower layer to transmit the prepared frame
     */
    LED_TX_ON();
    PHY_Netstk->lpr->send(p_data, len, p_err);
    if (*p_err != NETSTK_ERR_NONE) {
        LED_TX_OFF();
    }
}


/**
 * @brief   Frame reception handler
 *
 * @param   p_data      Pointer to buffer holding frame to receive
 * @param   len         Length of frame to receive
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void PHY_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
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


#if LOGGER_ENABLE
    /*
     * Logging
     */
    uint16_t data_len = len;
    uint8_t *p_dataptr = p_data;
    LOG_RAW("\r\n====================\r\n");
    LOG_RAW("PHY_RX: ");
    while (data_len--) {
        LOG_RAW("%02x", *p_dataptr++);
    }
    LOG_RAW("\n\r");
#endif


    /*
     * Inform the next higher layer
     */
    PHY_Netstk->mac->recv(p_data, len, p_err);
}


/**
 * @brief    Miscellaneous commands handler
 *
 * @param   cmd         Command to be issued
 * @param   p_val       Pointer to a variable related to the command
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void PHY_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
{
    *p_err = NETSTK_ERR_NONE;
    switch (cmd) {
        case NETSTK_CMD_TX_CBFNCT_SET:
            if (p_val == NULL) {
                *p_err = NETSTK_ERR_INVALID_ARGUMENT;
            } else {
                PHY_CbTxFnct = (nsTxCbFnct_t)p_val;
            }
            break;

        case NETSTK_CMD_TX_CBARG_SET:
            PHY_CbTxArg = p_val;
            break;

        case NETSTK_CMD_PHY_RSVD:
            break;

        default:
            PHY_Netstk->lpr->ioctrl(cmd, p_val, p_err);
            break;
    }
}


/**
 * @brief   Callback transmission handler
 */
static void PHY_CbTx(void *p_arg, e_nsErr_t *p_err)
{
    if (PHY_CbTxFnct) {
        PHY_CbTxFnct(PHY_CbTxArg, p_err);
    }
}

/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
#endif /* #if NETSTK_PHY_NULL_EN */
