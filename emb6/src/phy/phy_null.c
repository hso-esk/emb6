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
#include "emb6_conf.h"


#define     LOGGER_ENABLE        LOGGER_PHY
#include    "logger.h"


#if NETSTK_CFG_PHY_NULL_EN
/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
void PHY_Init(void *p_netstk, NETSTK_ERR *p_err);
void PHY_On(NETSTK_ERR *p_err);
void PHY_Off(NETSTK_ERR *p_err);
void PHY_Send(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err);
void PHY_Recv(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err);
void PHY_IOCtrl(NETSTK_IOC_CMD cmd, void *p_val, NETSTK_ERR *p_err);

static void PHY_CbTx(void *p_arg, NETSTK_ERR *p_err);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static s_ns_t *PHY_Netstk;
static void *PHY_CbTxArg;
static NETSTK_CBFNCT PHY_CbTxFnct;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
NETSTK_MODULE_DRV PHYDrvNull =
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
void PHY_Init(void *p_netstk, NETSTK_ERR *p_err)
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
void PHY_On(NETSTK_ERR *p_err)
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
void PHY_Off(NETSTK_ERR *p_err)
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
void PHY_Send(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err)
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
    PHY_Netstk->lpr->send(p_data, len, p_err);
}


/**
 * @brief   Frame reception handler
 *
 * @param   p_data      Pointer to buffer holding frame to receive
 * @param   len         Length of frame to receive
 * @param   p_err       Pointer to a variable storing returned error code
 */
void PHY_Recv(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err)
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
void PHY_IOCtrl(NETSTK_IOC_CMD cmd, void *p_val, NETSTK_ERR *p_err)
{
    *p_err = NETSTK_ERR_NONE;
    switch (cmd) {
        case NETSTK_CMD_TX_CBFNCT_SET:
            if (p_val == NULL) {
                *p_err = NETSTK_ERR_INVALID_ARGUMENT;
            } else {
                PHY_CbTxFnct = (NETSTK_CBFNCT)p_val;
            }
            break;

        case NETSTK_CMD_TX_CBARG_SET:
            PHY_CbTxArg = p_val;
            break;

        case NETSTK_CMD_PHY_XXX:
            break;

        default:
            PHY_Netstk->lpr->ioctrl(cmd, p_val, p_err);
            break;
    }
}


/**
 * @brief   Callback transmission handler
 */
static void PHY_CbTx(void *p_arg, NETSTK_ERR *p_err)
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
