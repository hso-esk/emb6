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
#include "emb6_conf.h"

#if NETSTK_CFG_MAC_NULL_EN
/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
void MAC_Init(void *p_netstk, NETSTK_ERR *p_err);
void MAC_On(NETSTK_ERR *p_err);
void MAC_Off(NETSTK_ERR *p_err);
void MAC_Send(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err);
void MAC_Recv(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err);
void MAC_IOCtrl(NETSTK_IOC_CMD cmd, NETSTK_CMD_VAL *p_val, NETSTK_ERR *p_err);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static s_ns_t *MAC_Netstk;

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
NETSTK_MODULE_DRV MACDrvNull =
{
   "MAC NULL",
    MAC_Init,
    MAC_On,
    MAC_Off,
    MAC_Send,
    MAC_Recv,
    MAC_IOCtrl
};


/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
void MAC_Init(void *p_netstk, NETSTK_ERR *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_netstk == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }
#endif


    MAC_Netstk = p_netstk;
    *p_err = NETSTK_ERR_NONE;
}


void MAC_On(NETSTK_ERR *p_err)
{
    MAC_Netstk->phy->on(p_err);
}


void MAC_Off(NETSTK_ERR *p_err)
{
    MAC_Netstk->phy->off(p_err);
}


void MAC_Send(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err)
{
    MAC_Netstk->phy->send(p_data, len, p_err);
}


void MAC_Recv(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err)
{
    MAC_Netstk->llc->recv(p_data, len, p_err);
}


void MAC_IOCtrl(NETSTK_IOC_CMD cmd, NETSTK_CMD_VAL *p_val, NETSTK_ERR *p_err)
{
    switch (cmd) {
        case NETSTK_CMD_MAC_XXX:
            *p_err = NETSTK_ERR_NONE;
            break;

        default:
            MAC_Netstk->phy->ioctrl(cmd, p_val, p_err);
            break;
    }
}



/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
#endif /* #if NETSTK_MAC_DRV_NULL_EN */
