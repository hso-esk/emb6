/**
 * @file    lpr_apss.c
 * @date    16.10.2015
 * @author  PN
 */


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6_conf.h"


#if NETSTK_LPR_NULL_EN
/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
void LPR_Init(void *p_netstk, NETSTK_ERR *p_err);
void LPR_On(NETSTK_ERR *p_err);
void LPR_Off(NETSTK_ERR *p_err);
void LPR_Send(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err);
void LPR_Recv(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err);
void LPR_IOCtrl(NETSTK_IOC_CMD cmd, NETSTK_CMD_VAL *p_val, NETSTK_ERR *p_err);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static s_ns_t *LPR_Netstk;

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
NETSTK_MODULE_DRV _LPRDrvAPSS =
{
   "LPR APSS",
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
void LPR_Init(void *p_netstk, NETSTK_ERR *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_netstk == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }
#endif


    LPR_Netstk = p_netstk;
    *p_err = NETSTK_ERR_NONE;
}


void LPR_On(NETSTK_ERR *p_err)
{
    LPR_Netstk->rf->on(p_err);
}


void LPR_Off(NETSTK_ERR *p_err)
{
    LPR_Netstk->rf->off(p_err);
}


void LPR_Send(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err)
{
    LPR_Netstk->rf->send(p_data, len, p_err);
}


void LPR_Recv(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err)
{
    LPR_Netstk->phy->recv(p_data, len, p_err);
}


void LPR_IOCtrl(NETSTK_IOC_CMD cmd, NETSTK_CMD_VAL *p_val, NETSTK_ERR *p_err)
{
    switch (cmd) {
        case NETSTK_CMD_LPR_XXX:
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
#endif /* #if NETSTK_LPR_APSS_EN */
