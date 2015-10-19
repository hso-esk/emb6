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
#include "emb6_conf.h"


#if NETSTK_CFG_LLC_NULL_EN
/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
void LLC_Init(void *p_netstk, NETSTK_ERR *p_err);
void LLC_On(NETSTK_ERR *p_err);
void LLC_Off(NETSTK_ERR *p_err);
void LLC_Send(NETSTK_ERR *p_err);
void LLC_Recv(NETSTK_ERR *p_err);
void LLC_IOCtrl(NETSTK_IOC_CMD cmd, void *p_val, NETSTK_ERR *p_err);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static s_ns_t *LLC_Netstk;

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
NETSTK_MODULE_DRV LLCDrvNull =
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
void LLC_Init(void *p_netstk, NETSTK_ERR *p_err)
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


void LLC_On(NETSTK_ERR *p_err)
{
    LLC_Netstk->mac->on(p_err);
}


void LLC_Off(NETSTK_ERR *p_err)
{
    LLC_Netstk->mac->off(p_err);
}


void LLC_Send(NETSTK_ERR *p_err)
{
    LLC_Netstk->mac->send(p_err);
}


void LLC_Recv(NETSTK_ERR *p_err)
{

}


void LLC_IOCtrl(NETSTK_IOC_CMD cmd, void *p_val, NETSTK_ERR *p_err)
{
    switch (cmd) {
        case NETSTK_CMD_LLC_XXX:
            *p_err = NETSTK_ERR_NONE;
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
#endif /* #if NETSTK_CFG_LLC_NULL_EN */
