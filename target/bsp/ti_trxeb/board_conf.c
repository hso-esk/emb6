/*
 * board_conf.c
 *
 *  Created on: 06.10.2015
 *      Author: nphuong
 */


#include "emb6_conf.h"
#include "board_conf.h"



/** Enable or disable logging */
#define        LOGGER_ENABLE          LOGGER_BSP
#include "logger.h"


/**
 * @brief   Configure Board Support Package module
 *
 * @param   ps_ns   Pointer to net stack structure
 * @return  1 if success; otherwise 0
 */
uint8_t board_conf(s_ns_t *p_netstk)
{
    uint8_t     c_ret = 1;
    e_nsErr_t   err = NETSTK_ERR_NONE;


    if (p_netstk != NULL) {
        p_netstk->llc   = &LLCDrv802154;
        p_netstk->mac   = &MACDrv802154;
        p_netstk->phy   = &PHYDrvNull;
        p_netstk->lpr   = &LPRDrvNull;
        p_netstk->rf    = &RFDrvCC1120;
    }
    else {
        LOG_ERR("Network stack pointer is NULL");
    }

    return c_ret;
}
