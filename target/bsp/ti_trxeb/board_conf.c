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
uint8_t board_conf(s_ns_t* ps_ns)
{
    uint8_t     c_ret = 1;
    NETSTK_ERR  err = NETSTK_ERR_NONE;


    if (ps_ns != NULL) {
        ps_ns->rf = &RFDrvCC1120;
        ps_ns->rf->init(ps_ns, &err);
        if (err != NETSTK_ERR_NONE) {
            c_ret = 0;
        }
    }
    else {
        LOG_ERR("Network stack pointer is NULL");
    }

    return c_ret;
}
