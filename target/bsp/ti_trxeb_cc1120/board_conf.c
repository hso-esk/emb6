<<<<<<< HEAD:target/bsp/ti_trxeb/board_conf.c
/*
 * board_conf.c
 *
 *  Created on: 06.10.2015
 *      Author: nphuong
 */


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"
#include "board_conf.h"


/** Enable or disable logging */
#define LOGGER_ENABLE       LOGGER_BSP
#include "logger.h"


/**
 * @brief   Configure Board Support Package module
 *
 * @param   ps_ns   Pointer to net stack structure
 * @return  1 if success; otherwise 0
 */
uint8_t board_conf(s_ns_t *p_netstk)
{
    uint8_t c_ret = 1;


    if (p_netstk != NULL) {
        p_netstk->dllc  = &DLLCDrv802154;
        p_netstk->mac   = &MACDrv802154;
        p_netstk->phy   = &PHYDrv802154;
        p_netstk->rf    = &RFDrvCC112x;
    }
    else {
        LOG_ERR("Network stack pointer is NULL");
        c_ret = 0;
    }

    return c_ret;
}
=======
/*
 * board_conf.c
 *
 *  Created on: 06.10.2015
 *      Author: nphuong
 */


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"
#include "board_conf.h"


/** Enable or disable logging */
#define LOGGER_ENABLE       LOGGER_BSP
#include "logger.h"


/**
 * @brief   Configure Board Support Package module
 *
 * @param   ps_ns   Pointer to net stack structure
 * @return  1 if success; otherwise 0
 */
uint8_t board_conf(s_ns_t *p_netstk)
{
    uint8_t c_ret = 1;


    if (p_netstk != NULL) {
        p_netstk->llc   = &LLCDrv802154;
        p_netstk->mac   = &MACDrv802154;
        p_netstk->phy   = &PHYDrvNull;
        p_netstk->rf    = &RFDrvCC1120;
    }
    else {
        LOG_ERR("Network stack pointer is NULL");
        c_ret = 0;
    }

    return c_ret;
}
>>>>>>> 2910463... Created board configurations for TI transceiver CC112x and CC120x:target/bsp/ti_trxeb_cc1120/board_conf.c
