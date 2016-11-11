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
    p_netstk->dllc = &dllc_driver_802154;
#if (NETSTK_CFG_LOW_POWER_MODE_EN == TRUE)
    p_netstk->mac  = &mac_driver_smartmac;
#else
    p_netstk->mac  = &mac_driver_802154;
#endif
    p_netstk->phy  = &phy_driver_802154;
    p_netstk->rf   = &rf_driver_ticc112x;
  } else {
    LOG_ERR("Network stack pointer is NULL");
    c_ret = 0;
  }

  return c_ret;
}
