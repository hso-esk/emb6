 /**
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file       board_conf.c
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
 * @author     STACKFORCE
 * @brief      Board configuration.
 */

/*============================================================================*/
/*                                 INCLUDES                                   */
/*============================================================================*/
#include "emb6.h"
#include "board_conf.h"


/*! Enable or disable logging. */
#define LOGGER_ENABLE       LOGGER_BSP
#include "logger.h"


/*============================================================================*/
/*                              board_conf() */
/*============================================================================*/
int8_t board_conf(s_ns_t *p_netstk)
{
    uint8_t c_ret = 0;

      if (p_netstk != NULL) {
        p_netstk->dllc = &dllc_driver_802154;
        p_netstk->mac  = &mac_driver_802154;
        p_netstk->phy  = &phy_driver_802154;
        p_netstk->rf   = &rf_driver_ticc13xx;
      } else {
        LOG_ERR("Network stack pointer is NULL");
        c_ret = -1;
      }

      return c_ret;
}
