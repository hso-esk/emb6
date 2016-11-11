/*
 * emb6 is licensed under the 3-clause BSD license. This license gives everyone
 * the right to use and distribute the code, either in binary or source code
 * format, as long as the copyright license is retained in the source code.
 *
 * The emb6 is derived from the Contiki OS platform with the explicit approval
 * from Adam Dunkels. However, emb6 is made independent from the OS through the
 * removal of protothreads. In addition, APIs are made more flexible to gain
 * more adaptivity during run-time.
 *
 * The license text is:
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**  	\addtogroup emb6
 *   @{
 *   	\addtogroup bsp Board Support Package
 *   @{
 *   	\addtogroup board
 *   @{
 *      \addtogroup mosesstick Board EFM32 Leopard Gecko Starter Kit specific configuration
 *   @{
 */
/*! \file   mosesstick/board_conf.c

    \author Phuong Nguyen,

    \brief  Board Configuration for EFM32 Leopard Gecko Starter Kit

    \version 0.0.1
*/

/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "board_conf.h"

#include "hwinit.h"
#include "emb6.h"
#include "logger.h"
#include "bsp.h"

/** Enable or disable logging */
#define        LOGGER_ENABLE          LOGGER_BSP

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
/** @} */
/** @} */
/** @} */
/** @} */
