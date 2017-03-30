 /**
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file       sf_mcu.c
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
 * @author     STACKFORCE
 * @brief      Handles the MCU.
 */

#define __DECL_SF_MCU_API_H__
#define __DECL_SF_MCU_H__

#ifndef USE_TI_RTOS
#error Please define if TI-RTOS is in use ore not
#endif /* USE_TI_RTOS */
/*==============================================================================
                            INCLUDE FILES
==============================================================================*/
/* Standart libraries */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Stack includes */
#include "sf_mcu.h"
/* Chip specific */
#include "driverlib/pwr_ctrl.h"
#include "driverlib/sys_ctrl.h"
#include "driverlib/interrupt.h"


#if !USE_TI_RTOS
/* BoardSupportPacket */
#include "bsp/srf06eb_cc26xx/drivers/source/bsp.h"
#else
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#endif /* !USE_TI_RTOS */
/*==============================================================================
                            DEFINES
==============================================================================*/

/*==============================================================================
                            FUNCTIONS
==============================================================================*/

/*============================================================================*/
/* sf_mcu_init() */
/*============================================================================*/
bool sf_mcu_init(void)
{
  #if !USE_TI_RTOS
  /* 0.3. Initialize the device and set the mcu speed to BSP_SYS_CLK_SPD (48MHz) */
  bspInit(BSP_SYS_CLK_SPD);

  /* 0.4. Configure DCDC */
  PowerCtrlSourceSet(PWRCTRL_PWRSRC_DCDC);
  #endif /* !USE_TI_RTOS */
  return true;
} /* sf_mcu_init() */


/*============================================================================*/
/*! sf_mcu_reset() */
/*============================================================================*/
void sf_mcu_reset(void)
{
  SysCtrlSystemReset();
} /* sf_mcu_reset() */

/*============================================================================*/
/*! sf_mcu_interrutpEnable() */
/*============================================================================*/
void sf_mcu_interruptEnable(void)
{
  IntMasterEnable();
}/* sf_mcu_interruptEnable() */

/*============================================================================*/
/*! sf_mcu_interrutpEnable() */
/*============================================================================*/
void sf_mcu_interruptDisable(void)
{
  IntMasterDisable();
}/* sf_mcu_interruptDisable() */
