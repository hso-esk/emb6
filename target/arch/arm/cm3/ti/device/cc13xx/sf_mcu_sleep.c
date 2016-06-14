/**
* @code
*  ___ _____ _   ___ _  _____ ___  ___  ___ ___
* / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
* \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
* |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
* embedded.connectivity.solutions.==============
* @endcode
*
* @file       sf_mcu_sleep.c
* @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
* @author     STACKFORCE
* @brief      Low power implementation.
*/
#ifdef __cplusplus
extern "C" {
#endif

#define __DECL_SF_MCU_SLEEP_H__

/*==============================================================================
                            INCLUDE FILES
==============================================================================*/
/* Standart libraries */
#include <stdint.h>
#include <stdbool.h>

/* Chip specific */
#include "driverlib/pwr_ctrl.h"
#include "driverlib/flash.h"
#include "driverlib/vims.h"
#include "driverlib/sys_ctrl.h"

#include "sf_mcu_sleep.h"
#include "sf_mcu.h"

/*==============================================================================
                            VARIABLES
==============================================================================*/
E_MCU_SLEEP_MODE_t ge_sleepMode = E_MCU_SLEEP_MODE_NONE;
/*==============================================================================
                            FUNCTIONS
==============================================================================*/
/*============================================================================*/
/* sf_mcu_sleep_init()*/
/*============================================================================*/
void sf_mcu_sleep_init(void)
{
  /* Set the sleep mode in a defined state */
  ge_sleepMode = E_MCU_SLEEP_MODE_NONE;
}/* sf_mcu_sleep_init() */

/*============================================================================*/
/* sf_mcu_sleep() */
/*============================================================================*/
void sf_mcu_sleep(E_MCU_SLEEP_MODE_t e_sleepMode)
{
 switch(e_sleepMode)
 {
   case E_MCU_SLEEP_MODE_SLEEP:
     ge_sleepMode = E_MCU_SLEEP_MODE_SLEEP;
     /* ToDo Enter LPM0, interrupts enabled */
     break;

   case E_MCU_SLEEP_MODE_DEEPSLEEP:
     ge_sleepMode = E_MCU_SLEEP_MODE_DEEPSLEEP;
     /* ToDo Enter LPMx, interrupts enabled */
     break;
 }/* switch() */
} /* sf_mcu_sleep() */

/*============================================================================*/
/* sf_mcu_sleep_getMode() */
/*============================================================================*/
E_MCU_SLEEP_MODE_t sf_mcu_sleep_getMode(void)
{
  return ge_sleepMode;
}/* sf_mcu_sleep_getMode() */

/*============================================================================*/
/* sf_mcu_sleep_wakeUp() */
/*============================================================================*/
void sf_mcu_sleep_wakeUp(void)
{
  if(ge_sleepMode != E_MCU_SLEEP_MODE_NONE)
  {
    ge_sleepMode = E_MCU_SLEEP_MODE_NONE;
  }/* if */
} /* sf_mcu_sleep_wakeUp() */

#ifdef __cplusplus
}
#endif
