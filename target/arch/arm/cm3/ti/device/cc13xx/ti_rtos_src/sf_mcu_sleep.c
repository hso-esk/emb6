
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
#if USE_TI_RTOS
#include <xdc/runtime/Error.h>
#include "emb6_semaphore.h"
#endif
#include "driverlib/pwr_ctrl.h"
#include "driverlib/flash.h"
#include "driverlib/vims.h"
#include "driverlib/sys_ctrl.h"



#include "sf_mcu_sleep.h"
#include "sf_mcu.h"

#if !USE_TI_RTOS
#error Please define a sleep handling for non RTOS devices
#endif /* !USE_TI_RTOS */
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
  if((e_sleepMode == E_MCU_SLEEP_MODE_SLEEP) ||
      (e_sleepMode == E_MCU_SLEEP_MODE_DEEPSLEEP))
  {
    /* Set the global variable to the selected sleep mode */
    ge_sleepMode = e_sleepMode;

    /* Block the wmbus task until the next opperation is needed */
    semaphore_pend();

    /* At this point the task is not longer blocked device will wake up */
    sf_mcu_sleep_wakeUp();
  }/* if */
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
