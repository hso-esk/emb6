
/*============================================================================*/
/*!
 * @file    sf_mcu_timerRtos.c
 *
 * @brief   TMR implementations.
 *
 * @author  Copyright (c) by Stackforce GmbH, Heitersheim, Germany,
 *          http://www.stackforce.de
 */
/*============================================================================*/
#define __DECL_SF_MCU_TIMER_H__

/*==============================================================================
                            INCLUDE FILES
==============================================================================*/
/* Standard libraries */
#include <stdint.h>
#include <stdbool.h>

#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Clock.h>

#include "sf_mcu_timer.h"
#include "sf_mcu_sleep.h"
#include "sf_mcu_timerRtos.h"

#include "emb6_semaphore.h"

/*==============================================================================
                            VARIABLES
==============================================================================*/
static fp_mcu_timer_cb gfp_tick = NULL;
/* Instance of the used clock */
Clock_Handle clockHandle;
/* Param for crating clock instance */
Clock_Params clkParams;
/*==============================================================================
                            FUNCTION PROTOTYPES
==============================================================================*/
/* Interrupt function used by the timer */
void loc_timerIsr(UArg arg0);

/*==============================================================================
                            FUNCTIONS
==============================================================================*/
/*============================================================================*/
/* sf_mcu_timerRtos_init() */
/*============================================================================*/
bool sf_mcu_timerRtos_init(uint16_t i_ticksPerSecond, Error_Block* ps_eb)
{
  /* Return value of this function */
  bool b_return = false;
  /** Variable to store the microseconds. */
  uint32_t l_microSec = 0U;


  /* Converts i_ticksPerSecond to system time units. */
  l_microSec = 100000 / (uint32_t)i_ticksPerSecond;

  /* Create a periodic Clock Instance with period = 50 system time units */
  Clock_Params_init(&clkParams);
  clkParams.arg = (l_microSec*10U);
  clkParams.period = l_microSec;
  clkParams.startFlag = false;
  clockHandle = Clock_create(loc_timerIsr, l_microSec, &clkParams, ps_eb);
  if(NULL != clockHandle)
  {
    b_return = true;
  }/* if */

  return b_return;

} /* sf_mcu_timerRtos_init() */

/*============================================================================*/
/* sf_mcu_timer_setCallback() */
/*============================================================================*/
bool sf_mcu_timer_setCallback(fp_mcu_timer_cb fp_tmr)
{
  bool b_ret = false;
  if(fp_tmr)
  {
    /* Save callback function. */
    gfp_tick = fp_tmr;
    b_ret = true;
  } /* if */

  return b_ret;
} /* sf_mcu_timer_setCallback() */

/*============================================================================*/
/* sf_mcu_timer_enable() */
/*============================================================================*/
void sf_mcu_timer_enable(void)
{
  Clock_start(clockHandle);
} /* sf_mcu_timer_enable() */

/*============================================================================*/
/* sf_mcu_timer_disable() */
/*============================================================================*/
void sf_mcu_timer_disable(void)
{
  Clock_stop(clockHandle);
} /* sf_mcu_timer_disable() */

/*==============================================================================
                            INTERRUPTS
==============================================================================*/

/*============================================================================*/
/* loc_timerIsr() */
/*============================================================================*/
void loc_timerIsr(UArg arg0)
{
  if(NULL != gfp_tick)
  {
    if(gfp_tick(arg0))
    {
      /* If the tick function returns with @ref true we have to check if we
       * have to unblock the wmbus_task */
      switch(sf_mcu_sleep_getMode())
      {
        case E_MCU_SLEEP_MODE_SLEEP:
        case E_MCU_SLEEP_MODE_DEEPSLEEP:
          semaphore_post();
          break;
        case E_MCU_SLEEP_MODE_NONE:
        default:
          break;
      } /* switch(sf_mcu_sleep_getMode()) */
    }
  }/* if */
}/* loc_timerIsr() */
