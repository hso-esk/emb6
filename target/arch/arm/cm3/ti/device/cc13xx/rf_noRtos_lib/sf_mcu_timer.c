 /**
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file       sf_mcu_timer.c
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
 * @author     STACKFORCE
 * @brief      Implementation of the mcu timer.
 */

#define __DECL_SF_MCU_TIMER_H__

/*==============================================================================
                            INCLUDE FILES
==============================================================================*/
/* Standart libraries */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <sf_mcu_timer.h>
#include <sf_mcu.h>
/** Driver lib include for the timer */
#include <driverlib/timer.h>
#include <driverlib/prcm.h>
#include <driverlib/interrupt.h>
/* BoardSupportPacket */
#include "bsp/srf06eb_cc26xx/drivers/source/bsp.h"
/*==============================================================================
                            VARIABLES
==============================================================================*/
static fp_mcu_timer_cb gfp_tick = NULL;
uint32_t gl_timerBase = GPT0_BASE;
uint32_t gl_timer = TIMER_A;
/** Variable to store the microseconds. */
static uint32_t gl_microSec = 0U;
/*==============================================================================
                            FUNCTION PROTOTYPES
==============================================================================*/
/* Interrupt function used by the timer */
void loc_timerIsr(void);

/*==============================================================================
                            FUNCTIONS
==============================================================================*/
/*============================================================================*/
/* sf_mcu_timer_init() */
/*============================================================================*/
bool sf_mcu_timer_init(uint16_t i_ticksPerSecond)
{
  uint32_t l_config = (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC_UP);
  uint32_t l_loadValue = 0x00U;

  /* Make sure the peripheral power domain is on using PRCMPowerDomainOn,
     power status can be checked with PRCMPowerDomainStatus */
  PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
  while(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON);

  /* Make sure the specific peripheral you want to use it enabled using
     PRCMPeripheralRunEnable(…) and then PRCMLoadSet() */
  PRCMPeripheralRunEnable(PRCM_PERIPH_TIMER0);
  /* Apply settings and wait for them to take effect */
  PRCMLoadSet();
  while(!PRCMLoadGet());

  /* Disable Timer function before configuring module */
  sf_mcu_timer_disable();

  /* Configure the timer as periodic timer. The counter is incremented based on
     the aux clock*/
  TimerConfigure(gl_timerBase, l_config);
  /* The 48MHz clock will not be prescaled */
  TimerPrescaleSet(gl_timerBase, gl_timer, 0U);
  /* To get an interrupt each 500us the timer Value is set to 24000
     24000 *(1/48000000) = 500Us  */
  l_loadValue = (BSP_SYS_CLK_SPD/i_ticksPerSecond);
  TimerIntervalLoadMode(gl_timerBase, gl_timer, TIMER_INTERVALLOAD_TIMEOUT);
  TimerLoadSet(gl_timerBase, gl_timer, l_loadValue);
  TimerIntRegister(gl_timerBase, gl_timer, loc_timerIsr);
  TimerIntEnable(gl_timerBase, TIMER_TIMA_TIMEOUT);

  IntMasterEnable();

  /* Converts i_ticksPerSecond to l_microSecCycle per tick. */
  gl_microSec = 1000000 / (uint32_t)i_ticksPerSecond;

  return true;
} /* sf_mcu_timer_init() */

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
  TimerEnable(gl_timerBase, gl_timer);
} /* sf_mcu_timer_enable() */

/*============================================================================*/
/* sf_mcu_timer_disable() */
/*============================================================================*/
void sf_mcu_timer_disable(void)
{
  TimerDisable(gl_timerBase, gl_timer);
} /* sf_mcu_timer_disable() */

/*==============================================================================
                            INTERRUPTS
==============================================================================*/

/*============================================================================*/
/* loc_timerIsr() */
/*============================================================================*/
void loc_timerIsr(void)
{
  /** Clears timer interrupt sources. */
  TimerIntClear(gl_timerBase, TIMER_TIMA_TIMEOUT);
  if(NULL != gfp_tick)
  {
    gfp_tick(gl_microSec);
  }/* if */
}/* loc_timerIsr() */
