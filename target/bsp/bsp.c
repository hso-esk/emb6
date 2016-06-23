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
/**  \addtogroup emb6
 *      @{
 *      \addtogroup bsp Board Support Package
 *   @{
 */
/*============================================================================*/
/*! \file   bsp.c

    \author Artem Yushev, 
            Phuong Nguyen
    \brief  Board support package implementation.

   \version 0.0.2
*/
/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#include "emb6.h"

#include "bsp.h"
#include "board_conf.h"
#include "etimer.h"
#include "random.h"
#include "evproc.h"


/*==============================================================================
                                     MACROS
 =============================================================================*/
#define     LOGGER_ENABLE        LOGGER_BSP
#include    "logger.h"


/*==============================================================================
                                     ENUMS
 =============================================================================*/


/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
 =============================================================================*/


/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
static        uint8_t bsp_wtgStop = 1;
static        uint8_t bsp_numNestedCriticalSection;
static        uint8_t bsp_spiLocked;
static struct etimer  bsp_ledsTims[LEDS_SUPPORTED];


/*==============================================================================
                                LOCAL CONSTANTS
 =============================================================================*/


/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
 =============================================================================*/
static void _bsp_ledCallback(c_event_t c_event, p_data_t p_data);


/*==============================================================================
                                LOCAL FUNCTIONS
 =============================================================================*/

/*============================================================================*/
/*  _bsp_ledCallback()                                                        */
/*============================================================================*/
static void _bsp_ledCallback(c_event_t c_event, p_data_t p_data)
{
  uint8_t j;
  for (j = 0; j < LEDS_SUPPORTED; j++) {
    if (etimer_expired(&bsp_ledsTims[j])) {
      if ((&(bsp_ledsTims[j])) == (struct etimer *) p_data) {
        // Timer has been found , so we can turn it off.
        hal_ledOff(j);
        etimer_stop((struct etimer *) p_data);
        break;
      }
    }
  }
} /* _bsp_ledCallback() */


/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

/*============================================================================*/
/*  bsp_init()                                                                */
/*============================================================================*/
uint8_t bsp_init (s_ns_t * ps_ns)
{
  /* Initialize hardware */
  if (!hal_init())
    return 0;

  /* Configure board */
  if (!board_conf(ps_ns))
    return 0;

  random_init(hal_getrand());

  /* initialize local variables */
  bsp_numNestedCriticalSection = 0;

  /* Normal exit*/
  return 1;
}/* bsp_init() */


/*============================================================================*/
/*  bsp_start()                                                               */
/*============================================================================*/
void bsp_start (void)
{

}/* bsp_start() */


/*============================================================================*/
/*  bsp_entry()                                                               */
/*============================================================================*/
void bsp_entry (void)
{
    /* Nothing to do */
}/* bsp_entry() */


/*============================================================================*/
/*  bsp_getChar()                                                             */
/*============================================================================*/
int bsp_getChar(void)
{
  /* Return the next character received using the console UART */
  /* Nothing to do */
  return 0;
} /* bsp_getChar() */


/*============================================================================*/
/*  bsp_led()                                                                 */
/*============================================================================*/
uint16_t bsp_led(en_bspLedIdx_t ui_led, en_bspLedAction_t en_ledAction)
{
  switch (en_ledAction) {
    case E_BSP_LED_ON:
      hal_ledOn(ui_led);
      break;
    case E_BSP_LED_OFF:
      hal_ledOff(ui_led);
      break;
    case E_BSP_LED_TOGGLE:
      hal_ledOn(ui_led);
      etimer_set(&bsp_ledsTims[ui_led], 20, _bsp_ledCallback);

      break;
    default:
      break;
  }
  return 0;
} /* bsp_led() */


/*============================================================================*/
/*  bsp_pinInit()                                                             */
/*============================================================================*/
void * bsp_pinInit(en_targetExtPin_t e_pinType)
{
  return hal_ctrlPinInit(e_pinType);
} /* bsp_pinInit() */


/*============================================================================*/
/*  bsp_pin()                                                                 */
/*============================================================================*/
uint8_t bsp_pin(en_bspPinAction_t e_pinAct, void * p_pin)
{
  switch (e_pinAct) {
    case E_BSP_PIN_SET:
      hal_pinSet(p_pin);
      break;
    case E_BSP_PIN_CLR:
      hal_pinClr(p_pin);
      break;
    case E_BSP_PIN_GET:
      return hal_pinGet(p_pin);
    default:
      return 1;
  }
  return 0;
} /* bsp_pin() */


/*============================================================================*/
/*  bsp_wdt()                                                                 */
/*============================================================================*/
void bsp_wdt(en_bspWdtAction_t en_wdtAct)
{
  switch (en_wdtAct) {
    case E_BSP_WDT_RESET:
      if (!bsp_wtgStop)
        hal_watchdogReset();
      break;
    case E_BSP_WDT_START:
      bsp_wtgStop--;
      if (!bsp_wtgStop)
        hal_watchdogStart();
      break;
    case E_BSP_WDT_STOP:
      bsp_wtgStop++;
      hal_watchdogStop();
      break;
    case E_BSP_WDT_PERIODIC:
      if (!bsp_wtgStop)
        hal_watchdogReset();
      break;
    default:
      break;
  }
} /* bsp_wdt() */


/*============================================================================*/
/*  bsp_get()                                                                 */
/*============================================================================*/
uint32_t bsp_get(en_bspParams_t en_param)
{
  uint32_t l_ret;
  switch (en_param) {
    case E_BSP_GET_TICK:
      l_ret = hal_getTick();
      break;
    case E_BSP_GET_SEC:
      l_ret = hal_getSec();
      break;
    case E_BSP_GET_TRES:
      l_ret = hal_getTRes();
      break;
    default:
      l_ret = 0;
      break;
  }
  return l_ret;
} /* bsp_get() */


/*============================================================================*/
/*  bsp_getrand()                                                             */
/*============================================================================*/
uint32_t bsp_getrand(uint32_t max)
{
  uint32_t ret;

  /* generate random number in a range of 0 to max */
  ret = (uint32_t) random_rand();
  ret = ret % max;
  return ret;
} /* bsp_getrand() */


/*============================================================================*/
/*  bsp_enterCritical()                                                       */
/*============================================================================*/
void bsp_enterCritical(void)
{
  /*
   * if bsp_numNestedCriticalSection is non-zero, then MCU is already in critical sections.
   * therefore there is no need to re-enter critical section.
   * Otherwise, hal_enterCritical() is called.
   */
  if (bsp_numNestedCriticalSection == 0) {
    hal_enterCritical();
  }

  /* increase number of nested critical sections */
  bsp_numNestedCriticalSection++;
} /* bsp_enterCritical() */


/*============================================================================*/
/*  bsp_exitCritical()                                                        */
/*============================================================================*/
void bsp_exitCritical(void)
{
  /*
   * If bsp_numNestedCriticalSection is non-zero, meaning that MCU is in nested critical sections.
   * Therefore it is NOT allowed to exit critical section.
   * Leaving critical section MUST be done in the most outer critical section.
   */
  if (bsp_numNestedCriticalSection > 0) {
    bsp_numNestedCriticalSection--;
    if (bsp_numNestedCriticalSection == 0) {
      /* the most outer critical section */
      hal_exitCritical();
    }
  }
} /* bsp_exitCritical() */


/*============================================================================*/
/*  bsp_getTick()                                                             */
/*============================================================================*/
clock_time_t bsp_getTick(void)
{
  return hal_getTick();
} /* bsp_getTick() */


/*============================================================================*/
/*  bsp_getSec()                                                              */
/*============================================================================*/
clock_time_t bsp_getSec(void)
{
  return hal_getSec();
} /* bsp_getSec() */


/*============================================================================*/
/*  bsp_delay_us()                                                            */
/*============================================================================*/
void bsp_delay_us(uint32_t i_delay)
{
  hal_delay_us(i_delay);
} /* bsp_delay_us() */


/*============================================================================*/
/*  bsp_extIntRegister()                                                      */
/*============================================================================*/
void bsp_extIntRegister(en_targetExtInt_t e_extInt, en_targetIntEdge_t e_edge,
    pfn_intCallb_t pfn_intCallback)
{
  hal_extiRegister(e_extInt, e_edge, pfn_intCallback);
} /* bsp_extIntRegister() */


/*============================================================================*/
/*  bsp_extIntClear()                                                         */
/*============================================================================*/
void bsp_extIntClear(en_targetExtInt_t e_extInt)
{
  hal_extiClear(e_extInt);
} /* bsp_extIntClear() */


/*============================================================================*/
/*  bsp_extIntEnable()                                                        */
/*============================================================================*/
void bsp_extIntEnable(en_targetExtInt_t e_extInt)
{
  hal_extiEnable(e_extInt);
} /* bsp_extIntEnable() */


/*============================================================================*/
/*  bsp_extIntDisable()                                                       */
/*============================================================================*/
void bsp_extIntDisable(en_targetExtInt_t e_extInt)
{
  hal_extiDisable(e_extInt);
} /* bsp_extIntDisable() */


/*============================================================================*/
/*  bsp_spiInit()                                                             */
/*============================================================================*/
void *bsp_spiInit(void)
{
  void *p_spi = NULL;

  p_spi = hal_spiInit();
  bsp_spiLocked = FALSE;
  return p_spi;
} /* bsp_spiInit() */


/*============================================================================*/
/*  bsp_spiSlaveSel()                                                         */
/*============================================================================*/
uint8_t bsp_spiSlaveSel(void *p_spi, bool enable)
{
  uint8_t ret = 1;

  if (enable == TRUE) {
    if (bsp_spiLocked == FALSE) {
      /* lock SPI handle */
      bsp_enterCritical();
      bsp_spiLocked = TRUE;

      /* enable SPI */
      ret = hal_spiSlaveSel(p_spi, TRUE);
    } else {
      /* SPI is being locked */
      ret = 0;
    }
  } else {
    if (bsp_spiLocked == TRUE) {
      /* unlock SPI handle */
      bsp_spiLocked = FALSE;

      /* disable SPI */
      ret = hal_spiSlaveSel(p_spi, FALSE);
      bsp_exitCritical();
    } else {
      /* SPI is already unlocked and disabled */
    }
  }
  return ret;
} /* bsp_spiSlaveSel() */


/*============================================================================*/
/*  bsp_spiTxRx()                                                             */
/*============================================================================*/
void bsp_spiTxRx(uint8_t *p_tx, uint8_t *p_rx, uint16_t len)
{
  hal_spiTxRx(p_tx, p_rx, len);
} /* bsp_spiTxRx() */


/*============================================================================*/
/*  bsp_spiRead()                                                             */
/*============================================================================*/
uint8_t bsp_spiRead(uint8_t * p_reg, uint16_t i_length)
{
  return hal_spiRead(p_reg, i_length);
} /* bsp_spiRead() */


/*============================================================================*/
/*  bsp_spiWrite()                                                            */
/*============================================================================*/
void bsp_spiWrite(uint8_t * value, uint16_t i_length)
{
  hal_spiWrite(value, i_length);
} /* bsp_spiWrite() */


/** @} */
/** @} */
