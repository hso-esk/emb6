/*============================================================================*/
/**
 * \file    rtc.c
 *
 * \author  Manuel Schappacher
 *
 * \brief   Real Time Clock
 *
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <stdlib.h>
#include <stdint.h>
#include <msp430.h>
#include "targetconfig.h"
#include "hal_types.h"
#include "rtc.h"
#include "int.h"


/*============================================================================*/
/*                            LOCAL VARIABLES                                 */
/*============================================================================*/

/** pointer to the callback function for the interrupt */
static pf_rtc_cb gpf_rtc_cb = NULL;


/*============================================================================*/
/*                          LOCAL FUNCTION PROTOTYPES                         */
/*============================================================================*/

/* prototype for the RTC interrupt */
static void _rtc_irq( void* p_params );

/*============================================================================*/
/*                             LOCAL FUNCTIONS                                */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief   RTC interrupt handler.
 *
 *          Handles interrupts inovked by the Real-Time Clock. Currently
 *          "minute interrupts" will be handled.
 */
/*============================================================================*/
static void _rtc_irq( void* p_params )
{
  if( RTCIV & RTCTEVIFG )
  {
    /* get the current time from RTC */
    s_rtc_time_t s_time;
    rtc_getTime( &s_time );

    /* callback */
    if( gpf_rtc_cb != NULL )
    {
      gpf_rtc_cb( &s_time );
    }
  }
}

/*============================================================================*/
/*                              FUNCTIONS                                     */
/*============================================================================*/

/*=============================================================================
 *  rtc_init()
 *============================================================================*/
void rtc_init(void)
{
  gpf_rtc_cb = NULL;
  /* Configure RTC_A */
  RTCCTL01 |= RTCTEVIE + RTCHOLD + RTCMODE;
} /* rtc_init() */

/*=============================================================================
 *  rtc_start()
 *============================================================================*/
void rtc_start( pf_rtc_cb pf_cb )
{
  /* register the RTC interrupt */
  int_irqRegister( E_INT_IRQ_SRC_RTC, _rtc_irq );
  gpf_rtc_cb = pf_cb;

  /* start calendar mode */
  RTCCTL01 &= ~(RTCHOLD);

} /* rtc_start() */

/*=============================================================================
 *  rtc_setTime()
 *============================================================================*/
void rtc_setTime( s_rtc_time_t* ps_time )
{
  /* stop RTC */
  uint16_t ui_reg = RTCCTL01;

  /* Set year month and day */
  RTCYEAR = ps_time->ui_year;
  RTCMON = ps_time->uc_mon;
  RTCDAY = ps_time->uc_day;

  /* Set hour minute and second */
  RTCHOUR = ps_time->uc_hour;
  RTCMIN = ps_time->uc_min;
  RTCSEC = ps_time->uc_sec;

  /* start RTC */
  RTCCTL01 = ui_reg;

} /* rtc_setTime() */

/*=============================================================================
 *  rtc_getTime()
 *============================================================================*/
void rtc_getTime( s_rtc_time_t* ps_time )
{
  if (ps_time != NULL)
  {
    /* Retrieve time from RTC */
    ps_time->ui_year = RTCYEAR;
    ps_time->uc_mon = RTCMON;
    ps_time->uc_day = RTCDAY;
    ps_time->uc_hour = RTCHOUR;
    ps_time->uc_min = RTCMIN;
    ps_time->uc_sec = RTCSEC;
  }
} /* rtc_getTime() */


