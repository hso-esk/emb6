#ifndef __RTC_H__
#define __RTC_H__

/*============================================================================*/
/**
 * \file    rtc.h
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
#include <msp430.h>
#include <stdint.h>
#include "targetconfig.h"

/*============================================================================*/
/*                           STRUCTURES AND TYPEDEFS                          */
/*============================================================================*/

/**
 * RTC time definition
 */
typedef struct S_RTC_TIME_T
{
  /** Year */
  uint16_t ui_year;
  /** Month */
  uint8_t uc_mon;
  /** Day */
  uint8_t uc_day;
  /** Hour */
  uint8_t uc_hour;
  /** Minute */
  uint8_t uc_min;
  /** Seconds */
  uint8_t uc_sec;

} s_rtc_time_t;

/** function definition for the RTC callback */
typedef void (*pf_rtc_cb)( s_rtc_time_t* ps_time );

/*============================================================================*/
/*                          FUNCTION PROTOTYPES                               */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief    Function initializes the RTC.
 *
 *           This function initializes the Real-Time Clock. After the
 *           initialization the clock is halted and must be started
 *           seperately.
 */
/*============================================================================*/
void rtc_init( void );


/*============================================================================*/
/**
 * @brief    Start the RTC.
 *
 *           This function starts the RTC. It also enables Interrupts for
 *           "minute events". Every minute an Interrupt will be generated and
 *           the according callback will be invoked.
 *
 * @param    pf_cb      Callback to register for the Interrupt.
 */
/*============================================================================*/
void rtc_start( pf_rtc_cb pf_cb );


/*============================================================================*/
/**
 * @brief    Set the time.
 *
 *           This function set the currrent time within the Real-Time
 *           Clock. Therefore the Clock will be stopped and restarted.
 *
 * @param    ps_time    Time to set.
 */
/*============================================================================*/
void rtc_setTime( s_rtc_time_t* ps_time );


/*============================================================================*/
/**
 * @brief    Get the time
 *
 *           This function retrieves the current time from the Real-Time Clock.
 *
 * @param    ps_time    Pointer to variable holding time to get.
 */
/*============================================================================*/
void rtc_getTime( s_rtc_time_t* ps_time );


#endif /* #ifndef __RTC_H__ */
