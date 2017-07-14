




/*==============================================================================
                            INCLUDE FILES
==============================================================================*/
#include "rtimer_arch.h"

/*==============================================================================
                          LOCAL FUNCTION PROTOTYPES
==============================================================================*/
void RTCIntHandler(void);

/*============================================================================*/
/* rtimer_arch_init() */
/*============================================================================*/
void rtimer_arch_init(void)
{

}/* rtimer_arch_init() */

/*============================================================================*/
/* rtimer_arch_schedule() */
/*============================================================================*/
/**
 * \brief Schedules an rtimer task to be triggered at time t
 * \param t The time when the task will need executed.
 *
 * \e t is an absolute time, in other words the task will be executed AT
 * time \e t, not IN \e t rtimer ticks.
 *
 * This function schedules a one-shot event with the RTC.
 *
 * This functions converts \e to a value suitable for the RTC.
 */
void
rtimer_arch_schedule(rtimer_clock_t t)
{
}

/*============================================================================*/
/* rtimer_arch_now() */
/*============================================================================*/
/**
 * \brief Returns the current real-time clock time
 * \return The current rtimer time in ticks
 *
 * The value is read from the AON RTC counter and converted to a number of
 * rtimer ticks
 *
 */
rtimer_clock_t
rtimer_arch_now()
{
  return 0;
}
/*==============================================================================
                            INTERRUPTS
==============================================================================*/
/*---------------------------------------------------------------------------*/
/* The RTC interrupt handler */
void RTCIntHandler(void)
{

}
