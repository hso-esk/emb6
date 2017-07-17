




/*==============================================================================
                            INCLUDE FILES
==============================================================================*/
#include "rtimer_arch.h"
#include "em_rtc.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_common.h"
#include "em_int.h"

/*============================================================================*/
/* rtimer_arch_init() */
/*============================================================================*/
static const RTC_Init_TypeDef initRTC =
{
  true,  // Start counting when init completed.
  false, // Disable updating RTC during debug halt.
  false  // Count until max. to wrap around.
};

void rtimer_arch_init(void)
{
  // Ensure LE modules are clocked.
  CMU_ClockEnable( cmuClock_CORELE, true );

  // Enable LFACLK in CMU (will also enable oscillator if not enabled).
  CMU_ClockSelectSet( cmuClock_LFA, cmuSelect_LFXO );

  // Set clock divider.
  CMU_ClockDivSet( cmuClock_RTC, cmuClkDiv_1 );

  // Enable RTC module clock.
  CMU_ClockEnable( cmuClock_RTC, true );

  // Initialize RTC.
  RTC_Init( &initRTC );

  // Disable RTC/RTCC interrupt generation.
  RTC_IntDisable( _RTC_IF_MASK );
  RTC_IntClear( _RTC_IF_MASK );

  RTC_CounterReset();

  // Clear and then enable RTC interrupts in NVIC.
  NVIC_ClearPendingIRQ(RTC_IRQn);
  NVIC_EnableIRQ(RTC_IRQn);

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
  RTC_IntClear( RTC_IF_COMP1 );

  RTC_CompareSet( 1, (t) & _RTC_COMP1_MASK );
    // Start the timer system by enabling the compare interrupt.
  RTC_IntEnable( RTC_IF_COMP1 );
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
  return  RTC_CounterGet();
}
/*==============================================================================
                            INTERRUPTS
==============================================================================*/
/*---------------------------------------------------------------------------*/
/* The RTC interrupt handler */
void RTCIntHandler(void)
{

}
