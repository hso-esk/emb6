




/*==============================================================================
                            INCLUDE FILES
==============================================================================*/
#include "rtimer_arch.h"
#include <driverlib/aon_event.h>
#include <driverlib/aon_rtc.h>
#include <driverlib/interrupt.h>

/*==============================================================================
                          LOCAL FUNCTION PROTOTYPES
==============================================================================*/
void AONRTCIntHandler(void);

/*============================================================================*/
/* rtimer_arch_init() */
/*============================================================================*/
void rtimer_arch_init(void)
{
	  bool interrupts_disabled;

	  /* Disable and clear interrupts */
	  interrupts_disabled = IntMasterDisable();

	  AONRTCDisable();

	  AONRTCEventClear(AON_RTC_CH0);

	  /* Setup the wakeup event */
	  AONEventMcuWakeUpSet(AON_EVENT_MCU_WU0, AON_EVENT_RTC_CH0);
	  AONRTCCombinedEventConfig(AON_RTC_CH0);

	  HWREG(AON_RTC_BASE + AON_RTC_O_SEC) = 0; // SOC_RTC_START_TICK_COUNT;

	  /* Enable the RTC */
	  AONRTCEnable();
	  IntRegister(INT_AON_RTC_COMB, &AONRTCIntHandler);
	  IntEnable(INT_AON_RTC_COMB);

	  /* Re-enable interrupts */
	  if(!interrupts_disabled) {
		  IntMasterEnable();
	  }

	  AONRTCCompareValueSet(AON_RTC_CH0, AONRTCCurrentCompareValueGet()+ 10000);
	  AONRTCChannelEnable(AON_RTC_CH0);

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
 * This function schedules a one-shot event with the AON RTC.
 *
 * This functions converts \e to a value suitable for the AON RTC.
 */
void
rtimer_arch_schedule(rtimer_clock_t t)
{
	/* rtc schedule one shot */
	uint32_t channel = AON_RTC_CH0;
	 if((channel != AON_RTC_CH0) && (channel != AON_RTC_CH1) && (channel != AON_RTC_CH2)) {
	    return;
	  }

	  /* Set the channel to fire a one-shot compare event at time==ticks */
	  AONRTCCompareValueSet(channel, t);
	  AONRTCChannelEnable(channel);
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
  return AONRTCCurrentCompareValueGet();
}
/*==============================================================================
                            INTERRUPTS
==============================================================================*/
/*---------------------------------------------------------------------------*/
/* The AON RTC interrupt handler */
void AONRTCIntHandler(void)
{

  if(AONRTCEventGet(AON_RTC_CH0)) {
	  AONRTCChannelDisable(AON_RTC_CH0);
    HWREG(AON_RTC_BASE + AON_RTC_O_EVFLAGS) = AON_RTC_EVFLAGS_CH0;
    rtimer_run_next();
  }
}
