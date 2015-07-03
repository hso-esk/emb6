/**
 * \addtogroup mcu MCU HAL library
 * @{
 * \defgroup samd20g18 MCU (SAMD20G18) HAL library
 * @{
 *
 */
/*============================================================================*/
/*! \file   samd20g18/hwinit.h

    \author Artem Yushev artem.yushev@hs-offenbrug.de

    \brief  Hardware dependent initialization header file for SAMD20G18.

   \version 0.0.1
*/

#ifndef HWINIT_H_
#define HWINIT_H_

/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
==============================================================================*/
// From module: Common SAM0 compiler driver
#include <compiler.h>
#include <status_codes.h>

// From module: Delay routines
#include <delay.h>

// From module: EXTINT - External Interrupt (Callback APIs)
#include <extint.h>
#include <extint_callback.h>

// From module: Interrupt management - SAM implementation
#include <interrupt.h>

// From module: PORT - GPIO Pin Control
#include <port.h>

// From module: Part identification macros
#include <parts.h>

// From module: SERCOM
#include <sercom.h>
#include <sercom_interrupt.h>

// From module: SERCOM SPI - Serial Peripheral Interface (Polled APIs)
#include <spi.h>

// From module: SERCOM USART - Serial Communications (Polled APIs)
#include <usart.h>

// From module: SYSTEM - Clock Management
#include <clock.h>
#include <gclk.h>

// From module: SYSTEM - Core System Driver
#include <system.h>

// From module: SYSTEM - I/O Pin Multiplexer
#include <pinmux.h>

// From module: SYSTEM - Interrupt Driver
#include <system_interrupt.h>

// From module: WDT - Watchdog Timer (Polled APIs)
#include <wdt.h>

// From module: RTC - Real Time Counter in Count Mode (Callback APIs)
#include <rtc_count.h>
#include <rtc_count_interrupt.h>
/*==============================================================================
                                     MACROS
==============================================================================*/
#define HOWMUCH
#define BSP_DELAY(a)
/* The AVR tick interrupt usually is done with an 8 bit counter around 128 Hz.
 * 125 Hz needs slightly more overhead during the interrupt, as does a 32 bit
 * clock_time_t.
 */
/* Clock ticks per second */
#define CLOCK_SECOND                         1000
#define CLOCK_LT(a,b)                          ((signed long)((a)-(b)) < 0)
#define INFINITE_TIME                         0xffffffff
#define RIME_CONF_BROADCAST_ANNOUNCEMENT_MAX_TIME \
                                            INFINITE_TIME/CLOCK_CONF_SECOND /* Default uses 600 */
#define COLLECT_CONF_BROADCAST_ANNOUNCEMENT_MAX_TIME \
                                            INFINITE_TIME/CLOCK_CONF_SECOND /* Default uses 600 */

# define EXTINT_CALLBACKS_MAX             10
# define     CONF_TICK_SEC                CLOCK_SECOND
/*==============================================================================
                                     ENUMS
==============================================================================*/

/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/

/*==============================================================================
                          GLOBAL VARIABLE DECLARATIONS
==============================================================================*/

/*==============================================================================
                                GLOBAL CONSTANTS
==============================================================================*/
#endif /* HWINIT_H_ */
/** @} */
/** @} */
