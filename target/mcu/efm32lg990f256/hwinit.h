/**
 * \addtogroup mcu MCU HAL library
 * @{
 * \defgroup efm32 MCU (EFM32LG990F256) HAL library
 * @{
 *
 */
/*============================================================================*/
/*! \file   efm32lg990f256/hwinit.h

    \author Manuel Schappacher manuel.schappacher@hs-offenburg.de

    \brief  Hardware dependent initialization header file for EFM32LG990F256.

   \version 0.0.1
*/

#ifndef HWINIT_H_
#define HWINIT_H_

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
# define CONF_TICK_SEC                    CLOCK_SECOND
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
