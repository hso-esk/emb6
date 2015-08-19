#ifndef HWINIT_H_
#define HWINIT_H_

/**
 * \addtogroup mcu MCU HAL library
 * @{
 * \defgroup linux PC emulation HAL library
 * @{
 *
 */
/*============================================================================*/
/*! \file   linux/hwinit.h

    \author Artem Yushev artem.yushev@hs-offenbrug.de

    \brief  This is an PC emulation library for upper layers.

   \version 0.0.1
*/
/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
==============================================================================*/


/*==============================================================================
                                     MACROS
==============================================================================*/

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

/* The AVR tick interrupt usually is done with an 8 bit counter around 128 Hz.
 * 125 Hz needs slightly more overhead during the interrupt, as does a 32 bit
 * clock_time_t.
 */
/* Clock ticks per second */
#define CLOCK_SECOND                         1000
#define CLOCK_LT(a,b)                          ((signed long)((a)-(b)) < 0)
#define INFINITE_TIME                         0xffffffff
#define RIME_CONF_BROADCAST_ANNOUNCEMENT_MAX_TIME INFINITE_TIME/CLOCK_CONF_SECOND /* Default uses 600 */
#define COLLECT_CONF_BROADCAST_ANNOUNCEMENT_MAX_TIME INFINITE_TIME/CLOCK_CONF_SECOND /* Default uses 600 */

// Macro for delay. It is essential to put some delay in linux emulation
// as without it program will "eat" all of a process working time
#define HOWMUCH                                500
#define BSP_DELAY(a)                        bsp_delay_us(a)
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
extern const uint8_t                         mac_address[8];

#endif /* HWINIT_H_ */
/** @} */
/** @} */
