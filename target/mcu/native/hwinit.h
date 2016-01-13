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
/*============================================================================*/

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

    \author Artem Yushev 

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
