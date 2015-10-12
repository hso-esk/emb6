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
/**
 * \addtogroup bsp
 * @{
 * \addtogroup mcu MCU HAL library
 * @{
 */
/**
 * \addtogroup linux
 * @{
 *
 * This is an PC emulation library for upper layers.
 *
 */
/*! \file   linux/linux.c

    \author Artem Yushev 

    \brief  This is an PC emulation library for upper layers.

   \version 0.0.1
*/
/*============================================================================*/
/*==============================================================================
                                     MACROS
==============================================================================*/
#define     LOGGER_ENABLE        LOGGER_HAL
#define        _POSIX_C_SOURCE        199309L

/*==============================================================================
                                 INCLUDE FILES
==============================================================================*/
#include <stdio.h>

#include "target.h"
#include "hwinit.h"
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include "logger.h"
/*==============================================================================
                                     ENUMS
==============================================================================*/

/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/

/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
==============================================================================*/

/*==============================================================================
                          VARIABLE DECLARATIONS
==============================================================================*/
static    struct timespec             tim = {0,0};
/*==============================================================================
                                LOCAL CONSTANTS
==============================================================================*/


/*==============================================================================
                                LOCAL FUNCTIONS
==============================================================================*/

/*==============================================================================
                                 API FUNCTIONS
==============================================================================*/


/*==============================================================================
  hal_delay_us()
 =============================================================================*/
void    hal_delay_us(uint32_t l_delay)
{
    tim.tv_nsec = l_delay*1000;
    nanosleep(&tim, NULL);
} /* hal_delay_us() */


void hal_enterCritical(void){}
void hal_exitCritical(void){};
void    hal_ledOff(uint16_t ui_led){}
void    hal_ledOn(uint16_t ui_led){}
int8_t     hal_init (void){
    return 1;
}
void     hal_watchdogReset(void){}
void     hal_watchdogStart(void){}
void     hal_watchdogStop(void){}


uint8_t    hal_getrand(void)
{
    // We don't need special kind of seed or rand.
    srand(time(NULL));
    int r = rand();

    return ((uint8_t) r);
}

clock_time_t hal_getTRes(void)
{
    return 1000;
}
/*==============================================================================
  hal_getTick()
 =============================================================================*/
uint32_t     hal_getTick(void)
{
      struct timeval tv;

      gettimeofday(&tv, NULL);

      return ((tv.tv_sec * 1000 + tv.tv_usec / 1000) & 0xffffffff);
} /* hal_getTick() */

/*==============================================================================
  hal_getSec()
 =============================================================================*/
uint32_t     hal_getSec(void)
{
      struct timeval tv;

      gettimeofday(&tv, NULL);

      return tv.tv_sec;
} /* hal_getSec() */
/** @} */
/** @} */
/** @} */
