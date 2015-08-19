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
/*! \file   linux/target.c

    \author Artem Yushev artem.yushev@hs-offenburg.de

    \brief  This is an PC emulation library for upper layers.

   \version 0.0.1
*/
/*============================================================================*/
/*==============================================================================
                                     MACROS
==============================================================================*/
#define     LOGGER_ENABLE        LOGGER_HAL
#if            LOGGER_ENABLE==TRUE
#define     LOGGER_SUBSYSTEM    "hal"
#endif
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
  hal_enterCritical()
==============================================================================*/
void hal_enterCritical(void)
{
} /* hal_enterCritical() */

/*==============================================================================
  hal_exitCritical()
==============================================================================*/
void hal_exitCritical(void)
{
}/* hal_exitCritical() */

/*==============================================================================
  hwinit_init()
==============================================================================*/
int8_t hal_init (void)
{
}/* hal_init() */

/*==============================================================================
  hal_extIntInit()
 =============================================================================*/
uint8_t hal_extIntInit(en_targetExtInt_t e_intSource,  pfn_intCallb_t pfn_intCallback)
{
    return 0;
} /* hal_extIntInit() */

/*==============================================================================
  hal_spiInit()
 =============================================================================*/
void hal_spiInit(void)
{
} /* hal_spiInit() */

/*==============================================================================
  hal_delay_us()
 =============================================================================*/
void    hal_delay_us(uint32_t l_delay)
{
    tim.tv_nsec = l_delay*1000;
    nanosleep(&tim, NULL);
} /* hal_delay_us() */


/*==============================================================================
  hal_setPin()
 =============================================================================*/
void    hal_pinSet(void * p_pin)
{
} /* hal_setPin() */

/*==============================================================================
  hal_clrPin()
 =============================================================================*/
void    hal_pinClr(void * p_pin)
{
} /* hal_clrPin() */

/*==============================================================================
  hal_getPin()
 =============================================================================*/
uint8_t    hal_pinGet(void * p_pin)
{
    return NULL;
} /* hal_getPin() */

/*==============================================================================
  hal_spiRead()
 =============================================================================*/
uint8_t hal_spiRead(uint8_t c_address)
{
    return 0;
} /* hal_spiRead() */

/*==============================================================================
  hal_spiWrite()
 =============================================================================*/
void hal_spiWrite(uint8_t c_address, uint8_t c_value)
{
} /* hal_spiWrite() */

/*==============================================================================
  hal_spiSubRead()
 =============================================================================*/
uint8_t hal_spiSubRead(        uint8_t c_address,
                        uint8_t c_mask,
                        uint8_t c_position)
{
    return 0;
} /* hal_spiSubRead() */

/*==============================================================================
  hal_spiSubWrite()
 =============================================================================*/
void hal_spiSubWrite(    uint8_t c_address,
                    uint8_t c_mask,
                    uint8_t c_position,
                    uint8_t c_value)
{
} /* hal_spiSubWrite() */

/*==============================================================================
  hal_spiTranOpen()
 =============================================================================*/
void hal_spiTranOpen(void)
{
} /* hal_spiTranOpen() */

/*==============================================================================
  hal_spiTranClose()
 =============================================================================*/
void hal_spiTranClose(void)
{
} /* hal_spiTranClose() */

/*==============================================================================
  hal_spiTranWrite()
 =============================================================================*/
void hal_spiTranWrite(uint8_t c_value)
{
} /* hal_spiTranWrite() */

/*==============================================================================
  hal_spiTranRead()
 =============================================================================*/
uint8_t hal_spiTranRead(void)
{
    return    0;
} /* hal_spiTranRead() */

/*==============================================================================
  hal_spiTranRead()
 =============================================================================*/
void hal_spiTranWait(void)
{
} /* hal_spiTranWait() */

/*==============================================================================
  hal_spiTranRead()
 =============================================================================*/
void     hal_watchdogReset(void)
{
} /* hal_watchdogReset() */

/*==============================================================================
  hal_spiTranRead()
 =============================================================================*/
void     hal_watchdogStart(void)
{
} /* hal_watchdogStart() */

/*==============================================================================
  hal_spiTranRead()
 =============================================================================*/
void     hal_watchdogStop(void)
{
} /* hal_watchdogStop() */


/*==============================================================================
  hal_getTick()
 =============================================================================*/
uint32_t     hal_getTick(void)
{
      struct timeval tv;

      gettimeofday(&tv, NULL);

      return tv.tv_sec * 1000 + tv.tv_usec / 1000;
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
