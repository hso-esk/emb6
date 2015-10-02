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
/**  \addtogroup emb6
 *      @{
 *      \addtogroup bsp Board Support Package
 *   @{
 *   \defgroup board Board specific configurations
 *
 *   Configurations for multiple development boards
 *   @{
 *
 *      \addtogroup atany900 Board AT-ANY-900 specific configuration
 *   @{
 */
/*! \file   atany900/board_conf.h

    \author Artem Yushev, 

    \brief  Board Configuration for AT-ANY-900

    \version 0.0.1
*/
#ifndef BOARD_CONF_H_
#define BOARD_CONF_H_


#include "emb6.h"
#include "avr/io.h"
#include "avr/interrupt.h"


#define RESET_DDR                               &(PORTA)
#define RESET_PORT                              &(PORTA)
#define RESET_PIN                               0x07
#define SLP_TR_DDR                              &(DDRB)
#define SLP_TR_PORT                             &(PORTB)
#define SLP_TR_PIN                              0x04
#define RADIO_INT_SOURCE                        0x05

#define HAL_LED_RED_PORT                        B
#define HAL_LED_YELLOW_PORT                     B
#define HAL_LED_GREEN_PORT                      B

#define HAL_LED_RED_PIN                         5
#define HAL_LED_YELLOW_PIN                      6
#define HAL_LED_GREEN_PIN                       7

#define HAL_LED_SET                             &= ~
#define HAL_LED_CLR                             |=


/*============================================================================*/
/*!
\brief    emb6 board configuration fuction

        This function chooses the transceiver driver for the specific board.

\param    pointer to global netstack struct

\return  success 1, failure 0

*/
/*============================================================================*/
uint8_t board_conf(s_ns_t* ps_nStack);


#endif /* BOARD_CONF_H_ */

/** @} */
/** @} */
/** @} */
/** @} */
