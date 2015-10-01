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
 *   \addtogroup board
 *   @{
 *      \addtogroup samd20xpro_rf212 Board SAM D20 Xplained Pro with AT86RF212 specific configuration
 *   @{
 */
/*! \file   samd20xpro_rf212/board_conf.h

    \author Artem Yushev, 

    \brief  Board Configuration for SAM D20 Xplained Pro Evaluation Kit

    \version 0.0.1
*/

#ifndef BOARD_CONF_H_
#define BOARD_CONF_H_


#include "emb6.h"

/*==============================================================================
                                     MACROS
==============================================================================*/


#define SAMD20_SPI0_SERCOM                        SERCOM0
#define SAMD20_SPI0_SERCOM_MUX_SETTING          SPI_SIGNAL_MUX_SETTING_E
#define SAMD20_SPI0_SERCOM_PMUX0                  PINMUX_PA04D_SERCOM0_PAD0
#define SAMD20_SPI0_SERCOM_PMUX1                  PINMUX_PA05D_SERCOM0_PAD1
#define SAMD20_SPI0_SERCOM_PMUX2                  PINMUX_PA06D_SERCOM0_PAD2
#define SAMD20_SPI0_SERCOM_PMUX3                  PINMUX_PA07D_SERCOM0_PAD3
#define SAMD20_SPI0_MOSI_PIN                    PIN_PA06
#define    SAMD20_SPI0_MISO_PIN                    PIN_PA04
#define    SAMD20_SPI0_SCK_PIN                        PIN_PA07
#define    SAMD20_SPI0_CS_PIN                        PIN_PA05
#define    SAMD20_SPI0_RST_PIN                        PIN_PA08
#define    SAMD20_SPI0_SLP_PIN                        PIN_PB08
#define    SAMD20_RADIO_IRQ_PIN                    PIN_PB09
#define    SAMD20_RADIO_IRQ_PINMUX                    PINMUX_PB09A_EIC_EXTINT9
#define    SAMD20_RADIO_IRQ_INPUT                    9


#define SAMD20_USART0_SERCOM                    SERCOM3
#define SAMD20_USART0_BAUDRATE                    38400
#define SAMD20_USART0_SERCOM_MUX_SETTING        USART_RX_3_TX_2_XCK_3
#define SAMD20_USART0_SERCOM_PMUX0                PINMUX_UNUSED
#define SAMD20_USART0_SERCOM_PMUX1                PINMUX_UNUSED
#define SAMD20_USART0_SERCOM_PMUX2                PINMUX_PA24C_SERCOM3_PAD2
#define SAMD20_USART0_SERCOM_PMUX3                PINMUX_PA25C_SERCOM3_PAD3

/** \name LED0 definitions
 *  @{ */
#define LED0_PIN                  PIN_PA14
#define LED0_ACTIVE               false
#define LED0_INACTIVE             !LED0_ACTIVE

#define LED1_PIN                  PIN_PA14
#define LED1_ACTIVE               false
#define LED1_INACTIVE             !LED0_ACTIVE

#define LED2_PIN                  PIN_PA14
#define LED2_ACTIVE               false
#define LED2_INACTIVE             !LED0_ACTIVE
/** @} */

/**
 * \name LED #0 definitions
 *
 * Wrapper macros for LED0, to ensure common naming across all Xplained Pro
 * boards.
 *
 *  @{ */
#define LED_0_NAME                "LED0 (yellow)"
#define LED_0_PIN                 LED0_PIN
#define LED_0_ACTIVE              LED0_ACTIVE
#define LED_0_INACTIVE            LED0_INACTIVE

#define LED_1_NAME                "LED0 (yellow)"
#define LED_1_PIN                 LED0_PIN
#define LED_1_ACTIVE              LED0_ACTIVE
#define LED_1_INACTIVE            LED0_INACTIVE

#define LED_2_NAME                "LED0 (yellow)"
#define LED_2_PIN                 LED0_PIN
#define LED_2_ACTIVE              LED0_ACTIVE
#define LED_2_INACTIVE            LED0_INACTIVE
/** @} */


/*============================================================================*/
/*!
\brief    emb6 board configuration fuction

        This function chooses the transceiver driver for the specific board.

\param    ps_nStack pointer to global netstack struct

\return  success 1, failure 0

*/
/*============================================================================*/
uint8_t board_conf(s_ns_t* ps_nStack);

#endif /* BOARD_CONF_H_ */
/** @} */
/** @} */
/** @} */
/** @} */
