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
 *      \addtogroup efm32stk3600_cc112x Board EFM32 Leopard Gecko Starter Kit specific configuration
 *   @{
 */
/*! \file   efm32stk3600_cc112x/board_conf.h

    \author Artem Yushev, 

    \brief  Board Configuration for EFM32 Leopard Gecko Starter Kit

    \version 0.0.1
*/

#ifndef BOARD_CONF_H_
#define BOARD_CONF_H_



#include "emb6.h"

/*==============================================================================
                                     MACROS
==============================================================================*/
/*
 * RF SPI pins
 */
#define EFM32_USART                         SPIDRV_MASTER_USART1
#define EFM32_USART_LOC                     _USART_ROUTE_LOCATION_LOC1

#define EFM32_IO_PORT_USART_CLK             gpioPortD
#define EFM32_IO_PIN_USART_CLK              2

#define EFM32_IO_PORT_USART_TX              gpioPortD
#define EFM32_IO_PIN_USART_TX               0

#define EFM32_IO_PORT_USART_RX              gpioPortD
#define EFM32_IO_PIN_USART_RX               1

#define EFM32_IO_PORT_USART_CS              gpioPortD
#define EFM32_IO_PIN_USART_CS               3



#define EFM32_IO_PORT_RF_RST                gpioPortD
#define EFM32_IO_PIN_RF_RST                 4

#define EFM32_IO_PORT_RF_IRQ                gpioPortD
#define EFM32_IO_PIN_RF_IRQ                 5

#define EFM32_IO_PORT_RF_SLP                gpioPortD
#define EFM32_IO_PIN_RF_SLP                 6


/*
 * TI transceiver CC112x/CC120x pins
 */
#define EFM32_IO_PORT_RF_IRQ_0              gpioPortC
#define EFM32_IO_PIN_RF_IRQ_0               3

#define EFM32_IO_PORT_RF_IRQ_2              gpioPortC
#define EFM32_IO_PIN_RF_IRQ_2               4

#define EFM32_IO_PORT_RF_IRQ_3              gpioPortC
#define EFM32_IO_PIN_RF_IRQ_3               5


/*
 * UART
 */
#define EFM32_UART                          UART0
#define EFM32_UART_LOC                      UART_ROUTE_LOCATION_LOC1

#define EFM32_UART_PORT_USART_TX            gpioPortE
#define EFM32_UART_PIN_USART_TX             0

#define EFM32_UART_PORT_USART_RX            gpioPortE
#define EFM32_UART_PIN_USART_RX             1


/*
 * LED Ports
 */

#define EFM32_IO_PORT_LED0                  gpioPortE
#define EFM32_IO_PIN_LED0                   2

#define EFM32_IO_PORT_LED1                  gpioPortE
#define EFM32_IO_PIN_LED1                   3

#define EFM32_LED_ACTIVE_HIGH               TRUE

/*
 * DELAY
 */
#define EMB6_PROC_DELAY                     0

#define CC112X_PART_VERSION                 0x21u

#ifndef MODULATION
#define MODULATION                          MODULATION_2FSK50
#endif

/* enable auto-acknowledgment of radio driver */
#define NETSTK_CFG_RF_SW_AUTOACK_EN         TRUE

/* radio transceiver does not support standard-specified checksum */
#define NETSTK_CFG_RF_CRC_EN                FALSE

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
