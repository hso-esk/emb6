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

/*
 * --- Module Description ---------------------------------------------------*
 */
/**
 *  \file       board_conf.h
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Definition of the Board Configuration.
 *
 *              The Board Configuration configures the underlying MCU and
 *              transceiver.
 */
#ifndef __BOARD_CONF_H__
#define __BOARD_CONF_H__

/*
 * --- Includes -------------------------------------------------------------*
 */


/*
 * --- Macro Definitions --------------------------------------------------- *
 */
/** Enable RF SPI */
#ifndef HAL_SUPPORT_RFSPI
#define HAL_SUPPORT_RFSPI                   TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */

/** USART used for RF SPI */
#define EFM32_RFSPI_USART                   SPIDRV_MASTER_USART1
/** Routing for the RF SPI pins */
#define EFM32_RFSPI_USART_LOC               _USART_ROUTE_LOCATION_LOC1
/** Port of the RF SPI clock pin */
#define EFM32_IO_PORT_USART_CLK             gpioPortD
/** Pin index of the RF SPI clock pin */
#define EFM32_IO_PIN_USART_CLK              2
/** Port of the RF SPI tx pin */
#define EFM32_IO_PORT_USART_TX              gpioPortD
/** Pin index of the RF SPI tx pin */
#define EFM32_IO_PIN_USART_TX               0
/** Port of the RF SPI rx pin */
#define EFM32_IO_PORT_USART_RX              gpioPortD
/** Pin index of the RF SPI rx pin */
#define EFM32_IO_PIN_USART_RX               1
/** Port of the RF SPI cs pin */
#define EFM32_IO_PORT_USART_CS              gpioPortD
/** Pin index of the RF SPI cs pin */
#define EFM32_IO_PIN_USART_CS               3


/** Enable RF control 0 */
#ifndef HAL_SUPPORT_RFCTRL0
#define HAL_SUPPORT_RFCTRL0                 TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */
/** Port of the RF control pin0 */
#define EFM32_IO_PORT_RF_CTRL_0             gpioPortC
/** Pin index of the RF control pin0 */
#define EFM32_IO_PIN_RF_CTRL_0              3

/** Enable RF control 1 */
#ifndef HAL_SUPPORT_RFCTRL1
#define HAL_SUPPORT_RFCTRL1                 TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */
/** Port of the RF control pin1 */
#define EFM32_IO_PORT_RF_CTRL_1             gpioPortC
/** Pin index of the RF control pin1 */
#define EFM32_IO_PIN_RF_CTRL_1              4

/** Enable RF control 2 */
#ifndef HAL_SUPPORT_RFCTRL2
#define HAL_SUPPORT_RFCTRL2                 TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */
/** Port of the RF control pin2 */
#define EFM32_IO_PORT_RF_CTRL_2             gpioPortC
/** Pin index of the RF control pin2 */
#define EFM32_IO_PIN_RF_CTRL_2              5


#if defined(HAL_SUPPORT_SLIPUART)
/** USART used for SLIP interface */
#define EFM32_SLIP_UART                     USART0
/** Routing for the SLIP UART pins */
#define EFM32_SLIP_UART_LOC                 USART_ROUTE_LOCATION_LOC1
/** Port of the SLIP UART TX pin */
#define EFM32_SLIP_UART_PORT_USART_TX       gpioPortE
/** Pin index of the SLIP UART TX pin */
#define EFM32_SLIP_UART_PIN_USART_TX        0
/** Port of the SLIP UART RX pin */
#define EFM32_SLIP_UART_PORT_USART_RX       gpioPortE
/** Pin index of the SLIP UART RX pin */
#define EFM32_SLIP_UART_PIN_USART_RX        1
/** Baudrate of SLIP UART */
#define EFM32_SLIP_UART_BAUD                115200
/** RX interrupt handler for SLIP UART */
#define EFM32_SLIP_UART_RXIRQHNDL           USART0_RX_IRQHandler
#endif /* #if defined(HAL_SUPPORT_SLIPUART) */


/** USART used for DEBUG interface */
#define EFM32_DEBUG_UART                    USART0
/** Routing for the DEBUG UART pins */
#define EFM32_DEBUG_UART_LOC                USART_ROUTE_LOCATION_LOC1
/** Port of the DEBUG UART TX pin */
#define EFM32_DEBUG_UART_PORT_USART_TX      gpioPortE
/** Pin index of the DEBUG UART TX pin */
#define EFM32_DEBUG_UART_PIN_USART_TX       0
/** Port of the DEBUG UART RX pin */
#define EFM32_DEBUG_UART_PORT_USART_RX      gpioPortE
/** Pin index of the DEBUG UART RX pin */
#define EFM32_DEBUG_UART_PIN_USART_RX       1
/** Baudrate of DEBUG UART */
#define EFM32_DEBUG_UART_BAUD               115200


/** Number of supported LEDs */
#ifndef HAL_SUPPORT_LEDNUM
#define HAL_SUPPORT_LEDNUM                  ( 2 )
#endif /* #ifndef HAL_SUPPORT_LEDNUM */

/** Enable LED0 */
#ifndef HAL_SUPPORT_LED0
#define HAL_SUPPORT_LED0                    TRUE
#endif /* #ifndef HAL_SUPPORT_LED0 */
/** LED0 port */
#define EFM32_IO_PORT_LED0                  gpioPortE
/** LED0 pin index */
#define EFM32_IO_PIN_LED0                   2

/** Enable LED1 */
#ifndef HAL_SUPPORT_LED1
#define HAL_SUPPORT_LED1                    TRUE
#endif /* #ifndef HAL_SUPPORT_LED1 */
/** LED1 port */
#define EFM32_IO_PORT_LED1                  gpioPortE
/** LED1 pin index */
#define EFM32_IO_PIN_LED1                   3


/*
 * --- Stack Macro Definitions ---------------------------------------------- *
 */
/** additional delay between consecutive iteration of emb6 process */
#define EMB6_PROC_DELAY                     ( 0 )

/** Default modulation scheme */
#ifndef MODULATION
#define MODULATION                          MODULATION_2FSK50
#endif

/** Enable auto-acknowledgment of radio driver */
#define NETSTK_CFG_RF_SW_AUTOACK_EN         TRUE

/** Radio transceiver does not support standard-specified checksum */
#define NETSTK_CFG_RF_CRC_EN                FALSE


/*
 *  --- Global Functions Definition ------------------------------------------*
 */

/**
 * board_conf()
 *
 * \brief   Configure board.
 *
 *          This function is used to configure the specific board for usage
 *          with emb::6. Therefore e.g. the transceiver and the according
 *          layers will be selected.
 *
 * \param   p_ns  Pointer to global netstack struct.
 *
 * \return  0 on success or negative value on error.
 */
int8_t board_conf( s_ns_t* p_ns );

#endif /* __BOARD_CONF_H__ */

