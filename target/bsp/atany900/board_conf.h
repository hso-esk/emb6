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
/* Enable RF SPI */
#ifndef HAL_SUPPORT_RFSPI
#define HAL_SUPPORT_RFSPI                   TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */

/* add platform-specific SPIx configurations */
/** RF SPI CLK data port */
#define ATM1281_RFSPI_CLK_PORT              PORTB
/** RF SPI CLK data direction */
#define ATM1281_RFSPI_CLK_DDR               DDRB
/** RF SPI CLK pin */
#define ATM1281_RFSPI_CLK_PIN               0x01

/** RF SPI TX data port */
#define ATM1281_RFSPI_TX_PORT               PORTB
/** RF SPI TX data direction */
#define ATM1281_RFSPI_TX_DDR                DDRB
/** RF SPI TX pin */
#define ATM1281_RFSPI_TX_PIN                0x02

/** RF SPI RX data port */
#define ATM1281_RFSPI_RX_PORT               PORTB
/** RF SPI RX data direction */
#define ATM1281_RFSPI_RX_DDR                DDRB
/** RF SPI RX pin */
#define ATM1281_RFSPI_RX_PIN                0x03

/** RF SPI CS data port */
#define ATM1281_RFSPI_CS_PORT               PORTB
/** RF SPI CS data direction */
#define ATM1281_RFSPI_CS_DDR                DDRB
/** RF SPI CS pin */
#define ATM1281_RFSPI_CS_PIN                0x00

/* add platform-specific RF_GPIOx configuration */
/** Enable RF control 0 */
#ifndef HAL_SUPPORT_RFCTRL0
#define HAL_SUPPORT_RFCTRL0                 TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */
/** RF Reset data port */
#define ATM1281_RESET_PORT                  PORTA
/** RF Reset data direction */
#define ATM1281_RESET_DDR                   DDRA
/** RF Reset pin */
#define ATM1281_RESET_PIN                   0x07

/** Enable RF control 1 */
#ifndef HAL_SUPPORT_RFCTRL1
#define HAL_SUPPORT_RFCTRL1                 TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */
/** RF Sleep data port */
#define ATM1281_SLP_TR_PORT                 PORTB
/** RF Sleep data direction */
#define ATM1281_SLP_TR_DDR                  DDRB
/** RF Sleep pin */
#define ATM1281_SLP_TR_PIN                  0x04

/** Enable RF control 2 */
#ifndef HAL_SUPPORT_RFCTRL2
#define HAL_SUPPORT_RFCTRL2                 TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */
/** RF IRQ data port */
#define ATM1281_RFIRQ_PORT                  PORTE
/** RF IRQ data direction */
#define ATM1281_RFIRQ_DDR                   DDRE
/** RF IRQ pin */
#define ATM1281_RFIRQ_PIN                   0x05
/** RF IRQ num */
#define ATM1281_RFIRQ_IRQNUM                INT5

/** Different UART baudrates available */
#define ATM1281_USART_BAUD_2400             207
#define ATM1281_USART_BAUD_4800             103
#define ATM1281_USART_BAUD_9600             51
#define ATM1281_USART_BAUD_14400            34
#define ATM1281_USART_BAUD_19200            25
#define ATM1281_USART_BAUD_28800            16
#define ATM1281_USART_BAUD_38400            12
#define ATM1281_USART_BAUD_57600            8
#define ATM1281_USART_BAUD_76800            6
#define ATM1281_USART_BAUD_115200           3
#define ATM1281_USART_BAUD_230400           1
#define ATM1281_USART_BAUD_250000           1
#define ATM1281_USART_BAUD_500000           0

/* add platform-specific SLIPUART configuration */
#if defined (HAL_SUPPORT_SLIPUART)
#define ATM1281_UART_SLIP_INST              1
#define ATM1281_UART_SLIP_BAUD              ATM1281_USART_BAUD_38400
#endif /* #if defined (HAL_SUPPORT_SLIP_UART) */

/* add debugging channel configuration */
#define ATM1281_UART_DEBUG_INST             1
#define ATM1281_UART_DEBUG_BAUD             ATM1281_USART_BAUD_38400

/* add platform-specific LEDs configuration */
/** Number of supported LEDs */
#ifndef HAL_SUPPORT_LEDNUM
#define HAL_SUPPORT_LEDNUM                  3
#endif /* #ifndef HAL_SUPPORT_LEDNUM */

/** Enable LED0 */
#ifndef HAL_SUPPORT_LED0
#define HAL_SUPPORT_LED0                    TRUE
#endif /* #ifndef HAL_SUPPORT_LED0 */
/** RF LED RED data port */
#define ATM1281_LED0_PORT                   PORTB
/** RF LED RED data direction */
#define ATM1281_LED0_DDR                    DDRB
/** RF LED RED pin */
#define ATM1281_LED0_PIN                    0x05

/** Enable LED1 */
#ifndef HAL_SUPPORT_LED1
#define HAL_SUPPORT_LED1                    TRUE
#endif /* #ifndef HAL_SUPPORT_LED2 */
/** RF LED YELLOW data port */
#define ATM1281_LED1_PORT                   PORTB
/** RF LED YELLOW data direction */
#define ATM1281_LED1_DDR                    DDRB
/** RF LED YELLOW pin */
#define ATM1281_LED1_PIN                    0x06

/** Enable LED2 */
#ifndef HAL_SUPPORT_LED2
#define HAL_SUPPORT_LED2                    TRUE
#endif /* #ifndef HAL_SUPPORT_LED2 */
/** RF LED GREEN data port */
#define ATM1281_LED2_PORT                   PORTB
/** RF LED GREEN data direction */
#define ATM1281_LED2_DDR                    DDRB
/** RF LED GREEN pin */
#define ATM1281_LED2_PIN                    0x07


/*
 * --- Stack Macro Definitions ---------------------------------------------- *
 */

/** Default modulation scheme */
#if !defined(MODULATION)
#define MODULATION                          MODULATION_BPSK20
#endif /* #if !defined(MODULATION) */

/** transceiver supports standard-specific checksum algorithm */
#define NETSTK_CFG_RF_CRC_EN                TRUE


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

