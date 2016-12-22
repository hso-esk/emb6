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
#define HAL_SUPPORT_RFSPI                     TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */

/* add platform-specific SPIx configurations */
#define SAMD20_RFSPI_SERCOM                   SERCOM0
#define SAMD20_RFSPI_SERCOM_MUX_SETTING       SPI_SIGNAL_MUX_SETTING_E
#define SAMD20_RFSPI_SERCOM_PMUX0             PINMUX_PA04D_SERCOM0_PAD0
#define SAMD20_RFSPI_SERCOM_PMUX1             PINMUX_UNUSED
#define SAMD20_RFSPI_SERCOM_PMUX2             PINMUX_PA06D_SERCOM0_PAD2
#define SAMD20_RFSPI_SERCOM_PMUX3             PINMUX_PA07D_SERCOM0_PAD3
/** RF SPI clock pin */
#define SAMD20_IO_PIN_SPI_CLK                 PIN_PA07
/** RF SPI MOSI pin */
#define SAMD20_IO_PIN_SPI_TX                  PIN_PA06
/** RF SPI MISO pin */
#define SAMD20_IO_PIN_SPI_RX                  PIN_PA04
/** RF SPI chip select pin */
#define SAMD20_IO_PIN_SPI_CS                  PIN_PA05

/* add platform-specific RF_GPIOx configuration */
/** Enable RF control 0 */
#ifndef HAL_SUPPORT_RFCTRL0
#define HAL_SUPPORT_RFCTRL0                   TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */
/** RF control 0 pin (reset) */
#define SAMD20_IO_PIN_RF_CTRL0                PIN_PB02
#define SAMD20_IO_PIN_RF_RST                  SAMD20_IO_PIN_RF_CTRL0

/** Enable RF control 1 */
#ifndef HAL_SUPPORT_RFCTRL1
#define HAL_SUPPORT_RFCTRL1                   TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */
/** RF control 1 pin (sleep) */
#define SAMD20_IO_PIN_RF_CTRL1                PIN_PB05
#define SAMD20_IO_PIN_RF_SLP                  SAMD20_IO_PIN_RF_CTRL1

/** Enable RF control 2 */
#ifndef HAL_SUPPORT_RFCTRL2
#define HAL_SUPPORT_RFCTRL2                   TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */
/** RF control 2 pin (interrupt) */
#define SAMD20_IO_PIN_RF_CTRL2                PIN_PB04
#define SAMD20_IO_PIN_RF_IRQ                  SAMD20_IO_PIN_RF_CTRL2
#define SAMD20_IO_PIN_RF_IRQ_PINMUX           PINMUX_PB04A_EIC_EXTINT4
#define SAMD20_IO_PIN_RF_IRQ_CHANNEL          4

/* add platform-specific SLIPUART configuration */
#if defined(HAL_SUPPORT_SLIPUART)
#define SAMD20_SLIP_UART_SERCOM               SERCOM3
#define SAMD20_SLIP_UART_BAUDRATE             38400
#define SAMD20_SLIP_UART_SERCOM_MUX_SETTING   USART_RX_3_TX_2_XCK_3
#define SAMD20_SLIP_UART_SERCOM_PMUX0         PINMUX_UNUSED
#define SAMD20_SLIP_UART_SERCOM_PMUX1         PINMUX_UNUSED
#define SAMD20_SLIP_UART_SERCOM_PMUX2         PINMUX_PA24C_SERCOM3_PAD2
#define SAMD20_SLIP_UART_SERCOM_PMUX3         PINMUX_PA25C_SERCOM3_PAD3
#define SAMD20_SLIP_UART_PIN_TX               PIN_PA24
#define SAMD20_SLIP_UART_PIN_RX               PIN_PA25
#endif /* #if defined(HAL_SUPPORT_SLIPUART) */

/* add debugging channel configuration */
#define SAMD20_DEBUG_UART_SERCOM              SERCOM3
#define SAMD20_DEBUG_UART_BAUDRATE            38400
#define SAMD20_DEBUG_UART_SERCOM_MUX_SETTING  USART_RX_3_TX_2_XCK_3
#define SAMD20_DEBUG_UART_SERCOM_PMUX0        PINMUX_UNUSED
#define SAMD20_DEBUG_UART_SERCOM_PMUX1        PINMUX_UNUSED
#define SAMD20_DEBUG_UART_SERCOM_PMUX2        PINMUX_PA24C_SERCOM3_PAD2
#define SAMD20_DEBUG_UART_SERCOM_PMUX3        PINMUX_PA25C_SERCOM3_PAD3
#define SAMD20_DEBUG_UART_PIN_TX              PIN_PA24
#define SAMD20_DEBUG_UART_PIN_RX              PIN_PA25

/* add platform-specific LEDs configuration */
/** Number of supported LEDs */
#ifndef HAL_SUPPORT_LEDNUM
#define HAL_SUPPORT_LEDNUM                    ( 1 )
#endif /* #ifndef HAL_SUPPORT_LEDNUM */

/** Enable LED0 */
#ifndef HAL_SUPPORT_LED0
#define HAL_SUPPORT_LED0                      TRUE
#endif /* #ifndef HAL_SUPPORT_LED0 */
/** LED0 (yellow) port */
#define SAMD20_IO_PIN_LED0                    PIN_PA14


/*
 * --- Stack Macro Definitions ---------------------------------------------- *
 */
/** additional delay between consecutive iteration of emb6 process */
#define EMB6_PROC_DELAY                       ( 1 )

/** Default modulation scheme */
#if !defined(MODULATION)
#define MODULATION                            MODULATION_BPSK20
#endif /* #if !defined(MODULATION) */

/** transceiver supports standard-specific checksum algorithm */
#define NETSTK_SUPPORT_HW_CRC                 TRUE


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

