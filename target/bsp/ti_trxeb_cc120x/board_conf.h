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

/** System tick timer */
#define MSP430_SYSTICK_TMR                  E_TMR_0

/** Enable RF SPI */
#ifndef HAL_SUPPORT_RFSPI
#define HAL_SUPPORT_RFSPI                   TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */

/* add platform-specific SPIx configurations */
/** Port of RF-SPI clock pin */
#define MSP430_IO_PORT_SPI_CLK              E_IO_PORT_P3
/** Pin of RF-SPI clock */
#define MSP430_IO_PIN_SPI_CLK               3
/** Mask of RF-SPI clock pin */
#define MSP430_IO_MASK_SPI_CLK              (1 << MSP430_IO_PIN_SPI_CLK)

/** Port of RF-SPI MOSI pin */
#define MSP430_IO_PORT_SPI_MOSI             E_IO_PORT_P3
/** Pin of RF-SPI clock */
#define MSP430_IO_PIN_SPI_MOSI              1
/** Mask of RF-SPI clock pin */
#define MSP430_IO_MASK_SPI_MOSI             (1 << MSP430_IO_PIN_SPI_MOSI)

/** Port of RF-SPI MISO pin */
#define MSP430_IO_PORT_SPI_MISO             E_IO_PORT_P3
/** Pin of RF-SPI clock */
#define MSP430_IO_PIN_SPI_MISO              2
/** Mask of RF-SPI clock pin */
#define MSP430_IO_MASK_SPI_MISO             (1 << MSP430_IO_PIN_SPI_MISO)

/** Port of RF-SPI CS pin */
#define MSP430_IO_PORT_SPI_CS               E_IO_PORT_P3
/** Pin of RF-SPI clock */
#define MSP430_IO_PIN_SPI_CS                0
/** Mask of RF-SPI clock pin */
#define MSP430_IO_MASK_SPI_CS               (1 << MSP430_IO_PIN_SPI_CS)

/* add platform-specific RF_GPIOx configuration */
/** Enable RF control 0 */
#ifndef HAL_SUPPORT_RFCTRL0
#define HAL_SUPPORT_RFCTRL0                 TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */
/** Port of RF control 0 pin */
#define MSP430_IO_PORT_RF_CTRL0             E_IO_PORT_P1
/** Pin of RF control 0 */
#define MSP430_IO_PIN_RF_CTRL0              7
/** Mask of RF control 0 pin */
#define MSP430_IO_MASK_RF_CTRL0             (1 << MSP430_IO_PIN_RF_CTRL0)

/** Enable RF control 1 */
#ifndef HAL_SUPPORT_RFCTRL1
#define HAL_SUPPORT_RFCTRL1                 TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */
/** Port of RF control 1 pin */
#define MSP430_IO_PORT_RF_CTRL1             E_IO_PORT_P1
/** Pin of RF control 1 */
#define MSP430_IO_PIN_RF_CTRL1              3
/** Mask of RF control 1 pin */
#define MSP430_IO_MASK_RF_CTRL1             (1 << MSP430_IO_PIN_RF_CTRL1)

/** Enable RF control 2 */
#ifndef HAL_SUPPORT_RFCTRL2
#define HAL_SUPPORT_RFCTRL2                 TRUE
#endif /* #ifndef HAL_SUPPORT_RFSPI */
/** Port of RF control 1 pin */
#define MSP430_IO_PORT_RF_CTRL2             E_IO_PORT_P1
/** Pin of RF control 1 */
#define MSP430_IO_PIN_RF_CTRL2              2
/** Mask of RF control 1 pin */
#define MSP430_IO_MASK_RF_CTRL2             (1 << MSP430_IO_PIN_RF_CTRL2)

/* add platform-specific SLIPUART configuration */
#if defined(HAL_SUPPORT_SLIPUART)
/** USART used for SLIP interface */
#define MSP430_SLIP_UART                    E_UART_SEL_UART1
/** Baudrate of SLIP UART */
#define MSP430_SLIP_UART_BAUD               115200
#endif /* #if defined(HAL_SUPPORT_SLIPUART) */

/* add platform-specific debug configuration */
/** UART used for DEBUG interface */
#define MSP430_DEBUG_UART                     E_UART_SEL_UART1
/** Baudrate of DEBUG UART */
#define MSP430_DEBUG_UART_BAUD                115200

#define MSP430_DEBUG_UART_RX_PORT             E_IO_PORT_P5
#define MSP430_DEBUG_UART_RX_PIN              7
#define MSP430_DEBUG_UART_RX_MSK              (1 << MSP430_DEBUG_UART_RX_PIN)

#define MSP430_DEBUG_UART_TX_PORT             E_IO_PORT_P5
#define MSP430_DEBUG_UART_TX_PIN              6
#define MSP430_DEBUG_UART_TX_MSK              (1 << MSP430_DEBUG_UART_TX_PIN)

/* add platform-specific LEDs configuration */
/** Number of supported LEDs */
#ifndef HAL_SUPPORT_LEDNUM
#define HAL_SUPPORT_LEDNUM                  ( 4 )
#endif /* #ifndef HAL_SUPPORT_LEDNUM */

#define HAL_LED_INVERTED                    TRUE

/** Enable LED0 */
#ifndef HAL_SUPPORT_LED0
#define HAL_SUPPORT_LED0                    TRUE
#endif /* #ifndef HAL_SUPPORT_LED0 */
/** LED0 port */
#define MSP430_IO_PORT_LED0                 E_IO_PORT_P4
/** LED0 pin index */
#define MSP430_IO_PIN_LED0                  0
/** LED0 pin mask */
#define MSP430_IO_MASK_LED0                 (1 << MSP430_IO_PIN_LED0)

/** Enable LED1 */
#ifndef HAL_SUPPORT_LED1
#define HAL_SUPPORT_LED1                    TRUE
#endif /* #ifndef HAL_SUPPORT_LED1 */
/** LED1 port */
#define MSP430_IO_PORT_LED1                 E_IO_PORT_P4
/** LED1 pin index */
#define MSP430_IO_PIN_LED1                  1
/** LED1 pin mask */
#define MSP430_IO_MASK_LED1                 (1 << MSP430_IO_PIN_LED1)

/** Enable LED2 */
#ifndef HAL_SUPPORT_LED2
#define HAL_SUPPORT_LED2                    TRUE
#endif /* #ifndef HAL_SUPPORT_LED2 */
/** LED2 port */
#define MSP430_IO_PORT_LED2                 E_IO_PORT_P4
/** LED2 pin index */
#define MSP430_IO_PIN_LED2                  2
/** LED2 pin mask */
#define MSP430_IO_MASK_LED2                 (1 << MSP430_IO_PIN_LED2)

/** Enable LED3 */
#ifndef HAL_SUPPORT_LED3
#define HAL_SUPPORT_LED3                    TRUE
#endif /* #ifndef HAL_SUPPORT_LED3 */
/** LED3 port */
#define MSP430_IO_PORT_LED3                 E_IO_PORT_P4
/** LED3 pin index */
#define MSP430_IO_PIN_LED3                  3
/** LED3 pin mask */
#define MSP430_IO_MASK_LED3                 (1 << MSP430_IO_PIN_LED3)


/*
 * --- Stack Macro Definitions ---------------------------------------------- *
 */
/** additional delay between consecutive iteration of emb6 process */
#define EMB6_PROC_DELAY                     ( 0 )

/** transceiver modulation scheme */
#if !defined(MODULATION)
#define MODULATION                          MODULATION_2FSK50
#endif /* #if !defined(MODULATION) */

/** transceiver supports auto-acknowledgment on software */
#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == FALSE)
#define NETSTK_SUPPORT_SW_RF_AUTOACK        TRUE
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == FALSE) */


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

