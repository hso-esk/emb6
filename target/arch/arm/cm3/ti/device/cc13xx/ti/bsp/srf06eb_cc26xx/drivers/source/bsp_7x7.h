//*****************************************************************************
//! @file       bsp_7x7.h
//! @brief      Board support package header file for CC26xx QFN48 7x7 EM and
//!             CC13xx QFN48 7x7 EM on SmartRF06EB.
//!
//! Revised     $Date: 2014-10-24 14:12:17 +0200 (fr, 24 okt 2014) $
//! Revision    $Revision: 14220 $
//
//  Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/
#ifndef __BSP_7X7_H__
#define __BSP_7X7_H__

#if !(defined(MODULE_CC13XX_7X7) || defined(MODULE_CC13XX_TEST_7X7) ||        \
      defined(MODULE_CC26XX_7X7) || defined(MODULE_CC26XX_TEST_7X7))
#error bsp_7x7.h: Unknown module definition!
#endif

/******************************************************************************
* If building with a C++ compiler, make all of the definitions in this header
* have a C binding.
******************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif


/******************************************************************************
* INCLUDES
*/


/******************************************************************************
* DEFINES
*/

//
// SPI defines (Common for LCD, SD reader and accelerometer)
//
#define BSP_SPI_BASE            SSI0_BASE
#define BSP_IOID_SPI_SCK        IOID_10
#define BSP_SPI_SCK             (1 << BSP_IOID_SPI_SCK)
#define BSP_IOID_SPI_MOSI       IOID_9
#define BSP_SPI_MOSI            (1 << BSP_IOID_SPI_MOSI)
#define BSP_IOID_SPI_MISO       IOID_8
#define BSP_SPI_MISO            (1 << BSP_IOID_SPI_MISO)


//
// Board LED defines
//
#ifdef FPGA_INCLUDED
#define BSP_IOID_LED_1          IOID_30
#define BSP_IOID_LED_2          IOID_28
#elif (defined(MODULE_CC13XX_TEST_7X7))
#define BSP_IOID_LED_1          IOID_26
#define BSP_IOID_LED_2          IOID_28
#else
// MODULE_CC26XX & MODULE_CC13XX
#define BSP_IOID_LED_1          IOID_25
#define BSP_IOID_LED_2          IOID_27
#endif
#define BSP_IOID_LED_3          IOID_7
#define BSP_IOID_LED_4          IOID_6
#define BSP_LED_1               (1 << BSP_IOID_LED_1)
#define BSP_LED_2               (1 << BSP_IOID_LED_2)
#define BSP_LED_3               (1 << BSP_IOID_LED_3)
#define BSP_LED_4               (1 << BSP_IOID_LED_4)
#define BSP_LED_ALL             (BSP_LED_1 | BSP_LED_2 | BSP_LED_3 |          \
                                 BSP_LED_4)     //!< Bitmask of all LEDs


//
// Board key defines
//
#if (defined(MODULE_CC13XX_TEST_7X7))
#define BSP_IOID_KEY_LEFT       IOID_15
#define BSP_IOID_KEY_RIGHT      IOID_19
#define BSP_IOID_KEY_UP         IOID_20
#else
// MODULE_CC26XX & MODULE_CC13XX
#define BSP_IOID_KEY_LEFT       IOID_15
#define BSP_IOID_KEY_RIGHT      IOID_18
#define BSP_IOID_KEY_UP         IOID_19
#endif
#define BSP_IOID_KEY_DOWN       IOID_12
#define BSP_IOID_KEY_SELECT     IOID_11
#define BSP_KEY_LEFT            (1 << BSP_IOID_KEY_LEFT)
#define BSP_KEY_RIGHT           (1 << BSP_IOID_KEY_RIGHT)
#define BSP_KEY_UP              (1 << BSP_IOID_KEY_UP)
#define BSP_KEY_DOWN            (1 << BSP_IOID_KEY_DOWN)
#define BSP_KEY_SELECT          (1 << BSP_IOID_KEY_SELECT)
#define BSP_KEY_DIR_ALL         (BSP_KEY_UP | BSP_KEY_DOWN | BSP_KEY_LEFT |   \
                                 BSP_KEY_RIGHT) //!< Bitmask of all dir. keys
#define BSP_KEY_ALL             (BSP_KEY_DIR_ALL | BSP_KEY_SELECT) //!< Bitmask of all keys


#ifdef BOARD_SMARTRF06EB
//
// 3.3-V domain defines
//
#define BSP_IOID_3V3_EN         IOID_13
#define BSP_3V3_EN              (1 << BSP_IOID_3V3_EN)


//
// Board LCD defines
//
#define BSP_IOID_LCD_MODE       IOID_4
#define BSP_LCD_MODE            (1 << BSP_IOID_LCD_MODE)
#define BSP_IOID_LCD_RST        IOID_5
#define BSP_LCD_RST             (1 << BSP_IOID_LCD_RST)
#define BSP_IOID_LCD_CS         IOID_14
#define BSP_LCD_CS              (1 << BSP_IOID_LCD_CS)
#define BSP_IOID_LCD_SCK        BSP_IOID_SPI_SCK
#define BSP_LCD_SCK             BSP_SPI_SCK
#define BSP_IOID_LCD_MOSI       BSP_IOID_SPI_MOSI
#define BSP_LCD_MOSI            BSP_SPI_MOSI


//
// Board accelerometer defines
//
#if (defined(MODULE_CC26XX_7X7) || defined(MODULE_CC13XX_7X7))
#define BSP_IOID_ACC_PWR        IOID_20
#else
// MODULE_CC26XX_TEST_7X7 || MODULE_CC13XX_TEST...
#define BSP_IOID_ACC_PWR        IOID_UNUSED
#endif // ifdef MODULE_CC26XX_7X7
#define BSP_ACC_PWR             (1 << BSP_IOID_ACC_PWR)

#if (defined(MODULE_CC13XX_TEST_7X7))
#define BSP_IOID_ACC_INT        IOID_UNUSED
#define BSP_ACC_INT             0
#define BSP_IOID_ACC_INT1       IOID_UNUSED
#define BSP_ACC_INT1            0
#else
// MODULE_CC26XX & MODULE_CC13XX
#define BSP_IOID_ACC_INT        IOID_28
#define BSP_ACC_INT             (1 << BSP_IOID_ACC_INT)
#define BSP_IOID_ACC_INT1       IOID_28         // ACC_INT1 == ACC_INT
#define BSP_ACC_INT1            (1 << BSP_IOID_ACC_INT1)
#endif // ifdef MODULE_CC13XX_7X7 ...

#define BSP_IOID_ACC_INT2       IOID_29
#define BSP_ACC_INT2            (1 << BSP_IOID_ACC_INT2)
#define BSP_IOID_ACC_CS         IOID_24
#define BSP_ACC_CS              (1 << BSP_IOID_ACC_CS)
#define BSP_IOID_ACC_SCK        BSP_IOID_SPI_SCK
#define BSP_ACC_SCK             BSP_SPI_SCK
#define BSP_IOID_ACC_MOSI       BSP_IOID_SPI_MOSI
#define BSP_ACC_MOSI            BSP_SPI_MOSI
#define BSP_IOID_ACC_MISO       BSP_IOID_SPI_MISO
#define BSP_ACC_MISO            BSP_SPI_MISO


//
// SD reader defines
//
#define BSP_IOID_SDCARD_CS      IOID_30
#define BSP_SDCARD_CS           (1 << BSP_IOID_SDCARD_CS)
#define BSP_IOID_SDCARD_SCK     BSP_IOID_SPI_SCK
#define BSP_SDCARD_SCK          BSP_SPI_SCK
#define BSP_IOID_SDCARD_MOSI    BSP_IOID_SPI_MOSI
#define BSP_SDCARD_MOSI         BSP_SPI_MOSI
#define BSP_IOID_SDCARD_MISO    BSP_IOID_SPI_MISO
#define BSP_SDCARD_MISO         BSP_SPI_MISO


//
// Board ambient lightsensor defines
//
#if (defined(MODULE_CC13XX_TEST_7X7))
#define BSP_IOID_ALS_PWR        IOID_UNUSED
#define BSP_ALS_PWR             0
#else
// MODULE_CC26XX & MODULE_CC13XX
#define BSP_IOID_ALS_PWR        IOID_26
#define BSP_ALS_PWR             (1 << BSP_IOID_ALS_PWR)
#endif // ifdef MODULE_CC13XX_...
#define BSP_IOID_ALS_OUT        IOID_23
#define BSP_ALS_OUT             (1 << BSP_IOID_ALS_OUT)


//
// UART backchannel defines
//
#if (defined(MODULE_CC13XX_TEST_7X7))
#define BSP_IOID_UART_RXD       IOID_UNUSED
#define BSP_UART_RXD            0
#define BSP_IOID_UART_TXD       IOID_UNUSED
#define BSP_UART_TXD            0
#define BSP_IOID_UART_CTS       IOID_UNUSED
#define BSP_UART_CTS            0
#define BSP_IOID_UART_RTS       IOID_UNUSED
#define BSP_UART_RTS            0
#else
// MODULE_CC26XX & MODULE_CC13XX
#define BSP_IOID_UART_RXD       IOID_2
#define BSP_UART_RXD            (1 << BSP_IOID_UART_RXD)
#define BSP_IOID_UART_TXD       IOID_3
#define BSP_UART_TXD            (1 << BSP_IOID_UART_TXD)
#define BSP_IOID_UART_CTS       IOID_0
#define BSP_UART_CTS            (1 << BSP_IOID_UART_CTS)
#if (defined(MODULE_CC26XX_7X7) || defined(MODULE_CC13XX_7X7))
#define BSP_IOID_UART_RTS       IOID_21
#define BSP_UART_RTS            (1 << BSP_IOID_UART_RTS)
#else
// MODULE_CC26XX_TEST_7X7 & ODULE_CC13XX_TEST_7X7
#define BSP_IOID_UART_RTS       IOID_UNUSED     // Not connected to CC26xx 7x7 test EM
#define BSP_UART_RTS            0
#endif // ifdef MODULE_CC26XX_7X7
#endif // ifdef MODULE_CC13XX_...

//#define BSP_UART_INT_BM         0xF0            //!< Interrupts handled by bsp
#endif // ifdef BOARD_SMARTRF06EB


/******************************************************************************
* Mark the end of the C bindings section for C++ compilers.
******************************************************************************/
#ifdef __cplusplus
}
#endif
#endif /* #ifndef __BSP_7X7_H__ */

