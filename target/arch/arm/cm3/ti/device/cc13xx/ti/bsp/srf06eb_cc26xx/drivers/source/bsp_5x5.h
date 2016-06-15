//*****************************************************************************
//! @file       bsp_5x5.h
//! @brief      Board support package header file for CC26xx QFN32 5x5 EM
//!             on SmartRF06EB, with access to the following 06EB peripherals:
//!             LCD (SPI), 2-pin UART, 2-4 keys (2 if trace enabled, 3 if
//!             network processor, 4 if not network processor) and 2 LEDs.
//!
//! Revised     $Date: 2013-10-30 16:10:38 +0100 (on, 30 okt 2013) $
//! Revision    $Revision: 10999 $
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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
#ifndef __BSP_5X5_H__
#define __BSP_5X5_H__

#if !(defined(MODULE_CC26XX_5X5) || defined(MODULE_CC26XX_TEST_5X5))
#error bsp_5x5.h: Unknown module definition!
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
#define BSP_IOID_SPI_MOSI       IOID_11
#define BSP_SPI_MOSI            (1 << BSP_IOID_SPI_MOSI)
#define BSP_IOID_SPI_MISO       IOID_12
#define BSP_SPI_MISO            (1 << BSP_IOID_SPI_MISO)


//
// Board LED defines
//
#ifdef MODULE_CC26XX_5X5
#define BSP_IOID_LED_1          IOID_UNUSED     // Not connected to CC26xx 5x5 EM
#define BSP_LED_1               0
#define BSP_IOID_LED_2          IOID_UNUSED     // Not connected to CC26xx 5x5 EM
#define BSP_LED_2               0
#define BSP_LED_ALL             (BSP_LED_1 | BSP_LED_2 | BSP_LED_3 |          \
                                 BSP_LED_4)     //!< Bitmask of all LEDs
#else
// MODULE_CC26XX_TEST_5X5
#define BSP_IOID_LED_1          IOID_7
#define BSP_LED_1               (1 << BSP_IOID_LED_1)
#define BSP_IOID_LED_2          IOID_8
#define BSP_LED_2               (1 << BSP_IOID_LED_2)
#endif // ifdef MODULE_CC26XX_5X5
#define BSP_IOID_LED_3          IOID_2
#define BSP_LED_3               (1 << BSP_IOID_LED_3)
#define BSP_IOID_LED_4          IOID_3
#define BSP_LED_4               (1 << BSP_IOID_LED_4)
#define BSP_LED_ALL             (BSP_LED_1 | BSP_LED_2 | BSP_LED_3 |          \
                                 BSP_LED_4)     //!< Bitmask of all LEDs


//
// Board key defines
//
#ifdef MODULE_CC26XX_5X5
#define BSP_IOID_KEY_LEFT       IOID_5          // Pin shared, TDO (trace)
#define BSP_KEY_LEFT            (1 << BSP_IOID_KEY_LEFT)
#define BSP_IOID_KEY_RIGHT      IOID_13
#define BSP_KEY_RIGHT           (1 << BSP_IOID_KEY_RIGHT)
#define BSP_IOID_KEY_UP         IOID_6          // Pin shared, TDI
#define BSP_KEY_UP              (1 << BSP_IOID_KEY_UP)
#else
// MODULE_CC26XX_TEST_5X5
#define BSP_IOID_KEY_LEFT       IOID_UNUSED     // Not connected to CC26xx 5x5 test EM
#define BSP_KEY_LEFT            0
#define BSP_IOID_KEY_RIGHT      IOID_UNUSED     // Not connected to CC26xx 5x5 test EM
#define BSP_KEY_RIGHT           0
#define BSP_IOID_KEY_UP         IOID_UNUSED     // Not connected to CC26xx 5x5 test EM
#define BSP_KEY_UP              0
#endif // ifdef MODULE_CC26XX_5X5
#define BSP_IOID_KEY_DOWN       IOID_4
#define BSP_KEY_DOWN            (1 << BSP_IOID_KEY_DOWN)
#define BSP_IOID_KEY_SELECT     IOID_9
#define BSP_KEY_SELECT          (1 << BSP_IOID_KEY_SELECT)
#define BSP_KEY_DIR_ALL         (BSP_KEY_UP | BSP_KEY_DOWN | BSP_KEY_LEFT |   \
                                 BSP_KEY_RIGHT) //!< Bitmask of all dir. keys
#define BSP_KEY_ALL             (BSP_KEY_DIR_ALL | BSP_KEY_SELECT) //!< Bitmask of all keys


#ifdef BOARD_SMARTRF06EB
//
// 3.3-V domain defines
//
#ifdef MODULE_CC26XX_5X5
#define BSP_IOID_3V3_EN         IOID_14
#define BSP_3V3_EN              (1 << BSP_IOID_3V3_EN)
#else
// MODULE_CC26XX_TEST_5X5
#define BSP_IOID_3V3_EN         IOID_UNUSED     // Not connected to CC26xx 5x5 test EM
#define BSP_3V3_EN              0
#endif // ifdef MODULE_CC26XX_5X5


//
// Board LCD defines
//
#ifdef MODULE_CC26XX_5X5
#define BSP_IOID_LCD_MODE       IOID_7
#define BSP_LCD_MODE            (1 << BSP_IOID_LCD_MODE)
#define BSP_IOID_LCD_RST        IOID_UNUSED     // Not connected to CC26xx 5x5 EM
#define BSP_LCD_RST             0
#define BSP_IOID_LCD_CS         IOID_8
#define BSP_LCD_CS              (1 << BSP_IOID_LCD_CS)
#else
// MODULE_CC26XX_TEST_5X5
#define BSP_IOID_LCD_MODE       IOID_UNUSED     // Not connected to CC26xx 5x5 test EM
#define BSP_LCD_MODE            0
#define BSP_IOID_LCD_RST        IOID_UNUSED     // Not connected to CC26xx 5x5 test EM
#define BSP_LCD_RST             0
#define BSP_IOID_LCD_CS         IOID_UNUSED     // Not connected to CC26xx 5x5 test EM
#define BSP_LCD_CS              0
#endif // ifdef MODULE_CC26XX_5X5
#define BSP_IOID_LCD_SCK        BSP_IOID_SPI_SCK
#define BSP_LCD_SCK             BSP_SPI_SCK
#define BSP_IOID_LCD_MOSI       BSP_IOID_SPI_MOSI
#define BSP_LCD_MOSI            BSP_SPI_MOSI
#define BSP_IOID_LCD_MISO       BSP_IOID_SPI_MISO
#define BSP_LCD_MISO            BSP_SPI_MISO


//
// Board accelerometer defines
//
#define BSP_IOID_ACC_PWR        IOID_UNUSED     // Not connected to CC26xx 5x5 EM
#define BSP_ACC_PWR             0
#define BSP_IOID_ACC_INT        IOID_UNUSED     // Not connected to CC26xx 5x5 EM
#define BSP_ACC_INT             0
#define BSP_IOID_ACC_INT1       IOID_UNUSED     // Not connected to CC26xx 5x5 EM
#define BSP_ACC_INT1            0
#define BSP_IOID_ACC_INT2       IOID_UNUSED     // Not connected to CC26xx 5x5 EM
#define BSP_ACC_INT2            0
#define BSP_IOID_ACC_CS         IOID_UNUSED     // Not connected to CC26xx 5x5 EM
#define BSP_ACC_CS              0
#define BSP_IOID_ACC_SCK        BSP_IOID_SPI_SCK
#define BSP_ACC_SCK             BSP_SPI_SCK
#define BSP_IOID_ACC_MOSI       BSP_IOID_SPI_MOSI
#define BSP_ACC_MOSI            BSP_SPI_MOSI
#define BSP_IOID_ACC_MISO       BSP_IOID_SPI_MISO
#define BSP_ACC_MISO            BSP_SPI_MISO


//
// SD reader defines
//
#define BSP_IOID_SDCARD_CS      IOID_UNUSED     // Not connected to CC26xx 5x5 EM
#define BSP_SDCARD_CS           0
#define BSP_IOID_SDCARD_SCK     BSP_IOID_SPI_SCK
#define BSP_SDCARD_SCK          BSP_SPI_SCK
#define BSP_IOID_SDCARD_MOSI    BSP_IOID_SPI_MOSI
#define BSP_SDCARD_MOSI         BSP_SPI_MOSI
#define BSP_IOID_SDCARD_MISO    BSP_IOID_SPI_MISO
#define BSP_SDCARD_MISO         BSP_SPI_MISO


//
// Board ambient lightsensor defines
//
#ifdef MODULE_CC26XX_5X5
#define BSP_IOID_ALS_PWR        IOID_UNUSED     // Not connected to CC26xx 5x5 EM
#define BSP_ALS_PWR             0
#define BSP_IOID_ALS_OUT        IOID_UNUSED     // Not connected to CC26xx 5x5 EM
#define BSP_ALS_OUT             0
#else
// MODULE_CC26XX_TEST_5X5
#define BSP_IOID_ALS_PWR        IOID_13
#define BSP_ALS_PWR             (1 << BSP_IOID_ALS_PWR)
#define BSP_IOID_ALS_OUT        IOID_14
#define BSP_ALS_OUT             (1 << BSP_IOID_ALS_OUT)
#endif  // ifdef MODULE_CC26XX_5X5


//
// UART backchannel defines
//
#define BSP_IOID_UART_RXD       IOID_1
#define BSP_UART_RXD            (1 << BSP_IOID_UART_RXD)
#define BSP_IOID_UART_TXD       IOID_0
#define BSP_UART_TXD            (1 << BSP_IOID_UART_TXD)
#define BSP_IOID_UART_CTS       IOID_UNUSED     // Not connected to CC26xx 5x5 EM
#define BSP_UART_CTS            0
#define BSP_IOID_UART_RTS       IOID_UNUSED     // Not connected to CC26xx 5x5 EM
#define BSP_UART_RTS            0
//#define BSP_UART_INT_BM         0xF0          //!< Interrupts handled by bsp
#endif // ifdef BOARD_SMARTRF06EB


/******************************************************************************
* Mark the end of the C bindings section for C++ compilers.
******************************************************************************/
#ifdef __cplusplus
}
#endif
#endif /* #ifndef __BSP_5X5_H__ */

