//*****************************************************************************
//! @file       bsp_4x4.h
//! @brief      Board support package header file for CC26xx QFN32 4x4 EM
//!             on SmartRF06EB, with access to the following 06EB peripherals:
//!             SPI, 2-pin UART, 1-3 keys (1 if trace enabled, 2 if network
//!             processor, 3 if not network processor) and 2 LEDs.
//!
//! Revised     $Date: 2013-10-28 10:44:55 +0100 (ma, 28 okt 2013) $
//! Revision    $Revision: 10965 $
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
#ifndef __BSP_4X4_H__
#define __BSP_4X4_H__

#if !(defined(MODULE_CC26XX_4X4) || defined(MODULE_CC26XX_TEST_4X4))
#error bsp_4x4.h: Unknown module definition!
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
#define BSP_IOID_SPI_SCK        IOID_8
#define BSP_SPI_SCK             (1 << BSP_IOID_SPI_SCK)
#define BSP_IOID_SPI_MOSI       IOID_9
#define BSP_SPI_MOSI            (1 << BSP_IOID_SPI_MOSI)
#define BSP_IOID_SPI_MISO       IOID_0
#define BSP_SPI_MISO            (1 << BSP_IOID_SPI_MISO)


//
// Board LED defines
//
#define BSP_IOID_LED_1          IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_LED_1               0
#define BSP_IOID_LED_2          IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_LED_2               0
#ifdef MODULE_CC26XX_4X4
#define BSP_IOID_LED_3          IOID_5
#define BSP_LED_3               (1 << BSP_IOID_LED_3)
#define BSP_IOID_LED_4          IOID_6
#define BSP_LED_4               (1 << BSP_IOID_LED_4)
#else
// MODULE_CC26XX_TEST_4X4
#define BSP_IOID_LED_3          IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_LED_3               0
#define BSP_IOID_LED_4          IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_LED_4               0
#endif
#define BSP_LED_ALL             (BSP_LED_1 | BSP_LED_2 | BSP_LED_3 |          \
                                 BSP_LED_4)     //!< Bitmask of all LEDs

//
// Board key defines
//
#define BSP_IOID_KEY_LEFT       IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_KEY_LEFT            0
#define BSP_IOID_KEY_RIGHT      IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_KEY_RIGHT           0
#ifdef MODULE_CC26XX_4X4
#define BSP_IOID_KEY_UP         IOID_4
#define BSP_KEY_UP              (1 << BSP_IOID_KEY_UP)
#define BSP_IOID_KEY_DOWN       IOID_3
#define BSP_KEY_DOWN            (1 << BSP_IOID_KEY_DOWN)
#else
// MODULE_CC26XX_TEST_4X4
#define BSP_IOID_KEY_UP         IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_KEY_UP              0
#define BSP_IOID_KEY_DOWN       IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_KEY_DOWN            0
#endif // #ifdef MODULE_CC26XX_4X4
#define BSP_IOID_KEY_SELECT     IOID_7
#define BSP_KEY_SELECT          (1 << BSP_IOID_KEY_SELECT)

#ifdef MODULE_CC26XX_4X4
#define BSP_KEY_DIR_ALL         (BSP_KEY_UP | \
                                 BSP_KEY_DOWN) //!< Bitmask of all dir. keys
#define BSP_KEY_ALL             (BSP_KEY_DIR_ALL | \
                                 BSP_KEY_SELECT) //!< Bitmask of all keys
#else
// MODULE_CC26XX_TEST_4X4
#define BSP_KEY_DIR_ALL         0
#define BSP_KEY_ALL             BSP_KEY_SELECT //!< Bitmask of all keys
#endif // #ifdef MODULE_CC26XX_4X4


#ifdef BOARD_SMARTRF06EB
//
// 3.3-V domain defines
//
#define BSP_IOID_3V3_EN         IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_3V3_EN              0


//
// Board LCD defines
//
#define BSP_IOID_LCD_MODE       IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_LCD_MODE            0
#define BSP_IOID_LCD_RST        IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_LCD_RST             0
#define BSP_IOID_LCD_CS         IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_LCD_CS              0
#define BSP_IOID_LCD_SCK        BSP_IOID_SPI_SCK
#define BSP_LCD_SCK             (1 << BSP_IOID_LCD_SCK)
#define BSP_IOID_LCD_MOSI       BSP_IOID_SPI_MOSI
#define BSP_LCD_MOSI            (1 << BSP_IOID_LCD_MOSI)
#define BSP_IOID_LCD_MISO       BSP_IOID_SPI_MISO
#define BSP_LCD_MISO            (1 << BSP_IOID_LCD_MISO)


//
// Board accelerometer defines
//
#define BSP_IOID_ACC_PWR        IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_ACC_PWR             0
#define BSP_IOID_ACC_INT        IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_ACC_INT             0
#define BSP_IOID_ACC_INT1       IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_ACC_INT1            0
#define BSP_IOID_ACC_INT2       IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_ACC_INT2            0
#define BSP_IOID_ACC_CS         IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_ACC_CS              0
#define BSP_IOID_ACC_SCK        BSP_IOID_SPI_SCK
#define BSP_IOID_ACC_MOSI       BSP_IOID_SPI_MOSI
#define BSP_IOID_ACC_MISO       BSP_IOID_SPI_MISO


//
// SD reader defines
//
#define BSP_IOID_SDCARD_CS      IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_SDCARD_CS           0
#define BSP_IOID_SDCARD_SCK     BSP_IOID_SPI_SCK
#define BSP_SDCARD_SCK          0
#define BSP_IOID_SDCARD_MOSI    BSP_IOID_SPI_MOSI
#define BSP_SDCARD_MOSI         0
#define BSP_IOID_SDCARD_MISO    BSP_IOID_SPI_MISO
#define BSP_SDCARD_MISO         0


//
// Board ambient lightsensor defines
//
#define BSP_IOID_ALS_PWR        IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_ALS_PWR             0
#define BSP_IOID_ALS_OUT        IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_ALS_OUT             0


//
// UART backchannel defines
//
#define BSP_IOID_UART_RXD       IOID_1
#define BSP_UART_RXD            (1 << BSP_IOID_UART_RXD)
#define BSP_IOID_UART_TXD       IOID_2
#define BSP_UART_TXD            (1 << BSP_IOID_UART_TXD)
#define BSP_IOID_UART_CTS       IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_UART_CTS            0
#define BSP_IOID_UART_RTS       IOID_UNUSED     // Not connected to CC26xx 4x4 EM
#define BSP_UART_RTS            0
//#define BSP_UART_INT_BM         0xF0          //!< Interrupts handled by bsp
#endif // ifdef BOARD_SMARTRF06EB


/******************************************************************************
* Mark the end of the C bindings section for C++ compilers.
******************************************************************************/
#ifdef __cplusplus
}
#endif
#endif /* #ifndef __BSP_4X4_H__ */

