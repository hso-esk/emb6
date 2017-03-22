//*****************************************************************************
//! @file       bsp.h
//! @brief      Board support package header file for CC26xx on SmartRF06EB.
//!
//! Revised     $Date: 2015-03-26 12:45:20 +0100 (to, 26 mar 2015) $
//! Revision    $Revision: 15463 $
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
#ifndef __SF_BSP_H__
#define __SF_BSP_H__


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
#include <stdint.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"          // Access to the GET_MCU_CLOCK define
#include "inc/hw_ioc.h"
#include "driverlib/ioc.h"
#include "driverlib/gpio.h"


//
// Include bsp file for the correct module
//
#if (defined(MODULE_CC26XX_4X4) || defined(MODULE_CC26XX_TEST_4X4))
#include "bsp_4x4.h"
#elif (defined(MODULE_CC26XX_5X5) || defined(MODULE_CC26XX_TEST_5X5))
#include "bsp_5x5.h"
#elif (defined(MODULE_CC13XX_7X7) || defined(MODULE_CC13XX_TEST_7X7))
#include "bsp_7x7.h"
#else
#if !(defined(MODULE_CC26XX_7X7) || defined(MODULE_CC26XX_TEST_7X7))
#define MODULE_CC26XX_7X7
#endif
#include "bsp_7x7.h"
#endif


/******************************************************************************
* DEFINES
*/
//
// Clock speed defines
//
#define BSP_CLK_SPD_48MHZ       48000000UL
#define BSP_CLK_SPD_24MHZ       24000000UL

//! Default system clock speed
#define BSP_SYS_CLK_SPD         GET_MCU_CLOCK
//! Default SPI clock speed. 6MHz is supported by all board peripherals and dividable by 24MHz.
#define BSP_SPI_CLK_SPD         6000000UL
//! Default UART clock speed (baud rate).
#define BSP_UART_CLK_SPD        115200UL


/******************************************************************************
* FUNCTION PROTOTYPES
*/
extern void bspInit(uint32_t ui32SysClockSpeed);
extern void bspSpiInit(uint32_t ui32ClockSpeed);
extern uint32_t bspSpiClockSpeedGet(void);
extern void bspSpiClockSpeedSet(uint32_t ui32ClockSpeed);
#ifdef BOARD_SMARTRF06EB
extern void bsp3V3DomainEnable(void);
extern void bsp3V3DomainDisable(void);
extern void bsp3V3DomainDisableForced(void);
extern uint8_t bsp3V3DomainEnabled(void);
#endif // ifdef BOARD_SMARTRF06EB
#ifdef FPGA_INCLUDED
extern void bspFpgaForceDir(uint32_t ui32Io, uint32_t forcedVal,
                            uint32_t pullVal);
#endif


/******************************************************************************
* Mark the end of the C bindings section for C++ compilers.
******************************************************************************/
#ifdef __cplusplus
}
#endif
#endif /* #ifndef __SF_BSP_H__ */
