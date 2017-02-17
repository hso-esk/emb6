//*****************************************************************************
//! @file       bsp.c
//! @brief      Board support package for CC26xx Cortex devices on SmartRF06EB.
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


/**************************************************************************//**
* @addtogroup bsp_api
* @{
******************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "bsp/srf06eb_cc26xx/drivers/source/bsp.h"


#include "driverlib/debug.h"
#include "driverlib/prcm.h"
#include "driverlib/interrupt.h"
#include "driverlib/ioc.h"
#include "driverlib/ssi.h"
#include "driverlib/sys_ctrl.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_prcm.h"
#include "driverlib/osc.h"
#include "driverlib/rom.h"


/******************************************************************************
* LOCAL VARIABLES
*/
#ifdef BOARD_SMARTRF06EB
//
// 3.3-V domain enable call counter
//
static int_fast32_t i8Bsp3V3DomainEnableCount = 0;
#endif


/******************************************************************************
* FUNCTIONS
*/

/**************************************************************************//**
* @brief    This function initializes the CC26xx clocks and I/O for use on
*           SmartRF06EB.
*
*           The function assumes that an external crystal oscillator is
*           available to the CC26xx. The CC26xx system clock is set to the
*           frequency given by input argument \e ui32SysClockSpeed. The I/O
*           system clock is set configured to the same value as the system
*           clock.
*
*           If the value of \e ui32SysClockSpeed is invalid, the system clock
*           is set to the highest allowed value.
*
* @param    ui32SysClockSpeed   is the system clock speed in Hz; it must be one
*                               of the following:
*           \li \b BSP_CLK_SPD_48MHZ
*           \li \b BSP_CLK_SPD_24MHZ
*
* @return   None
******************************************************************************/
void
bspInit(uint32_t ui32SysClockSpeed)
{
    //
    // Assert input arguments
    //
#ifdef FPGA_INCLUDED
    ASSERT((ui32SysClockSpeed == BSP_CLK_SPD_48MHZ) ||
           (ui32SysClockSpeed == BSP_CLK_SPD_24MHZ) ||
           (ui32SysClockSpeed == GET_MCU_CLOCK));
#else
    ASSERT((ui32SysClockSpeed == BSP_CLK_SPD_48MHZ) ||
           (ui32SysClockSpeed == BSP_CLK_SPD_24MHZ));
#endif

    //
    // Disable global interrupts
    //
    bool bIntDisabled = IntMasterDisable();

    //
    // Turn on all power domains
    //
    PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL | PRCM_DOMAIN_VIMS |
                      PRCM_DOMAIN_PERIPH | PRCM_DOMAIN_CPU |
                      PRCM_DOMAIN_SYSBUS);

    //
    // Wait for power on domains
    //
    uint32_t ui32DomainBm = (PRCM_DOMAIN_SERIAL |
                             PRCM_DOMAIN_PERIPH);
    while((PRCMPowerDomainStatus(ui32DomainBm) != PRCM_DOMAIN_POWER_ON))
    {
    }

//    //
//    // Configure and switch clock source (to XOSC)
//    //
//    // NB: Not tested and may not work!
//    OSCClockSourceSet(OSC_SRC_CLK_HF, OSC_XOSC_HF);
//    HapiHFSourceSafeSwitch();

    // Enable GPIO peripheral
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);

    //
    // Apply settings and wait for them to take effect
    //
    PRCMLoadSet();
    while(!PRCMLoadGet());

    //
    // Init LEDs off, output low
    //
#if (BSP_IOID_LED_1 != IOID_UNUSED)
    IOCPinTypeGpioOutput(BSP_IOID_LED_1);
#endif
#if (BSP_IOID_LED_2 != IOID_UNUSED)
    IOCPinTypeGpioOutput(BSP_IOID_LED_2);
#endif
#if (BSP_IOID_LED_3 != IOID_UNUSED)
    IOCPinTypeGpioOutput(BSP_IOID_LED_3);
#endif
#if (BSP_IOID_LED_4 != IOID_UNUSED)
    IOCPinTypeGpioOutput(BSP_IOID_LED_4);
#endif
#if (BSP_LED_ALL != 0)
    GPIO_clearMultiDio(BSP_LED_ALL);
#endif

    //
    // Keys (input pullup)
    //
#ifdef FPGA_INCLUDED
    bspFpgaForceDir(BSP_IOID_KEY_UP, 1, IOC_IOPULL_UP);
    bspFpgaForceDir(BSP_IOID_KEY_DOWN, 1, IOC_IOPULL_UP);
    bspFpgaForceDir(BSP_IOID_KEY_LEFT, 1, IOC_IOPULL_UP);
    bspFpgaForceDir(BSP_IOID_KEY_RIGHT, 1, IOC_IOPULL_UP);
    bspFpgaForceDir(BSP_IOID_KEY_SELECT, 1, IOC_IOPULL_UP);
#else

#if (BSP_IOID_KEY_UP != IOID_UNUSED)
    IOCPinTypeGpioInput(BSP_IOID_KEY_UP);
    IOCIOPortPullSet(BSP_IOID_KEY_UP, IOC_IOPULL_UP);
#endif

#if (BSP_IOID_KEY_DOWN != IOID_UNUSED)
    IOCPinTypeGpioInput(BSP_IOID_KEY_DOWN);
    IOCIOPortPullSet(BSP_IOID_KEY_DOWN, IOC_IOPULL_UP);
#endif
#if (BSP_IOID_KEY_LEFT != IOID_UNUSED)
    IOCPinTypeGpioInput(BSP_IOID_KEY_LEFT);
    IOCIOPortPullSet(BSP_IOID_KEY_LEFT, IOC_IOPULL_UP);
#endif
#if (BSP_IOID_KEY_RIGHT != IOID_UNUSED)
    IOCPinTypeGpioInput(BSP_IOID_KEY_RIGHT);
    IOCIOPortPullSet(BSP_IOID_KEY_RIGHT, IOC_IOPULL_UP);
#endif
#if (BSP_IOID_KEY_SELECT != IOID_UNUSED)
    IOCPinTypeGpioInput(BSP_IOID_KEY_SELECT);
    IOCIOPortPullSet(BSP_IOID_KEY_SELECT, IOC_IOPULL_UP);
#endif
#endif // ifdef FPGA_INCLUDED

#ifdef BOARD_SMARTRF06EB
    //
    // Turn off 3.3-V domain (lcd/sdcard power, output low)
    //
#if (BSP_IOID_3V3_EN != IOID_UNUSED)
    IOCPinTypeGpioOutput(BSP_IOID_3V3_EN);
    bsp3V3DomainDisableForced();
#endif

    //
    // LCD CSn (output high)
    //
#if (BSP_IOID_LCD_CS != IOID_UNUSED)
    IOCPinTypeGpioOutput(BSP_IOID_LCD_CS);
    GPIO_setDio(BSP_IOID_LCD_CS);
#endif

    //
    // SD Card reader CSn (output high)
    //
#if (BSP_IOID_SDCARD_CS != IOID_UNUSED)
    IOCPinTypeGpioOutput(BSP_IOID_SDCARD_CS);
    GPIO_setDio(BSP_IOID_SDCARD_CS);
#endif

    //
    // Accelerometer (PWR output low, CSn output high)
    //
#if (BSP_IOID_ACC_PWR != IOID_UNUSED)
    IOCPinTypeGpioOutput(BSP_IOID_ACC_PWR);
    GPIO_clearDio(BSP_IOID_ACC_PWR);
#endif
#if (BSP_IOID_ACC_CS != IOID_UNUSED)
    IOCPinTypeGpioOutput(BSP_IOID_ACC_CS);
    GPIO_setDio(BSP_IOID_ACC_CS);
#endif

    //
    // Ambient light sensor (off, output low)
    //
#if (BSP_IOID_ALS_PWR != IOID_UNUSED)
    IOCPinTypeGpioOutput(BSP_IOID_ALS_PWR);
    GPIO_clearDio(BSP_IOID_ALS_PWR);
#endif
#if (BSP_IOID_ALS_OUT != IOID_UNUSED)
    IOCPinTypeGpioInput(BSP_IOID_ALS_OUT);
    IOCIOPortPullSet(BSP_IOID_ALS_OUT, IOC_NO_IOPULL);
#endif

    //
    // UART Backchannel (TXD/RXD/CTS/RTS input pullup)
    //
#if (BSP_IOID_UART_RXD != IOID_UNUSED)
    IOCPinTypeGpioInput(BSP_IOID_UART_RXD);
    IOCIOPortPullSet(BSP_IOID_UART_RXD, IOC_IOPULL_UP);
#endif
#if (BSP_IOID_UART_TXD != IOID_UNUSED)
    IOCPinTypeGpioInput(BSP_IOID_UART_TXD);
    IOCIOPortPullSet(BSP_IOID_UART_TXD, IOC_IOPULL_UP);
#endif
#if (BSP_IOID_UART_CTS != IOID_UNUSED)
    IOCPinTypeGpioInput(BSP_IOID_UART_CTS);
    IOCIOPortPullSet(BSP_IOID_UART_CTS, IOC_IOPULL_UP);
#endif
#if (BSP_IOID_UART_RTS != IOID_UNUSED)
    IOCPinTypeGpioInput(BSP_IOID_UART_RTS);
    IOCIOPortPullSet(BSP_IOID_UART_RTS, IOC_IOPULL_UP);
#endif
#endif // ifdef BOARD_SMARTRF06EB

    //
    // Re-enable interrupt if initially enabled.
    //
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }
}


/**************************************************************************//**
* @brief    This function initializes SPI interface. The SPI is configured to
*           Motorola mode with clock idle high and data valid on the second
*           (rising) edge. The SSI module uses the I/O clock as clock source
*           (I/O clock frequency set in bspInit()).
*
*           Input argument \e ui32SpiClockSpeed must obey the following
*           criteria:
*           \li \e ui32SpiClockSpeed = srcClk / n
*           where n is integer, n >= 2, and srcClk is the clock frequency set
*           by bspInit().
*
* @param    ui32SpiClockSpeed   is the SPI clock speed in Hz
*
* @return   None
******************************************************************************/
void
bspSpiInit(uint32_t ui32SpiClockSpeed)
{
    uint32_t ui32Dummy;

    //
    // Enable clock for SSI0 when CM3 is in run mode
    //
    PRCMPeripheralRunEnable(PRCM_PERIPH_SSI0);
    PRCMLoadSet();
    while(!PRCMLoadGet());

    //
    // Disable SSI function before configuring module
    //
    SSIDisable(BSP_SPI_BASE);

    //
    // Map SSP signals to the correct GPIO pins and configure them as HW ctrl'd
    //
    IOCPinTypeSsiMaster(BSP_SPI_BASE, BSP_IOID_SPI_MISO, BSP_IOID_SPI_MOSI,
                        IOID_UNUSED /*FSS*/, BSP_IOID_SPI_SCK);
    //
    // Set SPI mode and speed
    //
    bspSpiClockSpeedSet(ui32SpiClockSpeed);

    //
    // Flush the RX FIFO
    //
    while(SSIDataGetNonBlocking(BSP_SPI_BASE, &ui32Dummy))
    {
    }
}


/**************************************************************************//**
* @brief    This function returns the clock speed of the BSP SPI interface.
*           It is assumed that the BSP SPI SSI module runs off the I/O clock.
*
* @return   Returns the SPI clock speed in Hz
******************************************************************************/
uint32_t
bspSpiClockSpeedGet(void)
{
    return GET_MCU_CLOCK;
}


/**************************************************************************//**
* @brief    This function configures the SPI interface to the given clock
*           speed, Motorola mode with clock idle high and data valid on the
*           second (rising) edge. For proper SPI function, the SPI interface
*           must first be initialized using bspSpiInit().
*
* @warning  Limitations apply to the allowed values of \e ui32ClockSpeed.
*           Please refer to device's driverlib documentation.
*
* @param    ui32ClockSpeed  is the SPI clock speed in Hz
*
* @return   None
******************************************************************************/
void
bspSpiClockSpeedSet(uint32_t ui32ClockSpeed)
{
    //
    // Disable SSI function
    //
    SSIDisable(BSP_SPI_BASE);

    //
    // Configure SSI module to Motorola/Freescale SPI mode 3:
    // Polarity  = 1, SCK steady state is high
    // Phase     = 1, Data changed on first and captured on second clock edge
    // Word size = 8 bits
    //
    SSIConfigSetExpClk(BSP_SPI_BASE, GET_MCU_CLOCK, SSI_FRF_MOTO_MODE_3,
                       SSI_MODE_MASTER, ui32ClockSpeed, 8);

    //
    // Enable the SSI function
    //
    SSIEnable(BSP_SPI_BASE);
}


#ifdef BOARD_SMARTRF06EB
/**************************************************************************//**
* @brief    This function enables the 3.3-V domain on SmartRF06EB. The LCD and
*           SD card reader are powered by the 3.3-V domain. This functon
*           increments a counter variable each time it is called. The function
*           assumes the 3.3-V domain enable pin is configured as output by,
*           for example, bspInit().
*           The 3.3-V domain needs up to approximately 400 us to settle when
*           enabled.
*
* @see      bsp3V3DomainDisable(), bsp3V3DomainDisableForced()
*
* @return   None
******************************************************************************/
void
bsp3V3DomainEnable(void)
{
    //
    // Keep score of the enable/disable relation
    //
    i8Bsp3V3DomainEnableCount++;

    //
    // Enable 3.3-V domain
    //
    GPIO_setDio(BSP_IOID_3V3_EN); // high
}


/**************************************************************************//**
* @brief    This function disables the 3.3-V domain on SmartRF06EB. This
*           function is "soft" and only disables the 3.3-V domain if counter
*           variable \e i8Bsp3V3DomainEnableCount is 1 or 0. The function
*           assumes the 3.3-V domain enable pin is configured as output by,
*           for example, bspInit().

*           This function decrements \e i8Bsp3V3DomainEnableCount and disables
*           the 3.3-V domain if \e i8Bsp3V3DomainEnableCount is less than or
*           equal to 0. If \e i8Bsp3V3DomainEnableCount is greater than 0
*           after decrement, function bsp3V3DomainEnable() has been called more
*           times than this function, and the 3.3-V domain will not be disabled.
*           To disable the 3.3-V domain irrespective of the value of
*           \e i8Bsp3V3DomainEnableCount, use bsp3V3DomainDisableForced().
*
* @see      bsp3V3DomainEnable(), bsp3V3DomainDisableForced()
*
* @return   None
******************************************************************************/
void
bsp3V3DomainDisable(void)
{
    //
    // Only disable 3.3-V domain if disable requests >= enable requests.
    //
    if((--i8Bsp3V3DomainEnableCount) <= 0)
    {
        //
        // Disable 3.3-V domain
        //
        GPIO_clearDio(BSP_IOID_3V3_EN); // low
        i8Bsp3V3DomainEnableCount = 0;
    }
}


/**************************************************************************//**
* @brief    This function disables the 3.3-V domain on SmartRF06EB. The function
*           assumes the 3.3-V domain enable pin is configured as output by,
*           for example, bspInit().
*           The 3.3-V domain needs approximately 400 us to fall below 0.5 V.
*
* @see      bsp3V3DomainEnable(), bsp3V3DomainDisable()
*
* @return   None
******************************************************************************/
void
bsp3V3DomainDisableForced(void)
{
    //
    // Disable 3.3-V domain and reset score variable
    //
    GPIO_clearDio(BSP_IOID_3V3_EN); // low
    i8Bsp3V3DomainEnableCount = 0;
}


/**************************************************************************//**
* @brief    This function returns the current state of the 3.3-V domain.
*
* @return   Returns 1 if 3.3-V domain is enabled
* @return   Returns 0 if 3.3-V domain is disabled
******************************************************************************/
uint8_t
bsp3V3DomainEnabled(void)
{
    return (GPIO_readDio(BSP_IOID_3V3_EN));
}
#endif // ifdef BOARD_SMARTRF06EB


#ifdef FPGA_INCLUDED
/**************************************************************************//**
* @brief    Force FPGA adapter board level shifters to drive signal from FPGA
*           before setting signal back to input pullup
*
* @param    ui32Io      is the IOID to reset.
* @param    forcedVal   is the value to output. 0 or 1
* @param    pullVal     is the final input pull value. Can be one of
*                       \li \c IOC_IOPULL_UP
*                       \li \c IOC_IOPULL_DOWN
*                       \li \c IOC_NO_IOPULL
*
* @return   None
******************************************************************************/
void bspFpgaForceDir(uint32_t ui32Io, uint32_t forcedVal, uint32_t pullVal)
{
    ASSERT((forcedVal < 2) ||
           (pullVal == IOC_IOPULL_UP) ||
           (pullVal == IOC_IOPULL_DOWN) ||
           (pullVal == IOC_NO_IOPULL));

    uint32_t ioBm = (1 << ui32Io);
    do
    {
        IOCPinTypeGpioOutput(ui32Io);
        GPIO_writeDio(ioBm, forcedVal);
        IOCPinTypeGpioInput(ui32Io);
        IOCIOPortPullSet(ui32Io, pullVal);
    }
    while((!!GPIO_readDio(ioBm)) != forcedVal);
}
#endif // ifdef FPGA_INCLUDED


/**************************************************************************//**
* Close the Doxygen group.
* @}
******************************************************************************/
