//*****************************************************************************
//! @file       lcd_srf06eb.c
//! @brief      SmartRF06EB specific device driver implementation for
//!             DOGM128W-6 LCD display. See lcd_dogm128_6.c for more info.
//!
//!             LCD animations in lcdSendBufferAnimated uses active state
//!             wait (CPU loop). Transition time is approximate, based on CPU
//!             clock speed.
//!
//! Revised     $Date: 2013-04-11 19:41:57 +0200 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9707 $
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
#ifndef LCD_EXCLUDE
#ifdef BOARD_SMARTRF06EB


/**************************************************************************//**
* @addtogroup lcd_dogm128_6_api
* @{
******************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "bsp/srf06eb_cc26xx/drivers/source/bsp.h"
#include "lcd_dogm128_6.h"

#include "driverlib/cpu.h"
#include "driverlib/sys_ctrl.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/ioc.h"


/******************************************************************************
* DEFINES
*/
// Base address of SSP module used by implementation
#define LCD_SSP_BASE            BSP_SPI_BASE

// Base address of timer module used by implementation
#define LCD_TIMER_BASE          GPTIMER0_BASE

// Macro for asserting LCD CSn (set low)
#define LCD_SPI_BEGIN()         GPIO_clearDio(BSP_IOID_LCD_CS);

// Macro for deasserting LCD CSn (set high)
#define LCD_SPI_END()           GPIO_setDio(BSP_IOID_LCD_CS);

// Macro for setting LCD mode signal low (command)
#define LCD_MODE_SET_CMD()      GPIO_clearDio(BSP_IOID_LCD_MODE);

// Macro for setting LCD mode signal (data)
#define LCD_MODE_SET_DATA()     GPIO_setDio(BSP_IOID_LCD_MODE);

// Determine if necessary IO is mapped in bsp.h
#if ((BSP_IOID_LCD_MODE == IOID_UNUSED) || (BSP_IOID_LCD_CS == IOID_UNUSED) || \
     (BSP_IOID_LCD_MOSI == IOID_UNUSED) || (BSP_IOID_LCD_SCK == IOID_UNUSED))
#ifndef LCD_IO_MISSING
#define LCD_IO_MISSING
#endif
#endif


/******************************************************************************
* VARIABLES AND LOCAL FUNCTIONS
*/
#ifndef LCD_IO_MISSING
static void lcdSendArray(const char *pcData, uint16_t ui1Length);

// LCD initialization command sequence
static const char lcdInitCmd[] =
{
    0x40, /*Display start line 0                    */
    0xA1, /*ADC reverse, 6 oclock viewing direction */
    0xC0, /*Normal COM0...COM63                     */
    0xA6, /*Display normal, not mirrored            */
    0xA2, /*Set Bias 1/9 (Duty 1/65)                */
    0x2F, /*Booster, Regulator and Follower On      */
    0xF8, /*Set internal Booster to 4x              */
    0x00, /*                                        */
    0x27, /*Contrast set                            */
    0x81, /*                                        */
    0x16, /* <- use value from LCD-MODULE .doc guide*/
    /*    for better contrast (not 0x10)      */
    0xAC, /*No indicator                            */
    0x00, /*                                        */
    0xAF, /*Display on                              */
    0xB0, /*Page 0 einstellen                       */
    0x10, /*High-Nibble of column address           */
    0x00  /*Low-Nibble of column address            */
};
#endif // ifndef LCD_IO_MISSING


/******************************************************************************
* FUNCTIONS
*/
/**************************************************************************//**
* @brief    This function initializes the LCD. This function assumes that the
*           SPI interface has been initialized using, for example, bspSpiInit().
*           lcdInit() must be run before you can use the LCD.
*
* @return   None
******************************************************************************/
void
lcdInit(void)
{
#ifndef LCD_IO_MISSING
    //
    // Enable 3.3-V domain
    //
    bsp3V3DomainEnable();

    //
    // Configure CSn and MODE(A0) as output high
    //
    IOCPinTypeGpioOutput(BSP_IOID_LCD_CS);
    IOCPinTypeGpioOutput(BSP_IOID_LCD_MODE);
    GPIO_setMultiDio(BSP_LCD_CS | BSP_LCD_MODE);

    //
    // Configure LCD RST as output low
    //
    IOCPinTypeGpioOutput(BSP_IOID_LCD_RST);
    GPIO_clearDio(BSP_IOID_LCD_RST);

    //
    // Delay ~100ms for LCD to be powered up
    //
    CPUdelay((GET_MCU_CLOCK / 10) / 3);

    //
    // Clear reset (set high)
    //
    GPIO_setDio(BSP_IOID_LCD_RST);

    //
    // Send LCD init commands
    //
    lcdSendCommand(lcdInitCmd, sizeof(lcdInitCmd));
#endif // ifndef LCD_IO_MISSING
}


/**************************************************************************//**
* @brief    This function clears the LCD display. This function acts directly
*           on the display and does not modify internal buffers.
*
* @return   None
******************************************************************************/
void
lcdClear(void)
{
#ifndef LCD_IO_MISSING
    uint32_t ui32Dummy;
    uint_fast8_t ui8Length, ui8Page;

    //
    // For each LCD page
    //
    for(ui8Page = 0; ui8Page < LCD_PAGE_ROWS; ui8Page++)
    {
        //
        // Set LCD's internal pointer
        //
        lcdGotoXY(0, ui8Page);

        //
        // Set number of data bytes to transfer
        //
        ui8Length = LCD_COLS;

        //
        // Tell LCD we are transferring data (not commands) and set CSn
        //
        LCD_MODE_SET_DATA();
        LCD_SPI_BEGIN();

        //
        // Write data
        //
        while(ui8Length--)
        {
            //
            // Send empty byte to SSP FIFO (function waits for space in FIFO)
            //
            SSIDataPut(LCD_SSP_BASE, 0x00);
        }

        //
        //  Wait for transfer to complete
        //
        while(SSIBusy(LCD_SSP_BASE))
        {
        }

        //
        // Clear CSn
        //
        LCD_SPI_END();
    }

    //
    // Flush SSP IN FIFO
    //
    while(SSIDataGetNonBlocking(LCD_SSP_BASE, &ui32Dummy))
    {
    }
#endif // ifndef LCD_IO_MISSING
}


/**************************************************************************//**
* @brief    This function initializes the LCD SPI interface to the maximum
*           allowed speed.
*
* @return   None
******************************************************************************/
void
lcdSpiInit(void)
{
    bspSpiInit((GET_MCU_CLOCK / 2));
}


/**************************************************************************//**
* @brief    This function sends \e ui8Len bytes of commands to the LCD
*           controller.
*
* @param    pcCmd       is a pointer to the array of commands.
* @param    ui8Len      is the number of bytes to send.
*
* @return   None
******************************************************************************/
void
lcdSendCommand(const char *pcCmd, uint8_t ui8Len)
{
#ifndef LCD_IO_MISSING
    //
    // Wait for ongoing transfers to complete
    //
    while(SSIBusy(LCD_SSP_BASE))
    {
    }

    //
    // Set CSn and indicate command to LCD (A0 low)
    //
    LCD_SPI_BEGIN();
    LCD_MODE_SET_CMD();

    //
    // Transfer commands
    //
    lcdSendArray(pcCmd, ui8Len);

    //
    // Clear CSn
    //
    LCD_SPI_END();
#endif // ifndef LCD_IO_MISSING
}


/**************************************************************************//**
* @brief    This function sends \e ui8Len bytes of data to be displayed on
*           the LCD.
*
* @param    pcData      is a pointer to the array of data.
* @param    ui1Len      is the number of bytes to send.
*
* @return   None
******************************************************************************/
void
lcdSendData(const char *pcData, uint16_t ui1Len)
{
#ifndef LCD_IO_MISSING
    //
    // Wait for ongoing transfers to complete
    //
    while(SSIBusy(LCD_SSP_BASE))
    {
    }

    //
    // Set CSn and indicate data to LCD (A0 high)
    //
    LCD_SPI_BEGIN();
    LCD_MODE_SET_DATA();

    //
    // Transfer data
    //
    lcdSendArray(pcData, ui1Len);

    //
    // Clear CSn
    //
    LCD_SPI_END();
#endif // ifndef LCD_IO_MISSING
}


/**************************************************************************//**
* @brief    This function updates the LCD display by creating an animated
*           transition between two display buffers. Two animations,
*           \b eLcdSlideLeft and \b eLcdSlideRight, slide the new screen left
*           or right, respectively.
*
*           Function lcdSendBuffer() updates the display to show the new
*           buffer instantanously. lcdSendBufferAnimated() on the other side,
*           makes a smooth transition into showing the new buffer.
*
*           \e pcToBuffer should point to the buffer the LCD display
*           transitions in to.
*           \e pcFromBuffer should point to the buffer that what was sent to
*           the LCD display last time lcdSendBuffer() or lcdSendBufferAnimated()
*           was called. By taking both the present and the upcoming display
*           buffers as parameters, lcdSendBufferAnimated() does not take up any
*           memory unless used.
*
*           Example:
*           -# Send a buffer to the display using for example lcdSendBuffer().
*           -# Manipulate a second buffer using \b lcdBuffer functions.
*           -# Run lcdSendBufferAnimated() to update display with a
*              smooth transition from the initial to the second buffer.
*
* @param    pcToBuffer      is a pointer to the buffer with the new display
*                           content.
* @param    pcFromBuffer    is a pointer to the buffer with the existing
*                           display content.
* @param    iMotion         indicates which animation to use for transition.
*                           Must be one of the following enumerated values:
*                           \li \b eLcdSlideLeft
*                           \li \b eLcdSlideRight
*
* @return   None
******************************************************************************/
void
lcdSendBufferAnimated(const char *pcToBuffer, const char *pcFromBuffer,
                      tLcdMotion iMotion)
{
#ifndef LCD_IO_MISSING
    uint32_t ui32DelayCycles = (GET_MCU_CLOCK == 48000000) ? 24000 : 1;
    char pcPageData[LCD_COLS];
    uint8_t ui8Offset, ui8PageIndex, ui8I;
    char *pcToBuf = (char *)pcToBuffer;
    char *pcFromBuf = (char *)pcFromBuffer;

    //
    // Buffers are the same, do not animate
    //
    if(pcToBuffer == pcFromBuffer)
    {
        lcdSendBuffer(pcToBuffer);
        return;
    }

#ifndef LCD_NO_DEFAULT_BUFFER
    // Use default buffer if null pointers
    if(!pcToBuf)
    {
        pcToBuf = lcdDefaultBuffer;
    }
    else if(!pcFromBuf)
    {
        pcFromBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    for(ui8Offset = 0; ui8Offset <= LCD_COLS; ui8Offset += 4)
    {
        // For each page
        for(ui8PageIndex = 0; ui8PageIndex < 8; ui8PageIndex++)
        {
            //  Assigning data to this page from both buffers
            for(ui8I = 0; ui8I < LCD_COLS; ui8I++)
            {
                if(iMotion == eLcdSlideLeft)
                {
                    if(ui8I + ui8Offset < LCD_COLS)
                    {
                        pcPageData[ui8I] = *(pcFromBuf + (ui8PageIndex * LCD_COLS + ui8I + ui8Offset));
                    }
                    else
                    {
                        pcPageData[ui8I] = *(pcToBuf + (ui8PageIndex * LCD_COLS + ui8I + ui8Offset - LCD_COLS));
                    }
                }
                else
                {
                    //
                    // eLcdSlideRight
                    //
                    if(ui8I - ui8Offset >= 0)
                    {
                        pcPageData[ui8I] = *(pcFromBuf + (ui8PageIndex * LCD_COLS + ui8I - ui8Offset));
                    }
                    else
                    {
                        pcPageData[ui8I] = *(pcToBuf + (ui8PageIndex * LCD_COLS + ui8I - ui8Offset + LCD_COLS));
                    }
                }
            }

            //
            // Set pointer to start of row/page and send the page
            //
            lcdGotoXY(0, ui8PageIndex);
            lcdSendData(pcPageData, LCD_COLS);
        }

        //
        // Active state wait
        //
       CPUdelay(ui32DelayCycles);        // ~ 1.5ms
    }
#endif // ifndef LCD_IO_MISSING
}


/**************************************************************************//**
* @brief    Sends \e ui1Length bytes from starting from address
*           \e pcData over SPI to the LCD controller. This function only pushes
*           data to the SPI module. It does not manipulate the LCD display's
*           CSn signal, nor the LCD mode signal (A0).
*
* @param    pcData      Pointer to the array to be sent to the LCD.
* @param    ui1Length    Number of bytes to send.
*
* @return   None
******************************************************************************/
#ifndef LCD_IO_MISSING
static void
lcdSendArray(const char *pcData, uint16_t ui1Length)
{

    uint32_t ui32Dummy;

    // Write data
    while(ui1Length--)
    {
        // Send byte to SSI FIFO (function waits until space in FIFO)
        SSIDataPut(LCD_SSP_BASE, *(pcData++));
    }

    //  Wait for transfer to complete
    while(SSIBusy(LCD_SSP_BASE))
    {
    }

    // Empty SSI in FIFO
    while(SSIDataGetNonBlocking(LCD_SSP_BASE, &ui32Dummy));
}
#endif // ifndef LCD_IO_MISSING


/**************************************************************************//**
* Close the Doxygen group.
* @}
******************************************************************************/
#endif // #ifdef BOARD_SMARTRF06EB
#endif // #ifndef LCD_EXCLUDE
