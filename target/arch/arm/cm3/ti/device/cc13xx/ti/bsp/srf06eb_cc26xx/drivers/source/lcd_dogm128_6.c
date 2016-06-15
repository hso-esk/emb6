//*****************************************************************************
//! @file       lcd_dogm128_6.c
//! @brief      Driver implementation for DOGM128W-6 LCD display. This file
//!             contains platform independent functions. Platform dependent
//!             functions are located in lcd_xxx.c where xxx is
//!             the target in question, for example, srf06eb or trxeb.
//!
//!             The (x,y) coordinate system
//!             used in this device driver is as described below:
//!             <pre>
//!             + ----->   x
//!             | +---------------------------------------------+
//!             | |(0,0)              PAGE 0             (127,0)|
//!             V |                   PAGE 1                    |
//!               |                    ...                      |
//!             y |                    ...                      |
//!               |                    ...                      |
//!               |                    ...                      |
//!               |                    ...                      |
//!               |(0,63)             PAGE 7            (127,63)|
//!               +---------------------------------------------+</pre>
//!
//! Revised     $Date: 2014-04-10 10:18:53 +0200 (to, 10 apr 2014) $
//! Revision    $Revision: 12839 $
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


/**************************************************************************//**
* @addtogroup   lcd_dogm128_6_api
* @{
******************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "lcd_dogm128_6.h"


/******************************************************************************
* DEFINES
*/


/******************************************************************************
* VARIABLES AND LOCAL FUNCTIONS
*/
#ifndef LCD_NO_DEFAULT_BUFFER
char lcdDefaultBuffer[LCD_BYTES] = {0};
#endif

static void lcdBufferLine(char *pcBuffer, uint8_t ui8XFrom, uint8_t ui8YFrom,
                          uint8_t ui8XTo, uint8_t ui8YTo, uint8_t ui8Draw);


/******************************************************************************
* FUNCTIONS
*/
// TARGET SPECIFIC FUNCTIONS (NOT IMPLEMENTED HERE):
// void lcdInit(void)
// void lcdSpiInit(void)
// void lcdClear(void)
// void lcdSendCommand(const char *pcCmd, uint8_t ui8Len)
// void lcdSendData(const char *pcData, uint16_t ui8Len)
// void lcdSendBufferAnimated(const char *pcToBuffer, const char *pcFromBuffer,
//                            tLcdMotion iMotion)


/**************************************************************************//**
* @brief    This function sends the specified buffer to the display. The buffer
*           size is assumed to be 1024 bytes. Passing \e pcBuffer as 0 will
*           send the default buffer. If \b LCD_NO_DEFAULT_BUFFER is defined,
*           passing \e pcBuffer as 0 will result in undefined behavior.
*
* @param    pcBuffer    is a pointer to the source buffer.
*
* @return   None
******************************************************************************/
void
lcdSendBuffer(const char *pcBuffer)
{
    uint8_t ui8Page;
    char *pcBuf = (char *)pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // For each page
    //
    for(ui8Page = 0; ui8Page < 8; ui8Page++)
    {
        //
        // Set LCD pointer to start of the correct page and send data
        //
        lcdGotoXY(0, ui8Page);
        lcdSendData(pcBuf + (ui8Page * LCD_COLS), LCD_COLS);
    }
}


/**************************************************************************//**
* @brief    This function sends the specfied part of \e pcBuffer to the
*           corresponding part on the LCD. This function assumes
*           \e ui8XFrom <= \e ui8XTo and \e iPageFrom <= \e iPageTo. The
*           resolution is given in coulmns [0--127] and pages [0--7].
*
* @param    pcBuffer    is a pointer to the buffer to send. The default
*                       buffer is sent if \e pcBuffer is 0.
* @param    ui8XFrom    is the lowest x-position (column) to write [0--127].
* @param    ui8XTo      is the highest x-position to write [ui8XFrom--127].
* @param    iPageFrom   is the first page to write. Must be one of the
*                       following enumerated values:
*                       \li \b eLcdPage0
*                       \li \b eLcdPage1
*                       \li \b eLcdPage2
*                       \li \b eLcdPage3
*                       \li \b eLcdPage4
*                       \li \b eLcdPage5
*                       \li \b eLcdPage6
*                       \li \b eLcdPage7
* @param    iPageTo     is the last page to write [iPageFrom--eLcdPage7].
*
* @return   None
******************************************************************************/
void
lcdSendBufferPart(const char *pcBuffer, uint8_t ui8XFrom, uint8_t ui8XTo,
                  tLcdPage iPageFrom, tLcdPage iPageTo)
{
    uint8_t ui8XRange, ui8Y, ui8YOffset, ui8YRange;
    char *pcBuf = (char *)pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // Assuming ui8XFrom <= ui8XTo
    //
    ui8XRange = ui8XTo - ui8XFrom + 1;

    //
    // Assuming iPageFrom <= iPage To
    //
    ui8YRange = iPageTo - iPageFrom;

    //
    // For each page, send data
    //
    for(ui8Y = 0; ui8Y <= ui8YRange; ui8Y++)
    {
        ui8YOffset = iPageFrom + ui8Y;
        lcdGotoXY(ui8XFrom, ui8YOffset);
        lcdSendData(pcBuf + (ui8YOffset * LCD_COLS) + ui8XFrom, ui8XRange);
    }
}


/**************************************************************************//**
* @brief    This function sets the internal data cursor of the LCD to the
*           location specified by \e ui8X and \e ui8Y. When data is sent to the
*           display, data will start printing at internal cursor location.
*
* @param    ui8X        is the column [0--127].
* @param    ui8Y        is the page [0--7].
*
* @return   None
******************************************************************************/
void
lcdGotoXY(uint8_t ui8X, uint8_t ui8Y)
{
    uint8_t cmd[] = {0xB0, 0x10, 0x00};

    //
    // Adding Y position, and X position (hi/lo nibble) to command array
    //
    cmd[0] = cmd[0] + ui8Y;
    cmd[2] = cmd[2] + (ui8X & 0x0F);
    cmd[1] = cmd[1] + (ui8X >> 4);

    lcdSendCommand((char *)cmd, 3);
}


/**************************************************************************//**
* @brief    This function sets the LCD contrast.
*
* @param    ui8Contrast  is the contrast value [0--63].
*
* @return   None
******************************************************************************/
void
lcdSetContrast(uint8_t ui8Contrast)
{
    char pCmd[2];

    //
    // Populate command array and send command
    //
    pCmd[0] = 0x81;
    pCmd[1] = (ui8Contrast & 0x3f);
    lcdSendCommand(pCmd, 2);
}


/**************************************************************************//**
* @brief    This function empties the LCD buffer specified by argument
*           \e pcBuffer by filling it with zeros.
*
* @param    pcBuffer    is a pointer to the target buffer.
*
* @return   None
******************************************************************************/
void
lcdBufferClear(char *pcBuffer)
{
    uint16_t ui8Idx;
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    for(ui8Idx = 0; ui8Idx < LCD_BYTES; ui8Idx++)
    {
        *(pcBuf + ui8Idx) = 0x00;
    }
}


/**************************************************************************//**
* @brief    This function clears the page specified by \e iPage in LCD buffer
*           specified by \e pcBuffer.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    iPage       is the page to clear. Must be one of the
*                       following enumerated values:
*                       \li \b eLcdPage0
*                       \li \b eLcdPage1
*                       \li \b eLcdPage2
*                       \li \b eLcdPage3
*                       \li \b eLcdPage4
*                       \li \b eLcdPage5
*                       \li \b eLcdPage6
*                       \li \b eLcdPage7
*
* @return   None
******************************************************************************/
void
lcdBufferClearPage(char *pcBuffer, tLcdPage iPage)
{
    uint8_t ui8Idx;
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // Clear page in buffer
    //
    for(ui8Idx = 0; ui8Idx < LCD_COLS; ui8Idx++)
    {
        *(pcBuf + (iPage * LCD_COLS + ui8Idx)) = 0x00;
    }
}


/**************************************************************************//**
* @brief    This function clears the pixels in a given piece of a page.
*           Resolution is given in coulmns [0--127] and pages [0--7]. The
*           function assumes \e ui8XFrom <= \e ui8XTo and
*           \e iPageFrom <= \e iPageTo.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    ui8XFrom    is the lowest x-position (column) to be cleared
*                       [0--127].
* @param    ui8XTo      is the highest x-position to be cleared [ui8XFrom--127].
* @param    iPageFrom   is the first page cleared. Must be one of the
*                       following enumerated values:
*                       \li \b eLcdPage0
*                       \li \b eLcdPage1
*                       \li \b eLcdPage2
*                       \li \b eLcdPage3
*                       \li \b eLcdPage4
*                       \li \b eLcdPage5
*                       \li \b eLcdPage6
*                       \li \b eLcdPage7
* @param    iPageTo     is the last page cleared [iPageFrom--eLcdPage7].
*
* @return   None
******************************************************************************/
void
lcdBufferClearPart(char *pcBuffer, uint8_t ui8XFrom, uint8_t ui8XTo,
                   tLcdPage iPageFrom, tLcdPage iPageTo)
{
    uint8_t ui8X, ui8XRange, ui8Y, ui8YRange;
    uint16_t ui1XFirstPos;
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // Assuming ui8YFrom <= ui8YTo
    //
    ui8XRange = ui8XTo - ui8XFrom;

    //
    // Assuming ui8YFrom <= ui8YTo
    //
    ui8YRange = iPageTo - iPageFrom;

    //
    // Clear buffer part
    //
    for(ui8Y = 0; ui8Y <= ui8YRange; ui8Y++)
    {
        ui1XFirstPos = (iPageFrom + ui8Y) * LCD_COLS + ui8XFrom;
        for(ui8X = 0; ui8X <= ui8XRange; ui8X++)
        {
            *(pcBuf + (ui1XFirstPos + ui8X)) = 0x00;
        }
    }
}


/**************************************************************************//**
* @brief    This function inverts the pixels (bits) in a given region of the
*           buffer specified by \e pcBuffer.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    ui8XFrom    is the first x-position (column) to invert [0--127].
* @param    ui8YFrom    is the first y-position (row) to invert [0--63].
* @param    ui8XTo      is the last x-position (column) to invert [0--127].
* @param    ui8YTo      is the last y-position (row) to invert [0--63].
*
* @return   None
******************************************************************************/
void
lcdBufferInvert(char *pcBuffer, uint8_t ui8XFrom, uint8_t ui8YFrom,
                uint8_t ui8XTo, uint8_t ui8YTo)
{
    uint8_t ui8I, ui8J, ui8Pow;
    uint8_t ui8FirstPage, ui8LastPage, ui8FirstPageMask, ui8LastPageMask;
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // Find the first and last page to invert on
    //
    ui8FirstPage = ui8YFrom / LCD_PAGE_ROWS;
    ui8LastPage = ui8YTo / LCD_PAGE_ROWS;

    //
    // Find the bitmask to invert with on first page
    //
    ui8FirstPageMask = 0xFF;
    ui8Pow = 1;

    //
    // Generate invert bitmask for the first page
    //
    for(ui8I = 0; ui8I < LCD_PAGE_ROWS; ui8I++)
    {
        if(ui8YFrom - ui8FirstPage * LCD_PAGE_ROWS > ui8I)
        {
            ui8FirstPageMask -= ui8Pow;
            ui8Pow *= 2;
        }
    }

    //
    // Find the bitmask to invert with on the last page
    //
    ui8LastPageMask = 0x00;
    ui8Pow = 1;
    for(ui8I = 0; ui8I < LCD_PAGE_ROWS; ui8I++)
    {
        if(ui8YTo - ui8LastPage * LCD_PAGE_ROWS >= ui8I)
        {
            ui8LastPageMask += ui8Pow;
            ui8Pow *= 2;
        }
    }

    //
    // Prevent error if ui8FirstPage==ui8LastPage
    //
    if(ui8FirstPage == ui8LastPage)
    {
        ui8LastPageMask ^= 0xFF;
    }

    //
    // Invert the given part of the first page
    //
    for(ui8I = ui8XFrom; ui8I <= ui8XTo; ui8I++)
    {
        *(pcBuf + (ui8FirstPage * LCD_COLS + ui8I)) ^= ui8FirstPageMask;
    }

    //
    // Invert the pages between first and last in the given section
    //
    for(ui8I = ui8FirstPage + 1; ui8I <= ui8LastPage - 1; ui8I++)
    {
        for(ui8J = ui8XFrom; ui8J <= ui8XTo; ui8J++)
        {
            *(pcBuf + (ui8I * LCD_COLS + ui8J)) ^= 0xFF;
        }
    }

    //
    // Invert the given part of the last page
    //
    for(ui8I = ui8XFrom; ui8I <= ui8XTo; ui8I++)
    {
        *(pcBuf + (ui8LastPage * LCD_COLS + ui8I)) ^= ui8LastPageMask;
    }
}


/**************************************************************************//**
* @brief    This function inverts a range of columns in the display buffer on a
*           specified page (for example, \b eLcdPage0). This function assumes
*           \e ui8XFrom <= \e ui8XTo.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    ui8XFrom    is the first x-position (column) to invert [0--127].
* @param    ui8XTo      is the last x-position  to invert [ui8XFrom--127].
* @param    iPage       is the page on which to invert. Must be one of the
*                       following enumerated values:
*                       \li \b eLcdPage0
*                       \li \b eLcdPage1
*                       \li \b eLcdPage2
*                       \li \b eLcdPage3
*                       \li \b eLcdPage4
*                       \li \b eLcdPage5
*                       \li \b eLcdPage6
*                       \li \b eLcdPage7
*
* @return   None
******************************************************************************/
void
lcdBufferInvertPage(char *pcBuffer, uint8_t ui8XFrom, uint8_t ui8XTo,
                    tLcdPage iPage)
{
    uint8_t ui8I;
    uint16_t ui1FirstPos = iPage * LCD_COLS + ui8XFrom;
    uint8_t ui8Range = ui8XTo - ui8XFrom;
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // Invert buffer range
    //
    for(ui8I = 0; ui8I < ui8Range; ui8I++)
    {
        *(pcBuf + (ui1FirstPos + ui8I)) ^= 0xFF;
    }
}


/**************************************************************************//**
* @brief    Returns the length a c-string in number of characters by looking
*           for the end-of-string character '<tt>\\0</tt>'.
*           Multiply by \b LCD_CHAR_WIDTH to get length in pixels.
*
* @param    pcStr       is the null-terminated string whose character length
*                       is determined.
*
* @return   Returns length of \e pcStr
******************************************************************************/
uint8_t
lcdGetStringLength(const char *pcStr)
{
    uint_fast8_t ui8I = 0;
    while(pcStr[ui8I] != '\0')
    {
        ui8I++;
    }
    return (ui8I);
}


/**************************************************************************//**
* @brief    This function returns the character length an integer will use on
*           the LCD display. For example, \e i32Number = 215 returns 3
*           and \e i32Number = --215 returns 4 (add one for the minus character).
*           Multiply result of lcdGetIntLength() by \b LCD_CHAR_WIDTH to
*           determine the number of pixels needed by \e i32Number.
*
* @param    i32Number    is the number whose character length is determined.
*
* @return   Returns the character length of \e i32Number.
******************************************************************************/
uint8_t
lcdGetIntLength(int32_t i32Number)
{
    uint8_t ui8NumOfDigits = 0;

    if(i32Number == 0)
    {
        //
        // The character zero also takes up one place.
        //
        return (1);
    }

    if(i32Number < 0)
    {
        //
        // Add one character to length (for minus sign)
        //
        i32Number *= (-1);
        ui8NumOfDigits++;
    }

    //
    // Count the number of digits used and return number.
    //
    while(i32Number >= 1)
    {
        i32Number /= 10;
        ui8NumOfDigits++;
    }
    return (ui8NumOfDigits);
}


/**************************************************************************//**
* @brief    This function returns the character length a float will need on the
*           LCD display. This function is used by lcdBufferPrintFloat() and
*           lcdBufferPrintFloatAligned(). \e ui8Decimals must be provided to
*           limit the number of decimals.
*
* @param    fNumber     is the number whose character length is determined.
* @param    ui8Decimals is the desired number of decimals to use (maximum 10).
*
* @return   Returns the character length of \e fNumber.
******************************************************************************/
uint8_t
lcdGetFloatLength(float fNumber, uint8_t ui8Decimals)
{
    uint8_t ui8I;
    uint8_t ui8NumOfDigits = 0;

    //
    // fThreshold defines how small a float must be to be considered negative.
    // For example, if a float is -0.001 and the number of decimals is 2,
    // then the number will be considered as 0.00 and not -0.00
    //
    float fThreshold = -0.5;
    for(ui8I = 0; ui8I < ui8Decimals; ui8I++)
    {
        fThreshold *= 0.1;
    }

    if(fNumber <= fThreshold)
    {
        //
        // Add one character for minus sign if number is negative.
        //
        ui8NumOfDigits++;

        //
        // Work only with positive part afterwards.
        //
        fNumber *= (-1);
    }

    //
    // Get character count of integer part, comma and decimal part
    //
    ui8NumOfDigits += lcdGetIntLength((int32_t)fNumber);
    if(ui8Decimals)
    {
        ui8NumOfDigits++;
    }
    ui8NumOfDigits += ui8Decimals;

    //
    // Return character count
    //
    return (ui8NumOfDigits);
}


/**************************************************************************//**
* @brief    This function writes a string to the buffer specified by
*           \e pcBuffer.
*
* @param    pcBuffer    is a pointer to the output buffer.
* @param    pcStr       is a pointer to the string to print.
* @param    ui8X        is the x-position (column) to begin printing [0--127].
* @param    iPage       is the page on which to print. Must be one of the
*                       following enumerated values:
*                       \li \b eLcdPage0
*                       \li \b eLcdPage1
*                       \li \b eLcdPage2
*                       \li \b eLcdPage3
*                       \li \b eLcdPage4
*                       \li \b eLcdPage5
*                       \li \b eLcdPage6
*                       \li \b eLcdPage7
*
* @return   None
******************************************************************************/
void
lcdBufferPrintString(char *pcBuffer, const char *pcStr, uint8_t ui8X,
                     tLcdPage iPage)
{
    uint8_t ui8I, ui8J;
    uint16_t firstIndex;
    uint8_t ui8StrSize = lcdGetStringLength(pcStr);
    uint16_t ui1FirstPos = iPage * LCD_COLS + ui8X;
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // Running through each letter in input string
    //
    for(ui8I = 0; ui8I < ui8StrSize; ui8I++)
    {
        if(pcStr[ui8I] == ' ')
        {
            //
            // Write space character
            //
            for(ui8J = 0; ui8J < LCD_CHAR_WIDTH; ui8J++)
            {
                *(pcBuf + (ui1FirstPos + LCD_CHAR_WIDTH * ui8I + ui8J)) = 0x00;
            }
        }
        else
        {
            //
            // Index to the beginning of the current letter in lcd_alphabet[]
            //
            firstIndex = ((uint16_t)(pcStr[ui8I]) - 33) * LCD_FONT_WIDTH;

            //
            // Stores each vertical column of the current letter in the result
            //
            for(ui8J = 0; ui8J < LCD_FONT_WIDTH; ui8J++)
            {
                *(pcBuf + (ui1FirstPos + LCD_CHAR_WIDTH * ui8I + ui8J)) =
                    lcd_alphabet[firstIndex + ui8J];
            }

            //
            // Add a single pixel spacing after each letter
            //
            *(pcBuf + (ui1FirstPos + LCD_CHAR_WIDTH * ui8I + LCD_FONT_WIDTH)) = 0x00;
        }
    }
}


/**************************************************************************//**
* @brief    This function writes a string to buffer \e pcBuffer as
*           specified by the \e iAlignment argument.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    pcStr       is a pointer to the string to print.
* @param    iAlignment  is the text alignment. Must be one of the following
*                       enumerated values:
*                       \li \b eLcdAlignLeft
*                       \li \b eLcdAlignCenter
*                       \li \b LCD_ALIGN_RIGHT
* @param    iPage       is the page on which to print. Must be one of the
*                       following enumerated values:
*                       \li \b eLcdPage0
*                       \li \b eLcdPage1
*                       \li \b eLcdPage2
*                       \li \b eLcdPage3
*                       \li \b eLcdPage4
*                       \li \b eLcdPage5
*                       \li \b eLcdPage6
*                       \li \b eLcdPage7
*
* @return   None
******************************************************************************/
void
lcdBufferPrintStringAligned(char *pcBuffer, const char *pcStr,
                            tLcdAlign iAlignment, tLcdPage iPage)
{
    uint8_t ui8X;
    uint8_t ui8StrSize = lcdGetStringLength(pcStr);

    //
    // Calculate X offset based on alignment
    //
    switch(iAlignment)
    {
    case eLcdAlignCenter:
        ui8X = LCD_COLS / 2 - ui8StrSize * LCD_CHAR_WIDTH / 2;
        break;
    case eLcdAlignRight:
        ui8X = LCD_COLS - ui8StrSize * LCD_CHAR_WIDTH;
        break;
    case eLcdAlignLeft:
    default:
        ui8X = 0;
        break;
    }

    //
    // Print string to buffer
    //
    lcdBufferPrintString(pcBuffer, pcStr, ui8X, iPage);
}


/**************************************************************************//**
* @brief    This function writes an integer to the buffer specified by
*           \e pcBuffer.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    i32Number    is the number to print.
* @param    ui8X        is the x-position (column) to begin printing [0--127].
* @param    iPage       is the page on which to print. Must be one of the
*                       following enumerated values:
*                       \li \b eLcdPage0
*                       \li \b eLcdPage1
*                       \li \b eLcdPage2
*                       \li \b eLcdPage3
*                       \li \b eLcdPage4
*                       \li \b eLcdPage5
*                       \li \b eLcdPage6
*                       \li \b eLcdPage7
*
* @return   None
******************************************************************************/
void
lcdBufferPrintInt(char *pcBuffer, int32_t i32Number, uint8_t ui8X,
                  tLcdPage iPage)
{
    int8_t i8I;
    uint8_t ui8J, ui8NumOfDigits;
    int32_t i32Temp, i32Digit, i32FirstIdx;
    uint16_t ui1FirstPos = iPage * LCD_COLS + ui8X;
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // I number is negative: write a minus at the first position, increment
    // position by one character and multiply number by (-1).
    //
    if(i32Number < 0)
    {
        for(ui8J = 0; ui8J < LCD_FONT_WIDTH; ui8J++)
        {
            *(pcBuf + (ui1FirstPos + ui8J)) =
                lcd_alphabet[12 * LCD_FONT_WIDTH + ui8J];
        }
        *(pcBuf + (ui1FirstPos + LCD_FONT_WIDTH)) = 0x00;   // Spacing
        ui1FirstPos += LCD_CHAR_WIDTH;
        i32Number *= (-1);
    }

    //
    // Find number of digits in i32Number (not including minus character)
    //
    ui8NumOfDigits = lcdGetIntLength(i32Number);

    //
    // For each digit (most significant first), write to buffer
    //
    for(i8I = ui8NumOfDigits - 1; i8I >= 0; i8I--)
    {
        i32Temp = i32Number / 10;
        i32Digit = i32Number - i32Temp * 10;
        i32FirstIdx = (i32Digit + 15) * LCD_FONT_WIDTH;
        for(ui8J = 0; ui8J < LCD_FONT_WIDTH; ui8J++)
        {
            *(pcBuf + (ui1FirstPos + LCD_CHAR_WIDTH * i8I + ui8J)) =
                lcd_alphabet[i32FirstIdx + ui8J];
        }

        //
        // Character spacing
        //
        *(pcBuf + (ui1FirstPos + LCD_CHAR_WIDTH * i8I + LCD_FONT_WIDTH)) = 0x00;
        i32Number = i32Temp;
    }
}


/**************************************************************************//**
* @brief    This function writes an integer to buffer \e pcBuffer as
*           specified by the \e ui8Alignment argument.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    i32Number    is the number to be printed.
* @param    iAlignment  is the text alignment. Must be one of the following
*                       enumerated values:
*                       \li \b eLcdAlignLeft
*                       \li \b eLcdAlignCenter
*                       \li \b eLcdAlignRight
* @param    iPage       is the page on which to print. Must be one of the
*                       following enumerated values:
*                       \li \b eLcdPage0
*                       \li \b eLcdPage1
*                       \li \b eLcdPage2
*                       \li \b eLcdPage3
*                       \li \b eLcdPage4
*                       \li \b eLcdPage5
*                       \li \b eLcdPage6
*                       \li \b eLcdPage7
*
* @return   None
******************************************************************************/
void
lcdBufferPrintIntAligned(char *pcBuffer, int32_t i32Number,
                         tLcdAlign iAlignment, tLcdPage iPage)
{
    uint8_t ui8X;
    uint8_t ui8StrSize = lcdGetIntLength(i32Number);

    //
    // Calculate X position based on alignment
    //
    switch(iAlignment)
    {
    case eLcdAlignCenter:
        ui8X = LCD_COLS / 2 - ui8StrSize * LCD_CHAR_WIDTH / 2;
        break;
    case eLcdAlignRight:
        ui8X = LCD_COLS - ui8StrSize * LCD_CHAR_WIDTH;
        break;
    case eLcdAlignLeft:
    default:
        ui8X = 0;
        break;
    }

    //
    // Print number to buffer
    //
    lcdBufferPrintInt(pcBuffer, i32Number, ui8X, iPage);
}


/**************************************************************************//**
* @brief    This function writes a number of data type float on the display
*           at a specified column and page. Use this function instead of
*           performing a float to c-string conversion and then using
*           lcdBufferPrintString().
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    fNumber     is the number to print.
* @param    ui8Decimals is the number of decimals to print, MAX = 10.
* @param    ui8X        is the x-position (column) to begin printing [0--127].
* @param    iPage       is the page on which to print. Must be one of the
*                       following enumerated values:
*                       \li \b eLcdPage0
*                       \li \b eLcdPage1
*                       \li \b eLcdPage2
*                       \li \b eLcdPage3
*                       \li \b eLcdPage4
*                       \li \b eLcdPage5
*                       \li \b eLcdPage6
*                       \li \b eLcdPage7
*
* @return   None
******************************************************************************/
void
lcdBufferPrintFloat(char *pcBuffer, float fNumber, uint8_t ui8Decimals,
                    uint8_t ui8X, tLcdPage iPage)
{
    uint8_t ui8I, ui8RoundUp, ui8NumNeg;
    int8_t i8J;
    int32_t i32IntPart, i32TmpInt;
    uint8_t decimalArray[11];
    float fThreshold;
    char *pcBuf = pcBuffer;

    ui8NumNeg = 0;

    //
    // Return early if number of decimals is too high
    //
    if(ui8Decimals > 10)
    {
        return;
    }

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // fThreshold defines how small a float must be to be considered negative.
    // For example, if a float is -0.001 and the number of decimals is 2,
    // then the number will be considered as 0.00, and not -0.00.
    //
    fThreshold = -0.5;
    for(ui8I = 0; ui8I < ui8Decimals; ui8I++)
    {
        fThreshold *= 0.1;
    }

    if(fNumber <= fThreshold)
    {
        fNumber *= -1;
        ui8NumNeg = 1;
    }

    //
    // Extract integer part
    //
    i32IntPart = (int32_t)fNumber;

    //
    // Storing (ui8Decimals+1) decimals in an array
    //
    for(ui8I = 0; ui8I < ui8Decimals + 1; ui8I++)
    {
        fNumber   = fNumber * 10;
        i32TmpInt = (int32_t)fNumber;
        i32TmpInt = i32TmpInt % 10;
        decimalArray[ui8I] = i32TmpInt;
    }

    //
    // Perform upwards rounding: This can correct the truncation error that
    // occurs when passing a float argument that is generated by division. ex:
    // (59/100)*100 = 58.9999961. If printing with 2 decimals, this will give
    // 59.00. This also indicates that many decimals should not be used ...
    //
    if(decimalArray[ui8Decimals] > 4)
    {
        ui8RoundUp = 1;
        for(i8J = ui8Decimals - 1; i8J >= 0; i8J--)
        {
            decimalArray[i8J] = decimalArray[i8J] + ui8RoundUp;
            if(decimalArray[i8J] == 10)
            {
                decimalArray[i8J] = 0;
            }
            else
            {
                ui8RoundUp = 0;
            }
        }
        if(ui8RoundUp == 1)
        {
            i32IntPart++;
        }
    }

    //
    // Print negative sign if applicable
    //
    if(ui8NumNeg == 1)
    {
        lcdBufferPrintString(pcBuf, "-", ui8X, iPage);
        ui8X += LCD_CHAR_WIDTH;
    }

    //
    // Print integer part
    //
    lcdBufferPrintInt(pcBuf, i32IntPart, ui8X, iPage);

    //
    // Print integer/decimal separator
    //
    ui8X += lcdGetIntLength(i32IntPart) * LCD_CHAR_WIDTH;
    lcdBufferPrintString(pcBuf, ".", ui8X, iPage);
    ui8X += LCD_CHAR_WIDTH;

    //
    // Print decimals
    //
    for(ui8I = 0; ui8I < ui8Decimals; ui8I++)
    {
        lcdBufferPrintInt(pcBuf, decimalArray[ui8I], ui8X, iPage);
        ui8X += LCD_CHAR_WIDTH;
    }
}


/**************************************************************************//**
* @brief    This function writes a float number to buffer \e pcBuffer as
*           specified by the \e iAlignment argument.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    fNumber     is the number to be printed.
* @param    ui8Decimals is the number of decimals to be printed, MAX = 10.
* @param    iAlignment  is the text alignment. Can be one of the following
*                       enumerated values:
*                       \li \b eLcdAlignLeft
*                       \li \b eLcdAlignCenter
*                       \li \b eLcdAlignRight
* @param    iPage       is the page on which to print. Must be one of the
*                       following enumerated values:
*                       \li \b eLcdPage0
*                       \li \b eLcdPage1
*                       \li \b eLcdPage2
*                       \li \b eLcdPage3
*                       \li \b eLcdPage4
*                       \li \b eLcdPage5
*                       \li \b eLcdPage6
*                       \li \b eLcdPage7
*
* @return   None
******************************************************************************/
void
lcdBufferPrintFloatAligned(char *pcBuffer, float fNumber, uint8_t ui8Decimals,
                           tLcdAlign iAlignment, tLcdPage iPage)
{
    uint8_t ui8X;
    uint8_t ui8StrSize = lcdGetFloatLength(fNumber, ui8Decimals);

    //
    // Calculate X offset based on alignment
    //
    switch(iAlignment)
    {
    case eLcdAlignCenter:
        ui8X = LCD_COLS / 2 - ui8StrSize * LCD_CHAR_WIDTH / 2;
        break;
    case eLcdAlignRight:
        ui8X = LCD_COLS - ui8StrSize * LCD_CHAR_WIDTH;
        break;
    case eLcdAlignLeft:
    default:
        ui8X = 0;
        break;
    }

    //
    // Print float to buffer
    //
    lcdBufferPrintFloat(pcBuffer, fNumber, ui8Decimals, ui8X, iPage);
}


/**************************************************************************//**
* @brief    This function draws a line in buffer \e pcBuffer from
*           (\e ui8XFrom,\e ui8YFrom) to (\e ui8XTo,\e ui8YTo). The function
*           uses Bresenham's line algorithm.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    ui8XFrom    is the start column [0--127].
* @param    ui8XTo      is the end column [0--127].
* @param    ui8YFrom    is the start row [0--63].
* @param    ui8YTo      is the end row [0--63].
*
* @return   None
******************************************************************************/
void
lcdBufferSetLine(char *pcBuffer, uint8_t ui8XFrom, uint8_t ui8YFrom,
                 uint8_t ui8XTo, uint8_t ui8YTo)
{
    //
    // Draw line
    //
    lcdBufferLine(pcBuffer, ui8XFrom, ui8YFrom, ui8XTo, ui8YTo, 1);
}


/**************************************************************************//**
* @brief    This function clears a line in buffer \e pcBuffer from
*           (\e ui8XFrom,\e ui8YFrom) to (\e ui8XTo,\e ui8YTo). The function
*           uses Bresenham's line algorithm.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    ui8XFrom    is the start column [0--127].
* @param    ui8XTo      is the end column [0--127].
* @param    ui8YFrom    is the start row [0--63].
* @param    ui8YTo      is the end row [0--63].
*
* @return   None
******************************************************************************/
void
lcdBufferClearLine(char *pcBuffer, uint8_t ui8XFrom, uint8_t ui8YFrom,
                   uint8_t ui8XTo, uint8_t ui8YTo)
{
    //
    // Clear line
    //
    lcdBufferLine(pcBuffer, ui8XFrom, ui8YFrom, ui8XTo, ui8YTo, 0);
}


/**************************************************************************//**
* @brief    This function draws a horizontal line from (\e ui8XFrom,\e ui8Y) to
*           (\e ui8XTo,\e ui8Y) into buffer \e pcBuffer.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    ui8XFrom    is the start column [0--127].
* @param    ui8XTo      is the end column [0--127].
* @param    ui8Y        is the row [0--63].
*
* @return   None
******************************************************************************/
void
lcdBufferSetHLine(char *pcBuffer, uint8_t ui8XFrom, uint8_t ui8XTo,
                  uint8_t ui8Y)
{
    uint8_t ui8I;
    uint8_t ui8Page = ui8Y / LCD_PAGE_ROWS;
    uint8_t bit  = ui8Y % LCD_PAGE_ROWS;
    uint8_t bitmask = 1 << bit;
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // Switch draw direction if ui8XTo < ui8XFrom
    //
    if(ui8XTo < ui8XFrom)
    {
        uint8_t ui8Temp = ui8XFrom;
        ui8XFrom = ui8XTo;
        ui8XTo = ui8Temp;
    }

    //
    // Draw line
    //
    for(ui8I = ui8XFrom; ui8I <= ui8XTo; ui8I++)
    {
        *(pcBuf + (ui8Page * LCD_COLS + ui8I)) |= bitmask;
    }
}


/**************************************************************************//**
* @brief    this function Clears a horizontal line from (\e ui8XFrom,\e ui8Y) to
*           (\e ui8XTo,\e ui8Y) from buffer \e pcBuffer.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    ui8XFrom     is the start column [0--127].
* @param    ui8XTo       is the end column [0--127].
* @param    ui8Y         is the row [0--63].
*
* @return   None
******************************************************************************/
void
lcdBufferClearHLine(char *pcBuffer, uint8_t ui8XFrom,
                    uint8_t ui8XTo, uint8_t ui8Y)
{
    uint8_t ui8I;
    uint8_t ui8Page = ui8Y / LCD_PAGE_ROWS;
    uint8_t ui8Bit  = ui8Y % LCD_PAGE_ROWS;
    uint8_t ui8Bitmask = 1 << ui8Bit;
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif

    //
    // Switch draw direction if ui8XTo < ui8XFrom
    //
    if(ui8XTo < ui8XFrom)
    {
        uint8_t ui8Temp = ui8XFrom;
        ui8XFrom = ui8XTo;
        ui8XTo = ui8Temp;
    }

    //
    // Clear line
    //
    for(ui8I = ui8XFrom; ui8I <= ui8XTo; ui8I++)
    {
        *(pcBuf + (ui8Page * LCD_COLS + ui8I)) &= ~ui8Bitmask;
    }
}

/**************************************************************************//**
* @brief    This function draws a vertical line from (ui8X,ui8YFrom) to
*           (ui8X,ui8YTo) into buffer \e pcBuffer.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    ui8X        is the x-position (column) of the line [0--127].
* @param    ui8YFrom    is the start row [0--63].
* @param    ui8YTo      is the end row [0--63].
*
* @return   None
******************************************************************************/
void
lcdBufferSetVLine(char *pcBuffer, uint8_t ui8X, uint8_t ui8YFrom,
                  uint8_t ui8YTo)
{
    uint8_t ui8I, ui8Pow;
    uint8_t ui8Page, ui8FirstPage, ui8LastPage;
    uint8_t ui8FirstPageMask, ui8LastPageMask;
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // Calculate first and last LCD page
    //
    ui8FirstPage = ui8YFrom / LCD_PAGE_ROWS;
    ui8LastPage = ui8YTo / LCD_PAGE_ROWS;

    //
    //  Find the bitmask to use with the first page
    //
    ui8FirstPageMask = 0xFF;
    ui8Pow = 1;
    for(ui8I = 0; ui8I < LCD_PAGE_ROWS; ui8I++)
    {
        if(ui8YFrom - ui8FirstPage * LCD_PAGE_ROWS > ui8I)
        {
            ui8FirstPageMask -= ui8Pow;
            ui8Pow *= 2;
        }
    }

    //
    // Find the bitmask to use with the last page
    //
    ui8LastPageMask = 0x00;
    ui8Pow = 1;
    for(ui8I = 0; ui8I < LCD_PAGE_ROWS; ui8I++)
    {
        if(ui8YTo - ui8LastPage * LCD_PAGE_ROWS >= ui8I)
        {
            ui8LastPageMask += ui8Pow;
            ui8Pow *= 2;
        }
    }

    //
    // Handle lines spanning over a single page
    //
    if(ui8LastPage == ui8FirstPage)
    {
        ui8FirstPageMask &= ui8LastPageMask;
        ui8LastPageMask = ui8FirstPageMask;
    }

    //
    // Draw line in buffer
    //
    *(pcBuf + (ui8FirstPage * LCD_COLS + ui8X)) |= ui8FirstPageMask;
    for(ui8Page = (ui8FirstPage + 1); ui8Page <= (ui8LastPage - 1); ui8Page++)
    {
        *(pcBuf + (ui8Page * LCD_COLS + ui8X)) |= 0xFF;
    }
    *(pcBuf + (ui8LastPage * LCD_COLS + ui8X)) |= ui8LastPageMask;
}


/**************************************************************************//**
* @brief    This function clears a vertical line from (\e ui8X,\e ui8YFrom) to
*           (\e ui8X,\e ui8YTo) from buffer specified by argument \e pcBuffer.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    ui8X        is the x-position (column) of the line [0--127].
* @param    ui8YFrom    is the start row [0--63].
* @param    ui8YTo      is the end row [0--63].
*
* @return   None
******************************************************************************/
void
lcdBufferClearVLine(char *pcBuffer, uint8_t ui8X, uint8_t ui8YFrom,
                    uint8_t ui8YTo)
{
    uint8_t ui8I, ui8Pow;
    uint8_t ui8Page, ui8FirstPage, ui8LastPage;
    uint8_t ui8FirstPageMask, ui8LastPageMask;
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // Calculate first and last LCD page
    //
    ui8FirstPage = ui8YFrom / LCD_PAGE_ROWS;
    ui8LastPage = ui8YTo / LCD_PAGE_ROWS;

    //
    // Find the bitmask to use with the first page
    //
    ui8FirstPageMask = 0xFF;
    ui8Pow = 1;
    for(ui8I = 0; ui8I < LCD_PAGE_ROWS; ui8I++)
    {
        if(ui8YFrom - ui8FirstPage * LCD_PAGE_ROWS > ui8I)
        {
            ui8FirstPageMask -= ui8Pow;
            ui8Pow *= 2;
        }
    }

    //
    // Find the bitmask to use with the last page
    //
    ui8LastPageMask = 0x00;
    ui8Pow = 1;
    for(ui8I = 0; ui8I < LCD_PAGE_ROWS; ui8I++)
    {
        if(ui8YTo - ui8LastPage * LCD_PAGE_ROWS >= ui8I)
        {
            ui8LastPageMask += ui8Pow;
            ui8Pow *= 2;
        }
    }

    //
    // Handle lines that span a single page
    //
    if(ui8LastPage == ui8FirstPage)
    {
        ui8FirstPageMask &= ui8LastPageMask;
        ui8LastPageMask = ui8FirstPageMask;
    }

    //
    // Clear line from buffer
    //
    *(pcBuf + (ui8FirstPage * LCD_COLS + ui8X)) &= ~ui8FirstPageMask;
    for(ui8Page = (ui8FirstPage + 1); ui8Page <= (ui8LastPage - 1); ui8Page++)
    {
        *(pcBuf + (ui8Page * LCD_COLS + ui8X)) &= 0x00;
    }
    *(pcBuf + (ui8LastPage * LCD_COLS + ui8X)) &= ~ui8LastPageMask;
}


/**************************************************************************//**
* @brief    This function draws a horizontal arrow from (\e ui8XFrom,\e ui8Y) to
*           (\e ui8XTo,\e ui8Y) to buffer specified by \e pcBuffer. The function
*           assumes \e ui8Y to be in the range [2--61] in order for arrowhead to
*           fit on the LCD.
*
* @param    pcBuffer    is a pointer to target buffer.
* @param    ui8XFrom    is the start column [0--127].
* @param    ui8XTo      is the end column [0--127].
* @param    ui8Y        is the the y-position (row) of the arrow [2--61].
*
* @return   None
******************************************************************************/
void
lcdBufferHArrow(char *pcBuffer, uint8_t ui8XFrom, uint8_t ui8XTo, uint8_t ui8Y)
{
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif

    if(ui8XTo > ui8XFrom)
    {
        //
        // Draw left-to-right arrow
        //
        lcdBufferSetHLine(pcBuf, ui8XFrom, ui8XTo, ui8Y);
        lcdBufferSetVLine(pcBuf, ui8XTo - 1, ui8Y - 1, ui8Y + 1);
        lcdBufferSetVLine(pcBuf, ui8XTo - 2, ui8Y - 2, ui8Y + 2);
    }
    else if(ui8XTo < ui8XFrom)
    {
        //
        // Draw right-to-left arrow
        //
        lcdBufferSetHLine(pcBuf, ui8XTo, ui8XFrom, ui8Y);
        lcdBufferSetVLine(pcBuf, ui8XTo + 1, ui8Y - 1, ui8Y + 1);
        lcdBufferSetVLine(pcBuf, ui8XTo + 2, ui8Y - 2, ui8Y + 2);
    }
}


/**************************************************************************//**
* @brief    This function draws a vertical arrow from (\e ui8X,\e ui8YFrom) to
*           (\e ui8X,\e ui8YTo) to the buffer specified by \e pcBuffer.
*           The function assumes that \e ui8X is in the range [2--125] for the
*           arrowhead to fit on the LCD.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    ui8X        is the the x-position (column) of the arrow [2--125].
* @param    ui8YFrom    is the start row [0--63].
* @param    ui8YTo      is the end row [0--63].
*
* @return   None
******************************************************************************/
void
lcdBufferVArrow(char *pcBuffer, uint8_t ui8X, uint8_t ui8YFrom, uint8_t ui8YTo)
{
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // Draw the line
    //
    lcdBufferSetVLine(pcBuf, ui8X, ui8YFrom, ui8YTo);

    //
    // Draw arrowhead
    //
    lcdBufferSetHLine(pcBuf, ui8X - 1, ui8X + 1, ui8YTo - 1);
    lcdBufferSetHLine(pcBuf, ui8X - 2, ui8X + 2, ui8YTo - 2);
}


/**************************************************************************//**
* @brief    This function sets a pixel on (\e ui8X,\e ui8Y).
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    ui8X        is the pixel x-position (column) [0--127].
* @param    ui8Y        is the pixel y-position (row) [0--63].
*
* @return   None
******************************************************************************/
void
lcdBufferSetPx(char *pcBuffer, uint8_t ui8X, uint8_t ui8Y)
{
    uint_fast8_t ui8Page = ui8Y / LCD_PAGE_ROWS;
    uint_fast8_t ui8Bitmask = 1 << (ui8Y % LCD_PAGE_ROWS);
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // Draw pixel
    //
    *(pcBuf + (ui8Page * LCD_COLS + ui8X)) |= ui8Bitmask;
}


/**************************************************************************//**
* @brief    This function clears the pixel at (\e ui8X,\e ui8Y).
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    ui8X         is the pixel x-position (column) [0--127].
* @param    ui8Y         is the pixel y-position (row) [0--63].
*
* @return   None
******************************************************************************/
void
lcdBufferClearPx(char *pcBuffer, uint8_t ui8X, uint8_t ui8Y)
{
    uint_fast8_t ui8Page = ui8Y / LCD_PAGE_ROWS;
    uint_fast8_t ui8Bitmask = 1 << (ui8Y % LCD_PAGE_ROWS);
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // Clear pixel
    //
    *(pcBuf + (ui8Page * LCD_COLS + ui8X)) &= ~ui8Bitmask;
}



/**************************************************************************//**
* @brief    This function copies the content of \e pcFromBuffer to
*           \e pcToBuffer. If either of the two arguments are 0, the default
*           buffer is used for this argument.
*
* @param    pcToBuffer      is a pointer to the destination buffer.
* @param    pcFromBuffer    is a pointer to the target buffer.
*
* @return   None
******************************************************************************/
void
lcdBufferCopy(const char *pcFromBuffer, char *pcToBuffer)
{
    char *pcTmpToBuf = pcToBuffer;
    char *pcTmpFromBuf = (char *)pcFromBuffer;
    register uint16_t i;

    //
    // If buffers are the same, do nothing
    //
    if(pcFromBuffer == pcToBuffer)
    {
        return;
    }

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcTmpFromBuf)
    {
        pcTmpFromBuf = lcdDefaultBuffer;
    }
    else if(!pcTmpToBuf)
    {
        pcTmpToBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    //
    // Copy
    //
    for(i = 0; i < LCD_BYTES; i++)
    {
        pcTmpToBuf[i] = pcTmpFromBuf[i];
    }
}


/******************************************************************************
* LOCAL FUNCTIONS
*/
/**************************************************************************//**
* @brief    Local function. Draws or clears a line from \e (ui8XFrom,ui8YFrom)
*           to \e (ui8XTo,ui8YTo). Uses Bresenham's line algorithm.
*
* @param    pcBuffer    is a pointer to the target buffer.
* @param    ui8XFrom    is the start column.
* @param    ui8XTo      is the end column.
* @param    ui8YFrom    is the start row.
* @param    ui8YTo      is the end row.
* @param    ui8Draw     indicates whether the line should be drawn or cleared;
*                       shall be eiter 1(0) to draw(clear) the line.
*
* @return   None
******************************************************************************/
static void
lcdBufferLine(char *pcBuffer, uint8_t ui8XFrom, uint8_t ui8YFrom,
              uint8_t ui8XTo, uint8_t ui8YTo, uint8_t ui8Draw)
{
    int8_t i8X, i8Y, i8DeltaY, i8DeltaX, i8D;
    int8_t i8XDir, i8YDir;
    char *pcBuf = pcBuffer;

#ifndef LCD_NO_DEFAULT_BUFFER
    //
    // Use default buffer if null pointer
    //
    if(!pcBuf)
    {
        pcBuf = lcdDefaultBuffer;
    }
#endif // LCD_NO_DEFAULT_BUFFER

    if(ui8XFrom == ui8XTo)
    {
        //
        // Vertical line
        //
        if(ui8Draw)
        {
            lcdBufferSetVLine(pcBuf, ui8XFrom, ui8YFrom, ui8YTo);
        }
        else
        {
            lcdBufferClearVLine(pcBuf, ui8XFrom, ui8YFrom, ui8YTo);
        }
    }
    else if(ui8YFrom == ui8YTo)
    {
        //
        // Horizontal line
        //
        if(ui8Draw)
        {
            lcdBufferSetHLine(pcBuf, ui8XFrom, ui8XTo, ui8YFrom);
        }
        else
        {
            lcdBufferClearHLine(pcBuf, ui8XFrom, ui8XTo, ui8YFrom);
        }
    }
    else
    {
        //
        // Diagonal Line => Bresenham's algorithm
        //

        //
        // Determine X and Y direction
        //
        i8XDir = (ui8XFrom > ui8XTo) ? -1 : 1;
        i8YDir = (ui8YFrom > ui8YTo) ? -1 : 1;

        //
        // Set start position and calculate X and Y delta
        //
        i8X = ui8XFrom;
        i8Y = ui8YFrom;
        i8DeltaY = ui8YTo - ui8YFrom;
        i8DeltaX = ui8XTo - ui8XFrom;

        //
        // Take absolute value of X and Y delta
        //
        if(i8DeltaY < 0)
        {
            i8DeltaY *= -1;
        }
        if(i8DeltaX < 0)
        {
            i8DeltaX *= -1;
        }

        //
        // Determine principal direction and draw line
        //
        if(i8DeltaX >= i8DeltaY)
        {
            i8D = (i8DeltaY << 1) - i8DeltaX;
            while(i8X != ui8XTo)
            {
                if(ui8Draw)
                {
                    lcdBufferSetPx(pcBuf, i8X, i8Y);
                }
                else
                {
                    lcdBufferClearPx(pcBuf, i8X, i8Y);
                }

                if(i8D < 0)
                {
                    i8D += (i8DeltaY << 1);
                }
                else
                {
                    i8D += ((i8DeltaY - i8DeltaX) << 1);
                    i8Y += i8YDir;
                }
                i8X += i8XDir;
            }
        }
        else
        {
            i8D = (i8DeltaX << 1) - i8DeltaY;
            while(i8Y != ui8YTo)
            {
                if(ui8Draw)
                {
                    lcdBufferSetPx(pcBuf, i8X, i8Y);
                }
                else
                {
                    lcdBufferClearPx(pcBuf, i8X, i8Y);
                }
                if(i8D < 0)
                {
                    i8D += (i8DeltaX << 1);
                }
                else
                {
                    i8D += ((i8DeltaX - i8DeltaY) << 1);
                    i8X += i8XDir;
                }
                i8Y += i8YDir;
            }
        }
    }
}


/**************************************************************************//**
* Close the Doxygen group.
* @}
******************************************************************************/
#endif // ifndef LCD_EXCLUDE
