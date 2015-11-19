#ifndef __LCD_H__
#define __LCD_H__

/*============================================================================*/
/**
 * \file    lcd.h
 *
 * \author  Tobias Neff
 *
 * \brief   LCD functions.
 *
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <msp430.h>
#include <stdint.h>
#include "targetconfig.h"

/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/

/**
 * Available LCD pages
 */
typedef enum E_LCD_PAGE_T {
  /** Page 0 */
  E_LCD_PAGE_0 = 0,
  /** Page 0 */
  E_LCD_PAGE_1 = 1,
  /** Page 0 */
  E_LCD_PAGE_2 = 2,
  /** Page 0 */
  E_LCD_PAGE_3 = 3,
  /** Page 0 */
  E_LCD_PAGE_4 = 4,
  /** Page 0 */
  E_LCD_PAGE_5 = 5,
  /** Page 0 */
  E_LCD_PAGE_6 = 6,
  /** Page 0 */
  E_LCD_PAGE_7 = 7

} e_lcd_page_t;

/*============================================================================*/
/*                          FUNCTION PROTOTYPES                               */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief    Initializes the LCD. Must be run after SPI initialization.
 */
/*============================================================================*/
void lcd_init( void );


/*============================================================================*/
/**
 * @brief    Clears the LCD display. This function acts directly on the display.
 */
/*============================================================================*/
void lcd_clear( void );


/*============================================================================*/
/**
 * @brief    This function writes a string to the buffer.
 *
 * @param    pc_buffer   is a pointer to the output buffer.
 * @param    pc_str      is a pointer to the string to print.
 * @param    uc_x        is the x-position (column) to begin printing [0--127].
 * @param    e_page      is the page on which to print. Must be one of the
 *                       following enumerated values:
 *                       \li \b E_LCD_PAGE_0
 *                       \li \b E_LCD_PAGE_1
 *                       \li \b E_LCD_PAGE_2
 *                       \li \b E_LCD_PAGE_3
 *                       \li \b E_LCD_PAGE_4
 *                       \li \b E_LCD_PAGE_5
 *                       \li \b E_LCD_PAGE_6
 *                       \li \b E_LCD_PAGE_7
 */
/*============================================================================*/
void lcd_bufferPrintString( char *pc_buffer, const char *pc_str, uint8_t uc_x,
    e_lcd_page_t e_page );


/*============================================================================*/
/**
 * @brief    This function clears the page specified by \e iPage in LCD buffer
 *           specified by \e pcBuffer.
 *
 * @param    pc_buffer   is a pointer to the target buffer.
 * @param    e_page      is the page to clear. Must be one of the
 *                       following enumerated values:
 *                       \li \b E_LCD_PAGE_0
 *                       \li \b E_LCD_PAGE_1
 *                       \li \b E_LCD_PAGE_2
 *                       \li \b E_LCD_PAGE_3
 *                       \li \b E_LCD_PAGE_4
 *                       \li \b E_LCD_PAGE_5
 *                       \li \b E_LCD_PAGE_6
 *                       \li \b E_LCD_PAGE_7
 */
/*============================================================================*/
void lcd_bufferClearPage( char *pc_buffer, e_lcd_page_t e_page );


/*============================================================================*/
/**
 * @brief    This function sends the specified buffer to the display.
 *
 *           The buffer size is assumed to be 1024 bytes. Passing \e pcBuffer
 *           as 0 will send the default buffer. If \b LCD_NO_DEFAULT_BUFFER is
 *           defined, passing \e pcBuffer as 0 will result in undefined behavior.
 *
 * @param    pc_buffer    is a pointer to the source buffer.
 */
/*============================================================================*/
void lcd_sendBuffer( const char *pc_buffer );

#endif /* #ifndef __LCD_H__ */
