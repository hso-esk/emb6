/*============================================================================*/
/**
 * \file    bsp.h
 *
 * \author  Tobias Neff
 *
 * \brief   Interface description for HAL/ BSP.
 *
 */
/*============================================================================*/

#ifndef __BSP_LUMS_H__
#define __BSP_LUMS_H__

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <stdint.h>
#include "bsp.h"
#include "emb6.h"
#include "targetconfig.h"

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/


/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/


/**
 * Available LCD pages (only forwarded from LCD module)
 */
typedef enum E_BSP_LCD_PAGE_T {
  /** Page 0 */
  E_BSP_LCD_PAGE_0 = 0,
  /** Page 0 */
  E_BSP_LCD_PAGE_1 = 1,
  /** Page 0 */
  E_BSP_LCD_PAGE_2 = 2,
  /** Page 0 */
  E_BSP_LCD_PAGE_3 = 3,
  /** Page 0 */
  E_BSP_LCD_PAGE_4 = 4,
  /** Page 0 */
  E_BSP_LCD_PAGE_5 = 5,
  /** Page 0 */
  E_BSP_LCD_PAGE_6 = 6,
  /** Page 0 */
  E_BSP_LCD_PAGE_7 = 7

} e_bsp_lcd_page_t;


/*============================================================================*/
/*                         STRUCTURES AND TYPEDEFS                            */
/*============================================================================*/

/** Prototype of a UART Rx callback */
typedef void (*pf_bsp_UartRxCb)( uint8_t c );

/*============================================================================*/
/**
 * @brief    Clears the LCD display. This function acts directly on the display.
 *
 * @return   None
 */
/*============================================================================*/
void bsp_lcdClear (void);


/*============================================================================*/
/**
 * @brief    This function writes a string to the buffer specified by
 *           \e pcBuffer.
 *
 * @param    pc_buffer   is a pointer to the output buffer.
 * @param    pc_str      is a pointer to the string to print.
 * @param    uc_x        is the x-position (column) to begin printing [0--127].
 * @param    e_page      is the page on which to print. Must be one of the
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
 */
/*============================================================================*/
void bsp_lcdBufferPrintString (char *pc_buffer, const char *pc_str,
    uint8_t uc_x, e_bsp_lcd_page_t e_page);


/*============================================================================*/
/**
 * @brief    This function clears the page specified by \e iPage in LCD buffer
 *           specified by \e pcBuffer.
 *
 * @param    pc_buffer   is a pointer to the target buffer.
 * @param    e_page      is the page to clear. Must be one of the
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
 */
/*============================================================================*/
void bsp_lcdBufferClearPage (char *pc_buffer, e_bsp_lcd_page_t e_page);


/*============================================================================*/
/**
 * @brief    This function sends the specified buffer to the display. The buffer
 *           size is assumed to be 1024 bytes. Passing \e pcBuffer as 0 will
 *            send the default buffer. If \b LCD_NO_DEFAULT_BUFFER is defined,
 *           passing \e pcBuffer as 0 will result in undefined behavior.
 *
 * @param    pc_buffer    is a pointer to the source buffer.
 *
 * @return   None
 */
/*============================================================================*/
void bsp_lcdSendBuffer (const char *pc_buffer);


#endif /* __BSP_LUMS_H__ */
