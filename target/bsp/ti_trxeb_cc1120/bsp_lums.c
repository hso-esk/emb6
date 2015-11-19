/*============================================================================*/
/**
 * \file    bsp.c
 *
 * \author  Tobias Neff
 *
 * \brief
 *
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <stdlib.h>
#include <stdint.h>
#include <msp430.h>
#include "targetconfig.h"
#include "bsp_lums.h"
#include "lcd.h"


/*============================================================================*/
/*                          LOCAL FUNCTION PROTOTYPES                         */
/*============================================================================*/


/*============================================================================*/
/*                              LOCAL VARIABLES                               */
/*============================================================================*/

/*============================================================================*/
/*                              LOCAL FUNCTIONS                               */
/*============================================================================*/

/*============================================================================*/
/*                                FUNCTIONS                                   */
/*============================================================================*/


/*=============================================================================
 * bsp_lcdClear()
 *============================================================================*/
void bsp_lcdClear (void)
{
#if( TARGET_CONFIG_LCD == TRUE )
  lcd_clear();
#endif /* #if( (TARGET_CONFIG_UART0 == TRUE) */
} /* bsp_lcdClear() */


/*=============================================================================
 * bsp_lcdBufferPrintString()
 *============================================================================*/
void bsp_lcdBufferPrintString (char *pc_buffer, const char *pc_str,
    uint8_t uc_x, e_bsp_lcd_page_t e_page)
{
#if( TARGET_CONFIG_LCD == TRUE )
  lcd_bufferPrintString(pc_buffer, pc_str, uc_x, (e_lcd_page_t)e_page);
#endif /* #if( TARGET_CONFIG_UART0 == TRUE ) */
} /* bsp_lcdBufferPrintString() */


/*=============================================================================
 * bsp_lcdBufferClearPage()
 *============================================================================*/
void bsp_lcdBufferClearPage (char *pc_buffer, e_bsp_lcd_page_t e_page)
{
#if( TARGET_CONFIG_LCD == TRUE )
  lcd_bufferClearPage(pc_buffer, (e_lcd_page_t)e_page);
#endif /* #if( TARGET_CONFIG_UART0 == TRUE ) */
} /* bsp_lcdBufferClearPage() */


/*=============================================================================
 * bsp_lcdSendBuffer()
 *============================================================================*/
void bsp_lcdSendBuffer (const char *pc_buffer)
{
#if( TARGET_CONFIG_LCD == TRUE )
  lcd_sendBuffer(pc_buffer);
#endif /* #if( (ARGET_CONFIG_UART0 == TRUE ) */
} /* bsp_lcdSendBuffer() */
