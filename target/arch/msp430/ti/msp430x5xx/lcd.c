/*============================================================================*/
/**
 * \file    lcd.c
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
#include "lcd.h"
#include "io.h"
#include "spi.h"

#if( TARGET_CONFIG_LCD == TRUE )

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/** Number of pixels in LCD display */
#define LCD_PIXELS              8192
/** Number of bytes needed in LCD buffer */
#define LCD_BYTES               1024
/** Number of pixel columns */
#define LCD_COLS                128
/** Number of pixel rows */
#define LCD_ROWS                64
/** Number of pages */
#define LCD_PAGES               8
/** Number of pixel rows per LCD page */
#define LCD_PAGE_ROWS           8

/*
 * The difference between LCD_CHAR_WIDTH and LCD_FONT_WIDTH
 * equals the character spacing on the LCD display.
 */

/** Space used for each character */
#define LCD_CHAR_WIDTH          6
/** The actual font character width */
#define LCD_FONT_WIDTH          5

/*============================================================================*/
/*                                MACROS                                      */
/*============================================================================*/

/** set LCD mode signal low (command) */
#define LCD_MODE_SET_CMD()      (*ps_led_pinMode.PORT->POUT &= ~ps_led_pinMode.MSK)

/** set LCD mode signal (data) */
#define LCD_MODE_SET_DATA()     (*ps_led_pinMode.PORT->POUT |= ps_led_pinMode.MSK)



/*============================================================================*/
/*                               CONSTANTS                                    */
/*============================================================================*/

/** LCD initialization command sequence */
static const char lcdInitCmd[] = {
  0x40, /* Display start line 0                    */
  0xa1, /* ADC reverse, 6 oclock viewing direction */
  0xc0, /* Normal COM0...COM63                     */
  0xa6, /* Display normal, not mirrored            */
  0xa2, /* Set Bias 1/9 (Duty 1/65)                */
  0x2f, /* Booster, Regulator and Follower On      */
  0xf8, /* Set internal Booster to 4x              */
  0x00, /*                                        */
  0x27, /* Contrast set                            */
  0x81, /*                                        */
  0x16, /* <- use value from LCD-MODULE .doc guide for better contrast (not 0x10)*/
  0xac, /* No indicator                            */
  0x00, /*                                        */
  0xaf, /* Display on                              */
  0xb0, /* Page 0 einstellen                       */
  0x10, /* High-Nibble of column address           */
  0x00  /* Low-Nibble of column address            */
};

/*============================================================================*/
/*                            LOCAL VARIABLES                                 */
/*============================================================================*/

/** external reference to the LCD alphabet */
extern const uint8_t lcd_alphabet[];

#ifndef LCD_NO_DEFAULT_BUFFER
char lcd_defaultBuffer[LCD_BYTES] = { 0 };
#endif

/** LCD MODE pin from IO module */
static s_io_pin_desc_t ps_led_pinMode = {
    &gps_io_port[TARGETCONFIG_LCD_MODE_PORT], TARGETCONFIG_LCD_MODE_PIN, TARGETCONFIG_LCD_MODE_MSK,
};

/** LCD RST pin from IO module */
static s_io_pin_desc_t ps_led_pinRst = {
    &gps_io_port[TARGETCONFIG_LCD_RST_PORT], TARGETCONFIG_LCD_RST_PIN, TARGETCONFIG_LCD_RST_MSK,
};

/** LCD PWR pin from IO module */
static s_io_pin_desc_t ps_led_pinPwr = {
    &gps_io_port[TARGETCONFIG_LCD_PWR_PORT], TARGETCONFIG_LCD_PWR_PIN, TARGETCONFIG_LCD_PWR_MSK,
};

/*============================================================================*/
/*                          LOCAL FUNCTION PROTOTYPES                         */
/*============================================================================*/

/* get length of a string */
uint8_t _lcd_getStringLength( const char *pc_str );

/* set the internal cursor of the LCD */
void _lcd_gotoXY( uint8_t uc_x, uint8_t uc_y );

/* Sends an array of of commands to the LCD */
void _lcd_sendCommand( const char *pc_cmd, uint8_t uc_len );

/* Sends an array of \e ui8Len size of data to the LCD. */
void _lcd_sendData( const char *pc_cmd, uint16_t ui_len );

/*============================================================================*/
/*                              LOCAL FUNCTIONS                               */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief    Returns the length a c-string in number of characters by looking
 *           for the end-of-string character '<tt>\\0</tt>'.
 *           Multiply by \b LCD_CHAR_WIDTH to get length in pixels.
 *
 * @param    pc_str      is the null-terminated string whose character length
 *                       is determined.
 *
 * @return   Returns length of \e pc_str
 */
/*============================================================================*/
uint8_t _lcd_getStringLength( const char *pc_str )
{
  uint_fast8_t uc_8I = 0;

  while (pc_str[uc_8I] != '\0')
  {
    uc_8I++;
  }
  return (uc_8I);
}


/*============================================================================*/
/**
 * @brief    This function sets the internal data cursor of the LCD to the
 *           location specified by \e ui8X and \e ui8Y. When data is sent to the
 *           display, data will start printing at internal cursor location.
 *
 * @param    uc_x        is the column [0--127].
 * @param    uc_y        is the page [0--7].
 *
 * @return   None
 */
/*============================================================================*/
void _lcd_gotoXY( uint8_t uc_x, uint8_t uc_y )
{
  uint8_t uc_cmd[] = { 0xB0, 0x10, 0x00 };

  /* Adding Y position, and X position (hi/lo nibble) to command array */
  uc_cmd[0] = uc_cmd[0] + uc_y;
  uc_cmd[2] = uc_cmd[2] + (uc_x & 0x0F);
  uc_cmd[1] = uc_cmd[1] + (uc_x >> 4);

  _lcd_sendCommand((char *) uc_cmd, 3);
}


/*============================================================================*/
/**
 * @brief       Sends an array of length \e ui8Len of commands to the LCD
 *              controller.
 *
 * @param       pc_cmd    The array of commands to be sent to the LCD
 * @param       uc_len   Number of elements/bytes in command
 *
 * @return      none
 */
/*============================================================================*/
void _lcd_sendCommand (const char *pc_cmd, uint8_t uc_len )
{
  /* Macro for setting LCD mode signal low (command) */
  LCD_MODE_SET_CMD();
  /* Send the commands */
  spi_spiSend(E_SPI_PORT_LCD, (char*) pc_cmd, uc_len, 0, 0, 0);
}


/*============================================================================*/
/**
 * @brief       Sends an array of \e ui8Len size of data to the LCD.
 *
 * @param       pc_cmd   The array of commands to be sent to the LCD
 * @param       ui_len   Number of elements/bytes in command
 *
 * @return      none
 */
/*============================================================================*/
void _lcd_sendData( const char *pc_cmd, uint16_t ui_len )
{
  /* Macro for setting LCD mode signal (data) */
  LCD_MODE_SET_DATA();
  /* Send the data */
  spi_spiSend(E_SPI_PORT_LCD, (char*) pc_cmd, ui_len, 0, 0, 0);
}


/*============================================================================*/
/*                              FUNCTIONS                                     */
/*============================================================================*/

/*=============================================================================
 * lcd_init()
 *============================================================================*/
void lcd_init( void )
{
  /* Enable power */
  *ps_led_pinPwr.PORT->POUT |= ps_led_pinPwr.MSK;

  /* Reset LCD by pulling Low */
  *ps_led_pinRst.PORT->POUT &= ~ps_led_pinRst.MSK;

  /* Wait ~ 100 ms (@ 16 MHz) and clear reset by pulling high */
  __delay_cycles(1600000);
  *ps_led_pinRst.PORT->POUT |= ps_led_pinRst.MSK;

  /* Send init command sequence */
  _lcd_sendCommand((char*) lcdInitCmd, sizeof(lcdInitCmd));
}/* lcd_init() */


/*=============================================================================
 * lcd_clear()
 *============================================================================*/
void lcd_clear( void )
{
  uint16_t ui_length;
  uint8_t uc_page;
  uint8_t uc_i;
  char pc_pageData[LCD_COLS];

  for (uc_i = 0; uc_i < LCD_COLS; uc_i++)
  {
    pc_pageData[uc_i] = 0x00;
  }

  /* For each page */
  for (uc_page = 0; uc_page < LCD_PAGE_ROWS; uc_page++)
  {
    /* Set pointer */
    _lcd_gotoXY(0, uc_page);

    ui_length = LCD_COLS;

    /* Tell LCD data is transferred */
    LCD_MODE_SET_DATA();

    spi_spiSend(E_SPI_PORT_LCD, pc_pageData, ui_length, 0, 0, 0);
  }
}/* lcd_clear() */


/*=============================================================================
 * lcd_bufferPrintString()
 *============================================================================*/
void lcd_bufferPrintString( char *pc_buffer, const char *pc_str, uint8_t uc_x,
    e_lcd_page_t e_page )
{

  uint8_t uc_8I, uc_8J;
  uint16_t ui_firstIndex;
  uint8_t uc_strSize = _lcd_getStringLength(pc_str);
  uint16_t ui_firstPos = e_page * LCD_COLS + uc_x;
  char *pc_buf = pc_buffer;

#ifndef LCD_NO_DEFAULT_BUFFER
  /* Use default buffer if null pointer */
  if (!pc_buf)
  {
    pc_buf = lcd_defaultBuffer;
  }
#endif /* LCD_NO_DEFAULT_BUFFER */

  /* Running through each letter in input string */
  for (uc_8I = 0; uc_8I < uc_strSize; uc_8I++)
  {
    if (pc_str[uc_8I] == ' ')
    {
      /* Write space character */
      for (uc_8J = 0; uc_8J < LCD_CHAR_WIDTH; uc_8J++)
      {
        *(pc_buf + (ui_firstPos + LCD_CHAR_WIDTH * uc_8I + uc_8J)) = 0x00;
      }
    } else
    {
      /* Index to the beginning of the current letter in lcd_alphabet[] */
      ui_firstIndex = ((uint16_t) (pc_str[uc_8I]) - 33) * LCD_FONT_WIDTH;

      /* Stores each vertical column of the current letter in the result */
      for (uc_8J = 0; uc_8J < LCD_FONT_WIDTH; uc_8J++)
      {
        *(pc_buf + (ui_firstPos + LCD_CHAR_WIDTH * uc_8I + uc_8J)) =
            lcd_alphabet[ui_firstIndex + uc_8J];
      }

      /* Add a single pixel spacing after each letter */
      *(pc_buf + (ui_firstPos + LCD_CHAR_WIDTH * uc_8I + LCD_FONT_WIDTH)) =
          0x00;
    }
  }
}/* lcd_bufferPrintString() */


/*=============================================================================
 * lcd_bufferClearPage()
 *============================================================================*/
void lcd_bufferClearPage( char *pc_buffer, e_lcd_page_t e_page )
{
  uint8_t uc_idx;
  char *pc_buf = pc_buffer;

#ifndef LCD_NO_DEFAULT_BUFFER
  /* Use default buffer if null pointer */
  if (!pc_buf)
  {
    pc_buf = lcd_defaultBuffer;
  }
#endif /* LCD_NO_DEFAULT_BUFFER */

  /* Clear page in buffer */
  for (uc_idx = 0; uc_idx < LCD_COLS; uc_idx++)
  {
    *(pc_buf + (e_page * LCD_COLS + uc_idx)) = 0x00;
  }
}/* lcd_bufferClearPage() */


/*=============================================================================
 * lcd_sendBuffer()
 *============================================================================*/
void lcd_sendBuffer( const char *pc_buffer )
{
  uint8_t uc_page;
  char *pc_buf = (char *) pc_buffer;

#ifndef LCD_NO_DEFAULT_BUFFER
  /* Use default buffer if null pointer */
  if (!pc_buf)
  {
    pc_buf = lcd_defaultBuffer;
  }
#endif /* LCD_NO_DEFAULT_BUFFER */

  /* For each page */
  for (uc_page = 0; uc_page < 8; uc_page++)
  {
    /* Set LCD pointer to start of the correct page and send data */
    _lcd_gotoXY(0, uc_page);
    _lcd_sendData(pc_buf + (uc_page * LCD_COLS), LCD_COLS);
  }
}/* lcd_sendBuffer() */

#endif /* #if( TARGET_CONFIG_LCD == TRUE ) */
