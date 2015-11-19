/*============================================================================*/
/**
 * \file    infoflash.c
 *
 * \author  Manuel Schappacher
 *
 * \brief   Info Flash
 *
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <stdint.h>
#include <msp430.h>
#include "targetconfig.h"
#include "infoflash.h"


/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/** Size of an info flash segment */
#define INFOFLASH_SEG_SIZE                    0x80

/** Start address of the info flash segment D */
#define INFOFLASH_SEGD_ADDR_START             0x1800
/** End address of the info flash segment D */
#define INFOFLASH_SEGD_ADDR_END               (INFOFLASH_SEGD_ADDR_START + (INFOFLASH_SEGD_SIZE-1))

/** Start address of the info flash segment C */
#define INFOFLASH_SEGC_ADDR_START             0x1880
/** End address of the info flash segment C */
#define INFOFLASH_SEGC_ADDR_END               (INFOFLASH_SEGC_ADDR_START + (INFOFLASH_SEG_SIZE-1))

/** Start address of the info flash segment B */
#define INFOFLASH_SEGB_ADDR_START             0x1900
/** End address of the info flash segment B */
#define INFOFLASH_SEGB_ADDR_END               (INFOFLASH_SEGB_ADDR_START + (INFOFLASH_SEG_SIZE-1))

/** Start address of the info flash segment A */
#define INFOFLASH_SEGA_ADDR_START             0x1980
/** End address of the info flash segment A */
#define INFOFLASH_SEGA_ADDR_END               (INFOFLASH_SEGA_ADDR_START + (INFOFLASH_SEG_SIZE-1))


/*============================================================================*/
/*                              FUNCTIONS                                     */
/*============================================================================*/

/*=============================================================================
 *  infoflash_init()
 *============================================================================*/
void infoflash_init (void)
{
  /* nothing to be done here */

} /* infoflash_init() */


/*=============================================================================
 *  infoflash_write()
 *============================================================================*/
uint16_t infoflash_write ( e_infoflash_seg_t e_sg, uint8_t* puc_data,
    uint16_t ui_len )
{
  uint16_t ui_ret = 0;
  int i = 0;

  /* check for valid parameters of the segments and the length of the
   data to write */
  if( (e_sg < E_INFOFLASH_SEG_MAX) && (ui_len <= INFOFLASH_SEG_SIZE) )
  {
    uint8_t* puc_dst;

    switch( e_sg )
    {
      case E_INFOFLASH_SEG_A:
        puc_dst = (uint8_t*)INFOFLASH_SEGD_ADDR_START;
        break;

      case E_INFOFLASH_SEG_B:
        puc_dst = (uint8_t*)INFOFLASH_SEGD_ADDR_START;
        break;

      case E_INFOFLASH_SEG_C:
        puc_dst = (uint8_t*)INFOFLASH_SEGD_ADDR_START;
        break;

      case E_INFOFLASH_SEG_D:
        puc_dst = (uint8_t*)INFOFLASH_SEGD_ADDR_START;
        break;

      default:
        break;
    }

    /* disable interrupts */
    __disable_interrupt();

    /* Clear Lock bit */
    FCTL3 = FWKEY;
    /* Set Erase bit  */
    FCTL1 = FWKEY+ERASE;
    /* Dummy write to erase Flash seg  */
    *(unsigned int *)puc_dst = 0;
    /* Set WRT bit for write operation  */
    FCTL1 = FWKEY+WRT;

    /* Write value to flash */
    for (i = 0; i < ui_len; i++)
      *puc_dst++ = *puc_data++;

    /* Clear WRT bit */
    FCTL1 = FWKEY;
    /* Set LOCK bit */
    FCTL3 = FWKEY+LOCK;

    /* enable interrupts */
    __enable_interrupt();

    ui_ret = ui_len;

  }
  return ui_ret;
} /* infoflash_write() */


/*=============================================================================
 *  infoflash_read()
 *============================================================================*/
uint16_t infoflash_read ( e_infoflash_seg_t e_sg, uint8_t* puc_data,
    uint16_t ui_len )
{
  uint16_t ui_ret = 0;
  int i = 0;

  /* check for valid parameters of the segments and the length of the
   data to write */
  if( (e_sg < E_INFOFLASH_SEG_MAX) && (ui_len <= INFOFLASH_SEG_SIZE) )
  {
    uint8_t* puc_src;

    switch( e_sg )
    {
      case E_INFOFLASH_SEG_A:
        puc_src = (uint8_t*)INFOFLASH_SEGD_ADDR_START;
        break;

      case E_INFOFLASH_SEG_B:
        puc_src = (uint8_t*)INFOFLASH_SEGD_ADDR_START;
        break;

      case E_INFOFLASH_SEG_C:
        puc_src = (uint8_t*)INFOFLASH_SEGD_ADDR_START;
        break;

      case E_INFOFLASH_SEG_D:
        puc_src = (uint8_t*)INFOFLASH_SEGD_ADDR_START;
        break;

      default:
        break;
    }

    /* disable interrupts */
    __disable_interrupt();

    /* Write value to flash */
    for (i = 0; i < ui_len; i++)
      *puc_data++ = *puc_src++;

    /* enable interrupts */
    __enable_interrupt();
  }
  return ui_ret;
} /* infoflash_read() */
