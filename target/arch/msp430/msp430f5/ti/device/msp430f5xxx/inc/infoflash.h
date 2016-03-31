#ifndef __INFOFLASH_H__
#define __INFOFLASH_H__

/*============================================================================*/
/**
 * \file    infoflash.h
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
#include <msp430.h>
#include <stdint.h>
#include "targetconfig.h"

/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/

/**
 * Available Infoflash segments.
 */
typedef enum E_INFOFLASH_SEG_T
{
  /** Segment A */
  E_INFOFLASH_SEG_A,
  /** Segment B */
  E_INFOFLASH_SEG_B,
  /** Segment C */
  E_INFOFLASH_SEG_C,
  /** Segment D */
  E_INFOFLASH_SEG_D,

  E_INFOFLASH_SEG_MAX,

} e_infoflash_seg_t;


/*============================================================================*/
/*                          FUNCTION PROTOTYPES                               */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief   Function initializes the info flash
 *
 *          This function initializes the info flash. This function has to be
 *          called before accessing the info flash.
 */
/*============================================================================*/
void infoflash_init( void );


/*============================================================================*/
/**
 * @brief   Write to the Info flash (maximum 128bytes per segment)
 *
 *          This function writes data to a specific segment of the info
 *          flash. Since the size of a segment is limited only 128B my be
 *          written.
 *
 * @param   e_sg      Segment to write the data to.
 * @param   puc_data  Data to write to the segment.
 * @param   ui_leng   Length of the data to write.
 *
 * @return  The length of the written bytes on success or 0 if an error occurred.
 */
/*============================================================================*/
uint16_t infoflash_write( e_infoflash_seg_t e_sg, uint8_t* puc_data,
    uint16_t ui_len );


/*============================================================================*/
/**
 * @brief    Read from the Info flash (maximum 128bytes per segment)
 *
 *          This function reads data from a specific segment of the info
 *          flash. Since the size of a segment is limited only 128B my be
 *          read.
 *
 * @param   e_sg      Segment to read the data from.
 * @param   puc_data  Buffer to read data to.
 * @param   ui_leng   Length of the data to read.
 *
 * @return  The length of the read bytes on success or 0 if an error occurred.
 */
/*============================================================================*/
uint16_t infoflash_read ( e_infoflash_seg_t e_sg, uint8_t* puc_data,
    uint16_t ui_len );


#endif /* #ifndef __INFOFLASH_H__ */
