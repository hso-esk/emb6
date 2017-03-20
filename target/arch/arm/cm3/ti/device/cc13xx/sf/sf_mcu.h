 /**
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file       sf_mcu.h
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
 * @author     STACKFORCE
 * @brief      API to the MCU interface.
 */

#ifndef __SF_MCU_H__
#define __SF_MCU_H__

#ifndef __DECL_SF_MCU_H__
#define __DECL_SF_MCU_H__ extern
#else
#define __MCU_SF_INIT_VAR__
#endif /* __DECL_SF_MCU_H__ */

/*! @defgroup sf_mcu sf_mcu driver
    This group describes the STACKFORCE mcu driver.
  @{  */

/*==============================================================================
                            INCLUDE FILES
==============================================================================*/

/*==============================================================================
                            DEFINES
==============================================================================*/
/*========================= FLASH ============================================*/
/*! Calculate the size of the nvm space available for the device. Note that the
   available space and the SECTOR_SIZE is not the same. To save RAM the
   available space is normally smaler than a flash page! */
#define MCU_INFOFLASH_SIZE         (TARGET_NVM_END_ADDR - TARGET_NVM_START_ADDR)

/* Currently it is only supported to write on more than one page so this check
   is necessary */
#if (MCU_INFOFLASH_SIZE > SECTOR_SIZE)
#error Currently we do not support more than one flash page
#endif /* (MCU_INFOFLASH_SIZE > SECTOR_SIZE) */
/*==============================================================================
                            FUNCTIONS
==============================================================================*/

/*============================================================================*/
/*!
 * @brief  Initializes the MCU.
 * @return Returns @ref true if installation was successful @ref false otherwise.
*/
/*============================================================================*/
bool sf_mcu_init(void);

/*============================================================================*/
/*!
 * @brief  Writes bytes into the data memory.
 * @param  l_addr       Start address of the memory to write into.
 * @param  *pc_data     Bytes to write into the data memory.
 * @param  i_len        Number of bytes to write.
 * @return Returns the number of bytes written.
 */
/*============================================================================*/
uint16_t sf_mcu_datamemory_write(uint8_t *pc_data, uint16_t i_len, uint32_t l_addr);

/*============================================================================*/
/*!
 * @brief  Reads bytes from the data memory.
 * @param  l_addr   Address of the memory to read out.
 * @param  *pc_data Memory to write the read data into.
 * @param  i_len    Number of bytes to read.
 * @return Returns the number of bytes read.
 */
/*============================================================================*/
uint16_t sf_mcu_datamemory_read(uint8_t *pc_data, uint16_t i_len, uint32_t l_addr);

/*============================================================================*/
/*!
 * @brief  Resets the software.
 */
/*============================================================================*/
void sf_mcu_reset(void);

/*============================================================================*/
/*!
  * @brief  Enable the interrupts.
*/
/*============================================================================*/
void sf_mcu_interruptEnable(void);

/*============================================================================*/
/*!
  * @brief  Disable the interrupts.
*/
/*============================================================================*/
void sf_mcu_interruptDisable(void);

/*! @} sf_mcu */
#endif /* __SF_MCU_H__ */
