 /**
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file       sf_mcu.c
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
 * @author     STACKFORCE
 * @brief      Handles the MCU.
 */

#define __DECL_SF_MCU_API_H__
#define __DECL_SF_MCU_H__


#ifndef USE_TI_RTOS
#error Please define if TI-RTOS is in use ore not
#endif /* USE_TI_RTOS */
/*==============================================================================
                            INCLUDE FILES
==============================================================================*/
/* Standart libraries */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Stack includes */
#include "sf_mcu.h"
#include "target_conf.h"
/* Chip specific */
#include "driverlib/pwr_ctrl.h"
#include "driverlib/flash.h"
#include "driverlib/vims.h"
#include "driverlib/sys_ctrl.h"
#include "driverlib/interrupt.h"


#if !USE_TI_RTOS
/* BoardSupportPacket */
#include "bsp/srf06eb_cc26xx/drivers/source/bsp.h"
#else
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#endif /* !USE_TI_RTOS */
/*==============================================================================
                            DEFINES
==============================================================================*/

#ifndef SECTOR_SIZE
#error Please define the size of one sector
#endif /* SECTOR_SIZE */

#ifndef TARGET_NVM_START_ADDR
#error Please define TARGET_NVM_START_ADDR in target_config.h or check include order
#endif /* TARGET_NVM_START_ADDR */

#ifndef TARGET_NVM_END_ADDR
#error Please define TARGET_NVM_END_ADDR in target_config.h or check include order
#endif /* TARGET_NVM_END_ADDR */

/*==============================================================================
                            FUNCTIONS
==============================================================================*/

/*============================================================================*/
/* sf_mcu_init() */
/*============================================================================*/
bool sf_mcu_init(void)
{
  #if !USE_TI_RTOS
  /* 0.3. Initialize the device and set the mcu speed to BSP_SYS_CLK_SPD (48MHz) */
  bspInit(BSP_SYS_CLK_SPD);

  /* 0.4. Configure DCDC */
  PowerCtrlSourceSet(PWRCTRL_PWRSRC_DCDC);
  #endif /* !USE_TI_RTOS */
  return true;
} /* sf_mcu_init() */


/*============================================================================*/
/*! sf_mcu_reset() */
/*============================================================================*/
void sf_mcu_reset(void)
{
  SysCtrlSystemReset();
} /* sf_mcu_reset() */

/*============================================================================*/
/*! sf_mcu_datamemory_write() */
/*============================================================================*/
uint16_t sf_mcu_datamemory_write(uint8_t *pc_data, uint16_t i_len, uint32_t l_addr)
{
  /* Counter variable. */
  uint16_t i;
  /* Start of the segment to write into. */
  uint8_t *pc_memory;
  /* Number of written bytes */
  uint16_t i_return = 0x00U;
  /* Array to save the flash content before erasing the flash */
  static uint8_t ac_flashPage[MCU_INFOFLASH_SIZE];
  /* Address of the first segment. */
  uint32_t l_addrSegmStart;

  /* Check the input parameters */
  if((i_len > 0x00U) && (NULL != pc_data) &&
     ((l_addr + i_len) <= MCU_INFOFLASH_SIZE))
  {
    /* Find the start of the flash page. */
    l_addrSegmStart = l_addr + TARGET_NVM_START_ADDR;
    while((l_addrSegmStart % SECTOR_SIZE) != 0)
    {
      l_addrSegmStart--;
    } /* while */

    /* Calculates the start address to write in.*/
    l_addr %= SECTOR_SIZE;

    /* Disable the cache */
    VIMSModeSet(VIMS_BASE, VIMS_MODE_DISABLED);
    while(VIMSModeGet(VIMS_BASE) != VIMS_MODE_DISABLED);

    /* Check if the flash is protected�*/
    if(FlashProtectionGet(l_addr) != FLASH_WRITE_PROTECT)
    {
      /* Set the local pointer to the start of the flash page */
      pc_memory = (uint8_t*) l_addrSegmStart;

      /* Store the flash data in local array */
      for(i = 0x00U; i < MCU_INFOFLASH_SIZE; i++)
      {
        ac_flashPage[i] = pc_memory[i];
      } /* for */

      /* Updates the current flash page. */
      for(i = 0x00U; i < i_len; i++)
      {
        ac_flashPage[(l_addr + i)] = pc_data[i];
      } /* for */

      /* Erase and rewrite the flash */
      if (FlashSectorErase(l_addrSegmStart) == FAPI_STATUS_SUCCESS)
      {
        if(FlashProgram(ac_flashPage, l_addrSegmStart, MCU_INFOFLASH_SIZE) == FAPI_STATUS_SUCCESS)
        {
          i_return = i_len;
        }/* if */
      }
    }/* if  */
    /* Re-enable the cache */
    VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);
  }/* if */

  return i_return;
} /* sf_mcu_datamemory_write() */

/*============================================================================*/
/*! sf_mcu_datamemory_read() */
/*============================================================================*/
uint16_t sf_mcu_datamemory_read(uint8_t *pc_data, uint16_t i_len, uint32_t l_addr)
{
  /* Counter variable. */
  uint16_t i;
  /* Pointer to the segment to read from. */
  uint8_t *pc_memory;

  pc_memory = ((uint8_t*) (TARGET_NVM_START_ADDR)) + l_addr;

   for(i = 0x00U; i < i_len ;i++)
   {
     pc_data[i] = pc_memory[i];
   } /* for */

  return i_len;
} /* sf_mcu_datamemory_read() */


/*============================================================================*/
/*! sf_mcu_interrutpEnable() */
/*============================================================================*/
void sf_mcu_interruptEnable(void)
{
  IntMasterEnable();
}/* sf_mcu_interruptEnable() */

/*============================================================================*/
/*! sf_mcu_interrutpEnable() */
/*============================================================================*/
void sf_mcu_interruptDisable(void)
{
  IntMasterDisable();
}/* sf_mcu_interruptDisable() */
