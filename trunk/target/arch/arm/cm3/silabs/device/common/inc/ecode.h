/***************************************************************************//**
 * @file ecode.h
 * @brief Energy Aware drivers error code definitions.
 * @version 3.20.5
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2013 Energy Micro AS, http://www.energymicro.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.@n
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.@n
 * 3. This notice may not be removed or altered from any source distribution.@n
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
#ifndef __SILICON_LABS_ECODE_H__
#define __SILICON_LABS_ECODE_H__

/***************************************************************************//**
 * @addtogroup EM_Drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @brief Typedef for API function errorcode return values.
 *
 * @details
 *        Bit 24-31:  sub-system, for example emLib, eaDrivers, …@n
 *        Bit 16-23:  module, for example UART, LCD, …@n
 *        Bit 0-15:   error code, specific error code
 ******************************************************************************/
typedef uint32_t Ecode_t;

#define ECODE_EMDRV_BASE  ( 0xF0000000 )  ///< Base value for all EMDRV errorcodes.

#define ECODE_OK          ( 0 )           ///< Generic success return value.

#define ECODE_EMDRV_RTCDRV_BASE   ( ECODE_EMDRV_BASE | 0x00001000 ) ///< Base value for RTCDRV errorcodes.
#define ECODE_EMDRV_SPIDRV_BASE   ( ECODE_EMDRV_BASE | 0x00002000 ) ///< Base value for SPIDRV errorcodes.
#define ECODE_EMDRV_NVM_BASE      ( ECODE_EMDRV_BASE | 0x00003000 ) ///< Base value for NVM errorcodes.

/** @} (end addtogroup EM_Drivers) */

#endif // __SILICON_LABS_ECODE_H__
