/*
 * --- License --------------------------------------------------------------*
 */
/*
 * emb6 is licensed under the 3-clause BSD license. This license gives everyone
 * the right to use and distribute the code, either in binary or source code
 * format, as long as the copyright license is retained in the source code.
 *
 * The emb6 is derived from the Contiki OS platform with the explicit approval
 * from Adam Dunkels. However, emb6 is made independent from the OS through the
 * removal of protothreads. In addition, APIs are made more flexible to gain
 * more adaptivity during run-time.
 *
 * The license text is:
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * --- Module Description ---------------------------------------------------*
 */
/**
 *  \file       serialapi.h
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Serial API.
 *
 *              The serial API is used to control the stack via an external
 *              interface such as UART or Ethernet. This module does not
 *              include the lower layer handling such as access to the physical
 *              interface or protocol handling.
 */
#ifndef __SERIALAPI_H__
#define __SERIALAPI_H__

/*
 * --- Includes -------------------------------------------------------------*
 */
#include <stdint.h>
#include "emb6.h"

/*
 *  --- Macros ------------------------------------------------------------- *
 */



/*
 *  --- Enumerations --------------------------------------------------------*
 */


/**
 * \brief   Serial API types.
 */
typedef enum
{
  /** General response to acknowledge a command. A device/host shall issue
    * this response for every command, in case no specific response exists.
    * This is required to see if the command was received and accepted. */
  e_serial_api_type_ret,

  /** A host can use this device to check the availability of a device
   * and vice versa. */
  e_serial_api_type_ping = 0x10,

  /** Set a configuration parameter. */
  e_serial_api_type_cfg_set = 0x20,

  /** Get a configuration parameter. */
  e_serial_api_type_cfg_get,

  /** return a configuration parameter. */
  e_serial_api_type_cfg_rsp,

  /** Initialize communication module. A host has to issue this command
    * at the beginning of the communication or to reset the device. */
  e_serial_api_type_device_init = 0x31,

  /** Start the communication module. The communication module tries to
    * access and connect to the network. */
  e_serial_api_type_device_start,

  /** Stop the communication module. Stops the operation of the
    * communication module. */
  e_serial_api_type_device_stop,

  /**
    * Get the Status of the communication module. This is required
    * for the sensor module to know when the communication is ready
    * or if an error occurred. */
  e_serial_api_type_status_get = 0x40,

  /** Returns the status of the communication module. The device
    * creates this response either when requested using the
    * STATUS_GET command or automatically if the status of the
    * communication module has changed. */
  e_serial_api_type_status_ret,

  /** Obtain the last error of the communication module. */
  e_serial_api_type_error_get = 0x50,

  /** Returns the latest error of the communication module. */
  e_serial_api_type_error_ret,

  e_serial_api_type_max

} e_serial_api_type_t;


/**
 * \brief   Return and status codes.
 */
typedef enum
{
  /** Describes a positive return value e.g. after
    * a command was issued. */
  e_serial_api_ret_ok = 0x00,

  /** A general error occurred during the operation
    * (e.g. CRC error). */
  e_serial_api_ret_error,

  /** The command is not valid or supported. */
  e_serial_api_ret_error_cmd,

  /** The parameters are invalid or not supported. */
  e_serial_api_ret_error_param,

  /** The device is in an undefined state. Usually a device enters this
    * state after power-up. The host shall initialize a device that resides
    * in this state. */
  e_serial_api_status_undef = 0x20,

  /** The initialization of a device finished properly. It is ready to start
    * its operation. */
  e_serial_api_status_init,

  /** The device started its operation. In this state, the device is usually
    * trying to access and connect to the wireless network. */
  e_serial_api_status_started,

  /** The device stopped its operation. */
  e_serial_api_status_stopped,

  /** The device has access to the network and is capable to communicate. All
    * the commands directed to higher layer shall fail if the device is not
    * within this state. */
  e_serial_api_status_network,

  /** The device is in an error state. */
  e_serial_api_status_error,

} e_serial_api_ret_t;


/**
 * \brief   Specific error codes.
 *
 *          The ERROR_GET command allows retaining a more specific error code
 *          in case a device retains in its error state STATE_ERROR.
 */
typedef enum
{
  /** No error */
  e_serial_api_error_no,

  /** Unknown error */
  e_serial_api_error_unknown,

  /** Fatal error */
  e_serial_api_error_fatal,

} e_serial_api_error_t;


/**
 * \brief   Specific configuration IDs.
 *
 *          The CFG_GET/SET command allow setting or reading the actual
 *          configuration. Therefore specific identifiers are required.
 */
typedef enum
{
  /** MAC address */
  e_serial_api_cfgid_macaddr,

  /** PAN ID */
  e_serial_api_cfgid_panid,

  /** Operation mode */
  e_serial_api_cfgid_opmode,

  /** Radio channel */
  e_serial_api_cfgid_rfch,

} e_serial_api_cfgid_t;


/*
 *  --- Type Definitions -----------------------------------------------------*
 */

/** frameID */
typedef uint8_t serialapi_frameID_t;

/** MAC address configuration*/
typedef uint8_t serialapi_cfg_macaddr_t[UIP_802154_LONGADDR_LEN];

/** PAN ID configuration */
typedef uint16_t serialapi_cfg_panid_t;

/** Operation Mode configuration */
typedef uint8_t serialapi_cfg_opmode_t;

/** Radio Channel configuration */
typedef uint8_t serialapi_cfg_rfch_t;

/** Format of a general RET response. */
typedef uint8_t serialapi_ret_t;

/** Format of a CFG GET/SET request. */
typedef uint8_t serialapi_cfg_getset_t;

/** Format of a STATUS GET response. */
typedef uint8_t serialapi_status_ret_t;


/*
 *  --- Global Functions Definition ------------------------------------------*
 */

/**
 * \brief   Initializes the serial API.
 *
 * \return  0 on success, otherwise -1
 */
int8_t serialApiInit( void );


#endif /* __SERIALAPI_H__ */

