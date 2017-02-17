/******************************************************************************
*  Filename:       rf_def.h
*  Revised:        $ $
*  Revision:       $ $
*
*  Description:    This file contains definitions for the rf driver
*
*  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

//*****************************************************************************
//
//! \addtogroup rfdef
//! @{
//
//*****************************************************************************


#ifndef RF_DEF_H
#define RF_DEF_H

#include <stdlib.h>
#include <stdint.h>

#include <rflib/rf_config.h>
#include <driverlib/rf_common_cmd.h>

#define RFC_STATIC static
#define RFC_STATIC_INLINE static inline

#define RFC_GET_OTHERS( x )             (((x) & 0x00FF) >> 0)

#define RFC_RETURN_BYTE1( x )            (((x) & 0x0000FF00) >> 8)
#define RFC_RETURN_BYTE2( x )            (((x) & 0x00FF0000) >> 16)
#define RFC_RETURN_BYTE3( x )            (((x) & 0xFF000000) >> 24)

#define rfc_cmdStatus_t uint8_t

#define RFC_CMDSTATUS_PENDING                   0x0   //!< The command has not yet been parsed
#define RFC_CMDSTATUS_DONE                      0x1   //!< Command successfully parsed
#define RFC_CMDSTATUS_ILLEGAL_POINTER           0x81  //!< The pointer signaled in CMDR is not valid
#define RFC_CMDSTATUS_UNKNOWN_COMMAND           0x82  //!< The command number in the command structure is unknown
#define RFC_CMDSTATUS_UNKNOWN_DIR_COMMAND       0x83  //!< The command number for a direct command is unknown,
                                                      //!< or the command is not a direct command
                                                      //!<
#define RFC_CMDSTATUS_CONTEXT_ERROR             0x85  //!< An immediate or direct command was issued in a context where it is not supported
#define RFC_CMDSTATUS_SCHEDULING_ERROR          0x86  //!< A radio operation command was attempted to be scheduled
                                                      //!< while another operation was already running in the RF core
                                                      //!<
#define RFC_CMDSTATUS_PAR_ERROR                 0x87  //!< There were errors in the command parameters that are parsed on submission. 
#define RFC_CMDSTATUS_QUEUE_ERROR               0x88  //!< An operation on a data entry queue was attempted that was not
                                                      //!< supported by the queue in its current state
                                                      //!<
#define RFC_CMDSTATUS_QUEUE_BUSY                0x89  //!< An operation on a data entry was attempted while that entry was busy


/* Allocated space for high level commands */
union rfc_cmds
{
  rfc_CMD_FS_POWERDOWN_t cmdFsPowerDown;
};

//*****************************************************************************
//
//! \enum rfc_returnValue_t
//!
//! \brief ReturnValue enum, possible return values from API functions
//! not returning special codes
//!
//! All the necessary details about this enumeration.
//
//*****************************************************************************
typedef enum {
  RFC_OK                                = 0, //!< Successful
  RFC_ERROR                             = 1, //!< General error
  RFC_ERROR_PATCH                       = 2, //!< Patching failed
  RFC_ERROR_SETUP                       = 3, //!< Setup command returned with error
  RFC_ERROR_RPID                        = 4, //!< Unknown rpid
  RFC_ERROR_MODE                        = 5, //!< Unsupported mode
} rfc_returnValue_t;

//*****************************************************************************
//
//! \enum rfc_RPID_t
//!
//! \brief The RPID is a 16-bit integer which combines protocols, device, versions
//  and eventually other metadata information.
//  Bits[15:8]  = Protocol
//  Bits[7:0]   = Reserved
//!
//! All the necessary details about this enumeration.
//
//*****************************************************************************
typedef enum {
  RFC_RPID_UNDEF                = 0x0000,  //!< Undefined RPID
  RFC_GFSK                      = 0x0300,  //!< Generic FSK
  RFC_BLE                       = 0x0101,  //!< BLE
  RFC_802_15_4                  = 0x0202,  //!< IEEE 802.15.4
  RFC_OOK                       = 0x0403,  //!< OOK
  RFC_LRM                       = 0x0700,  //!< LRM
  RFC_T_MODE                    = 0x0503,  //!< T_MODE
} rfc_RPID_t;

//*****************************************************************************
//
//! \struct rfc_config_t
//!
//! \brief This structure holds the internal variables of the library
//!
//! All the necessary details about this struct.
//
//*****************************************************************************
typedef struct
{
  rfc_RPID_t rpid;                       //!< Radio Protocol Identifier
  rfc_radioOp_t *currentCmd;             //!< Pointer to last sent command
  uint8_t bTx;                           //!< If in TX or RX mode
} rfc_config_t;



//*****************************************************************************
//      DEFINES 
//*****************************************************************************


#endif

//*****************************************************************************
//
//! Close the Doxygen group.
//! @}
//
//*****************************************************************************
