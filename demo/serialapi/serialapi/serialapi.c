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
 *  \file       serialapi.c
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


/*
 * --- Includes -------------------------------------------------------------*
 */
#include "emb6.h"
#include "bsp.h"
#include "evproc.h"
#include "serialapi.h"
#include "sf_serialmac.h"

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


/**
 * \brief   Definition of a callback function for commands received.
 *
 *          This function is used to define a handler used for the reception
 *          and handling of a specific command.
 *
 * \param   p_cmd   Buffer containing the command payload.
 * \param   cmdLen  Length of the command.
 * \param   Buffer for the response.
 * \param   Length of the buffer.
 *
 * \return Length of the generated response.
 */
typedef uint16_t (*fn_serialApiHndl_t)( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */

/** Called by the stack in case new data was available from the RX interface.
 * For further details have a look at the function definitions. */
static void _event_callback( c_event_t ev, p_data_t data );

/** Callback function in case a PING command was received. For further
 * details have a look at the function definition.*/
static uint16_t _hndl_ping( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a CFG_GET command was received. For further
 * details have a look at the function definition.*/
static uint16_t _hndl_cfgSet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a PING command was received. For further
 * details have a look at the function definition.*/
static uint16_t _hndl_cfgGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a DEV_INIT command was received. For further
 * details have a look at the function definition.*/
static uint16_t _hndl_devInit( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a DEV_START command was received. For further
 * details have a look at the function definition.*/
static uint16_t _hndl_devStart( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a DEV_STOP command was received. For further
 * details have a look at the function definition.*/
static uint16_t _hndl_devStop( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a STATUS_GET command was received. For further
 * details have a look at the function definition.*/
static uint16_t _hndl_statusGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a ERR_GET command was received. For further
 * details have a look at the function definition.*/
static uint16_t _hndl_errGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );


/*
 *  --- Local Variables ---------------------------------------------------- *
 */

/** Serial MAC context */
static void(*_fn_tx)(uint16_t len, void* p_param) = NULL;

/** Pointer to the Tx buffer */
static uint8_t* _p_txBuf = NULL;
/** Length of the Tx buffer */
static uint16_t _txBufLen = 0;
/** Tx parameter */
static void* _p_txParam = NULL;

/*
 *  --- Local Functions ---------------------------------------------------- *
 */

/**
 * \brief   Callback function of the stack for new data on the RX interface.
 *
 *          This function is called by the stack everytime new data is
 *          available on the RX interface. This is required to separate
 *          Interrupts from regular operations.
 *
 * \param   ev    The type of the event.
 * \param   data  Extra data.
 */
void _event_callback( c_event_t ev, p_data_t data )
{
  if( ev == EVENT_TYPE_STATUS_CHANGE )
  {
    uint8_t* p_txBuf = _p_txBuf;

   /* set the according RET ID */
    *((serialapi_frameID_t*)p_txBuf) = e_serial_api_type_status_ret;
    p_txBuf += sizeof(serialapi_frameID_t);

    /* set OK return value to indicate ping response */
    serialapi_ret_t* p_ret = (serialapi_ret_t*)p_txBuf;
    p_txBuf += sizeof(serialapi_ret_t);

    switch( emb6_getStatus() )
    {
      case STACK_STATUS_IDLE:
        *p_ret = e_serial_api_status_stopped;
        break;

      case STACK_STATUS_ACTIVE:
        *p_ret = e_serial_api_status_started;
        break;

      default:
        *p_ret = e_serial_api_status_undef;
        break;
    }

    EMB6_ASSERT_RET( _fn_tx != NULL, );
    /* Call the associated Tx function with the according
     * parameter. */
    if( p_txBuf - _p_txBuf )
        /* transmit response */
        _fn_tx( (p_txBuf - _p_txBuf), _p_txParam );
  }
}


/**
 * \brief   This event is raised by the serial MAC if a frame was received.
 *
 *          Everytime a full and valid frame was received by the serial
 *          MAC it raises such an event.
 *
 * \param   p_data      Payload of the frame.
 * \param   len         Length of the frame.
 */
static int8_t _rx_data( uint8_t* p_data, uint16_t len )
{
  serialapi_frameID_t id;
  size_t bufLeft = len;
  uint8_t* p_dataPtr = (uint8_t*)p_data;

  fn_serialApiHndl_t f_hndl = NULL;
  uint16_t hndlRet = 0;

  /* A frame has been received */
  /* check parameters */
  EMB6_ASSERT_RET( (p_dataPtr != NULL), -1 );
  EMB6_ASSERT_RET( (bufLeft >= sizeof(serialapi_frameID_t)), -1 );

  /* get ID */
  id = *p_dataPtr;
  p_dataPtr += sizeof(serialapi_frameID_t);
  bufLeft -= sizeof(serialapi_frameID_t);

  EMB6_ASSERT_RET( id < e_serial_api_type_max, -1 );

  switch( id )
  {
    /* A ping was requested from the host. Reply with a simple
     * RET frame to indicate that the device is alive. */
    case e_serial_api_type_ping:
      f_hndl = _hndl_ping;
      break;

    /* A configuration value shall be set. Get the type and value
     * for the configuration and try to set it. */
    case e_serial_api_type_cfg_set:
      f_hndl = _hndl_cfgSet;
      break;

    /* A configuration value was requested. Get the type of the
     * configuration and return its value. */
    case e_serial_api_type_cfg_get:
      f_hndl = _hndl_cfgGet;
      break;

    /* Device initialization was requested. Reset all the internal
     * parameters and prepare for operation. */
    case e_serial_api_type_device_init:
      f_hndl = _hndl_devInit;
      break;

    /* Start of the device was requested. Check the given configuration
     * and start the network. */
    case e_serial_api_type_device_start:
      f_hndl = _hndl_devStart;
      break;

    /* Stop the device. */
    case e_serial_api_type_device_stop:
      f_hndl = _hndl_devStop;
      break;

    /* Get the current status of the device. */
    case e_serial_api_type_status_get:
      f_hndl = _hndl_statusGet;
      break;

    /* Get the current error state of the device. */
    case e_serial_api_type_error_get:
      f_hndl = _hndl_errGet;
      break;

    default:
      /* invalid frame type received or not handled. */
      break;
  }

  /* call the according handler */
  if( f_hndl != NULL )
    hndlRet = f_hndl( p_dataPtr, bufLeft, _p_txBuf, _txBufLen );

  if( hndlRet )
  {
      EMB6_ASSERT_RET( _fn_tx != NULL, -1 );
      /* Call the associated Tx function with the according
       * parameter. */
      _fn_tx( hndlRet, _p_txParam );
      return 0;
  }

  return -1;

}


/**
 * \brief   Callback function for PING commands.
 *
 *          This function is called whenever a PING command was received. It
 *          replies with an RET response to indicate that the device
 *          is available.
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static uint16_t _hndl_ping( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  /* A ping was requested from the host. Reply with a simple
   * RET frame to indicate that the device is alive. */

  EMB6_ASSERT_RET( p_cmd != NULL, 0 );
  EMB6_ASSERT_RET( p_rpl != NULL, 0 );
  EMB6_ASSERT_RET( rplLen >= sizeof(serialapi_frameID_t), 0 );

  uint8_t* p_txBuf = p_rpl;

  /* set the according RET ID */
  *((serialapi_frameID_t*)p_txBuf) = e_serial_api_type_ret;
  p_txBuf += sizeof(serialapi_frameID_t);
  rplLen -= sizeof(serialapi_frameID_t);

  EMB6_ASSERT_RET( rplLen >= sizeof(serialapi_ret_t), 0 );

  /* set OK return value to indicate ping response */
  *((serialapi_ret_t*)p_txBuf) = e_serial_api_ret_ok;
  p_txBuf += sizeof(serialapi_ret_t);

  return p_txBuf - p_rpl;
}


/**
 * \brief   Callback function for XXX commands.
 *
 *          This function is called whenever a XXX
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static uint16_t _hndl_cfgSet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  /* A configuration value shall be set. Get the type and value
   * for the configuration and try to set it. */

  EMB6_ASSERT_RET( p_cmd != NULL, 0 );
  EMB6_ASSERT_RET( p_rpl != NULL, 0 );

  uint8_t* p_data = p_cmd;
  uint8_t* p_txBuf = p_rpl;

  const s_ns_t* p_stack = emb6_get();
  EMB6_ASSERT_RET( (p_stack != NULL), 0 );

  /* set the according RET ID */
  *((serialapi_frameID_t*)p_txBuf) = e_serial_api_type_ret;
  p_txBuf += sizeof(serialapi_frameID_t);
  serialapi_ret_t* p_ret = (serialapi_ret_t*)p_txBuf;
  p_txBuf += sizeof(serialapi_ret_t);

  /* get the type of configuration */
  serialapi_cfg_getset_t* p_cfgset = (serialapi_cfg_getset_t*)p_data;
  p_data += sizeof(serialapi_cfg_getset_t);
  cmdLen -= sizeof(serialapi_cfg_getset_t);

  if( p_stack->status != STACK_STATUS_IDLE )
  {
    /* invalid state to set the configuration */
    *p_ret = e_serial_api_ret_error;
  }
  else
  {
    /* Check which type of configuration shall be set */
    switch( *p_cfgset )
    {
      /* MAc address shall be set */
      case e_serial_api_cfgid_macaddr:
      {
        serialapi_cfg_macaddr_t* p_cfgMac;
        EMB6_ASSERT_RET( (cmdLen >= sizeof(serialapi_cfg_macaddr_t)), 0 );

        /* get the configuration value */
        p_cfgMac = (serialapi_cfg_macaddr_t*)p_data;
        p_data += sizeof(serialapi_cfg_getset_t);
        cmdLen -= sizeof(serialapi_cfg_getset_t);

        /** Try to set the MAC Address and set the return
         * type accordingly */
        /* set the configuration */
        memcpy( mac_phy_config.mac_address, p_cfgMac, sizeof(serialapi_cfg_macaddr_t) );
        *p_ret = e_serial_api_ret_ok;

        break;
      }

      /* PAN ID shall be set */
      case e_serial_api_cfgid_panid:
      {
        serialapi_cfg_panid_t* p_cfgPan;
        EMB6_ASSERT_RET( (cmdLen >= sizeof(serialapi_cfg_panid_t)), 0 );

        /* get the configuration value */
        p_cfgPan = (serialapi_cfg_panid_t*)p_data;
        p_data += sizeof(serialapi_cfg_panid_t);
        cmdLen -= sizeof(serialapi_cfg_panid_t);

        /** Try to set the PAN ID set the return
         * type accordingly */
        /* set the configuration */
        mac_phy_config.pan_id = *p_cfgPan;
        *p_ret = e_serial_api_ret_ok;
        break;
      }

      /* Operation mode shall be set */
      case e_serial_api_cfgid_opmode:
      {
        serialapi_cfg_opmode_t* p_cfgOpM;
        EMB6_ASSERT_RET( (cmdLen >= sizeof(serialapi_cfg_opmode_t)), 0 );

        /* get the configuration value */
        p_cfgOpM = (serialapi_cfg_opmode_t*)p_data;
        p_data += sizeof(serialapi_cfg_opmode_t);
        cmdLen -= sizeof(serialapi_cfg_opmode_t);

        /** Try to set the operation mode and set the return
         * type accordingly */
        /* set the configuration */
        mac_phy_config.op_mode = *p_cfgOpM;
        *p_ret = e_serial_api_ret_ok;
        break;
      }

      /* Radio channel shall be set */
      case e_serial_api_cfgid_rfch:
      {
        serialapi_cfg_rfch_t* p_cfgRfCh;
        EMB6_ASSERT_RET( (cmdLen >= sizeof(serialapi_cfg_rfch_t)), 0 );

        /* get the configuration value */
        p_cfgRfCh = (serialapi_cfg_rfch_t*)p_data;
        p_data += sizeof(serialapi_cfg_rfch_t);
        cmdLen -= sizeof(serialapi_cfg_rfch_t);

        /** Try to set the channel and set the return
         * type accordingly */
        /* set the configuration */
        mac_phy_config.chan_num = *p_cfgRfCh;
        *p_ret = e_serial_api_ret_ok;
        break;
      }

      /* Invalid configuration value */
      default:
        *p_ret = e_serial_api_ret_error_param;
        break;
    }
  }

  return p_txBuf - p_rpl;
}


/**
 * \brief   Callback function for XXX commands.
 *
 *          This function is called whenever a XXX
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static uint16_t _hndl_cfgGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  /* A configuration value was requested. Get the type of the
 * configuration and return its value. */

  EMB6_ASSERT_RET( p_cmd != NULL, 0 );
  EMB6_ASSERT_RET( p_rpl != NULL, 0 );

  uint8_t* p_data = p_cmd;
  uint8_t* p_txBuf = p_rpl;

  serialapi_frameID_t* p_id = (serialapi_frameID_t*)p_txBuf;
  p_txBuf += sizeof(serialapi_frameID_t);
  serialapi_cfg_getset_t* p_cfgRsp = (serialapi_cfg_getset_t*)p_txBuf;
  p_txBuf += sizeof(serialapi_cfg_getset_t);

  EMB6_ASSERT_RET( (cmdLen >= sizeof(serialapi_cfg_getset_t)), 0 );

  /* get the type of configuration */
  serialapi_cfg_getset_t* p_cfgget = (serialapi_cfg_getset_t*)p_data;
  p_data += sizeof(serialapi_cfg_getset_t);
  cmdLen -= sizeof(serialapi_cfg_getset_t);

  /* set ID of the response frame */
  *p_cfgRsp = *p_cfgget;

  /* Check which type of configuration shall be set */
  switch( *p_cfgget )
  {
    /* MAc address shall be set */
    case e_serial_api_cfgid_macaddr:
    {
      /* reply with a configuration response */
      *p_id = e_serial_api_type_cfg_rsp;

      /* set the configuration value */
      serialapi_cfg_macaddr_t* p_cfgMac = (serialapi_cfg_macaddr_t*)p_txBuf;
      p_txBuf += sizeof(serialapi_cfg_macaddr_t);

      /* Copy value from configuration to frame */
      memcpy( p_cfgMac, mac_phy_config.mac_address, sizeof(serialapi_cfg_macaddr_t) );
      break;
    }

    /* PAN ID shall be set */
    case e_serial_api_cfgid_panid:
    {
      /* reply with a configuration response */
      *p_id = e_serial_api_type_cfg_rsp;

      /* set the configuration value */
      serialapi_cfg_panid_t* p_cfgPan = (serialapi_cfg_panid_t*)p_txBuf;
      p_txBuf += sizeof(serialapi_cfg_panid_t);

      /* Copy value from configuration to frame */
      *p_cfgPan = mac_phy_config.pan_id;
      break;
    }

    /* Operation mode shall be set */
    case e_serial_api_cfgid_opmode:
    {
      /* reply with a configuration response */
      *p_id = e_serial_api_type_cfg_rsp;

      /* set the configuration value */
      serialapi_cfg_opmode_t* p_cfgOpM = (serialapi_cfg_opmode_t*)p_txBuf;
      p_txBuf += sizeof(serialapi_cfg_opmode_t);

      /* Copy value from configuration to frame */
      *p_cfgOpM = mac_phy_config.op_mode;
      break;
    }

    /* Radio channel shall be set */
    case e_serial_api_cfgid_rfch:
    {
      /* reply with a configuration response */
      *p_id = e_serial_api_type_cfg_rsp;

      /* set the configuration value */
      serialapi_cfg_rfch_t* p_cfgRfCh = (serialapi_cfg_rfch_t*)p_txBuf;
      p_txBuf += sizeof(serialapi_cfg_rfch_t);

      /* Copy value from configuration to frame */
      *p_cfgRfCh = mac_phy_config.chan_num;
      break;
    }

    default:
      /* reply with an error response */
      *p_id = e_serial_api_type_ret;

      /* reset frame buffer pointer */
      p_txBuf -= sizeof(serialapi_cfg_getset_t);

      /* set error return value to indicate invalid frame */
      serialapi_ret_t* p_ret = (serialapi_ret_t*)p_txBuf;
      p_txBuf += sizeof(serialapi_ret_t);
      *p_ret = e_serial_api_ret_error_cmd;
      break;
  }

  return p_txBuf - p_rpl;
}


/**
 * \brief   Callback function for XXX commands.
 *
 *          This function is called whenever a XXX
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static uint16_t _hndl_devInit( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  /* Device initialization was requested. Reset all the internal
   * parameters and prepare for operation. */

  EMB6_ASSERT_RET( p_cmd != NULL, 0 );
  EMB6_ASSERT_RET( p_rpl != NULL, 0 );

  uint8_t* p_txBuf = p_rpl;
  e_nsErr_t err;

  /* set the according id */
  *((serialapi_frameID_t*)p_txBuf) = e_serial_api_type_ret;
  p_txBuf += sizeof(serialapi_frameID_t);

  /* prepare return value */
  serialapi_ret_t* p_ret = (serialapi_ret_t*)p_txBuf;
  p_txBuf += sizeof(serialapi_ret_t);

  /* (Re-)Initialize the stack */
  emb6_init( NULL, &err );

  /* the event must be re-registered */
  evproc_regCallback( EVENT_TYPE_STATUS_CHANGE, _event_callback );

  if(err == NETSTK_ERR_NONE )
    *p_ret = e_serial_api_ret_ok;
  else
    *p_ret = e_serial_api_ret_error;

  return p_txBuf - p_rpl;
}


/**
 * \brief   Callback function for XXX commands.
 *
 *          This function is called whenever a XXX
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static uint16_t _hndl_devStart( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  /* Start of the device was requested. Check the given configuration
   * and start the network. */

  EMB6_ASSERT_RET( p_cmd != NULL, 0 );
  EMB6_ASSERT_RET( p_rpl != NULL, 0 );

  uint8_t* p_txBuf = p_rpl;
  e_nsErr_t err;

  /* set the according RET ID */
  *((serialapi_frameID_t*)p_txBuf) = e_serial_api_type_ret;
  p_txBuf += sizeof(serialapi_frameID_t);

  /* set OK return value to indicate ping response */
  serialapi_ret_t* p_ret = (serialapi_ret_t*)p_txBuf;
  p_txBuf += sizeof(serialapi_ret_t);
  *p_ret = e_serial_api_ret_ok;

  /* start the stack */
  emb6_start( &err );
  if(err == NETSTK_ERR_NONE )
    *p_ret = e_serial_api_ret_ok;
  else
    *p_ret = e_serial_api_ret_error;

  return p_txBuf - p_rpl;

}


/**
 * \brief   Callback function for XXX commands.
 *
 *          This function is called whenever a XXX
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static uint16_t _hndl_devStop( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  /* Stop the device. */

  EMB6_ASSERT_RET( p_cmd != NULL, 0 );
  EMB6_ASSERT_RET( p_rpl != NULL, 0 );

  uint8_t* p_txBuf = p_rpl;
  e_nsErr_t err;

  /* set the according RET ID */
  *((serialapi_frameID_t*)p_txBuf) = e_serial_api_type_ret;
  p_txBuf += sizeof(serialapi_frameID_t);

  /* set OK return value to indicate ping response */
  serialapi_ret_t* p_ret = (serialapi_ret_t*)p_txBuf;
  p_txBuf += sizeof(serialapi_ret_t);
  *p_ret = e_serial_api_ret_ok;

  /* stop the stack */
  emb6_stop( &err );
  if(err == NETSTK_ERR_NONE )
    *p_ret = e_serial_api_ret_ok;
  else
    *p_ret = e_serial_api_ret_error;

  /* the event must be re-registered */
  evproc_regCallback( EVENT_TYPE_STATUS_CHANGE, _event_callback );

  return p_txBuf - p_rpl;
}


/**
 * \brief   Callback function for XXX commands.
 *
 *          This function is called whenever a XXX
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static uint16_t _hndl_statusGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  /* Get the current status of the device. */

  EMB6_ASSERT_RET( p_cmd != NULL, 0 );
  EMB6_ASSERT_RET( p_rpl != NULL, 0 );

  uint8_t* p_txBuf = p_rpl;

  const s_ns_t* p_stack = emb6_get();
  EMB6_ASSERT_RET( (p_stack != NULL), 0 );

  /* set the according RET ID */
  *((serialapi_frameID_t*)p_txBuf) = e_serial_api_type_status_ret;
  p_txBuf += sizeof(serialapi_frameID_t);

  /* set OK return value to indicate ping response */
  serialapi_ret_t* p_ret = (serialapi_ret_t*)p_txBuf;
  p_txBuf += sizeof(serialapi_ret_t);

  switch( emb6_getStatus() )
  {
    case STACK_STATUS_IDLE:
      *p_ret = e_serial_api_status_stopped;
      break;

    case STACK_STATUS_ACTIVE:
      *p_ret = e_serial_api_status_started;
      break;

    default:
      *p_ret = e_serial_api_status_undef;
      break;
  }

  return p_txBuf - p_rpl;
}


/**
 * \brief   Callback function for XXX commands.
 *
 *          This function is called whenever a XXX
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static uint16_t _hndl_errGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  /* Get the current error state of the device. */

  EMB6_ASSERT_RET( p_cmd != NULL, 0 );
  EMB6_ASSERT_RET( p_rpl != NULL, 0 );

  uint8_t* p_txBuf = p_rpl;

  /* set the according RET ID */
  *((serialapi_frameID_t*)p_txBuf) = e_serial_api_type_error_ret;
  p_txBuf += sizeof(serialapi_frameID_t);

  /* set OK return value to indicate ping response */
  serialapi_ret_t* p_ret = (serialapi_ret_t*)p_txBuf;
  p_txBuf += sizeof(serialapi_ret_t);
  *p_ret = e_serial_api_error_unknown;

  return p_txBuf - p_rpl;

}


/*
 *  --- Global Functions  ---------------------------------------------------*
 */

/*---------------------------------------------------------------------------*/
/*
* serialApiInit()
*/
int8_t serialApiInit( uint8_t* p_txBuf, uint16_t txBufLen,
        void(*fn_tx)(uint16_t len, void* p_param), void* p_txParam )

{
    int8_t ret = 0;

    /* set buffer description */
    _p_txBuf = p_txBuf;
    _txBufLen = txBufLen;
    _p_txParam = p_txParam;

    /* set Tx function */
    _fn_tx = fn_tx;

    /* register events */
    evproc_regCallback( EVENT_TYPE_STATUS_CHANGE, _event_callback );

    return ret;
} /* serialApiInit() */


/*---------------------------------------------------------------------------*/
/*
* serialApiInput()
*/
int8_t serialApiInput( uint8_t* p_data, uint16_t len, uint8_t valid )
{
    int ret = -1;

    if( valid )
    {
        /* process the frame */
        ret = _rx_data( p_data, len );
    }


    return ret;

} /* serialApiInput() */

