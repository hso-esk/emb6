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
#include "uip.h"


/*
 *  --- Macros --------------------------------------------------------------*
 */

#define SERIAL_API_SET_FIELD( dst, dstlen, type, src)       \
    do{                                                     \
      *((type*)dst) = src;                                  \
      dst += sizeof(type);                                  \
      dstlen -= sizeof(type);                               \
    } while (0);

#define SERIAL_API_SET_FIELD_MEM( dst, dstlen, src, size)   \
    do{                                                     \
      memcpy(dst, src, size);                               \
      dst += size;                                          \
      dstlen -= size;                                       \
    } while (0);


#define SERIAL_API_SET_FIELD_STR( dst, dstlen, str)         \
    do{                                                     \
      int len;                                              \
      len = snprintf((char*)dst, dstlen, "%s", str );       \
      dst += len;                                           \
      dstlen -= len;                                        \
    } while (0);

#define SERIAL_API_GET_FIELD( dst, src, srclen, type)       \
    do{                                                     \
      dst = *((type*)(src));                                \
      src += sizeof(type);                                  \
      srclen -= sizeof(type);                               \
    } while (0);

#define SERIAL_API_GET_FIELD_MEM( dst, src, srclen, size)   \
    do{                                                     \
      memcpy(dst, src, size);                               \
      src += size;                                          \
      srclen -= size;                                       \
    } while (0);

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

  /** A host can use this device to check the version of the device. */
  e_serial_api_type_vers_get = 0x11,

  /** Response to a version request. */
  e_serial_api_type_vers_rsp = 0x12,

  /** Set a configuration parameter. */
  e_serial_api_type_cfg_set = 0x20,

  /** Get a configuration parameter. */
  e_serial_api_type_cfg_get,

  /** return a configuration parameter. */
  e_serial_api_type_cfg_rsp,

  /** Stop the communication module. Stops the operation of the
    * communication module. */
  e_serial_api_type_device_stop = 0x30,

  /** Start the communication module. The communication module tries to
    * access and connect to the network. */
  e_serial_api_type_device_start = 0x31,

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

  /** The device stopped its operation. */
  e_serial_api_status_stopped = 0x30 ,

  /** The device started its operation. In this state, the device is usually
    * trying to access and connect to the wireless network. */
  e_serial_api_status_started,

  /** The device has access to the network and is capable to communicate. All
    * the commands directed to higher layer shall fail if the device is not
    * within this state. */
  e_serial_api_status_network,

  /** The device is in an error state. */
  e_serial_api_status_error = 0x3E,

  /** The device is in an undefined state. Usually a device enters this
    * state after power-up. The host shall initialize a device that resides
    * in this state. */
  e_serial_api_status_undef = 0x3F,


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
 * Serial API protocol context for next higher layer.
 */
typedef struct
{
  /** frame ID of the protocol */
  serialapi_frameID_t id;

  /** callback of the protocol initialization*/
  fn_serial_ApiInit_t p_finit;

  /** callback of the protocol input*/
  fn_serial_ApiInput_t p_fin;

} s_serialapi_prot_t;

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
typedef int32_t (*fn_serialApiHndl_t)( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */

/** Called by the stack in case a registered event occured.
 * For further details have a look at the function definitions. */
static void _event_callback( c_event_t ev, p_data_t data );


/** Transmit data.
 * For further details have a look at the function definitions. */
static void _txData( uint16_t len, void* p_param );

/** Receive data.
 * For further details have a look at the function definitions. */
static int8_t _rx_data( uint8_t* p_data, uint16_t len );

/** Generate a generic status response.
 * For further details have a look at the function definitions. */
static int32_t _rsp_status( uint8_t* p_rpl, uint16_t rplLen,
    e_serial_api_type_t type, e_serial_api_ret_t ret );


/** Callback function in case a PING command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_ping( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a VERSION_GET command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_vers( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a CFG_GET command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_cfgSet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a PING command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_cfgGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a DEV_START command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_devStart( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a DEV_STOP command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_devStop( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a STATUS_GET command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_statusGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a ERR_GET command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_errGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );


/*
 *  --- Local Variables ---------------------------------------------------- *
 */

/** Pointer to the TX function */
static void(*_fn_tx)(uint16_t len, void* p_param) = NULL;

/** Pointer to the Tx buffer */
static uint8_t* _p_txBuf = NULL;
/** Length of the Tx buffer */
static uint16_t _txBufLen = 0;
/** Tx parameter */
static void* _p_txParam = NULL;

static s_serialapi_prot_t protCtx;

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
  int32_t ret = 0;
  uint8_t* p_txBuf = _p_txBuf;
  uint16_t txBufLen = _txBufLen;

  if( ev == EVENT_TYPE_STATUS_CHANGE )
  {

    /* Call the status get handler */
    ret = _hndl_statusGet( NULL, 0, p_txBuf, txBufLen );
  }

  if( ret > 0 )
  {
      EMB6_ASSERT_RET( _fn_tx != NULL, );
      /* Call the associated Tx function with the according
       * parameter. */
      _fn_tx( ret, _p_txParam );
  }
}


/*
 * \brief   Callback from higher instance if data shall be transmitted.
 *
 *          This function is registered as tx functions for higher layers
 *          to transmit the actual data in the Tx buffer..
 *
 * \param   len         Length to transmit.
 * \param   p_param     Parameter registered during initialization.
 */
static void _txData( uint16_t len, void* p_param )
{
  /* set the frame ID before sending */
  serialapi_frameID_t* p_id = (serialapi_frameID_t*) p_param;
  *_p_txBuf = *p_id;

  /* transmit the frame */
  _fn_tx( (len + sizeof(serialapi_frameID_t)), NULL );

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

  int8_t ret = 0;
  serialapi_frameID_t id;
  size_t bufLeft = len;
  uint8_t* p_dataPtr = p_data;
  uint8_t* p_txBuf = _p_txBuf;
  uint16_t txBufLen = _txBufLen;

  fn_serialApiHndl_t f_hndl = NULL;

  /* A frame has been received */
  /* check parameters */
  EMB6_ASSERT_RET( (p_dataPtr != NULL), -1 );
  EMB6_ASSERT_RET( (bufLeft >= sizeof(serialapi_frameID_t)), -1 );

  /* get ID */
  id = *p_dataPtr;
  p_dataPtr += sizeof(serialapi_frameID_t);
  bufLeft -= sizeof(serialapi_frameID_t);

  switch( id )
  {
    /* A ping was requested from the host. Reply with a simple
     * RET frame to indicate that the device is alive. */
    case e_serial_api_type_ping:
      f_hndl = _hndl_ping;
      break;

    /* The version info was requested from the host. Reply with the
     * according response */
    case e_serial_api_type_vers_get:
      f_hndl = _hndl_vers;
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
      /* check if the ID was registered */
      if( (protCtx.id == id ) && (protCtx.p_fin != NULL) )
        return protCtx.p_fin( p_dataPtr, bufLeft, TRUE );
      else
        ret = -2;
      break;
  }

  /* call the according handler */
  if( f_hndl != NULL )
  {
    ret = f_hndl( (p_dataPtr), bufLeft, _p_txBuf, _txBufLen );
    if( ret > 0 )
    {
        EMB6_ASSERT_RET( _fn_tx != NULL, -1 );
        /* Call the associated Tx function with the according
         * parameter. */
        _fn_tx( ret, _p_txParam );
        ret = 0;
    }
  }

  if( ret != 0 )
  {
    /* Reply wit a return frame containing the according error code */
    EMB6_ASSERT_RET( txBufLen >= sizeof(serialapi_frameID_t), -1 );
    SERIAL_API_SET_FIELD( p_txBuf, txBufLen, serialapi_frameID_t,
        e_serial_api_type_ret );
    switch( ret )
    {
      case -2:
        EMB6_ASSERT_RET( txBufLen >= sizeof(e_serial_api_ret_t), -1 );
        SERIAL_API_SET_FIELD( p_txBuf, txBufLen, e_serial_api_ret_t,
            e_serial_api_ret_error_cmd );
        break;

      case -3:
        EMB6_ASSERT_RET( txBufLen >= sizeof(e_serial_api_ret_t), -1 );
        SERIAL_API_SET_FIELD( p_txBuf, txBufLen, e_serial_api_ret_t,
            e_serial_api_ret_error_param );
        break;

      default:
        EMB6_ASSERT_RET( txBufLen >= sizeof(e_serial_api_ret_t), -1 );
        SERIAL_API_SET_FIELD( p_txBuf, txBufLen, e_serial_api_ret_t,
            e_serial_api_ret_error );
        break;
    }

    ret = p_txBuf - _p_txBuf;
  }

  if( ret > 0 )
  {
    EMB6_ASSERT_RET( _fn_tx != NULL, -1 );
    _fn_tx( (p_txBuf - _p_txBuf), _p_txParam );
    ret = 0;
  }

  return ret;
}


/**
 * \brief   Generates a generic response.
 *
 *          This generates a generic response that is used to reply
 *          to several commands.
 *
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 * \ret     Return value to use for the response
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static int32_t _rsp_status( uint8_t* p_rpl, uint16_t rplLen,
    e_serial_api_type_t type, e_serial_api_ret_t ret )
{
  uint8_t* p_txBuf = p_rpl;

  EMB6_ASSERT_RET( p_rpl != NULL, -1 );

  /* set the according RET ID */
  EMB6_ASSERT_RET( rplLen >= sizeof(serialapi_frameID_t), -1 );
  SERIAL_API_SET_FIELD( p_txBuf, rplLen, serialapi_frameID_t,
      type );

  /* set OK return value to indicate ping response */
  EMB6_ASSERT_RET( rplLen >= sizeof(serialapi_ret_t), -1 );
  SERIAL_API_SET_FIELD( p_txBuf, rplLen, serialapi_ret_t,
      ret );

  return p_txBuf - p_rpl;
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
static int32_t _hndl_ping( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret;

  EMB6_ASSERT_RET( p_rpl != NULL, -1 );
  ret = _rsp_status( p_rpl, rplLen, e_serial_api_type_ret,
      e_serial_api_ret_ok );

  return ret;
}


/**
 * \brief   Callback function for VERSION_GET commands.
 *
 *          This function is called whenever a VERSION_GET command was received. It
 *          replies with a VERSION_RSP response to indicate that the device
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
static int32_t _hndl_vers( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret = 0;
  uint8_t* p_data = p_cmd;
  uint8_t* p_txBuf = p_rpl;

  EMB6_ASSERT_RET( p_cmd != NULL, -1 );
  EMB6_ASSERT_RET( p_rpl != NULL, -1 );

  EMB6_ASSERT_RET( rplLen >= sizeof(serialapi_frameID_t), -1 );
  SERIAL_API_SET_FIELD( p_txBuf, rplLen, serialapi_frameID_t,
      e_serial_api_type_vers_rsp );

  /* set version */
  EMB6_ASSERT_RET( rplLen >= strlen(LWM2M_DEVICE_FIRMWARE_VERSION), -1 );
  SERIAL_API_SET_FIELD_STR( p_txBuf, rplLen, LWM2M_DEVICE_FIRMWARE_VERSION);

  if( ret == 0 )
    ret = p_txBuf - p_rpl;

  return ret;
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
static int32_t _hndl_cfgSet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret = 0;
  uint8_t* p_data = p_cmd;
  const s_ns_t* p_stack = emb6_get();

  EMB6_ASSERT_RET( p_cmd != NULL, -1 );
  EMB6_ASSERT_RET( p_rpl != NULL, -1 );
  EMB6_ASSERT_RET( (p_stack != NULL), -1 );

  /* get the type of configuration */
  serialapi_cfg_getset_t cfgsetId;
  EMB6_ASSERT_RET( cmdLen >= sizeof(serialapi_cfg_getset_t), -3 );
  SERIAL_API_GET_FIELD( cfgsetId, p_data, cmdLen, serialapi_cfg_getset_t );

  EMB6_ASSERT_RET( p_stack->status == STACK_STATUS_IDLE, -1 );

  /* Check which type of configuration shall be set */
  switch( cfgsetId )
  {
    /* MAc address shall be set */
    case e_serial_api_cfgid_macaddr:
    {
      /* get the configuration value */
      EMB6_ASSERT_RET( (cmdLen >= sizeof(serialapi_cfg_macaddr_t)), -3 );
      SERIAL_API_GET_FIELD_MEM( mac_phy_config.mac_address, p_data,
          cmdLen, sizeof(serialapi_cfg_macaddr_t) );
      break;
    }

    /* PAN ID shall be set */
    case e_serial_api_cfgid_panid:
    {
      /* get the configuration value */
      EMB6_ASSERT_RET( (cmdLen >= sizeof(serialapi_cfg_panid_t)), -3 );
      SERIAL_API_GET_FIELD( mac_phy_config.pan_id, p_data,
          cmdLen, serialapi_cfg_panid_t );

      mac_phy_config.pan_id = UIP_HTONS( mac_phy_config.pan_id );
      break;
    }

    /* Operation mode shall be set */
    case e_serial_api_cfgid_opmode:
    {
      /* get the configuration value */
      EMB6_ASSERT_RET( (cmdLen >= sizeof(serialapi_cfg_opmode_t)), -3 );
      SERIAL_API_GET_FIELD( mac_phy_config.op_mode, p_data,
          cmdLen, serialapi_cfg_opmode_t );
      break;
    }

    /* Radio channel shall be set */
    case e_serial_api_cfgid_rfch:
    {
      /* get the configuration value */
      EMB6_ASSERT_RET( (cmdLen >= sizeof(serialapi_cfg_rfch_t)), -3 );
      SERIAL_API_GET_FIELD( mac_phy_config.chan_num, p_data,
          cmdLen, serialapi_cfg_rfch_t );
      break;
    }

    /* Invalid configuration value */
    default:
      ret = -3;
      break;
  }

  if( ret == 0 )
  {
    EMB6_ASSERT_RET( p_rpl != NULL, -1 );
    ret = _rsp_status( p_rpl, rplLen, e_serial_api_type_ret,
        e_serial_api_ret_ok );
  }

  return ret;
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
static int32_t _hndl_cfgGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret = 0;
  uint8_t* p_data = p_cmd;
  uint8_t* p_txBuf = p_rpl;

  EMB6_ASSERT_RET( p_cmd != NULL, -1 );
  EMB6_ASSERT_RET( p_rpl != NULL, -1 );

  EMB6_ASSERT_RET( rplLen >= sizeof(serialapi_frameID_t), -1 );
  SERIAL_API_SET_FIELD( p_txBuf, rplLen, serialapi_frameID_t,
      e_serial_api_type_cfg_rsp );

  /* get the type of configuration */
  serialapi_cfg_getset_t cfgsetId;
  EMB6_ASSERT_RET( (cmdLen >= sizeof(serialapi_cfg_getset_t)), -3 );
  SERIAL_API_GET_FIELD( cfgsetId, p_data, cmdLen, serialapi_cfg_getset_t );

  /* set ID of the response frame */
  EMB6_ASSERT_RET( rplLen >= sizeof(serialapi_cfg_getset_t), -1 );
  SERIAL_API_SET_FIELD( p_txBuf, rplLen, serialapi_cfg_getset_t,
      cfgsetId);

  /* Check which type of configuration shall be set */
  switch( cfgsetId )
  {
    /* MAc address shall be set */
    case e_serial_api_cfgid_macaddr:

      /* set the configuration value */
      EMB6_ASSERT_RET( rplLen >= sizeof(serialapi_cfg_macaddr_t), -1 );
      SERIAL_API_SET_FIELD_MEM( p_txBuf, rplLen, mac_phy_config.mac_address,
          sizeof(serialapi_cfg_macaddr_t) )

      break;

    /* PAN ID shall be set */
    case e_serial_api_cfgid_panid:

      /* set the configuration value */
      EMB6_ASSERT_RET( rplLen >= sizeof(serialapi_cfg_panid_t), -1 );
      SERIAL_API_SET_FIELD( p_txBuf, rplLen, serialapi_cfg_panid_t,
          UIP_HTONS( mac_phy_config.pan_id ) );
      break;


    /* Operation mode shall be set */
    case e_serial_api_cfgid_opmode:

      /* set the configuration value */
      EMB6_ASSERT_RET( rplLen >= sizeof(serialapi_cfg_opmode_t), -1 );
      SERIAL_API_SET_FIELD( p_txBuf, rplLen, serialapi_cfg_opmode_t,
          mac_phy_config.op_mode );
      break;

    /* Radio channel shall be set */
    case e_serial_api_cfgid_rfch:

      /* set the configuration value */
      EMB6_ASSERT_RET( rplLen >= sizeof(serialapi_cfg_rfch_t), -1 );
      SERIAL_API_SET_FIELD( p_txBuf, rplLen, serialapi_cfg_rfch_t,
          mac_phy_config.chan_num );
      break;

    default:
      ret = -3;
      break;
  }

  if( ret == 0 )
    ret = p_txBuf - p_rpl;

  return ret;
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
static int32_t _hndl_devStart( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret;

  EMB6_ASSERT_RET( p_rpl != NULL, -1 );
  ret = _rsp_status( p_rpl, rplLen, e_serial_api_type_ret,
      e_serial_api_ret_ok );

  /* put event to the queue */
  evproc_putEvent(E_EVPROC_HEAD, EVENT_TYPE_REQ_START, NULL);
  return ret;

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
static int32_t _hndl_devStop( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret;

  EMB6_ASSERT_RET( p_rpl != NULL, -1 );
  ret = _rsp_status( p_rpl, rplLen, e_serial_api_type_ret,
      e_serial_api_ret_ok );

  /* put event to the queue */
  evproc_putEvent(E_EVPROC_HEAD, EVENT_TYPE_REQ_STOP, NULL);
  return ret;
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
static int32_t _hndl_statusGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret;
  e_serial_api_ret_t e_ret;
  const s_ns_t* p_stack = emb6_get();

  EMB6_ASSERT_RET( p_rpl != NULL, 0 );
  EMB6_ASSERT_RET( (p_stack != NULL), -1 );

  switch( emb6_getStatus() )
  {
    case STACK_STATUS_IDLE:
      e_ret = e_serial_api_status_stopped;
      break;

    case STACK_STATUS_ACTIVE:
      e_ret = e_serial_api_status_started;
      break;

    case STACK_STATUS_NETWORK:
      e_ret = e_serial_api_status_network;
      break;

    default:
      e_ret = e_serial_api_status_undef;
      break;
  }

  /* set the according status */
  EMB6_ASSERT_RET( p_rpl != NULL, -1 );
  ret = _rsp_status( p_rpl, rplLen, e_serial_api_type_status_ret,
      e_ret );

  return ret;
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
static int32_t _hndl_errGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  uint8_t* p_txBuf = p_rpl;

  EMB6_ASSERT_RET( p_rpl != NULL, 0 );

  /* set the according frame id of the reply */
  EMB6_ASSERT_RET( rplLen >= sizeof(serialapi_frameID_t), -1 );
  SERIAL_API_SET_FIELD( p_txBuf, rplLen, serialapi_frameID_t,
      e_serial_api_type_error_ret );

  /* set the according error */
  EMB6_ASSERT_RET( rplLen >= sizeof(serialapi_ret_t), -1 );
  SERIAL_API_SET_FIELD( p_txBuf, rplLen, serialapi_ret_t,
      e_serial_api_error_no );

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

    /* reset protocol index */
    protCtx.id = 0;
    protCtx.p_fin = NULL;

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
    uint8_t* p_txBuf = _p_txBuf;
    uint16_t txBufLen = _txBufLen;

    if( valid )
    {
        /* process the frame */
        ret = _rx_data( p_data, len );
    }

    if( ret != 0 )
    {

      /* set the according frame id of the reply  */
      EMB6_ASSERT_RET( txBufLen >= sizeof(serialapi_frameID_t), -1 );
      SERIAL_API_SET_FIELD( p_txBuf, txBufLen, serialapi_frameID_t,
          e_serial_api_type_status_ret );

      /* A general error occurred */
      EMB6_ASSERT_RET( txBufLen >= sizeof(serialapi_frameID_t), -1 );
      SERIAL_API_SET_FIELD( p_txBuf, txBufLen, serialapi_frameID_t,
          e_serial_api_ret_error );

      EMB6_ASSERT_RET( _fn_tx != NULL, -1 );
      _fn_tx( (p_txBuf - _p_txBuf), _p_txParam );
      ret = 0;
    }

    return ret;
} /* serialApiInput() */


/*---------------------------------------------------------------------------*/
/*
* serialApiInput()
*/
int8_t serialApiRegister( serialapi_frameID_t id,
        fn_serial_ApiInit_t pf_init, fn_serial_ApiInput_t pf_in )
{
    int ret = -1;

    EMB6_ASSERT_RET( (pf_init != NULL), -1 );
    EMB6_ASSERT_RET( (pf_in != NULL), -1 );

    /* register protocol handler */
    protCtx.id = id;
    protCtx.p_finit = pf_init;
    protCtx.p_fin = pf_in;

    /* initialize the upper protocol */
    protCtx.p_finit( (_p_txBuf + sizeof(serialapi_frameID_t)),
        (_txBufLen - sizeof(serialapi_frameID_t)), _txData, &protCtx.id );

    ret = 0;
    return ret;
} /* serialApiInput() */

