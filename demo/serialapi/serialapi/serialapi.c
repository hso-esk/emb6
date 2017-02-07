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
 *  --- Macros ------------------------------------------------------------- *
 */

/** Maximum length of a serial API frame */
#define SERIALAPI_FRAMEBUF_MAX              255

/** Length of the input buffer */
#define SERIALAPI_RX_BUF_LEN                128
/** Length of the output buffer */
#define SERIALAPI_TX_BUF_LEN                128

/*
 *  --- Type Definitions ----------------------------------------------------*
 */

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

/**
 * Definition of a serial API command.
 */
typedef struct
{
  /** ID of the command */
  uint8_t id;

  /** callback for the command */
  fn_serialApiHndl_t cb;

} s_serialApiCmd_t;

/**
 * Context of the serial API.
 */
typedef struct
{
  /* The context has a context for each command */
  s_serialApiCmd_t cmds[e_serial_api_type_max];

} s_serialApiCtx_t;


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */

/* Callback when receiving data from lower interface */
static void _rxDataCb( void* p_data );

/* Read data from rx buffer */
static uint16_t _readRxBuf( uint8_t *p_data, uint16_t len );

/*Read function to be used by the MAC for RX. */
static size_t _serialMacRead( void *port_handle, char *frame_buffer,
    size_t frame_buffer_length );

/* Function which returns the number of bytes waiting on  input to be
 * used by the MAC for RX. */
static size_t _serialMacReadWait( void *port_handle );

/* Write function to be used by the MAC for TX. */
static size_t _serialMacWrite( void *port_handle, char *frame_buffer,
    size_t frame_buffer_length );

/**
 * Function called by the serial MAC in case an RX frame event occurred.
 * For further details have a look at the function definitions. */
static void _serialmac_rxframe_evt( void* mac_context, char* frame_buffer,
    size_t frame_buffer_length );

/**
 * Function called by the serial MAC in case an RX buffer is requested.
 * For further details have a look at the function definitions. */
static void _serialmac_rxbuf_evt( void* mac_context, char* frame_buffer,
    size_t frame_buffer_length );

/**
 * Function used to ignore incoming events from the serial MAC.
 * For further details have a look at the function definitions. */
static void _serialmac_ignore_evt( void* mac_context, char* frame_buffer,
    size_t frame_buffer_length );

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
static struct sf_serialmac_ctx* p_macCtx;

/** Output frame buffer */
static uint8_t a_frameBufTx[SERIALAPI_FRAMEBUF_MAX];
/** Pointer to the output frame buffer */
static uint8_t* p_frameBufTx;
/** Input frame buffer */
static uint8_t a_frameBufRx[SERIALAPI_FRAMEBUF_MAX];
/** Pointer to the input frame buffer */
static uint8_t* p_frameBufRx;

/** Pointer to the UART instance used for the serial MAC protocol */
static void* p_uart;

/** Input ring buffer. */
volatile uint8_t bufferRx[SERIALAPI_RX_BUF_LEN];
/** Pointer to the Rx ring buffer's current write position */
volatile uint8_t *bufferRxWrite;
/** Pointer to the Rx ring buffer's current read position */
volatile uint8_t *bufferRxRead;
/** Number of bytes in the input buffer. */
volatile uint16_t bufferRxLen;
/** Set if there was a Rx buffer overflow. */
volatile bool bufferRxOverflow;
/*
 *  --- Local Functions ---------------------------------------------------- *
 */


/**
 * \brief   Callback if new data is available on the interface.
 *
 *          This function shall be called if new data is available on the
 *          underlying interface. It puts the data into a local buffer
 *          used for later processing of the data.
 *
 * \param   p_data    Data received.
 */
static void _rxDataCb( void* p_data )
{
  EMB6_ASSERT_RET( (p_data != NULL), );

  /* get actual byte */
  uint8_t data = *((uint8_t*)p_data);

  /** Check for space in the buffer */
  if(SERIALAPI_RX_BUF_LEN > bufferRxLen)
  {
    /* write data and increase length and write pointer */
    *bufferRxWrite = data;
    bufferRxWrite++;
    bufferRxLen++;

    /** Check for an overflow of the read pointer and adjust if required. */
    if(bufferRxWrite == &bufferRx[SERIALAPI_RX_BUF_LEN])
    {
      bufferRxWrite = bufferRx;
    }
  }
  else
  {
    /* indicate overflow */
    bufferRxOverflow = true;
  }

  /* put event to the queue */
  evproc_putEvent(E_EVPROC_HEAD, EVENT_TYPE_SLIP_POLL, NULL);

}/* _rxDataCb() */


/**
 * \brief   Read data from Rx buffer.
 *
 *          This function reads data from the RX buffer if it was filled
 *          from the interface RX callback before..
 *
 * \param   p_data    Buffer to store the data.
 * \param   len       Length of the buffer.
 *
 * \return  The number of read bytes.
 */
static uint16_t _readRxBuf( uint8_t *p_data, uint16_t len )
{
  uint16_t i;

  EMB6_ASSERT_RET( (p_data != NULL), 0 );
  EMB6_ASSERT_RET( (len > 0), 0 );

  /** Read from Rx-buffer until len or size of the buffer is reached. */
  for( i = 0; (i < len) && (0 < bufferRxLen); i++)
  {
    bsp_enterCritical();

    /** Write to the specified data pointer and increase the read pointer. */
    p_data[i] = *bufferRxRead++;
    /** Decrease the number of bytes in buffer. */
    bufferRxLen--;
    /** Check for an overflow of the read pointer and adjust if required. */
    if(bufferRxRead == &bufferRx[SERIALAPI_RX_BUF_LEN])
    {
      bufferRxRead = bufferRx;
    } /* if */

    bsp_exitCritical();
  }/* for */

  return i;
} /* _readRxBuf() */


static size_t _serialMacRead( void *port_handle, char *frame_buffer,
    size_t frame_buffer_length )
{
  /* read data from buffer */
  return _readRxBuf( (uint8_t*)frame_buffer, frame_buffer_length );
}

/* Function which returns the number of bytes waiting on  input to be
 * used by the MAC for RX. */
static size_t _serialMacReadWait( void *port_handle )
{
  /* return number of bytes in the buffer */
  return bufferRxLen;
}

/* Write function to be used by the MAC for TX. */
static size_t _serialMacWrite( void *port_handle, char *frame_buffer,
    size_t frame_buffer_length )
{
  /* write data to UART */
  return bsp_uartTx( port_handle, (uint8_t*)frame_buffer, frame_buffer_length );
}


/**
 * \brief   This event is raised by the serial MAC if a frame was received.
 *
 *          Everytime a full and valid frame was received by the serial
 *          MAC it raises such an event.
 *
 * \param   mac_contect           Context the event comes from.
 * \param   frame_buffer          Buffer holding the received frame.
 * \param   frame_buffer_length   Length of the frame.
 */
static void _serialmac_rxframe_evt( void* mac_context, char* frame_buffer,
    size_t frame_buffer_length )
{
  serialapi_frameID_t id;
  size_t bufLeft = frame_buffer_length;
  uint8_t* p_data = (uint8_t*)frame_buffer;

  fn_serialApiHndl_t f_hndl = NULL;
  uint16_t hndlRet = 0;

  /* A frame has been received */
  /* check parameters */
  EMB6_ASSERT_RET( (mac_context != NULL),  );
  EMB6_ASSERT_RET( (bufLeft >= sizeof(serialapi_frameID_t)),  );

  /* Reset frame buffer pointer */
  p_frameBufTx = a_frameBufTx;

  /* get ID */
  id = *p_data;
  p_data += sizeof(serialapi_frameID_t);
  bufLeft -= sizeof(serialapi_frameID_t);

  EMB6_ASSERT_RET( id < e_serial_api_type_max,  );

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
    hndlRet = f_hndl( p_data, bufLeft, p_frameBufTx, SERIALAPI_FRAMEBUF_MAX );

  if( hndlRet )
  {
    /* send reply */
    sf_serialmac_tx_frame ( mac_context, hndlRet,
        (const char *)p_frameBufTx, hndlRet );

    sf_serialmac_hal_tx_callback( p_macCtx );
  }

}

/**
 * \brief   Called whenever a frame header was received.
 *
 *          If a frame header was received by the serial MAC it uses this
 *          function to signal the need for buffer to store the payload
 *          of the frame.
 */
static void _serialmac_rxbuf_evt( void* mac_context, char* frame_buffer,
    size_t frame_buffer_length )
{
  /* Start the receive procedure and provide a buffer for the
   * payload of the frame. */
  sf_serialmac_rx_frame( mac_context, (char*)p_frameBufRx,
      SERIALAPI_FRAMEBUF_MAX );
}

/**
 * \brief   Common function to ignore incoming events.
 *
 *          Some of the events are not required within this
 *          implementation. Therefore they can be ignored by registering
 *          this function.
 */
static void _serialmac_ignore_evt( void* mac_context, char* frame_buffer,
    size_t frame_buffer_length )
{
  /* not required */
}


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
  if( ev == EVENT_TYPE_SLIP_POLL )
  {
    /* Call RX handler from serial MAC*/
    sf_serialmac_hal_rx_callback( p_macCtx );
  }
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
  *((serialapi_frameID_t*)p_frameBufTx) = e_serial_api_type_ret;
  p_txBuf += sizeof(serialapi_frameID_t);

  /* prepare return value */
  serialapi_ret_t* p_ret = (serialapi_ret_t*)p_txBuf;
  p_txBuf += sizeof(serialapi_ret_t);

  /* (Re-)Initialize the stack */
  emb6_init( NULL, &err );
  /* the event must be re-registered */
  evproc_regCallback( EVENT_TYPE_SLIP_POLL, _event_callback );

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
  evproc_regCallback( EVENT_TYPE_SLIP_POLL, _event_callback );

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
int8_t serialApiInit( void )
{
  int8_t ret = -1;
  p_macCtx = &_macCtx;

  /* Init the Rx-buffer variables. */
  bufferRxWrite = bufferRx;
  bufferRxRead = bufferRx;
  bufferRxLen = 0U;
  bufferRxOverflow = false;

  /* set rx frame buffer pointer */
  p_frameBufRx = a_frameBufRx;

  /* get the according UART from the BSP */
  p_uart = bsp_uartInit( EN_HAL_UART_SLIP );
  EMB6_ASSERT( p_uart != NULL );

  /* register callback for RX */
  bsp_periphIRQRegister( EN_HAL_PERIPHIRQ_SLIPUART_RX, _rxDataCb, NULL );

  /* Initialize the serial MAC */
  sf_serialmac_init( p_macCtx, p_uart, _serialMacRead, _serialMacReadWait,
      _serialMacWrite, _serialmac_rxframe_evt, _serialmac_rxbuf_evt,
      _serialmac_ignore_evt, _serialmac_ignore_evt );

  /* register event */
  evproc_regCallback( EVENT_TYPE_SLIP_POLL, _event_callback );

  return ret;
} /* serialApiInit() */

