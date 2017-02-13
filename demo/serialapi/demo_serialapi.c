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
 *  \file       demo_niki.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Demo to show how to use the simple UDP socket interface.
 *
 *              This Demo shows how to use the simple UDP socket interface. Therefore
 *              the demo is divided into a server and a client application. The
 *              client transmits data periodically to the server including a
 *              fixed payload and a sequence counter. The server replies with
 *              the same sequence number.
 *              The demo makes use of the simplified Berkley Sockets alike interface
 *              to transmit and receive the data.
 *              The server is defined as the DAG-Root within the network and
 *              its IP address is retrieved automatically. The Server just
 *              replies to the node it got the packet from.
 *              This demo is mainly used to show how to use the UDP socket
 *              interface and to show basic connectivity.
 */

/*
 * --- Includes -------------------------------------------------------------*
 */
#include "emb6.h"
#include "bsp.h"
#include "evproc.h"
#include "demo_serialapi.h"
#include "serialapi.h"
#include "sf_serialmac.h"

/*
 *  --- Macros ------------------------------------------------------------- *
 */
#define LOGGER_ENABLE               LOGGER_DEMO_SERIALAPI
#if LOGGER_ENABLE == TRUE
#define LOGGER_SUBSYSTEM            "SERIALAPI"
#endif /* #if LOGGER_ENABLE == TRUE */
#include "logger.h"


/** Maximum length of a serial API frame */
#define SERIALAPI_FRAMEBUF_MAX              255

/** Length of the input buffer */
#define SERIALAPI_RX_BUF_LEN                SERIALAPI_FRAMEBUF_MAX
/** Length of the output buffer */
#define SERIALAPI_TX_BUF_LEN                SERIALAPI_FRAMEBUF_MAX


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */

/* Callback when receiving data from lower interface */
static void _rxDataCb( void* p_data );

/* Callback from higher instance if data shall be transmitted */
static void _txDataCb( uint16_t len, void* p_param );

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


/*
 *  --- Local Variables ---------------------------------------------------- *
 */

/** Serial MAC context */
static struct sf_serialmac_ctx* p_macCtx;

/** Output frame buffer */
static uint8_t a_frameBufTx[SERIALAPI_TX_BUF_LEN];
/** Pointer to the output frame buffer */
static uint8_t* p_frameBufTx;
/** Input frame buffer */
static uint8_t a_frameBufRx[SERIALAPI_RX_BUF_LEN];
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


/* Callback from higher instance if data shall be transmitted
 *
 */

/**
 * \brief   allback from higher instance if data shall be transmitted.
 *
 *          This function is registered as tx functions for higher layers
 *          to transmit the actual data in the Tx buffer..
 *
 * \param   len         Length to transmit.
 * \param   p_param     Parameter registered during initialization.
 */
static void _txDataCb( uint16_t len, void* p_param )
{
    /* send data */
    sf_serialmac_tx_frame ( p_macCtx, len,
            (const char *)p_frameBufTx, len );

    /* process tx */
    sf_serialmac_hal_tx_callback( p_macCtx );
}


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
 * \param   frame_buffer          Buffer holding the received frame.
 * \param   frame_buffer_length   Length of the frame.
 */
static void _serialmac_rxframe_evt( void* mac_context, char* frame_buffer,
    size_t frame_buffer_length )
{
  /* A frame has been received */
  /* check parameters and forward frame*/
  EMB6_ASSERT_RET( (mac_context != NULL),  );
  serialApiInput( (uint8_t*)frame_buffer, frame_buffer_length, TRUE );
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
  else if( ev == EVENT_TYPE_STATUS_CHANGE )
  {

  }
}

/*
 * --- Global Function Definitions ----------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* demo_serialAPIInit()
*/
int8_t demo_serialApiInit( void )
{
  p_macCtx = &_macCtx;

  /* Init the Rx-buffer variables. */
  bufferRxWrite = bufferRx;
  bufferRxRead = bufferRx;
  bufferRxLen = 0U;
  bufferRxOverflow = false;

  /* set rx and tx frame buffer pointer */
  p_frameBufRx = a_frameBufRx;
  p_frameBufTx = a_frameBufTx;

  /* get the according UART from the BSP */
  p_uart = bsp_uartInit( EN_HAL_UART_SLIP );
  EMB6_ASSERT( p_uart != NULL );

  /* register callback for RX */
  bsp_periphIRQRegister( EN_HAL_PERIPHIRQ_SLIPUART_RX, _rxDataCb, NULL );

  /* Initialize the serial MAC */
  sf_serialmac_init( p_macCtx, p_uart, _serialMacRead, _serialMacReadWait,
      _serialMacWrite, _serialmac_rxframe_evt, _serialmac_rxbuf_evt,
      _serialmac_ignore_evt, _serialmac_ignore_evt );

  /* initialize serial API */
  serialApiInit( p_frameBufTx, SERIALAPI_TX_BUF_LEN, _txDataCb, NULL );

  /* register events */
  evproc_regCallback( EVENT_TYPE_SLIP_POLL, _event_callback );
  evproc_regCallback( EVENT_TYPE_STATUS_CHANGE, _event_callback );

  /* Always success */
  return 0;

} /* demo_serialAPIInit() */


/*---------------------------------------------------------------------------*/
/*
* demo_serialApiCfg()
*/
int8_t demo_serialApiConf( s_ns_t *p_netstk )
{
  int8_t i_ret = -1;

  if (p_netstk != NULL) {
    if (p_netstk->c_configured == FALSE) {
      p_netstk->hc = &hc_driver_sicslowpan;
      p_netstk->frame = &framer_802154;
      p_netstk->dllsec = &dllsec_driver_null;
      i_ret = 1;
    } else {
      if ((p_netstk->hc == &hc_driver_sicslowpan) &&
          (p_netstk->frame == &framer_802154) &&
          (p_netstk->dllsec == &dllsec_driver_null)) {
        i_ret = 1;
      } else {
        p_netstk = NULL;
        i_ret = 0;
      }
    }
  }
  return i_ret;
} /* demo_serialApiCfg() */
