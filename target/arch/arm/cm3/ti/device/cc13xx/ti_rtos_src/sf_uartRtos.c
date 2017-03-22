
/**
  @file       sf_uartRtos.c
  @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
  @author     STACKFORCE
  @brief      UART implementations supporting the TI-RTOS.
*/

#define __DECL_SF_UART_H__

/*==============================================================================
                            INCLUDE FILES
==============================================================================*/
#include <stdlib.h>
#include <stddef.h>
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
/* Driver include  */
#include <ti/drivers/UART.h>

/* Example/Board Header files */
#include "Board.h"

#include "emb6_semaphore.h"
#include "sf_uart.h"
#include "sf_mcu_sleep.h"
/*==============================================================================
                            CONFIGURATION
==============================================================================*/

/*! Sets the length of the Rx ringbuffer. */
#define UART_BUFFER_RX_LEN            40U
/*! Number od bytes to read before calling the rx callback  */
#define UART_RX_LEN                    1U
/*==============================================================================
                            VARIABLES
==============================================================================*/
/* Buffer for the uart-module */
volatile uint8_t gc_uart_RtosBuffer[10];
/*! Input ring buffer. */
volatile uint8_t gc_uart_bufferRx[UART_BUFFER_RX_LEN];
/*! Pointer to the Rx ring buffer's current write position */
volatile uint8_t *gpc_uart_bufferRxWrite;
/*! Pointer to the Rx ring buffer's current read position */
volatile uint8_t *gpc_uart_bufferRxRead;
/* Global value which indictes if rx is active ore not */
volatile bool gb_readActive;

/*! Callback for UART Rx. */
sf_uart_rx_cb gf_uart_rx_cb = NULL;

/************************* UART Interface *************************************/
/* Handle for the uart-interface */
UART_Handle uartHandle;
/*==============================================================================
                            FUNCTION PROTOTYPES
==============================================================================*/
/* Uart read callback. Set by building the uart handle */
static void loc_uartReadCallback(UART_Handle handle, void *buf, size_t count);
/* Local function to handle the uart  read */
static void loc_uartRead(void);
/*==============================================================================
                          LOCAL  FUNCTIONS
==============================================================================*/

/*============================================================================*/
/*!
 * @brief Function called in case of a RX interrupt. This function will trigger
 *        the related SWI to handle the receiption.
 */
/*============================================================================*/
static void loc_uartReadCallback(UART_Handle handle, void *buf, size_t count)
{
  size_t i;

  gb_readActive = false;

  /* Copy bytes from uartRtos RX buffer to uart internal rx buf */
  for( i = 0; i < count; i++)
  {
    *gpc_uart_bufferRxWrite = ((uint8_t*)buf)[i];
    /* when using SLIP call the cb function */
    if(gf_uart_rx_cb != NULL)
        gf_uart_rx_cb(*gpc_uart_bufferRxWrite);
    gpc_uart_bufferRxWrite++;
    /*! Check for an overflow of the read pointer and adjust if required. */
    if(gpc_uart_bufferRxWrite >= &gc_uart_bufferRx[UART_BUFFER_RX_LEN])
    {
      gpc_uart_bufferRxWrite = gc_uart_bufferRx;
    } /* if */
  }/* for */

  loc_uartRead();
  /* Unblock the wmbus_task if needed */
  switch(sf_mcu_sleep_getMode())
  {
    case E_MCU_SLEEP_MODE_SLEEP:
    case E_MCU_SLEEP_MODE_DEEPSLEEP:
      semaphore_post();
      break;
    case E_MCU_SLEEP_MODE_NONE:
    default:
      break;
  } /* switch(sf_mcu_sleep_getMode()) */
}/* loc_uartReadCallback() */

/*============================================================================*/
/*!
 * @brief Small helper function to avoid dublicated code while serial
 *        receiption.
 */
/*============================================================================*/
static void loc_uartRead(void)
{
  if(gb_readActive == false)
  {
    if(UART_read(uartHandle, (void*)gc_uart_RtosBuffer, UART_RX_LEN) != UART_ERROR)
    {
      gb_readActive = true;
    }/* if */
  }/* if */

}/* loc_uartRead() */
/*==============================================================================
                          API  FUNCTIONS
==============================================================================*/

/*============================================================================*/
/*! sf_uart_init() */
/*============================================================================*/
bool sf_uart_init(void)
{
  bool b_return = false;
  UART_Params uartParams;

  /*! Init the Rx-buffer variables. */
  gpc_uart_bufferRxWrite = gc_uart_bufferRx;
  gpc_uart_bufferRxRead = gc_uart_bufferRx;
  gb_readActive = false;

  /* Create a UART with data processing off. */
  UART_Params_init(&uartParams);
  uartParams.writeDataMode = UART_DATA_BINARY;
  uartParams.readDataMode = UART_DATA_BINARY;
  uartParams.readReturnMode = UART_RETURN_FULL;
  uartParams.readEcho = UART_ECHO_OFF;
  uartParams.baudRate = 115200;
  uartParams.dataLength = UART_LEN_8;
  uartParams.stopBits = UART_STOP_ONE;
  uartParams.parityType = UART_PAR_NONE;


  uartParams.readMode = UART_MODE_CALLBACK;
  uartParams.writeMode = UART_MODE_BLOCKING;
  uartParams.readCallback = loc_uartReadCallback;

  uartHandle = UART_open(Board_UART0, &uartParams);

  if (uartHandle != NULL)
  {
    /* Enable the rx interrupt. The callback function "loc_uartReadCallback()"
     * will be called after the first byte was received */
    loc_uartRead();
    b_return = true;
  }/* if */

  return b_return;
} /* sf_uart_init() */

/*============================================================================*/
/* sf_uart_run() */
/*============================================================================*/
void sf_uart_run(void)
{
  /* If a uart is used it is assumed that it is possible to receive a serial
   * frame at any time. So we have to ensure that the device is in rx mode */
  if(gb_readActive == false)
  {
    UART_close(uartHandle);
    sf_uart_init();
  }
}/* sf_uart_run() */

/*============================================================================*/
/* sf_uart_write() */
/*============================================================================*/
uint16_t sf_uart_write(uint8_t *pc_data, uint16_t i_len)
{
  int i;
  uint16_t i_txLen = 0x00U;

  /* Call the RTOS uart write function. This function will block until all byte
   * are sent */
  i = UART_write(uartHandle, (void*)pc_data, i_len);

  /* Check if writing the data was succesfull */
  if(i != UART_ERROR)
  {
    i_txLen = (uint16_t)i;
  }/* if */

  /*! Return the number of written byte. */
  return i_txLen;
} /* sf_uart_write() */

/*============================================================================*/
/* sf_uart_read() */
/*============================================================================*/
uint16_t sf_uart_read(uint8_t *pc_data, uint16_t i_len)
{
  uint16_t i;

  for(i = 0U; (i < i_len) && (sf_uart_cntRxBytes() > 0U); i++)
  {
    /* Read the Rx buffer and decrement the received bytes */
    pc_data[i] = *gpc_uart_bufferRxRead++;

    /** Check for an overflow of the read pointer and adjust if required. */
    if(gpc_uart_bufferRxRead == &gc_uart_bufferRx[UART_BUFFER_RX_LEN])
    {
      gpc_uart_bufferRxRead = gc_uart_bufferRx;
    } /* if */
  } /* for */
  return i;
} /* sf_uart_read() */

/*============================================================================*/
/* sf_uart_cntRxBytes() */
/*============================================================================*/
uint16_t sf_uart_cntRxBytes(void)
{
  uint16_t i_cnt;

  if(gpc_uart_bufferRxWrite > gpc_uart_bufferRxRead)
  {
    i_cnt = gpc_uart_bufferRxWrite - gpc_uart_bufferRxRead;
  }
  else if(gpc_uart_bufferRxWrite < gpc_uart_bufferRxRead)
  {
    i_cnt = &gc_uart_bufferRx[UART_BUFFER_RX_LEN] - gpc_uart_bufferRxRead;
    i_cnt +=  gpc_uart_bufferRxWrite - gc_uart_bufferRx;
  }
  else
  {
    i_cnt = 0x00;
  }/* if() ... else if ... else */

  return i_cnt;
} /* sf_uart_cntRxBytes() */

/*============================================================================*/
/* sf_uart_cntTxBytes() */
/*============================================================================*/
uint16_t sf_uart_cntTxBytes(void)
{
  /* The writing is implemented as blocking function. So if the write returns
   * all byte are transmitted */
  return 0x00U;
} /* sf_uart_cntTxBytes() */

/*============================================================================*/
/*! sf_uart_isRxOverflow() */
/*============================================================================*/
bool sf_uart_isRxOverflow(void)
{
  return false;
} /* sf_uart_isRxOverflow() */

/*============================================================================*/
/*! sf_uart_setRxCb() */
/*============================================================================*/
void sf_uart_setRxCb(sf_uart_rx_cb  rx_cb)
{
  gf_uart_rx_cb = rx_cb;
} /* sf_uart_setRxCb() */
