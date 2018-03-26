/**
* @code
*  ___ _____ _   ___ _  _____ ___  ___  ___ ___
* / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
* \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
* |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
* embedded.connectivity.solutions.==============
* @endcode
*
* @file       sf_uart.c
* @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
* @author     STACKFORCE
* @brief      UART implementation.
*/

#define __DECL_SF_UART_H__

/*==============================================================================
                            INCLUDE FILES
==============================================================================*/

/* Standart libraries */
#include <stdint.h>
#include <stdbool.h>
#include "target_conf.h"

#include "sf_mcu.h"
#include "sf_uart.h"

/* Driver lib include  */
#include "driverlib/uart.h"
#include "driverlib/prcm.h"
#include "driverlib/ioc.h"
/* bsp includes */
#include "bsp/srf06eb_cc26xx/drivers/source/bsp.h"

#include "hal.h"

/* Which events to trigger a UART interrupt */
#define UART_RX_INTERRUPT_TRIGGERS        (UART_INT_RX | UART_INT_RT)

/* Which events to trigger a UART interrupt */
#define UART_TX_INTERRUPT_TRIGGERS        (UART_INT_TX)

/* All interrupt masks */
#define UART_INTERRUPT_ALL                (UART_INT_OE | UART_INT_BE | UART_INT_PE | \
                                           UART_INT_FE | UART_INT_RT | UART_INT_TX | \
                                           UART_INT_RX | UART_INT_CTS)

/*==============================================================================
                            CONFIGURATION
==============================================================================*/
#ifndef UART_IOID_RXD
#error Please define a valid rx-pin
#endif /* UART_IOID_RXD */

#ifndef UART_IOID_TXD
#error Please define a valid tx-pin
#endif /* UART_IOID_RXD */

#if (UART_IOID_RXD == UART_IOID_TXD)
#error Please use different pins for rx and tx opperation
#endif /* UART_IOID_RXD == UART_IOID_TXD */

#ifndef UART_TX_BLOCKING
#define UART_TX_BLOCKING            1
#endif /* #ifndef UART_TX_BLOCKING */


#ifndef USE_FIFO
#define USE_FIFO                    FALSE
#endif /* #ifndef UART_TX_BLOCKING */

/*! Sets the length of the Rx ringbuffer. */
#ifndef UART_BUFFER_RX_LEN
#define UART_BUFFER_RX_LEN          128U
#endif /* #ifndef UART_BUFFER_RX_LEN */

/*! Sets the length of the Tx ringbuffer. */
#ifndef UART_BUFFER_TX_LEN
#define UART_BUFFER_TX_LEN          128U
#endif /* #ifndef UART_BUFFER_TX_LEN */

/*! Default baudrate used for the uart */
#ifndef UART_DEFAULT_BAUD
#define UART_DEFAULT_BAUD           115200U
#endif /* #ifndef UART_DEFAULT_BAUD */
/*==============================================================================
                            VARIABLES
==============================================================================*/
/*! Input ring buffer. */
volatile uint8_t gc_uart_bufferRx[UART_BUFFER_RX_LEN];
/*! Pointer to the Rx ring buffer's current write position */
volatile uint8_t *gpc_uart_bufferRxWrite;
/*! Pointer to the Rx ring buffer's current read position */
volatile uint8_t *gpc_uart_bufferRxRead;
/*! Number of bytes in the input buffer. */
volatile uint16_t gi_uart_bufferRxLen;
/*! Set if there was a Rx buffer overflow. */
volatile bool gb_uart_bufferRxOverflow;

/*! Input ring buffer. */
volatile uint8_t gc_uart_bufferTx[UART_BUFFER_TX_LEN];
/*! Pointer to the Tx ring buffer's current write position */
volatile uint8_t *gpc_uart_bufferTxWrite;
/*! Pointer to the Tx ring buffer's current read position */
volatile uint8_t *gpc_uart_bufferTxRead;
/*! Number of bytes in the input buffer. */
volatile uint16_t gi_uart_bufferTxLen;


/*! Callback for UART Rx. */
sf_uart_rx_cb gf_uart_rx_cb = NULL;

/*==============================================================================
                            FUNCTION PROTOTYPES
==============================================================================*/
void loc_writeUartTxFifo(void);
void UART0IntHandler(void);

/*==============================================================================
                          LOCAL  FUNCTIONS
==============================================================================*/

/*============================================================================*/
/*!
 * @brief Writes to the tx fifo of the CC13xx
 */
/*============================================================================*/
void loc_writeUartTxFifo(void)
{
  if(gi_uart_bufferTxLen > 0x00U)
  {
    /* Put a character in the output buffer fifo. */
    while(UARTCharPutNonBlocking(UART0_BASE, *gpc_uart_bufferTxRead))
    {
      /* Increrase the tx read pointer */
      gpc_uart_bufferTxRead++;
      /*! Decrease the number of bytes in Tx-ringbuffer. */
      gi_uart_bufferTxLen--;
      /*! Check for an overflow of the write pointer and adjust if required. */
      if(gpc_uart_bufferTxRead == &gc_uart_bufferTx[UART_BUFFER_TX_LEN])
      {
        gpc_uart_bufferTxRead = gc_uart_bufferTx;
      } /* if */
      if(gi_uart_bufferTxLen == 0x00U)
      {
        /* Leave this loop if there are no more byte to send */
        break;
      }/* if */
    }/* while() */
  }/* if */
  else
  {
    /* All bytes transmitted. Disable the tx interrupt */
    UARTIntDisable(UART0_BASE, UART_INT_TX);
  }
}/* loc_writeUartTxFifo() */

/*============================================================================*/
/*!
 * @brief Reads from the rx fifo of the CC13xx
 */
/*============================================================================*/

void loc_readUartRxFifo(void)
{

#if USE_FIFO
  /*! If the Rx-ringbuffer is full, disable the Rx-interrupt. */
  if(UART_BUFFER_RX_LEN <= gi_uart_bufferRxLen)
  {
    //UARTIntDisable(UART0_BASE, UART_INT_RX);
    gb_uart_bufferRxOverflow = true;
    /* Clear the rx fifo. */
    while(UARTCharsAvail(UART0_BASE) == true)
    {
      /* Copy data into RX Buffer && clear buffer at the same time  */
      uint8_t rxData = UARTCharGet(UART0_BASE);
      if(gf_uart_rx_cb != NULL)
          gf_uart_rx_cb(rxData);
    }/* while */
  }
  else
  {
    /* Read a character out of the input buffer fifo. */
    while(UARTCharsAvail(UART0_BASE) == true)
    {
      /*! Read the next byte into the Rx-ringbuffer. */
      *gpc_uart_bufferRxWrite = (uint8_t) UARTCharGet(UART0_BASE);
      if(gf_uart_rx_cb != NULL)
          gf_uart_rx_cb(*gpc_uart_bufferRxWrite);

      gpc_uart_bufferRxWrite++;
      /*! Increase the number of bytes in Rx-ringbuffer. */
      gi_uart_bufferRxLen++;

      /*! Check for an overflow of the read pointer and adjust if required. */
      if(gpc_uart_bufferRxWrite == &gc_uart_bufferRx[UART_BUFFER_RX_LEN])
      {
        gpc_uart_bufferRxWrite = gc_uart_bufferRx;
      } /* if */
    }/* while() */
  }/* if...else */
#else

  while(UARTCharsAvail(UART0_BASE) == true)
  {
    uint8_t rxData = UARTCharGetNonBlocking(UART0_BASE);
    if(gf_uart_rx_cb != NULL)
        gf_uart_rx_cb(rxData);
  }
#endif
}/* loc_readUartRxFifo() */
/*******************************************************************************

  The interrupt handler for the UART0 interrupt.

*******************************************************************************/
void UART0IntHandler(void)
{

  uint32_t flags;
  /* Read out the masked interrupt status */
  flags = UARTIntStatus(UART0_BASE, true);

  /* Clear all UART interrupt flags */
  UARTIntClear(UART0_BASE, UART_INTERRUPT_ALL);

  if((flags & UART_RX_INTERRUPT_TRIGGERS) != 0) {
    /*
     * If this was a FIFO RX or an RX timeout, read all bytes available in the
     * RX FIFO.
     */
    while(UARTCharsAvail(UART0_BASE)) {
        /* Fill the rx fifo */
        loc_readUartRxFifo();
    }
  }

  if( flags & UART_TX_INTERRUPT_TRIGGERS ) {
        /* Fill the tx fifo */
        loc_writeUartTxFifo();
  }
}
/*==============================================================================
                            FUNCTIONS
==============================================================================*/

/*============================================================================*/
/*! sf_uart_init() */
/*============================================================================*/
bool sf_uart_init(void)
{
  /*! Init the Rx-buffer variables. */
  gpc_uart_bufferRxWrite = gc_uart_bufferRx;
  gpc_uart_bufferRxRead = gc_uart_bufferRx;
  gi_uart_bufferRxLen = 0U;
  gb_uart_bufferRxOverflow = false;
  /*! Init the Tx-buffer variables. */
  gpc_uart_bufferTxWrite = gc_uart_bufferTx;
  gpc_uart_bufferTxRead = gc_uart_bufferTx;
  gi_uart_bufferTxLen = 0U;

  /* Make sure the peripheral power domain is on using PRCMPowerDomainOn,
     power status can be checked with PRCMPowerDomainStatus */
  PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL);
  while(PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON);

  /* Make sure the specific peripheral you want to use it enabled using
     PRCMPeripheralRunEnable(�) and then PRCMLoadSet() */
  PRCMPeripheralRunEnable(PRCM_PERIPH_UART0);

  /* Allow UART to run when system sleeps */
  PRCMPeripheralSleepEnable(PRCM_PERIPH_UART0);

  /* Apply settings and wait for them to take effect */
  PRCMLoadSet();
  while(!PRCMLoadGet());

  /* Disable UART function before configuring module */
  UARTDisable(UART0_BASE);

  /* Map signals to the correct GPIO pins and configure them as HW ctrl'd */
  IOCPinTypeUart(UART0_BASE, UART_IOID_RXD, UART_IOID_TXD, IOID_UNUSED,
                 IOID_UNUSED);

  /* Initialize the UART. Set the baud rate, number of data bits, turn off parity,
     number of stop bits, and stick mode. */
  UARTConfigSetExpClk(UART0_BASE, BSP_SYS_CLK_SPD, UART_DEFAULT_BAUD,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

  /* Enable the uart */
  UARTEnable(UART0_BASE);
  
  /* Disable the FIFO. In RX case we need to get an interrupt after each
     received byte (not possible with fifo). */
  UARTFIFODisable(UART0_BASE);

  /* Enable RX interrupt of UART module */
  UARTIntEnable(UART0_BASE, UART_INT_RX);

  /* Enable the UART0 interrupt on the processor (NVIC). */
  UARTIntRegister(UART0_BASE, UART0IntHandler);

  /* reset Rx callback */
  gf_uart_rx_cb = NULL;

  return true;
} /* sf_uart_init() */

/*============================================================================*/
/* sf_uart_write() */
/*============================================================================*/
uint16_t sf_uart_write(uint8_t *pc_data, uint16_t i_len)
{
  uint16_t i;

  /* Disable TX interrupt of UART module. To avoid underflow of the ty len
    counter "gi_uart_bufferTxLen" => will be decreased in function
    loc_writeUartTxFifo() this function will also be called in the isr! */
  UARTIntDisable(UART0_BASE, UART_INT_TX);

#ifdef UART_TX_BLOCKING
  for( i = 0; i < i_len; i++)
	  UARTCharPut(UART0_BASE, pc_data[i]);
#else

  /*! Verify length-value. */
  if((0U == i_len) || (UART_BUFFER_TX_LEN <= gi_uart_bufferTxLen))
  {
    i = 0U;
  } /* if */
  else
  {
    /*! Write to Tx-ringbuffer until i_len or UART_BUFFER_TX_LEN is reached. */
    for(i = 0U; (i < i_len) && (UART_BUFFER_TX_LEN > gi_uart_bufferTxLen); i++)
    {
      /*! Write current byte to the write pointers address within the ringbuffer
          and increase the pointer. */
      *gpc_uart_bufferTxWrite++ = pc_data[i];
      /*! Increase the number of bytes in ringbuffer. */
      gi_uart_bufferTxLen++;
      /*! Check for an overflow of the write pointer and adjust if required. */
      if(gpc_uart_bufferTxWrite == &gc_uart_bufferTx[UART_BUFFER_TX_LEN])
      {
        gpc_uart_bufferTxWrite = gc_uart_bufferTx;
      } /* if */
    } /* for */
  }/* if...else */

  loc_writeUartTxFifo();

  /* Enable TX interrupt of UART module */
  UARTIntEnable(UART0_BASE, UART_INT_TX);
#endif /* #ifdef UART_TX_BLOCKING */

  /*! Return the number of bytes written until the loop was left. */
  return i;
} /* sf_uart_write() */

/*============================================================================*/
/* sf_uart_read() */
/*============================================================================*/
uint16_t sf_uart_read(uint8_t *pc_data, uint16_t i_len)
{
  uint16_t i;

  /*! Read from Rx-ringbuffer until i_len or UART_BUFFER_RX_LEN is reached. */
  for(i = 0U; (i < i_len) && (0U < gi_uart_bufferRxLen); i++)
  {
    /*! Write to the specified data pointer and increase the read pointer. */
    pc_data[i] = *gpc_uart_bufferRxRead++;
    /*! Decrease the number of bytes in ringbuffer. */
    gi_uart_bufferRxLen--;
    /*! Check for an overflow of the read pointer and adjust if required. */
    if(gpc_uart_bufferRxRead == &gc_uart_bufferRx[UART_BUFFER_RX_LEN])
    {
      gpc_uart_bufferRxRead = gc_uart_bufferRx;
    } /* if */
  }/* for */

  /* Enable RX interrupt of UART module */
  UARTIntEnable(UART0_BASE, UART_INT_RX);

  return i;
} /* sf_uart_read() */

/*============================================================================*/
/* sf_uart_cntRxBytes() */
/*============================================================================*/
uint16_t sf_uart_cntRxBytes(void)
{
  return gi_uart_bufferRxLen;
} /* sf_uart_cntRxBytes() */

/*============================================================================*/
/* sf_uart_cntTxBytes() */
/*============================================================================*/
uint16_t sf_uart_cntTxBytes(void)
{
  return (UART_BUFFER_TX_LEN - gi_uart_bufferTxLen);
} /* sf_uart_cntTxBytes() */

/*============================================================================*/
/*! sf_uart_isRxOverflow() */
/*============================================================================*/
bool sf_uart_isRxOverflow(void)
{
  bool b_return;

  b_return = gb_uart_bufferRxOverflow;
  gb_uart_bufferRxOverflow = false;
  return b_return;
} /* sf_uart_isRxOverflow() */

/*============================================================================*/
/*! sf_uart_setRxCb() */
/*============================================================================*/
void sf_uart_setRxCb(sf_uart_rx_cb  rx_cb)
{
  gf_uart_rx_cb = rx_cb;
} /* sf_uart_setRxCb() */
