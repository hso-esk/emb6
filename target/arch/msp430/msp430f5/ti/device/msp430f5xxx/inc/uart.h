#ifndef __UART_H__
#define __UART_H__

/*============================================================================*/
/**
 * \file    uart.h
 *
 * \author  Tobias Neff
 *
 * \brief   UART functions.
 *
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <stdint.h>
#include <msp430.h>
#include "targetconfig.h"
#include "int.h"

/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/
typedef enum E_UART_SEL_T
{
#if( TARGET_CONFIG_UART0 == TRUE )
  /* UART0 */
  E_UART_SEL_UART0,
#endif /* #if( TARGET_CONFIG_UART0 == TRUE ) */
#if( TARGET_CONFIG_UART1 == TRUE )
  /* UART1 */
  E_UART_SEL_UART1,
#endif /* #if( TARGET_CONFIG_UART1 == TRUE ) */
#if( TARGET_CONFIG_UART3 == TRUE )
  /* UART3 */
  E_UART_SEL_UART3,
#endif /* #if( TARGET_CONFIG_UART3 == TRUE ) */

  E_UART_SEL_MAX
} e_uart_sel_t;

/*============================================================================*/
/*                     STRUCTURES AND OTHER TYPEDEFS                          */
/*============================================================================*/

/**
 * Description of a UART.
 *
 * Defines all the elements to access a UARTs fuctions and to configure
 * a UART properly.
 */
typedef struct S_UART_DESC_T
{
  /** Control 0 */
  REG8B CTL0;
  /** Control 1 */
  REG8B CTL1;
  /** Baud Rate Control 0 */
  REG8B BR0;
  /** Baud Rate Control 1 */
  REG8B BR1;
  /** Modulation Control */
  REG8B MCTL;
  /** Interrupt Enable */
  REG8B IE;
  /** interrupt flag */
  REG8B IFG;
  /** interrupt vector */
  REG16B IV;
  /** Status */
  REG8B_CONST STAT;
  /** TX Buffer */
  REG8B TXBUF;
  /** RX Buffer */
  REG8B_CONST RXBUF;

  /** interrupt source */
  e_int_irq_src_t e_irqSrc;
  /** interrupt handler to use */
  pf_int_cb pf_isr;

} s_uart_desc_t;

/** Definition for RX callback functions */
typedef void (*pf_uart_rxCb)( uint8_t c );

/*============================================================================*/
/*                          FUNCTION PROTOTYPES                               */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief   Initialize UART module.
 *
 *          This function initializes the UART module. This function has to be
 *          called before accessing a UART.
 */
/*============================================================================*/
void uart_init( void );


/*============================================================================*/
/**
 * @brief   Configures UART module.
 *
 *          Before a specific UART can be used it has to be configured. This
 *          for example includes setting the baudrate and the RX callback
 *          function.
 *
 * @param   e_uart       Select UART module to configure.
 * @param   ul_baudRate  Select baud rate to use.
 *
 * @return  0 on success or negative value on error.
 */
/*============================================================================*/
int8_t uart_config( e_uart_sel_t e_uart, uint32_t ul_baudRate,
  pf_uart_rxCb pf_cB );


/*============================================================================*/
/**
 * @brief   Sends data over a UART.
 *
 *          This function is used to transmit data using a specific UART.
 *
 * @param   e_uart       Select UART module. Valid values: UART1, UART3
 * @param   pc_array     Data to be sent.
 * @param   ui_size      Size of the data.
 *
 * @return  The number of transmitted bytes or negative value on error.
 */
/*============================================================================*/
int32_t uart_send(e_uart_sel_t e_uart, char* pc_array, uint16_t ui_size);

/*============================================================================*/
/**
 * @brief   Sends one byte over a UART.
 *
 *          This function is used to transmit data using a specific UART.
 *
 * @param   e_uart       Select UART module. Valid values: UART1, UART3
 * @param   c            Char to be sent.
 *
 * @return  Transmitted character
 */
/*============================================================================*/
char uart_sendbyte( e_uart_sel_t e_uart, char c );

#endif /* #ifndef __UART_H__ */
