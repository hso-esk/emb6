/*============================================================================*/
/**
 * \file    uart.c
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
#include <msp430.h>
#include <stdlib.h>
#include <stdint.h>
#include "targetconfig.h"
#include "hal_types.h"
#include "uart.h"
#include "mcu.h"
#include "int.h"


/*============================================================================*/
/*                          LOCAL FUNCTION PROTOTYPES                         */
/*============================================================================*/

#if( TARGET_CONFIG_UART0 == TRUE )
/* UART0 interrupt handler */
static void _uart_irqHandler0( void* p_params );
#endif /* #if( TARGET_CONFIG_UART0 == TRUE ) */

#if( TARGET_CONFIG_UART1 == TRUE )
/* UART1 interrupt handler */
static void _uart_irqHandler1( void* p_params );
#endif /* #if( TARGET_CONFIG_UART1 == TRUE ) */

#if( TARGET_CONFIG_UART3 == TRUE )
/* UART3 interrupt handler */
static void _uart_irqHandler3( void* p_params );
#endif /* #if( TARGET_CONFIG_UART3 == TRUE ) */


/*============================================================================*/
/*                            LOCAL VARIABLES                                 */
/*============================================================================*/

/** References to the according UART RX callbacks */
pf_uart_rxCb gpf_uartCB[E_UART_SEL_MAX] = { NULL };

/**
 * UART definitions.
 */
s_uart_desc_t gps_uart[E_UART_SEL_MAX] = {
#if (TARGET_CONFIG_UART0 == TRUE)
  /* UART0 */
  {&UCA0CTLW0_H, &UCA0CTLW0_L, &UCA0BRW_L, &UCA0BRW_H, &UCA0MCTL, &UCA0ICTL_L, &UCA0ICTL_H, &UCA0IV,
   &UCA0STAT, &UCA0TXBUF, &UCA0RXBUF, E_INT_IRQ_SRC_UART0, _uart_irqHandler0},
#endif /* #if (TARGET_CONFIG_UART0 == TRUE) */
#if (TARGET_CONFIG_UART1 == TRUE)
  /* UART1 */
  {&UCA1CTLW0_H, &UCA1CTLW0_L, &UCA1BRW_L, &UCA1BRW_H, &UCA1MCTL, &UCA1ICTL_L, &UCA1ICTL_H, &UCA1IV,
   &UCA1STAT, &UCA1TXBUF, &UCA1RXBUF, E_INT_IRQ_SRC_UART1, _uart_irqHandler1},
#endif /* #if (TARGET_CONFIG_UART1 == TRUE) */
#if (TARGET_CONFIG_UART3 == TRUE)
  /* UART3 */
  {&UCA3CTLW0_H, &UCA3CTLW0_L, &UCA3BRW_L, &UCA3BRW_H, &UCA3MCTL, &UCA3ICTL_L, &UCA3ICTL_H, &UCA3IV,
   &UCA3STAT, &UCA3TXBUF, &UCA3RXBUF, E_INT_IRQ_SRC_UART3, _uart_irqHandler3},
#endif /* #if (TARGET_CONFIG_UART3 == TRUE) */
};

/*============================================================================*/
/*                              LOCAL FUNCTIONS                               */
/*============================================================================*/

#if (TARGET_CONFIG_UART0 == TRUE)
/*============================================================================*/
/**
 * @brief   UART0 interrupt handler.
 *
 *          This function is invoked on every UART0 interrupt if a callback
 *          was registerd before. Every time an RX interrupt is recognized
 *          the registered callback will be invoked including the received byte.
 */
/*============================================================================*/
static void _uart_irqHandler0( void* p_params )
{
  e_uart_sel_t e_uart = E_UART_SEL_UART0;
  if( *gps_uart[e_uart].IV & USCI_UCRXIFG )
  {
    /* RX interrupt ... invoke callback if registered */
    if( gpf_uartCB[e_uart] != NULL )
        gpf_uartCB[e_uart]( *gps_uart[e_uart].RXBUF );
  }
}
#endif /* #if (TARGET_CONFIG_UART0 == TRUE) */


#if (TARGET_CONFIG_UART1 == TRUE)
/*============================================================================*/
/**
 * @brief   UART1 interrupt handler.
 *
 *          This function is invoked on every UART1 interrupt if a callback
 *          was registerd before. Every time an RX interrupt is recognized
 *          the registered callback will be invoked including the received byte.
 */
/*============================================================================*/
static void _uart_irqHandler1( void* p_params )
{
  e_uart_sel_t e_uart = E_UART_SEL_UART1;
  if( *gps_uart[e_uart].IV & USCI_UCRXIFG )
  {
    /* RX interrupt ... invoke callback if registered */
    if( gpf_uartCB[e_uart] != NULL )
        gpf_uartCB[e_uart]( *gps_uart[e_uart].RXBUF );
  }
}
#endif /* #if (TARGET_CONFIG_UART1 == TRUE) */


#if (TARGET_CONFIG_UART3 == TRUE)
/*============================================================================*/
/**
 * @brief   UART3 interrupt handler.
 *
 *          This function is invoked on every UART3 interrupt if a callback
 *          was registerd before. Every time an RX interrupt is recognized
 *          the registered callback will be invoked including the received byte.
 */
/*============================================================================*/
static void _uart_irqHandler3( void* p_params )
{
  e_uart_sel_t e_uart = E_UART_SEL_UART3;
  if( *gps_uart[e_uart].IV & USCI_UCRXIFG )
  {
    /* RX interrupt ... invoke callback if registered */
    if( gpf_uartCB[e_uart] != NULL )
        gpf_uartCB[e_uart]( *gps_uart[e_uart].RXBUF );
  }
}
#endif /* #if (TARGET_CONFIG_UART0 == TRUE) */

/*============================================================================*/
/*                              FUNCTIONS                                     */
/*============================================================================*/

/*=============================================================================
 * uart_init()
 *============================================================================*/
void uart_init( void )
{
  int i = 0;

  /* Initialize all the UART callbacks */
  for( i = 0; i < E_UART_SEL_MAX; i++ )
    gpf_uartCB[i] = NULL;

} /* uart_init() */

/*=============================================================================
 * uart_config()
 *============================================================================*/
int8_t uart_config( e_uart_sel_t e_uart, uint32_t ul_baudRate, pf_uart_rxCb pf_cB )
{
  int8_t i_ret = 0;

  uint16_t ui_preScaler;
  /* get system clock speed */
  uint32_t ul_sysClock = mcu_sysClockSpeedGet();
  /* calculate divider */
  ui_preScaler = (uint16_t) (ul_sysClock / ul_baudRate);

  /* Put UART into reset */
  *gps_uart[e_uart].CTL1 = UCSWRST;
  /* Configure as UART, no parity, LSB first, 8 bit, one stop bit */
  *gps_uart[e_uart].CTL0 = 0;

  /* Set the clock source as SMCLK */
  *gps_uart[e_uart].CTL1 |= UCSSEL_2;

  /* Configure prescaler (BR0 + BR1 * 256) */
  *gps_uart[e_uart].BR0 = (ui_preScaler & 0xFF);
  *gps_uart[e_uart].BR1 = ((ui_preScaler & 0xFF00) >> 8);

  /* Disable modulation */
  *gps_uart[e_uart].MCTL = UCBRS0;

  /* Register interrupt */
  gpf_uartCB[e_uart] = pf_cB;
  int_irqRegister(gps_uart[e_uart].e_irqSrc, gps_uart[e_uart].pf_isr);

  /* Release from reset */
  *gps_uart[e_uart].CTL1 &= ~UCSWRST;

  /* Enable RX interrupt */
  *gps_uart[e_uart].IE |= UCRXIE;

  return i_ret;
}

/*=============================================================================
 * uart_send()
 *============================================================================*/
int32_t uart_send( e_uart_sel_t e_uart, char* pc_array, uint16_t ui_size )
{
  int i_ret = -1;

  if( pc_array != NULL )
  {
    i_ret = ui_size;

    while( ui_size-- )
    {
      /* wait until TX buffer is ready */
      while (!( *gps_uart[e_uart].IFG & UCTXIFG));

      /* transmit next byte and increment data pointer*/
      *gps_uart[e_uart].TXBUF = *pc_array;
      pc_array++;
    }
    /* wait until the last byte has been sent */
    while( *gps_uart[e_uart].STAT & UCBUSY );
  }
  return i_ret;
}

/*=============================================================================
 * uart_sendbyte()
 *============================================================================*/
char uart_sendbyte( e_uart_sel_t e_uart, char c )
{
  /* wait until TX buffer is ready */
  while (!( *gps_uart[e_uart].IFG & UCTXIFG));

  /* transmit next byte and increment data pointer*/
  *gps_uart[e_uart].TXBUF = c;

  /* wait until the last byte has been sent */
  while( *gps_uart[e_uart].STAT & UCBUSY );

  return c;
}
