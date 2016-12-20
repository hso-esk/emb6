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
 * Copyright (c) 2016,
 * Hochschule Offenburg, University of Applied Sciences
 * Institute of reliable Embedded Systems and Communications Electronics.
 * All rights reserved.
 */

/*
 * --- Module Description ---------------------------------------------------*
 */
/**
 *  \file       hal_template.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      HAL implementation for MSP430x5xx MCU.
 *
 */

/*
 *  --- Includes -------------------------------------------------------------*
 */
#include <msp430.h>
#include "hal.h"

#include "mcu.h"
#include "io.h"
#include "tmr.h"
#include "uart.h"
#include "rt_tmr.h"

/*
 *  --- Macros ------------------------------------------------------------- *
 */
/** Enable/Disable logger */
#define LOGGER_ENABLE                   LOGGER_HAL
#include "logger.h"

/** Number of ticks per second */
#define MSP430_TICK_SECONDS             ( 1000u )

/** Tick pre-scaler */
#define MSP430_TICK_SCALER              ( 1u )

/** Enable/Disable high logic */
#define MSP430_IO_HIGH_LOGIC            FALSE

/** Pin direction */
#define MSP430_IO_PIN_DIR_INPUT         0
#define MSP430_IO_PIN_DIR_OUTPUT        1

/*
 * --- Type Definitions -----------------------------------------------------*
 */

/**
 * \brief   Description of a single Pin.
 *
 *          A pin consists of several attributes such as its port
 *          and pin numbers and the output mode and IRQ callbacks.
 */
typedef struct
{
  /** Port */
  s_io_port_desc_t* PORT;
  /** Pin */
  uint8_t PIN;
  /** Mask */
  uint8_t MSK;
  /** Direction */
  uint8_t DIR;
  /** IRQ callback */
  pf_hal_irqCb_t pf_cb;

} s_hal_gpio_pin_t;


/**
 * \brief   Description of an SPI interface.
 *
 *          An SPI interface of the EFM32 consists of the according
 *          pins and handles.
 */
typedef struct
{
  /** clk pin */
  s_hal_gpio_pin_t* p_clkPin;
  /** tx  pin */
  s_hal_gpio_pin_t* p_txPin;
  /** rx pin */
  s_hal_gpio_pin_t* p_rxPin;
  /** chip select pin */
  s_hal_gpio_pin_t* p_csPin;

} s_hal_spi_t;


/**
 * \brief   Description of an UART interface.
 *
 *          A UART interface of the EFM32 consists of the according
 *          pins and handles.
 */
typedef struct
{
  /* TODO missing MCU-specific attributes */

  /** tx pin */
  s_hal_gpio_pin_t* p_txPin;
  /** rx  pin */
  s_hal_gpio_pin_t* p_rxPin;

} s_hal_uart_t;


/** IRQ structure declaration */
typedef struct
{
  /** callback function */
  pf_hal_irqCb_t pf_cb;
  /** data pointer */
  void* p_data;

} s_hal_irq;


/*
 *  --- Local Variables ---------------------------------------------------- *
 */
/** Ticks since startup */
static clock_time_t volatile l_hal_tick;
/** Seconds since startup */
static clock_time_t volatile l_hal_sec;

/** Definition of the IOs */
static s_hal_gpio_pin_t s_hal_gpio[EN_HAL_PIN_MAX] = {
  /* TODO missing LEDs definition */
#if defined(HAL_SUPPORT_LED0)
  {&gps_io_port[MSP430_IO_PORT_LED0], MSP430_IO_PIN_LED0, MSP430_IO_MASK_LED0, MSP430_IO_PIN_DIR_OUTPUT, NULL},
#endif /* #if defined(HAL_SUPPORT_LED0) */
#if defined(HAL_SUPPORT_LED1)
  {&gps_io_port[MSP430_IO_PORT_LED1], MSP430_IO_PIN_LED1, MSP430_IO_MASK_LED1, MSP430_IO_PIN_DIR_OUTPUT, NULL},
#endif /* #if defined(HAL_SUPPORT_LED1) */
#if defined(HAL_SUPPORT_LED2)
  {&gps_io_port[MSP430_IO_PORT_LED2], MSP430_IO_PIN_LED2, MSP430_IO_MASK_LED2, MSP430_IO_PIN_DIR_OUTPUT, NULL},
#endif /* #if defined(HAL_SUPPORT_LED2) */
#if defined(HAL_SUPPORT_LED3)
  {&gps_io_port[MSP430_IO_PORT_LED3], MSP430_IO_PIN_LED3, MSP430_IO_MASK_LED3, MSP430_IO_PIN_DIR_OUTPUT, NULL},
#endif /* #if defined(HAL_SUPPORT_LED3) */

  /* TODO missing RF_SPI definition */
#if defined(HAL_SUPPORT_RFSPI)
  {&gps_io_port[MSP430_IO_PORT_SPI_CLK], MSP430_IO_PIN_SPI_CLK, MSP430_IO_MASK_SPI_CLK, MSP430_IO_PIN_DIR_OUTPUT, NULL},
  {&gps_io_port[MSP430_IO_PORT_SPI_MOSI], MSP430_IO_PIN_SPI_MOSI, MSP430_IO_MASK_SPI_MOSI, MSP430_IO_PIN_DIR_OUTPUT, NULL},
  {&gps_io_port[MSP430_IO_PORT_SPI_MISO], MSP430_IO_PIN_SPI_MISO, MSP430_IO_MASK_SPI_MISO, MSP430_IO_PIN_DIR_INPUT, NULL},
  {&gps_io_port[MSP430_IO_PORT_SPI_CS], MSP430_IO_PIN_SPI_CS, MSP430_IO_MASK_SPI_CS, MSP430_IO_PIN_DIR_OUTPUT, NULL},
#endif /* #if defined(HAL_SUPPORT_RFSPI) */

#if defined(HAL_SUPPORT_RFCTRL0)
  {&gps_io_port[MSP430_IO_PORT_RF_CTRL0], MSP430_IO_PIN_RF_CTRL0, MSP430_IO_MASK_RF_CTRL0, MSP430_IO_PIN_DIR_INPUT, NULL},
#endif /* #if defined(HAL_SUPPORT_RFCTRL0) */
#if defined(HAL_SUPPORT_RFCTRL1)
  {&gps_io_port[MSP430_IO_PORT_RF_CTRL1], MSP430_IO_PIN_RF_CTRL1, MSP430_IO_MASK_RF_CTRL1, MSP430_IO_PIN_DIR_INPUT, NULL},
#endif /* #if defined(HAL_SUPPORT_RFCTRL1) */
#if defined(HAL_SUPPORT_RFCTRL2)
  {&gps_io_port[MSP430_IO_PORT_RF_CTRL2], MSP430_IO_PIN_RF_CTRL2, MSP430_IO_MASK_RF_CTRL2, MSP430_IO_PIN_DIR_INPUT, NULL},
#endif /* #if defined(HAL_SUPPORT_RFCTRL2) */

#if defined(HAL_SUPPORT_SLIPUART)
  {NULL, NULL, NULL, MSP430_IO_PIN_DIR_INPUT, NULL},
#endif /* #if defined(HAL_SUPPORT_SLIPUART) */

};


#if defined(HAL_SUPPORT_RFSPI)
/** Definition of the SPI interface */
static s_hal_spi_t s_hal_spi = {
  /* TODO add missing definition */

  .p_clkPin = &s_hal_gpio[EN_HAL_PIN_RFSPICLK],
  .p_txPin  = &s_hal_gpio[EN_HAL_PIN_RFSPITX],
  .p_rxPin  = &s_hal_gpio[EN_HAL_PIN_RFSPIRX],
  .p_csPin  = &s_hal_gpio[EN_HAL_PIN_RFSPICS]
};
#endif /* #if defined(HAL_SUPPORT_RFSPI) */


#if defined(HAL_SUPPORT_SLIPUART)
/** Definition of the SPI interface */
static s_hal_uart_t s_hal_uart = {
  /* TODO add missing definition */

  .p_txPin  = &s_hal_gpio[EN_HAL_PIN_SLIPUARTTX],
  .p_rxPin  = &s_hal_gpio[EN_HAL_PIN_SLIPUARTRX],
};
#endif /* #if defined(HAL_SUPPORT_RFSPI) */


/** Definition of the peripheral callback functions */
static s_hal_irq s_hal_irqs[EN_HAL_PERIPHIRQ_MAX];


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */
static void _hal_tcInit( void );
static void _hal_tcCb( void *arg );
static void _hal_uartRxCb( uint8_t c );

/*
 *  --- Local Functions ---------------------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* _hal_clkInit()
*/
static void _hal_tcInit( void )
{
  uint16_t period;

  /* initialize timer */
  tmr_init();

  period = (uint16_t)(MSP430_TICK_SECONDS / MSP430_TICK_SCALER);
  tmr_config(MSP430_SYSTICK_TMR, period);
  tmr_start(MSP430_SYSTICK_TMR, _hal_tcCb );

} /* _hal_clkInit() */

/*---------------------------------------------------------------------------*/
/*
* _hal_tcCb()
*/
static void _hal_tcCb( void *arg )
{
  (void)&arg;

  /* Indicate timer update to the emb6 timer */
  hal_enterCritical();
  if (l_hal_tick % MSP430_TICK_SECONDS == 0) {
    l_hal_sec++;
  }
  l_hal_tick++;
  hal_exitCritical();

  /* update real-time timers*/
  rt_tmr_update();

} /* _hal_tcCb() */

#if defined(HAL_SUPPORT_UART)
#if defined(HAL_SUPPORT_PERIPHIRQ_SLIPUART_RX)
/*---------------------------------------------------------------------------*/
/*
* _hal_uartRxCb()
*/
static void _hal_uartRxCb( uint8_t c )
{
  if( s_hal_irqs[EN_HAL_PERIPHIRQ_SLIPUART_RX].pf_cb != NULL )
  {
    s_hal_irqs[EN_HAL_PERIPHIRQ_SLIPUART_RX].pf_cb( &c );
  }
} /* _hal_uartRxCb() */
#endif /* #if defined(HAL_SUPPORT_PERIPHIRQ_SLIPUART_RX) */
#endif /* #if defined(HAL_SUPPORT_UART) */


#if (LOGGER_LEVEL > 0) && (HAL_SUPPORT_SLIPUART == FALSE)
/*---------------------------------------------------------------------------*/
/*
* putchar()
*/
#if !defined(GCC_COMPILER)
int putchar(int c)
{
    uart_send(MSP430_DEBUG_UART, (char *)&c, 1);
    return c;
}
#endif /*#if !defined(GCC_COMPILER) */
#endif /* #if (LOGGER_LEVEL > 0) && (HAL_SUPPORT_SLIPUART == FALSE) */


/*
 * --- Global Function Definitions ----------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* hal_init()
*/
int8_t hal_init( void )
{
  /* initialize IO */
  io_init();

  /* initialize system clock */
  mcu_sysClockInit( MCU_SYSCLK_25MHZ );

  /* initialize external interrupts */
  int_init();

  /* initialize timer counter */
  _hal_tcInit();

  /* Enable global interrupt */
  _BIS_SR(GIE);

  return 0;
} /* hal_init() */

/*---------------------------------------------------------------------------*/
/*
* hal_enterCritical()
*/
int8_t hal_enterCritical( void )
{
  __disable_interrupt();
  return 0;
} /* hal_enterCritical() */

/*---------------------------------------------------------------------------*/
/*
* hal_exitCritical()
*/
int8_t hal_exitCritical( void )
{
  __enable_interrupt();
  return 0;
} /* hal_exitCritical() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogStart()
*/
int8_t hal_watchdogStart( void )
{
  /* TODO missing implementation */
  return -1;
} /* hal_watchdogStart() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogReset()
*/
int8_t hal_watchdogReset( void )
{
  /* TODO missing implementation */
  return -1;
} /* hal_watchdogReset() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogStop()
*/
int8_t hal_watchdogStop( void )
{
  /* TODO missing implementation */
  return -1;
} /* hal_watchdogStop() */

/*---------------------------------------------------------------------------*/
/*
* hal_getrand()
*/
uint32_t hal_getrand( void )
{
  /* TODO missing implementation */
  return l_hal_tick;
} /* hal_getrand() */

/*---------------------------------------------------------------------------*/
/*
* hal_getTick()
*/
clock_time_t hal_getTick( void )
{
  return l_hal_tick;
} /* hal_getTick() */

/*---------------------------------------------------------------------------*/
/*
* hal_getSec()
*/
clock_time_t hal_getSec( void )
{
  return l_hal_sec;
} /* hal_getSec() */

/*---------------------------------------------------------------------------*/
/*
* hal_getTRes()
*/
clock_time_t hal_getTRes( void )
{
  return MSP430_TICK_SECONDS;
} /* hal_getTRes() */

/*---------------------------------------------------------------------------*/
/*
* hal_delayUs()
*/
int8_t hal_delayUs( uint32_t delay )
{
  /*
   * Note(s)
   *
   * hal_delay_us() is only called by emb6.c to make a delay multiple of 500us,
   * which is equivalent to 1 systick
   */
  uint32_t tick_stop;

  hal_enterCritical();
  tick_stop  = l_hal_tick;
  tick_stop += delay * MSP430_TICK_SCALER / MSP430_TICK_SECONDS;
  hal_exitCritical();
  while (tick_stop > l_hal_tick) {
    /* do nothing */
  }

  return 0;
} /* hal_delayUs() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinInit()
*/
void* hal_pinInit( en_hal_pin_t pin )
{
  s_hal_gpio_pin_t* p_pin = NULL;
  p_pin = &s_hal_gpio[pin];

  /* configure pin as GPIO */
  *p_pin->PORT->PSEL &= ~p_pin->MSK;

  /* configure pin direction */
  if( p_pin->DIR == MSP430_IO_PIN_DIR_OUTPUT )
    *p_pin->PORT->PDIR |= p_pin->MSK;
  else
    *p_pin->PORT->PDIR &= ~p_pin->MSK;

  return p_pin;
} /* hal_pinInit() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinSet()
*/
int8_t hal_pinSet( void* p_pin, uint8_t val )
{
  s_io_pin_desc_t* p_gpioPin = (s_io_pin_desc_t *)p_pin;

  EMB6_ASSERT_RET( p_gpioPin != NULL, -1 );

  if( val )
  {
    io_set(p_gpioPin);
  }
  else
  {
    io_clear(p_gpioPin);
  }
  return 0;
} /* hal_pinSet() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinGet()
*/
int8_t hal_pinGet( void* p_pin )
{
  s_io_pin_desc_t* p_gpioPin = (s_io_pin_desc_t *)p_pin;

  EMB6_ASSERT_RET( p_gpioPin != NULL, -1 );

  return io_get(p_gpioPin);
} /* hal_pinGet() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQRegister()
*/
int8_t hal_pinIRQRegister( void* p_pin, en_hal_irqedge_t edge,
    pf_hal_irqCb_t pf_cb )
{
  s_io_pin_desc_t* p_gpioPin = (s_io_pin_desc_t *)p_pin;

  EMB6_ASSERT_RET( p_gpioPin != NULL, -1 );

  /* supported external interrupts are treated evenly
   * edge=1: rising; otherwise falling */
  io_extiRegister(p_gpioPin, (edge == EN_HAL_IRQEDGE_RISING) || (edge == EN_HAL_IRQEDGE_EITHER), pf_cb);

  return 0;
} /* hal_pinIRQRegister() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQEnable()
*/
int8_t hal_pinIRQEnable( void* p_pin )
{
  s_io_pin_desc_t* p_gpioPin = (s_io_pin_desc_t *)p_pin;

  EMB6_ASSERT_RET( p_gpioPin != NULL, -1 );

  io_extiEnable( p_gpioPin );
  return 0;
} /* hal_pinIRQEnable() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQDisable()
*/
int8_t hal_pinIRQDisable( void* p_pin )
{
  s_io_pin_desc_t* p_gpioPin = (s_io_pin_desc_t *)p_pin;

  EMB6_ASSERT_RET( p_gpioPin != NULL, -1 );

  io_extiDisable( p_gpioPin );
  return 0;
} /* hal_pinIRQDisable() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQClear()
*/
int8_t hal_pinIRQClear( void* p_pin )
{
  s_io_pin_desc_t* p_gpioPin = (s_io_pin_desc_t *)p_pin;

  EMB6_ASSERT_RET( p_gpioPin != NULL, -1 );

  io_extiClear( p_gpioPin );
  return 0;
} /* hal_pinIRQClear() */

#if defined(HAL_SUPPORT_SPI)
/*---------------------------------------------------------------------------*/
/*
* hal_spiInit()
*/
void* hal_spiInit( en_hal_spi_t spi )
{
  spi_trxInit(4);
  return s_hal_spi_t;
} /* hal_spiInit() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiTRx()
*/
int32_t hal_spiTRx( void* p_spi, uint8_t* p_tx, uint8_t* p_rx, uint16_t len )
{
  EMB6_ASSERT_RET( p_spi != NULL, -1 );
  EMB6_ASSERT_RET( p_spi != &s_hal_spi_t, -1 );

  spi_rfTxRx(p_tx, p_rx, len);
  return len;
} /* hal_spiTRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiRx()
*/
int32_t hal_spiRx( void* p_spi, uint8_t * p_rx, uint16_t len )
{
  EMB6_ASSERT_RET( p_spi != NULL, -1 );
  EMB6_ASSERT_RET( p_spi != &s_hal_spi_t, -1 );

  return spi_rfRead(p_rx, len);
} /* hal_spiRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiTx()
*/
int32_t hal_spiTx( void* p_spi, uint8_t* p_tx, uint16_t len )
{
  EMB6_ASSERT_RET( p_spi != NULL, -1 );
  EMB6_ASSERT_RET( p_spi != &s_hal_spi_t, -1 );

  spi_rfWrite(p_tx, len);
  return len;
} /* hal_spiTx() */
#endif /* #if defined(HAL_SUPPORT_SPI) */

#if defined(HAL_SUPPORT_UART)
/*---------------------------------------------------------------------------*/
/*
* hal_uartInit()
*/
void* hal_uartInit( en_hal_uart_t uart )
{
  uart_init();
  uart_config(MSP430_SLIP_UART, MSP430_SLIP_UART_BAUD, _hal_uartRxCb);
  return &s_hal_uart;
} /* hal_uartInit() */

/*---------------------------------------------------------------------------*/
/*
* hal_uartRx()
*/
int32_t hal_uartRx( void* p_uart, uint8_t * p_rx, uint16_t len )
{
  EMB6_ASSERT_RET( p_uart != &s_hal_uart, -1 );

  /* not supported */
  return -1;
} /* hal_uartRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_uartTx()
*/
int32_t hal_uartTx( void* p_uart, uint8_t* p_tx, uint16_t len )
{
  EMB6_ASSERT_RET( p_uart != &s_hal_uart, -1 );

  return uart_send(MSP430_SLIP_UART, p_tx, len);
} /* hal_uartTx() */
#endif /* #if defined(HAL_SUPPORT_UART) */

/*---------------------------------------------------------------------------*/
/*
* hal_periphIRQRegister()
*/
int8_t hal_periphIRQRegister( en_hal_periphirq_t irq, pf_hal_irqCb_t pf_cb,
    void* p_data )
{
  /* set the callback and data pointer */
  s_hal_irqs[irq].pf_cb = pf_cb;
  s_hal_irqs[irq].p_data = p_data;

  return 0;
} /* hal_periphIRQRegister() */

/*---------------------------------------------------------------------------*/
/*
* hal_debugInit()
*/
int8_t hal_debugInit( void )
{
  /* #1:  check if debugging utility is enabled (i.e., LOGGER_LEVEL > 0) ?
   * #2:  check if debugging channel is available (i.e., UART or Trace) ?
   * #3:  initialize debugging utility if #1 and #2 conditions are met
   */
#if (LOGGER_LEVEL > 0) && (HAL_SUPPORT_SLIPUART == FALSE)
  s_io_pin_desc_t s_pin_rx = {
    &gps_io_port[MSP430_DEBUG_UART_RX_PORT], MSP430_DEBUG_UART_RX_PIN, MSP430_DEBUG_UART_RX_MSK,
  };
  s_io_pin_desc_t s_pin_tx = {
    &gps_io_port[MSP430_DEBUG_UART_TX_PORT], MSP430_DEBUG_UART_TX_PIN, MSP430_DEBUG_UART_TX_MSK,
  };

  /* Set Rx Pin as alternate function and input*/
  *s_pin_rx.PORT->PSEL |= s_pin_rx.MSK;
  *s_pin_rx.PORT->PDIR &= ~s_pin_rx.MSK;

   /* Set Tx Pin as alternate function and output*/
  *s_pin_tx.PORT->PSEL |= s_pin_tx.MSK;
  *s_pin_tx.PORT->PDIR |= s_pin_tx.MSK;

  /* initialize UART */
  uart_init();
  uart_config(MSP430_DEBUG_UART, MSP430_DEBUG_UART_BAUD, NULL);
#endif /* #if (LOGGER_LEVEL > 0) && (HAL_SUPPORT_SLIPUART == FALSE) */
  return 0;
} /* hal_debugInit() */
