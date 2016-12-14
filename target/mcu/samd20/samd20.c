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
 *  \brief      HAL implementation template.
 *
 */

/*
 *  --- Includes -------------------------------------------------------------*
 */
#include "hal.h"
#include "rt_tmr.h"
#include "assert.h"

#include <compiler.h>
#include <status_codes.h>

// From module: Delay routines
#include <delay.h>

// From module: EXTINT - External Interrupt (Callback APIs)
#include <extint.h>
#include <extint_callback.h>

// From module: Interrupt management - SAM implementation
#include <interrupt.h>

// From module: PORT - GPIO Pin Control
#include <port.h>

// From module: Part identification macros
#include <parts.h>

// From module: SERCOM
#include <sercom.h>
#include <sercom_interrupt.h>

#if defined(HAL_SUPPORT_RFSPI)
// From module: SERCOM SPI - Serial Peripheral Interface (Polled APIs)
#include <spi.h>
#endif /* #if defined(HAL_SUPPORT_RFSPI) */

#if defined(HAL_SUPPORT_UART) || (LOGGER_LEVEL > 0)
// From module: SERCOM USART - Serial Communications (Polled APIs)
#include <usart.h>
#include "stdio_serial.h"
#endif /* #if defined(HAL_SUPPORT_UART) */

// From module: SYSTEM - Clock Management
#include <clock.h>
#include <gclk.h>

// From module: SYSTEM - Core System Driver
#include <system.h>

// From module: SYSTEM - I/O Pin Multiplexer
#include <pinmux.h>

// From module: SYSTEM - Interrupt Driver
#include <system_interrupt.h>

// From module: WDT - Watchdog Timer (Polled APIs)
#include <wdt.h>

// From module: RTC - Real Time Counter in Count Mode (Callback APIs)
#include <rtc_count.h>
#include <rtc_count_interrupt.h>


/*
 *  --- Macros ------------------------------------------------------------- *
 */
/** Enable/Disable logger */
#define LOGGER_ENABLE                   LOGGER_HAL
#include "logger.h"

/** Number of ticks per second */
#define SAMD20_TICK_SECONDS             ( 1000u )

/** Pin output direction */
#define SAMD20_DIR_OUT                  ( 1 )
/** Pin input direction */
#define SAMD20_DIR_IN                   ( 0 )

/** Pin active state */
#define SAMD20_IO_PIN_ACTIVE            ( 0 )
/** Pin inactive state */
#define SAMD20_IO_PIN_INACTIVE          ( 1 )

/** Pin initial value */
#define SAMD20_IO_PIN_UP                ( 1 )
/** Pin inactive state */
#define SAMD20_IO_PIN_DOWN              ( 0 )

/** Invalid external interrupt line */
#define SAMD20_IO_IRQ_INVALID           ( 0xFF )

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
  /* TODO missing MCU-specific attributes */
  /** Pin */
  uint8_t pin;
  /** Direction */
  uint8_t direction;
  /** Initial value */
  uint8_t value;

  /** MUX position the GPIO pin should be configured to. */
  uint32_t mux;
  /** push-pull setting */
  uint8_t pushPull;
  /** External interrupt line */
  uint8_t channel;
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
  /* TODO missing MCU-specific attributes */

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
  {SAMD20_IO_PIN_LED0, SAMD20_DIR_OUT, SAMD20_IO_PIN_UP, 0, PORT_PIN_PULL_UP, SAMD20_IO_IRQ_INVALID, NULL},
#endif /* #if defined(HAL_SUPPORT_LED0) */
#if defined(HAL_SUPPORT_LED1)
  {SAMD20_IO_PIN_LED1, SAMD20_DIR_OUT, SAMD20_IO_PIN_UP, 0, PORT_PIN_PULL_UP, SAMD20_IO_IRQ_INVALID, NULL},
#endif /* #if defined(HAL_SUPPORT_LED1) */
#if defined(HAL_SUPPORT_LED2)
  {SAMD20_IO_PIN_LED2, SAMD20_DIR_OUT, SAMD20_IO_PIN_UP, 0, PORT_PIN_PULL_UP, SAMD20_IO_IRQ_INVALID, NULL},
#endif /* #if defined(HAL_SUPPORT_LED2) */

  /* TODO missing RF_SPI definition */
#if defined(HAL_SUPPORT_RFSPI)
  {SAMD20_IO_PIN_SPI_CLK, SAMD20_DIR_OUT, SAMD20_IO_PIN_UP, 0, PORT_PIN_PULL_UP, SAMD20_IO_IRQ_INVALID, NULL},
  {SAMD20_IO_PIN_SPI_TX, SAMD20_DIR_OUT, SAMD20_IO_PIN_UP, 0, PORT_PIN_PULL_UP, SAMD20_IO_IRQ_INVALID, NULL},
  {SAMD20_IO_PIN_SPI_RX, SAMD20_DIR_IN, SAMD20_IO_PIN_UP, 0, PORT_PIN_PULL_UP, SAMD20_IO_IRQ_INVALID, NULL},
  {SAMD20_IO_PIN_SPI_CS, SAMD20_DIR_OUT, SAMD20_IO_PIN_UP, 0, PORT_PIN_PULL_UP, SAMD20_IO_IRQ_INVALID, NULL},
#endif /* #if defined(HAL_SUPPORT_RFSPI) */

#if defined(HAL_SUPPORT_RFCTRL0)
  {SAMD20_IO_PIN_RF_RST, SAMD20_DIR_OUT, SAMD20_IO_PIN_UP, 0, PORT_PIN_PULL_UP, SAMD20_IO_IRQ_INVALID, NULL},
#endif /* #if defined(HAL_SUPPORT_RFCTRL0) */
#if defined(HAL_SUPPORT_RFCTRL1)
  {SAMD20_IO_PIN_RF_SLP, SAMD20_DIR_OUT, SAMD20_IO_PIN_UP, 0, PORT_PIN_PULL_UP, SAMD20_IO_IRQ_INVALID, NULL},
#endif /* #if defined(HAL_SUPPORT_RFCTRL1) */
#if defined(HAL_SUPPORT_RFCTRL2)
  {SAMD20_IO_PIN_RF_IRQ, SAMD20_DIR_IN, SAMD20_IO_PIN_UP, 0, PORT_PIN_PULL_UP, SAMD20_IO_PIN_RF_IRQ_CHANNEL, NULL},
#endif /* #if defined(HAL_SUPPORT_RFCTRL2) */

#if defined(HAL_SUPPORT_SLIPUART)
  {SAMD20_SLIP_UART_PIN_TX, SAMD20_DIR_OUT, SAMD20_IO_PIN_UP, 0, PORT_PIN_PULL_UP, SAMD20_IO_IRQ_INVALID, NULL},
  {SAMD20_SLIP_UART_PIN_RX, SAMD20_DIR_IN, SAMD20_IO_PIN_UP, 0, PORT_PIN_PULL_UP, SAMD20_IO_IRQ_INVALID, NULL},
#endif /* #if defined(HAL_SUPPORT_SLIPUART) */

};


#if defined(HAL_SUPPORT_RFSPI)
/** Definition of the SPI interface */
static struct spi_module s_hal_spi;
#endif /* #if defined(HAL_SUPPORT_RFSPI) */


#if defined(HAL_SUPPORT_SLIPUART)
/** Definition of the SLIP UART interface */
static struct usart_module s_hal_uart;
#endif /* #if defined(HAL_SUPPORT_SLIPUART) */


/** Definition of the peripheral callback functions */
static s_hal_irq s_hal_irqs[EN_HAL_PERIPHIRQ_MAX];


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */
static void _hal_wdtInit( void );
static void _hal_tcInit( void );
static void _hal_tcCb( void );

/*
 *  --- Local Functions ---------------------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* _hal_wdtInit()
*/
static void _hal_wdtInit( void )
{
#if 0   /* FIXME outdated implementation */
  struct wdt_conf config_wdt;

  /* use default configuration */
  wdt_get_config_defaults( &config_wdt );

  /* configure watchdog */
  config_wdt.always_on = false;
  config_wdt.clock_source = GCLK_GENERATOR_4;
  config_wdt.timeout_period = WDT_PERIOD_2048CLK;

  /* initialize the Watchdog with the user settings */
  wdt_init( &config_wdt );

  /* enable watchdog */
  wdt_enable();
#endif

} /* _hal_wdtInit() */

/*---------------------------------------------------------------------------*/
/*
* _hal_tcInit()
*/
static void _hal_tcInit( void )
{
  struct rtc_count_config config_rtc_count;
  struct rtc_module rtc_instance;

  /* set to default */
  rtc_count_get_config_defaults( &config_rtc_count );

  /* configure the timer */
  config_rtc_count.prescaler = RTC_COUNT_PRESCALER_DIV_1;
  config_rtc_count.mode = RTC_COUNT_MODE_16BIT;
  config_rtc_count.continuously_update = true;

  /* initialize */
  rtc_count_init( &rtc_instance, RTC, &config_rtc_count );

  /* set period */
  rtc_count_set_period( &rtc_instance, 32 );

  /* register and enable timer callback */
  rtc_count_register_callback( &rtc_instance, _hal_tcCb, RTC_COUNT_CALLBACK_OVERFLOW );
  rtc_count_enable_callback( &rtc_instance, RTC_COUNT_CALLBACK_OVERFLOW );

  /* enable the timer */
  rtc_count_enable( &rtc_instance );

} /* _hal_tcInit() */

/*---------------------------------------------------------------------------*/
/*
* _hal_tcCb()
*/
static void _hal_tcCb( void )
{
  /* Indicate timer update to the emb6 timer */
  hal_enterCritical();
  if (l_hal_tick % SAMD20_TICK_SECONDS == 0) {
    l_hal_sec++;
  }
  l_hal_tick++;
  hal_exitCritical();

  /* update real-time timers*/
  rt_tmr_update();

} /* _hal_tcCb() */


/*
 * --- Global Function Definitions ----------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* hal_init()
*/
int8_t hal_init( void )
{
  /* initialize system */
  system_init();

  /* initialize watchdog */
  _hal_wdtInit();

  /* initialize timer counter */
  _hal_tcInit();

  /* enable system interrupt */
  system_interrupt_enable_global();

  return 0;
} /* hal_init() */

/*---------------------------------------------------------------------------*/
/*
* hal_enterCritical()
*/
int8_t hal_enterCritical( void )
{
  system_interrupt_enter_critical_section();
  return 0;
} /* hal_enterCritical() */

/*---------------------------------------------------------------------------*/
/*
* hal_exitCritical()
*/
int8_t hal_exitCritical( void )
{
  system_interrupt_leave_critical_section();
  return 0;
} /* hal_exitCritical() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogStart()
*/
int8_t hal_watchdogStart( void )
{
  /* FIXME implementation is outdated */
  /* wdt_enable(); */
  return -1;
} /* hal_watchdogStart() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogReset()
*/
int8_t hal_watchdogReset( void )
{
  /* FIXME implementation is outdated */
  /* wdt_reset_count(); */
  return -1;
} /* hal_watchdogReset() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogStop()
*/
int8_t hal_watchdogStop( void )
{
  /* FIXME implementation is outdated */
  /* wdt_disable(); */
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
  return SAMD20_TICK_SECONDS;
} /* hal_getTRes() */

/*---------------------------------------------------------------------------*/
/*
* hal_delayUs()
*/
int8_t hal_delayUs( uint32_t delay )
{
  delay_us( delay );
  return 0;
} /* hal_delayUs() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinInit()
*/
void* hal_pinInit( en_hal_pin_t pin )
{
  s_hal_gpio_pin_t* p_pin = NULL;
  struct port_config pin_conf;

  EMB6_ASSERT_RET( pin < EN_HAL_PIN_MAX, NULL );

  /* set configuration to default */
  port_get_config_defaults( &pin_conf );
  p_pin = &s_hal_gpio[pin];

  /* configure the pin */
  pin_conf.direction = p_pin->direction;
  port_pin_set_config( p_pin->pin, &pin_conf );
  port_pin_set_output_level( p_pin->pin, p_pin->value );

  return p_pin;
} /* hal_pinInit() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinSet()
*/
int8_t hal_pinSet( void* p_pin, uint8_t val )
{
  s_hal_gpio_pin_t* p_gpioPin = (s_hal_gpio_pin_t *)p_pin;

  EMB6_ASSERT_RET( p_gpioPin != NULL, -1 );

  port_pin_set_output_level(p_gpioPin->pin, val);
  return 0;
} /* hal_pinSet() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinGet()
*/
int8_t hal_pinGet( void* p_pin )
{
  s_hal_gpio_pin_t* p_gpioPin = (s_hal_gpio_pin_t *)p_pin;

  EMB6_ASSERT_RET( p_gpioPin != NULL, -1 );

  return port_pin_get_output_level(p_gpioPin->pin);
} /* hal_pinGet() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQRegister()
*/
int8_t hal_pinIRQRegister( void* p_pin, en_hal_irqedge_t edge,
    pf_hal_irqCb_t pf_cb )
{
  struct extint_chan_conf s_chanConfig;
  s_hal_gpio_pin_t* p_gpioPin = (s_hal_gpio_pin_t *)p_pin;

  EMB6_ASSERT_RET( p_gpioPin != NULL, -1 );
  EMB6_ASSERT_RET( p_gpioPin->channel < SAMD20_IO_IRQ_INVALID, -1 );

  hal_enterCritical();

  /* set configuration to default */
  extint_chan_get_config_defaults( &s_chanConfig );

  /* configure */
  s_chanConfig.gpio_pin = p_gpioPin->pin;
  s_chanConfig.gpio_pin_mux = p_gpioPin->mux;
  s_chanConfig.gpio_pin_pull = p_gpioPin->pushPull;
  /* mapping edge detection */
  switch (edge)
  {
    case EN_HAL_IRQEDGE_FALLING:
      s_chanConfig.detection_criteria = EXTINT_DETECT_FALLING;
      break;

    case EN_HAL_IRQEDGE_RISING:
      s_chanConfig.detection_criteria = EXTINT_DETECT_RISING;
      break;

    case EN_HAL_IRQEDGE_EITHER:
      s_chanConfig.detection_criteria = EXTINT_DETECT_BOTH;
      break;
  }

  /* apply the configuration */
  extint_chan_set_config( p_gpioPin->channel, &s_chanConfig );

  /* register interrupt callback */
  extint_register_callback( (extint_callback_t)p_gpioPin->pf_cb,
      p_gpioPin->channel, EXTINT_CALLBACK_TYPE_DETECT );

  hal_exitCritical();

  return 0;
} /* hal_pinIRQRegister() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQEnable()
*/
int8_t hal_pinIRQEnable( void* p_pin )
{
  s_hal_gpio_pin_t* p_gpioPin = (s_hal_gpio_pin_t *)p_pin;

  EMB6_ASSERT_RET( p_gpioPin != NULL, -1 );

  extint_chan_enable_callback( p_gpioPin->channel, EXTINT_CALLBACK_TYPE_DETECT );
  return 0;
} /* hal_pinIRQEnable() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQDisable()
*/
int8_t hal_pinIRQDisable( void* p_pin )
{
  s_hal_gpio_pin_t* p_gpioPin = (s_hal_gpio_pin_t *)p_pin;

  EMB6_ASSERT_RET( p_gpioPin != NULL, -1 );

  extint_chan_disable_callback( p_gpioPin->channel, EXTINT_CALLBACK_TYPE_DETECT );
  return 0;
} /* hal_pinIRQDisable() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQClear()
*/
int8_t hal_pinIRQClear( void* p_pin )
{
  s_hal_gpio_pin_t* p_gpioPin = (s_hal_gpio_pin_t *)p_pin;

  EMB6_ASSERT_RET( p_gpioPin != NULL, -1 );

  extint_chan_clear_detected( p_gpioPin->channel );
  return 0;
} /* hal_pinIRQClear() */

#if defined(HAL_SUPPORT_SPI)
/*---------------------------------------------------------------------------*/
/*
* hal_spiInit()
*/
void* hal_spiInit( en_hal_spi_t spi )
{
  struct spi_slave_inst_config s_slaveConf;
  struct spi_slave_inst s_spiSlave;
  struct spi_config s_masterConf;

  /* configure SPI slave */
  spi_slave_inst_get_config_defaults( &s_slaveConf );
  s_slaveConf.ss_pin = s_hal_gpio[EN_HAL_PIN_RFSPICS].pin;
  spi_attach_slave( &s_spiSlave, &s_slaveConf );

  /* configure SPI master */
  spi_get_config_defaults( &s_masterConf );
  s_masterConf.mux_setting = SAMD20_RFSPI_SERCOM_MUX_SETTING;
  s_masterConf.pinmux_pad0 = SAMD20_RFSPI_SERCOM_PMUX0;
  s_masterConf.pinmux_pad1 = SAMD20_RFSPI_SERCOM_PMUX1;
  s_masterConf.pinmux_pad2 = SAMD20_RFSPI_SERCOM_PMUX2;
  s_masterConf.pinmux_pad3 = SAMD20_RFSPI_SERCOM_PMUX3;

  /* initialize SPI master */
  spi_init( &s_hal_spi, SAMD20_RFSPI_SERCOM, &s_masterConf );

  /* enable SPI master */
  spi_enable( &s_hal_spi );

  return &s_hal_spi;
} /* hal_spiInit() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiTRx()
*/
int32_t hal_spiTRx( void* p_spi, uint8_t* p_tx, uint8_t* p_rx, uint16_t len )
{
  struct spi_module* p_spiDrv = (struct spi_module* )p_spi;
  uint16_t i;

  EMB6_ASSERT_RET( p_spiDrv != NULL, -1 );
  EMB6_ASSERT_RET( len > 0, -1 );

  for (i = 0; i < len; i++)
  {
    if ( STATUS_OK != spi_write( p_spiDrv, p_tx[i] ) )
      return -1;

    if ( STATUS_OK != spi_read( p_spiDrv, (uint16_t *)&p_rx[i] ) )
      return -1;
  }
  return len;
} /* hal_spiTRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiRx()
*/
int32_t hal_spiRx( void* p_spi, uint8_t * p_rx, uint16_t len )
{
  struct spi_module* p_spiDrv = (struct spi_module* )p_spi;

  EMB6_ASSERT_RET( p_spiDrv != NULL, -1 );

  if ( STATUS_OK == spi_read_buffer_wait( p_spiDrv, p_rx, len, 0 ) )
    return len;
  else
    return -1;
} /* hal_spiRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiTx()
*/
int32_t hal_spiTx( void* p_spi, uint8_t* p_tx, uint16_t len )
{
  struct spi_module* p_spiDrv = (struct spi_module* )p_spi;

  EMB6_ASSERT_RET( p_spiDrv != NULL, -1 );

  if ( STATUS_OK == spi_write_buffer_wait( p_spiDrv, p_tx, len ) )
    return len;
  else
    return -1;
} /* hal_spiTx() */
#endif /* #if defined(HAL_SUPPORT_SPI) */

#if defined(HAL_SUPPORT_UART)
/*---------------------------------------------------------------------------*/
/*
* hal_uartInit()
*/
void* hal_uartInit( en_hal_uart_t uart )
{
  struct usart_config s_usartConfig;

  EMB6_ASSERT_RET( uart < EN_HAL_UART_MAX, NULL );

  /* set configuration to default */
  usart_get_config_defaults( &s_usartConfig );

  /* configure UART */
  s_usartConfig.baudrate = SAMD20_SLIP_UART_BAUDRATE;
  s_usartConfig.mux_setting = SAMD20_SLIP_UART_SERCOM_MUX_SETTING;
  s_usartConfig.pinmux_pad0 = SAMD20_SLIP_UART_SERCOM_PMUX0;
  s_usartConfig.pinmux_pad1 = SAMD20_SLIP_UART_SERCOM_PMUX1;
  s_usartConfig.pinmux_pad2 = SAMD20_SLIP_UART_SERCOM_PMUX2;
  s_usartConfig.pinmux_pad3 = SAMD20_SLIP_UART_SERCOM_PMUX3;

  /* initialize UART */
  while( STATUS_OK != usart_init( &s_hal_uart, SAMD20_SLIP_UART_SERCOM, &s_usartConfig ) )
  {
    /* do nothing */
  }

  /* enable UART */
  usart_enable( &s_hal_uart );

  // Initialize Serial Interface using Stdio Library
  //stdio_serial_init( &s_hal_uart, SAMD20_SLIP_UART_SERCOM, &s_usartConfig );

  return NULL;
} /* hal_uartInit() */

/*---------------------------------------------------------------------------*/
/*
* hal_uartRx()
*/
int32_t hal_uartRx( void* p_uart, uint8_t * p_rx, uint16_t len )
{
  struct usart_module* p_uartDrv = (struct usart_module* )p_uart;

  EMB6_ASSERT_RET( p_uartDrv != NULL, -1 );

  if (STATUS_OK == usart_read_buffer_wait(p_uartDrv, p_rx, len))
    return len;
  else
    return -1;
} /* hal_uartRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_uartTx()
*/
int32_t hal_uartTx( void* p_uart, uint8_t* p_tx, uint16_t len )
{
  struct usart_module* p_uartDrv = (struct usart_module* )p_uart;

  EMB6_ASSERT_RET( p_uartDrv != NULL, -1 );

  if (STATUS_OK == usart_write_buffer_wait(p_uartDrv, p_tx, len))
    return len;
  else
    return -1;
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
  /* Is debugging channel is available? As currently only debugging SLIPUART share the same UART channel */
  struct usart_config s_usartConfig;
  static struct usart_module s_hal_debugUart;

  /* set configuration to default */
  usart_get_config_defaults( &s_usartConfig );

  /* configure UART */
  s_usartConfig.baudrate = SAMD20_DEBUG_UART_BAUDRATE;
  s_usartConfig.mux_setting = SAMD20_DEBUG_UART_SERCOM_MUX_SETTING;
  s_usartConfig.pinmux_pad0 = SAMD20_DEBUG_UART_SERCOM_PMUX0;
  s_usartConfig.pinmux_pad1 = SAMD20_DEBUG_UART_SERCOM_PMUX1;
  s_usartConfig.pinmux_pad2 = SAMD20_DEBUG_UART_SERCOM_PMUX2;
  s_usartConfig.pinmux_pad3 = SAMD20_DEBUG_UART_SERCOM_PMUX3;

  /* initialize UART */
  while( STATUS_OK != usart_init( &s_hal_debugUart, SAMD20_DEBUG_UART_SERCOM, &s_usartConfig ) )
  {
    /* do nothing */
  }

  /* enable UART */
  usart_enable( &s_hal_debugUart );

  /* initialize Serial Interface using Stdio Library */
  stdio_serial_init( &s_hal_debugUart, SAMD20_DEBUG_UART_SERCOM, &s_usartConfig );

#endif /* #if (LOGGER_LEVEL > 0) && (HAL_SUPPORT_SLIPUART == FALSE) */
  return 0;
} /* hal_debugInit() */
