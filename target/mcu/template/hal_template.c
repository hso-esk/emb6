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


/*
 *  --- Macros ------------------------------------------------------------- *
 */
/** Enable/Disable logger */
#define LOGGER_ENABLE                   LOGGER_HAL
#include "logger.h"

/** Number of ticks per second */
#define HAL_TEMPLATE_TICK_SECONDS       ( 1000u )


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
/** Definition of the IOs */
static s_hal_gpio_pin_t s_hal_gpio[EN_HAL_PIN_MAX] = {
  /* TODO missing LEDs definition */
#if defined(HAL_SUPPORT_LED0)
  {NULL, NULL, NULL},
#endif /* #if defined(HAL_SUPPORT_LED0) */
#if defined(HAL_SUPPORT_LED1)
  {NULL, NULL, NULL},
#endif /* #if defined(HAL_SUPPORT_LED1) */
#if defined(HAL_SUPPORT_LED2)
  {NULL, NULL, NULL},
#endif /* #if defined(HAL_SUPPORT_LED2) */

  /* TODO missing RF_SPI definition */
#if defined(HAL_SUPPORT_RFSPI)
  {NULL, NULL, NULL},
#endif /* #if defined(HAL_SUPPORT_RFSPI) */

#if defined(HAL_SUPPORT_RFCTRL0)
  {NULL, NULL, NULL},
#endif /* #if defined(HAL_SUPPORT_RFCTRL0) */
#if defined(HAL_SUPPORT_RFCTRL1)
  {NULL, NULL, NULL},
#endif /* #if defined(HAL_SUPPORT_RFCTRL1) */
#if defined(HAL_SUPPORT_RFCTRL2)
  {NULL, NULL, NULL},
#endif /* #if defined(HAL_SUPPORT_RFCTRL2) */

#if defined(HAL_SUPPORT_SLIPUART)
  {NULL, NULL, NULL},
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
static void _hal_wdInit( void );
static void _hal_clkInit( void );
static void _hal_tcInit( void );
static void _hal_tcCb( void );

/*
 *  --- Local Functions ---------------------------------------------------- *
 */


/*
 * --- Global Function Definitions ----------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* hal_init()
*/
int8_t hal_init( void )
{
  return -1;
} /* hal_init() */

/*---------------------------------------------------------------------------*/
/*
* hal_enterCritical()
*/
int8_t hal_enterCritical( void )
{
  return -1;
} /* hal_enterCritical() */

/*---------------------------------------------------------------------------*/
/*
* hal_exitCritical()
*/
int8_t hal_exitCritical( void )
{
  return -1;
} /* hal_exitCritical() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogStart()
*/
int8_t hal_watchdogStart( void )
{
  return -1;
} /* hal_watchdogStart() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogReset()
*/
int8_t hal_watchdogReset( void )
{
  return -1;
} /* hal_watchdogReset() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogStop()
*/
int8_t hal_watchdogStop( void )
{
  return -1;
} /* hal_watchdogStop() */

/*---------------------------------------------------------------------------*/
/*
* hal_getrand()
*/
uint32_t hal_getrand( void )
{
  return 0;
} /* hal_getrand() */

/*---------------------------------------------------------------------------*/
/*
* hal_getTick()
*/
clock_time_t hal_getTick( void )
{
  return 0;
} /* hal_getTick() */

/*---------------------------------------------------------------------------*/
/*
* hal_getSec()
*/
clock_time_t hal_getSec( void )
{
  return 0;
} /* hal_getSec() */

/*---------------------------------------------------------------------------*/
/*
* hal_getTRes()
*/
clock_time_t hal_getTRes( void )
{
  return HAL_TEMPLATE_TICK_SECONDS;
} /* hal_getTRes() */

/*---------------------------------------------------------------------------*/
/*
* hal_delayUs()
*/
int8_t hal_delayUs( uint32_t delay )
{
  return -1;
} /* hal_delayUs() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinInit()
*/
void* hal_pinInit( en_hal_pin_t pin )
{
  return -1;
} /* hal_pinInit() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinSet()
*/
int8_t hal_pinSet( void* p_pin, uint8_t val )
{
  return -1;
} /* hal_pinSet() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinGet()
*/
int8_t hal_pinGet( void* p_pin )
{
  return -1;
} /* hal_pinGet() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQRegister()
*/
int8_t hal_pinIRQRegister( void* p_pin, en_hal_irqedge_t edge,
    pf_hal_irqCb_t pf_cb )
{
  /* Not implemented */
  return -1;
} /* hal_pinIRQRegister() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQEnable()
*/
int8_t hal_pinIRQEnable( void* p_pin )
{
  /* Not implemented */
  return -1;
} /* hal_pinIRQEnable() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQDisable()
*/
int8_t hal_pinIRQDisable( void* p_pin )
{
  /* Not implemented */
  return -1;
} /* hal_pinIRQDisable() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQClear()
*/
int8_t hal_pinIRQClear( void* p_pin )
{
  return -1;
} /* hal_pinIRQClear() */

#if defined(HAL_SUPPORT_SPI)
/*---------------------------------------------------------------------------*/
/*
* hal_spiInit()
*/
void* hal_spiInit( en_hal_spi_t spi )
{
  return NULL;
} /* hal_spiInit() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiTRx()
*/
int32_t hal_spiTRx( void* p_spi, uint8_t* p_tx, uint8_t* p_rx, uint16_t len )
{
  return -1;
} /* hal_spiTRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiRx()
*/
int32_t hal_spiRx( void* p_spi, uint8_t * p_rx, uint16_t len )
{
  return -1;
} /* hal_spiRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiTx()
*/
int32_t hal_spiTx( void* p_spi, uint8_t* p_tx, uint16_t len )
{
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
  return NULL;
} /* hal_uartInit() */

/*---------------------------------------------------------------------------*/
/*
* hal_uartRx()
*/
int32_t hal_uartRx( void* p_uart, uint8_t * p_rx, uint16_t len )
{
  return 0;
} /* hal_uartRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_uartTx()
*/
int32_t hal_uartTx( void* p_uart, uint8_t* p_tx, uint16_t len )
{
  return 0;
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
  return -1;
} /* hal_debugInit() */
