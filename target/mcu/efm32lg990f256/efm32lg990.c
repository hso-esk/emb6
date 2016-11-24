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
 *  \brief      HAL implementation for EFM32LG990 MCU.
 *
 */

/*
 *  --- Includes -------------------------------------------------------------*
 */
#include "hal.h"
#include "assert.h"
#include "rt_tmr.h"

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_wdog.h"
#include "em_int.h"
#include "em_gpio.h"
#include "spidrv.h"
#include "em_usart.h"
#include "em_timer.h"
#include "gpiointerrupt.h"

/*
 *  --- Macros ------------------------------------------------------------- *
 */
/** Enable/Disable logger */
#define LOGGER_ENABLE                   LOGGER_HAL
#include "logger.h"

/** Number of ticks per second */
#ifndef EFM32_TICK_SECONDS
#define EFM32_TICK_SECONDS              ( 1000u )
#endif /* #ifndef EFM32_TICK_SECONDS */

#ifndef EFM32_LED_ACTIVE_LOW
#define EFM32_LED_ACTIVE_LOW            FALSE
#endif /* #ifndef EFM32_LED_ACTIVE_HIGH */

#if EFM32_LED_ACTIVE_LOW
#define EFM32_LED_OUT_VAL               1
#else
#define EFM32_LED_OUT_VAL               0
#endif /* #if EFM32_LED_ACTIVE_LOW */

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
  GPIO_Port_TypeDef port;
  /** Pin */
  uint8_t pin;
  /** Mode */
  GPIO_Mode_TypeDef mode;
  /** Value */
  uint8_t val;
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
  /** handle to the SPI driver */
  SPIDRV_HandleData_t hndl;
  /** handle pointer to the SPI driver */
  SPIDRV_Handle_t pHndl;
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
  /** handle to the UART driver */
  USART_TypeDef* p_hndl;
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

/** Debug Uart instance */
USART_TypeDef *uartStdio = NULL;

/** Definition of the IOs */
static s_hal_gpio_pin_t s_hal_gpio[EN_HAL_PIN_MAX] = {

#if defined(HAL_SUPPORT_LED0)
  {EFM32_IO_PORT_LED0, EFM32_IO_PIN_LED0, gpioModePushPull, EFM32_LED_OUT_VAL, EFM32_LED_ACTIVE_LOW, NULL}, /* LED0 */
#endif /* #if defined(HAL_SUPPORT_LED0) */
#if defined(HAL_SUPPORT_LED1)
  {EFM32_IO_PORT_LED1, EFM32_IO_PIN_LED1, gpioModePushPull, EFM32_LED_OUT_VAL, EFM32_LED_ACTIVE_LOW, NULL}, /* LED1 */
#endif /* #if defined(HAL_SUPPORT_LED1) */
#if defined(HAL_SUPPORT_LED2)
  {EFM32_IO_PORT_LED2, EFM32_IO_PIN_LED2, gpioModePushPull, EFM32_LED_OUT_VAL, EFM32_LED_ACTIVE_LOW, NULL}, /* LED2 */
#endif /* #if defined(HAL_SUPPORT_LED2) */
#if defined(HAL_SUPPORT_LED3)
  {EFM32_IO_PORT_LED3, EFM32_IO_PIN_LED3, gpioModePushPull, EFM32_LED_OUT_VAL, EFM32_LED_ACTIVE_LOW, NULL}, /* LED3 */
#endif /* #if defined(HAL_SUPPORT_LED3) */
#if defined(HAL_SUPPORT_LED4)
  {EFM32_IO_PORT_LED4, EFM32_IO_PIN_LED4, gpioModePushPull, EFM32_LED_OUT_VAL, EFM32_LED_ACTIVE_LOW, NULL}, /* LED4 */
#endif /* #if defined(HAL_SUPPORT_LED4) */

#if defined(HAL_SUPPORT_RFSPI)
  {EFM32_IO_PORT_USART_CLK, EFM32_IO_PIN_USART_CLK, gpioModePushPull, 0, FALSE, NULL}, /* RF SPI CLK */
  {EFM32_IO_PORT_USART_TX, EFM32_IO_PIN_USART_TX, gpioModePushPull, 0, FALSE, NULL}, /* RF SPI TX */
  {EFM32_IO_PORT_USART_RX, EFM32_IO_PIN_USART_RX, gpioModeInputPull, 0, FALSE, NULL}, /* RF SPI RX */
  {EFM32_IO_PORT_USART_CS, EFM32_IO_PIN_USART_CS, gpioModePushPull, 0, FALSE, NULL}, /* RF SPI CS */
#endif /* #if defined(HAL_SUPPORT_RFSPI) */

#if defined(HAL_SUPPORT_RFCTRL0)
  {EFM32_IO_PORT_RF_CTRL_0, EFM32_IO_PIN_RF_CTRL_0, gpioModeInputPull, FALSE, 0, NULL}, /* RF CTRL 0 */
#endif /* #if defined(HAL_SUPPORT_RFCTRL0) */
#if defined(HAL_SUPPORT_RFCTRL1)
  {EFM32_IO_PORT_RF_CTRL_1, EFM32_IO_PIN_RF_CTRL_1, gpioModeInputPull, FALSE, 1, NULL}, /* RF CTRL 1 */
#endif /* #if defined(HAL_SUPPORT_RFCTRL1) */
#if defined(HAL_SUPPORT_RFCTRL2)
  {EFM32_IO_PORT_RF_CTRL_2, EFM32_IO_PIN_RF_CTRL_2, gpioModeInputPull, FALSE, 0, NULL}, /* RF CTRL 2 */
#endif /* #if defined(HAL_SUPPORT_RFCTRL2) */
#if defined(HAL_SUPPORT_RFCTRL3) || defined(HAL_SUPPORT_RFCTRL4) || defined(HAL_SUPPORT_RFCTRL5)
#error "RFCTRL configuration is not supported."
#endif /* #if defined(HAL_SUPPORT_RFCTRL3) || defined(HAL_SUPPORT_RFCTRL4) || defined(HAL_SUPPORT_RFCTRL5) */

#if defined(HAL_SUPPORT_SLIPUART)
  {EFM32_SLIP_UART_PORT_USART_TX, EFM32_SLIP_UART_PIN_USART_TX, gpioModePushPull, 0, FALSE, NULL}, /* RF SPI CLK */
  {EFM32_SLIP_UART_PORT_USART_RX, EFM32_SLIP_UART_PIN_USART_RX, gpioModeInputPull, 0, FALSE, NULL}, /* RF SPI CLK */
#endif /* #if defined(HAL_SUPPORT_SLIPUART) */

};


#if defined(HAL_SUPPORT_RFSPI)
/** Definition of the SPI interface */
static s_hal_spi_t s_hal_spi = {
  .pHndl  =  NULL,
  .p_clkPin = &s_hal_gpio[EN_HAL_PIN_RFSPICLK],
  .p_txPin  = &s_hal_gpio[EN_HAL_PIN_RFSPITX],
  .p_rxPin  = &s_hal_gpio[EN_HAL_PIN_RFSPIRX],
  .p_csPin  = &s_hal_gpio[EN_HAL_PIN_RFSPICS]
};
#endif /* #if defined(HAL_SUPPORT_RFSPI) */


#if defined(HAL_SUPPORT_SLIPUART)
/** Definition of the SPI interface */
static s_hal_uart_t s_hal_uart = {
  .p_hndl  =  EFM32_SLIP_UART,
  .p_txPin  = &s_hal_gpio[EN_HAL_PIN_SLIPUARTTX],
  .p_rxPin  = &s_hal_gpio[EN_HAL_PIN_SLIPUARTRX],
};
#endif /* #if defined(HAL_SUPPORT_SLIPUART) */


/** Definition of the peripheral callback functions */
static s_hal_irq s_hal_irqs[EN_HAL_PERIPHIRQ_MAX];


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */
static void _hal_clksInit( void );
static void _hal_wdcInit( void );
#if defined(HAL_SUPPORT_UART)
static void _hal_uartInit( void );
#endif /* #if defined(HAL_SUPPORT_UART) */
static void _hal_tcInit( void );
static void _hal_tcCb( void );
static void _hal_extiCb( uint8_t pin );

/*
 *  --- Local Functions ---------------------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* _hal_clkInit()
*/
static void _hal_clksInit( void )
{
  /* enable required clocks */
  CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_TIMER1, true);
#if defined(HAL_SUPPORT_RFSPI)
  CMU_ClockEnable(cmuClock_USART0, true);
#endif /* #if defined(HAL_SUPPORT_RFSPI) */

} /* _hal_clkInit() */

/*---------------------------------------------------------------------------*/
/*
* _hal_wdInit()
*/
static void _hal_wdcInit( void )
{
  WDOG_Init_TypeDef wd = {
    FALSE, FALSE, FALSE, FALSE, FALSE, FALSE,
    wdogClkSelLFRCO, wdogPeriod_256k
  };

  /* disable watchdog at initialization*/
  wd.enable = 0;
  WDOG_Init( &wd );

} /* _hal_wdInit() */


#if defined(HAL_SUPPORT_UART)
/*---------------------------------------------------------------------------*/
/*
* _hal_uartInit()
*/
static void _hal_uartInit( s_hal_uart_t* p_uart )
{
  USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

  /* Prepare struct for initializing UART in asynchronous mode*/
  uartInit.enable       = usartDisable;         /* Don't enable UART upon intialization */
  uartInit.refFreq      = 0;                    /* Provide information on reference frequency. When set to 0, the reference frequency is */
  uartInit.baudrate     = EFM32_SLIP_UART_BAUD; /* Baud rate */
  uartInit.oversampling = usartOVS16;           /* Oversampling. Range is 4x, 6x, 8x or 16x */
  uartInit.databits     = usartDatabits8;       /* Number of data bits. Range is 4 to 10 */
  uartInit.parity       = usartNoParity;        /* Parity mode */
  uartInit.stopbits     = usartStopbits1;       /* Number of stop bits. Range is 0 to 2 */
  uartInit.mvdis        = false;                /* Disable majority voting */
  uartInit.prsRxEnable  = false;                /* Enable USART Rx via Peripheral Reflex System */
  uartInit.prsRxCh      = usartPrsRxCh0;        /* Select PRS channel if enabled */

  /* Initialize USART with uartInit struct */
  USART_InitAsync(p_uart->p_hndl, &uartInit);

  /* Configure GPIO pins */
  GPIO_PinModeSet(p_uart->p_txPin->port, p_uart->p_txPin->pin, p_uart->p_txPin->mode, p_uart->p_txPin->val);
  GPIO_PinModeSet(p_uart->p_rxPin->port, p_uart->p_rxPin->pin, p_uart->p_rxPin->mode, p_uart->p_rxPin->val);

  /* Prepare UART Rx and Tx interrupts */
  USART_IntClear(p_uart->p_hndl, _USART_IFC_MASK);
  USART_IntEnable(p_uart->p_hndl, USART_IEN_RXDATAV);
  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
  NVIC_ClearPendingIRQ(USART0_TX_IRQn);
  NVIC_EnableIRQ(USART0_RX_IRQn);

  /* Enable I/O pins at default location */
  p_uart->p_hndl->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | EFM32_SLIP_UART_LOC;

  /* Enable UART */
  USART_Enable(p_uart->p_hndl, usartEnable);

} /* _hal_uartInit() */
#endif /* #if defined(HAL_SUPPORT_UART) */

/*---------------------------------------------------------------------------*/
/*
* _hal_tcInit()
*/
static void _hal_tcInit( void )
{
  uint32_t l_ticks;
  TIMER_TypeDef* ps_timer = TIMER1;
  TIMER_Init_TypeDef s_timerInit = TIMER_INIT_DEFAULT;

  s_timerInit.enable = true;
  s_timerInit.prescale = timerPrescale2;
  s_timerInit.riseAction = timerInputActionReloadStart;

  /* Calculate ticks */
  l_ticks = SystemHFClockGet() / 2 / EFM32_TICK_SECONDS;

  /* configure timer for 1ms */
  TIMER_TopSet(ps_timer, l_ticks);
  /* enable timer interrupts */
  NVIC_DisableIRQ(TIMER1_IRQn);
  NVIC_ClearPendingIRQ(TIMER1_IRQn);
  NVIC_EnableIRQ(TIMER1_IRQn);
  TIMER_IntEnable(ps_timer, TIMER_IEN_OF);

  /* initialize and start timer */
  TIMER_Init(ps_timer, &s_timerInit);

} /* _hal_tcInit() */

/*---------------------------------------------------------------------------*/
/*
* _hal_tcCb()
*/
static void _hal_tcCb( void )
{
  /* Indicate timer update to the emb6 timer */
  hal_enterCritical();
  if (l_hal_tick % EFM32_TICK_SECONDS == 0) {
    l_hal_sec++;
  }
  l_hal_tick++;
  hal_exitCritical();

  /* update real-time timers*/
  rt_tmr_update();

} /* _hal_tcCb() */

/*---------------------------------------------------------------------------*/
/*
* _hal_extiCb()
*/
static void _hal_extiCb( uint8_t pin )
{
  int i;
  for( i = 0; i < EN_HAL_PIN_MAX; i++ )
  {
    if( s_hal_gpio[i].pin == pin )
    {
      /* find the correct interrupt occurred */
      if( s_hal_gpio[i].pf_cb )
      {
        /* call the according callback function */
        s_hal_gpio[i].pf_cb( NULL );
      }
    }
  }

} /* _hal_extiCb() */

/*---------------------------------------------------------------------------*/
/*
* TIMER1_IRQHandler()
*/
void TIMER1_IRQHandler( void )
{
  uint32_t flags;

  flags = TIMER_IntGet(TIMER1);
  _hal_tcCb();
  TIMER_IntClear(TIMER1, flags);

} /* TIMER1_IRQHandler() */


/*
 * --- Global Function Definitions ----------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* hal_init()
*/
int8_t hal_init( void )
{
  /* reset callbacks */
  memset( s_hal_irqs, 0, sizeof(s_hal_irqs) );

  /* initialize Chip */
  CHIP_Init();

  /* initialize clocks */
  _hal_clksInit();

  /* initialize watchdog */
  _hal_wdcInit();

  /* initialize SysTicks */
  _hal_tcInit();

  /* initialize external interrupts */
  GPIOINT_Init();

  return 0;
} /* hal_init() */

/*---------------------------------------------------------------------------*/
/*
* hal_enterCritical()
*/
int8_t hal_enterCritical( void )
{
  INT_Disable();
  return 0;
} /* hal_enterCritical() */

/*---------------------------------------------------------------------------*/
/*
* hal_exitCritical()
*/
int8_t hal_exitCritical( void )
{
  INT_Enable();
  return 0;
} /* hal_exitCritical() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogStart()
*/
int8_t hal_watchdogStart( void )
{
  WDOG_Init_TypeDef wdog_init;
  /* start it automatically after the initialization */
  wdog_init.enable = 1;
  /* don't run it during debugging */
  wdog_init.debugRun = 1;
  /* run it at EM2 energy mode */
  wdog_init.em2Run = 1;
  /* run it at EM3 energy mode */
  wdog_init.em3Run = 1;
  /* don't block the MCU from entering to EM4 energy mode (shutdown) */
  wdog_init.em4Block = 0;
  /* don't block software from disabling of LF oscillators */
  wdog_init.swoscBlock = 0;
  /* don't lock the wdog configuration - possible to change the wdog configuration at run time */
  wdog_init.lock = 0;
  /* chose the clock source for the wdog
   if it have to work at EM3 - choose Ultra Low Energy Clock: 1KHz
   for example I choose the Low Energy Clock: 32.768 KHz  */
  wdog_init.clkSel = wdogClkSelLFXO;
  /* choose period for wdog  */
  wdog_init.perSel = wdogPeriod_64k;
  /*
   In this case, I configured the wdog to reset the MCU each 2 seconds :
   Formula -> Time = (1/Clock) * period
   2 sec = (1/32768) * 65537
   */

  /* So, lets init it:  */
  WDOG_Init(&wdog_init);

  /* enable */
  WDOG_Enable(true);

  return 0;
} /* hal_watchdogStart() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogReset()
*/
int8_t hal_watchdogReset( void )
{
  WDOG_Feed();
  return 0;
} /* hal_watchdogReset() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogStop()
*/
int8_t hal_watchdogStop( void )
{
  WDOG_Enable( FALSE );
  return 0;
} /* hal_watchdogStop() */

/*---------------------------------------------------------------------------*/
/*
* hal_getrand()
*/
uint32_t hal_getrand( void )
{
  /* TODO missing implementation */
  return 0;
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
  return EFM32_TICK_SECONDS;
} /* hal_getTRes() */

/*---------------------------------------------------------------------------*/
/*
* hal_delayUs()
*/
int8_t hal_delayUs( uint32_t delay )
{
  volatile int i = 0;
  uint32_t l_ticks = ((SystemCoreClockGet() / 1000000) * delay) / 17;
  for (i = 0; i < l_ticks; i++) {
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

  /* configure the pin */
  GPIO_PinModeSet(p_pin->port, p_pin->pin, p_pin->mode, p_pin->val);
  GPIO_PinOutSet(p_pin->port, p_pin->pin);

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

  if( (p_gpioPin != NULL) && (p_gpioPin->mode != gpioModeInput) &&
      (p_gpioPin->mode != gpioModeInputPull) )
  {
    p_gpioPin->val = val ? 1 : 0;
    if( p_gpioPin->val )
      GPIO_PinOutSet(p_gpioPin->port, p_gpioPin->pin);
    else
      GPIO_PinOutClear(p_gpioPin->port, p_gpioPin->pin);

    return p_gpioPin->val;
  }
  else
    return -1;

} /* hal_pinSet() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinGet()
*/
int8_t hal_pinGet( void* p_pin )
{
  s_hal_gpio_pin_t *p_gpioPin = (s_hal_gpio_pin_t*) p_pin;

  EMB6_ASSERT_RET( p_gpioPin != NULL, -1 );

  if( (p_gpioPin->mode != gpioModeInput) || (p_gpioPin->mode != gpioModeInputPull) )
    return p_gpioPin->val;
  else
    return GPIO_PinInGet(p_gpioPin->port, p_gpioPin->pin);

} /* hal_pinGet() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQRegister()
*/
int8_t hal_pinIRQRegister( void* p_pin, en_hal_irqedge_t edge,
    pf_hal_irqCb_t pf_cb )
{
  s_hal_gpio_pin_t* p_pinGpio = (s_hal_gpio_pin_t*)p_pin;

  EMB6_ASSERT_RET( p_pinGpio != NULL, -1 );

  /* register callback function */
  p_pinGpio->pf_cb = pf_cb;
  GPIOINT_CallbackRegister( p_pinGpio->pin, _hal_extiCb );

  /* configure external interrupt GPIOs */
  GPIO_PinModeSet( p_pinGpio->port, p_pinGpio->pin, p_pinGpio->mode, p_pinGpio->val );

  GPIO_IntConfig( p_pinGpio->port, p_pinGpio->pin,
      (edge == EN_HAL_IRQEDGE_RISING) || (edge == EN_HAL_IRQEDGE_EITHER),
      (edge == EN_HAL_IRQEDGE_RISING) || (edge == EN_HAL_IRQEDGE_EITHER),
      FALSE );

  return 0;
} /* hal_pinIRQRegister() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQEnable()
*/
int8_t hal_pinIRQEnable( void* p_pin )
{
  uint32_t pin_msk;
  s_hal_gpio_pin_t* p_pinGpio = (s_hal_gpio_pin_t*)p_pin;

  EMB6_ASSERT_RET( p_pinGpio != NULL, -1 );
  EMB6_ASSERT_RET( p_pinGpio->pf_cb != NULL, -1 );

  pin_msk = 1 << (p_pinGpio->pin);
  GPIO_IntEnable(pin_msk);

  return 0;
} /* hal_pinIRQEnable() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQDisable()
*/
int8_t hal_pinIRQDisable( void* p_pin )
{
  uint32_t pin_msk;
  s_hal_gpio_pin_t* p_pinGpio = (s_hal_gpio_pin_t*)p_pin;

  EMB6_ASSERT_RET( p_pinGpio != NULL, -1 );
  EMB6_ASSERT_RET( p_pinGpio->pf_cb != NULL, -1 );

  pin_msk = 1 << (p_pinGpio->pin);
  GPIO_IntDisable(pin_msk);

  return 0;
} /* hal_pinIRQDisable() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQClear()
*/
int8_t hal_pinIRQClear( void* p_pin )
{
  uint32_t pin_msk;
  s_hal_gpio_pin_t* p_pinGpio = (s_hal_gpio_pin_t*)p_pin;

  EMB6_ASSERT_RET( p_pinGpio != NULL, -1 );
  EMB6_ASSERT_RET( p_pinGpio->pf_cb != NULL, -1 );

  pin_msk = 1 << (p_pinGpio->pin);
  GPIO_IntClear(pin_msk);

  return 0;
} /* hal_pinIRQClear() */

#if defined(HAL_SUPPORT_SPI)
/*---------------------------------------------------------------------------*/
/*
* hal_spiInit()
*/
void* hal_spiInit( en_hal_spi_t spi )
{
  /* configure SPI */
  SPIDRV_Init_t spiInit = EFM32_RFSPI_USART;
  spiInit.portLocation = EFM32_RFSPI_USART_LOC;
  spiInit.csControl = spidrvCsControlApplication;

  /* configure SPI clock frequency of 4MHz */
  spiInit.bitRate = 4000000;

  /* initialize SPI */
  s_hal_spi.pHndl = &s_hal_spi.hndl;
  SPIDRV_Init( s_hal_spi.pHndl, &spiInit );

  /* configure manual chip select pin */
  GPIO_PinModeSet( s_hal_spi.p_csPin->port, s_hal_spi.p_csPin->pin, s_hal_spi.p_csPin->mode, s_hal_spi.p_csPin->val );
  /* set chip select pin */
  GPIO_PinOutSet( s_hal_spi.p_csPin->port, s_hal_spi.p_csPin->pin );

  return &s_hal_spi;
} /* hal_spiInit() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiTRx()
*/
int32_t hal_spiTRx( void* p_spi, uint8_t* p_tx, uint8_t* p_rx, uint16_t len )
{
  int i;
  s_hal_spi_t* p_SpiDrv = (s_hal_spi_t*) p_spi;

  EMB6_ASSERT_RET( p_SpiDrv != NULL, -1 );
  EMB6_ASSERT_RET( p_tx != NULL, -1 );
  EMB6_ASSERT_RET( p_rx != NULL, -1 );

  for( i = 0; i < len; i++ )
    p_rx[i] = USART_SpiTransfer(p_SpiDrv->pHndl->initData.port, p_tx[i]);

  return len;
} /* hal_spiTRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiRx()
*/
int32_t hal_spiRx( void* p_spi, uint8_t * p_rx, uint16_t len )
{
  int i;
  s_hal_spi_t* p_SpiDrv = (s_hal_spi_t*) p_spi;

  EMB6_ASSERT_RET( p_SpiDrv != NULL, -1 );
  EMB6_ASSERT_RET( p_rx != NULL, -1 );

  for( i = 0; i < len; i++) {
    p_rx[i] = USART_SpiTransfer(p_SpiDrv->pHndl->initData.port, 0xff);
  }
  return len;
} /* hal_spiRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiTx()
*/
int32_t hal_spiTx( void* p_spi, uint8_t* p_tx, uint16_t len )
{
  int i;
  s_hal_spi_t* p_SpiDrv = (s_hal_spi_t*) p_spi;

  EMB6_ASSERT_RET( p_SpiDrv != NULL, -1 );
  EMB6_ASSERT_RET( p_tx != NULL, -1 );

  for( i = 0; i < len; i++) {
    USART_SpiTransfer(p_SpiDrv->pHndl->initData.port, p_tx[i]);
  }

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
  EMB6_ASSERT_RET( uart < EN_HAL_UART_MAX, NULL );

  /* Call UART initialization function with the according
   * UART instance */
  _hal_uartInit( &s_hal_uart );
  return &s_hal_uart;
} /* hal_uartInit() */

/*---------------------------------------------------------------------------*/
/*
* hal_uartRx()
*/
int32_t hal_uartRx( void* p_uart, uint8_t * p_rx, uint16_t len )
{
  s_hal_uart_t* p_uartDrv;
  p_uartDrv = (s_hal_uart_t*)p_uart;

  EMB6_ASSERT_RET( p_uartDrv != NULL, -1 );
  EMB6_ASSERT_RET( p_rx != NULL, -1 );
  EMB6_ASSERT_RET( len > 0, 0 );

  /* check if data is available */
  if( !(p_uartDrv->p_hndl->STATUS & USART_STATUS_RXDATAV) )
    return 0;

  *p_rx = USART_Rx( p_uartDrv->p_hndl );
  return 1;
} /* hal_uartRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_uartTx()
*/
int32_t hal_uartTx( void* p_uart, uint8_t* p_tx, uint16_t len )
{
  uint16_t lenTmp = len;
  s_hal_uart_t* p_uartDrv;
  p_uartDrv = (s_hal_uart_t*)p_uart;

  EMB6_ASSERT_RET( p_uartDrv != NULL, -1 );
  EMB6_ASSERT_RET( p_tx != NULL, -1 );
  EMB6_ASSERT_RET( len > 0, 0 );

  while( lenTmp-- )
    USART_Tx( p_uartDrv->p_hndl, *p_tx++ );

  return len;
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
  USART_TypeDef *p_uartDebug = EFM32_DEBUG_UART;
  USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

  /* Prepare struct for initializing UART in asynchronous mode*/
  uartInit.enable       = usartDisable;           /* Don't enable UART upon initialization */
  uartInit.refFreq      = 0;                      /* Provide information on reference frequency. When set to 0, the reference frequency is */
  uartInit.baudrate     = EFM32_DEBUG_UART_BAUD;  /* Baud rate */
  uartInit.oversampling = usartOVS16;             /* Oversampling. Range is 4x, 6x, 8x or 16x */
  uartInit.databits     = usartDatabits8;         /* Number of data bits. Range is 4 to 10 */
  uartInit.parity       = usartNoParity;          /* Parity mode */
  uartInit.stopbits     = usartStopbits1;         /* Number of stop bits. Range is 0 to 2 */
  uartInit.mvdis        = false;                  /* Disable majority voting */
  uartInit.prsRxEnable  = false;                  /* Enable USART Rx via Peripheral Reflex System */
  uartInit.prsRxCh      = usartPrsRxCh0;          /* Select PRS channel if enabled */

  /* Initialize USART with uartInit struct */
  USART_InitAsync(p_uartDebug, &uartInit);

  /* Configure GPIO pins */
  GPIO_PinModeSet(gpioPortE, 13, gpioModePushPull, 1);
  GPIO_PinModeSet(gpioPortE, 12, gpioModeInput, 0);

  /* Prepare UART Rx and Tx interrupts */
  USART_IntClear(p_uartDebug, _USART_IFC_MASK);
  USART_IntEnable(p_uartDebug, USART_IEN_RXDATAV);
  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
  NVIC_ClearPendingIRQ(USART0_TX_IRQn);
  NVIC_EnableIRQ(USART0_RX_IRQn);

  /* Enable I/O pins at UART1 location #3 */
  p_uartDebug->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | EFM32_DEBUG_UART_LOC;

  /* Enable UART */
  USART_Enable(p_uartDebug, usartEnable);

  /* set external UART instance */
  uartStdio = p_uartDebug;
  /* disable STDIO buffer */
  setvbuf(stdout , NULL , _IONBF , 0);

  return 0;
} /* hal_debugInit() */
