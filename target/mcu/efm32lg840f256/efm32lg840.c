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
 *  \file       efm32lg840.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      HAL implementation for the EFM32LG840 micro controller.
 *
 *              This HAL implementation provides the according functions to
 *              make emb::6 run on an EFM32LG840 micro controller.
 */

/*
 *  --- Includes -------------------------------------------------------------*
 */
#include "emb6.h"
#include "hal.h"
#include "emb6assert.h"

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
#if (HAL_SUPPORT_MCU_SLEEP == TRUE)
#include "sleep.h"
#endif /* #if (HAL_SUPPORT_MCU_SLEEP == TRUE) */
#include "rt_tmr.h"


/*
 * --- Macro Definitions --------------------------------------------------- *
 */
#define LOGGER_ENABLE                 LOGGER_HAL
#include "logger.h"

/** number of ticks per second */
#define EFM32_TICK_SECONDS            1000

#if defined(EFM32_LED_ACTIVE_LOW)
#define EFM32_LED_OUT_VAL               0
#else
#define EFM32_LED_OUT_VAL               1
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

/**
 * \brief   Description of an interrupt.
 *
 *          An interrupt consists of the according callback function
 *          and a data pointer.
 */
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
  {EFM32_IO_PORT_LED0, EFM32_IO_PIN_LED0, gpioModePushPull, EFM32_LED_OUT_VAL, NULL}, /* LED0 */
#endif /* #if defined(HAL_SUPPORT_LED0) */
#if defined(HAL_SUPPORT_LED1)
  {EFM32_IO_PORT_LED1, EFM32_IO_PIN_LED1, gpioModePushPull, EFM32_LED_OUT_VAL, NULL}, /* LED1 */
#endif /* #if defined(HAL_SUPPORT_LED1) */
#if defined(HAL_SUPPORT_LED2)
  {EFM32_IO_PORT_LED2, EFM32_IO_PIN_LED2, gpioModePushPull, EFM32_LED_OUT_VAL, NULL}, /* LED2 */
#endif /* #if defined(HAL_SUPPORT_LED2) */
#if defined(HAL_SUPPORT_LED3)
  {EFM32_IO_PORT_LED3, EFM32_IO_PIN_LED3, gpioModePushPull, EFM32_LED_OUT_VAL, NULL}, /* LED3 */
#endif /* #if defined(HAL_SUPPORT_LED3) */
#if defined(HAL_SUPPORT_LED4)
  {EFM32_IO_PORT_LED4, EFM32_IO_PIN_LED4, gpioModePushPull, EFM32_LED_OUT_VAL, NULL}, /* LED4 */
#endif /* #if defined(HAL_SUPPORT_LED4) */

#if defined(HAL_SUPPORT_RFSPI)
  {EFM32_IO_PORT_USART_CLK, EFM32_IO_PIN_USART_CLK, gpioModePushPull, 0, NULL}, /* RF SPI CLK */
  {EFM32_IO_PORT_USART_TX, EFM32_IO_PIN_USART_TX, gpioModePushPull, 0, NULL}, /* RF SPI TX */
  {EFM32_IO_PORT_USART_RX, EFM32_IO_PIN_USART_RX, gpioModeInputPull, 0, NULL}, /* RF SPI RX */
  {EFM32_IO_PORT_USART_CS, EFM32_IO_PIN_USART_CS, gpioModePushPull, 0, NULL}, /* RF SPI CS */
#endif /* #if defined(HAL_SUPPORT_RFSPI) */

#if defined(HAL_SUPPORT_RFCTRL0)
  {EFM32_IO_PORT_RF_CTRL_0, EFM32_IO_PIN_RF_CTRL_0, EFM32_IO_PINMODE_RF_CTRL_0, 0, NULL}, /* RF CTRL 0 */
#endif /* #if defined(HAL_SUPPORT_RFCTRL0) */
#if defined(HAL_SUPPORT_RFCTRL1)
  {EFM32_IO_PORT_RF_CTRL_1, EFM32_IO_PIN_RF_CTRL_1, EFM32_IO_PINMODE_RF_CTRL_1, 1, NULL}, /* RF CTRL 1 */
#endif /* #if defined(HAL_SUPPORT_RFCTRL1) */
#if defined(HAL_SUPPORT_RFCTRL2)
  {EFM32_IO_PORT_RF_CTRL_2, EFM32_IO_PIN_RF_CTRL_2, EFM32_IO_PINMODE_RF_CTRL_2, 1, NULL}, /* RF CTRL 2 DO NOT CHANGE !!! */
#endif /* #if defined(HAL_SUPPORT_RFCTRL2) */

#if defined(HAL_SUPPORT_RFCTRL3) || defined(HAL_SUPPORT_RFCTRL4) || defined(HAL_SUPPORT_RFCTRL5)
#error "RFCTRL configuration is not supported."
#endif /* #if defined(HAL_SUPPORT_RFCTRL3) || defined(HAL_SUPPORT_RFCTRL4) || defined(HAL_SUPPORT_RFCTRL5) */

#if defined(HAL_SUPPORT_SLIPUART)
  {EFM32_SLIP_UART_PORT_USART_TX, EFM32_SLIP_UART_PIN_USART_TX, gpioModePushPull, 0, NULL}, /* UART_TX */
  {EFM32_SLIP_UART_PORT_USART_RX, EFM32_SLIP_UART_PIN_USART_RX, gpioModeInputPull, 0, NULL}, /* UART_RX */
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


#if (HAL_SUPPORT_MCU_SLEEP == TRUE)
/** low-power timer ID */
static RTCDRV_TimerID_t s_hal_lowpowerTimerId;
/** sleep duration in ticks */
static uint32_t l_hal_sleepDuration;
#endif


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */

/* Initialization of the watchdog */
static void _hal_wdcInit( void );

/* Initialization of the tick counter */
static void _hal_tcInit( void );

/* Callback for the Tick Counter */
static void _hal_tcCb( void );

/* Initialization of clocks */
static void _hal_clksInit( void );

#if defined(HAL_SUPPORT_UART)
/* Initialization of uart */
static void _hal_uartInit( s_hal_uart_t* p_uart );
#endif /* #if defined(HAL_SUPPORT_UART) */

/* Common callback for external interrupts */
static void _hal_extiCb( uint8_t pin );

#if (HAL_SUPPORT_MCU_SLEEP == TRUE)
/* callback for real-time clock used in low-power mode */
static void _hal_rtcCallback(RTCDRV_TimerID_t id, void *p_data);
/* callback function to be called as MCU is about to enter sleep mode */
static void _hal_sleepCb(SLEEP_EnergyMode_t mode);
/* callback function to be called as MCU just exits from sleep mode */
static void _hal_wakeupCb(SLEEP_EnergyMode_t mode);
#endif /* #if (HAL_SUPPORT_MCU_SLEEP == TRUE) */


/*
 *  --- Local Functions ---------------------------------------------------- *
 */

/**
 * \brief   Initialization of the watchdog.
 *
 *          This function initializes the watchdog. Therefore it sets the
 *          according timeouts but disableds the watchdog per default.
 */
static void _hal_wdcInit(void)
{
  WDOG_Init_TypeDef wd = {
    FALSE, FALSE, FALSE, FALSE, FALSE, FALSE,
    wdogClkSelLFRCO, wdogPeriod_256k
  };

  /* disable watchdog at initialization*/
  wd.enable = 0;
  WDOG_Init(&wd);

} /* _hal_tcInit() */


/**
 * \brief   Initialization of the tick counter.
 *
 *          The tick counter is used as the internal time reference. This
 *          function initializes the tick counter by configuring the local
 *          timers and the according callback functions to count the ticks.
 */
static void _hal_tcInit(void)
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


/**
 * \brief   Timer callback to increase system ticks.
 *
 *          This callback is used to increase the internal tick
 *          counter. This function is called during the timer interrupt
 *          handler.
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
} /* _isr_tc_interrupt() */


/**
 * \brief   Initialization of the clocks.
 *
 *          EFM controllers use different clocks for the peripherals. This
 *          function enables the required clocks
 */
static void _hal_clksInit( void )
{
  /* enable required clocks */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_TIMER1, true);
#if defined(HAL_SUPPORT_RFSPI)
  CMU_ClockEnable(cmuClock_USART0, true);
#endif /* #if defined(HAL_SUPPORT_RFSPI) */
  CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);
}


#if defined(HAL_SUPPORT_UART)
/**
 *    \brief    Initialize UART.
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
  GPIO_PinModeSet(p_uart->p_txPin->port, p_uart->p_txPin->pin,
      p_uart->p_txPin->mode, p_uart->p_txPin->val);
  GPIO_PinModeSet(p_uart->p_rxPin->port, p_uart->p_rxPin->pin,
      p_uart->p_rxPin->mode, p_uart->p_rxPin->val);

  /* Prepare UART Rx and Tx interrupts */
  USART_IntClear(p_uart->p_hndl, _USART_IFC_MASK);
  USART_IntEnable(p_uart->p_hndl, USART_IEN_RXDATAV);
  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
  NVIC_ClearPendingIRQ(USART0_TX_IRQn);
  NVIC_EnableIRQ(USART0_RX_IRQn);

  /* Enable I/O pins at UART1 location #3 */
  p_uart->p_hndl->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC3;

  /* Enable UART */
  USART_Enable(p_uart->p_hndl, usartEnable);
}
#endif /* #if defined(HAL_SUPPORT_UART) */


/**
 * \brief   External interrupt handler.
 *
 *          The external interrupt handler function is used to trigger
 *          the registered callbacks once an external interrupt occurred.
 *
 * \param   pin   Pin that triggered the interrupt.
 */
static void _hal_extiCb( uint8_t pin )
{
  int i;
  for( i = EN_HAL_PIN_RFCTRL0; i < EN_HAL_PIN_MAX; i++ )
  {
    if( s_hal_gpio[i].pin == pin )
    {
      /*find the correct interrupt occurred */
      if( s_hal_gpio[i].pf_cb)
      {
        /* call the according callback function */
        s_hal_gpio[i].pf_cb( NULL );
        return;
      }
    }
  }
}

#if (HAL_SUPPORT_MCU_SLEEP == TRUE)
/**
 * \brief   callback for real-time clock used in low-power mode
 *
 *          This function is currently used for debugging purpose only
 */
static void _hal_rtcCallback(RTCDRV_TimerID_t id, void *p_data)
{

}


/**
 * \brief   callback function to be called as MCU is about to enter sleep mode
 */
static void _hal_sleepCb(SLEEP_EnergyMode_t mode)
{
  /* turn peripherals off */
  RTCDRV_StartTimer(s_hal_lowpowerTimerId, rtcdrvTimerTypeOneshot,
      l_hal_sleepDuration, _hal_rtcCallback, NULL);
}


/**
 * \brief   callback function to be called as MCU just exits from sleep mode
 */
static void _hal_wakeupCb(SLEEP_EnergyMode_t mode)
{
  uint32_t timeRemain = 0;

  /* compute the actual sleep duration */
  RTCDRV_TimeRemaining(s_hal_lowpowerTimerId, &timeRemain);
  l_hal_sleepDuration -= timeRemain;

  /* force wake-up timer to stop */
  RTCDRV_StopTimer(s_hal_lowpowerTimerId);
}
#endif /* #if (HAL_SUPPORT_MCU_SLEEP == TRUE) */


/**
 * \brief   Timer1 interrupt handler.
 */
void TIMER1_IRQHandler( void )
{
  uint32_t flags;

  flags = TIMER_IntGet(TIMER1);
  _hal_tcCb();
  TIMER_IntClear(TIMER1, flags);
}

#if defined(HAL_SUPPORT_SLIPUART)
#if defined(HAL_SUPPORT_PERIPHIRQ_SLIPUART_RX)
/**
 * \brief   USART0 interrupt handler.
 */
void EFM32_SLIP_UART_RXIRQHNDL(void)
{
  /* Check for RX data valid interrupt */
  if( s_hal_uart.p_hndl->IF & USART_IF_RXDATAV )
  {
    /* Copy data into RX Buffer */
    uint8_t rxData = USART_Rx( s_hal_uart.p_hndl );
    if( s_hal_irqs[EN_HAL_PERIPHIRQ_SLIPUART_RX].pf_cb != NULL )
    {
      s_hal_irqs[EN_HAL_PERIPHIRQ_SLIPUART_RX].pf_cb( &rxData );
    }
  }
}
#endif /* #if defined(HAL_SUPPORT_PERIPHIRQ_SLIPUART_RX) */
#endif /* #if defined(HAL_SUPPORT_SLIPUART) */


/*
 * --- Global Functions ---------------------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* hal_init()
*/
int8_t hal_init (void)
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

#if (HAL_SUPPORT_MCU_SLEEP == TRUE)
  /* initialize RTC driver. */
  SLEEP_Init(_hal_sleepCb, _hal_wakeupCb);
  SLEEP_SleepBlockBegin(sleepEM3);

  /* initialize low-power RTC driver. */
  RTCDRV_Init();
  RTCDRV_AllocateTimer(&s_hal_lowpowerTimerId);
#endif /* #if (HAL_SUPPORT_MCU_SLEEP == TRUE) */

  /* initialize external interrupts */
  GPIOINT_Init();

  /* set priority of IRQs */
  NVIC_SetPriorityGrouping(4);
#if defined(HAL_SUPPORT_SLIPUART)
  NVIC_SetPriority(USART0_RX_IRQn, 0);
  NVIC_SetPriority(USART0_TX_IRQn, 1);
#endif /* #if defined(HAL_SUPPORT_SLIPUART) */
  NVIC_SetPriority(GPIO_EVEN_IRQn, 2);
  NVIC_SetPriority(GPIO_ODD_IRQn, 3);
  NVIC_SetPriority(TIMER1_IRQn, 4);

  return 0;
}/* hal_init() */


/*---------------------------------------------------------------------------*/
/*
* hal_enterCritical()
*/
int8_t hal_enterCritical( void )
{
  /* enter critical section */
  INT_Disable();
  return 0;
} /* hal_enterCritical() */


/*---------------------------------------------------------------------------*/
/*
* hal_exitCritical()
*/
int8_t hal_exitCritical( void )
{
  /* exit critical section */
  INT_Enable();
  return 0;
}/* hal_exitCritical() */


/*---------------------------------------------------------------------------*/
/*
* hal_watchdogStart()
*/
int8_t hal_watchdogStart(void)
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
  WDOG_Enable( true);
  return 0;

} /* hal_watchdogStart() */


/*---------------------------------------------------------------------------*/
/*
* hal_watchdogReset()
*/
int8_t hal_watchdogReset(void)
{
  WDOG_Feed();
  return 0;
} /* hal_watchdogReset() */


/*---------------------------------------------------------------------------*/
/*
* hal_watchdogStop()
*/
int8_t hal_watchdogStop(void)
{
  WDOG_Enable(false);
  return 0;
} /* hal_watchdogStop() */


/*---------------------------------------------------------------------------*/
/*
* hal_getrand()
*/
uint32_t hal_getrand(void)
{
  // TODO implement this function
  return 0;
}/* hal_getrand() */


/*---------------------------------------------------------------------------*/
/*
* hal_getTick()
*/
clock_time_t hal_getTick(void)
{
  return l_hal_tick;
} /* hal_getTick() */


/*---------------------------------------------------------------------------*/
/*
* hal_getSec()
*/
clock_time_t hal_getSec(void)
{
  return l_hal_sec;
} /* hal_getSec() */


/*---------------------------------------------------------------------------*/
/*
* hal_getTRes()
*/
clock_time_t hal_getTRes(void)
{
  return EFM32_TICK_SECONDS;
} /* hal_getTRes() */


/*---------------------------------------------------------------------------*/
/*
* hal_delayUs()
*/
int8_t hal_delayUs( uint32_t delay )
{
  volatile int i = 0, j = 0;
  uint32_t l_ticks = ((SystemCoreClockGet() / 1000000) * delay) / 20;
  for (i = 0; i < l_ticks; i++) {
    j++;  /* prevent for-loop to be removed during optimization */
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
  s_hal_gpio_pin_t* p_gpioPin;
  p_gpioPin = (s_hal_gpio_pin_t *)p_pin;
  if( (p_gpioPin != NULL) && (p_gpioPin->mode != gpioModeInput) &&
      (p_gpioPin->mode != gpioModeInputPull) )
  {
    p_gpioPin->val = val ? 1 : 0;
    if( p_gpioPin->val )
    {
      GPIO_PinOutSet(p_gpioPin->port, p_gpioPin->pin);
    }
    else
    {
      GPIO_PinOutClear(p_gpioPin->port, p_gpioPin->pin);
    }
    return p_gpioPin->val;
  }
  else
    return -1;
} /* hal_pinSet() */


/*---------------------------------------------------------------------------*/
/*
* hal_pinGet()
*/
int8_t hal_pinGet(void *p_pin)
{
  s_hal_gpio_pin_t *p_gpioPin;

  if (p_pin == NULL) {
    return -1;
  }

  p_gpioPin = (s_hal_gpio_pin_t*) p_pin;
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

  if( p_pinGpio == NULL )
    return -1;

  /* register callback function */
  p_pinGpio->pf_cb = pf_cb;
  GPIOINT_CallbackRegister( p_pinGpio->pin, _hal_extiCb );

  /* configure external interrupt GPIOs */
  GPIO_PinModeSet( p_pinGpio->port, p_pinGpio->pin, p_pinGpio->mode, p_pinGpio->val );

  GPIO_IntConfig( p_pinGpio->port, p_pinGpio->pin,
      (edge == EN_HAL_IRQEDGE_RISING) || (edge == EN_HAL_IRQEDGE_EITHER),
      (edge == EN_HAL_IRQEDGE_FALLING) || (edge == EN_HAL_IRQEDGE_EITHER),
      FALSE );

  return 0;
} /* hal_extiRegister() */


/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQEnable()
*/
int8_t hal_pinIRQEnable( void* p_pin )
{
  uint32_t pin_msk;
  s_hal_gpio_pin_t* p_pinGpio = (s_hal_gpio_pin_t*)p_pin;

  if( (p_pinGpio == NULL) || (p_pinGpio->pf_cb == NULL) )
    return -1;

  pin_msk = 1 << (p_pinGpio->pin);
  GPIO_IntEnable(pin_msk);

  return 0;
} /* hal_extiEnable() */


/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQDisable()
*/
int8_t hal_pinIRQDisable( void* p_pin )
{
  uint32_t pin_msk;
  s_hal_gpio_pin_t* p_pinGpio = (s_hal_gpio_pin_t*)p_pin;

  if( (p_pinGpio == NULL) || (p_pinGpio->pf_cb == NULL) )
    return -1;

  pin_msk = 1 << (p_pinGpio->pin);
  GPIO_IntDisable(pin_msk);

  return 0;
} /* hal_pinIRQRegister() */


/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQClear()
*/
int8_t hal_pinIRQClear( void* p_pin )
{
  uint32_t pin_msk;
  s_hal_gpio_pin_t* p_pinGpio = (s_hal_gpio_pin_t*)p_pin;

  if( (p_pinGpio == NULL) || (p_pinGpio->pf_cb == NULL) )
    return -1;

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
  SPIDRV_Init(s_hal_spi.pHndl, &spiInit);

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

  if( (p_SpiDrv == NULL) || (p_tx == NULL) || (p_rx == NULL) )
    return -1;

  for( i = 0; i < len; i++)
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

  if( (p_SpiDrv == NULL) || (p_rx == NULL) )
    return -1;

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

  if( (p_SpiDrv == NULL) || (p_tx == NULL) )
    return -1;

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
}/* hal_uartInit() */



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

  if( len == 0 )
    return 0;

  /* check if data is available */
  if( !(p_uartDrv->p_hndl->STATUS & USART_STATUS_RXDATAV) )
    return 0;

  *p_rx = USART_Rx( p_uartDrv->p_hndl );
  return 1;
}/* hal_uartRx() */


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

  if( len == 0 )
    return 0;

  while( lenTmp-- )
    USART_Tx( p_uartDrv->p_hndl, *p_tx++ );
  return len;
}/* hal_uartTx() */
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
  static USART_TypeDef *p_uartDebug = EFM32_DEBUG_UART;
  USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

  /* enable UART clock */
  CMU_ClockEnable(cmuClock_USART0, true);

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
  GPIO_PinModeSet(EFM32_DEBUG_UART_PORT_USART_TX, EFM32_DEBUG_UART_PIN_USART_TX, gpioModePushPull, 1);
  GPIO_PinModeSet(EFM32_DEBUG_UART_PORT_USART_RX, EFM32_DEBUG_UART_PIN_USART_RX, gpioModeInput, 0);

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

#endif /* #if (LOGGER_LEVEL > 0) && (HAL_SUPPORT_SLIPUART == FALSE) */
  return 0;
}

#if (HAL_SUPPORT_MCU_SLEEP == TRUE)
/*---------------------------------------------------------------------------*/
/*
* hal_sleepDuration()
*/
clock_time_t hal_sleepDuration( void )
{
  return l_hal_sleepDuration;
} /* hal_sleepDuration() */


/*---------------------------------------------------------------------------*/
/*
* hal_sleepEnter()
*/
int8_t hal_sleepEnter( uint32_t duration )
{
  l_hal_sleepDuration = duration;
  SLEEP_Sleep();
  return 0;
} /* hal_sleepEnter() */


/*---------------------------------------------------------------------------*/
/*
* hal_adjustTick()
*/
int8_t hal_adjustTick( uint32_t ticks )
{
  l_hal_tick += ticks;
  return 0;
} /* hal_adjustTick() */
#endif /* #if (HAL_SUPPORT_MCU_SLEEP == TRUE) */

