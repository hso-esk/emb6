/*!
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file       cc13xx.c
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
 * @author     STACKFORCE
 * @brief      This is the 6lowpan-stack driver for the cc13xx mcu.
 */

/*! @defgroup emb6_mcu emb6 stack mcu driver
    This group is the mcu driver for the emb6 stack.
  @{  */

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <target_conf.h>

#include "emb6.h"
#include "hal.h"
#include "bsp.h"
#include "board_conf.h"

#include "sf_mcu.h"
#include "sf_mcu_timer.h"
#include "sf_uart.h"
#include "driverlib/interrupt.h"
#include "rt_tmr.h"

#if USE_TI_RTOS
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>
#include <xdc/runtime/System.h>
#else
#include "bsp/srf06eb_cc26xx/drivers/source/bsp_led.h"
#endif


/*! Enable or disable logging. */
#define     LOGGER_ENABLE        LOGGER_HAL
#include    "logger.h"

#ifndef __TI_ARM__
#warning stdout is not redirected!
#endif /* __TI_ARM__ */

/*============================================================================*/
/*                               MACROS                                       */
/*============================================================================*/
 /*! Systicks per second. */
#define TARGET_CFG_SYSTICK_RESOLUTION           (clock_time_t)( 1000u )
 /*! Timer scaler to get systicks */
#define TARGET_CFG_SYSTICK_SCALER               (clock_time_t)(    2u )

/*! Defines the mcu ticks per second. */
#define MCU_TICKS_PER_SECOND                    2000U
/*! Status for succeeded init functions. */
#define MCU_INIT_STATUS_OK                      0x00U
/*! Compares X with @ref MCU_INIT_STATUS_OK. */
#define MCU_INIT_RET_STATUS_CHECK(X)            ( X == MCU_INIT_STATUS_OK )

/*============================================================================*/
/*                                ENUMS                                       */
/*============================================================================*/

/*============================================================================*/
/*                  STRUCTURES AND OTHER TYPEDEFS                             */
/*============================================================================*/

/*============================================================================*/
/*                       LOCAL FUNCTION PROTOTYPES                            */
/*============================================================================*/
static int8_t _hal_uart_init();
static int8_t _hal_systick(void);
static void _hal_isrSysTick(uint32_t l_count);
/*============================================================================*/
/*                           LOCAL VARIABLES                                  */
/*============================================================================*/

/*! Hal tick counter */
static clock_time_t volatile hal_ticks;


/**
 * \brief   Description of a single Pin.
 *
 *          A pin consists of several attributes such as its port
 *          and pin numbers and the output mode and IRQ callbacks.
 */
typedef struct
{
  /** Pin */
  uint8_t pin;
  /** Value */
  uint8_t val;
  /** IRQ callback */
  pf_hal_irqCb_t pf_cb;
} s_hal_gpio_pin_t;
/*
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

/** Definition of the IOs */
static s_hal_gpio_pin_t s_hal_gpio[EN_HAL_PIN_MAX] = {
#if defined(HAL_SUPPORT_LED0)
  {CC1310_LED0, 0, NULL}, /* LED0 */
#endif /* #if defined(HAL_SUPPORT_LED0) */
#if defined(HAL_SUPPORT_LED1)
  {CC1310_LED1, 0, NULL}, /* LED1 */
#endif /* #if defined(HAL_SUPPORT_LED1) */
#if defined(HAL_SUPPORT_LED2)
  {CC1310_LED2, 0, NULL}, /* LED2 */
#endif /* #if defined(HAL_SUPPORT_LED2) */
#if defined(HAL_SUPPORT_LED3)
  {CC1310_LED3, 0, NULL}, /* LED3 */
#endif /* #if defined(HAL_SUPPORT_LED3) */
};
/** Definition of the peripheral callback functions */
s_hal_irq s_hal_irqs[EN_HAL_PERIPHIRQ_MAX];

#if USE_TI_RTOS

#define Board_LED0          IOID_25
#define Board_LED1          IOID_27
#define Board_LED2          IOID_7
#define Board_LED3          IOID_6

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config ledPinTable[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

#endif

/*============================================================================*/
/*                           LOCAL FUNCTIONS                                  */
/*============================================================================*/

#ifdef __TI_ARM__
/* The functions fputc and fputs are used to redirect stdout to
 * the UART interface.
 *
 * This functinality can only be used with the __TI_ARM__ compiler! */

/* fputc writes a single character to the UART interface. */
int fputc(int _c, register FILE *_fp)
{
  sf_uart_write((uint8_t*)&_c, 0x01U);
  return (unsigned char)_c;
}

/* fputs writes a string with a defined length to the interface. */
int fputs(const char *_ptr, register FILE *_fp)
{
  uint16_t i_len = strlen(_ptr);
  sf_uart_write((uint8_t*)_ptr, i_len);

  return i_len;
}
#endif /* __TI_ARM__ */

/*!
 * @brief This function initializes the UART interface.
 *
 * @return 0 for success, else initialization failed.
 */
int8_t _hal_uart_init()
{
  int si_retStaus = 0;
  bool b_return = false;

  uint8_t p_data[] = { "\r\n\r\n========== BEGIN ===========\r\n\r\n" };
  uint16_t i_dataLen = sizeof(p_data);

  /* Initialize UART */
  b_return = sf_uart_init();

  if (b_return) {
    if (!(sf_uart_write(p_data, i_dataLen) == i_dataLen))
    {
        si_retStaus = 1;
    }
  }

  return si_retStaus;
}

/*!
 * @brief This function updates the systicks.
 *
 * It is used as a callback function for the timer module.
 *
 * @param l_count Unused parameter (API compatibility to sf_mcu_timer).
 */
static void _hal_isrSysTick(uint32_t l_count)
{
  /* Increase ticks */
  hal_ticks++;

  /* Check if the timer has to be updated */
  if ((hal_ticks % TARGET_CFG_SYSTICK_SCALER ) == 0)
  {
    rt_tmr_update();
  }
}

/*!
 * @brief This function functions enables to update the systicks.
 *
 * @return 0 for success, else > 0.
 */
static int8_t _hal_systick(void)
{
  int8_t sc_retStatus = 1;
  bool b_successCheck = false;

  /* Configure the timer to call _hal_isrSysTick */
  b_successCheck = sf_mcu_timer_setCallback((fp_mcu_timer_cb) &_hal_isrSysTick);
  if(b_successCheck)
  {
    /* Enable the timer */
    sf_mcu_timer_enable();
    /* Function successful. */
    sc_retStatus = 0;
  }
  return sc_retStatus;
}


/*!
 * @brief  UART RX call back function.
 *
 * @param  received char
 */
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

/*==============================================================================
                             API FUNCTIONS
 ==============================================================================*/

/*============================================================================*/
/* hal_enterCritical() */
/*============================================================================*/
int8_t hal_enterCritical(void)
{
  /* Disable the interrutps */
  sf_mcu_interruptDisable();
  return 0;
} /* hal_enterCritical() */

/*============================================================================*/
/* hal_exitCritical() */
/*============================================================================*/
int8_t hal_exitCritical(void)
{
  /* Enbale the interrupts */
  sf_mcu_interruptEnable();
  return 0;
}/* hal_exitCritical() */

/*============================================================================*/
/* hal_init() */
/*============================================================================*/
int8_t hal_init(void)
{
  uint8_t c_retStatus = 0U;
  bool b_functionStatus = true;

#if USE_TI_RTOS
  /* Open LED pins */
  ledPinHandle = PIN_open(&ledPinState, ledPinTable);
  if(!ledPinHandle) {
      System_abort("Error initializing board LED pins\n");
  }
#endif

  /* Initialize the mcu */
  b_functionStatus = sf_mcu_init();
  if(b_functionStatus)
  {
    #if USE_TI_RTOS==0U
    /* Initialize the timer. MCU_TICKS_PER_SECOND specifies
     * the tick interval.
     * If TI_RTOS is used, then the timer initialization should be done in the main function. */
    b_functionStatus = sf_mcu_timer_init(MCU_TICKS_PER_SECOND);
    #endif

    if(!b_functionStatus)
      return -1;
    c_retStatus = _hal_systick();
    if(!MCU_INIT_RET_STATUS_CHECK(c_retStatus))
      return -1;
  }

  /* Initialize hal_ticks */
  hal_ticks = 0x00U;

  return 0U;
}/* hal_init() */

/*============================================================================*/
/* hal_getrand() */
/*============================================================================*/
uint32_t hal_getrand(void)
{
  return 1;
}/* hal_getrand() */

/*============================================================================*/
/* hal_ledOff() */
/*============================================================================*/
void hal_ledOff(uint16_t ui_led)
{
  switch( ui_led )
  {
  case CC1310_LED0:
#if USE_TI_RTOS
    PIN_setOutputValue(ledPinHandle, Board_LED0, 0);
#else
    bspLedClear( BSP_LED_1 );
#endif
  break;
  case CC1310_LED1:
#if USE_TI_RTOS
    PIN_setOutputValue(ledPinHandle, Board_LED1, 0);
#else
    bspLedClear( BSP_LED_2 );
#endif
  break;
  case CC1310_LED2:
#if USE_TI_RTOS
    PIN_setOutputValue(ledPinHandle, Board_LED2, 0);
#else
    bspLedClear( BSP_LED_3 );
#endif
  break;
  case CC1310_LED3:
#if USE_TI_RTOS
    PIN_setOutputValue(ledPinHandle, Board_LED3, 0);
#else
    bspLedClear( BSP_LED_4 );
#endif
  break;
 }
}/* hal_ledOff() */

/*============================================================================*/
/* hal_ledOn() */
/*============================================================================*/
void hal_ledOn(uint16_t ui_led)
{
  switch( ui_led )
  {
  case CC1310_LED0:
#if USE_TI_RTOS
    PIN_setOutputValue(ledPinHandle, Board_LED0, 1);
#else
    bspLedClear( BSP_LED_1 );
#endif
  break;
  case CC1310_LED1:
#if USE_TI_RTOS
    PIN_setOutputValue(ledPinHandle, Board_LED1, 1);
#else
    bspLedClear( BSP_LED_2 );
#endif
  break;
  case CC1310_LED2:
#if USE_TI_RTOS
    PIN_setOutputValue(ledPinHandle, Board_LED2, 1);
#else
    bspLedClear( BSP_LED_3 );
#endif
break;
  case CC1310_LED3:
#if USE_TI_RTOS
    PIN_setOutputValue(ledPinHandle, Board_LED3, 1);
#else
    bspLedClear( BSP_LED_4 );
#endif
  break;
}
}/* hal_ledOn() */

/*============================================================================*/
/* hal_pinInit() */
/*============================================================================*/
void* hal_pinInit( en_hal_pin_t pin )
{
    s_hal_gpio_pin_t* p_pin = NULL;
    p_pin = &s_hal_gpio[pin];
    p_pin->val=1;
    hal_ledOn(p_pin->pin);
  return p_pin ;
} /* hal_pinInit() */

/*============================================================================*/
/* hal_pinSet() */
/*============================================================================*/
int8_t hal_pinSet( void* p_pin, uint8_t val )
{
    s_hal_gpio_pin_t* p_gpioPin;
    p_gpioPin = (s_hal_gpio_pin_t *)p_pin;
    if(val)
    {
        hal_ledOn(p_gpioPin->pin);
        p_gpioPin->val=1;
    }
    else
    {
        hal_ledOff(p_gpioPin->pin);
        p_gpioPin->val=0;
    }
return 0;
} /* hal_pinSet() */

/*============================================================================*/
/* hal_pinClr() */
/*============================================================================*/
void hal_pinClr(void * p_pin)
{
  /* Not needed because of integrated IF */
} /* hal_pinClr() */

/*============================================================================*/
/* hal_pinGet() */
/*============================================================================*/
int8_t hal_pinGet(void * p_pin)
{
	  s_hal_gpio_pin_t* p_gpioPin;
	  p_gpioPin = (s_hal_gpio_pin_t *)p_pin;
	  return p_gpioPin->val;
} /* hal_pinGet() */

/*============================================================================*/
/* hal_watchdogReset() */
/*============================================================================*/
int8_t hal_watchdogReset(void)
{
  /* Not needed because the stack will not use this function */
    return 0;
} /* hal_watchdogReset() */

/*============================================================================*/
/* hal_watchdogStart() */
/*============================================================================*/
int8_t hal_watchdogStart(void)
{
  /* Not needed because the stack will not use this function */
    return 0;
} /* hal_watchdogStart() */

/*============================================================================*/
/* hal_watchdogStop() */
/*============================================================================*/
int8_t hal_watchdogStop(void)
{
  /* Not needed because the stack will not use this function */
    return 0 ;
} /* hal_watchdogStop() */

/*============================================================================*/
/* hal_getTick() */
/*============================================================================*/
clock_time_t hal_getTick(void)
{
  return TmrCurTick;
} /* hal_getTick() */

/*============================================================================*/
/* hal_getSec() */
/*============================================================================*/
clock_time_t hal_getSec(void)
{
  clock_time_t secs = 0;

  /* Calculate the seconds */
  secs = TmrCurTick / TARGET_CFG_SYSTICK_RESOLUTION;

  return secs;
} /* hal_getSec() */

/*============================================================================*/
/* hal_getTRes() */
/*============================================================================*/
clock_time_t hal_getTRes(void)
{
  return TARGET_CFG_SYSTICK_RESOLUTION ;
} /* hal_getSec() */

/*============================================================================*/
/* hal_delayUs() */
/*============================================================================*/
int8_t hal_delayUs(uint32_t i_delay)
{
  /*
   * Note(s)
   *
   * hal_delay_us() is only called by emb6.c to make a delay multiple of 500us,
   * which is equivalent to 1 systick
   */
  uint32_t tick_stop;

  tick_stop = hal_ticks;
  tick_stop += i_delay / 500;
  while (tick_stop > hal_ticks)
  {
    /* do nothing */
  }
  return 0;
} /* hal_delay_us() */



#if defined(HAL_SUPPORT_SPI)
void* hal_spiInit( en_hal_spi_t spi )
{
  /* Not needed because of integrated IF */
  return NULL;
} /* hal_spiInit() */


int32_t hal_spiTRx( void* p_spi, uint8_t* p_tx, uint8_t* p_rx, uint16_t len )
{
  /* Not needed because of integrated IF */
  return 0U;
} /* hal_spiRead() */


int32_t hal_spiRx( void* p_spi, uint8_t * p_rx, uint16_t len )
{
  /* Not needed because of integrated IF */
    return 0U;
}

int32_t hal_spiTx( void* p_spi, uint8_t* p_tx, uint16_t len )
{
  /* Not needed because of integrated IF */
    return 0U;
}
#endif

/*---------------------------------------------------------------------------*/
/*
* hal_uartInit()
*/
#if defined(HAL_SUPPORT_UART)
void* hal_uartInit( en_hal_uart_t uart )
{
	_hal_uart_init();
    sf_uart_setRxCb( _hal_uartRxCb );
    return _hal_uartRxCb;
}/* hal_uartInit() */


/*---------------------------------------------------------------------------*/
/*
* hal_uartRx()
*/
int32_t hal_uartRx( void* p_uart, uint8_t * p_rx, uint16_t len )
{
    EMB6_ASSERT_RET( p_rx != NULL, -1 );

    if( len == 0 )
      return 0;

    sf_uart_read(p_rx, len);
    return len;
}/* hal_uartRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_uartTx()
*/
int32_t hal_uartTx( void* p_uart, uint8_t* p_tx, uint16_t len )
{
    EMB6_ASSERT_RET( p_tx != NULL, -1 );

    if( len == 0 )
      return 0;

    sf_uart_write(p_tx, len);
    return len;
}/* hal_uartTx() */
#endif /* #if defined(HAL_SUPPORT_UART) */

/*============================================================================*/
/* hal_pinIRQClear() */
/*============================================================================*/
int8_t hal_pinIRQClear( void* p_pin )
{
  /* Not implemented */
  return 0;
} /* hal_pinIRQClear() */

/*============================================================================*/
/* hal_pinIRQEnable() */
/*============================================================================*/
int8_t hal_pinIRQEnable( void* p_pin )
{
  /* Not implemented */
  return 0;
} /* hal_pinIRQEnable() */

/*============================================================================*/
/* hal_pinIRQDisable() */
/*============================================================================*/
int8_t hal_pinIRQDisable( void* p_pin )
{
  /* Not implemented */
  return 0;
} /* hal_pinIRQDisable() */

/*============================================================================*/
/* hal_pinIRQRegister() */
/*============================================================================*/
int8_t hal_pinIRQRegister( void* p_pin, en_hal_irqedge_t edge,
    pf_hal_irqCb_t pf_cb )
{
  /* Not implemented */
  return 0;
} /* hal_pinIRQRegister() */

/*============================================================================*/
/* hal_periphIRQRegister() */
/*============================================================================*/
int8_t hal_periphIRQRegister( en_hal_periphirq_t irq, pf_hal_irqCb_t pf_cb,
    void* p_data )
{
    /* set the callback and data pointer */
    s_hal_irqs[irq].pf_cb = pf_cb;
    s_hal_irqs[irq].p_data = p_data;
    return 0;
} /* hal_periphIRQRegister() */

/*============================================================================*/
/* hal_debugInit() */
/*============================================================================*/
int8_t hal_debugInit( void )
{
#if (LOGGER_LEVEL > 0) && (HAL_SUPPORT_SLIPUART == FALSE)
  sf_uart_init();
#endif
  return 0;
} /* hal_debugInit() */

/*! @} 6lowpan_mcu */
