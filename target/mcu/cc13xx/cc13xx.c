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

#include "emb6.h"
#include "hwinit.h"
#include "hal.h"
#include "bsp.h"
#include "board_conf.h"

#include "sf_mcu.h"
#include "sf_mcu_timer.h"
#include "sf_uart.h"
#include "bsp_led.h"
#if CC13XX_LCD_ENABLE
#include "lcd_dogm128_6.h"
#endif /* #if CC13XX_LCD_ENABLE */
#include "driverlib/interrupt.h"
#include "rt_tmr.h"

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
#define MCU_INIT_STATUS_OK                      0x01U
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
static bool _hal_uart_init();
static bool _hal_systick(void);
static void _hal_isrSysTick(uint32_t l_count);
/*============================================================================*/
/*                           LOCAL VARIABLES                                  */
/*============================================================================*/

/*! Hal tick counter */
static clock_time_t volatile hal_ticks;


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
  /** Pin */
  uint8_t pin;
  /** Value */
  uint8_t val;
  /** IRQ callback */
  pf_hal_irqCb_t pf_cb;

} s_hal_gpio_pin_t;

/**
 * \brief   Description of an interrupt.
 *
 *          An interrupt consists of the according callback function
 *          and a data pointer.
 */


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


#if  0
  defined(HAL_SUPPORT_SLIPUART)
  {EFM32_SLIP_UART_PORT_USART_TX, EFM32_SLIP_UART_PIN_USART_TX, gpioModePushPull, 0, NULL}, /* UART_TX */
  {EFM32_SLIP_UART_PORT_USART_RX, EFM32_SLIP_UART_PIN_USART_RX, gpioModeInputPull, 0, NULL}, /* UART_RX */
#endif

};

/** Definition of the peripheral callback functions */
s_hal_irq s_hal_irqs[EN_HAL_PERIPHIRQ_MAX];


/*============================================================================*/
/*                           LOCAL FUNCTIONS                                  */
/*============================================================================*/
/*
int putchar(int _c)
{
  sf_uart_write((uint8_t*)&_c, 0x01U);
  return (unsigned char)_c;
}*/

#ifdef 0 // __TI_ARM__
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
 * @return true for success, false if initialization failed.
 */
bool _hal_uart_init()
{
  bool b_return = false;
  uint8_t p_data[] = { "\r\n\r\n========== BEGIN ===========\r\n\r\n" };
  uint16_t i_dataLen = sizeof(p_data);

  /* Initialize UART */
  b_return = sf_uart_init();

  if (b_return) {
    if (!(sf_uart_write(p_data, i_dataLen) == i_dataLen))
    {
      b_return = false;
    }
  }

  return b_return;
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
 * @return true if actions succeeds, else false.
 */
static bool _hal_systick(void)
{
  bool b_success = false;

  /* Configure the timer to call _hal_isrSysTick */
  b_success = sf_mcu_timer_setCallback((fp_mcu_timer_cb) &_hal_isrSysTick);
  if(b_success)
  {
    /* Enable the timer */
    sf_mcu_timer_enable();
  }
  return b_success;
}

/*==============================================================================
                             API FUNCTIONS
 ==============================================================================*/

/*!
 * @brief Disables all interrupts.
 *
 * This function disables all interrupts when the
 * program enters critical sections.
 */
int8_t hal_enterCritical(void)
{
  /* Disable the interrutps */
  sf_mcu_interruptDisable();
  return 0;
} /* hal_enterCritical() */

/*!
 * @brief Enables all interrupts.
 *
 */
int8_t hal_exitCritical(void)
{
  /* Enbale the interrupts */
  sf_mcu_interruptEnable();
  return 0;
}/* hal_exitCritical() */

/*!
 * @brief This function initializes all of the MCU peripherals.
 *
 * @return Status code.
 */
int8_t hal_init(void)
{
  uint8_t c_retStatus = 0U;

#if CC13XX_LCD_ENABLE
  char lcdBuf[LCD_BYTES];
#endif /* #if CC13XX_LCD_ENABLE */

  /* Initialize the mcu */
  c_retStatus = sf_mcu_init();
  if(MCU_INIT_RET_STATUS_CHECK(c_retStatus))
  {
    /* Initialize the timer. MCU_TICKS_PER_SECOND specifies
     * the tick interval. */
    c_retStatus = sf_mcu_timer_init(MCU_TICKS_PER_SECOND);
    if(MCU_INIT_RET_STATUS_CHECK(c_retStatus))
    {
      c_retStatus = _hal_uart_init();
      if(MCU_INIT_RET_STATUS_CHECK(c_retStatus))
      {
        c_retStatus = _hal_systick();
      }
    }
  }

  /* Initialize hal_ticks */
  hal_ticks = 0x00U;

  /* initialize LEDs */
  bspLedInit( BSP_LED_ALL );

#if CC13XX_LCD_ENABLE
  /* initialize LCD */
  lcdSpiInit();
  lcdInit();
  lcdClear();

  /* Send Simple Message */
  lcdBufferClear( lcdBuf );
  lcdSendBuffer( lcdBuf );
#endif /* #if CC13XX_LCD_ENABLE */

  return (!c_retStatus);
}/* hal_init() */

/*!
 * @brief This function calculates a random number.
 *
 * Function is not used by the stack and thus not implemented.
 *
 * @return Always 1U.
 */
uint32_t hal_getrand( void )
{
  return 1;
}/* hal_getrand() */

/*!
 * @brief This function turns a led off.
 *
 * @param ui_led Descriptor of a led.
 */
void hal_ledOff(uint16_t ui_led)
{
    switch( ui_led )
    {
        case CC1310_LED0:
            bspLedClear( BSP_LED_1 );
            break;

        case CC1310_LED1:
            bspLedClear( BSP_LED_2 );
            break;

        case CC1310_LED2:
            bspLedClear( BSP_LED_3 );
            break;

        case CC1310_LED3:
            bspLedClear( BSP_LED_4 );
            break;
    }
}/* hal_ledOff() */

/*!
 * @brief This function turns a led on.
 *
 *
 * @param ui_led Descriptor of a led.
 */
void hal_ledOn(uint16_t ui_led)
{
    switch( ui_led )
    {
        case CC1310_LED0:  /* LED mask */
            bspLedSet( BSP_LED_1 );
            break;

        case CC1310_LED1:
            bspLedSet( BSP_LED_2 );
            break;

        case CC1310_LED2:
            bspLedSet( BSP_LED_3 );
            break;

        case CC1310_LED3:
            bspLedSet( BSP_LED_4 );
            break;
    }
}/* hal_ledOn() */

/*!
 * @brief This function makes a delay.
 *
 * The delay value should only be a multiple of 500us.
 *
 * @param i_delay Delay in micro seconds.
 */
int8_t hal_delayUs(uint32_t i_delay)
{
  /*
   * Note(s)
   *
   * hal_delay_us() is only called by emb6.c to make a delay multiple of 500us,
   * which is equivalent to 1 systick
   */
  uint32_t tick_stop;

  hal_enterCritical();
  tick_stop = hal_ticks;
  tick_stop += i_delay / 500;
  hal_exitCritical();
  while (tick_stop > hal_ticks)
  {
    /* do nothing */
  }
  return 0;
} /* hal_delay_us() */


/*!
 * @brief This function sets a particular pin.
 *
 * Not implemented, because of integrated transceiver.
 *
 * @param p_pin Pointer to a pin.
 */
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


/*!
 * @brief This function clears a particular pin.
 *
 * Not implemented, because of integrated transceiver.
 *
 * @param p_pin Pointer to a pin.
 */
void hal_pinClr(void * p_pin)
{
    s_hal_gpio_pin_t* p_gpioPin;
    p_gpioPin = (s_hal_gpio_pin_t *)p_pin;

        hal_ledOff(p_gpioPin->pin);
        p_gpioPin->val=0;
} /* hal_pinClr() */

/*!
 * @brief This function returns the pin status.
 *
 * Not implemented, because of integrated transceiver.
 *
 * @param p_pin Pointer to a pin.
 * @return Status of the pin.
 */
int8_t hal_pinGet(void * p_pin)
{
    s_hal_gpio_pin_t* p_gpioPin;
    p_gpioPin = (s_hal_gpio_pin_t *)p_pin;

  return p_gpioPin->val;
} /* hal_pinGet() */

/*!
 * @brief This function initializes the SPI interface.
 *
 * Not implemented, because of integrated transceiver.
 *
 * @return Pointer to an allocated memory.
 */
void* hal_spiInit(void)
{
  /* Not needed because of integrated IF */
  return NULL;
} /* hal_spiInit() */

/*!
 * @brief This function selects or deselects the SPI slave.
 *
 * Not implemented, because of integrated transceiver.
 *
 * @param p_spi Pointer to a SPI entitiy.
 * @param action true to select, false to deselect an SPI entitiy.
 *
 * @return 1 if action succeeded, else 0.
 */
uint8_t hal_spiSlaveSel(void * p_spi, bool action)
{
  /* Not needed because of integrated IF */
  return 0U;
} /* hal_spiSlaveSel() */

/*!
 * @brief This function reads data from the SPI interface.
 *
 * Not implemented, because of integrated transceiver.
 *
 * @param p_reg Pointer to buffer storing the data.
 * @param i_length Length of data to be received.
 */
uint8_t hal_spiRead(uint8_t * p_reg, uint16_t i_length)
{
  /* Not needed because of integrated IF */
  return 0U;
} /* hal_spiRead() */

/*!
 * @brief This function writes data to the SPI interface.
 *
 * Not implemented, because of integrated transceiver.
 *
 * @param c_value Pointer to the data to write.
 * @param i_length Length of data to be written.
 */
void hal_spiWrite(uint8_t * c_value, uint16_t i_length)
{
  /* Not needed because of integrated IF */
} /* hal_spiWrite() */

/*!
 * @brief This function simultaneously reads and writes data.
 *
 * Not implemented, because of integrated transceiver.
 *
 * @param p_tx Pointer to the data to write.
 * @param p_rx Pointer to the buffer storing the received data.
 * @param leh Size of data to send.
 */
void hal_spiTxRx(uint8_t *p_tx, uint8_t *p_rx, uint16_t len)
{
  /* Not needed because of integrated IF */
}




#if defined(HAL_SUPPORT_UART)

/*---------------------------------------------------------------------------*/
/*
* hal_uartInit()
*/
void* hal_uartInit( en_hal_uart_t uart )
{
    sf_uart_init();
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


/*!
 * @brief This function resets the watchdog timer.
 *
 * Function is not used by the stack and thus not implemented.
 */
int8_t hal_watchdogReset(void)
{
  /* Not needed because the stack will not use this function */
    return 0;
} /* hal_watchdogReset() */

/*!
 * @brief This function starts the watchdog timer.
 *
 * Function is not used by the stack and thus not implemented.
 */
int8_t hal_watchdogStart(void)
{
  /* Not needed because the stack will not use this function */
    return 0;
} /* hal_watchdogStart() */

/*!
 * @brief This function stops the watchdog timer.
 *
 * Function is not used by the stack and thus not implemented.
 */
int8_t hal_watchdogStop(void)
{
  /* Not needed because the stack will not use this function */
    return 0 ;
} /* hal_watchdogStop() */

/*!
 * @brief This function returns the system ticks.
 *
 * @return Current system tick.
 */
clock_time_t hal_getTick(void)
{
  return TmrCurTick;
} /* hal_getTick() */

/*!
 * @brief This function returns seconds.
 *
 * @return Seconds.
 */
clock_time_t hal_getSec(void)
{
  clock_time_t secs = 0;

  /* Calculate the seconds */
  secs = TmrCurTick / TARGET_CFG_SYSTICK_RESOLUTION;

  return secs;
} /* hal_getSec() */

/*!
 * @brief This function returns the time resolution.
 *
 * @return Resolution in ms.
 */
clock_time_t hal_getTRes(void)
{
  return TARGET_CFG_SYSTICK_RESOLUTION ;
} /* hal_getSec() */


void hal_extiRegister(en_hal_irqedge_t e_edge, pf_hal_irqCb_t pf_cbFnct)
{
    /* Not used */
} /* hal_extiRegister() */

/*! @} 6lowpan_mcu */


int8_t hal_debugInit( void )
{
  return 0;
} /* hal_debugInit() */

int8_t hal_periphIRQRegister( en_hal_periphirq_t irq, pf_hal_irqCb_t pf_cb,
    void* p_data )
{
  /* set the callback and data pointer */
  s_hal_irqs[irq].pf_cb = pf_cb;
  s_hal_irqs[irq].p_data = p_data;
  return 0;
} /* hal_periphIRQRegister() */

void* hal_pinInit( en_hal_pin_t pin )
{
    s_hal_gpio_pin_t* p_pin = NULL;
    p_pin = &s_hal_gpio[pin];
    //p_pin->pin=(uint8_t)pin;
    p_pin->val=1;
    hal_ledOn(p_pin->pin);

  return p_pin ;
} /* hal_pinInit() */


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

int8_t hal_pinIRQRegister( void* p_pin, en_hal_irqedge_t edge,
    pf_hal_irqCb_t pf_cb )
{
  /* Not implemented */
  return -1;
} /* hal_pinIRQRegister() */
