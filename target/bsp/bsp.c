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
 *  \file       bsp.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Definition of the Board Support Package used by emb::6.
 *
 *              The BSP is an intermediate layer between emb::6 and the HAL. The
 *              stacks calls functions from the BSP which calls functions from the
 *              HAL. However the BSP provides some common additional logic e.g.
 *              for toggling LEDs.
 */

/*
 *  --- Includes -------------------------------------------------------------*
 */
#include "emb6.h"
#include "emb6assert.h"
#include "bsp.h"
#include "board_conf.h"
#include "etimer.h"
#include "random.h"
#include "evproc.h"


/*
 *  --- Macros ------------------------------------------------------------- *
 */
#define LOGGER_ENABLE             LOGGER_BSP
#include "logger.h"


/*
 * --- Type Definitions -----------------------------------------------------*
 */


/**
 * \brief   Defines an LED in the BSP context.
 *
 *          An LED in the BSP context consists of several attributes such
 *          as the according pin, an inverted flag and a toggle timer.
 */
typedef struct S_BSP_LED_T
{
  /** Pin of the LED */
  void* p_pin;

  /** Blink timer */
  struct etimer blinkTmr;

} s_bsp_led_t;

/**
 * \brief   Defines an SPI in the BSP context.
 *
 *          An SPI in the BSP context consists of several attributes such
 *          as the according SPI instance, and a locked status.
 */
typedef struct S_BSP_SPI_T
{
  /** Instance of the SPI */
  void* p_spi;

  /** locked attribute */
  uint8_t locked;

} s_bsp_spi_t;

/*
 *  --- Local Variables ---------------------------------------------------- *
 */
static uint8_t bsp_wtgStop = 1;
static uint8_t bsp_numNestedCriticalSection;

#if defined(HAL_SUPPORT_LED)
/** LEDs available from BSP */
s_bsp_led_t bsp_leds[HAL_NUM_LEDS];
#endif /* #if defined(HAL_SUPPORT_LED) */

#if defined(HAL_SUPPORT_SPI)
/** SPIs available from BSP */
s_bsp_spi_t bsp_spis[HAL_NUM_SPIS];
#endif /* #if defined(HAL_SUPPORT_SPI) */

/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */
#if defined(HAL_SUPPORT_LED)
/* Callback function for LED toggle timer */
static void _bsp_ledCallback(c_event_t c_event, p_data_t p_data);
#endif /* #if defined(HAL_SUPPORT_LED) */


/*
 *  --- Local Functions ---------------------------------------------------- *
 */

#if defined(HAL_SUPPORT_LED)
/**
 * _bsp_ledCallback()
 *
 * \brief   Initialize the Board Support Package and the underlying HAL.
 *
 *          This function is called at the initialization to initialize the
 *          Board Support Package and the underlying Hardware Abstraction Layer.
 *          During this initialization the according implementation prepares the
 *          general hardware for the according board.
 *
 * \param   event   Event that triggered the callback.
 * \param   p_data  Data pointer that belongs to the element.
 *
 * \return  0 on success or negative value on error.
 */
static void _bsp_ledCallback( c_event_t event, p_data_t p_data )
{
#if defined(HAL_LED_INVERTED)
  int inv = 1;
#else
  int inv = 0;
#endif

  uint8_t j;
  for( j = 0; j < HAL_NUM_LEDS; j++ )
  {
    if( etimer_expired( &bsp_leds[j].blinkTmr ) )
    {
      if( ( &(bsp_leds[j].blinkTmr) ) == (struct etimer *) p_data )
      {
        /* Timer has been found , so we can turn it off. */
        etimer_stop( (struct etimer *) p_data );

        /* toggle the according pin value */
        if( bsp_leds[j].p_pin != NULL )
         // hal_pinSet( bsp_leds[j].p_pin, !hal_pinGet( bsp_leds[j].p_pin) );
         hal_pinSet( bsp_leds[j].p_pin, inv ? TRUE : FALSE );
        break;
      }
    }
  }
} /* _bsp_ledCallback() */
#endif /* #if defined(HAL_SUPPORT_LED) */


/*
 * --- Global Function Definitions ----------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* bsp_init()
*/
int8_t bsp_init (s_ns_t * ps_ns)
{
  /* Pointer to netstack must not be NULL */
  EMB6_ASSERT_RET( ps_ns != NULL, -1 );

#if (HAL_SUPPORT_LEDNUM > 0)
  en_hal_pin_t i;
#endif /* #if (HAL_SUPPORT_LEDNUM > 0) */

  /* Initialize internal structures */
#if defined(HAL_SUPPORT_LED)
  memset( bsp_leds, 0, sizeof(bsp_leds) );
#endif /* #if defined(HAL_SUPPORT_LED) */

#if defined(HAL_SUPPORT_SPI)
  memset( bsp_spis, 0, sizeof(bsp_spis) );
#endif /* #if defined(HAL_SUPPORT_SPI) */

  /* Initialize hardware */
  EMB6_ASSERT_RET( hal_init() == 0, -2 );
  EMB6_ASSERT_RET( hal_debugInit() == 0, -2 );

#if (HAL_SUPPORT_LEDNUM > 0)
  /* get all the LEDs */
  for( i = EN_HAL_PIN_LED0; i < HAL_SUPPORT_LEDNUM; i++ )
  {
    /* initialize according led pin */
    bsp_leds[i].p_pin = bsp_pinInit( i );
  }
#endif /* #if (HAL_SUPPORT_LEDNUM > 0) */

  /* force to turn LED off */
  bsp_led( HAL_LED0 | HAL_LED1 | HAL_LED2 | HAL_LED3, EN_BSP_LED_OP_OFF );

  /* Configure board */
  EMB6_ASSERT_RET( board_conf( ps_ns ) == 0, -3 );

  /* initialize standard random with seed from hardware */
  random_init( hal_getrand() );

  /* initialize local variables */
  bsp_numNestedCriticalSection = 0;

  /* Initialize rtimer module if supported*/
#if (HAL_SUPPORT_RTIMER == TRUE)
bsp_rtimer_init();
#endif

  /* Normal exit*/
  return 0;
}/* bsp_init() */

/*---------------------------------------------------------------------------*/
/*
* bsp_enterCritical()
*/
int8_t bsp_enterCritical( void )
{
  /*
   * if bsp_numNestedCriticalSection is non-zero, then MCU is already in critical sections.
   * therefore there is no need to re-enter critical section.
   * Otherwise, hal_enterCritical() is called.
   */
  if( bsp_numNestedCriticalSection == 0 )
  {
    hal_enterCritical();
  }

  /* increase number of nested critical sections */
  bsp_numNestedCriticalSection++;

  return 0;
} /* bsp_enterCritical() */


/*---------------------------------------------------------------------------*/
/*
* bsp_exitCritical()
*/
int8_t bsp_exitCritical( void )
{
  /*
   * If bsp_numNestedCriticalSection is non-zero, meaning that MCU is in nested critical sections.
   * Therefore it is NOT allowed to exit critical section.
   * Leaving critical section MUST be done in the most outer critical section.
   */
  if( bsp_numNestedCriticalSection > 0 )
  {
    bsp_numNestedCriticalSection--;
    if( bsp_numNestedCriticalSection == 0 )
    {
      /* the most outer critical section */
      hal_exitCritical();
    }
  }

  return 0;
} /* bsp_exitCritical() */


/*---------------------------------------------------------------------------*/
/*
* bsp_watchdog()
*/
int8_t bsp_watchdog( en_bsp_wd_ctrl_t ctrl )
{
  switch( ctrl )
  {
    case EN_BSP_WD_RESET:
      if (!bsp_wtgStop)
        hal_watchdogReset();
      break;
    case EN_BSP_WD_START:
      bsp_wtgStop--;
      if (!bsp_wtgStop)
        hal_watchdogStart();
      break;
    case EN_BSP_WD_STOP:
      bsp_wtgStop++;
      hal_watchdogStop();
      break;
    case EN_BSP_WD_PERIODIC:
      if (!bsp_wtgStop)
        hal_watchdogReset();
      break;
    default:
      break;
  }

  return 0;
} /* bsp_watchdog() */


/*---------------------------------------------------------------------------*/
/*
* bsp_getrand()
*/
uint32_t bsp_getrand( uint32_t min, uint32_t max )
{
  uint32_t ret;

  if(min == 0 && max == 0)
    return (uint32_t) hal_getrand();

  if( max <= min )
    return 0;

  /* generate random number in a range of min to max */
  ret = (uint32_t) hal_getrand();
  ret = ret % max;
  return ret;
} /* bsp_getrand() */


/*---------------------------------------------------------------------------*/
/*
* bsp_getTick()
*/
clock_time_t bsp_getTick( void )
{
  return hal_getTick();
} /* bsp_getTick() */


/*---------------------------------------------------------------------------*/
/*
* bsp_getSec()
*/
clock_time_t bsp_getSec( void )
{
  return hal_getSec();
} /* bsp_getSec() */


/*---------------------------------------------------------------------------*/
/*
* bsp_getTRes()
*/
clock_time_t bsp_getTRes( void )
{
  return hal_getTRes();
} /* bsp_getTRes() */


/*---------------------------------------------------------------------------*/
/*
* bsp_delayUs()
*/
int8_t bsp_delayUs( uint32_t delay )
{
  return hal_delayUs( delay );
} /* bsp_delayUs() */


/*---------------------------------------------------------------------------*/
/*
* bsp_pinInit()
*/
void* bsp_pinInit( en_hal_pin_t pin )
{
  return hal_pinInit( pin );
} /* bsp_pinInit() */

/*---------------------------------------------------------------------------*/
/*
* bsp_pinSet()
*/
int8_t bsp_pinSet( void* p_pin, uint8_t val )
{
  if(p_pin == NULL)
    return -1;

  return hal_pinSet( p_pin, val );
} /* bsp_pinSet() */

/*---------------------------------------------------------------------------*/
/*
* bsp_pinGet()
*/
int8_t bsp_pinGet( void* p_pin )
{
  EMB6_ASSERT_RET( p_pin != NULL, -1 );
  return hal_pinGet( p_pin );
} /* bsp_pinGet() */


/*---------------------------------------------------------------------------*/
/*
* bsp_led()
*/
int8_t bsp_led( uint8_t led, en_bsp_led_op_t op )
{
#if defined(HAL_SUPPORT_LED)
#if defined(HAL_LED_INVERTED)
  int inv = 1;
#else
  int inv = 0;
#endif /* #if defined(HAL_LED_INVERTED)*/
  int i;

  uint8_t ledMsk = led;

  for( i = 0; i < HAL_SUPPORT_LEDNUM; i++ )
  {
    uint8_t ledVal = ledMsk & 1;  /* least significant bit 0b0101 */

    /* next LED */
    ledMsk = (ledMsk >> 1);

    if( (ledVal == 0) && (op != EN_BSP_LED_OP_SET) )
      /* LED not selected */
      continue;

    switch( op )
    {
      case EN_BSP_LED_OP_ON:
        hal_pinSet( bsp_leds[i].p_pin, inv ? FALSE : TRUE );
        break;

      case EN_BSP_LED_OP_OFF:
        hal_pinSet( bsp_leds[i].p_pin, inv ? TRUE : FALSE );
        break;

      case EN_BSP_LED_OP_SET:
        hal_pinSet( bsp_leds[i].p_pin, inv ? !ledVal : ledVal );
        break;

      case EN_BSP_LED_OP_TOGGLE:
        hal_pinSet( bsp_leds[i].p_pin, !hal_pinGet(bsp_leds[i].p_pin) );
        break;

      case EN_BSP_LED_OP_BLINK:
        hal_pinSet( bsp_leds[i].p_pin, inv ? FALSE : TRUE );
        //hal_pinSet( bsp_leds[i].p_pin, !hal_pinGet(bsp_leds[i].p_pin) );
        etimer_set(&bsp_leds[i].blinkTmr, 20, _bsp_ledCallback);
        break;

      default:
        return -2;
    }
  }
#endif /* #if defined(HAL_SUPPORT_LED) */
  return 0;
} /* bsp_led() */


/*---------------------------------------------------------------------------*/
/*
* bsp_pinIRQRegister()
*/
int8_t bsp_pinIRQRegister( void* p_pin, en_hal_irqedge_t edge,
    pf_hal_irqCb_t pf_cb )
{
  EMB6_ASSERT_RET( p_pin != NULL, -1 );
  return hal_pinIRQRegister( p_pin, edge, pf_cb );
} /* bsp_pinIRQRegister() */


/*---------------------------------------------------------------------------*/
/*
* bsp_pinIRQEnable()
*/
int8_t bsp_pinIRQEnable( void* p_pin )
{
  EMB6_ASSERT_RET( p_pin != NULL, -1 );
  return hal_pinIRQEnable( p_pin );
} /* bsp_pinIRQEnable() */


/*---------------------------------------------------------------------------*/
/*
* bsp_pinIRQDisable()
*/
int8_t bsp_pinIRQDisable( void* p_pin )
{
  EMB6_ASSERT_RET( p_pin != NULL, -1 );
  return hal_pinIRQDisable( p_pin );
} /* bsp_pinIRQDisable() */

/*---------------------------------------------------------------------------*/
/*
* bsp_pinIRQClear()
*/
int8_t bsp_pinIRQClear( void* p_pin )
{
  EMB6_ASSERT_RET( p_pin != NULL, -1 );
  return hal_pinIRQClear( p_pin );
} /* bsp_pinIRQClear() */


#if defined(HAL_SUPPORT_SPI)
/*---------------------------------------------------------------------------*/
/*
* bsp_pinIRQClear()
*/
void* bsp_spiInit( en_hal_spi_t spi )
{
  bsp_spis[spi].p_spi = hal_spiInit( spi);
  bsp_spis[spi].locked = FALSE;

  return &bsp_spis[spi];
} /* bsp_spiInit() */


/*---------------------------------------------------------------------------*/
/*
* bsp_spiSlaveSel()
*/
int8_t bsp_spiSlaveSel( void* p_spi, void* p_cs, uint8_t select, uint8_t inv  )
{
  uint8_t ret = 0;
  s_bsp_spi_t* p_bspSpi = (s_bsp_spi_t*)p_spi;
  EMB6_ASSERT_RET( p_bspSpi != NULL, -1 );
  EMB6_ASSERT_RET( p_cs != NULL, -1 );

  if( select == TRUE)
  {
    if( p_bspSpi->locked == FALSE )
    {
      /* lock SPI handle */
      bsp_enterCritical();
      p_bspSpi->locked = TRUE;

      /* enable SPI */
      ret = hal_pinSet( p_cs, inv ? FALSE : TRUE );
    }
    else
    {
      /* SPI is being locked */
      ret = 0;
    }
  }
  else
  {
    if( p_bspSpi->locked == TRUE )
    {
      /* unlock SPI handle */
      p_bspSpi->locked = FALSE;

      /* disable SPI */
      ret = hal_pinSet( p_cs, inv ? TRUE : FALSE );
      bsp_exitCritical();
    }
    else
    {
      /* SPI is already unlocked and disabled */
    }
  }
  return ret;
} /* bsp_spiSlaveSel() */


/*---------------------------------------------------------------------------*/
/*
* bsp_spiTRx()
*/
int32_t bsp_spiTRx( void* p_spi, uint8_t* p_tx, uint8_t* p_rx, uint16_t len )
{
  EMB6_ASSERT_RET( p_spi != NULL, -1 );
  EMB6_ASSERT_RET( p_tx != NULL, -1 );
  EMB6_ASSERT_RET( p_rx != NULL, -1 );
  return hal_spiTRx( ((s_bsp_spi_t *)p_spi)->p_spi, p_tx, p_rx, len );
} /* bsp_spiTRx() */


/*---------------------------------------------------------------------------*/
/*
* bsp_spiRx()
*/
int32_t bsp_spiRx( void* p_spi, uint8_t * p_rx, uint16_t len )
{
  EMB6_ASSERT_RET( p_spi != NULL, -1 );
  EMB6_ASSERT_RET( p_rx != NULL, -1 );
  return hal_spiRx( ((s_bsp_spi_t *)p_spi)->p_spi, p_rx, len );
} /* bsp_spiRx() */


/*---------------------------------------------------------------------------*/
/*
* bsp_spiTx()
*/
int32_t bsp_spiTx( void* p_spi, uint8_t* p_tx, uint16_t len )
{
  EMB6_ASSERT_RET( p_spi != NULL, -1 );
  EMB6_ASSERT_RET( p_tx != NULL, -1 );
  return hal_spiTx( ((s_bsp_spi_t *)p_spi)->p_spi, p_tx, len );
} /* hal_spiTx() */
#endif /* #if defined(HAL_SUPPORT_SPI) */


#if defined(HAL_SUPPORT_UART)

/*---------------------------------------------------------------------------*/
/*
* bsp_uartInit()
*/
void* bsp_uartInit( en_hal_uart_t uart )
{
  /* Call HAL function directly */
  return hal_uartInit( uart );
}/* bsp_uartInit() */



/*---------------------------------------------------------------------------*/
/*
* bsp_uartRx()
*/
int32_t bsp_uartRx( void* p_uart, uint8_t * p_rx, uint16_t len )
{
  /* Call HAL function directly */
  return hal_uartRx( p_uart, p_rx, len );
}/* bsp_uartRx() */



/*---------------------------------------------------------------------------*/
/*
* bsp_uartTx()
*/
int32_t bsp_uartTx( void* p_uart, uint8_t* p_tx, uint16_t len )
{
  /* Call HAL function directly */
  return hal_uartTx( p_uart, p_tx, len );
}/* bsp_uartTx() */
#endif /* #if defined(HAL_SUPPORT_UART) */


/*---------------------------------------------------------------------------*/
/*
* bsp_periphIRQRegister()
*/
int8_t bsp_periphIRQRegister( en_hal_periphirq_t irq, pf_hal_irqCb_t pf_cb,
    void* p_data )
{
  return hal_periphIRQRegister( irq, pf_cb, p_data );
} /* bsp_periphIRQRegister() */

/*---------------------------------------------------------------------------*/
/*
* bsp_getChar()
*/
int bsp_getChar(void)
{
  /* Return the next character received using the console UART */
  /* Nothing to do */
  return 0;
} /* bsp_getChar() */


#if defined(HAL_SUPPORT_RTC)
/*---------------------------------------------------------------------------*/
/*
* bsp_rtcSetTime()
*/
int8_t bsp_rtcSetTime( en_hal_rtc_t *p_rtc )
{
  EMB6_ASSERT_RET( p_rtc != NULL, -1 );
  return hal_rtcSetTime(p_rtc);
}

/*---------------------------------------------------------------------------*/
/*
* bsp_rtcGetTime()
*/
int8_t bsp_rtcGetTime( en_hal_rtc_t *p_rtc )
{
  EMB6_ASSERT_RET( p_rtc != NULL, -1 );
  return hal_rtcGetTime(p_rtc);
}
#endif /* #if defined(HAL_SUPPORT_RTC) */

#if (HAL_SUPPORT_RTIMER == TRUE)
/*---------------------------------------------------------------------------*/
/*
* bsp_rtimer_init()
*/
void bsp_rtimer_init()
{
  hal_rtimer_init();
}
/*
* bsp_rtimer_arch_schedule()
*/
void bsp_rtimer_schedule(rtimer_clock_t t)
{
 hal_rtimer_arch_schedule(t);
}

rtimer_clock_t bsp_rtimer_arch_now()
{
  return hal_rtimer_arch_now();
}

rtimer_clock_t bsp_rtimer_arch_second()
{
	return hal_rtimer_arch_second();
}

int32_t bsp_us_to_rtimerTiscks(int32_t us)
{
  return hal_us_to_rtimerTiscks(us);
}

int32_t bsp_rtimerTick_to_us(int32_t ticks)
{
  return hal_rtimerTick_to_us(ticks);
}

uint32_t bsp_rtimerTick_to_us_64(uint32_t ticks)
{
  return hal_rtimerTick_to_us_64(ticks);
}

#endif /* #if defined(HAL_SUPPORT_RTIMER) */

#if (HAL_SUPPORT_MCU_SLEEP == TRUE)
/*---------------------------------------------------------------------------*/
/*
* bsp_sleepDuration()
*/
clock_time_t bsp_sleepDuration( void )
{
  return hal_sleepDuration();
} /* bsp_sleepDuration() */


/*---------------------------------------------------------------------------*/
/*
* bsp_sleepEnter()
*/
int8_t bsp_sleepEnter( uint32_t duration )
{
  return hal_sleepEnter(duration);
} /* bsp_sleepEnter() */


/*---------------------------------------------------------------------------*/
/*
* bsp_adjustTick()
*/
int8_t bsp_adjustTick( clock_time_t ticks )
{
  return hal_adjustTick(ticks);
} /* bsp_adjustTick */
#endif /* #if (HAL_SUPPORT_MCU_SLEEP == TRUE) */


#if defined(HAL_SUPPORT_NVM)
/*---------------------------------------------------------------------------*/
/*
* bsp_nvmWrite()
*/
int8_t bsp_nvmWrite( uint8_t *pc_data, uint16_t i_len, uint32_t l_addr )
{
  EMB6_ASSERT_RET( pc_data != NULL, -1 );
  return hal_nvmWrite( pc_data, i_len, l_addr );
}

/*---------------------------------------------------------------------------*/
/*
* bsp_nvmRead()
*/
int8_t bsp_nvmRead( uint8_t *pc_data, uint16_t i_len, uint32_t l_addr )
{
  EMB6_ASSERT_RET( pc_data != NULL, -1 );
  return hal_nvmRead( pc_data, i_len, l_addr );
}
#endif /* #if defined(HAL_SUPPORT_NVM) */
