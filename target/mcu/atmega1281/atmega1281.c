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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>


/*
 * --- Macro Definitions --------------------------------------------------- *
 */

/* Concatenate macro*/
#define CONCAT(x, y)                      x##y
#define CONCAT2(x, y, z)                  x##y##z

#define UCSR(num, let)                    CONCAT2(UCSR,num,let)
#define RXEN(x)                           CONCAT(RXEN,x)
#define TXEN(x)                           CONCAT(TXEN,x)
#define RXCIE(x)                          CONCAT(RXCIE,x)
#define UCSZ(x,y)                         CONCAT2(UCSZ,x,y)
#define UBRR(x,y)                         CONCAT2(UBRR,x,y)
#define UDRE(x)                           CONCAT(UDRE,x)
#define UDRIE(x)                          CONCAT(UDRIE,x)
#define UDR(x)                            CONCAT(UDR,x)
#define RXVECT(x)                         CONCAT2(USART,x,_RX_vect)

#define LOGGER_ENABLE                     LOGGER_HAL
#include "logger.h"

/** CPU frequency */
#define ATM1281_CPU_FREQ                  8000000UL
/** Number of ticks per second */
#define ATM1281_TICK_SECONDS              125
/* Prescaler for timer 0  */
#define ATM1281_TMR0_PRESC                256

/** Wait for an SPI transaction to be finished */
#define ATM1281_SPI_TRAN_WAIT()           while ((SPSR & (1 << SPIF)) == 0) {;}
/** Read from SPI */
#define ATM1281_SPI_TRAN_READ()           (SPDR)
/** Write to SPI */
#define ATM1281_SPI_TRAN_WRITE(c_value)   SPDR = (c_value);


/** Port output direction */
#define ATM1281_DIR_OUT                   1
/** Port input direction */
#define ATM1281_DIR_IN                    0

/*
 *  --- Type Definitions -----------------------------------------------------*
 */
/** Port pin assignment */
typedef uint8_t regType_t;

/** Pin description */
typedef struct
{
  /** Port register*/
  volatile regType_t *p_port;
  /** Direction register */
  volatile regType_t *p_ddr;
  /** Pin */
  uint8_t pin;
  /** Direction */
  uint8_t dir;
  /** Initial value */
  uint8_t init;
  /** IRQNum */
  uint8_t irqNum;
  /** IRQ callback */
  pf_hal_irqCb_t pf_cb;

} s_hal_gpio_pin_t;


/** SPI descriptor */
typedef struct
{
  /** clk select pin */
  s_hal_gpio_pin_t* p_clkPin;
  /** tx  pin */
  s_hal_gpio_pin_t* p_txPin;
  /** rx pin */
  s_hal_gpio_pin_t* p_rxPin;
  /** chip select pin */
  s_hal_gpio_pin_t* p_csPin;

} s_hal_spi_t;


typedef struct
{
  /** callback function */
  pf_hal_irqCb_t pf_cb;
  /** data pointer */
  void* p_data;

} s_hal_irq;

/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */

/* Initialization of the timer */
static void _hal_tim0Init( void );

#if (LOGGER_LEVEL > 0) && (!defined (HAL_SUPPORT_SLIPUART))
/** Ouput function to wire a character to UART */
static int _hal_debugPutChar( char c, FILE * stream );
#endif /* #if (LOGGER_LEVEL > 0) && (!defined (HAL_SUPPORT_SLIPUART)) */

/** Delay loop */
inline static void _hal_delayLoop( uint16_t __count ) __attribute__((always_inline));

/* Common callback for external interrupts */
static void _hal_extiCb( uint8_t pin );


/*
 *  --- Local Variables ---------------------------------------------------- *
 */

/** Ticks since startup */
clock_time_t volatile l_tick;
/** Seconds since startup */
clock_time_t volatile l_sec;

static uint8_t volatile c_sreg;
static int8_t c_nested = 0;

/** Definition of the IOs */
static s_hal_gpio_pin_t s_hal_gpio[EN_HAL_PIN_MAX] = {

#if defined(HAL_SUPPORT_LED0)
  {&(ATM1281_LED0_PORT), &(ATM1281_LED0_DDR), ATM1281_LED0_PIN, ATM1281_DIR_OUT, 0, 0xFF, NULL}, /* LED0 */
#endif /* #if defined(HAL_SUPPORT_LED0) */
#if defined(HAL_SUPPORT_LED1)
  {&(ATM1281_LED1_PORT), &(ATM1281_LED1_DDR), ATM1281_LED1_PIN, ATM1281_DIR_OUT, 0, 0xFF, NULL}, /* LED1 */
#endif /* #if defined(HAL_SUPPORT_LED1) */
#if defined(HAL_SUPPORT_LED2)
  {&(ATM1281_LED2_PORT), &(ATM1281_LED2_DDR), ATM1281_LED2_PIN, ATM1281_DIR_OUT, 0, 0xFF, NULL}, /* LED2 */
#endif /* #if defined(HAL_SUPPORT_LED2) */
#if defined(HAL_SUPPORT_LED3)
  {&(ATM1281_LED3_PORT), &(ATM1281_LED3_DDR), ATM1281_LED3_PIN, ATM1281_DIR_OUT, 0, 0xFF, NULL}, /* LED3 */
#endif /* #if defined(HAL_SUPPORT_LED3) */
#if defined(HAL_SUPPORT_LED4)
  {&(ATM1281_LED4_PORT), &(ATM1281_LED4_DDR), ATM1281_LED4_PIN, ATM1281_DIR_OUT, 0, 0xFF, NULL}, /* LED4 */
#endif /* #if defined(HAL_SUPPORT_LED4) */

#if defined(HAL_SUPPORT_RFSPI)
  {&(ATM1281_RFSPI_CLK_PORT), &(ATM1281_RFSPI_CLK_DDR), ATM1281_RFSPI_CLK_PIN, ATM1281_DIR_OUT, 0, 0xFF, NULL}, /* RF SPI CLK */
  {&(ATM1281_RFSPI_TX_PORT), &(ATM1281_RFSPI_TX_DDR), ATM1281_RFSPI_TX_PIN, ATM1281_DIR_OUT, 0, 0xFF, NULL}, /* RF SPI TX */
  {&(ATM1281_RFSPI_RX_PORT), &(ATM1281_RFSPI_RX_DDR), ATM1281_RFSPI_RX_PIN, ATM1281_DIR_IN, 0, 0xFF, NULL}, /* RF SPI RX */
  {&(ATM1281_RFSPI_CS_PORT), &(ATM1281_RFSPI_CS_DDR), ATM1281_RFSPI_CS_PIN, ATM1281_DIR_OUT, 1, 0xFF, NULL}, /* RF SPI CS */
#endif /* #if defined(HAL_SUPPORT_RFSPI) */

#if defined(HAL_SUPPORT_RFCTRL0)
  {&(ATM1281_RESET_PORT), &(ATM1281_RESET_DDR), ATM1281_RESET_PIN, ATM1281_DIR_OUT, 0, 0xFF, NULL}, /* RF CTRL 0 = RESET */
#endif /* #if defined(HAL_SUPPORT_RFCTRL1) */
#if defined(HAL_SUPPORT_RFCTRL1)
  {&(ATM1281_SLP_TR_PORT), &(ATM1281_SLP_TR_DDR), ATM1281_SLP_TR_PIN, ATM1281_DIR_OUT, 1, 0xFF, NULL}, /* RF CTRL 1 = SLEEP */
#endif /* #if defined(HAL_SUPPORT_RFCTRL1) */
#if defined(HAL_SUPPORT_RFCTRL2)
  {&(ATM1281_RFIRQ_PORT), &(ATM1281_RFIRQ_DDR), ATM1281_RFIRQ_PIN, ATM1281_DIR_IN, 0, ATM1281_RFIRQ_IRQNUM, NULL}, /* RF CTRL 2 = IRQ */
#endif /* #if defined(HAL_SUPPORT_RFCTRL2) */
};


#if defined(HAL_SUPPORT_RFSPI)
/** Definition of the SPI interface */
static s_hal_spi_t s_hal_spi = {
  .p_clkPin = &s_hal_gpio[EN_HAL_PIN_RFSPICLK],
  .p_txPin  = &s_hal_gpio[EN_HAL_PIN_RFSPITX],
  .p_rxPin  = &s_hal_gpio[EN_HAL_PIN_RFSPIRX],
  .p_csPin  = &s_hal_gpio[EN_HAL_PIN_RFSPICS]
};
#endif /* #if defined(HAL_SUPPORT_RFSPI) */


/** Definition of the peripheral callback functions */
static s_hal_irq s_hal_irqs[EN_HAL_PERIPHIRQ_MAX];


#if (LOGGER_LEVEL > 0) && (!defined (HAL_SUPPORT_SLIPUART))
/** Redirection of stdout to UART */
static FILE st_usartStdout = FDEV_SETUP_STREAM( _hal_debugPutChar, NULL,
        _FDEV_SETUP_WRITE );
#endif /* #if (LOGGER_LEVEL > 0) && (!defined (HAL_SUPPORT_SLIPUART)) */


/*
 *  --- Local Functions ---------------------------------------------------- *
 */

/**
 * \brief   Initialization of the timer.
 *
 *          This function initializes the timer. Therefore it sets the
 *          according timeouts but disableds the watchdog per default.
 */
static void _hal_tim0Init( void )
{
    /* Select internal clock */
    ASSR = 0x00;

    /* Set counter to zero */
    TCNT0 = 0;

    /*
     * Set comparison register:
     * Crystal freq. is ATM1281_CPU_FREQ, prescale is ATM1281_TMR0_PRESC,
     * We want ATM1281_TICK_SECONDS ticks / sec:
     * ATM1281_CPU_FREQ = CONF_TMR0_PRESCALE * CONF_TICK_SEC * OCR0A, less 1 for CTC mode
     */
    OCR0A = ATM1281_CPU_FREQ / ATM1281_TMR0_PRESC / ATM1281_TICK_SECONDS - 1;

    /*
     * Set timer control register:
     *  - prescale according to AVR_CONF_TMR0_PRESCALE
     *  - counter reset via comparison register (WGM01)
     */
    TCCR0A = _BV( WGM01 );
    TCCR0B = _BV( CS02 );

    /* Clear interrupt flag register */
    TIFR0 = TIFR0;

    /*
     * Raise interrupt when value in OCR0 is reached. Note that the
     * counter value in TCNT0 is cleared automatically.
     */
    TIMSK0 = _BV( OCIE0A );

} /* _hal_tim0Init() */


#if (LOGGER_LEVEL > 0) && (!defined (HAL_SUPPORT_SLIPUART))
/*----------------------------------------------------------------------------*/
/**
 * \brief   Output character using the debug interface.
 *
 *          This function writes a single charachter to the debug interface
 *          once it is ready.
 *
 * \param   c         Charachter to write.
 * \param   stream    Stream to write the data to.
 *
 * \return  Always returns 0.
 */
static int _hal_debugPutChar( char c, FILE * stream )
{
  /* Do nothing until data have been received and is read to be read from UDR */
  while( ( UCSR(ATM1281_UART_DEBUG_INST,A) &
      ( 1 << UDRE(ATM1281_UART_DEBUG_INST) ) ) == 0 );
  UDR(ATM1281_UART_DEBUG_INST) = c;
  return 0;
} /* hal_uart_putchar() */
#endif /* #if (LOGGER_LEVEL > 0) && (!defined (HAL_SUPPORT_SLIPUART)) */

/*----------------------------------------------------------------------------*/
/** \brief  This function implements a low level delay as builin AVR delay
 *             function operates only with constant parameter.
 *
 *  \param  __count        How much ticks should be delayed.
 *  \retval None
 */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/**
 * \brief   Loop to generate a specific delay..
 *
 *          This function implements a low level delay as builin AVR delay
 *          function operates only with constant parameter.
 *
 * \param   count     Ticks to delay.
 *
 */
static void _hal_delayLoop( uint16_t count )
{
    __asm__ volatile (
            "1: sbiw %0,1" "\n\t"
            "brne 1b"
            : "=w" (count)
            : "0" (count)
    );
}

/**
 * \brief   External interrupt handler.
 *
 *          The external interrupt handler function is used to trigger
 *          the registered callbacks once an external interrupt occurred.
 *
 * \param   pin   Pin that triggered the interrupt.
 * \param   pin   Edge of the interrupt.
 */
static void _hal_extiCb( uint8_t pin )
{
  int i;
  for( i = 0; i < EN_HAL_PIN_MAX; i++ )
  {
    if( s_hal_gpio[i].pin == pin )
    {
      /*find the correct interrupt occurred */
      if( s_hal_gpio[i].pf_cb)
      {

        /* call the according callback function */
        s_hal_gpio[i].pf_cb( NULL );
      }
    }
  }
}


ISR( TIMER0_COMPA_vect )
{
    /* Indicate timer update to the emb6 timer */
    if( l_tick % ATM1281_TICK_SECONDS == 0 )
        l_sec++;
    l_tick++;
} /* AVR_OUTPUT_COMPARE_INT() */

ISR( INT0_vect )
{
   /* forward to the common callback function with the
   * pin index as parameter */
  _hal_extiCb( INT0 );
}

ISR( INT1_vect )
{
  /* forward to the common callback function with the
   * pin index and edge as parameter */
  _hal_extiCb( INT1 );
}

ISR( INT2_vect )
{
  /* forward to the common callback function with the
   * pin index as parameter */
  _hal_extiCb( INT2 );
}

ISR( INT3_vect )
{
  /* forward to the common callback function with the
   * pin index as parameter */
  _hal_extiCb( INT3 );
}
ISR( INT4_vect )
{
  /* forward to the common callback function with the
   * pin index as parameter */
  _hal_extiCb( INT4 );
}

ISR( INT5_vect )
{
  /* forward to the common callback function with the
   * pin index as parameter */
  _hal_extiCb( INT5 );
}

ISR( INT6_vect )
{
  /* forward to the common callback function with the
   * pin index as parameter */
  _hal_extiCb( INT6 );
}

ISR( INT7_vect )
{
  /* forward to the common callback function with the
   * pin index as parameter */
  _hal_extiCb( INT7 );
}

#if !defined (HAL_SUPPORT_SLIPUART)
ISR( RXVECT(ATM1281_UART_DEBUG_INST) )
{
  /* Do nothing */
}
#else
ISR( RXVECT(ATM1281_UART_SLIP_INST) )
{
    char c_rxbyte;
    c_rxbyte = UDR(ATM1281_UART_SLIP_INST);
    if( s_hal_irqs[EN_HAL_PERIPHIRQ_SLIPUART_RX].pf_cb != NULL )
      s_hal_irqs[EN_HAL_PERIPHIRQ_SLIPUART_RX].pf_cb ( &c_rxbyte );
}
#endif /* #if !defined (HAL_SUPPORT_SLIPUART) */


/*
* --- Global Functions ---------------------------------------------------- *
*/

/*---------------------------------------------------------------------------*/
/*
* hal_init()
*/
int8_t hal_init( void )
{
    hal_enterCritical();

    /* Init timer*/
    _hal_tim0Init();

    MCUSR = 0;

    hal_exitCritical();
    return 0;
}/* hal_init() */


/*---------------------------------------------------------------------------*/
/*
* hal_enterCritical()
*/
int8_t hal_enterCritical( void )
{
    if( !c_nested )
    {
        c_sreg = SREG;
        cli();
        c_nested++;
    }
    else
        c_nested++;

    return 0;
} /* hal_enterCritical() */


/*---------------------------------------------------------------------------*/
/*
* hal_exitCritical()
*/
int8_t hal_exitCritical( void )
{
    if( c_nested == 1 )
    {
        SREG = c_sreg;
        sei();
        c_nested--;
    }
    else
        c_nested--;

    return 0;
}/* hal_exitCritical() */


/*---------------------------------------------------------------------------*/
/*
* hal_watchdogStart()
*/
int8_t hal_watchdogStart( void )
{
    wdt_enable(WDTO_2S);
    return 0;
} /* hal_watchdogStart() */


/*---------------------------------------------------------------------------*/
/*
* hal_watchdogReset()
*/
int8_t hal_watchdogReset( void )
{
    wdt_reset();
    return 0;
} /* hal_watchdogReset() */


/*---------------------------------------------------------------------------*/
/*
* hal_watchdogStop()
*/
int8_t hal_watchdogStop( void )
{
    wdt_disable();
    return 0;
} /* hal_watchdogStop() */


/*---------------------------------------------------------------------------*/
/*
* hal_getrand()
*/
uint32_t hal_getrand( void )
{
  uint32_t ret = 0;

    ADMUX = 0x1E; //Select AREF as reference, measure 1.1 volt bandgap reference.
    ADCSRA = 1 << ADEN; //Enable ADC, not free running, interrupt disabled, fastest clock
    ADCSRA |= 1 << ADSC;          //Start conversion
    while( ADCSRA & ( 1 << ADSC ) )
        ; //Wait till done
    ret = ADC;
    ADCSRA = 0;                 //Disable ADC
    return ret;
}/* hal_getrand() */


/*---------------------------------------------------------------------------*/
/*
* hal_getTick()
*/
clock_time_t hal_getTick( void )
{
    return l_tick;
} /* hal_getTick() */


/*---------------------------------------------------------------------------*/
/*
* hal_getSec()
*/
clock_time_t hal_getSec( void )
{
  return l_sec;
} /* hal_getSec() */


/*---------------------------------------------------------------------------*/
/*
* hal_getTRes()
*/
clock_time_t hal_getTRes( void )
{
  return ATM1281_TICK_SECONDS;
}


/*---------------------------------------------------------------------------*/
/*
* hal_delayUs()
*/
int8_t hal_delayUs( uint32_t delay )
{
    if( delay < 4 )
        return 0;
    delay -= 3;
    _hal_delayLoop( ( delay * 4 ) / 2 );

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

  /* Configure the Pin */
  (*( p_pin->p_port )) &= ~(1 << p_pin->pin);
  if( p_pin->dir == ATM1281_DIR_IN )
    (*( p_pin->p_ddr )) &= ~(1 << p_pin->pin);
  else
  {
    (*(p_pin->p_ddr)) |= (1 << p_pin->pin);
    if( p_pin->init )
      (*( p_pin->p_port )) |= (1 << p_pin->pin);
    else
      (*( p_pin->p_port )) &= ~(1 << p_pin->pin);
  }

  return (void *)p_pin;
} /* hal_pinInit() */


/*---------------------------------------------------------------------------*/
/*
* hal_pinSet()
*/
int8_t hal_pinSet( void* p_pin, uint8_t val )
{
  s_hal_gpio_pin_t* p_gpioPin;
  p_gpioPin = (s_hal_gpio_pin_t *)p_pin;

  EMB6_ASSERT_RET(p_gpioPin != NULL, -1);
  EMB6_ASSERT_RET(p_gpioPin->dir == ATM1281_DIR_OUT, -1);

  if( val )
    (*( p_gpioPin->p_port )) |= (1 << p_gpioPin->pin);
  else
    (*( p_gpioPin->p_port )) &= ~(1 << p_gpioPin->pin);

  return (val ? 1 : 0);

} /* hal_pinSet() */


/*---------------------------------------------------------------------------*/
/*
* hal_pinGet()
*/
int8_t hal_pinGet(void *p_pin)
{
  s_hal_gpio_pin_t* p_gpioPin;
  p_gpioPin = (s_hal_gpio_pin_t *)p_pin;

  EMB6_ASSERT_RET(p_gpioPin != NULL, -1);
  return (((*( p_gpioPin->p_port )) >> p_gpioPin->pin) & 1);

} /* hal_pinGet() */


/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQRegister()
*/
int8_t hal_pinIRQRegister( void* p_pin, en_hal_irqedge_t edge,
    pf_hal_irqCb_t pf_cb )
{
  uint8_t iscx = 0;

  s_hal_gpio_pin_t* p_gpioPin;
  p_gpioPin = (s_hal_gpio_pin_t*)p_pin;

  EMB6_ASSERT_RET(p_gpioPin != NULL, -1);
  hal_enterCritical();

  /* we expect a valid IRQ */
  EMB6_ASSERT_RET(p_gpioPin->irqNum != 0xFF, -1);

  /* we expect that the pin is configured as input */
  EMB6_ASSERT_RET(p_gpioPin->dir == ATM1281_DIR_IN, -1);

  /* enable IRQ */
  EIMSK |= ( 1 << p_gpioPin->irqNum );

  switch( edge )
  {
    case EN_HAL_IRQEDGE_EITHER:
      iscx = 0x01;
      break;

    case EN_HAL_IRQEDGE_RISING:
      iscx = 0x03;
      break;

    case EN_HAL_IRQEDGE_FALLING:
      iscx = 0x02;
      break;

    default:
      iscx = 0x01;
  }

  /* Enable rising and falling edge detection */
  if( p_gpioPin->irqNum <= 3  )
  {
    EICRA &= ~(0x03  << ( p_gpioPin->pin << 1 ));
    EICRA |= (iscx  << ( p_gpioPin->pin << 1 ));
  }
  else
  {
    EICRB &= ~(0x03  << ( p_gpioPin->pin << 1 ));
    EICRB |= (iscx  << ( p_gpioPin->pin << 1 ));
  }

  /* set the callback and edge*/
  p_gpioPin->pf_cb = pf_cb;

  hal_exitCritical();
  return 0;

} /* hal_extiRegister() */


/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQEnable()
*/
int8_t hal_pinIRQEnable( void* p_pin )
{
  s_hal_gpio_pin_t* p_gpioPin;
  p_gpioPin = (s_hal_gpio_pin_t*)p_pin;

  EMB6_ASSERT_RET(p_gpioPin != NULL, -1);
  hal_enterCritical();

  EIMSK |= ( 1 << p_gpioPin->irqNum );

  hal_exitCritical();
  return 0;

} /* hal_pinIRQEnable() */


/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQDisable()
*/
int8_t hal_pinIRQDisable( void* p_pin )
{
  s_hal_gpio_pin_t* p_gpioPin;
  p_gpioPin = (s_hal_gpio_pin_t*)p_pin;

  EMB6_ASSERT_RET(p_gpioPin != NULL, -1);
  hal_enterCritical();

  EIMSK &= ~ ( 1 << p_gpioPin->irqNum );

  hal_exitCritical();
  return 0;
} /* hal_pinIRQDisable() */


/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQClear()
*/
int8_t hal_pinIRQClear( void* p_pin )
{
  s_hal_gpio_pin_t* p_gpioPin;
  p_gpioPin = (s_hal_gpio_pin_t*)p_pin;

  EMB6_ASSERT_RET(p_gpioPin != NULL, -1);
  hal_enterCritical();

  EIFR |= ( 1 << p_gpioPin->irqNum );

  hal_exitCritical();
  return 0;
} /* hal_pinIRQClear() */


#if defined(HAL_SUPPORT_SPI) && defined(HAL_SUPPORT_RFSPI)
/*---------------------------------------------------------------------------*/
/*
* hal_spiInit()
*/
void* hal_spiInit( en_hal_spi_t spi )
{
  /* RF SPI is supported omly */
  EMB6_ASSERT_RET(spi == EN_HAL_SPI_RF, NULL);

   /* initialize the according pins */
  s_hal_spi.p_clkPin = hal_pinInit( EN_HAL_PIN_RFSPICLK );
  EMB6_ASSERT_RET(s_hal_spi.p_clkPin != NULL, NULL);
  s_hal_spi.p_txPin = hal_pinInit( EN_HAL_PIN_RFSPITX );
  EMB6_ASSERT_RET(s_hal_spi.p_txPin != NULL, NULL);
  s_hal_spi.p_rxPin = hal_pinInit( EN_HAL_PIN_RFSPIRX );
  EMB6_ASSERT_RET(s_hal_spi.p_rxPin != NULL, NULL);
  s_hal_spi.p_csPin = hal_pinInit( EN_HAL_PIN_RFSPICS );
  EMB6_ASSERT_RET(s_hal_spi.p_csPin != NULL, NULL);

  hal_enterCritical();
  /* Run SPI at max speed */
  SPCR = ( 1 << SPE ) | ( 1 << MSTR ); /* Enable SPI module and master operation. */
  SPSR = ( 1 << SPI2X ); /* Enable doubled SPI speed in master mode. */
  hal_exitCritical();

  return &s_hal_spi;
} /* hal_spiInit() */


/*---------------------------------------------------------------------------*/
/*
* hal_spiTRx()
*/
int32_t hal_spiTRx( void* p_spi, uint8_t* p_tx, uint8_t* p_rx, uint16_t len )
{
  uint16_t lenTmp = len;
  EMB6_ASSERT_RET( p_spi != NULL, -1);
  EMB6_ASSERT_RET( p_rx != NULL, -1);
  EMB6_ASSERT_RET( p_tx != NULL, -1);
  EMB6_ASSERT_RET( p_spi == &s_hal_spi, -1);

  while( lenTmp-- )
  {
    /*Send Register address and write register content.*/
    ATM1281_SPI_TRAN_WRITE( *p_tx++ );
    ATM1281_SPI_TRAN_WAIT();
    *p_rx++ = ATM1281_SPI_TRAN_READ();
  }

  return len;

} /* hal_spiTRx() */


/*---------------------------------------------------------------------------*/
/*
* hal_spiRx()
*/
int32_t hal_spiRx( void* p_spi, uint8_t * p_rx, uint16_t len )
{
  uint16_t lenTmp = len;
  EMB6_ASSERT_RET( p_spi != NULL, -1);
  EMB6_ASSERT_RET( p_rx != NULL, -1);
  EMB6_ASSERT_RET( p_spi == &s_hal_spi, -1);

  while( lenTmp-- )
  {
    ATM1281_SPI_TRAN_WRITE( 0 );
    ATM1281_SPI_TRAN_WAIT();
    *p_rx++ = ATM1281_SPI_TRAN_READ();
  }
  return len;

} /* hal_spiRx() */


/*---------------------------------------------------------------------------*/
/*
* hal_spiTx()
*/
int32_t hal_spiTx( void* p_spi, uint8_t* p_tx, uint16_t len )
{
  uint16_t lenTmp = len;
  EMB6_ASSERT_RET( p_spi != NULL, -1);
  EMB6_ASSERT_RET( p_tx != NULL, -1);
  EMB6_ASSERT_RET( p_spi == &s_hal_spi, -1);

  while( lenTmp-- )
  {
      /*Send Register address and write register content.*/
    ATM1281_SPI_TRAN_WRITE( *p_tx++ );
    ATM1281_SPI_TRAN_WAIT();
  }

  return len;
} /* hal_spiTx() */
#endif /* #if defined(HAL_SUPPORT_SPI) && defined(HAL_SUPPORT_RFSPI) */


#if defined(HAL_SUPPORT_UART)
/*---------------------------------------------------------------------------*/
/*
* hal_uartInit()
*/
void* hal_uartInit( en_hal_uart_t uart )
{
  EMB6_ASSERT_RET( uart < EN_HAL_UART_MAX, NULL );

  /* Initialize the pins */
  //EMB6_ASSERT_RET( hal_pinInit( EN_HAL_PIN_SLIPUARTTX) != NULL, NULL );
  //EMB6_ASSERT_RET( hal_pinInit( EN_HAL_PIN_SLIPUARTRX) != NULL, NULL );

  /* Configure UART */
  UBRR(ATM1281_UART_SLIP_INST,H) = (uint8_t)(ATM1281_UART_SLIP_BAUD >> 8);
  UBRR(ATM1281_UART_SLIP_INST,L)= (uint8_t)ATM1281_UART_SLIP_BAUD;
  UCSR(ATM1281_UART_SLIP_INST,B) = (1 << RXCIE(ATM1281_UART_SLIP_INST)) |
      (1 << TXEN(ATM1281_UART_SLIP_INST)) |
      (1 << RXEN(ATM1281_UART_SLIP_INST));
  UCSR(ATM1281_UART_SLIP_INST,C)= ((1 << UCSZ(ATM1281_UART_SLIP_INST,1)) |
      (1 << UCSZ(ATM1281_UART_SLIP_INST,0)));

  return NULL;

}/* bsp_uartInit() */



/*---------------------------------------------------------------------------*/
/*
* hal_uartRx()
*/
int32_t hal_uartRx( void* p_uart, uint8_t * p_rx, uint16_t len )
{
  EMB6_ASSERT_RET( p_uart != NULL, -1 );
  EMB6_ASSERT_RET( p_rx != NULL, -1 );

  if( len == 0 )
    return 0;

  *p_rx = UDR(ATM1281_UART_SLIP_INST);
  return 1;
}/* hal_uartRx() */


/*---------------------------------------------------------------------------*/
/*
* hal_uartTx()
*/
int32_t hal_uartTx( void* p_uart, uint8_t* p_tx, uint16_t len )
{
  uint16_t lenTmp = len;
  EMB6_ASSERT_RET( p_uart != NULL, -1 );
  EMB6_ASSERT_RET( p_tx != NULL, -1 );

  if( len == 0 )
    return 0;

  while( lenTmp--)
  {
    while( ( UCSR(ATM1281_UART_SLIP_INST,A) &
        ( 1 << UDRE(ATM1281_UART_SLIP_INST) ) ) == 0 );
    UDR(ATM1281_UART_SLIP_INST) = *p_tx++;
  }
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
  /* Initialize the pins */
  //EMB6_ASSERT_RET( hal_pinInit( EN_HAL_PIN_DEBUGUARTTX) != NULL, -1 );
  //EMB6_ASSERT_RET( hal_pinInit( EN_HAL_PIN_DEBUGUARTRX) != NULL, -1 );
  /* Configure UART */
  UBRR(ATM1281_UART_DEBUG_INST,H) = (uint8_t)(ATM1281_UART_DEBUG_BAUD >> 8);
  UBRR(ATM1281_UART_DEBUG_INST,L)= (uint8_t)ATM1281_UART_DEBUG_BAUD;
  UCSR(ATM1281_UART_DEBUG_INST,B) = (1 << RXCIE(ATM1281_UART_DEBUG_INST)) |
      (1 << TXEN(ATM1281_UART_DEBUG_INST)) |
      (1 << RXEN(ATM1281_UART_DEBUG_INST));
  UCSR(ATM1281_UART_DEBUG_INST,C)= ((1 << UCSZ(ATM1281_UART_DEBUG_INST,1)) |
      (1 << UCSZ(ATM1281_UART_DEBUG_INST,0)));
  stdout = &st_usartStdout;
#endif /* #if (LOGGER_LEVEL > 0) && (HAL_SUPPORT_SLIPUART == FALSE) */
  return 0;
}

