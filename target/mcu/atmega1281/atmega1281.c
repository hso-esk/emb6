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
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
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
 */
/*============================================================================*/
/**
 * \addtogroup bsp
 * @{
 * \addtogroup mcu MCU HAL library
 * @{
 * \addtogroup atmega1281
 * @{
 */
/*! \file   atmega1281/atmega1281.c

 \author Artem Yushev 

 \brief  Hardware dependent initialization

 \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 ==============================================================================*/
#include "emb6.h"
#include "board_conf.h"
#include "avr/wdt.h"
#include "math.h"
#include "target.h"
#include "hwinit.h"
#include "bsp.h"
/*==============================================================================
 MACROS
 ==============================================================================*/

#define SPI_TRAN_OPEN()            hal_enterCritical(); \
        PORT( SSPORT ) &= ~( 1 << SSPIN );
#define SPI_TRAN_CLOSE()        PORT( SSPORT ) |= ( 1 << SSPIN ); \
        hal_exitCritical();
#define SPI_TRAN_WAIT()            while ((SPSR & (1 << SPIF)) == 0) {;}
#define SPI_TRAN_READ()            (SPDR)
#define SPI_TRAN_WRITE(c_value)    SPDR = (c_value);

/*==============================================================================
 ENUMS
 ==============================================================================*/

/*==============================================================================
 STRUCTURES AND OTHER TYPEDEFS
 ==============================================================================*/
// Spi discriptor typedef
typedef uint8_t spiDesc_t;
// Externa interrupt typedef
typedef uint8_t extInt_t;
// For port pin assignment
typedef uint8_t regType_t;

typedef struct pinDesc_t
{
    volatile regType_t *pc_ddr;
    volatile regType_t *pc_port;
    uint8_t c_pin;
} pinDesc_t;

/*==============================================================================
 LOCAL FUNCTION PROTOTYPES
 ==============================================================================*/

/* Init peripheral functions */
static void _hal_tim0Init( void );
static void _hal_ledsInit( void );
//        static void     _hal_tim1Init(void);
static void _hal_usartInit( void );
int _hal_uart1PutChar( char c, FILE * stream );
inline static void _hal_delay_loop( uint16_t __count ) __attribute__((always_inline));
/*==============================================================================
 VARIABLE DECLARATIONS
 ==============================================================================*/
clock_time_t volatile l_tick;
clock_time_t volatile l_sec;
static uint8_t volatile c_sreg;
static int8_t c_nested = 0;
static FILE st_usartStdout = FDEV_SETUP_STREAM( _hal_uart1PutChar, NULL,
        _FDEV_SETUP_WRITE );
pfn_intCallb_t isr_radioCallb = NULL;
pfn_intCallb_t isr_rxCallb = NULL;

//const     uint8_t mac_address[8] = {RADIO_MAC_ADDR};
/*==============================================================================
 LOCAL CONSTANTS
 ==============================================================================*/

/*==============================================================================
 LOCAL FUNCTIONS
 ==============================================================================*/
/*----------------------------------------------------------------------------*/
/** \brief  This function configure timer0 of the atmega1281 MCU
 *  \param         none
 *
 *  \return        none
 */
/*---------------------------------------------------------------------------*/
static void _hal_tim0Init( void )
{
    /* Select internal clock */
    ASSR = 0x00;

    /* Set counter to zero */
    TCNT0 = 0;

    /*
     * Set comparison register:
     * Crystal freq. is F_CPU, prescale is CONF_TMR0_PRESCALE,
     * We want CONF_TICK_SEC ticks / sec:
     * F_CPU = CONF_TMR0_PRESCALE * CONF_TICK_SEC * OCR0A, less 1 for CTC mode
     */
    OCR0A = F_CPU / CONF_TMR0_PRESCALE / CONF_TICK_SEC - 1;

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

/*----------------------------------------------------------------------------*/
/** \brief  This function configure timer1 of the atmega1281 MCU
 *  \param         none
 *
 *  \return        none
 */
/*---------------------------------------------------------------------------*/
//static void _hal_tim1Init(void)
//{
//    /*TIMER1 Specific Initialization.*/
//    TCCR1B                 =     ( 1 << ICES1 ) | ( 1 << CS11 ) | ( 1 << CS10 );       /* Set clock prescaler */
//    TIFR1                 |=     ( 1 << ICF1  );             /* Clear Input Capture Flag. */
//    TIMSK1                 |=     ( 1 << TOIE1 );             /* Enable Timer1 overflow interrupt. */
//
//} /* _hal_tim1Init() */
static void _hal_ledsInit( void )
{
#if defined BOARD_BLUEBEAN
    PORTB &= ~(1 << PB5);
    DDRB |= (1 << DDB5);

    PORTB &= ~(1 << PB6);
    DDRB |= (1 << DDB6);

    PORTB &= ~(1 << PB7);
    DDRB |= (1 << DDB7);
#endif

#if defined BOARD_ATANY_900
    PORTB |= (1 << PB5);
    DDRB |= (1 << DDB5);

    PORTB |= (1 << PB6);
    DDRB |= (1 << DDB6);

    PORTB |= (1 << PB7);
    DDRB |= (1 << DDB7);
#endif
}

/*----------------------------------------------------------------------------*/
/** \brief  This function configure UART1 of the atmega1281 MCU
 *  \param         none
 *
 *  \return        none
 */
/*---------------------------------------------------------------------------*/
static void _hal_usartInit( void )
{
    /* USART configured as follow:
     - BaudRate = 115200 baud
     - Word Length = 8 Bits
     - ONE Stop Bit
     - NO parity
     - Hardware flow control disabled (RTS and CTS signals)
     - Receive and transmit enabled
     */
    UBRR(USART,H) = (uint8_t)( USART_BAUD_38400 >> 8 );
    UBRR(USART,L) = (uint8_t)USART_BAUD_38400;
    UCSR(USART,B) = ( 1 << RXCIE( USART ) ) | ( 1 << TXEN( USART ) )
            | ( 1 << RXEN( USART ) );
    UCSR(USART,C) =
            ( ( 1 << UCSZ( USART, 1 ) ) | ( 1 << UCSZ( USART, 0 ) ) );

    stdout = &st_usartStdout;
} /* _hal_usartInit() */

/*----------------------------------------------------------------------------*/
/** \brief  This function outputs character using USART1.
 *
 *  \param  c        character for output
 *  \param    stream    file stream
 *  \retval None
 */
/*----------------------------------------------------------------------------*/
int _hal_uart1PutChar( char c, FILE * stream )
{
    // Do nothing until data have been received and is read to be read from UDR
    while( ( UCSR1A & ( 1 << UDRE1 ) ) == 0 )
    {
    };
    UDR1 = c;
    return 0;
} /* hal_uart_putchar() */

/*----------------------------------------------------------------------------*/
/** \brief  This function implements a low level delay as builin AVR delay
 *             function operates only with constant parameter.
 *
 *  \param  __count        How much ticks should be delayed.
 *  \retval None
 */
/*----------------------------------------------------------------------------*/
void _hal_delay_loop( uint16_t __count )
{
    __asm__ volatile (
            "1: sbiw %0,1" "\n\t"
            "brne 1b"
            : "=w" (__count)
            : "0" (__count)
    );
}

///*******************************************************************************
//* Function Name  : ISR(D_USART1_RX_vect)
//* Description    : This function handles USART1 global interrupt request.
//* Input          : None
//* Output         : None
//* Return         : None
//*******************************************************************************/
//ISR(USART1_RX_vect)
//{
//    uint8_t c;
//    /* Receive char*/
//    c = UDR1;
//    if (gpst_sciUart1)
//    {
//        /* Call handler*/
//        sci_receive(gpst_sciUart1, c);
//    }
//} /* ISR(D_USART1_RX_vect) */

/*******************************************************************************
 * Function Name  : TIMER0_COMPA_vect
 * Description    : This function handles TIM1 overflow and update interrupt
 *                  request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
ISR( TIMER0_COMPA_vect )
{
    /* Indicate timer update to the emb6 timer */
    if( l_tick % CONF_TICK_SEC == 0 )
        l_sec++;
    l_tick++;
} /* AVR_OUTPUT_COMPARE_INT() */

ISR( INT5_vect )
{
    if( isr_radioCallb != NULL )
        isr_radioCallb( NULL );
}

ISR( USART1_RX_vect )
{
    char c_rxbyte;
    c_rxbyte = UDR1; // Fetch the received byte value into the variable "ByteReceived"
    if( isr_rxCallb != NULL )
        isr_rxCallb( &c_rxbyte );
}
/*==============================================================================
 API FUNCTIONS
 ==============================================================================*/
/*==============================================================================
 hal_enterCritical()
 ==============================================================================*/
void hal_enterCritical( void )
{
    if( !c_nested )
    {
        c_sreg = SREG;
        cli();
        c_nested++;
    }
    else
        c_nested++;
} /* hal_enterCritical() */

/*==============================================================================
 hal_exitCritical()
 ==============================================================================*/
void hal_exitCritical( void )
{
    if( c_nested == 1 )
    {
        SREG = c_sreg;
        sei();
        c_nested--;
    }
    else
        c_nested--;
}/* hal_exitCritical() */

/*==============================================================================
 hwinit_init()
 ==============================================================================*/
int8_t hal_init( void )
{
    hal_enterCritical();
    /* Init USART*/
    _hal_usartInit();
    /* Init timer*/
    _hal_tim0Init();
    /* Init leds */
#if    LEDS_ON_BOARD == TRUE
    _hal_ledsInit();
#endif /* LEDS_ON_BOARD == TRUE */
    //    _hal_tim1Init();
    //    LOG_INFO("%s\n\r","Checking MCUSR...");
    //    if(MCUSR & (1<<PORF )) LOG_INFO("%s\n\r","Power-on reset.");
    //    if(MCUSR & (1<<EXTRF)) LOG_INFO("%s\n\r","External reset!");
    //    if(MCUSR & (1<<BORF )) LOG_INFO("%s\n\r","Brownout reset!");
    //    if(MCUSR & (1<<WDRF )) LOG_INFO("%s\n\r","Watchdog reset!");
    //    if(MCUSR & (1<<JTRF )) LOG_INFO("%s\n\r","JTAG reset!\n");
    MCUSR = 0;
    hal_exitCritical();
    return 1;
}/* hal_init() */

/*==============================================================================
 hal_getrand()
 ==============================================================================*/
uint8_t hal_getrand( void )
{
    uint8_t ret = 0;

    ADMUX = 0x1E; //Select AREF as reference, measure 1.1 volt bandgap reference.
    ADCSRA = 1 << ADEN; //Enable ADC, not free running, interrupt disabled, fastest clock
    ADCSRA |= 1 << ADSC;          //Start conversion
    while( ADCSRA & ( 1 << ADSC ) )
        ; //Wait till done
    ret = ADC;
    ADCSRA = 0;                 //Disable ADC
    return ret;
}/* hal_getrand() */

void hal_ledOn( uint16_t ui_led )
{
#if    LEDS_ON_BOARD == TRUE
    hal_enterCritical();
    switch (ui_led)
    {
        case E_BSP_LED_RED:
        PORT(HAL_LED_RED_PORT) HAL_LED_SET ( 1 << PIN(HAL_LED_RED_PIN));
        break;
        case E_BSP_LED_YELLOW:
        PORT(HAL_LED_YELLOW_PORT) HAL_LED_SET ( 1 << PIN(HAL_LED_YELLOW_PIN));
        break;
        case E_BSP_LED_GREEN:
        PORT(HAL_LED_GREEN_PORT) HAL_LED_SET ( 1 << PIN(HAL_LED_GREEN_PIN));
        break;
        default:
        break;
    }
    hal_exitCritical();
#endif  /* LEDS_ON_BOARD == TRUE */
}

void hal_ledOff( uint16_t ui_led )
{
#if    LEDS_ON_BOARD == TRUE
    hal_enterCritical();
    switch (ui_led)
    {
        case E_BSP_LED_RED:
        PORT(HAL_LED_RED_PORT) HAL_LED_CLR ( 1 << PIN(HAL_LED_RED_PIN));
        break;
        case E_BSP_LED_YELLOW:
        PORT(HAL_LED_YELLOW_PORT) HAL_LED_CLR ( 1 << PIN(HAL_LED_YELLOW_PIN));
        break;
        case E_BSP_LED_GREEN:
        PORT(HAL_LED_GREEN_PORT) HAL_LED_CLR ( 1 << PIN(HAL_LED_GREEN_PIN));
        break;
        default:
        break;
    }
    hal_exitCritical();
#endif  /* LEDS_ON_BOARD == TRUE */
}

/*==============================================================================
 hal_extIntInit()
 =============================================================================*/
uint8_t hal_extIntInit( en_targetExtInt_t e_intSource,
        pfn_intCallb_t pfn_intCallback )
{
    int8_t c_ret = 0;
    hal_enterCritical();

    if( pfn_intCallback == NULL )
    {
        //        LOG_ERR("You are tying to initialise NULL pointer function.");
        return 0;
    }

    switch( e_intSource )
    {
        case E_TARGET_RADIO_INT:
            EIMSK |= ( 1 << INT5 );
            EICRB |= ( ( 1 << ISC51 ) | ( 1 << ISC50 ) );
            PORTE &= ~( 1 << PE5 );
            DDRE &= ~( 1 << DDE5 );
            isr_radioCallb = pfn_intCallback;
            c_ret = 1;
            break;
        case E_TARGET_USART_INT:
            isr_rxCallb = pfn_intCallback;
            break;
        default:
            c_ret = 0;
            break;
    }
    hal_exitCritical();
    return c_ret;

} /* hal_extIntInit() */

/*==============================================================================
 hal_delay_us()
 =============================================================================*/
void hal_delay_us( uint32_t i_delay )
{
    if( i_delay < 4 )
        return;
    i_delay -= 3;
    _hal_delay_loop( ( i_delay * 4 ) / 2 );
} /* hal_delay_us() */

/*==============================================================================
 hal_ctrlPinInit()
 =============================================================================*/
void * hal_ctrlPinInit( en_targetExtPin_t e_pinType )
{
    pinDesc_t * p_pin = (pinDesc_t *)malloc( sizeof(pinDesc_t) );

    if( p_pin != NULL )
    {
        switch( e_pinType )
        {
            case E_TARGET_RADIO_RST:
                p_pin->pc_ddr = RESET_DDR;
                p_pin->pc_port = RESET_PORT;
                p_pin->c_pin = RESET_PIN;

                ( *( p_pin->pc_port ) ) &= ~( 1 << p_pin->c_pin );
                ( *( p_pin->pc_ddr ) ) |= ( 1 << p_pin->c_pin );

                break;
            case E_TARGET_RADIO_SLPTR:
                p_pin->pc_ddr = SLP_TR_DDR;
                p_pin->pc_port = SLP_TR_PORT;
                p_pin->c_pin = SLP_TR_PIN;

                ( *( p_pin->pc_port ) ) &= ~( 1 << p_pin->c_pin );
                ( *( p_pin->pc_ddr ) ) |= ( 1 << p_pin->c_pin );

                break;
            default:
                break;
        }
    }
    return (void *)p_pin;
} /* hal_ctrlPinInit */

/*==============================================================================
 hal_pinInit()
 =============================================================================*/
void * hal_pinInit( en_targetExtPin_t e_pinType )
{
    pinDesc_t * p_pin = (pinDesc_t *)malloc( sizeof(pinDesc_t) );

    if( p_pin != NULL )
    {
        switch( e_pinType )
        {
            case E_TARGET_RADIO_RST:
                p_pin->pc_ddr = RESET_DDR;
                p_pin->pc_port = RESET_PORT;
                p_pin->c_pin = RESET_PIN;

                ( *( p_pin->pc_port ) ) &= ~( 1 << p_pin->c_pin );
                ( *( p_pin->pc_ddr ) ) |= ( 1 << p_pin->c_pin );

                break;
            case E_TARGET_RADIO_SLPTR:
                p_pin->pc_ddr = SLP_TR_DDR;
                p_pin->pc_port = SLP_TR_PORT;
                p_pin->c_pin = SLP_TR_PIN;

                ( *( p_pin->pc_port ) ) &= ~( 1 << p_pin->c_pin );
                ( *( p_pin->pc_ddr ) ) |= ( 1 << p_pin->c_pin );

                break;
            default:
                break;
        }
    }
    return (void *)p_pin;
}
/*==============================================================================
 hal_pinSet()
 =============================================================================*/
void hal_pinSet( void * p_pin )
{
    pinDesc_t * st_pin = (pinDesc_t *)p_pin;
    if( p_pin == NULL )
        return;
    hal_enterCritical();
    ( *( st_pin->pc_port ) ) |= ( 1 << st_pin->c_pin );
    hal_exitCritical();
} /* hal_pinSet() */

/*==============================================================================
 hal_pinClr()
 =============================================================================*/
void hal_pinClr( void * p_pin )
{
    pinDesc_t * st_pin = (pinDesc_t *)p_pin;
    if( p_pin == NULL )
        return;
    hal_enterCritical();
    ( *( st_pin->pc_port ) ) &= ~( 1 << st_pin->c_pin );
    hal_exitCritical();
} /* hal_pinClr() */

/*==============================================================================
 hal_pinGet()
 =============================================================================*/
uint8_t hal_pinGet( void * p_pin )
{
    pinDesc_t * st_pin = (pinDesc_t *)p_pin;
    if( p_pin == NULL )
        return 0xFF;
    // (PORTX & (1 << PINX)) >> PINX
    return ( ( *( st_pin->pc_port ) ) & ( 1 << st_pin->c_pin ) );
} /* hal_pinGet() */

/*==============================================================================
 hal_spiInit()
 =============================================================================*/
void * hal_spiInit( void )
{
    // Atmega1281 has only one spi interface.
    spiDesc_t * p_spi = (spiDesc_t *)malloc( sizeof(spiDesc_t) );

    if( p_spi != NULL )
    {
        memset( p_spi, 1, sizeof(spiDesc_t) );
        hal_enterCritical();
        /*SPI Specific Initialization.*/
        /* Set SS, CLK and MOSI as output. */
        /* To avoid a SPI glitch, the port register shall be set before the DDR register */
        PORT( SPIPORT ) |= ( 1 << SSPIN ) | ( 1 << SCKPIN ); /* Set SS and CLK high */
        DDR( SPIPORT ) |= ( 1 << SSPIN ) | ( 1 << SCKPIN )
                | ( 1 << MOSIPIN );
        DDR( SPIPORT ) &= ~( 1 << MISOPIN ); /* MISO input */

        /* Run SPI at max speed */
        SPCR = ( 1 << SPE ) | ( 1 << MSTR ); /* Enable SPI module and master operation. */
        SPSR = ( 1 << SPI2X ); /* Enable doubled SPI speed in master mode. */
        hal_exitCritical();
        *p_spi = 1;
    }
    return p_spi;
} /* hal_spiInit() */

/*==============================================================================
 hal_spiSlaveSel()
 =============================================================================*/
//!  [hal_spiSlaveSel]
uint8_t hal_spiSlaveSel( void * p_spi, bool action )
{
    if( p_spi == 0 )
    {
        //        LOG_ERR("%s\n\r","SPI was not initialized!");
        return 0;
    }
    if( action )
    {
        //! [select_slave]
        SPI_TRAN_OPEN()
        ;
        //! [select_slave]
    }
    else
    {
        //! [deselect_slave]
        SPI_TRAN_CLOSE()
        ;
        //! [deselect_slave]
    }
    return 1;
} //!  [hal_spiSlaveSel]

/*==============================================================================
 hal_spiRead()
 =============================================================================*/
uint8_t hal_spiRead( uint8_t * p_reg, uint16_t i_length )
{
    while( i_length-- )
    {
        SPI_TRAN_WRITE( 0 );
        SPI_TRAN_WAIT();
        *p_reg++ = SPI_TRAN_READ();
    }
    return *p_reg;
} /* hal_spiRead() */

/*==============================================================================
 hal_spiWrite()
 =============================================================================*/
void hal_spiWrite( uint8_t * c_value, uint16_t i_length )
{
    while( i_length-- )
    {
        /*Send Register address and write register content.*/
        SPI_TRAN_WRITE( *c_value++ );
        SPI_TRAN_WAIT();
    }
} /* hal_spiWrite() */

/*==============================================================================
 hal_spiTranRead()
 =============================================================================*/
void hal_watchdogReset( void )
{
    //    wdt_reset();
} /* hal_watchdogReset() */

/*==============================================================================
 hal_spiTranRead()
 =============================================================================*/
void hal_watchdogStart( void )
{
    //wdt_enable(WDTO_2S);
} /* hal_watchdogStart() */

/*==============================================================================
 hal_spiTranRead()
 =============================================================================*/
void hal_watchdogStop( void )
{
    //    wdt_disable();
} /* hal_watchdogStop() */

/*==============================================================================
 hal_getTick()
 =============================================================================*/
clock_time_t hal_getTick( void )
{
    return l_tick;
} /* hal_getTick() */

/*==============================================================================
 hal_getSec()
 =============================================================================*/
clock_time_t hal_getSec( void )
{
    return l_sec;
} /* hal_getSec() */

clock_time_t hal_getTRes( void )
{
    return CLOCK_SECOND;
}

/** @} */
/** @} */
/** @} */
