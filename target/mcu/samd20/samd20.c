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
 * \addtogroup samd20g18
 * @{
 */
/*! \file   samd20/samd20.c

 \author Artem Yushev

 \brief  Hardware dependent initialization for SAMD20G18

 \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 ==============================================================================*/
#include "emb6.h"
#include "emb6_conf.h"
#include "board_conf.h"
#include "math.h"
#include "target.h"
#include "hwinit.h"
#include "bsp.h"

#include "stdio_serial.h"

/*==============================================================================
 MACROS
 ==============================================================================*/
#define     LOGGER_ENABLE        LOGGER_HAL
#include    "logger.h"

/*==============================================================================
 ENUMS
 ==============================================================================*/

/*==============================================================================
 STRUCTURES AND OTHER TYPEDEFS
 ==============================================================================*/

/*==============================================================================
 LOCAL FUNCTION PROTOTYPES
 ==============================================================================*/

/* Init peripheral functions */
static void _hal_tcInit( void );
static void _hal_wdtInit( void );
static void _isr_tc_interrupt( void );
static void _hal_usartInit( void );
pfn_intCallb_t _radio_callback = NULL;
void _isr_radio_callback( uint32_t channel );
/*==============================================================================
 VARIABLE DECLARATIONS
 ==============================================================================*/
static clock_time_t volatile l_tick;
static clock_time_t volatile l_sec;
static struct usart_module st_usartInst;
static struct spi_module st_masterInst;
static struct spi_slave_inst st_spi;
struct rtc_module rtc_instance;

typedef struct spi_slave_inst spiDesc_t;
typedef uint16_t regType_t;
typedef uint8_t pinDesc_t;

typedef struct extInt
{
    struct extint_chan_conf st_chan;
    uint32_t l_pin;
    uint32_t l_pinMux;
    uint8_t c_pinPull;
    uint8_t c_detCrit;
    uint32_t l_line;
} extInt_t;

//const     uint8_t mac_address[8] = {RADIO_MAC_ADDR};

pinDesc_t st_slp_tr = SAMD20_SPI0_SLP_PIN;
pinDesc_t st_rst = SAMD20_SPI0_RST_PIN;
pinDesc_t st_ss = SAMD20_SPI0_CS_PIN;
/* Definition of a external pin for xplainedpro board and rf212 */
extInt_t radio_extInt = { { }, SAMD20_RADIO_IRQ_PIN, SAMD20_RADIO_IRQ_PINMUX,
        EXTINT_PULL_NONE, EXTINT_DETECT_RISING, SAMD20_RADIO_IRQ_INPUT };
/*==============================================================================
 LOCAL CONSTANTS
 ==============================================================================*/

/*==============================================================================
 LOCAL FUNCTIONS
 ==============================================================================*/
/*----------------------------------------------------------------------------*/
/** \brief  This function configure timer counter of the atmel samd20 MCU
 *  \param         none
 *
 *  \return        none
 */
/*---------------------------------------------------------------------------*/
static void _hal_tcInit( void )
{
    //! [init_conf]
    struct rtc_count_config config_rtc_count;
    rtc_count_get_config_defaults( &config_rtc_count );
    //! [init_conf]

    //! [set_config]
    config_rtc_count.prescaler = RTC_COUNT_PRESCALER_DIV_1;
    config_rtc_count.mode = RTC_COUNT_MODE_16BIT;
    config_rtc_count.continuously_update = true;
    //! [set_config]
    //! [init_rtc]
    rtc_count_init( &rtc_instance, RTC, &config_rtc_count );
    //! [init_rtc]

    //! [period]
    rtc_count_set_period( &rtc_instance, 32 );
    //! [period]

    //! [reg_callback]
    rtc_count_register_callback( &rtc_instance, _isr_tc_interrupt,
            RTC_COUNT_CALLBACK_OVERFLOW );
    //! [reg_callback]
    //! [en_callback]
    rtc_count_enable_callback( &rtc_instance, RTC_COUNT_CALLBACK_OVERFLOW );
    //! [en_callback]

    //! [enable]
    rtc_count_enable( &rtc_instance );
    //! [enable]
} /* _hal_tcInit() */

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
     - BaudRate = 34800 baud
     - Word Length = 8 Bits
     - ONE Stop Bit
     - NO parity
     - Hardware flow control disabled (RTS and CTS signals)
     - Receive and transmit enabled
     */
//! [setup_config]
    struct usart_config config_usart;
//! [setup_config]
//! [setup_config_defaults]
    usart_get_config_defaults( &config_usart );
//! [setup_config_defaults]

//! [setup_change_config]
    config_usart.baudrate = SAMD20_USART0_BAUDRATE;
    config_usart.mux_setting = SAMD20_USART0_SERCOM_MUX_SETTING;
    config_usart.pinmux_pad0 = SAMD20_USART0_SERCOM_PMUX0;
    config_usart.pinmux_pad1 = SAMD20_USART0_SERCOM_PMUX1;
    config_usart.pinmux_pad2 = SAMD20_USART0_SERCOM_PMUX2;
    config_usart.pinmux_pad3 = SAMD20_USART0_SERCOM_PMUX3;
//! [setup_change_config]

//! [setup_set_config]
    while( usart_init( &st_usartInst, SAMD20_USART0_SERCOM, &config_usart )
            != STATUS_OK )
    {
    }
//! [setup_set_config]

//! [setup_enable]
    usart_enable( &st_usartInst );
//! [setup_enable]

    // Initialize Serial Interface using Stdio Library
    stdio_serial_init( &st_usartInst, SAMD20_USART0_SERCOM, &config_usart );
    //stdout = &st_usartStdout;
} /* _hal_usartInit() */

/*----------------------------------------------------------------------------*/
/** \brief  This function configure watchdog timer of the atmel samd20 MCU
 *  \param         none
 *
 *  \return        none
 */
/*---------------------------------------------------------------------------*/
//! [wdt_setup]
void _hal_wdtInit( void )
{
    /* Create a new configuration structure for the Watchdog settings and fill
     * with the default module settings. */
    //! [setup_1]
    struct wdt_conf config_wdt;
    //! [setup_1]
    //! [setup_2]
    wdt_get_config_defaults( &config_wdt );
    //! [setup_2]

    /* Set the Watchdog configuration settings */
    //! [setup_3]
    config_wdt.always_on = false;
    config_wdt.clock_source = GCLK_GENERATOR_4;
    config_wdt.timeout_period = WDT_PERIOD_2048CLK;
    //! [setup_3]

    /* Initialize and enable the Watchdog with the user settings */
    //! [setup_4]
//    wdt_init(&config_wdt);
    //! [setup_4]
    // It is not a good idea to use watchdog timers
    //! [setup_5]
//    wdt_enable();
    //! [setup_5]
} //! [wdt_setup]

void _hal_ledInit( void )
{
#if    LEDS_ON_BOARD == TRUE
    struct port_config pin_conf;
    port_get_config_defaults(&pin_conf);

    /* Configure LEDs as outputs, turn them off */
    pin_conf.direction = PORT_PIN_DIR_OUTPUT;
    port_pin_set_config(LED_0_PIN, &pin_conf);
    port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);
    port_pin_set_config(LED_1_PIN, &pin_conf);
    port_pin_set_output_level(LED_1_PIN, LED_1_INACTIVE);
    port_pin_set_config(LED_2_PIN, &pin_conf);
    port_pin_set_output_level(LED_2_PIN, LED_2_INACTIVE);
#endif
}

void _isr_tc_interrupt( void )
{
    /* Indicate timer update to the emb6 timer */
    if( l_tick % CONF_TICK_SEC == 0 )
        l_sec++;
    l_tick++;
} /* _isr_tc_interrupt() */

void _isr_radio_callback( uint32_t channel )
{
    if( _radio_callback != NULL )
        _radio_callback( NULL );
}

/*==============================================================================
 API FUNCTIONS
 ==============================================================================*/
/*==============================================================================
 hal_enterCritical()
 ==============================================================================*/
void hal_enterCritical( void )
{
    system_interrupt_enter_critical_section();
} /* hal_enterCritical() */

/*==============================================================================
 hal_exitCritical()
 ==============================================================================*/
void hal_exitCritical( void )
{
    system_interrupt_leave_critical_section();
}/* hal_exitCritical() */

/*==============================================================================
 hwinit_init()
 ==============================================================================*/
int8_t hal_init( void )
{
    system_init();

    _hal_usartInit();
    _hal_wdtInit();
    _hal_tcInit();
    _hal_ledInit();
    delay_init();
    system_interrupt_enable_global();
    return 1;
}/* hal_init() */

/*==============================================================================
 hal_getrand()
 ==============================================================================*/
uint8_t hal_getrand( void )
{
    // TODO implement this function
    return 1;
}/* hal_getrand() */

/*==============================================================================
 hal_ledOff()
 ==============================================================================*/
#if    LEDS_ON_BOARD == TRUE
void hal_ledOff(uint16_t ui_led)
{
    hal_enterCritical();
    switch (ui_led)
    {
        case E_BSP_LED_RED:
        port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);
        break;
        case E_BSP_LED_YELLOW:
        port_pin_set_output_level(LED_1_PIN, LED_1_INACTIVE);
        break;
        case E_BSP_LED_GREEN:
        port_pin_set_output_level(LED_2_PIN, LED_2_INACTIVE);
        break;
        default:
        break;
    }
    hal_exitCritical();
}
#endif  /* LEDS_ON_BOARD == TRUE */
/*==============================================================================
 hal_ledOn()
 ==============================================================================*/
#if    LEDS_ON_BOARD == TRUE
void hal_ledOn(uint16_t ui_led)
{
    hal_enterCritical();
    switch (ui_led)
    {
        case E_BSP_LED_RED:
        port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
        break;
        case E_BSP_LED_YELLOW:
        port_pin_set_output_level(LED_1_PIN, LED_1_ACTIVE);
        break;
        case E_BSP_LED_GREEN:
        port_pin_set_output_level(LED_2_PIN, LED_2_ACTIVE);
        break;
        default:
        break;
    }
    hal_exitCritical();
}
#endif  /* LEDS_ON_BOARD == TRUE */
/*==============================================================================
 hal_extIntInit()
 =============================================================================*/
uint8_t hal_extIntInit( en_targetExtInt_t e_intSource,
        pfn_intCallb_t pfn_intCallback )
{
    int8_t c_ret = 0;

    if( pfn_intCallback != NULL )
    {
        switch( e_intSource )
        {
            case E_TARGET_RADIO_INT:
                //! [setup_init]
                //
                //! [conf_channel]
                //! [setup_1]
                extint_chan_get_config_defaults( &( radio_extInt.st_chan ) );
                //! [setup_1]

                //! [setup_2]
                radio_extInt.st_chan.gpio_pin = radio_extInt.l_pin; //EXT1_IRQ_PIN;
                radio_extInt.st_chan.gpio_pin_mux = radio_extInt.l_pinMux; //MUX_PB04A_EIC_EXTINT4;
                radio_extInt.st_chan.gpio_pin_pull = radio_extInt.c_pinPull; //EXTINT_PULL_DOWN;
                radio_extInt.st_chan.detection_criteria =
                        radio_extInt.c_detCrit; //EXTINT_DETECT_RISING;
                //! [setup_2]

                //! [setup_3]
                extint_chan_set_config( (uint8_t)radio_extInt.l_line,
                        &( radio_extInt.st_chan ) );
                //! [setup_3]

                _radio_callback = pfn_intCallback;
                //! [setup_4]
                extint_register_callback(
                        (extint_callback_t)_isr_radio_callback,
                        (uint8_t)radio_extInt.l_line,
                        EXTINT_CALLBACK_TYPE_DETECT );
                //! [setup_4]
                //! [setup_5]
                extint_chan_enable_callback( (uint8_t)radio_extInt.l_line, //EXT1_IRQ_INPUT,
                        EXTINT_CALLBACK_TYPE_DETECT );
                //! [setup_6]

                //! [conf_channel]
                //! [setup_init]
                system_interrupt_enable_global();
                c_ret = 1;
                break;
            case E_TARGET_USART_INT:
                break;
            default:
                break;
        }
    }
    return c_ret;

} /* hal_extIntInit() */

/*==============================================================================
 hal_delay_us()
 =============================================================================*/
void hal_delay_us( uint32_t i_delay )
{
    delay_us( i_delay );
} /* hal_delay_us() */

uint8_t hal_gpioPinInit( uint8_t c_pin, uint8_t c_dir, uint8_t c_initState )
{
    struct port_config pin_conf;
    port_get_config_defaults( &pin_conf );

    /* Configure SPI interface */
    port_get_config_defaults( &pin_conf );

    pin_conf.direction = c_dir;
    port_pin_set_config( c_pin, &pin_conf );
    port_pin_set_output_level( c_pin, c_initState );

    return 0;
}

/*==============================================================================
 hal_ctrlPinInit()
 =============================================================================*/
void * hal_ctrlPinInit( en_targetExtPin_t e_pinType )
{
    struct port_config pin_conf;
    port_get_config_defaults( &pin_conf );
    pinDesc_t * p_pin = NULL;

    /* Configure SPI interface */
    port_get_config_defaults( &pin_conf );

    switch( e_pinType )
    {
        case E_TARGET_RADIO_RST:
            pin_conf.direction = PORT_PIN_DIR_OUTPUT;
            port_pin_set_config( st_rst, &pin_conf );    //    RST_PIN
            port_pin_set_output_level( st_rst, true );    //    RST_PIN
            p_pin = &st_rst;
            break;
        case E_TARGET_RADIO_SLPTR:
            pin_conf.direction = PORT_PIN_DIR_OUTPUT;
            port_pin_set_config( st_slp_tr, &pin_conf );    //    SLP_PIN
            port_pin_set_output_level( st_slp_tr, true );    //    SLP_PIN
            p_pin = &st_slp_tr;
            break;
        default:
            free( p_pin );
            break;
    }

    return p_pin;
} /* hal_ctrlPinInit */

/*==============================================================================
 hal_pinSet()
 =============================================================================*/
void hal_pinSet( void * p_pin )
{
    hal_enterCritical();
    port_pin_set_output_level( *( (pinDesc_t *)p_pin ), true );
    hal_exitCritical();
} /* hal_pinSet() */

/*==============================================================================
 hal_pinClr()
 =============================================================================*/
void hal_pinClr( void * p_pin )
{
    hal_enterCritical();
    port_pin_set_output_level( *( (pinDesc_t *)p_pin ), false );
    hal_exitCritical();
} /* hal_pinClr() */

/*==============================================================================
 hal_pinGet()
 =============================================================================*/
uint8_t hal_pinGet( void * p_pin )
{
    return (uint8_t)port_pin_get_output_level( *( (pinDesc_t *)p_pin ) );
} /* hal_pinGet() */

/*==============================================================================
 hal_spiInit()
 =============================================================================*/
void * hal_spiInit( void )
{
    //! [slave_config]
    struct spi_slave_inst_config st_slaveDevConf;
    //! [slave_config]
    //! [master_config]
    struct spi_config st_masterConf;
    //! [master_config]

    /* Configure and initialize software device instance of peripheral slave */
    //! [slave_conf_defaults]
    spi_slave_inst_get_config_defaults( &st_slaveDevConf );
    //! [slave_conf_defaults]
    //! [ss_pin]
    st_slaveDevConf.ss_pin = st_ss;
    //! [ss_pin]
    //! [slave_init]
    spi_attach_slave( &st_spi, &st_slaveDevConf );
    //! [slave_init]
    /* Configure, initialize and enable SERCOM SPI module */
    //! [conf_defaults]
    spi_get_config_defaults( &st_masterConf );

    //! [conf_defaults]
    //! [mux_setting]
    st_masterConf.mux_setting = SAMD20_SPI0_SERCOM_MUX_SETTING;
    //! [mux_setting]
    /* Configure pad 0 for data in */
    //! [di]
    st_masterConf.pinmux_pad0 = SAMD20_SPI0_SERCOM_PMUX0;
    //! [di]
    /* Configure pad 1 as unused */
    //! [ss]
    st_masterConf.pinmux_pad1 = PINMUX_UNUSED;
    //! [ss]
    /* Configure pad 2 for data out */
    //! [do]
    st_masterConf.pinmux_pad2 = SAMD20_SPI0_SERCOM_PMUX2;
    //! [do]
    /* Configure pad 3 for SCK */
    //! [sck]
    st_masterConf.pinmux_pad3 = SAMD20_SPI0_SERCOM_PMUX3;
    //! [sck]

    //! [init]
    spi_init( &st_masterInst, SAMD20_SPI0_SERCOM, &st_masterConf );
    //! [init]

    //! [enable]
    spi_enable( &st_masterInst );

    //! [enable]

    return ( (void *)&st_spi );
} /* hal_spiInit() */

/*==============================================================================
 hal_spiSlaveSel()
 =============================================================================*/
//!  [hal_spiSlaveSel]
uint8_t hal_spiSlaveSel( void * p_spi, bool action )
{
    if( p_spi == NULL )
    {
//        LOG_ERR("SPI was not initialized!");
        return 0;
    }
    if( action )
    {
        //! [select_slave]
        hal_enterCritical();
        spi_select_slave( &st_masterInst, (spiDesc_t *)p_spi, true );
        //! [select_slave]
    }
    else
    {
        //! [deselect_slave]
        spi_select_slave( &st_masterInst, (spiDesc_t *)p_spi, false );
        hal_exitCritical();
        //! [deselect_slave]
    }
    return 1;
} //!  [hal_spiSlaveSel]

/*==============================================================================
 hal_spiRead()
 =============================================================================*/
uint8_t hal_spiRead( uint8_t * p_reg, uint16_t i_length )
{
    spi_read_buffer_wait( &st_masterInst, p_reg, i_length, 0 );
    return *p_reg;
} /* hal_spiRead() */

/*==============================================================================
 hal_spiWrite()
 =============================================================================*/
void hal_spiWrite( uint8_t * c_value, uint16_t i_length )
{
    if( spi_write_buffer_wait( &st_masterInst, c_value, i_length )
            != STATUS_OK )
//        LOG_ERR("%s\n\r","SPI write error!");
        ;
} /* hal_spiWrite() */

/*==============================================================================
 hal_spiTranRead()
 =============================================================================*/
void hal_watchdogReset( void )
{
//    wdt_reset_count();
} /* hal_watchdogReset() */

/*==============================================================================
 hal_spiTranRead()
 =============================================================================*/
void hal_watchdogStart( void )
{
    //wdt_enable();
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

/*==============================================================================
 hal_getTRes()
 =============================================================================*/
clock_time_t hal_getTRes( void )
{
    return CLOCK_SECOND;
} /* hal_getSec() */

/** @} */
/** @} */
/** @} */
