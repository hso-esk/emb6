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
 *  --- Module Description ---------------------------------------------------*
 */
/**
 *  \file       hal.h
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Interface description for the Hardware Abstraction Layer.
 *
 *              This files provides the Hardware Abstraction Interface that is
 *              from within emb::6 to access hardware related functions. In
 *              case a new platform shall be introduced, this interface has
 *              to be implemented.
 */
#ifndef __HAL_H__
#define __HAL_H__

/*
 *  --- Includes -------------------------------------------------------------*
 */
#include "emb6.h"
#include "board_conf.h"

/*
 * --- Macro Definitions --------------------------------------------------- *
 */

/* Enable SPI access a soon as one of the SPI interfaces is supported. */
#if defined(HAL_SUPPORT_LED0) || defined(HAL_SUPPORT_LED1) || defined(HAL_SUPPORT_LED2) || defined(HAL_SUPPORT_LED3)
#define HAL_SUPPORT_LED             TRUE
#endif /* #if defined(HAL_SUPPORT_LED0) || defined(HAL_SUPPORT_LED1) || defined(HAL_SUPPORT_LED2) || defined(HAL_SUPPORT_LED3) */


/* Enable SPI access a soon as one of the SPI interfaces is supported. */
#if defined(HAL_SUPPORT_RFSPI)
#define HAL_SUPPORT_SPI             TRUE
#endif /* #if defined(HAL_SUPPORT_RFSPI) */


/* Enable UART access a soon as one of the UART interfaces is supported. */
#if defined(HAL_SUPPORT_SLIPUART) || defined(HAL_SUPPORT_USERUART)
#define HAL_SUPPORT_UART            TRUE
#endif /* #if defined(HAL_SUPPORT_UART) */


/** Maximum number of LEDs */
#define HAL_NUM_LED_MAX             5

/** LED 0 */
#define HAL_LED0                    (1 << 0)
/** LED ! */
#define HAL_LED1                    (1 << 1)
/** LED 2 */
#define HAL_LED2                    (1 << 2)
/** LED 3 */
#define HAL_LED3                    (1 << 3)
/** LED 4 */
#define HAL_LED4                    (1 << 4)

#if (HAL_SUPPORT_LEDNUM > HAL_NUM_LED_MAX)
#error "Invalid configuration for HAL_SUPPORT_LEDNUM"
#else
/** number of available LEDs */
#define HAL_NUM_LEDS                (HAL_SUPPORT_LEDNUM)
#endif /* #if (HAL_SUPPORT_LEDNUM > HAL_NUM_LED_MAX) */


/** number of available SPIs */
#define HAL_NUM_SPIS                (EN_HAL_SPI_MAX)

/*
 *  --- Type Definitions -----------------------------------------------------*
 */

/**
 * \brief   Describes the available pins.
 *
 *          emb::6 uses several pins during its operation that need to be
 *          configured dynamically. Therefore this enumeration provides
 *          a list of all the pins that are available to emb::6 and which
 *          must be implemented by the according HAL implementation.
 */
typedef enum EN_HAL_PIN_T
{
#if (HAL_SUPPORT_LEDNUM > 0)
    /** LED0 */
    EN_HAL_PIN_LED0,
#endif /* (HAL_SUPPORT_LEDNUM > 0) */
#if (HAL_SUPPORT_LEDNUM > 1)
    /** LED1 */
    EN_HAL_PIN_LED1,
#endif /* #if (HAL_SUPPORT_LEDNUM > 1) */
#if (HAL_SUPPORT_LEDNUM > 2)
    /** LED2 */
    EN_HAL_PIN_LED2,
#endif /* #if (HAL_SUPPORT_LEDNUM > 2) */
#if (HAL_SUPPORT_LEDNUM > 3)
    /** LED3 */
    EN_HAL_PIN_LED3,
#endif /* #if (HAL_SUPPORT_LEDNUM > 3) */
#if (HAL_SUPPORT_LEDNUM > 4)
    /** LED4 */
    EN_HAL_PIN_LED4,
#endif /* #if (HAL_SUPPORT_LEDNUM > 4) */

#if defined(HAL_SUPPORT_RFSPI)
    /** RF SPI CLK */
    EN_HAL_PIN_RFSPICLK,
    /** RF SPI TX */
    EN_HAL_PIN_RFSPITX,
    /** RF SPI RX */
    EN_HAL_PIN_RFSPIRX,
    /** RF SPI CS */
    EN_HAL_PIN_RFSPICS,
#endif /* #if defined(HAL_SUPPORT_RFSPI) */

#if defined(HAL_SUPPORT_RFCTRL0)
    /** RF CTRL0 */
    EN_HAL_PIN_RFCTRL0,
#endif /* #if defined(HAL_SUPPORT_RFCTRL0) */
#if defined(HAL_SUPPORT_RFCTRL1)
    /** RF CTRL1 */
    EN_HAL_PIN_RFCTRL1,
#endif /* #if defined(HAL_SUPPORT_RFCTRL1) */
#if defined(HAL_SUPPORT_RFCTRL2)
    /** RF CTRL2 */
    EN_HAL_PIN_RFCTRL2,
#endif /* #if defined(HAL_SUPPORT_RFCTRL2) */
#if defined(HAL_SUPPORT_RFCTRL3)
    /** RF CTRL3 */
    EN_HAL_PIN_RFCTRL3,
#endif /* #if defined(HAL_SUPPORT_RFCTRL3) */
#if defined(HAL_SUPPORT_RFCTRL4)
    /** RF CTRL4 */
    EN_HAL_PIN_RFCTRL4,
#endif /* #if defined(HAL_SUPPORT_RFCTRL4) */
#if defined(HAL_SUPPORT_RFCTRL5)
    /** RF CTRL5 */
    EN_HAL_PIN_RFCTRL5,
#endif /* #if defined(HAL_SUPPORT_RFCTRL5) */

#if defined(HAL_SUPPORT_SLIPUART)
    /** SLIP UART TX */
    EN_HAL_PIN_SLIPUARTTX,
    /** SLIP UART RX */
    EN_HAL_PIN_SLIPUARTRX,
#endif /* #if defined(HAL_SUPPORT_SLIPUART) */

#if defined(HAL_SUPPORT_USERUART)
    /** SLIP UART TX */
    EN_HAL_PIN_USERUARTTX,
    /** SLIP UART RX */
    EN_HAL_PIN_USERUARTRX,
#endif /* #if defined(HAL_SUPPORT_SLIPUART) */

    EN_HAL_PIN_MAX

} en_hal_pin_t;

/**
 * \brief   Describes the different types of interrupt edges.
 *
 *          Depending on the peripherals and the micro controller different
 *          kinds of interrupt have to be handled. This enumeration provides
 *          different kinds of interrupt edges.
 */
typedef enum EN_HAL_IRQEDGE_T
{
    /** Falling Edge Interrupt */
    EN_HAL_IRQEDGE_FALLING,
    /** Rising Edge Interrupt. */
    EN_HAL_IRQEDGE_RISING,
    /** Either Edge Interrupt. */
    EN_HAL_IRQEDGE_EITHER,

} en_hal_irqedge_t;


/**
 * \brief   Describes the different types of peripherals interrupts.
 *
 *          Depending on its purpose emb::6 can have several interrupts from
 *          peripherals. This enumeration shows which interrupts must be available
 *          to emb::6.
 */
typedef enum EN_HAL_PERIPHIRQ_T
{
#if defined(HAL_SUPPORT_SLIPUART)
#if defined(HAL_SUPPORT_PERIPHIRQ_SLIPUART_RX)
    /** SLIP UART RX IRQ */
    EN_HAL_PERIPHIRQ_SLIPUART_RX,
#endif /* #if defined(HAL_SUPPORT_PERIPHIRQ_SLIPUART_RX) */
#endif /* #if defined(HAL_SUPPORT_SLIPUART) */

#if defined(HAL_SUPPORT_USERUART)
#if defined(HAL_SUPPORT_PERIPHIRQ_USERUART_RX)
    /** SLIP UART RX IRQ */
    EN_HAL_PERIPHIRQ_USERUART_RX,
#endif /* #if defined(HAL_SUPPORT_PERIPHIRQ_USERUART_RX) */
#endif /* #if defined(HAL_SUPPORT_UARTUSER) */

    EN_HAL_PERIPHIRQ_MAX

} en_hal_periphirq_t;

/**
 * \brief   Describes the different types of SPI interfaces.
 *
 *          Depending on its purpose emb::6 can have several SPI based
 *          interfaces. This enumeration shows which interfaces must be
 *          available to emb::6.
 */
typedef enum EN_HAL_SPI_T
{
#if defined(HAL_SUPPORT_RFSPI)
    /** RF SPI */
    EN_HAL_SPI_RF,
#endif /* #if defined(HAL_SUPPORT_RFSPI) */

    EN_HAL_SPI_MAX

} en_hal_spi_t;

/**
 * \brief   Describes the different types of UART interfaces.
 *
 *          Depending on its purpose emb::6 can have several UART based
 *          interfaces. This enumeration shows which interfaces must be
 *          available to emb::6.
 */
typedef enum EN_HAL_UART_T
{
#if defined(HAL_SUPPORT_SLIPUART)
    /** RF SPI */
    EN_HAL_UART_SLIP,
#endif /* #if defined(HAL_SUPPORT_SLIPUART) */

#if defined(HAL_SUPPORT_USERUART)
    /** RF SPI */
    EN_HAL_UART_USER,
#endif /* #if defined(HAL_SUPPORT_USERUART) */

    EN_HAL_UART_MAX

} en_hal_uart_t;

/**
 * \brief   Describes real-time clock.
 *
 */
typedef struct EN_HAL_RTC_T
{
  /** Year */
  uint8_t ui_year;
  /** Month */
  uint8_t uc_mon;
  /** Day */
  uint8_t uc_day;
  /** Hour */
  uint8_t uc_hour;
  /** Minute */
  uint8_t uc_min;
  /** Seconds */
  uint8_t uc_sec;

} en_hal_rtc_t;

/**
 * \brief   Function prototype for an interrupt callback.
 *
 *          The HAL allows to register specific interrupts to callback
 *          functions which must have the same prototype as this one.
 *
 * \param   p_data    Data delivered by the callback function.
 */
typedef void (*pf_hal_irqCb_t)( void* p_data );


/*
 *  --- Global Functions Definition ------------------------------------------*
 */

/**
 * hal_init()
 *
 * \brief   Initialize the Hardware Abstraction Layer.
 *
 *          This function is called at the initialization to initialize the
 *          Hardware Abstraction Layer. During this initialization the
 *          according implementation shall prepare the general hardware such
 *          as the configuration of the clocks or IOs.
 *
 * \return  0 on success and < 0  in case of an error.
 */
int8_t hal_init( void );


/**
 * hal_enterCritical()
 *
 * \brief   Enter critical section e.g. to prevent interrupt during execution.
 *
 *          This function is called whenever a critical section is entered. In
 *          a critical section no interrupt routines shall be executed in order
 *          to avoid unwanted behavior of the software. Once the critical
 *          section was left pending interrupt routines can be executed.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_enterCritical( void );


/**
 * hal_exitCritical()
 *
 * \brief   Exit a critical section and allow interrupts.
 *
 *          Every time a critical section was finished this function will be
 *          called in order to allow the execution of interrupt service
 *          routines.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_exitCritical( void );


/**
 * hal_watchdogStart()
 *
 * \brief   Start a watchdog timer.
 *
 *          Watchdog timer are used to prevent a system from hanging. Therefore
 *          a watchdog has to be triggered continuously to reset it. Otherwise
 *          if it expires it will reset the system. This function is used to
 *          start the watchdog timer.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_watchdogStart( void );


/**
 * hal_watchdogSReset()
 *
 * \brief   Reset/Trigger a watchdog timer.
 *
 *          Watchdog timer are used to prevent a system from hanging. Therefore
 *          a watchdog has to be triggered continuously to reset it. Otherwise
 *          if it expires it will reset the system. This functionn is used to
 *          reset/trigger the watchdog timer.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_watchdogReset( void );


/**
 * hal_watchdogStop()
 *
 * \brief   Stop a watchdog timer.
 *
 *          Watchdog timer are used to prevent a system from hanging. Therefore
 *          a watchdog has to be triggered continuously to reset it. Otherwise
 *          if it expires it will reset the system. This function is used to
 *          stop the watchdog timer.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_watchdogStop( void );


/**
 * hal_getrand()
 *
 * \brief   Provide a random value.
 *
 *          Random values are used within emb::6 e.g. to generate random
 *          timeouts. Therefore a random seed is required that shall be
 *          provided by the hardware if this function is called. The
 *          random number can be generated e.g. by an ADC.
 *
 * \return  A random number.
 */
uint32_t hal_getrand( void );


/**
 * hal_getTick()
 *
 * \brief   Return system clock in ticks.
 *
 *          The software internal clock counts in ticks, whereas the resolution
 *          is dependent on the underlying HAL implementation. This function
 *          returns the current clock value in ticks.
 *
 * \return  Current system clock value in ticks.
 */
clock_time_t hal_getTick( void );


/**
 * hal_getSec()
 *
 * \brief   Return the current system clock in seconds.
 *
 *          The software internal clock counts in ticks, whereas the resolution
 *          is dependent on the underlying HAL implementation. This function
 *          returns the current clock value in seconds.
 *
 * \return  Current system clock value in seconds.
 */
clock_time_t hal_getSec( void );


/**
 * hal_getTRes()
 *
 * \brief   Return system time resolution.
 *
 *          The software internal clock counts in ticks, whereas the resolution
 *          is dependent on the underlying HAL implementation. This function
 *          returns the clock resolution as ticks per second.
 *
 * \return  Current system clock value in ticks.
 */
clock_time_t hal_getTRes( void );


/**
 * hal_delayUs()
 *
 * \brief   Wait for a specific time before continuing execution.
 *
 *          This function provides the possibility to delay the execution for
 *          a specific amount of time. The duration of the execution can be
 *          specified in microsecods.
 *
 * \param  delay     Time of delay in microseconds.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_delayUs( uint32_t delay );


/**
 * hal_pinInit()
 *
 * \brief   Initializes a specific Pin.
 *
 *          This function is used to initialize a specific pin. The initialization
 *          is implemented in the according HAL together with the board config
 *          to create a valid mapping of the pin.
 *
 * \param   pin   The pin to initialize.
 *
 * \return  NULL on failure (e.g. not implemented) or a pointer to the hardware
 *          dependent pin structure used for further operations.
 */
void* hal_pinInit( en_hal_pin_t pin );


/**
 * hal_pinSet()
 *
 * \brief   Set the value of a specific pin.
 *
 *          This function is used to set the value of a specific pin. The
 *          value of the pin can either be 0 or 1.
 *
 * \param   pin   The pin to set the value for.
 * \param   val   Value to set. All values other than 0 will be treated as 1.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_pinSet( void* p_pin, uint8_t val );


/**
 * hal_pinGet()
 *
 * \brief   Get the value of a specific pin.
 *
 *          This function is used to get the value of a specific pin. The
 *          value of the pin can either be 0 or 1.
 *
 * \param   pin   The pin to get the value from.
 *
   \return  The value of the pin [0,1] on success or negative value on error.
 *
 */
int8_t hal_pinGet( void* p_pin );


/**
 * hal_pinIRQRegister()
 *
 * \brief   Register and configure an external interrupt.
 *
 *          The stack uses several so called external interrupts e.g. used
 *          for the communication with the radio module or for the the UART
 *          access. This function configures such an external interrupt. By
 *          default, the external interrupt is disabled after being configured.
 *
 * \param   p_pin   Pin to which the interrupt belongs to.
 * \param   edge    Edge type to trigger the interrupt
 * \param   pf_cb   Callback used when the interrupt occurs.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_pinIRQRegister( void* p_pin, en_hal_irqedge_t edge,
    pf_hal_irqCb_t pf_cb );


/**
 * hal_pinIRQEnable()
 *
 * \brief   Enable an external interrupt.
 *
 *          The stack uses several so called external interrupts e.g. used
 *          for the communication with the radio module or for the the UART
 *          access. This function is used to enable an external interrupt.
 *
 * \param   p_pin   Pin of the external interrupt to enable.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_pinIRQEnable( void* p_pin );


/**
 * hal_pinIRQDisable()
 *
 * \brief   Disable an external interrupt.
 *
 *          The stack uses several so called external interrupts e.g. used
 *          for the communication with the radio module or for the the UART
 *          access. This function is used to disable an external interrupt.
 *
 * \param   p_pin   Pin of the external interrupt to disable.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_pinIRQDisable( void* p_pin );

/**
 * hal_pinIRQClear()
 *
 * \brief   Clear an external interrupt.
 *
 *          The stack uses several so called external interrupts e.g. used
 *          for the communication with the radio module or for the the UART
 *          access. This function is used to clear an external interrupt e.g
 *          after it has occurred or to ignore it.
 *
 * \param   p_pin   Pin of the external interrupt to clear.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_pinIRQClear( void* p_pin );

#if defined(HAL_SUPPORT_SPI)
/**
 * hal_spiInit()
 *
 * \brief   Initialize SPI interface.
 *
 *          The stack uses an SPi interface to communicate with most of the
 *          transceiver drivers. Therefore the HAl has to provide the
 *          according functions to access the SPI interface. This function
 *          initializes the SPI e.g. by configuring the according PINs and
 *          the SPI core.
 *
 * \param   spi   SPI type to initialize.
 *
 * \return  A pointer to the SPI instance on success or NULL in case of an error.
 */
void* hal_spiInit( en_hal_spi_t spi );


/**
 * hal_spiTRx()
 *
 * \brief   Simultaneously transmit and received data via SPI.
 *
 *          The stack uses an SPI interface to communicate with most of the
 *          transceiver drivers. Therefore the HAl has to provide the
 *          according functions to access the SPI interface. This function
 *          simultaneously transmitts and receives data.
 *
 * \param   p_spi   The SPI interface to read/write.
 * \param   p_tx    Transmit buffer.
 * \param   p_rx    Receive buffer.
 * \param   len     Length of the buffer.
 *
 * \return  The number of bytes transmitted/received on success or negative
 *          value on error.
 */
int32_t hal_spiTRx( void* p_spi, uint8_t* p_tx, uint8_t* p_rx, uint16_t len );


/**
 * hal_spiRx()
 *
 * \brief   Read data from SPI.
 *
 *          The stack uses an SPI interface to communicate with most of the
 *          transceiver drivers. Therefore the HAl has to provide the
 *          according functions to access the SPI interface. This function
 *          receives data from the SPI interface.
 *
 * \param   p_spi   The SPI interface to read.
 * \param   p_rx    Receive buffer.
 * \param   len     Length of the buffer.
 *
 * \return  The number of bytes received on success or negative value on error.
 */
int32_t hal_spiRx( void* p_spi, uint8_t * p_rx, uint16_t len );


/**
 * hal_spiTx()
 *
 * \brief   Transmit data via SPI.
 *
 *          The stack uses an SPI interface to communicate with most of the
 *          transceiver drivers. Therefore the HAl has to provide the
 *          according functions to access the SPI interface. This function
 *          transmits data via the SPI interface.
 *
 * \param   p_spi   The SPI interface to write.
 * \param   p_tx    Transmit buffer.
 * \param   len     Length of the buffer.
 *
 * \return  The number of bytes transmitted on success or negative value on error.
 */
int32_t hal_spiTx( void* p_spi, uint8_t* p_tx, uint16_t len );
#endif /* #if defined(HAL_SUPPORT_SPI) */

#if defined(HAL_SUPPORT_UART)
/**
 * hal_uartInit()
 *
 * \brief   Initialize UART interface.
 *
 *          The stack uses several UARTs. Therefore the HAl has to provide the
 *          according functions to access the UART interfaces. This function
 *          initializes the UART e.g. by configuring the according PINs,
 *          the core and the BAUD rate.
 *
 * \param   uart  UART type to initialize.
 *
 * \return  A pointer to the UART instance on success or NULL in case of an error.
 */
void* hal_uartInit( en_hal_uart_t uart );


/**
 * hal_uartRx()
 *
 * \brief   Read data from UART.
 *
 *          The stack uses several UARTs. Therefore the HAl has to provide the
 *          according functions to access the UART interfaces. This function
 *          receives data from the UART interface.
 *
 * \param   p_uart  The UART interface to read.
 * \param   p_rx    Receive buffer.
 * \param   len     Length of the buffer.
 *
 * \return  The number of bytes received on success or negative value on error.
 */
int32_t hal_uartRx( void* p_uart, uint8_t * p_rx, uint16_t len );


/**
 * hal_uartTx()
 *
 * \brief   Transmit data via UART.
 *
 *          The stack uses several UARTs. Therefore the HAl has to provide the
 *          according functions to access the UART interfaces. This function
 *          transmits data via the UART interface.
 *
 * \param   p_uart  The UART interface to write.
 * \param   p_tx    Transmit buffer.
 * \param   len     Length of the buffer.
 *
 * \return  The number of bytes transmitted on success or negative value on error.
 */
int32_t hal_uartTx( void* p_uart, uint8_t* p_tx, uint16_t len );
#endif /* #if defined(HAL_SUPPORT_UART) */

/**
 * hal_periphIRQRegister()
 *
 * \brief   Register an interrupt from a peripheral.
 *
 *          XXX.
 *
 * \param   irq     Type of IRQ to register.
 * \param   pf_cb   Callback function to register for the IRQ.
 * \param   p_data  Callback specific data.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_periphIRQRegister( en_hal_periphirq_t irq, pf_hal_irqCb_t pf_cb,
    void* p_data );


/**
 * hal_debugInit()
 *
 * \brief   Initialization of the debugging.
 *
 *          The stack provides debug prints using the regular printf function
 *          from the standard library. Therefore the output interface (e.g.
 *          UART) has to be initialized properly to accept the prints. This
 *          function is called at initialization of the stack to provide
 *          a properly configured debug interface.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_debugInit( void );


#if defined(HAL_SUPPORT_RTC)
/**
 * hal_rtcSetTime()
 *
 * \brief   Set current Real-Time clock data
 *
 *			XXX
 *
 * \param   p_rtc  	Pointer to RTC struct holding data to set.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_rtcSetTime( en_hal_rtc_t *p_rtc );


/**
 * hal_rtcGetTime()
 *
 * \brief   Get current Real-Time clock data.
 *
 *			XXX
 *
 * \param   p_rtc  	Pointer to RTC struct holding data to read.
 *
 * \return  0 on success or negative value on error.
 */
int8_t hal_rtcGetTime( en_hal_rtc_t *p_rtc );
#endif /* #if defined(HAL_SUPPORT_RTC) */

#endif /* __HAL_H__ */
