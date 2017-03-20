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
 *  \file       bsp.h
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
#ifndef __BSP_H__
#define __BSP_H__

/*
 * --- Includes -------------------------------------------------------------*
 */
#include "hal.h"


/*
 * --- Type Definitions -----------------------------------------------------*
 */
/**
 * \brief Defines the action to access an LED.
 *
 *        LEDS can be access on different ways. The BSP supports
 *        enabling, disabling, toggling and blinking of LEDs.
 */
typedef enum EN_BSP_LED_OP_T
{
    /** Turn on the selected LED */
    EN_BSP_LED_OP_ON,
    /** Turn off the selected LED */
    EN_BSP_LED_OP_OFF,
    /** set an LED mask */
    EN_BSP_LED_OP_SET,
    /** Toggle the selected LED */
    EN_BSP_LED_OP_TOGGLE,
    /** Let the selected LED blink */
    EN_BSP_LED_OP_BLINK

} en_bsp_led_op_t;


/**
 * \brief Defines the action to control the watchdog.
 *
 *        Several actions are available to control a watchdog such as
 *        to start or stop it. This enumeration shows the available
 *        actions to access the watchdog.
 */
typedef enum EN_BSP_WD_CTRL_T
{
    /** Reset watchdog timer */
    EN_BSP_WD_RESET,
    /** Start watchdog timer */
    EN_BSP_WD_START,
    /** Stop watchdog timer */
    EN_BSP_WD_STOP,
    /** Periodic watchdog timer */
    EN_BSP_WD_PERIODIC

} en_bsp_wd_ctrl_t;


/*
 *  --- Global Functions Definition ------------------------------------------*
 */
/**
 * bsp_init()
 *
 * \brief   Initialize the Board Support Package and the underlying HAL.
 *
 *          This function is called at the initialization to initialize the
 *          Board Support Package and the underlying Hardware Abstraction Layer.
 *          During this initialization the according implementation prepares the
 *          general hardware for the according board.
 *
 * \param  p_ns   Pointer to the network stack structure.
 *
 * \return  0 on success or negative value on error.
 */
int8_t bsp_init( s_ns_t* p_ns );

/**
 * bsp_enterCritical()
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
int8_t bsp_enterCritical( void );


/**
 * bsp_exitCritical()
 *
 * \brief   Exit a critical section and allow interrupts.
 *
 *          Every time a critical section was finished this function will be
 *          called in order to allow the execution of interrupt service
 *          routines.
 *
 * \return  0 on success or negative value on error.
 */
int8_t bsp_exitCritical( void );


/**
 * bsp_watchdog()
 *
 * \brief   Controls a watchdog timer.
 *
 *          Watchdog timer are used to prevent a system from hanging. Therefore
 *          a watchdog has to be triggered continuously to reset it. Otherwise
 *          if it expires it will reset the system. This function is used to
 *          control the watchdog timer.
 *
 * \param   ctrl  Used to control the watchdog. See enumeration description.
 *
 * \return  0 on success or negative value on error.
 */
int8_t bsp_watchdog( en_bsp_wd_ctrl_t ctrl );


/**
 * bsp_getrand()
 *
 * \brief   Provide a random value.
 *
 *          Random values are used within emb::6 e.g. to generate random
 *          timeouts. Therefore a random seed is required that shall be
 *          provided by the hardware if this function is called. The
 *          random number can be generated e.g. by an ADC.
 *
 * \param   min   Minimum random value to get.
 * \param   max   Maximum random value to get.
 *
 * \return  A random number in the range of [min,max].
 */
uint32_t bsp_getrand( uint32_t min, uint32_t max );


/**
 * bsp_getTick()
 *
 * \brief   Return system clock in ticks.
 *
 *          The software internal clock counts in ticks, whereas the resolution
 *          is dependent on the underlying HAL implementation. This function
 *          returns the current clock value in ticks.
 *
 * \return  Current system clock value in ticks.
 */
clock_time_t bsp_getTick( void );


/**
 * bsp_getSec()
 *
 * \brief   Return the current system clock in seconds.
 *
 *          The software internal clock counts in ticks, whereas the resolution
 *          is dependent on the underlying HAL implementation. This function
 *          returns the current clock value in seconds.
 *
 * \return  Current system clock value in seconds.
 */
clock_time_t bsp_getSec( void );


/**
 * bsp_getTRes()
 *
 * \brief   Return system time resolution.
 *
 *          The software internal clock counts in ticks, whereas the resolution
 *          is dependent on the underlying HAL implementation. This function
 *          returns the clock resolution as ticks per second.
 *
 * \return  Current system clock value in ticks.
 */
clock_time_t bsp_getTRes( void );


/**
 * bsp_delayUs()
 *
 * \brief   Wait for a specific time before continuing execution.
 *
 *          This function provides the possibility to delay the execution for
 *          a specific amount of time. The duration of the execution can be
 *          specified in microsecods.
 *
 * \param   delay    Time of delay in microseconds.
 *
 * \return  0 on success or negative value on error.
 */
int8_t bsp_delayUs( uint32_t delay );


/**
 * bsp_pinInit()
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
void* bsp_pinInit( en_hal_pin_t pin );


/**
 * bsp_pinSet()
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
int8_t bsp_pinSet( void* p_pin, uint8_t val );


/**
 * bsp_pinGet()
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
int8_t bsp_pinGet( void* p_pin );


/**
 * bsp_led()
 *
 * \brief   Control an LED.
 *
 *          This function is used to control a specific LED. An LED can either
 *          be enabled, disabled, toggled or it can blink. For blinking, an
 *          internal timer will be used.
 *
 * \param   led   The LED(s) to control as a bitmask.
 * \param   op    Operation to perform.
 *
 * \return  The value of the led [0,1] on success or negative value on error.
 */
int8_t bsp_led( uint8_t led, en_bsp_led_op_t op );


/**
 * bsp_pinIRQRegister()
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
int8_t bsp_pinIRQRegister( void* p_pin, en_hal_irqedge_t edge,
    pf_hal_irqCb_t pf_cb );


/**
 * bsp_pinIRQEnable()
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
int8_t bsp_pinIRQEnable( void* p_pin );


/**
 * bsp_pinIRQDisable()
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
int8_t bsp_pinIRQDisable( void* p_pin );

/**
 * bsp_pinIRQClear()
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
int8_t bsp_pinIRQClear( void* p_pin );


/**
 * hal_spiInit()
 *
 * \brief   Initialize SPI interface.
 *
 *          The stack uses an SPI interface to communicate with most of the
 *          transceiver drivers. Therefore the HAl has to provide the
 *          according functions to access the SPI interface. This function
 *          initializes the SPI e.g. by configuring the according PINs and
 *          the SPI core.
 *
 * \param   spi     SPI type to initialize.
 *
 * \return  A pointer to the SPI instance on success or NULL in case of an error.
 */
void* bsp_spiInit( en_hal_spi_t spi );


#if defined(HAL_SUPPORT_SPI)
/**
 * bsp_spiSlaveSel()
 *
 * \brief   Select an SPI slave.
 *
 *          The stack uses an SPI interface to communicate with most of the
 *          transceiver drivers. Therefore the HAl has to provide the
 *          according functions to access the SPI interface. This function
 *          (de-)asserts the SPI slave select line.
 *
 * \param   p_spi   The SPI interface to (de-)select.
 * \param   p_cs    CS pin to use.
 * \param   select  1 (or greater) to select the interface and 0 to deselect.
 * \param   inv     Use inverted logic or not.
 *
 * \return  0 on success or negative value on error.
 */
int8_t bsp_spiSlaveSel( void* p_spi, void* p_cs, uint8_t select, uint8_t inv );


/**
 * bsp_spiTxRx()
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
int32_t bsp_spiTRx( void* p_spi, uint8_t* p_tx, uint8_t* p_rx, uint16_t len );


/**
 * bsp_spiRx()
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
int32_t bsp_spiRx( void* p_spi, uint8_t * p_rx, uint16_t len );


/**
 * bsp_spiTx()
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
int32_t bsp_spiTx( void* p_spi, uint8_t* p_tx, uint16_t len );
#endif /* #if defined(HAL_SUPPORT_SPI) */


#if defined(HAL_SUPPORT_UART)
/**
 * bsp_uartInit()
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
void* bsp_uartInit( en_hal_uart_t uart );


/**
 * bsp_uartRx()
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
int32_t bsp_uartRx( void* p_uart, uint8_t * p_rx, uint16_t len );


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
int32_t bsp_uartTx( void* p_uart, uint8_t* p_tx, uint16_t len );
#endif /* #if defined(HAL_SUPPORT_UART) */


/**
 * bsp_periphIRQRegister()
 *
 * \brief   Register an interrupt from a peripheral.
 *
 *          This function can be used to register the IRQ of a peripheral
 *          to a specific callback. This is used e.g for the extif app
 *          to get characters from UART.
 *
 * \param   irq     Type of IRQ to register.
 * \param   pf_cb   Callback function to register for the IRQ.
 * \param   p_data  Callback specific data.
 *
 * \return  0 on success or negative value on error.
 */
int8_t bsp_periphIRQRegister( en_hal_periphirq_t irq, pf_hal_irqCb_t pf_cb,
    void* p_data );


/**
 * bsp_getChar()
 *
 * \brief   Returns a character from the stdin.
 *
 * \return  character on success or negative value on error.
 */
int bsp_getChar( void );


#if defined(HAL_SUPPORT_RTC)
/**
 * bsp_rtcSetTime()
 *
 * \brief   Set current Real-Time clock data
 *
 *			XXX
 *
 * \param   p_rtc  	Pointer to RTC struct holding data to set.
 *
 * \return  0 on success or negative value on error.
 */
int8_t bsp_rtcSetTime( en_hal_rtc_t *p_rtc );


/**
 * bsp_rtcGetTime()
 *
 * \brief   Get current Real-Time clock data.
 *
 *			XXX
 *
 * \param   p_rtc  	Pointer to RTC struct holding data to read.
 *
 * \return  0 on success or negative value on error.
 */
int8_t bsp_rtcGetTime( en_hal_rtc_t *p_rtc );
#endif /* #if defined(HAL_SUPPORT_RTC) */

#endif /* __BSP_H__ */

