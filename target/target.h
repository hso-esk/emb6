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
/**  \addtogroup emb6
 *      @{
 *      \addtogroup bsp Board Support Package
 *   @{
 */
/*! \file   target.h

    \author Artem Yushev, 

    \brief  \brief Hardware initialization API

    \version 0.0.1
*/

#ifndef TARGET_H_
#define TARGET_H_

#include "emb6.h"

typedef enum E_TARGET_EXTINT {
    E_TARGET_RADIO_INT,
    E_TARGET_USART_INT
}en_targetExtInt_t;


typedef enum E_TARGET_EXTPIN {
    E_TARGET_RADIO_RST,
    E_TARGET_RADIO_SLPTR,
}en_targetExtPin_t;
/*----------------------------------------------------------------------------*/
/** \brief      This function has to enter a critical section to prevent
 *                 interrupting part of a code.
 *
 *  \return        none
 */
/*---------------------------------------------------------------------------*/
void hal_enterCritical(void);

/*----------------------------------------------------------------------------*/
/** \brief      This function has to exit a critical section to enable
 *                 interrupting part of a code.
 *
 *  \return        none
 */
/*---------------------------------------------------------------------------*/
void hal_exitCritical(void);

/*----------------------------------------------------------------------------*/
/** \brief      This function should initialize all of the MCU periphery
 *
 *  \return        status
 */
/*---------------------------------------------------------------------------*/
int8_t     hal_init (void);

/*----------------------------------------------------------------------------*/
/** \brief      This function returns a random number from adc. Preferably using
 *                 is for initialize random number generator.
 *
 *  \return        none
 */
/*---------------------------------------------------------------------------*/
uint8_t    hal_getrand(void);

/*----------------------------------------------------------------------------*/
/** \brief      This function should turns on particular bound by macro LED
 *  \param         ui_led    Descriptor of a led
 *
 *  \return        none
 */
/*---------------------------------------------------------------------------*/
void    hal_ledOn(uint16_t ui_led);


/*----------------------------------------------------------------------------*/
/** \brief      This function should turns off particular bound by macro LED
 *  \param         ui_led    Descriptor of a led
 *
 *  \return        none
 */
/*---------------------------------------------------------------------------*/
void    hal_ledOff(uint16_t ui_led);
/*----------------------------------------------------------------------------*/
/** \brief      This function configures external interrupt
 *  \param  e_extInt     source of an interrupt
 *  \param    pfn_intCallback    Callback function pointer
 *
 *  \return        none
 */
/*---------------------------------------------------------------------------*/
uint8_t    hal_extIntInit(en_targetExtInt_t e_extInt, pfn_intCallb_t pfn_intCallback);

/*============================================================================*/
/** \brief  This function makes a delay on the target MCU
 *
 *  \param  i_delay Delay in micro seconds
 *
 *  \retval    none
 */
/*============================================================================*/
void    hal_delay_us(uint32_t i_delay);

/*============================================================================*/
/** \brief  This function initialise given gpio pin
 *
 *  \param  c_pin        pin number
 *  \param    c_dir        dir
 *  \param    c_initState    state
 *
 *  \retval err code
 */
/*============================================================================*/
uint8_t  hal_gpioPinInit(uint8_t c_pin, uint8_t c_dir, uint8_t c_initState);

/*============================================================================*/
/** \brief  This function initialise given control pin
 *
 *  \param  e_pinType    Type of a pin
 *
 *  \retval    pointer to a pin
 */
/*============================================================================*/
void *     hal_ctrlPinInit(en_targetExtPin_t e_pinType);

/*============================================================================*/
/** \brief  This function sets particular pin on the target
 *
 *  \param  p_pin        Pointer to a pin number
 *
 *  \retval    none
 */
/*============================================================================*/
void    hal_pinSet(void * p_pin);

/*============================================================================*/
/** \brief  This function clears particular pin on the target
 *
 *  \param  p_pin        Pointer to a pin number
 *
 *  \retval    none
 */
/*============================================================================*/
void    hal_pinClr(void * p_pin);

/*============================================================================*/
/** \brief  This function returns pin state on the target
 *
 *  \param  p_pin        Pointer to a pin number
 *
 *  \retval    none
 */
/*============================================================================*/
uint8_t    hal_pinGet(void * p_pin);

/*----------------------------------------------------------------------------*/
/** \brief  This function configures SPI interface
 *
 *  \return        pointer to an allocated memory
 */
/*---------------------------------------------------------------------------*/
void *     hal_spiInit(void);

/*----------------------------------------------------------------------------*/
/** \brief  This function selects slave with which we will work
 *  \param         p_spi    Pointer to spi description entity
 *  \param        action    true or false
 *
 *  \return        0 if failed, 1 id ok
 */
/*---------------------------------------------------------------------------*/
uint8_t hal_spiSlaveSel(void * p_spi, bool action);

uint8_t hal_spiTransceive( uint8_t * txData, uint8_t * p_reg);

/*----------------------------------------------------------------------------*/
/** \brief  This function reads data via given SPI interface.
 *     TODO    Implement selecting of an interface instance.
 *  \param  p_reg Pointer where to store received data
 *  \param    i_length Length of a data to be received
 *
 *  \returns The actual value of the read register.
 */
/*----------------------------------------------------------------------------*/
uint8_t hal_spiRead(uint8_t * p_reg, uint16_t i_length);

/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value via given SPI interface
 *          registers.
 *
 *
 *  \param  value         Pointer to a value.
 *  \param  i_length     Length of a data to be received
 */
/*----------------------------------------------------------------------------*/
void    hal_spiWrite(uint8_t * value, uint16_t i_length);



/*============================================================================*/
/** \brief  This function will reset watchdog timer
 *
 *  \retval    none
 */
/*============================================================================*/
void     hal_watchdogReset(void);

/*============================================================================*/
/** \brief  This function will enable watchdog timer
 *
 *  \retval    none
 */
/*============================================================================*/
void     hal_watchdogStart(void);

/*============================================================================*/
/** \brief  This function will stop watchdog timer
 *
 *  \retval    none
 */
/*============================================================================*/
void     hal_watchdogStop(void);

/*============================================================================*/
/** \brief  This function will return system ticks
 *
 *  \retval    ticks
 */
/*============================================================================*/
clock_time_t     hal_getTick(void);

/*============================================================================*/
/** \brief  This function will return seconds
 *
 *  \retval    seconds
 */
/*============================================================================*/
clock_time_t     hal_getSec(void);

/*============================================================================*/
/** \brief  This function will time resolution
 *             How many ticks in one second
 *
 *  \retval    time resolution
 */
/*============================================================================*/
clock_time_t hal_getTRes(void);
#endif /* TARGET_H_ */
/** @} */
/** @} */
