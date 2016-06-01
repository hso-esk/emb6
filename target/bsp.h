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
/**  \addtogroup emb6
 *      @{
 *      \addtogroup bsp Board Support Package
 *   @{
 */
/*! \file   bsp.h

    \author Artem Yushev, 
            Phuong Nguyen

    \brief  Definition of the API for board support packaged (BSP) used by
            the embetter_main.c demo implementation.

    \details The demo implementation of embetter_main.c uses the following
            services from the hardware:
              - initialization: bsp_init() is called after the emBetter stack
                is initialized and leaves room for the initialization of other
                parts of the BSP such as starting the timer.
              - start: bsp_start() is called when the initialization of all
                protocols is finished. Here the hardware dependent interfaces,
                such as an Ethernet interface can be initialized.
              - console: for handling menu commands, characters read by a
                console must be provided.
              - LED: some boards provide LED support and this is to be added
                in the corresponding function.


    \version 0.0.1
*/

#ifndef __BSP_H__
#define __BSP_H__

/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
==============================================================================*/
#include "target.h"

/*==============================================================================
                                     MACROS
==============================================================================*/
#define LEDS_SUPPORTED                5

#define BSP_PIN_DIROUTPUT           TRUE
#define BSP_PIN_DIRINPUT            FALSE

#define BSP_PIN_UP                  TRUE
#define BSP_PIN_DOWN                FALSE


/*==============================================================================
                                     ENUMS
==============================================================================*/
//! Define the action the function bsp_led() must provide
typedef enum E_BSP_LED_ACTION
{
    //! Turn on the selected LED
    E_BSP_LED_ON,
    //! Turn off the selected LED
    E_BSP_LED_OFF,
    //! Toggle the selected LED
    E_BSP_LED_TOGGLE
} en_bspLedAction_t;

//! Define the leds color
typedef enum E_BSP_LED_IDX
{
    //! Define LED0
    E_BSP_LED_0 = 0x00,
    //! Define LED1
    E_BSP_LED_1 = 0x01,
    //! Define LED2
    E_BSP_LED_2 = 0x02,
    //! Define LED3
    E_BSP_LED_3 = 0x03,
    //! Define LED4
    E_BSP_LED_4 = 0x04,
} en_bspLedIdx_t;

//! Define the action the function bsp_wdt() must provide
typedef enum E_BSP_WDT_ACTION
{
    //! Reset watchdog timer
    E_BSP_WDT_RESET,
    //! Start watchdog timer
    E_BSP_WDT_START,
    //! Stop watchdog timer
    E_BSP_WDT_STOP,
    //! Periodic watchdog timer
    E_BSP_WDT_PERIODIC
} en_bspWdtAction_t;

//! Define the action the function bsp_wdt() must provide
typedef enum E_BSP_PIN_ACTION
{
    //! Configure pin
    E_BSP_PIN_CONF,
    //! Set pin
    E_BSP_PIN_SET,
    //! Reset pin
    E_BSP_PIN_CLR,
    //! Get pin
    E_BSP_PIN_GET
} en_bspPinAction_t;

//! Define the action the function bsp_wdt() must provide
typedef enum E_BSP_GET_TYPE
{
    //! Ticks
    E_BSP_GET_TICK,
    //! Seconds
    E_BSP_GET_SEC,
    //! Ticks in one seconds (time resolution)
    E_BSP_GET_TRES
}en_bspParams_t;


/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/


/*==============================================================================
                               MACROS
==============================================================================*/
#define LED_RX_ON()                 bsp_led(E_BSP_LED_0, E_BSP_LED_ON)
#define LED_RX_OFF()                bsp_led(E_BSP_LED_0, E_BSP_LED_OFF)
#define LED_TX_ON()                 bsp_led(E_BSP_LED_1, E_BSP_LED_ON)
#define LED_TX_OFF()                bsp_led(E_BSP_LED_1, E_BSP_LED_OFF)
#define LED_SCAN_ON()               bsp_led(E_BSP_LED_2, E_BSP_LED_ON)
#define LED_SCAN_OFF()              bsp_led(E_BSP_LED_2, E_BSP_LED_OFF)
#define LED_MEAS_ON()               bsp_led(E_BSP_LED_4, E_BSP_LED_ON)
#define LED_MEAS_OFF()              bsp_led(E_BSP_LED_4, E_BSP_LED_OFF)
#define LED_ERROR()                 bsp_led(E_BSP_LED_4, E_BSP_LED_ON)
#define LED_ERR_ON()                bsp_led(E_BSP_LED_4, E_BSP_LED_ON)
#define LED_ERR_OFF()               bsp_led(E_BSP_LED_4, E_BSP_LED_OFF)


/*==============================================================================
                         FUNCTION PROTOTYPES OF THE API
==============================================================================*/

/*============================================================================*/
/*!
    \brief  Initialization of the board support package

            This function is called after the initialization of the emBetter
            TCP/IP stack. At this point, internal data structures of all modules
            of the emBetter stack are reset to their initial states.
            A BSP can initialize at this point the timer and the UART for the
            console.

     \param * to a network stack structure
*/
/*============================================================================*/
uint8_t bsp_init(s_ns_t *);

/*============================================================================*/
/*!
    \brief  Startup code for the board support package

            This function is called by embetter_main.c after all protocols are
            initialized. The BSP may initialize network interfaces such as an
            Ethernet interface here.
*/
/*============================================================================*/
void bsp_start(void);

/*============================================================================*/
/*!
    \brief  Services and protocols of the board support package

            This function is called periodically out of the main() application
            loop. If a BSP requires to perform actions on a periodic base, this
            function is to be used for such actions.

*/
/*============================================================================*/
void bsp_entry(void);

/*============================================================================*/
/*!
    \brief  Returns a character from the stdin

            Polls characters from the stdin, this function must be non-blocking.

    \return >=0  the value of the character that was received
    \return -1   no character is present or an error occurred

*/
/*============================================================================*/
int bsp_getChar(void);

/*============================================================================*/
/*!
    \brief  Do some LED toggling

    \param  ui_led          The index of the LED that should be manipulated
    \param  en_ledAction    The action that should be executed

    \return 0       LED was off and is set now to state en_ledAction
    \return 1       LED was on and is set now to state en_ledAction
*/
/*============================================================================*/
uint16_t bsp_led(en_bspLedIdx_t ui_led, en_bspLedAction_t en_ledAction);


/*============================================================================*/
/** \brief  This function enters critical section on the target
 *
 */
/*============================================================================*/
void bsp_enterCritical(void);

/*============================================================================*/
/** \brief  This function exits critical section on the target
 *
 */
/*============================================================================*/
void bsp_exitCritical(void);

/*============================================================================*/
/** \brief  This function will return system ticks
 *
 *  \retval    ticks
 */
/*============================================================================*/
clock_time_t bsp_getTick(void);

/*============================================================================*/
/** \brief  This function will return seconds
 *
 *  \retval    seconds
 */
/*============================================================================*/
clock_time_t bsp_getSec(void);

/*============================================================================*/
/** \brief  This function makes a delay on the target
 *
 *  \param  i_delay Delay in microseconds
 */
/*============================================================================*/
void bsp_delay_us(uint32_t i_delay);

/*============================================================================*/
/** \brief  This function initialize given control pin
 *
 *  \param  e_pinType    Type of a pin
 *
 *  \retval    pointer to a pin
 */
/*============================================================================*/
void * bsp_pinInit(en_targetExtPin_t e_pinType);

/*============================================================================*/
/** \brief  This function control a GPIO pin
 *
 *  \param  e_pinAct  action to be taken on the pin
 *  \param  p_pin     point to the pin description
 */
/*============================================================================*/
uint8_t bsp_pin(en_bspPinAction_t e_pinAct, void * p_pin);

/*============================================================================*/
/** \brief      This function configures external interrupt. By default, the
 *              external interrupt is disabled after being configured.
 *
 *  \param          e_extInt            source of an interrupt
 *  \param          e_edge              edge upon which the interrupt occurs
 *  \param          pfn_intCallback     callback function pointer
 *
 *  \return         none
 */
/*============================================================================*/
void bsp_extIntRegister(en_targetExtInt_t e_extInt, en_targetIntEdge_t e_edge,
    pfn_intCallb_t pfn_intCallback);

/*============================================================================*/
/** \brief      This function clear state of external interrupt
 *
 *  \param          e_extInt            source of an interrupt
 *
 *  \return         none
 */
/*============================================================================*/
void bsp_extIntClear(en_targetExtInt_t e_extInt);

/*============================================================================*/
/** \brief      This function enables external interrupt
 *
 *  \param          e_extInt            source of an interrupt
 *
 *  \return         none
 */
/*============================================================================*/
void bsp_extIntEnable(en_targetExtInt_t e_extInt);

/*============================================================================*/
/** \brief      This function disable external interrupt
 *
 *  \param          e_extInt            source of an interrupt
 *
 *  \return         none
 */
/*============================================================================*/
void bsp_extIntDisable(en_targetExtInt_t e_extInt);

/*----------------------------------------------------------------------------*/
/** \brief  This function configures SPI interface
 *
 *  \return        pointer to an allocated memory
 */
/*---------------------------------------------------------------------------*/
void *bsp_spiInit(void);

/*----------------------------------------------------------------------------*/
/** \brief  This function selects slave with which we will work
 *  \param         p_spi    Pointer to spi description entity
 *  \param        action    true or false
 *
 *  \return        0 if failed, 1 id ok
 */
/*----------------------------------------------------------------------------*/
uint8_t bsp_spiSlaveSel(void * p_spi, bool action);

/*----------------------------------------------------------------------------*/
/** \brief  This function simultaneously transmit and receive data from SPI
 *
 *  \param         p_tx    Pointer to buffer holding data to send
 *  \param         p_rx    Pointer to buffer holding data to send
 *  \param         len     Size of data to send
 *
 *  \return        none
 */
/*----------------------------------------------------------------------------*/
void bsp_spiTxRx(uint8_t *p_tx, uint8_t *p_rx, uint16_t len);

/*----------------------------------------------------------------------------*/
/** \brief  This function reads data via given SPI interface.
 *     TODO    Implement selecting of an interface instance.
 *  \param  p_reg Pointer where to store received data
 *  \param    i_length Length of a data to be received
 *
 *  \returns The actual value of the read register.
 */
/*----------------------------------------------------------------------------*/
uint8_t bsp_spiRead(uint8_t * p_reg, uint16_t i_length);

/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value via given SPI interface
 *          registers.
 *
 *
 *  \param  value         Pointer to a value.
 *  \param  i_length     Length of a data to be received
 */
/*----------------------------------------------------------------------------*/
void bsp_spiWrite(uint8_t * value, uint16_t i_length);


/*============================================================================*/
/** \brief  This function will manipulate a watchdog timer
 *    \param wdtAct    type of an action with watchdog timer: reset, start or stop.
 */
/*============================================================================*/
void bsp_wdt(en_bspWdtAction_t wdtAct);

/*============================================================================*/
/** \brief  This function will return parameter specified by type
 *
 */
/*============================================================================*/
uint32_t bsp_get(en_bspParams_t en_param);



/**
 * @brief   This function returns a upper-bounded random number
 * @param   max     Maximum possible value of the returned random number
 * @return
 */
uint32_t bsp_getrand(uint32_t max);

#endif /* __BSP_H__ */
/** @} */
/** @} */
