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
#define LEDS_SUPPORTED                3

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
typedef enum E_BSP_LED_COLOR
{
    //! Define Red color
    E_BSP_LED_RED = 0x00,
    //! Define YELLOW color
    E_BSP_LED_YELLOW = 0x01,
    //! Define GREEN color
    E_BSP_LED_GREEN = 0x02
} en_bspLedColor_t;




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
uint16_t bsp_led(en_bspLedColor_t ui_led, en_bspLedAction_t en_ledAction);

/*============================================================================*/
/** \brief  This function enters critical section on the target
 *
 */
/*============================================================================*/
#define        bsp_enterCritical()             hal_enterCritical()

/*============================================================================*/
/** \brief  This function exits critical section on the target
 *
 */
/*============================================================================*/
#define     bsp_exitCritical()              hal_exitCritical()

/*============================================================================*/
/** \brief  This function initialize external interrupt on the target
 *
 *  \param  e_intSource     source of an interrupt
 *  \param    fn_p        Callback function pointer
 *
 */
/*============================================================================*/
#define        bsp_extIntInit(e_intSource, fn_p) hal_extIntInit(e_intSource,fn_p)


/*============================================================================*/
/** \brief  This function makes a delay on the target
 *
 *  \param  i_delay Delay in useconds
 *
 */
/*============================================================================*/
#define        bsp_delay_us(i_delay)           hal_delay_us(i_delay)

/*============================================================================*/
/** \brief  This function initialize a specific pin
 *
 *  \param  e_pinType Type of a a pin
 *    \return err code
 */
/*============================================================================*/
#define     bsp_pinInit(e_pinType)          hal_ctrlPinInit(e_pinType)

uint8_t bsp_pin(en_bspPinAction_t e_pinAct, void * p_pin);


#define     bsp_spiInit()                   hal_spiInit()

uint8_t bsp_spiRegRead(void * p_spi, uint8_t c_addr);

void bsp_spiFrameRead(void * p_spi,uint8_t c_addr, uint8_t * pc_data, uint16_t * pi_len);

uint8_t bsp_spiBitRead(void * p_spi, uint8_t c_addr, uint8_t c_mask, uint8_t c_pos );

void bsp_spiRegWrite(void * p_spi, uint8_t c_addr, uint8_t  c_data);

void bsp_spiFrameWrite(void * p_spi, uint8_t c_addr, uint8_t * pc_data, uint8_t c_len);


/*============================================================================*/
/** \brief  This function will manipulate a watchdog timer
 *    \param wdtAct    type of an action with watchdog timer: reset, start or stop.
 */
/*============================================================================*/
void bsp_wdt(en_bspWdtAction_t wdtAct);

/*============================================================================*/
/** \brief  This function will return system ticks
 *
 */
/*============================================================================*/
#define     bsp_getTick()              hal_getTick()

/*============================================================================*/
/** \brief  This function will return seconds
 *
 */
/*============================================================================*/
#define     bsp_getSec()               hal_getSec()

/*============================================================================*/
/** \brief  This function will return parameter specifed by type
 *
 */
/*============================================================================*/
uint32_t         bsp_get(en_bspParams_t en_param);
#endif /* __BSP_H__ */
/** @} */
/** @} */
