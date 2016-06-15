//*****************************************************************************
//! @file       io_pin_int.c
//! @brief      I/O pin interrupt handler for CC2538. This is an extension to
//!             the driverlib GPIO module, allowing GPIO pins on the same GPIO
//!             port to have different interrupt handlers.
//!
//!             \b Use:
//!
//!             1) Register a custom interrupt handler using ioPinIntRegister.
//!
//!             2) Unregister a custom interrupt handler using ioPinIntRegister
//!
//!             When registering a custom interrupt handler to GPIO a pin, a
//!             generic interrupt handler is assigned to the GPIO port. The
//!             generic interrupt handler calls the custom interrupt handler.
//!
//!             If an interrupt is triggered on a pin without a custom interrupt
//!             handler, the generic interrupt handler simply clears the pin's
//!             interrupt flag.
//!
//!             Example using driverlib and the extension: Assuming interrupts
//!             are disabled:
//!             \code
//!             // Register interrupt handler to GPIO port C, pin 0
//!             ioPinIntRegister(GPIO_C_BASE, GPIO_PIN_0, &myIsr);
//!
//!             // Set interrupt type to rising edge
//!             GPIOIntTypeSet(GPIO_C_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);
//!
//!             // Enable pin interrupt (driverlib function)
//!             GPIOPinIntEnable(GPIO_C_BASE, GPIO_PIN_0);
//!
//!             // Enable master interrupt (driverlib function)
//!             IntMasterEnable();
//!             \endcode
//!
//! Revised     $Date: 2013-04-11 19:41:57 +0200 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9707 $
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/
#ifndef IO_PIN_INT_EXCLUDE


/**************************************************************************//**
* @addtogroup io_pin_int_api
* @{
******************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/ioc.h"
#include "driverlib/gpio.h"
#include "io_pin_int.h"

#include "bsp/srf06eb_cc26xx/drivers/source/bsp.h"


//*****************************************************************************
//
// Number of GPIO pins
//
//*****************************************************************************
#define NUM_GPIO_PINS 32

/******************************************************************************
* LOCAL VARIABLES AND PROTOTYPES
*/
//
// Lookup variables (a set bit in these variables means the corresponding GPIO
// pin has a custom ISR)
static uint_fast32_t ui32PinHasIsr;

//
// Function pointer arrays
//
static void (*pfnGpioPortIsr[NUM_IO_MAX])(void *pEvent);

//
// Generic GPIO port ISRs
//
static void ioPortIsr(void);


#ifdef IO_PIN_INT_DEBUG
static volatile uint32_t ioPinInt_intCount;
static volatile uint32_t ioPinInt_intAndIsrCount;
static volatile uint32_t ioPinInt_falseintCount;
#endif


/**************************************************************************//**
* @brief    Register an interrupt handler to the GPIO pin (or pins) specified
*           by bitmask \e ui32Pins.
* 			This function registers a general ISR to the GPIO port and then
*           assigns the ISR specified by \e pfnIntHandler to the given pins.
*
* @param    ui32Pins        is the bit-packed representation of the pin (or
*                           pins).
* @param    pfnIntHandler   is a pointer to the interrupt handler function.
*
* @return   None
******************************************************************************/
void
ioPinIntRegister(uint32_t ui32Pins, void (*pfnIntHandler)(void *pEvent))
{
    volatile uint_fast32_t ui32Idx;

    //
    // Disable global interrupts
    //
    bool bIntDisabled = IntMasterDisable();

    //
    // If function pointer is not null, register that pin has a custom
    // handler to the fast access 'ui8PortXPinHasIsr' variables.
    //
    if(pfnIntHandler)
    {
        ui32PinHasIsr |= ui32Pins;

        //
        // Register generic function as interrupt handler
        //
        IOCIntRegister(&ioPortIsr);
    }

    //
    // Clear interrupts on specified pins
    //
    GPIO_clearEventMultiDio(ui32Pins);

    //
    // Iterate over port pins and store handler into the correct lookup table.
    //
    for(ui32Idx = 0; ui32Idx < NUM_GPIO_PINS; ui32Idx++)
    {
        if(ui32Pins & (1 << ui32Idx))
        {
            pfnGpioPortIsr[ui32Idx] = pfnIntHandler;
        }
    }

    //
    // If interrupt was enabled, re-enable
    //
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }
}


/**************************************************************************//**
* @brief    Unregister the interrupt handler to GPIO pin (or pins) specified by
*           bitmask \e ui32Pins.
*
* @param    ui32Pins    is the bit-packed representation of the pin (or pins).
*
* @return   None
******************************************************************************/
void
ioPinIntUnregister(uint32_t ui32Pins)
{
    //
    // Disable global interrupts
    //
    bool bIntDisabled = IntMasterDisable();

#ifdef DEBUG
    //
    // Register null function to pins
    // Doing this is not necessary, but is less confusing during debug. If not
    // cleared, the pins' interrupt vector tables may be non-null even when
    // no custom ISR is actually registered. Note that it is the
    // ui32PinHasIsr variable that is used to decide whether a custom
    // interrupt is registered or not.
    //
    ioPinIntRegister(ui32Pins, 0);
#endif

    //
    // Clear "pin has ISR" variables
    //
    ui32PinHasIsr &= ~ui32Pins;

    //
    // If interrupt was enabled, re-enable
    //
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }
}


/******************************************************************************
* LOCAL FUNCTIONS
*/

/**************************************************************************//**
* @internal
*
* @brief    General purpose IO interrupt function for GPIO module.
*           If an interrupt function handler is defined for the pins with
*           its interrupt flag set, this function is called. The interrupt
*           flag for this pin is then cleared. If no custom ISR is registered,
*           function simply clears the interrupt pin flag(s) and returns.
*
* @return   None
******************************************************************************/
static void
ioPortIsr(void)
{
    register uint_fast32_t ui32IntBm;
    volatile uint_fast32_t ui32IsrBm;

    //
    // Read interrupt mask
    //
    ui32IntBm = (HWREG(GPIO_BASE + GPIO_O_EVFLAGS31_0) & GPIO_DIO_ALL_MASK);

    //
    // Create mask of pins with interrupt _and_ custom handler
    //
    ui32IsrBm = (ui32IntBm & ui32PinHasIsr);

    //
    // Clear event flags for pins w/o custom handler.
    //
    HWREG(GPIO_BASE + GPIO_O_EVFLAGS31_0) = (ui32IntBm & ~(ui32IsrBm));

    //
    // If no pins have custom handler, return early.
    //
    if(ui32IsrBm == 0)
    {
        __asm(" NOP"); // Must be here in debug, otherwise unable to set breakpoint.
        return;
    }

    //
    // Run custom ISRs
    //
    register uint_fast32_t ui32Idx, ui32PinBm;
    for(ui32Idx = 0; ui32Idx < NUM_GPIO_PINS; ui32Idx++)
    {
        ui32PinBm = (1 << ui32Idx);
        if(ui32PinBm & ui32IsrBm)
        {
            //
            // Run custom ISR and clear flag.
            //
            (*pfnGpioPortIsr[ui32Idx])((void *)ui32PinBm);
            HWREG(GPIO_BASE + GPIO_O_EVFLAGS31_0) = ui32PinBm;
        }
    }
}


/**************************************************************************//**
* Close the Doxygen group.
* @}
******************************************************************************/
#endif // #ifndef IO_PIN_INT_EXCLUDE
