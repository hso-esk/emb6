//*****************************************************************************
//! @file       bsp_key.c
//! @brief      Key board support package for CC13xx/CC26xx on SmartRF06EB/BB.
//!             Key debounce is by default implemented using a timer.
//!             The user may register custom ISRs using the bspKeyIntRegister()
//!             function.
//!
//!             If a custom ISR is registered, it will be called prior to
//!             starting the debounce timer.
//!
//! Revised     $Date: 2013-04-11 19:41:57 +0200 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9707 $
//
//  Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
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
#ifndef BSP_KEY_EXCLUDE


/**************************************************************************//**
* @addtogroup   bsp_key_api
* @{
******************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_prcm.h"

#include "bsp_key.h"
#include "bsp_led.h"
#include "io_pin_int.h"

#include "driverlib/interrupt.h"
#include "driverlib/sys_ctrl.h"
#include "driverlib/ioc.h"
#include "driverlib/watchdog.h"
#include "driverlib/prcm.h"
#include "driverlib/debug.h"


/******************************************************************************
* DEFINES
*/
// Number of keys on board.
#define BSP_KEY_COUNT           5
// Active wait debounce macro
#define BSP_KEY_DEBOUNCE(expr)  { do {uint16_t i; for(i = 0; i < 500; i++) {  \
                                  if (!(expr)) i = 0; } } while(0);}
#define BSP_KEY_POLL_CFG        (IOC_CURRENT_2MA  | IOC_STRENGTH_AUTO |       \
                                 IOC_IOPULL_UP    | IOC_SLEW_DISABLE  |       \
                                 IOC_HYST_DISABLE | IOC_NO_EDGE       |       \
                                 IOC_INT_DISABLE  | IOC_IOMODE_NORMAL |       \
                                 IOC_NO_WAKE_UP   | IOC_INPUT_ENABLE)

#define BSP_KEY_ISR_CFG         (IOC_CURRENT_2MA  | IOC_STRENGTH_AUTO |       \
                                 IOC_IOPULL_UP    | IOC_SLEW_DISABLE  |       \
                                 IOC_HYST_DISABLE | IOC_RISING_EDGE   |       \
                                 IOC_INT_ENABLE   | IOC_IOMODE_NORMAL |       \
                                 IOC_NO_WAKE_UP   | IOC_INPUT_ENABLE)


/******************************************************************************
* LOCAL VARIABLES AND FUNCTION PROTOTYPES
*/
static volatile uint_fast32_t ui32BspKeysPressed;
static volatile uint_fast32_t ui32BspKeyIntDisabledMask;
#if (BSP_KEY_ALL != 0)
static void (*bspKeysIsrTable[BSP_KEY_COUNT])(void);
#endif
static uint_fast32_t ui32BspKeyMode;

static void bspKeyPushedISR(void *pEvent);
static void bspKeyTimerISR(void);
static void bspKeyTimerEnable(void);
static void bspKeyTimerDisable(void);


/******************************************************************************
* FUNCTIONS
*/
/**************************************************************************//**
* @brief    This function initializes key GPIO as input pullup and disables
*           interrupts. If \e ui32Mode is \b BSP_KEY_MODE_POLL, key presses are
*           handled using polling and active state debounce. Functions starting
*           with \b bspKeyInt then do nothing.
*
*           If \e ui32Mode is \b BSP_KEY_MODE_ISR, key presses are handled by
*           interrupts, and debounce is implemented using a timer.
*
* @param    ui32Mode is the operation mode; must be one of the following:
*                   \li \b BSP_KEY_MODE_POLL for polling-based handling
*                   \li \b BSP_KEY_MODE_ISR for interrupt-based handling
* @return   None
******************************************************************************/
void
bspKeyInit(uint32_t ui32Mode)
{
#if (BSP_KEY_ALL != 0)
    //
    // Store mode
    //
    ui32BspKeyMode = ui32Mode;

    //
    // Enable clock for GPIO when CM3 is in run mode
    //
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);

    //
    // Disable interrupts and clear flags
    //
#if (BSP_IOID_KEY_UP != IOID_UNUSED)
    IOCIntDisable(BSP_IOID_KEY_UP);
#endif
#if (BSP_IOID_KEY_DOWN != IOID_UNUSED)
    IOCIntDisable(BSP_IOID_KEY_DOWN);
#endif
#if (BSP_IOID_KEY_LEFT != IOID_UNUSED)
    IOCIntDisable(BSP_IOID_KEY_LEFT);
#endif
#if (BSP_IOID_KEY_RIGHT != IOID_UNUSED)
    IOCIntDisable(BSP_IOID_KEY_RIGHT);
#endif
#if (BSP_IOID_KEY_SELECT != IOID_UNUSED)
    IOCIntDisable(BSP_IOID_KEY_SELECT);
#endif
    GPIO_clearEventMultiDio(BSP_KEY_ALL);

    //
    // Initialize keys (input pullup)
    //
#ifdef FPGA_INCLUDED
    //
    // Write all pins as output high before setting them as input pullup
    //
    bspFpgaForceDir(BSP_IOID_KEY_UP, 1, IOC_IOPULL_UP);
    bspFpgaForceDir(BSP_IOID_KEY_DOWN, 1, IOC_IOPULL_UP);
    bspFpgaForceDir(BSP_IOID_KEY_LEFT, 1, IOC_IOPULL_UP);
    bspFpgaForceDir(BSP_IOID_KEY_RIGHT, 1, IOC_IOPULL_UP);
    bspFpgaForceDir(BSP_IOID_KEY_SELECT, 1, IOC_IOPULL_UP);

    //
    // Static vars not initialized to 0 on FPGA
    //
    for(int i = 0; i < BSP_KEY_COUNT; i++)
    {
        bspKeysIsrTable[i] = 0;
    }
    ui32BspKeysPressed = 0;
    ui32BspKeyIntDisabledMask = 0;
#endif // FPGA_INCLUDED


    uint32_t ui32PinConfig = BSP_KEY_POLL_CFG;
    if(ui32BspKeyMode == BSP_KEY_MODE_ISR)
    {
        ui32PinConfig = BSP_KEY_ISR_CFG;
    }

#if (BSP_IOID_KEY_UP != IOID_UNUSED)
    IOCPortConfigureSet(BSP_IOID_KEY_UP, IOC_PORT_GPIO, ui32PinConfig);
#endif
#if (BSP_IOID_KEY_DOWN != IOID_UNUSED)
    IOCPortConfigureSet(BSP_IOID_KEY_DOWN, IOC_PORT_GPIO, ui32PinConfig);
#endif
#if (BSP_IOID_KEY_LEFT != IOID_UNUSED)
    IOCPortConfigureSet(BSP_IOID_KEY_LEFT, IOC_PORT_GPIO, ui32PinConfig);
#endif
#if (BSP_IOID_KEY_RIGHT != IOID_UNUSED)
    IOCPortConfigureSet(BSP_IOID_KEY_RIGHT, IOC_PORT_GPIO, ui32PinConfig);
#endif
#if (BSP_IOID_KEY_SELECT != IOID_UNUSED)
    IOCPortConfigureSet(BSP_IOID_KEY_SELECT, IOC_PORT_GPIO, ui32PinConfig);
#endif

    if(ui32BspKeyMode == BSP_KEY_MODE_ISR)
    {
        //
        // Connect bspKeyPushedISR() to key pins
        //
        ioPinIntRegister(BSP_KEY_ALL, &bspKeyPushedISR);

        //
        // Make sure timer is disabled
        //
        bspKeyTimerDisable();

        //
        // Configure timer
        //
        WatchdogUnlock();
        WatchdogIntTypeSet(WATCHDOG_INT_TYPE_INT);
        WatchdogReloadSet((GET_MCU_CLOCK / 12));
        WatchdogIntRegister(&bspKeyTimerISR);
    }

    //
    // Set GPIO module to input mode
    //
    GPIO_setOutputEnableDio(BSP_KEY_ALL, GPIO_OUTPUT_DISABLE);
#endif // if (BSP_KEY_ALL != 0)
}


/**************************************************************************//**
* @brief    This function returns a bitmask of keys pushed.
*
* @note     If keys are handled using polling (\b BSP_KEY_MODE_POLL), the
*           returned bitmask will never contain a combination of multiple key
*           bitmasks, for example, (\b BSP_KEY_LEFT |\b BSP_KEY_UP).
*           Furthermore, in this case argument \e ui8ReadMask is ignored.
*
* @param    ui32ReadMask    is a bitmask of keys to read. Read keys are cleared
*                           and new key presses can be registered. Use
*                           \b BSP_KEY_ALL to read status of all keys.
*
* @return   Returns bitmask of pushed keys
******************************************************************************/
uint32_t
bspKeyPushed(uint32_t ui32ReadMask)
{
#if (BSP_KEY_ALL != 0)
    //
    // Check input args
    //
    ASSERT((ui32ReadMask & BSP_KEY_ALL) != 0);

    if(ui32BspKeyMode == BSP_KEY_MODE_ISR)
    {
        uint32_t ui32Bm = 0;

        //
        // Disable global interrupts
        //
        bool bIntDisabled = IntMasterDisable();

        //
        // Critical section
        //
        ui32Bm = ui32BspKeysPressed;
        ui32BspKeysPressed &= ~ui32ReadMask;

        //
        // Re-enable interrupt if initially enabled.
        //
        if(!bIntDisabled)
        {
            IntMasterEnable();
        }

        //
        // Return bitmask of pushed keys
        //
        return (ui32Bm & ui32ReadMask);
    }
    else
    {
        //
        // Read pin states (all at once)
        //
        uint32_t ui32Pins = (~HWREG(GPIO_BASE + GPIO_O_DIN31_0) &
                             ui32ReadMask);

        //
        // Return early
        //
        if(ui32Pins == 0)
        {
            return 0;
        }

        //
        // Check LEFT key
        //
#if (BSP_KEY_LEFT != 0)
        if(ui32Pins & BSP_KEY_LEFT)
        {
#ifdef FPGA_INCLUDED
            bspFpgaForceDir(BSP_IOID_KEY_LEFT, 1, IOC_IOPULL_UP);
#else
            BSP_KEY_DEBOUNCE(GPIO_readDio(BSP_IOID_KEY_LEFT));
#endif
            return BSP_KEY_LEFT;
        }
#endif // if (BSP_KEY_LEFT != 0)

        //
        // Check RIGHT key
        //
#if (BSP_KEY_RIGHT != 0)
        if(ui32Pins & BSP_KEY_RIGHT)
        {
#ifdef FPGA_INCLUDED
            bspFpgaForceDir(BSP_IOID_KEY_RIGHT, 1, IOC_IOPULL_UP);
#else
            BSP_KEY_DEBOUNCE(GPIO_readDio(BSP_IOID_KEY_RIGHT));
#endif
            return BSP_KEY_RIGHT;
        }
#endif // if (BSP_KEY_RIGHT != 0)

        //
        // Check UP key
        //
#if (BSP_KEY_UP != 0)
        if(ui32Pins & BSP_KEY_UP)
        {
#ifdef FPGA_INCLUDED
            bspFpgaForceDir(BSP_IOID_KEY_UP, 1, IOC_IOPULL_UP);
#else
            BSP_KEY_DEBOUNCE(GPIO_readDio(BSP_IOID_KEY_UP));
#endif
            return BSP_KEY_UP;
        }
#endif // if (BSP_KEY_UP != 0)

        //
        // Check DOWN key
        //
#if (BSP_KEY_DOWN != 0)
        if(ui32Pins & BSP_KEY_DOWN)
        {
#ifdef FPGA_INCLUDED
            bspFpgaForceDir(BSP_IOID_KEY_DOWN, 1, IOC_IOPULL_UP);
#else
            BSP_KEY_DEBOUNCE(GPIO_readDio(BSP_IOID_KEY_DOWN));
#endif
            return BSP_KEY_DOWN;
        }
#endif // if (BSP_KEY_DOWN != 0)

        //
        // Check SELECT key
        //
#if (BSP_KEY_SELECT != 0)
        if(ui32Pins & BSP_KEY_SELECT)
        {
#ifdef FPGA_INCLUDED
            bspFpgaForceDir(BSP_IOID_KEY_SELECT, 1, IOC_IOPULL_UP);
#else
            BSP_KEY_DEBOUNCE(GPIO_readDio(BSP_IOID_KEY_SELECT));
#endif
            return BSP_KEY_SELECT;
        }
#endif // if (BSP_KEY_SELECT != 0)
    }

#endif // if (BSP_KEY_ALL != 0)
    //
    // No keys pressed
    //
    return 0;
}


/**************************************************************************//**
* @brief    This function reads the directional event. If multiple keys are
*           registered as "pressed", this function will only return the
*           directional event of the first key. Remaining key events will
*           be ignored. \sa bspKeyPushed()
*
* @return   Returns \b BSP_KEY_EVT_LEFT if LEFT key has been pressed.
* @return   Returns \b BSP_KEY_EVT_RIGHT if RIGHT key has been pressed.
* @return   Returns \b BSP_KEY_EVT_UP if UP key has been pressed.
* @return   Returns \b BSP_KEY_EVT_DOWN if DOWN key has been pressed.
* @return   Returns \b BSP_KEY_EVT_NONE if no key has been pressed.
******************************************************************************/
uint32_t
bspKeyGetDir(void)
{
#if (BSP_KEY_DIR_ALL != 0)
    //
    // Get bitmask of pressed keys
    //
    uint32_t ui32Bm = bspKeyPushed(BSP_KEY_ALL);

    //
    // Return directional event based on bitmask of pressed keys
    //
#if (BSP_KEY_LEFT != 0)
    if(ui32Bm & BSP_KEY_LEFT)
    {
        return BSP_KEY_EVT_LEFT;
    }
#endif
#if (BSP_KEY_RIGHT != 0)
    if(ui32Bm & BSP_KEY_RIGHT)
    {
        return BSP_KEY_EVT_RIGHT;
    }
#endif
#if (BSP_KEY_UP != 0)
    if(ui32Bm & BSP_KEY_UP)
    {
        return BSP_KEY_EVT_UP;
    }
#endif
#if (BSP_KEY_DOWN != 0)
    if(ui32Bm & BSP_KEY_DOWN)
    {
        return BSP_KEY_EVT_DOWN;
    }
#endif
#if (BSP_KEY_SELECT != 0)
    if(ui32Bm & BSP_KEY_SELECT)
    {
        return BSP_KEY_EVT_SELECT;
    }
#endif
#endif // if (BSP_KEY_DIR_ALL != 0)
    return BSP_KEY_EVT_NONE;
}


/**************************************************************************//**
* @brief    This function registers a custom ISR to keys specified by
*           \e ui32Keys.
*
* @note     If bspKeyInit() was initialized with argument \b BSP_KEY_MODE_POLL,
*           this function does nothing.
*
* @param    ui32Keys    is an ORed bitmask of keys (for example BSP_KEY_1).
* @param    pfnHandler  is a void function pointer to ISR.
*
* @return   None
******************************************************************************/
void
bspKeyIntRegister(uint32_t ui32Keys, void (*pfnHandler)(void))
{
#if (BSP_KEY_ALL != 0)
    //
    // Check input args
    //
    ASSERT((ui32Keys & BSP_KEY_ALL) != 0);

    if(ui32BspKeyMode == BSP_KEY_MODE_ISR)
    {
        //
        // Assign handler to the specified key(s)
        //
#if (BSP_KEY_SELECT != 0)
        if(ui32Keys & BSP_KEY_SELECT)
        {
            bspKeysIsrTable[0] = pfnHandler;
        }
#endif
#if (BSP_KEY_LEFT != 0)
        if(ui32Keys & BSP_KEY_LEFT)
        {
            bspKeysIsrTable[1] = pfnHandler;
        }
#endif
#if (BSP_KEY_RIGHT != 0)
        if(ui32Keys & BSP_KEY_RIGHT)
        {
            bspKeysIsrTable[2] = pfnHandler;
        }
#endif
#if (BSP_KEY_UP != 0)
        if(ui32Keys & BSP_KEY_UP)
        {
            bspKeysIsrTable[3] = pfnHandler;
        }
#endif
#if (BSP_KEY_DOWN != 0)
        if(ui32Keys & BSP_KEY_DOWN)
        {
            bspKeysIsrTable[4] = pfnHandler;
        }
#endif
    }
#endif // if (BSP_KEY_ALL != 0)
}


/**************************************************************************//**
* @brief    This function clears the custom ISR from keys specified by
*           \e ui32Keys.
*
* @note     If bspKeyInit() was initialized with argument \b BSP_KEY_MODE_POLL,
*           this function does nothing.
*
* @param    ui32Keys     is an ORed bitmask of keys (for example BSP_KEY_1).
*
* @return   None
******************************************************************************/
void
bspKeyIntUnregister(uint32_t ui32Keys)
{
#if (BSP_KEY_ALL != 0)
    if(ui32BspKeyMode == BSP_KEY_MODE_ISR)
    {
        //
        // Unregister handler for the specified keys
        //
        bspKeyIntRegister(ui32Keys, 0);
    }
#endif // if (BSP_KEY_ALL != 0)
}


/**************************************************************************//**
* @brief    This function enables interrupts on specified key GPIO pins.
*
* @note     If bspKeyInit() was initialized with argument \b BSP_KEY_MODE_POLL,
*           this function does nothing.
*
* @param    ui32Keys     is an ORed bitmask of keys (for example BSP_KEY_1).
*
* @return   None
******************************************************************************/
void
bspKeyIntEnable(uint32_t ui32Keys)
{
#if (BSP_KEY_ALL != 0)
    //
    // Check input argument
    //
    ASSERT((ui32Keys & BSP_KEY_ALL) != 0);

    if(ui32BspKeyMode == BSP_KEY_MODE_ISR)
    {
#if (BSP_KEY_UP != 0)
        if(ui32Keys & BSP_KEY_UP)
        {
            IOCIntEnable(BSP_IOID_KEY_UP);
        }
#endif
#if (BSP_KEY_DOWN != 0)
        if(ui32Keys & BSP_KEY_DOWN)
        {
            IOCIntEnable(BSP_IOID_KEY_DOWN);
        }
#endif
#if (BSP_KEY_LEFT != 0)
        if(ui32Keys & BSP_KEY_LEFT)
        {
            IOCIntEnable(BSP_IOID_KEY_LEFT);
        }
#endif
#if (BSP_KEY_RIGHT != 0)
        if(ui32Keys & BSP_KEY_RIGHT)
        {
            IOCIntEnable(BSP_IOID_KEY_RIGHT);
        }
#endif
#if (BSP_KEY_SELECT != 0)
        if(ui32Keys & BSP_KEY_SELECT)
        {
            IOCIntEnable(BSP_IOID_KEY_SELECT);
        }
#endif
    }
#endif // if (BSP_KEY_ALL != 0)s
}


/**************************************************************************//**
* @brief    This function disables interrupts on specified key GPIOs.
*
* @note     If bspKeyInit() was initialized with argument \b BSP_KEY_MODE_POLL,
*           this function does nothing.
*
* @param    ui32Keys     is an ORed bitmask of keys (for example BSP_KEY_1).
*
* @return   None
******************************************************************************/
void
bspKeyIntDisable(uint32_t ui32Keys)
{
#if (BSP_KEY_ALL != 0)
    //
    // Check input argument
    //
    ASSERT((ui32Keys & BSP_KEY_ALL) != 0);

    if(ui32BspKeyMode == BSP_KEY_MODE_ISR)
    {
#if (BSP_KEY_UP != 0)
        if(ui32Keys & BSP_KEY_UP)
        {
            IOCIntDisable(BSP_IOID_KEY_UP);
        }
#endif
#if (BSP_KEY_DOWN != 0)
        if(ui32Keys & BSP_KEY_DOWN)
        {
            IOCIntDisable(BSP_IOID_KEY_DOWN);
        }
#endif
#if (BSP_KEY_LEFT != 0)
        if(ui32Keys & BSP_KEY_LEFT)
        {
            IOCIntDisable(BSP_IOID_KEY_LEFT);
        }
#endif
#if (BSP_KEY_RIGHT != 0)
        if(ui32Keys & BSP_KEY_RIGHT)
        {
            IOCIntDisable(BSP_IOID_KEY_RIGHT);
        }
#endif
#if (BSP_KEY_SELECT != 0)
        if(ui32Keys & BSP_KEY_SELECT)
        {
            IOCIntDisable(BSP_IOID_KEY_SELECT);
        }
#endif
    }
#endif // if (BSP_KEY_ALL != 0)
}


/**************************************************************************//**
* @brief    This function clears interrupt flags on selected key GPIOs.
*
* @note     If bspKeyInit() was initialized with argument \b BSP_KEY_MODE_POLL,
*           this function does nothing.
*
* @param    ui32Keys     is an ORed bitmask of keys (for example BSP_KEY_1).
*
* @return   None
******************************************************************************/
void
bspKeyIntClear(uint32_t ui32Keys)
{
#if (BSP_KEY_ALL != 0)
    ASSERT((ui32Keys & BSP_KEY_ALL) != 0);

    if(ui32BspKeyMode == BSP_KEY_MODE_ISR)
    {
      GPIO_clearEventMultiDio(ui32Keys & BSP_KEY_ALL);
    }
#endif // if (BSP_KEY_ALL != 0)
}


/******************************************************************************
* LOCAL FUNCTIONS
*/
/**************************************************************************//**
* @brief    Interrupt Service Routine for an activated directional key.
*           Stores the pin where the interrupt occured, disables the interrupt
*           on that pin and starts the debouncing by use of WDT.
*
* @return   None
******************************************************************************/
static void
bspKeyPushedISR(void *pEvent)
{
#if (BSP_KEY_ALL != 0)
    //
    // Only treat key events
    //
    register uint_fast32_t ui32IrqBm = (((uint32_t)pEvent) & BSP_KEY_ALL);

    //
    // Return early if false interrupt
    //
    if(ui32IrqBm == 0)
    {
        __asm(" NOP");
        return;
    }

    //
    // Disable timer
    //
    bspKeyTimerDisable();

    //
    // Critical section: Disable global interrupts, update volatile
    // variables and re-enable global interrupts.
    //
    bool bIntDisabled = IntMasterDisable();

    //
    // Register new key presses
    //
    ui32BspKeysPressed |= ui32IrqBm;
    ui32BspKeyIntDisabledMask |= ui32IrqBm;

    //
    // End of critical section
    //
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }

    //
    // Disable interrupts on keys where interrupt flag was set.
    //
    if(ui32IrqBm & BSP_KEY_LEFT)
    {
#if (BSP_IOID_KEY_LEFT != IOID_UNUSED)
        IOCIntDisable(BSP_IOID_KEY_LEFT);
#endif
    }
    if(ui32IrqBm & BSP_KEY_RIGHT)
    {
#if (BSP_IOID_KEY_RIGHT != IOID_UNUSED)
        IOCIntDisable(BSP_IOID_KEY_RIGHT);
#endif
    }
    if(ui32IrqBm & BSP_KEY_UP)
    {
#if (BSP_IOID_KEY_UP != IOID_UNUSED)
        IOCIntDisable(BSP_IOID_KEY_UP);
#endif
    }
    if(ui32IrqBm & BSP_KEY_DOWN)
    {
#if (BSP_IOID_KEY_DOWN != IOID_UNUSED)
        IOCIntDisable(BSP_IOID_KEY_DOWN);
#endif
    }
    if(ui32IrqBm & BSP_KEY_SELECT)
    {
#if (BSP_IOID_KEY_SELECT != IOID_UNUSED)
        IOCIntDisable(BSP_IOID_KEY_SELECT);
#endif
    }

    //
    // Run custom ISRs (unrolled for speed)
    //
    if((ui32IrqBm & BSP_KEY_SELECT) && (bspKeysIsrTable[0] != 0))
    {
        (*bspKeysIsrTable[0])();
    }
    else if((ui32IrqBm & BSP_KEY_LEFT) && (bspKeysIsrTable[1] != 0))
    {
        (*bspKeysIsrTable[1])();
    }
    else if((ui32IrqBm & BSP_KEY_RIGHT) && (bspKeysIsrTable[2] != 0))
    {
        (*bspKeysIsrTable[2])();
    }
    else if((ui32IrqBm & BSP_KEY_UP) && (bspKeysIsrTable[3] != 0))
    {
        (*bspKeysIsrTable[3])();
    }
    else if((ui32IrqBm & BSP_KEY_DOWN) && (bspKeysIsrTable[4] != 0))
    {
        (*bspKeysIsrTable[4])();
    }

    //
    // Start the debounce timer
    //
    bspKeyTimerEnable();
#endif // if (BSP_KEY_ALL != 0)
}


/**************************************************************************//**
* @brief    Interrupt Service Routine for an activated key.
*           Stores the pin where the interrupt occured, disables the interrupt
*           on that pin and starts the debouncing timer.
*
* @return   None
******************************************************************************/
static void
bspKeyTimerISR(void)
{
    uint_fast32_t ui32DisBm = ui32BspKeyIntDisabledMask;

    //
    // Disable timer
    //
    bspKeyTimerDisable();

#if (BSP_KEY_ALL != 0)
    if(ui32DisBm & BSP_KEY_ALL)
    {
        //
        // Clear pending interrupts
        //
        GPIO_clearEventMultiDio((ui32DisBm & BSP_KEY_ALL));

        //
        // Re-enable the pin interrupts
        //
        if(ui32DisBm & BSP_KEY_LEFT)
        {
#ifdef FPGA_INCLUDED
            bspFpgaForceDir(BSP_IOID_KEY_LEFT, 1, IOC_IOPULL_UP);
            IOCPortConfigureSet(BSP_IOID_KEY_LEFT, IOC_PORT_GPIO, BSP_KEY_ISR_CFG);
#endif
            IOCIntEnable(BSP_IOID_KEY_LEFT);
        }
        if(ui32DisBm & BSP_KEY_RIGHT)
        {
#ifdef FPGA_INCLUDED
            bspFpgaForceDir(BSP_IOID_KEY_RIGHT, 1, IOC_IOPULL_UP);
            IOCPortConfigureSet(BSP_IOID_KEY_RIGHT, IOC_PORT_GPIO, BSP_KEY_ISR_CFG);
#endif
            IOCIntEnable(BSP_IOID_KEY_RIGHT);
        }
        if(ui32DisBm & BSP_KEY_UP)
        {
#ifdef FPGA_INCLUDED
            bspFpgaForceDir(BSP_IOID_KEY_UP, 1, IOC_IOPULL_UP);
            IOCPortConfigureSet(BSP_IOID_KEY_UP, IOC_PORT_GPIO, BSP_KEY_ISR_CFG);
#endif
            IOCIntEnable(BSP_IOID_KEY_UP);
        }
        if(ui32DisBm & BSP_KEY_DOWN)
        {
#ifdef FPGA_INCLUDED
            bspFpgaForceDir(BSP_IOID_KEY_DOWN, 1, IOC_IOPULL_UP);
            IOCPortConfigureSet(BSP_IOID_KEY_DOWN, IOC_PORT_GPIO, BSP_KEY_ISR_CFG);
#endif
            IOCIntEnable(BSP_IOID_KEY_DOWN);
        }
        if(ui32DisBm & BSP_KEY_SELECT)
        {
#ifdef FPGA_INCLUDED
            bspFpgaForceDir(BSP_IOID_KEY_SELECT, 1, IOC_IOPULL_UP);
            IOCPortConfigureSet(BSP_IOID_KEY_SELECT, IOC_PORT_GPIO, BSP_KEY_ISR_CFG);
#endif
            IOCIntEnable(BSP_IOID_KEY_SELECT);
        }
    }
#endif // if (BSP_KEY_ALL != 0)

    //
    // Clear bitmask
    //
    ui32BspKeyIntDisabledMask = 0;
}


/**************************************************************************//**
* @brief    Configure and start debounce timer.
*
* @return   None
******************************************************************************/
static void
bspKeyTimerEnable(void)
{
    WatchdogUnlock();
    WatchdogEnable();
    WatchdogIntClear();
    IntPendClear(INT_WDT_IRQ);
#ifdef DEBUG
    WatchdogStallEnable();   // WDT stops on debug halt
#endif
    IntEnable(INT_WDT_IRQ);
}


/**************************************************************************//**
* @brief    Disable debounce timer.
*
* @return   None
******************************************************************************/
static void
bspKeyTimerDisable(void)
{
    IntDisable(INT_WDT_IRQ);
    IntPendClear(INT_WDT_IRQ);
    WatchdogUnlock();
    WatchdogIntClear();
#ifdef DEBUG
    WatchdogStallEnable();   // WDT stops on debug halt
#endif
}


/**************************************************************************//**
* Close the Doxygen group.
* @}
******************************************************************************/
#endif // #ifndef BSP_KEY_EXCLUDE
