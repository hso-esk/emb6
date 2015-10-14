#ifndef __MCU_H__
#define __MCU_H__

/*============================================================================*/
/**
 * \file    mcu.h
 *
 * \author  Tobias Neff
 *
 * \brief   .
 *
 *          .
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <stdint.h>
#include <stdlib.h>
#include <msp430.h>
#include "targetconfig.h"

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/** Clock speed define for 1MHz */
#define MCU_SYSCLK_1MHZ         1000000UL
/** Clock speed define for 4MHz */
#define MCU_SYSCLK_4MHZ         4000000UL
/** Clock speed define for 8MHz */
#define MCU_SYSCLK_8MHZ         8000000UL
/** Clock speed define for 12MHz */
#define MCU_SYSCLK_12MHZ        12000000UL
/** Clock speed define for 16MHz */
#define MCU_SYSCLK_16MHZ        16000000UL
/** Clock speed define for 20MHz */
#define MCU_SYSCLK_20MHZ        20000000UL
/** Clock speed define for 25MHz */
#define MCU_SYSCLK_25MHZ        25000000UL

/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/

/**
 * Available Power Modes
 */
typedef enum E_MCU_PM_T
{
  /* Active Mode */
  E_MCU_PM_AM,
  /* Low Power Mode 0 */
  E_MCU_PM_0,
  /* Low Power Mode 1 */
  E_MCU_PM_1,
  /* Low Power Mode 2 */
  E_MCU_PM_2,
  /* Low Power Mode 3 */
  E_MCU_PM_3,
  /* Low Power Mode 4 */
  E_MCU_PM_4

} e_mcu_pm_t;


/*============================================================================*/
/*                          FUNCTION PROTOTYPES                               */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief    Function initializes the MSP430F5438a clocks and I/O for use on
 *           SmartRF06EB.
 *
 *           The function assumes an external crystal oscillator to be available
 *           to the MSP430F5438a. The MSP430F5438a main system clock (MCLK) and
 *           Sub Main System Clock (SMCLK) are set to the frequency given by
 *           input argument \e ui32SysClockSpeed. ACLK is set to 32768 Hz.
 *
 * @param    ul_sysClockSpeed   is the system clock speed in MHz; it must be
 *                               one of the following:
 *           \li \b MCU_SYSCLK_1MHZ
 *           \li \b MCU_SYSCLK_4MHZ
 *           \li \b MCU_SYSCLK_8MHZ
 *           \li \b MCU_SYSCLK_12MHZ
 *           \li \b MCU_SYSCLK_16MHZ
 *           \li \b MCU_SYSCLK_20MHZ
 *           \li \b MCU_SYSCLK_25MHZ
 *
 */
/*============================================================================*/
void mcu_sysClockInit (uint32_t ul_sysClockSpeed);


/*============================================================================*/
/**
 * @brief    Function returns the system clock speed.
 *
 * @return   Returns the system clock speed.
 */
/*============================================================================*/
uint32_t mcu_sysClockSpeedGet( void );


/*============================================================================*/
/**
 * @brief   Enter a power mode.
 *
 *          This function enters a specific current power mode. By entering a
 *          specific power mode the power consumption can be affected. How ever
 *          depending on the power mode not all of the functions are working.
 *
 * @param   e_pm        Power mode to set.
 */
/*============================================================================*/
void mcu_enterPm( e_mcu_pm_t e_pm );

/*============================================================================*/
/**
 * @brief   Enter critical section.
 *
 *          This function enters a critical section. By entering a
 *          critical section all interrupts are disabled.
 */
/*============================================================================*/
void mcu_enterCS( void );


/*============================================================================*/
/**
 * @brief   Exit critical section.
 *
 *          This function exits a critical section. By exiting a
 *          critical section all interrupts are enabled.
 */
/*============================================================================*/
void mcu_exitCS( void );


#endif /* #ifndef __MCU_H__ */
