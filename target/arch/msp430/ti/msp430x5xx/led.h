#ifndef __LED_H__
#define __LED_H__

/*============================================================================*/
/**
 * \file    led.h
 *
 * \author  Tobias Neff
 *
 * \brief   LED functions.
 *
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <stdint.h>
#include <msp430.h>
#include "targetconfig.h"

/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/

/**
 * Available Keys
 */
typedef enum E_LED_T
{
#if( TARGET_CONFIG_LED1 == TRUE )
  /** LED 1 */
  E_LED_1,
#endif /* #if( TARGET_CONFIG_LED1 == TRUE ) */

#if( TARGET_CONFIG_LED2 == TRUE )
  /** LED 2 */
  E_LED_2,
#endif /* #if( TARGET_CONFIG_LED2 == TRUE ) */

#if( TARGET_CONFIG_LED3 == TRUE )
  /** LED 3 */
  E_LED_3,
#endif /* #if( TARGET_CONFIG_LED3 == TRUE ) */
#if( TARGET_CONFIG_LED4 == TRUE )
  /** LED 4 */
  E_LED_4,
#endif /* #if( TARGET_CONFIG_LED4 == TRUE ) */

  E_LED_MAX
} e_led_t;


/*============================================================================*/
/*                          FUNCTION PROTOTYPES                               */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief    Initializes the LEDs.
 *
 *           This function initializes the LEDs. The function shall be called
 *           at least once at the start of the program.
 */
/*============================================================================*/
void led_init( void );

/*============================================================================*/
/**
 * @brief    Sets an LED.
 *
 *           This function sets a specific LED.
 *
 * @param    e_led      LED to set.
 */
/*============================================================================*/
void led_set( e_led_t e_led );


/*============================================================================*/
/**
 * @brief    Clears an LED
 *
 *           This function clears a specific LED.
 *
 * @param    e_led      LED to clear.
 */
/*============================================================================*/
void led_clear( e_led_t e_led );


/*============================================================================*/
/**
 * @brief    Toggle an LED.
 *
 *           This function toggles a specific LED.
 *
 * @param    e_led      LED to toggle.
 */
/*============================================================================*/
void led_toggle( e_led_t e_led );

#endif /* #ifndef __LED_H__ */
