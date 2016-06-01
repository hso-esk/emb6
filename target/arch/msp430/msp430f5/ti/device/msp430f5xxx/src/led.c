/*============================================================================*/
/**
 * \file    led.c
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
#include <msp430.h>
#include <stdint.h>
#include "targetconfig.h"
#include "led.h"
#include "io.h"

/*============================================================================*/
/*                                LOCAL VARIABLES                             */
/*============================================================================*/

/** All the available led pins from the IO module */
static s_io_pin_desc_t gps_led_pin[] = {

#if( TARGET_CONFIG_LED1 == TRUE )
    {&gps_io_port[TARGETCONFIG_LED1_PORT], TARGETCONFIG_LED1_PIN, TARGETCONFIG_LED1_MSK},
#endif /* #if( TARGET_CONFIG_LED1 == TRUE ) */
#if( TARGET_CONFIG_LED2 == TRUE )
    {&gps_io_port[TARGETCONFIG_LED2_PORT], TARGETCONFIG_LED2_PIN, TARGETCONFIG_LED2_MSK},
#endif /* #if( TARGET_CONFIG_LED2 == TRUE ) */
#if( TARGET_CONFIG_LED3 == TRUE )
    {&gps_io_port[TARGETCONFIG_LED3_PORT], TARGETCONFIG_LED3_PIN, TARGETCONFIG_LED3_MSK},
#endif /* #if( TARGET_CONFIG_LED3 == TRUE ) */
#if( TARGET_CONFIG_LED4 == TRUE )
    {&gps_io_port[TARGETCONFIG_LED4_PORT], TARGETCONFIG_LED4_PIN, TARGETCONFIG_LED4_MSK},
#endif /* #if( TARGET_CONFIG_LED4 == TRUE ) */

};


/*============================================================================*/
/*                              FUNCTIONS                                     */
/*============================================================================*/

/*=============================================================================
 * led_init()
 *============================================================================*/
void led_init( void )
{
  int i = 0;

  /* Clear all LEDS */
  for( i = 0; i < E_LED_MAX; i++ )
    led_clear( (e_led_t)i );
}/* led_init() */

/*=============================================================================
 * led_set()
 *============================================================================*/
void led_set( e_led_t e_led )
{
#if( TARGET_CONFIG_LED_HL == TRUE )
    /* Set pin high to activate */
  io_set( &gps_led_pin[e_led] );
#else
  /* Set pin low to activate */
  io_clear( &gps_led_pin[e_led] );
#endif

}/* led_set() */

/*=============================================================================
 * led_clear()
 *============================================================================*/
void led_clear( e_led_t e_led )
{
#if( TARGET_CONFIG_LED_HL == TRUE )
  /* Set pin low to deactivate */
  io_clear( &gps_led_pin[e_led] );
#else
  /* Set pin high to deactivate */
  io_set( &gps_led_pin[e_led] );
#endif
}/* led_clear() */

/*=============================================================================
 * led_toggle()
 *============================================================================*/
void led_toggle(  e_led_t e_led  )
{
  /* Toggle */
  *gps_led_pin[e_led].PORT->POUT ^= gps_led_pin[e_led].MSK;
}/* led_toggle() */
