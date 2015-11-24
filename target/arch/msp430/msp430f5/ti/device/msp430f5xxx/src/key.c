/*============================================================================*/
/**
 * \file    key.c
 *
 * \author  Tobias Neff
 *
 * \brief   Key functions.
 *
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <msp430.h>
#include <stdint.h>
#include "targetconfig.h"
#include "key.h"
#include "io.h"

/*============================================================================*/
/*                                MACROS                                      */
/*============================================================================*/
/* Active wait debounce macro */
#define KEY_DEBOUNCE(expr)  do{ uint16_t ui; for (ui=0; ui<500; ui++) {         \
                              if (!(expr)) ui = 0; } }while(0)



/*============================================================================*/
/*                                LOCAL VARIABLES                             */
/*============================================================================*/

/** All the availabel key pins from the IO module */
static s_io_pin_desc_t ps_key_pin[] = {

#if( TARGET_CONFIG_KEY1 == TRUE )
    {&gps_io_port[TARGETCONFIG_KEY1_PORT], TARGETCONFIG_KEY1_PIN, TARGETCONFIG_KEY1_MSK},
#endif /* #if( TARGET_CONFIG_KEY1 == TRUE ) */
#if( TARGET_CONFIG_KEY2 == TRUE )
    {&gps_io_port[TARGETCONFIG_KEY2_PORT], TARGETCONFIG_KEY2_PIN, TARGETCONFIG_KEY2_MSK},
#endif /* #if( TARGET_CONFIG_KEY2 == TRUE ) */
#if( TARGET_CONFIG_KEY3 == TRUE )
    {&gps_io_port[TARGETCONFIG_KEY3_PORT], TARGETCONFIG_KEY3_PIN, TARGETCONFIG_KEY3_MSK},
#endif /* #if( TARGET_CONFIG_KEY3 == TRUE ) */
#if( TARGET_CONFIG_KEY4 == TRUE )
    {&gps_io_port[TARGETCONFIG_KEY4_PORT], TARGETCONFIG_KEY4_PIN, TARGETCONFIG_KEY4_MSK},
#endif /* #if( TARGET_CONFIG_KEY4 == TRUE ) */
#if( TARGET_CONFIG_KEY5 == TRUE )
    {&gps_io_port[TARGETCONFIG_KEY5_PORT], TARGETCONFIG_KEY5_PIN, TARGETCONFIG_KEY5_MSK},
#endif /* #if( TARGET_CONFIG_KEY5 == TRUE ) */

};


/*============================================================================*/
/*                              FUNCTIONS                                     */
/*============================================================================*/

/*=============================================================================
 *  key_init()
 *============================================================================*/
void key_init( void )
{
  /* nothing to do for the initialization */
} /* key_init() */

/*=============================================================================
 *  key_pushed()
 *============================================================================*/
uint8_t key_pushed( e_key_t e_key )
{
  /* check if the key is pushed */
  if( ~(*ps_key_pin[e_key].PORT->PIN) & ps_key_pin[e_key].MSK )
    return 1;
  else
    return 0;
} /* key_pushed() */


/*=============================================================================
 *  key_intRegister()
 *============================================================================*/
void key_intRegister( e_key_t e_key, pf_key_cb pf_cb )
{
  /* enable IO interrupt for the rising edge */
    io_extiRegister( &ps_key_pin[e_key], INT_EDGE_RISING, pf_cb );
    io_extiEnable(&ps_key_pin[e_key]);
} /* key_intRegister() */

/*=============================================================================
 *  key_intUnregister()
 *============================================================================*/
void key_intUnregister( e_key_t e_key )
{
  /* Disable IRQ */
    io_extiDisable( &ps_key_pin[e_key] );
} /* key_intUnregister() */


/*=============================================================================
 *  key_intClear()
 *============================================================================*/
void key_intClear( e_key_t e_key )
{
  /* clear IRQ */
    io_extiClear( &ps_key_pin[e_key] );
} /* key_intClear() */

