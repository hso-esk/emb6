#ifndef __KEY_H__
#define __KEY_H__

/*============================================================================*/
/**
 * \file    key.h
 *
 * \author  Tobias Neff
 *
 * \brief   KEY functions.
 *
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <msp430.h>
#include <stdint.h>
#include "targetconfig.h"

/*============================================================================*/
/*                              TYPEDEFS                                      */
/*============================================================================*/

/** Prototype of an interrupt callback */
typedef void (*pf_key_cb)( void *p_arg);


/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/

/**
 * Available Keys
 */
typedef enum E_KEY_T
{
#if( TARGET_CONFIG_KEY1 == TRUE )
  /** KEY 1 */
  E_KEY_1,
#endif /* #if( TARGET_CONFIG_KEY1 == TRUE ) */
#if( TARGET_CONFIG_KEY2 == TRUE )
  /** KEY 2 */
  E_KEY_2,
#endif /* #if( TARGET_CONFIG_KEY2 == TRUE ) */
#if( TARGET_CONFIG_KEY3 == TRUE )
  /** KEY 3 */
  E_KEY_3,
#endif /* #if( TARGET_CONFIG_KEY3 == TRUE ) */
#if( TARGET_CONFIG_KEY4 == TRUE )
  /** KEY 4 */
  E_KEY_4,
#endif /* #if( TARGET_CONFIG_KEY4 == TRUE ) */
#if( TARGET_CONFIG_KEY5 == TRUE )
  /** KEY 5 */
  E_KEY_5,
#endif /* #if( TARGET_CONFIG_KEY5 == TRUE ) */

  E_KEY_MAX
} e_key_t;


/*============================================================================*/
/*                          FUNCTION PROTOTYPES                               */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief    initializes the key module.
 */
/*============================================================================*/
void key_init( void );

/*============================================================================*/
/**
 * @brief    This function returns if a key is pushed.
 *
 * @note     This function can be used to check if a specific key was
 *           pushed. If a key was pushed this function will return a positive
 *           value.
 *
 * @param    e_key    Key to check.
 *
 * @return   Positive value if key was pushed or 0 if not.
 */
/*============================================================================*/
uint8_t key_pushed( e_key_t e_key );


/*============================================================================*/
/**
 * @brief    Enable interrupt on a specfific key.
 *
 *           This function can be used to register a callback for a
 *           specific key. This also enables the the according interrupt
 *           for that key and invokes the callback every time the interrupt
 *           gets active.
 *
 * @param    e_key      Key to register interrupt for.
 * @param    pf_cb      Callback for the interrupt.
 */
/*============================================================================*/
void key_intRegister( e_key_t e_key, pf_key_cb pf_cb );


/*============================================================================*/
/**
 * @brief    Disable interrupt for a specific key.
 *
 *           This function can be used to disable a callback for a
 *           specific key. This also disables the the according interrupt.
 *
 * @param    e_key     Key to unregister interrupt for.
 */
/*============================================================================*/
void key_intUnregister( e_key_t e_key );


/*============================================================================*/
/**
 * @brief    This function clears interrupt flags on selected key GPIOs.
 *
 *           Clears the interrupt for a specific key. This function shall be
 *           called every time within the according callback.
 *
 * @param    e_key     Key to clear interrupt for.
 */
/*============================================================================*/
void key_intClear( e_key_t e_key );


#endif /* #ifndef __KEY_H__ */
