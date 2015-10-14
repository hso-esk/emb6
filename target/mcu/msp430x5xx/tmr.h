#ifndef __TMR_H__
#define __TMR_H__

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <stdint.h>
#include <stdlib.h>
#include <msp430.h>
#include "hal_types.h"
#include "targetconfig.h"
#include "int.h"

/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/

/**
 * Available Timer.
 */
typedef enum E_TMR_T
{
  /* Timer 0 */
  E_TMR_0,
  /* TIMER 1 */
  E_TMR_1,

  E_TMR_MAX
} e_tmr_t;


/*============================================================================*/
/*                     STRUCTURES AND OTHER TYPEDEFS                          */
/*============================================================================*/

/**
 * Description of a Timer.
 *
 * Defines all the elements to access a Timers fuctions and to configure
 * a Timer properly.
 */
typedef struct S_TMR_DESC_T
{
  /** Control */
  REG16B CTL;
  /** Capture/Compare Control 0*/
  REG16B CCTL0;
  /** Counter */
  REG16B R;
  /** Capture Compare 0 */
  REG16B CCR0;
  /** Expansion register */
  REG16B EX;

  /** interrupt source */
  e_int_irq_src_t e_irqSrc;
  /** interrupt handler to use */
  pf_int_cb pf_isr;

} s_tmr_desc_t;

/** Prototype of a timer interrupt callback */
typedef void (*pf_tmr_cb)( void *p_arg);


/*============================================================================*/
/*                               FUNCTIONS                                    */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief   Function initializes the timers.
 *
 *          This functoin initializes the timers. The function has to be called
 *          at least once before any timer operation.
 *
 * @return  0 on success or a negative alue on error.
 */
/*============================================================================*/
int8_t tmr_init( void );


/*============================================================================*/
/**
 * @brief    Function configures a timer.
 *
 *           This function is used to configure a timer. A timer configuration
 *           currently consits of the timout of the timer in ms.
 *
 * @param    e_tmr      Timer to configure.
 * @param    ui_tot     Period of the timer in us for T0 and ms for T1.
 *
 * @return  0 on success or a negative value on error.
 */
/*============================================================================*/
int8_t tmr_config( e_tmr_t e_tmr, uint16_t ui_tot );


/*============================================================================*/
/**
 * @brief    Function starts a timer.
 *
 *           This function starts a timer. The timer must have been
 *           initialized and configured first.
 *
 * @param    e_tmr      Timer to configure.
 * @param    pf_cb      Callback to be invoked on expire.
 *
 * @return  0 on success or a negative value on error.
 */
/*============================================================================*/
int8_t tmr_start( e_tmr_t e_tmr, pf_tmr_cb pf_cb );


/*============================================================================*/
/**
 * @brief    Function stops a timer.
 *
 * @param    e_tmr      Timer to configure.
 *
 * @return  0 on success or a negative value on error.
 */
/*============================================================================*/
int8_t tmr_stop( e_tmr_t e_tmr );


/*============================================================================*/
/**
 * @brief    Retrieve the current timer value of timer 1.
 *
 *           This function can be used to retrieve the current value of
 *           a timers counter in ms. This function can only be used for
 *           Timer 1.
 *
 * @return  The current caounter value as ms.
 */
/*============================================================================*/
uint16_t tmr_getT1Count( void );

#endif /* #ifndef __TMR_H__ */
