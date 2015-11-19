/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <stdint.h>
#include <stdlib.h>
#include <msp430.h>
#include "targetconfig.h"
#include "tmr.h"
#include "io.h"
#include "int.h"
#include "led.h"

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/** Frequency of the clock used for the timer */
#define TMR_CLK_FREQ                    32768


/*============================================================================*/
/*                          LOCAL FUNCTION PROTOTYPES                         */
/*============================================================================*/

/* Timer 0 interrupt handler */
static void _tmr_irqHandler0( void* p_params );

/* Timer 1 interrupt handler */
static void _tmr_irqHandler1( void* p_params );

/*============================================================================*/
/*                            LOCAL VARIABLES                                 */
/*============================================================================*/

/** References to the according Timer callbacks */
pf_tmr_cb gpf_tmrCB[E_TMR_MAX] = { NULL };

/**
 * Timer definitions.
 */
s_tmr_desc_t gps_tmr[E_TMR_MAX] = {

  /* TimerA */
  {&TA1CTL, &TA1CCTL0, &TA1R, &TA1CCR0,&TA1EX0, E_INT_IRQ_SRC_T1, _tmr_irqHandler0},

  /* TimerB */
  {&TB0CTL, &TB0CCTL0, &TB0R, &TB0CCR0,&TB0EX0, E_INT_IRQ_SRC_T2, _tmr_irqHandler1},

};

/*============================================================================*/
/*                              LOCAL FUNCTIONS                               */
/*============================================================================*/


/*============================================================================*/
/**
 * @brief   TimerA interrupt handler.
 *
 *          This function is invoked on every TimerA interrupt if a callback
 *          was registerd before. Every time an interrupt is recognized
 *          the registered callback will be invoked.
 */
/*============================================================================*/
static void _tmr_irqHandler0( void* p_params )
{
  e_tmr_t e_tmr = E_TMR_0;

  /* Invoke callback if registered */
  if( gpf_tmrCB[e_tmr] != NULL )
      gpf_tmrCB[e_tmr](NULL);
}


/*============================================================================*/
/**
 * @brief   TimerA interrupt handler.
 *
 *          This function is invoked on every TimerA interrupt if a callback
 *          was registerd before. Every time an interrupt is recognized
 *          the registered callback will be invoked.
 */
/*============================================================================*/
static void _tmr_irqHandler1( void* p_params )
{
  e_tmr_t e_tmr = E_TMR_1;

  /* Invoke callback if registered */
  if( gpf_tmrCB[e_tmr] != NULL )
      gpf_tmrCB[e_tmr](NULL);
}


/*============================================================================*/
/*                              FUNCTIONS                                     */
/*============================================================================*/

/*=============================================================================
 *  tmr_init()
 *============================================================================*/
int8_t tmr_init( void )
{
  int i = 0;
  int8_t i_ret = 0;

  for( i = 0; i < E_TMR_MAX; i++ )
  {
    tmr_stop( (e_tmr_t)i );
    gpf_tmrCB[i] = NULL;
  }

  return i_ret;
}/* tmr_init() */

/*=============================================================================
 *  tmr_config()
 *============================================================================*/
int8_t tmr_config( e_tmr_t e_tmr, uint16_t ui_tot  )
{
  int8_t i_ret = 0;
  uint16_t ui_count;

  if( e_tmr == E_TMR_0 )
  {
    /* Set ACLK as clock source and enable count up mode */
    *gps_tmr[e_tmr].CTL = TASSEL_1 | MC_0 | CNTL_0;

    /* configure timer for timeout value in us */
    ui_count = ( (TMR_CLK_FREQ * ui_tot) / 1000000 );
  }
  else
  {
    /* Set ACLK as clock source and enable count up mode and input divider = 3*/
    *gps_tmr[e_tmr].CTL = TBSSEL_1 | MC_0 | CNTL_0 | ID_3;
    /* additional divider = 4 */
    *gps_tmr[e_tmr].EX = TBIDEX_3;

    /* calculate timer ticks for the timout value in ms */
    ui_count = ( ui_tot );
  }

  /* set the counter value */
  *gps_tmr[e_tmr].CCR0 = ui_count;

  return i_ret;
}/* tmr_config() */

/*=============================================================================
 *  tmr_start()
 *============================================================================*/
int8_t tmr_start( e_tmr_t e_tmr, pf_tmr_cb pf_cb )
{
  int8_t i_ret = -1;

  /* register the callback */
  gpf_tmrCB[e_tmr] = pf_cb;
  int_irqRegister( gps_tmr[e_tmr].e_irqSrc, gps_tmr[e_tmr].pf_isr );

  /* reset the timer register */
  *gps_tmr[e_tmr].R = 0;

  /* enable the timer interrupt */
  *gps_tmr[e_tmr].CCTL0 |= CCIE;

  /* start the timer */
  *gps_tmr[e_tmr].CTL |= MC_1;

  i_ret = 0;

  return i_ret;
}/* tmr_start() */


/*=============================================================================
 *  tmr_stop()
 *============================================================================*/
int8_t tmr_stop( e_tmr_t e_tmr )
{
  int i_ret = 0;

  /* disable interrupt */
  *gps_tmr[e_tmr].CCTL0 &= ~CCIE;

  /* reset the timer register */
  *gps_tmr[e_tmr].R = 0;

  /* unregister callback and interrupt handler */
  int_irqUnregister( gps_tmr[e_tmr].e_irqSrc );
  gpf_tmrCB[e_tmr] = NULL;

  return i_ret;
}/* tmr_stop() */


/*=============================================================================
 *  tmr_getT1Count()
 *============================================================================*/
uint16_t tmr_getT1Count( void )
{
  int16_t i_ret = 0;
  e_tmr_t e_tmr = E_TMR_1;

  /* get counter value */
  i_ret = *gps_tmr[e_tmr].R;

  return i_ret;
}/* tmr_getT1Count() */
