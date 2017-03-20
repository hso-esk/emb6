/*
 * --- License --------------------------------------------------------------*
 */
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
 * Copyright (c) 2016,
 * Hochschule Offenburg, University of Applied Sciences
 * Institute of reliable Embedded Systems and Communications Electronics.
 * All rights reserved.
 */

/*
 *  --- Module Description ---------------------------------------------------*
 */
/**
 *  \file       ctimer.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Callback timer module for emb::6.
 *
 *              This module provides so-called callback timer. A callback timer
 *              can be used to trigger the execution of a callback function
 *              after a specific period.
 */

/*
 *  --- Includes -------------------------------------------------------------*
 */
#include "emb6.h"
#include "bsp.h"
#include "evproc.h"
#include "ctimer.h"
#include "timer.h"
#include "clist.h"


#define LOGGER_ENABLE                   LOGGER_CTIMER
#define LOGGER_SUBSYSTEM                "ctim"
#include "logger.h"

/*
 *  --- Local Variables ---------------------------------------------------- *
 */

/** List for the callback timer. */
LIST(gp_ctimList);


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */

/* Refresh the callback timer list. For further declaration please
 * refer to the function definition */
static void _ctimer_refresh( c_event_t event, void* data );


/*
 *  --- Local Functions ---------------------------------------------------- *
 */

/**
 * \brief   Refresh the callback timer list.
 *
 *          This function iterates through the list of registered
 *          callback timers and checks if they expired. If yes
 *          the according callback will be called.
 *          This function is given to the event timer as callback. It compares
 *          the event timer structures given as parameter of the callback to identify
 *          which of the callback timer expired actually.
 *
 * \param   event   New event
 * \param   data    Pointer to the data
 *
 */
void _ctimer_refresh( c_event_t event, void* data )
{
    struct ctimer *pst_cTim;

    for( pst_cTim = list_head(gp_ctimList); pst_cTim != NULL;
            pst_cTim = pst_cTim->next )
    {
      if( &pst_cTim->etimer == data )
      {
          list_remove( gp_ctimList, pst_cTim );
          if( pst_cTim->f != NULL )
          {
              pst_cTim->f( pst_cTim->ptr );
          }
          break;
      }
    }
}


/*
 * --- Global Function Definitions ----------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* ctimer_init()
*/
void ctimer_init( void )
{
    /* initialize timer list */
    list_init( gp_ctimList );

} /* ctimer_init() */


/*---------------------------------------------------------------------------*/
/*
* ctimer_set()
*/
void ctimer_set( struct ctimer* c, clock_time_t t,
        fn_ctimer_cb_t f, void* ptr )
{
    LOG_INFO("ctimer_set %p %u", c, (unsigned)t);
    c->f = f;
    c->ptr = ptr;

    /* set the associated etimer and add the timer to the list */
    etimer_set( &c->etimer, t, _ctimer_refresh );
    list_add( gp_ctimList, c );

} /* ctimer_set() */


/*---------------------------------------------------------------------------*/
/*
* ctimer_stop()
*/
void ctimer_stop( struct ctimer* pst_stopTim )
{
    /* Stop the timer and remove it from the list */
    etimer_stop( &pst_stopTim->etimer );
    list_remove( gp_ctimList, pst_stopTim );

} /* ctimer_stop() */


/*---------------------------------------------------------------------------*/
/*
* ctimer_reset()
*/
void ctimer_reset( struct ctimer* c )
{
  /* Reset the timer.*/
  etimer_reset( &c->etimer );

  /* Put timer to the end of the list */
  list_remove( gp_ctimList, c );
  list_add( gp_ctimList, c );

} /* ctimer_reset() */


/*---------------------------------------------------------------------------*/
/*
* ctimer_restart()
*/
void ctimer_restart( struct ctimer* c )
{
  /* Restart the timer */
  etimer_restart( &c->etimer );

  /* Put timer to the end of the list */
  list_remove( gp_ctimList, c );
  list_add( gp_ctimList, c );

} /* ctimer_restart() */


/*---------------------------------------------------------------------------*/
/*
* ctimer_expired()
*/
int ctimer_expired( struct ctimer* pst_checkTim )
{
    /* Check if the timer expired */
    return etimer_expired( &pst_checkTim->etimer );

} /* ctimer_expired() */
