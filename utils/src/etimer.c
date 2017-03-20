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
 *  \file       etimer.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Event timer module for emb::6.
 *
 *              This module provides so-called event timer. A event timer
 *              can be used to trigger the execution of an event
 *              after a specific period.
 */

/*
 *  --- Includes -------------------------------------------------------------*
 */

#include "emb6.h"
#include "etimer.h"
#include "clist.h"

#define LOGGER_ENABLE                   LOGGER_ETIMER
#define LOGGER_SUBSYSTEM                "etim"
#include    "logger.h"


/*
 * --- Macro Definitions --------------------------------------------------- *
 */

/** Shall the timer list be printed if debug is enabled */
#define ETIMER_PRINT_LIST               FALSE

#if ( (LOGGER_ENABLE == TRUE) && (ETIMER_PRINT_LIST == TRUE) )
#undef ETIMER_PRINT_LIST
#define ETIMER_PRINT_LIST()             _etimer_print_list()
#else
#undef ETIMER_PRINT_LIST
#define ETIMER_PRINT_LIST()
#endif /* #if ( (LOGGER_ENABLE == TRUE) && (ETIMER_PRINT_LIST == TRUE) ) */

/*
 *  --- Local Variables ---------------------------------------------------- *
 */

/** List for the event timer. */
LIST(gp_etimList);

/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */

/* Add a timer to the list. For further declaration please
 * refer to the function definition */
static void _etimer_addTimer( struct etimer *pst_timer );

#if LOGGER_ENABLE == TRUE
/* Print timer list. For further declaration please
 * refer to the function definition */
static void _etimer_print_list( void );
#endif /* #if LOGGER_ENABLE == TRUE */


/**
 * \brief   Add timer to the timer list.
 *
 *          This function adds a new timer to the list. In case the timer
 *          already exists it will be removed and put to the end of the list.
 *
 * \param   pst_timer   Pointer to a timer to be added.
 */
static void _etimer_addTimer( struct etimer *pst_timer )
{
    list_remove(gp_etimList, pst_timer);
    list_add(gp_etimList, pst_timer);
    pst_timer->active = TMR_ACTIVE;
}


#if LOGGER_ENABLE == TRUE
/**
 * \brief   Print list of timers.
 *
 *          This function prints all the timers that are currently
 *          registered..
 */
void _etimer_print_list( void )
{
    struct etimer * st_temp;
    uint8_t j=0;
    LOG_INFO("%s\n\r","timer list");
    for (st_temp = list_head(gp_etimList); \
        st_temp != NULL; \
        st_temp = list_item_next(st_temp), j++) {
        LOG_RAW("%d | %p : %p : %lu : %lu\n\r",j,st_temp,st_temp->next,st_temp->timer.start,st_temp->timer.interval);
    }
}
#endif /* #if LOGGER_ENABLE == TRUE */

/*
 * --- Global Function Definitions ----------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* etimer_init()
*/
void etimer_init(void)
{
    /* initialize list */
    list_init(gp_etimList);

} /* etimer_init */


/*---------------------------------------------------------------------------*/
/*
* etimer_request_poll()
*/
void etimer_request_poll(void)
{
    struct etimer* pst_tTim = list_head(gp_etimList);
    struct etimer* pst_nTim = NULL;

    if( pst_tTim == NULL )
        return;
    /* Important to remember that all of the etimer structure are stored in
     * the different modules, that means that etimer library just manages linking
     * between them. */

    /* Until we will not point to  NULL keep searching through the list */
    while( pst_tTim != NULL )
    {
        /* get next element in in the list */
        pst_nTim = list_item_next( pst_tTim );

        /* We take one by one timer and check if it is expired */
        if( timer_expired( &(pst_tTim->timer) ) )
        {
            LOG_INFO("delete %p from list\n\r",pst_tTim);
            ETIMER_PRINT_LIST();

            /* Store pointer to a next timer, to check all timers in a list
             * Generate timer expired event */
            evproc_putEvent( E_EVPROC_TAIL,EVENT_TYPE_TIMER_EXP, pst_tTim );

            /* Remove matched timer from the list and set the active flag */
            list_remove( gp_etimList, pst_tTim );
            pst_tTim->active = TMR_NOT_ACTIVE;

        }
        pst_tTim = pst_nTim;
    }

} /* etimer_request_poll() */


/*---------------------------------------------------------------------------*/
/*
* etimer_set()
*/
void etimer_set( struct etimer* pst_et, clock_time_t l_interval,
        pfn_callback_t pfn_callback )
{
    /* set the underlying timer */
    timer_set(&pst_et->timer, l_interval);

    /* add the timer to the list and register the callback for the timer.*/
    _etimer_addTimer( pst_et );
    evproc_regCallback( EVENT_TYPE_TIMER_EXP, pfn_callback );

    LOG_INFO("add new timer %p\n\r",pst_et);
    ETIMER_PRINT_LIST();

}/* etimer_set() */


/*---------------------------------------------------------------------------*/
/*
* etimer_stop()
*/
void etimer_stop( struct etimer* pst_et )
{
    /* remove the timer from the list and stop it */
    list_remove( gp_etimList, pst_et );
    pst_et->active = TMR_NOT_ACTIVE;

    LOG_INFO("stop timer %p\n\r",pst_et);

} /* etimer_stop() */


/*---------------------------------------------------------------------------*/
/*
* etimer_reset()
*/
void etimer_reset( struct etimer* pst_et )
{
    /* reset the timer and add it to the end of the list */
    timer_reset( &pst_et->timer );
    _etimer_addTimer( pst_et );

    LOG_INFO("reset timer %p\n\r",pst_et);

}/* etimer_reset() */


/*---------------------------------------------------------------------------*/
/*
* etimer_restart()
*/
void etimer_restart( struct etimer* pst_et )
{
    /* reset the timer and add it to the end of the list */
    timer_restart(&pst_et->timer);
    _etimer_addTimer(pst_et);

    LOG_INFO("restart timer %p\n\r",pst_et);

}/* etimer_restart() */


/*---------------------------------------------------------------------------*/
/*
* etimer_adjust()
*/
void etimer_adjust( struct etimer* pst_et, int32_t l_timediff )
{
    /* adjust the timer */
    pst_et->timer.start += l_timediff;

    LOG_INFO("adjust timer %p\n\r",pst_et);

}/* etimer_adjust() */


/*---------------------------------------------------------------------------*/
/*
* etimer_expired()
*/
int etimer_expired( struct etimer* pst_et )
{
    /* check the timer state and return */
    return ( pst_et->active == TMR_NOT_ACTIVE );

} /* etimer_expired() */


/*---------------------------------------------------------------------------*/
/*
* etimer_expiration_time()
*/
clock_time_t etimer_expiration_time(struct etimer *pst_et)
{
    /* Get the expiration time of the timer and return. */
    return pst_et->timer.start + pst_et->timer.interval;

} /* etimer_expiration_time() */


/*---------------------------------------------------------------------------*/
/*
* etimer_start_time()
*/
clock_time_t etimer_start_time(struct etimer *pst_et)
{
    /* Return the start time of the timer */
    return pst_et->timer.start;

}/* etimer_start_time() */


/*---------------------------------------------------------------------------*/
/*
* etimer_nextEvent()
*/
clock_time_t etimer_nextEvent(void)
{
    struct etimer* pst_tTim = list_head(gp_etimList);
    struct etimer* pst_nTim = NULL;

    clock_time_t ct_expTime;
    clock_time_t ct_nextExpTime = 0;

    /* no items in list */
    if( list_length(gp_etimList) == 0 )
        return 0;

    /* search timer with next expiration time */
    while( pst_tTim != NULL )
    {
        /* get next element in list */
        pst_nTim = list_item_next( pst_tTim );

        /* timer enabled? */
        if( pst_tTim->active == TMR_ACTIVE )
        {
            /* get timer expiration time */
            ct_expTime = pst_tTim->timer.start + pst_tTim->timer.interval;

            if ( ct_nextExpTime == 0)
                ct_nextExpTime = ct_expTime;
            else
            {
                if (ct_expTime < ct_nextExpTime)
                    /* save next expiration time */
                    ct_nextExpTime = ct_expTime;
            }
        }

        /* next timer */
        pst_tTim = pst_nTim;
    }

    return ct_nextExpTime;

} /* etimer_nextEvent() */


