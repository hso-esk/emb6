/**
 *   \addtogroup etimer Event timer library
 *   @{
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
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
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
 */
/*============================================================================*/
/*!
    \file   etimer.c

    \author Artem Yushev 

    \brief  Functions to manage Contiki timers
            Contiki has two main type of timers callback timers (ctimer) and
            event timer (etimer)
            The first one push callbacks after timer expires and the second one
            push event when timer expires.

  \version  0.1
*/
/*============================================================================*/

/*==============================================================================
                             INCLUDE FILES
==============================================================================*/

#include "etimer.h"
#include "clist.h"

#include "emb6_conf.h"
#include "emb6.h"

#include "bsp.h"

/*==============================================================================
                             LOCAL MACROS
==============================================================================*/
#define     LOGGER_ENABLE        LOGGER_ETIMER
#if            LOGGER_ENABLE     ==     TRUE
#define        LOGGER_SUBSYSTEM    "etim"
#endif
#include    "logger.h"

/*==============================================================================
                            LOCAL VARIABLES
==============================================================================*/
static             clock_time_t     l_nextExp = 0;
LIST(gp_etimList);
static     char         gc_init = 0;
/*==============================================================================
                             LOCAL FUNCTIONS
==============================================================================*/
/*============================================================================*/
/*!
*   \brief   Add timer to the timer list
*
*    \param        pst_timer        Pointer to a timer to be added
*    \retval        none
*/
/*============================================================================*/
static void _etimer_addTimer(struct etimer *pst_timer)
{
    list_remove(gp_etimList, pst_timer);
    list_add(gp_etimList, pst_timer);
    pst_timer->active = TMR_ACTIVE;
}

void etimer_print_list(void)
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
/*==============================================================================
                             API FUNCTIONS
==============================================================================*/
/*============================================================================*/
/*  etimer_init()                                                     */
/*============================================================================*/
void etimer_init(void)
{
    if (gc_init)
        return;
    list_init(gp_etimList);
    gc_init = 1;
} /* etimer_init */

/*============================================================================*/
/*  etimer_request_poll()                                                     */
/*============================================================================*/
void etimer_request_poll(void)
{
    struct    etimer     *    pst_tTim = list_head(gp_etimList);
    struct    etimer    *    pst_nTim = NULL; ///! Next timer
    if (pst_tTim == NULL)
        return;
    // Importamt to remember that all of the etimer structure are stored in
    // the different modules, that means that etimer library just manages linking
    // between them.
    // Until we will not point to the NULL keep searching through the list
    while(pst_tTim != NULL) {
        pst_nTim = list_item_next(pst_tTim);
        // We take one by one timer and check if it is expired
        if(timer_expired(&(pst_tTim->timer))) {
            LOG_INFO("delete %p from list\n\r",pst_tTim);
//            etimer_print_list();
            // Store pointer to a next timer, to check all timers in a list
            // Generate timer expired event
            evproc_putEvent(E_EVPROC_TAIL,EVENT_TYPE_TIMER_EXP,pst_tTim);
            // Remove matched timer from the list
            list_remove(gp_etimList, pst_tTim);
            // Change active flag
            pst_tTim->active = TMR_NOT_ACTIVE;
        } /* if */
        pst_tTim = pst_nTim;
    } /* while */
} /* etimer_request_poll() */

/*============================================================================*/
/*  etimer_set()                                                                 */
/*============================================================================*/
void etimer_set(struct etimer *pst_et, clock_time_t l_interval, pfn_callback_t pfn_callback)
{
    timer_set(&pst_et->timer, l_interval);
    _etimer_addTimer(pst_et);
    evproc_regCallback(EVENT_TYPE_TIMER_EXP, pfn_callback);
    LOG_INFO("add new timer %p\n\r",pst_et);
//    etimer_print_list();
}/* etimer_set() */

/*============================================================================*/
/*  etimer_reset()                                                                 */
/*============================================================================*/
void etimer_reset(struct etimer *pst_et)
{
    timer_reset(&pst_et->timer);
    _etimer_addTimer(pst_et);
    LOG_INFO("reset timer %p\n\r",pst_et);
}/* etimer_reset() */

/*============================================================================*/
/*  etimer_restart()                                                                 */
/*============================================================================*/
void
etimer_restart(struct etimer *pst_et)
{
    timer_restart(&pst_et->timer);
        _etimer_addTimer(pst_et);
      LOG_INFO("restart timer %p\n\r",pst_et);
}/* etimer_restart() */

/*============================================================================*/
/*  etimer_adjust()                                                                 */
/*============================================================================*/
void etimer_adjust(struct etimer *pst_et, int32_t l_timediff)
{
    pst_et->timer.start += l_timediff;
}/* etimer_adjust() */

/*============================================================================*/
/*  etimer_expired()                                                                 */
/*============================================================================*/
int etimer_expired(struct etimer *pst_et)
{
  return ( pst_et->active == TMR_NOT_ACTIVE );
}
/*============================================================================*/
/*  etimer_expiration_time()                                                                 */
/*============================================================================*/
clock_time_t etimer_expiration_time(struct etimer *pst_et)
{
  return pst_et->timer.start + pst_et->timer.interval;
} /* etimer_expiration_time() */

/*============================================================================*/
/*  etimer_start_time()                                                                 */
/*============================================================================*/
clock_time_t etimer_start_time(struct etimer *pst_et)
{
  return pst_et->timer.start;
}/* etimer_start_time() */

/*============================================================================*/
/*  etimer_pending()                                                                 */
/*============================================================================*/
int etimer_pending(void)
{
  return gp_etimList != NULL;
}/* etimer_pending() */

/*============================================================================*/
/*  etimer_next_expiration_time()                                                                 */
/*============================================================================*/
clock_time_t etimer_next_expiration_time(void)
{
  return etimer_pending() ? l_nextExp : 0;
} /* etimer_next_expiration_time() */

/*============================================================================*/
/*  etimer_stop()                                                                 */
/*============================================================================*/
void etimer_stop(struct etimer *pst_et)
{
    list_remove(gp_etimList, pst_et);
    pst_et->active = TMR_NOT_ACTIVE;
} /* etimer_stop() */

/*============================================================================*/
/*  etimer_nextEvent()                                                */
/*============================================================================*/
clock_time_t etimer_nextEvent(void)
{
  struct    etimer  *   pst_tTim = list_head(gp_etimList);
  struct    etimer  *   pst_nTim = NULL; ///! Next timer
  clock_time_t  ct_expTime;
  clock_time_t  ct_nextExpTime = 0;

  if ( list_length(gp_etimList) == 0 ) /* no items in list */
    return TMR_NOT_ACTIVE;
  else /* search timer with next expiration time */
  {
    while(pst_tTim != NULL)
    {
      pst_nTim = list_item_next(pst_tTim); /* get next element in list */

      if (pst_tTim->active == TMR_ACTIVE) /* timer enabled? */
      {
        ct_expTime = pst_tTim->timer.start + pst_tTim->timer.interval; /* get timer expiration time */

        if ( ct_nextExpTime == 0)
          ct_nextExpTime = ct_expTime;
        else
        {
          if (ct_expTime < ct_nextExpTime) /* save next expiration time */
            ct_nextExpTime = ct_expTime;
        }
      } /* timer enabled? */

      pst_tTim = pst_nTim;
    }
      return (ct_nextExpTime);
  }
} /* etimer_nextEvent() */

/** @} */
