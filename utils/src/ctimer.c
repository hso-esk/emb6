/**
 *   \addtogroup ctimer Callback timer library
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
    \file   ctimer.c

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
#include "emb6.h"
#include "emb6_conf.h"

#include "bsp.h"
#include "evproc.h"
#include "ctimer.h"
#include "timer.h"
#include "clist.h"

/*==============================================================================
                             LOCAL MACROS
==============================================================================*/
#define     LOGGER_ENABLE        LOGGER_CTIMER
#if            LOGGER_ENABLE     ==     TRUE
#define        LOGGER_SUBSYSTEM    "ctim"
#endif
#include    "logger.h"

/*==============================================================================
                            LOCAL VARIABLES
==============================================================================*/
LIST(gp_ctimList);
static     char         gc_init = 0;

void ctimer_refresh(c_event_t event, void * data);

/*==============================================================================
                             LOCAL FUNCTIONS
==============================================================================*/
/**
 * \brief      Refresh a callback timer list
 * \param event        New event
 * \param data        Pointer to the data
 *
 * \sa ctimer_refresh()
 */
void ctimer_refresh(c_event_t event, void * data)
{
    struct ctimer *pst_cTim;
    for(pst_cTim = list_head(gp_ctimList); \
        pst_cTim != NULL; \
        pst_cTim = pst_cTim->next) {
      if(&pst_cTim->etimer == data) {
          list_remove(gp_ctimList, pst_cTim);
          if(pst_cTim->f != NULL) {
              pst_cTim->f(pst_cTim->ptr);
          }
          break;
      }
    }
}
/*==============================================================================
                             GLOBAL FUNCTIONS
==============================================================================*/
/*============================================================================*/
/*  ctimer_init()                                                     */
/*============================================================================*/
void ctimer_init(void)
{
    if (gc_init)
        return;
    etimer_init();
    struct ctimer *c;
    list_init(gp_ctimList);
    for(c = list_head(gp_ctimList); c != NULL; c = c->next) {
        etimer_set(&c->etimer, c->etimer.timer.interval, ctimer_refresh);
    }
    gc_init = 1;
}
/*============================================================================*/
/*  ctimer_set()                                                     */
/*============================================================================*/
void ctimer_set(struct ctimer *c, clock_time_t t,
       void (*f)(void *), void *ptr)
{
    LOG_INFO("ctimer_set %p %u", c, (unsigned)t);
    c->f = f;
    c->ptr = ptr;
    if(gc_init) {
        etimer_set(&c->etimer, t, ctimer_refresh);
    } else {
        c->etimer.timer.interval = t;
    }

    list_add(gp_ctimList, c);
}
/*============================================================================*/
/*  ctimer_reset()                                                     */
/*============================================================================*/
void ctimer_reset(struct ctimer *c)
{
  if(gc_init) {
    etimer_reset(&c->etimer);
  }

  list_remove(gp_ctimList, c);
  list_add(gp_ctimList, c);
}
/*============================================================================*/
/*  ctimer_restart()                                                     */
/*============================================================================*/
void ctimer_restart(struct ctimer *c)
{
  if(gc_init) {
    etimer_restart(&c->etimer);
  }

  list_remove(gp_ctimList, c);
  list_add(gp_ctimList, c);
}
/*============================================================================*/
/*  ctimer_stop()                                                     */
/*============================================================================*/
void ctimer_stop(struct ctimer *pst_stopTim)
{
    if(gc_init) {
        etimer_stop(&pst_stopTim->etimer);
    } else {
        pst_stopTim->etimer.next = NULL;
        pst_stopTim->etimer.active = TMR_NOT_ACTIVE;
    }
    list_remove(gp_ctimList, pst_stopTim);
}
/*============================================================================*/
/*  ctimer_expired()                                                     */
/*============================================================================*/
int ctimer_expired(struct ctimer *pst_checkTim)
{
    struct ctimer *pst_t;
    if(gc_init) {
        return etimer_expired(&pst_checkTim->etimer);
    }
    for(pst_t = list_head(gp_ctimList); pst_t != NULL; pst_t = pst_t->next) {
        if(pst_t == pst_checkTim) {
            return 0;
        }
    }
    return 1;
}
/** @} */
