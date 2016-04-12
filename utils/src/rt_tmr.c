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

/**
 * @file    rt_tmr.c
 * @author  Phuong Nguyen
 * @brief   Timer management module
 */


#include <string.h>

#define RT_TMR_MODULE
#include "rt_tmr.h"
#include "bsp.h"

s_rt_tmr_t *pTmrListHead;
s_rt_tmr_t *pTmrListTail;
rt_tmr_qty_t TmrListQty;
rt_tmr_tick_t TmrCurTick;


static void rt_tmr_link(s_rt_tmr_t *p_new);
static void rt_tmr_unlink(s_rt_tmr_t *p_rem);

/**
 * @brief   Add a timer to the double-linked list of timers.
 * @param   p_tmr   Point to timer to add
 */
static void rt_tmr_link(s_rt_tmr_t *p_new)
{
  rt_tmr_qty_t ix;
  s_rt_tmr_t *p_found;


  if (pTmrListHead == (s_rt_tmr_t*) 0) {
    pTmrListHead = p_new;
    pTmrListTail = p_new;
  } else {
    p_found = pTmrListHead;
    for (ix = 0; ix < TmrListQty; ix++) {
      if (p_found->counter < p_new->counter) {
        p_found = p_found->pnext;
      } else {
        /* found insertion place */
        break;
      }
    }

    if (p_found == (s_rt_tmr_t *) 0) {
      /* new timer is linked at tail of timer list: tail <-> new -> NULL */
      pTmrListTail->pnext = p_new;
      p_new->pprev = pTmrListTail;
      pTmrListTail = p_new;

    } else {
      if (p_found == pTmrListHead) {
        /* new timer is linked at head of timer list: head -> new <-> found */
        p_new->pnext = pTmrListHead;
        pTmrListHead->pprev = p_new;
        pTmrListHead = p_new;

      } else {
        /* link new timer to left of the found timer, middle of timer list */
        (p_found->pprev)->pnext = p_new;
        p_new->pprev = p_found->pprev;
        p_new->pnext = p_found;
        p_found->pprev = p_new;
      }
    }
  }

  /* increase number of linked timers */
  TmrListQty++;
}

/**
 * @brief   Remove a timer from the double-linked list of timers
 * @param   p_tmr   Point to timer to remove
 */
static void rt_tmr_unlink(s_rt_tmr_t *p_rem)
{
  if (p_rem == pTmrListHead) {
    /* unlink at head: head -> (rem->pnext) */
    if (p_rem->pnext == ((s_rt_tmr_t *)0)) {
      /* head -> rem -> NULL */
      pTmrListHead = (s_rt_tmr_t *)0;
      pTmrListTail = (s_rt_tmr_t *)0;

    } else {
      /* head -> rem -> tmrX -> ... */
      pTmrListHead = p_rem->pnext;
      (p_rem->pnext)->pprev = (s_rt_tmr_t *)0;

    }
  } else if (p_rem == pTmrListTail) {
    /* ... -> tmrX <-> rem -> NULL */
    pTmrListTail = p_rem->pprev;
    (p_rem->pprev)->pnext = (s_rt_tmr_t *)0;

  } else {
    /* ... -> tmrX <-> rem <-> tmrY -> ... */
    (p_rem->pprev)->pnext = p_rem->pnext;
    (p_rem->pnext)->pprev = p_rem->pprev;

  }

  p_rem->pnext = (s_rt_tmr_t *)0;
  p_rem->pprev = (s_rt_tmr_t *)0;
  TmrListQty--;
}

/**
 * @brief Initialize timer management module
 */
void rt_tmr_init(void)
{
  pTmrListHead = (s_rt_tmr_t *) 0;
  pTmrListTail = (s_rt_tmr_t *) 0;
  TmrCurTick = 0;
  TmrListQty = 0;
}

/**
 * @brief   Create a timer. The timer to be created should be set to 0 before
 *          calling Tmr_Create function.
 * @param   p_tmr   Point to timer to create
 * @param   type    Type of timer: periodic or one_shot
 * @param   period  Timer counter
 * @param   fnct    Callback function upon the timer interrupt presence
 * @param   p_cbarg Callback function argument
 */
void rt_tmr_create(s_rt_tmr_t *p_tmr, e_rt_tmr_type_t type, rt_tmr_tick_t period, pf_cb_void_t pfnct, void *parg)
{
  if ((p_tmr  != (s_rt_tmr_t *) 0) &&
      (period != (rt_tmr_tick_t) 0) &&
      (p_tmr->state == E_RT_TMR_STATE_INIT)) {
    memset(p_tmr, 0, sizeof(s_rt_tmr_t));
    p_tmr->type = type;
    p_tmr->period = period;
    p_tmr->counter = 0;
    p_tmr->state = E_RT_TMR_STATE_CREATED;
    p_tmr->cbFnct = pfnct;
    p_tmr->cbArg = parg;
    p_tmr->pnext = (s_rt_tmr_t *) 0;
    p_tmr->pprev = (s_rt_tmr_t *) 0;
  }
}


/**
 * @brief   Start a timer. Afterwards the time is added to the double-linked
 *          list of timer.
 *
 * @param   p_tmr
 */
void rt_tmr_start(s_rt_tmr_t *p_tmr)
{
  bsp_enterCritical();
  if ((p_tmr->state == E_RT_TMR_STATE_CREATED) ||
      (p_tmr->state == E_RT_TMR_STATE_STOPPED)) {
    /* update new counter */
    p_tmr->counter = TmrCurTick + p_tmr->period;

    /* link timer to timer list */
    rt_tmr_link(p_tmr);

    /* change timer state to RUNNING */
    p_tmr->state = E_RT_TMR_STATE_RUNNING;
  }
  bsp_exitCritical();
}

/**
 * @brief   Stop a timer. Afterwards the timer is removed from the double-linked
 *          list of timer.
 *
 * @param   p_tmr
 */
void rt_tmr_stop(s_rt_tmr_t *p_tmr)
{
  bsp_enterCritical();
  if (p_tmr->state == E_RT_TMR_STATE_RUNNING) {
    /* timer is running and linked in timer list, then first have it unlinked */
    rt_tmr_unlink(p_tmr);
  }
  p_tmr->state = E_RT_TMR_STATE_STOPPED;
  bsp_exitCritical();
}

/**
 * @brief
 * @param   delay
 */
void rt_tmr_delay(rt_tmr_tick_t delay)
{
  rt_tmr_tick_t tick_end;

  if (delay) {
    tick_end = TmrCurTick + delay;

    while (tick_end > TmrCurTick) {
      /* Do nothing */
    }
  }
}

/**
 * @brief   Achieve remaining ticks of a timer until it interrupts.
 * @param   p_tmr   Point to the timer
 * @return
 */
rt_tmr_tick_t rt_tmr_getRemain(s_rt_tmr_t *p_tmr)
{
  return (p_tmr->counter - TmrCurTick);
}

/**
 * @brief   Achieve current operating state of a timer.
 * @param   p_tmr   Point to the timer
 */
e_rt_tmr_state_t rt_tmr_getState(s_rt_tmr_t *p_tmr)
{
  return p_tmr->state;
}

/**
 * @brief   Update the double-linked list of timers upon system clock interrupt
 *          event. This function should be called at rate of
 *          UTIL_TMR_TICK_FREQ_IN_HZ
 */
void rt_tmr_update(void)
{
  s_rt_tmr_t *p_tmr;

  /* update timer tick */
  TmrCurTick++;

  while (pTmrListHead != (s_rt_tmr_t *)0) {
    /* always check head timer */
    p_tmr = pTmrListHead;
    if (p_tmr->counter > TmrCurTick) {
      break;
    }

    p_tmr->state = E_RT_TMR_STATE_TRIGGERED;
    rt_tmr_unlink(p_tmr);

    if (p_tmr->cbFnct != (pf_cb_void_t )0) {
      p_tmr->cbFnct(p_tmr->cbArg);
    }

    if (p_tmr->type == E_RT_TMR_TYPE_PERIODIC) {
      p_tmr->counter = TmrCurTick + p_tmr->period;
      rt_tmr_link(p_tmr);
      p_tmr->state = E_RT_TMR_STATE_RUNNING;
    } else {
      p_tmr->state = E_RT_TMR_STATE_STOPPED;
    }
  }
}
