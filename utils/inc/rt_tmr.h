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
 * @file    lib_tmr.h
 * @author  PN
 * @brief   Timer management module
 */

#ifndef RT_TMR_PRESENT
#define RT_TMR_PRESENT

#ifdef RT_TMR_MODULE
#define RT_TMR_EXT
#else
#define RT_TMR_EXT     extern
#endif

#include <stdint.h>

/*
 ********************************************************************************
 *                           DATA TYPES DECLARATION
 ********************************************************************************
 */
typedef uint32_t rt_tmr_tick_t;
typedef uint32_t rt_tmr_qty_t;

typedef void (*pf_cb_void_t)(void*);


typedef enum {
  E_RT_TMR_TYPE_ONE_SHOT,
  E_RT_TMR_TYPE_PERIODIC
} e_rt_tmr_type_t;

typedef enum {
  E_RT_TMR_STATE_INIT,
  E_RT_TMR_STATE_CREATED,
  E_RT_TMR_STATE_RUNNING,
  E_RT_TMR_STATE_STOPPED,
  E_RT_TMR_STATE_TRIGGERED,
  E_RT_TMR_STATE_MAX,
} e_rt_tmr_state_t;

typedef struct s_rt_tmr s_rt_tmr_t;
struct s_rt_tmr {
  s_rt_tmr_t        *pnext;
  s_rt_tmr_t        *pprev;
  e_rt_tmr_type_t    type;
  rt_tmr_tick_t      period;
  rt_tmr_tick_t      counter;
  e_rt_tmr_state_t   state;
  pf_cb_void_t       cbFnct;
  void              *cbArg;
};

/*
 ********************************************************************************
 *                                   DEFINES
 ********************************************************************************
 */
#define RT_TMR_CFG_DEBUG_EN                                    ( 0u )

#define RT_TMR_CFG_TICK_FREQ_IN_HZ          (rt_tmr_tick_t )( 1000u )


/*
 ********************************************************************************
 *                           GLOBAL VARIABLES DECLARATION
 ********************************************************************************
 */
extern s_rt_tmr_t     *pTmrListHead;
extern s_rt_tmr_t     *pTmrListTail;
extern rt_tmr_tick_t    TmrCurTick;
extern rt_tmr_qty_t     TmrListQty;


/*
 ********************************************************************************
 *                           API FUNCTIONS DECLARATIONS
 ********************************************************************************
 */
void rt_tmr_init(void);
void rt_tmr_create(s_rt_tmr_t *p_tmr, e_rt_tmr_type_t type, rt_tmr_tick_t period, pf_cb_void_t fnct, void *p_cb);
void rt_tmr_stop(s_rt_tmr_t *p_tmr);
void rt_tmr_start(s_rt_tmr_t *p_tmr);
void rt_tmr_delay(rt_tmr_tick_t delay);
void rt_tmr_update(void);
rt_tmr_tick_t rt_tmr_getRemain(s_rt_tmr_t *p_tmr);
e_rt_tmr_state_t rt_tmr_getState(s_rt_tmr_t *p_tmr);

#endif /* RT_TMR_PRESENT */
