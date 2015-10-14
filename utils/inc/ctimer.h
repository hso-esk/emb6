#ifndef ALT_ctimer_H_
#define ALT_ctimer_H_
/**
 *   \addtogroup utils
 *   @{
*/
/**
 *   \defgroup ctimer Callback timer library
 *
 *   This is a set of functions to call particular function after etimer event was
 *   detected.
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
    \file   ctimer.h

    \author Artem Yushev 

    \brief  Functions to manage Contiki timers
            Contiki has two main type of timers callback timers (ctimer) and
            event timer (etimer)
            The first one push callbacks after timer expires and the second one
            push event when timer expires.

  \version  0.1
*/
/*============================================================================*/
/*=============================================================================
                                 INCLUDES
 =============================================================================*/
#include "etimer.h"

/*=============================================================================
                        STRUCTURES AND OTHER TYPEDEFS
 =============================================================================*/
struct ctimer {
  struct     ctimer         *next;
  struct     etimer         etimer;
              void             (*f)(void *);
              void             *ptr;
};

/*==============================================================================
                          FUNCTION PROTOTYPES
==============================================================================*/
/**
 * \brief      Reset a callback timer with the same interval as was
 *             previously set.
 * \param c    A pointer to the callback timer.
 *
 *             This function resets the callback timer with the same
 *             interval that was given to the callback timer with the
 *             ctimer_set() function. The start point of the interval
 *             is the exact time that the callback timer last
 *             expired. Therefore, this function will cause the timer
 *             to be stable over time, unlike the ctimer_restart()
 *             function.
 *
 * \sa ctimer_restart()
 */
void ctimer_reset(struct ctimer *c);

/**
 * \brief      Restart a callback timer from the current point in time
 * \param c    A pointer to the callback timer.
 *
 *             This function restarts the callback timer with the same
 *             interval that was given to the ctimer_set()
 *             function. The callback timer will start at the current
 *             time.
 *
 *             \note A periodic timer will drift if this function is
 *             used to reset it. For periodic timers, use the
 *             ctimer_reset() function instead.
 *
 * \sa ctimer_reset()
 */
void ctimer_restart(struct ctimer *c);

/**
 * \brief      Set a callback timer.
 * \param c    A pointer to the callback timer.
 * \param t    The interval before the timer expires.
 * \param f    A function to be called when the timer expires.
 * \param ptr  An opaque pointer that will be supplied as an argument to the callback function.
 *
 *             This function is used to set a callback timer for a time
 *             sometime in the future. When the callback timer expires,
 *             the callback function f will be called with ptr as argument.
 *
 */
void ctimer_set(struct ctimer *c, clock_time_t t,
        void (*f)(void *), void *ptr);

/**
 * \brief      Stop a pending callback timer.
 * \param c    A pointer to the pending callback timer.
 *
 *             This function stops a callback timer that has previously
 *             been set with ctimer_set(), ctimer_reset(), or ctimer_restart().
 *             After this function has been called, the callback timer will be
 *             expired and will not call the callback function.
 *
 */
void ctimer_stop(struct ctimer *c);

/**
 * \brief      Check if a callback timer has expired.
 * \param c    A pointer to the callback timer
 * \return     Non-zero if the timer has expired, zero otherwise.
 *
 *             This function tests if a callback timer has expired and
 *             returns true or false depending on its status.
 */
int ctimer_expired(struct ctimer *c);

/**
 * \brief      Initialize the callback timer library.
 *
 *             This function initializes the callback timer library and
 *             should be called from the system boot up code.
 */
void ctimer_init(void);




#endif /* ALT_ctimer_H_ */
 /** @} */
/** @} */
