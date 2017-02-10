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
 *  \file       ctimer.h
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
#ifndef __CTIMER_H__
#define __CTIMER_H__


/*
 *  --- Includes -------------------------------------------------------------*
 */
#include <stdint.h>
#include "etimer.h"


/*
 *  --- Type Definitions -----------------------------------------------------*
 */

/** definition of a ctimer callback function */
typedef void (*fn_ctimer_cb_t)(void*);

/**
 * \brief   Structure of a ctimer.
 *
 *          The ctimer structure is used e.g. to access a specific callback
 *          timer e.g. to stop or to reset.
 */
struct ctimer
{
    /** Pointer to the next timer structure in the timer list */
    struct ctimer *next;

    /** An event timer that is used for the callback timer */
    struct etimer etimer;

    /** Callback function registered for the callback timer. */
    fn_ctimer_cb_t f;

    /** Additional data used as parameter for the registered callback */
    void* ptr;
};


/*
 *  --- Global Functions Definition ------------------------------------------*
 */


/**
 * ctimer_init()
 *
 * \brief   Initialize the callback timer library.
 *
 *          This function initializes the callback timer library and
 *          should be called from the system boot up code.
 */
void ctimer_init( void );


/**
 * \brief   Set a callback timer.
 *
 *          This function is used to set a callback timer for a time
 *          sometime in the future. When the callback timer expires,
 *          the callback function will be called with its argument.
 *
 * \param   c    Pointer to the callback timer to set.
 * \param   t    The interval before the timer expires.
 * \param   f    Function to be called when the timer expires.
 * \param   ptr  Opaque pointer that will be supplied as an argument to the callback function.
 *
 *
 */
void ctimer_set( struct ctimer* c, clock_time_t t,
        fn_ctimer_cb_t f, void* ptr );


/**
 * \brief   Stop a pending callback timer.
 *
 *          This function stops a callback timer that has previously
 *          been started. After this function has been called, the callback
 *          timer will be expired and will not call the callback function.
 *
 * \param   c    Pointer to the pending callback timer.
 *
 */
void ctimer_stop( struct ctimer* c );


/**
 * ctimer_reset()
 *
 * \brief   Reset a callback timer with its initial interval.
 *
 *          This function resets the callback timer with the same
 *          interval that was given to the callback timer with the
 *          ctimer_set() function. The start point of the interval
 *          is the exact time that the callback timer last
 *          expired. Therefore, this function will cause the timer
 *          to be stable over time, unlike the ctimer_restart()
 *          function.
 *
 * \param   c    Pointer to the callback timer to reset.
 *
 */
void ctimer_reset( struct ctimer *c );


/**
 * ctimer_restart()
 *
 * \brief   Restart a callback timer from the current point in time.
 *
 *          This function restarts the callback timer with the same
 *          interval that was given to the ctimer_set()
 *          function. The callback timer will start at the current
 *          time.
 *          The periodic timer will drift if this function is
 *          used to reset it. For periodic timers, use the
 *          ctimer_reset() function instead.
 *
 * \param   c    Pointer to the callback timer to restart.
 *
 */
void ctimer_restart( struct ctimer *c );


/**
 * \brief   Check if a callback timer has expired.
 *
 *          This function tests if a callback timer has expired and
 *             returns true or false depending on its status.
 *
 * \param   c    Pointer to the callback timer
 *
 * \return  Non-zero if the timer has expired, zero otherwise.
 *
 */
int ctimer_expired( struct ctimer* c);


#endif /* __CTIMER_H__ */

