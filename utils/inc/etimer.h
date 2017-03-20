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
 *  \file       etimer.h
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
#ifndef __ETIMER_H__
#define __ETIMER_H__


/*
 *  --- Includes -------------------------------------------------------------*
 */
#include <stdint.h>
#include "timer.h"
#include "evproc.h"


/*
 * --- Macro Definitions --------------------------------------------------- *
 */

/** Timer is not active */
#define TMR_ACTIVE                1
/** Timer is active */
#define TMR_NOT_ACTIVE            0


/*
 *  --- Type Definitions -----------------------------------------------------*
 */

/**
 * \brief   Structure of an etimer.
 *
 *          The etimer structure is used e.g. to access a specific event
 *          timer e.g. to stop or to reset.
 */
struct etimer
{
    /** Pointer to the next etimer structure in the list. */
    struct etimer* next;

    /** Structure to store timer information. */
    struct timer timer;

    /** Flag indicating whether etimer has expired or not. */
    uint8_t active;
};


/*
 *  --- Global Functions Definition ------------------------------------------*
 */

/**
 * etimer_init()
 *
 * \brief      Initialize the event timer module.
 *
 *             This function initializes the event timer module and
 *             should be called from the system boot up code.
 */
void etimer_init( void );


/**
 * etimer_request_poll()
 *
 * \brief   Make the event timer aware that the clock has changed
 *
 *          This function is used to inform the event timer module
 *          that the system clock has been updated. Typically, this
 *          function would be called from the timer interrupt
 *          handler when the clock has ticked.
 */
void etimer_request_poll(void);


/**
 * etimer_set()
 *
 * \brief   Set an event timer.
 *
 *          This function is used to set an event timer for a time
 *          sometime in the future. When the event timer expires,
 *          an according event will be posted to the and the associated
 *          callback will be called.
 *
 * \param   et          Pointer to the event timer.
 * \param   interval    The interval before the timer expires.
 * \param   callback    Callback function to call if the event was fired.
 *
 */
void etimer_set( struct etimer* et, clock_time_t interval,
        pfn_callback_t callback);


/**
 * etimer_stop()
 *
 * \brief   Stop a pending event timer.
 *
 *          This function stops an event timer that has previously
 *          been set with etimer_set() or etimer_reset(). After
 *          this function has been called, the event timer will not
 *          emit any event when it expires.
 *
 * \param   et   Pointer to the pending event timer to stop.
 *
 */
void etimer_stop(struct etimer *et);


/**
 * etimer_reset()
 *
 * \brief   Reset an event timer with its initial interval.
 *
  *         This function resets the event timer with the same
 *          interval that was given to the event timer with the
 *          etimer_set() function. The start point of the interval
 *          is the exact time that the event timer last
 *          expired. Therefore, this function will cause the timer
 *          to be stable over time, unlike the etimer_restart()
 *          function.
 *
 * \param   et      A pointer to the event timer to reset.
 */
void etimer_reset( struct etimer* et );


/**
 * etimer_restart()
 *
 * \brief   Restart an event timer from the current point in time.
 *
 *          This function restarts the event timer with the same
 *          interval that was given to the etimer_set()
 *          function. The event timer will start at the current
 *          time.
 *          The periodic timer will drift if this function is
 *          used to reset it. For periodic timers, use the
 *          etimer_reset() function instead.
 *
 * \param   et   Pointer to the event timer to restart.
 *
 */
void etimer_restart( struct etimer* et );


/**
 * etimer_adjust()
 *
 * \brief   Adjust the expiration time for an event timer
 *
 *          This function is used to adjust the time the event
 *          timer will expire. It can be used to synchronize
 *          periodic timers without the need to restart the timer
 *          or change the timer interval.
 *          This function should only be used for small
 *          adjustments. For large adjustments use etimer_set()
 *          instead.
 *          A periodic timer will drift unless the
 *          etimer_reset() function is used.
 *
 * \param   et      Pointer to the event timer to adjust.
 * \param   td      Time difference to adjust the expiration time with.
 *
 */
void etimer_adjust( struct etimer* et, int32_t td );


/**
 * etimer_expired()
 *
 * \brief   Check if an event timer has expired.
 *
 *          This function tests if an event timer has expired and
 *          returns true or false depending on its status.
 *
 * \param   et  Pointer to the event timer to check.
 *
 * \return  Non-zero if the timer has expired, zero otherwise.
 *
 */
int etimer_expired( struct etimer *et );


/**
 * etimer_expiration_time()
 *
 * \brief   Get the expiration time for the event timer.
 *
 * \param   et  Pointer to the event timer to get the expiration time from.
 *
 * \return  The expiration time for the event timer.
 *
 */
clock_time_t etimer_expiration_time( struct etimer* et );


/**
 * etimer_start_time()
 *
 * \brief   Get the start time for the event timer.
 *
 *          This function returns the start time (when the timer
 *          was last set) for an event timer.
 *
 * \param   et  Pointer to the event timer to get the start time from.
 *
 * \return  The start time for the event timer. *
 */
clock_time_t etimer_start_time( struct etimer* et );


/**
 * etimer_request_poll()
 *
 * \brief   Get next event.
 *
 *             This functions returns next expiration time of all
 *             pending event timers. In case no event is pending,
 *             zero will be returned.
 *
 * \return     Next expiration time. If there are no pending event
 *             timers this function returns zero.
 */
clock_time_t etimer_nextEvent( void );


#endif /* __ETIMER_H__ */

