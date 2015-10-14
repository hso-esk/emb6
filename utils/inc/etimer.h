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
 *   \addtogroup utils
 *   @{
*/
/**
 *   \defgroup etimer Event timer library
 *
 *   This is a set of functions to generate particular event for all of the
 *   subscribed functions
 *   @{
*/
/*!
    \file   etimer.h

    \author Artem Yushev 

    \brief  Functions to manage Contiki timers
            Contiki has two main type of timers callback timers (ctimer) and
            event timer (etimer)
            The first one push callbacks after timer expires and the second one
            push event when timer expires.

  \version  0.1
*/
#ifndef __ETIMER_H_
#define __ETIMER_H_
/*============================================================================*/

/*=============================================================================
                                 INCLUDES
 =============================================================================*/
#include "timer.h"
#include "evproc.h"

/*=============================================================================
                                 MACROS
 =============================================================================*/

#define TMR_ACTIVE                1
#define TMR_NOT_ACTIVE            0

/*=============================================================================
                        STRUCTURES AND OTHER TYPEDEFS
 =============================================================================*/
struct etimer {
    struct     etimer     *next; /**<  Pointer to the next etimer structure in list */
    struct     timer     timer; /**<  Structure to store start timestamp and interval.*/
    uint8_t    active;/**<  Flag indicating either etimer has expired or not*/
};


/**
 * \brief      Set an event timer.
 * \param et   A pointer to the event timer
 * \param interval The interval before the timer expires.
 * \param callback callback function pointer
 *
 *             This function is used to set an event timer for a time
 *             sometime in the future. When the event timer expires,
 *             the event PROCESS_EVENT_TIMER will be posted to the
 *             process that called the etimer_set() function.
 *
 */
void etimer_set(struct etimer *et, clock_time_t interval, pfn_callback_t callback);

/**
 * \brief      Reset an event timer with the same interval as was
 *             previously set.
 * \param et   A pointer to the event timer.
 *
 *             This function resets the event timer with the same
 *             interval that was given to the event timer with the
 *             etimer_set() function. The start point of the interval
 *             is the exact time that the event timer last
 *             expired. Therefore, this function will cause the timer
 *             to be stable over time, unlike the etimer_restart()
 *             function.
 *
 * \sa etimer_restart()
 */
CCIF void etimer_reset(struct etimer *et);

/**
 * \brief      Restart an event timer from the current point in time
 * \param et   A pointer to the event timer.
 *
 *             This function restarts the event timer with the same
 *             interval that was given to the etimer_set()
 *             function. The event timer will start at the current
 *             time.
 *
 *             \note A periodic timer will drift if this function is
 *             used to reset it. For periodic timers, use the
 *             etimer_reset() function instead.
 *
 * \sa etimer_reset()
 */
void etimer_restart(struct etimer *et);

/**
 * \brief      Adjust the expiration time for an event timer
 * \param et   A pointer to the event timer.
 * \param td   The time difference to adjust the expiration time with.
 *
 *             This function is used to adjust the time the event
 *             timer will expire. It can be used to synchronize
 *             periodic timers without the need to restart the timer
 *             or change the timer interval.
 *
 *             \note This function should only be used for small
 *             adjustments. For large adjustments use etimer_set()
 *             instead.
 *
 *             \note A periodic timer will drift unless the
 *             etimer_reset() function is used.
 *
 * \sa etimer_set()
 * \sa etimer_reset()
 */
void etimer_adjust(struct etimer *et, int32_t td);

/**
 * \brief      Get the expiration time for the event timer.
 * \param et   A pointer to the event timer
 * \return     The expiration time for the event timer.
 *
 *             This function returns the expiration time for an event timer.
 */
clock_time_t etimer_expiration_time(struct etimer *et);

/**
 * \brief      Get the start time for the event timer.
 * \param et   A pointer to the event timer
 * \return     The start time for the event timer.
 *
 *             This function returns the start time (when the timer
 *             was last set) for an event timer.
 */
clock_time_t etimer_start_time(struct etimer *et);

/**
 * \brief      Check if an event timer has expired.
 * \param et   A pointer to the event timer
 * \return     Non-zero if the timer has expired, zero otherwise.
 *
 *             This function tests if an event timer has expired and
 *             returns true or false depending on its status.
 */
CCIF int etimer_expired(struct etimer *et);

/**
 * \brief      Stop a pending event timer.
 * \param et   A pointer to the pending event timer.
 *
 *             This function stops an event timer that has previously
 *             been set with etimer_set() or etimer_reset(). After
 *             this function has been called, the event timer will not
 *             emit any event when it expires.
 *
 */
void etimer_stop(struct etimer *et);

/*==============================================================================
                          FUNCTION PROTOTYPES
==============================================================================*/
/**
 * \brief      Initialize an event timer library.
 *
 *             This function initializes the event timer library and
 *             should be called from the system boot up code.
 */
void etimer_init(void);


/**
 * \name Functions called from timer interrupts, by the system
 * @{
 */
/**
 * \brief      Make the event timer aware that the clock has changed
 *
 *             This function is used to inform the event timer module
 *             that the system clock has been updated. Typically, this
 *             function would be called from the timer interrupt
 *             handler when the clock has ticked.
 */
void etimer_request_poll(void);

/**
 * \brief      Check if there are any non-expired event timers.
 * \return     True if there are active event timers, false if there are
 *             no active timers.
 *
 *             This function checks if there are any active event
 *             timers that have not expired.
 */
int etimer_pending(void);

/**
 * \brief      Get next event timer expiration time.
 * \return     Next expiration time of all pending event timers.
 *             If there are no pending event timers this function
 *           returns 0.
 *
 *             This functions returns next expiration time of all
 *             pending event timers.
 */
clock_time_t etimer_next_expiration_time(void);

/**
 * \brief      Get next event.
 * \return     Next expiration time.
 *             If there are no pending event timers this function
 *         returns 0.
 *
 *             This functions returns next expiration time of all
 *             pending event timers.
 */
clock_time_t etimer_nextEvent(void);



#endif /* ETIMER_H_ */
/** @} */
/** @} */
/** @} */
