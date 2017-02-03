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
 *  \file       evproc.h
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Event processing module for emb::6.
 *
 *              The evproc library provides function for scheduling and executing
 *              events. A substitutional lib between the Contiki engine and
 *              the emb::6 new architecture.
 */
#ifndef __EVPROC_H__
#define __EVPROC_H__


/*
 *  --- Includes -------------------------------------------------------------*
 */
#include <stdint.h>


/*
 * --- Macro Definitions --------------------------------------------------- *
 */

/** Defines all of the event that functions can operate */
#define EVENT_TYPES {             \
  EVENT_TYPE_TIMER_EXP,           \
  EVENT_TYPE_TCP_POLL,            \
  EVENT_TYPE_UDP_POLL,            \
  EVENT_TYPE_PCK_INPUT,           \
  EVENT_TYPE_ICMP6,               \
  EVENT_TYPE_TCPIP,               \
  EVENT_TYPE_SLIP_POLL,           \
  NETSTK_APP_EVENT_TX,            \
  NETSTK_MAC_ULE_EVENT,           \
  NETSTK_RF_EVENT,                \
  EVENT_TYPE_PCK_LL               \
}

/** No event */
#define EVENT_TYPE_NONE                     (  0U )
/** Timer expired event */
#define EVENT_TYPE_TIMER_EXP                (  1U )
/** CP poll event */
#define EVENT_TYPE_TCP_POLL                 (  2U )
/** UDP poll event */
#define EVENT_TYPE_UDP_POLL                 (  3U )
/** New packet in buffer event */
#define EVENT_TYPE_PCK_INPUT                (  4U )
/** New icmp6 packet event */
#define EVENT_TYPE_ICMP6                    (  5U )
/** New tcpip event */
#define EVENT_TYPE_TCPIP                    (  6U )
/**  Process slip handler */
#define EVENT_TYPE_SLIP_POLL                (  7U )

/*
 * New event defines
 */
#define NETSTK_APP_EVENT_TX                 (  8U )
#define NETSTK_MAC_ULE_EVENT                ( 10U )
#define NETSTK_RF_EVENT                     ( 11U )

#define OBLIG_EVENT_PRIOR                   ( 15U )
#define EVENT_TYPE_PCK_LL                   ( 15U )     ///< New low level packet received

#define EVENT_TYPES_COUNT                   ( 14U )     ///< Counter of events in /ref EVENT_TYPES macro

/** Maximum amount of callbacks allowed */
#define MAX_CALLBACK_COUNT                  ( 13U )

/** Maximum number of events that can be queued */
#define EVPROC_QUEUE_SIZE                   ( 20U )



/*
 *  --- Type Definitions -----------------------------------------------------*
 */

/**
 * \brief Result code for event processing library
 */
typedef enum
{
    /** Unknown type */
    E_UNKNOWN_TYPE = -4,

    /** No such function in the list */
    E_NO_SUCH_FUNC = -3,

    /** Invalid parameter */
    E_INVALID_PARAM = -2,

    /** End of a list was reached */
    E_END_OF_LIST = -1,

    /** Success */
    E_SUCCESS = 1,

    /** Function already in the list */
    E_FUNC_IN_LIST = 2,

    /** No events in a queue */
    E_QUEUE_EMPTY = 3

}en_evprocResCode_t;

/*!
 * \brief Different type of actions to work with events queue
 */
typedef enum
{
    /** Put event into a head of a queue */
    E_EVPROC_HEAD,

    /** Put event into a tail of a queue */
    E_EVPROC_TAIL,

    /** Call all subscribed functions immediately */
    E_EVPROC_EXEC

}en_evprocAction_t;


/** Type of an event (unsigned char)*/
typedef unsigned char c_event_t;

/** Type of a data transfered with events (void *) */
typedef void* p_data_t;

/** Type of a callback function */
typedef void (*pfn_callback_t)( c_event_t c_event, p_data_t p_data );



/*
 *  --- Global Functions Definition ------------------------------------------*
 */


/**
 * evproc_regCallback()
 *
 * \brief   Register new callback function for a particular event.
 *
 *          This function compare parameter function with existing callbacks
 *          in list. If it is present there nothing happens, if not it is added
 *          to the end of a list.
 *
 * \param   c_eventType     Type of event on which we want to register callback.
 * \param   pfn_callback    Pointer on the function which should be called.
 *
 * \return  Error code from according enumeration.
 */
en_evprocResCode_t evproc_regCallback(c_event_t c_eventType,
    pfn_callback_t pfn_callback );


/**
 * evproc_unregCallback()
 *
 * \brief   Unregister callback function for a particular event.
 *
 *          This function searching for a given function pointer in a
 *          registration list and delete it in case of success.
 *
 * \param   c_eventType     Type of event to deregister callback.
 * \param   pfn_callback    Pointer on the function that shall be deregistered.
 *
 * \return  Error code from according enumeration.
 */
en_evprocResCode_t evproc_unregCallback( c_event_t c_eventType,
    pfn_callback_t pfn_callback );


/**
 * evproc_putEvent()
 *
 * \brief   Process input event in accordance with an action type.
 *
 *          The event process route works with FIFO buffer with an ability
 *          to add new elements not only in a head, but also in a tail of
 *          a queue. Furthermore its possible to directly execute an event.
 *
 *
 * \param   e_actType     Action type what should be done with this event.
 * \param   c_eventType   Type of event to put/execute.
 * \param   p_data        Specific data that can be used in the callback.
 *
 *
 * \return  Error code from according enumeration.
 */
en_evprocResCode_t evproc_putEvent( en_evprocAction_t e_actType,
    c_event_t c_eventType, p_data_t p_data );


/**
 * evproc_nextEvent()
 *
 * \brief   Process next event from the event queue.
 *
 *          This function takes the next event from the queue and calls
 *          the registered callbacks.
 *
 * \return  Error code from according enumeration.
 */
en_evprocResCode_t evproc_nextEvent( void );

#endif /* __EVPROC_H__*/

