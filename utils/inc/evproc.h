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
 * \addtogroup utils
 * @{
 */
/**
 * \defgroup evproc Event processing library
 *
 * The evproc library provides function for scheduling and executing
 * events. A substitutional lib between Contiki engine and new architecture.
 *
 * @{
 */
/*!
    \file   evproc.h

    \author Artem Yushev (artem.yushev@hs-offenburg.de)

  \version  0.2
*/
/*============================================================================*/
#ifndef EVPROC_H_
#define EVPROC_H_


/*=============================================================================
                                 INCLUDES
 =============================================================================*/
#include <stdint.h>


/*=============================================================================
                                 MACROS
 =============================================================================*/
/// Defines all of the event that functions can operate
#define EVENT_TYPES        {EVENT_TYPE_TIMER_EXP, \
                            EVENT_TYPE_TCP_POLL, \
                            EVENT_TYPE_UDP_POLL, \
                            EVENT_TYPE_PCK_INPUT, \
                            EVENT_TYPE_ICMP6, \
                            EVENT_TYPE_TCPIP, \
                            EVENT_TYPE_SLIP_POLL, \
                            EVENT_TYPE_PCK_LL }

#define EVENT_TYPE_NONE             0x00    ///< No event
#define EVENT_TYPE_TIMER_EXP        0x01    ///< Timer expired event
#define EVENT_TYPE_TCP_POLL         0x02    ///< TCP poll event
#define EVENT_TYPE_UDP_POLL         0x03    ///< UDP poll event
#define EVENT_TYPE_PCK_INPUT        0x04    ///< New packet in buffer event
#define EVENT_TYPE_ICMP6            0x05    ///< New icmp6 packet event
#define EVENT_TYPE_TCPIP            0x06    ///< New tcpip event
#define EVENT_TYPE_SLIP_POLL        0x07    ///< Process slip handler
#define OBLIG_EVENT_PRIOR           0x0a
#define EVENT_TYPE_PCK_LL           0x0a    ///< New low level packet received


#define EVENT_TYPES_COUNT           8       ///< Counter of events in /ref EVENT_TYPES macro
#define MAX_CALLBACK_COUNT          7       ///< Maximal amount of callbacks in /ref st_funcRegList_t list
#define EVPROC_QUEUE_SIZE           20      ///< Maximal amount of events in /ref pst_evList queue
/*=============================================================================
                                 ENUMS
 =============================================================================*/
/*!
 * \brief Result code for event processing library
 * */
typedef enum {
    E_UNKNOWN_TYPE   = -4,     ///< Unknown type
    E_NO_SUCH_FUNC   = -3,     ///< No such function in the list
    E_INVALID_PARAM  = -2,     ///< Invalid parameter
    E_END_OF_LIST    = -1,     ///< End of a list was reached
    E_SUCCESS        = 1,      ///< Success
    E_FUNC_IN_LIST   = 2,      ///< Function already in the list
    E_QUEUE_EMPTY    = 3       ///< No events in a queue
}en_evprocResCode_t;

/*!
 * \brief Different type of actions to work with events queue
 * */
typedef enum {
    E_EVPROC_HEAD,            ///< Put event into a head of a queue
    E_EVPROC_TAIL,            ///< Put event into a tail of a queue
    E_EVPROC_EXEC            ///< Call all subscribed functions immediately
}en_evprocAction_t;

/*=============================================================================
                        STRUCTURES AND OTHER TYPEDEFS
 =============================================================================*/
/*! Type of an event (unsigned char)*/
typedef unsigned char c_event_t;

/*! Type of a data transfered with events (void *) */
typedef void *    p_data_t;

/*! Type of a callback function */
typedef void         (*pfn_callback_t)( c_event_t c_event, p_data_t p_data );



/*==============================================================================
                          FUNCTION PROTOTYPES
==============================================================================*/
/*============================================================================*/
/*!
    \brief  Register new callback function for a particular event

            This function compare parameter function with existing callbacks
            in list. If it is present there nothing happens, if not it is added
            to the end of a list.

    \param  c_eventType            Type of event on which we want to register callback
    \param    pfn_callback        Pointer on the function which should be called
                                whenever c_event_type was generated

    \return    \ref E_UNKNOWN_TYPE        Unknown event type
    \return    \ref E_INVALID_PARAM    Invalid input parameter
    \return    \ref E_FUNC_IN_LIST        Callback already exist in the list for this type of
                                        an event
    \return \ref E_END_OF_LIST        End of a list has been reached
    \return \ref E_SUCCESS            Callback successfully registered

    \code
            // This function is called whenever mac layer has already processed
            // input data packet and trying to call upper layer handler
            tcpip_input(void)
            {
                // This function will call every callback function which has been
                // registered for this type of event. Unlike E_EVPROC_TAIL or
                // E_EVPROC_HEAD this function will immediately call all the
                // registered functions.
                evproc_putEvent(E_EVPROC_EXEC,EVENT_TYPE_PCK_INPUT,NULL);
            }

            // This function is handles events internally in tcpip layer
            static void eventhandler(c_event_t ev, p_data_t  data)
            {
                //...
                switch(ev) {
                    //...
                     case EVENT_TYPE_PCK_INPUT:
                          LOG_INFO("%s\n\r","Packet received!");
                     break;
                    //...
                }
                //...
            }

            // Register particular handler for particular event. You may register
            // Several different event type for one handler
            // (limited by EVENT_TYPES_COUNT macro). And several different
            // callbacks for one event type (limited by MAX_CALLBACK_COUNT macro)
            void tcpip_init(void)
            {
                //...
                evproc_regCallback(EVENT_TYPE_PCK_INPUT,eventhandler);
                //...
            }
    \endcode
*/
/*============================================================================*/
en_evprocResCode_t evproc_regCallback(c_event_t c_eventType, pfn_callback_t pfn_callback);

/*============================================================================*/
/*!
\brief   Unregister callback function for a particular event

         This function searching for a given function pointer in a registration
         list and delete it in case of success.

\param  c_evenType            Type of event on which we want to register callback
\param    pfn_callback        Pointer on the function which should be called
                            whenever c_event_type was generated

\return    \ref E_UNKNOWN_TYPE        Unknown event type
\return    \ref E_NO_SUCH_FUNC        Given function pointer wasn't found in a list
\return \ref E_SUCCESS            Callback successfully unregistered
*/
/*============================================================================*/
en_evprocResCode_t evproc_unregCallback(    c_event_t         c_evenType, \
                                            pfn_callback_t     pfn_callback);

/*============================================================================*/
/*!
\brief   Process input event in accordance with an action type.

         The event process route works with FIFO buffer with an ability to add
         new elements not only in a head, but also in a tail of a queue.

\param  e_actType            Action type - what should be done with this event.
                            See \ref en_evprocAction_t
\param  c_eventType            Type of event on which we want to register callback.
\param    p_data                Pointer on the function which should be called
                            whenever c_event_type was generated.

\return    \ref E_END_OF_LIST        End of a queue has been reached.
\return    \ref E_UNKNOWN_TYPE        Unknown type of an action.
\return \ref E_SUCCESS            Event was executed or added to the queue.
*/
/*============================================================================*/
en_evprocResCode_t evproc_putEvent(    en_evprocAction_t     e_actType, \
                                        c_event_t             c_eventType, \
                                        p_data_t             p_data);

/*============================================================================*/
/*!
\brief   Take next event from the queue compare with a registration list and
        call all of subscribers.

         This function takes event only from the head of a queue

\return    \ref E_QUEUE_EMPTY        No events in the queue.
\return    \ref E_UNKNOWN_TYPE        Input type of an event was not found in the
                                registration list.
\return \ref E_SUCCESS            Function executed.
*/
/*============================================================================*/
en_evprocResCode_t evproc_nextEvent(void);



#endif /* EVPROC_H_ */

/** @} */
/** @} */
