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
 *   \addtogroup evproc Event processing library
 *   @{
*/
/*!
    \file   evproc.c

    \author Artem Yushev 

    \brief  Functions for event driver engine. A substitutional lib between
            Contiki engine and new architecture.

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
#include "clist.h"
#include "logger.h"

/*==============================================================================
                             LOCAL MACROS
==============================================================================*/
//! Enable or disable logging
#define     LOGGER_ENABLE        LOGGER_EVPROC


/*==============================================================================
                             LOCAL CONSTANTS
==============================================================================*/

/*==============================================================================
                    LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==============================================================================*/

/*!
 * \struct st_funcReg_t
 *
 * \brief Type of a structure to store every callback for particular event.
 * */
typedef struct {
    c_event_t        c_event; ///< Event type
    pfn_callback_t     pfn_callbList[MAX_CALLBACK_COUNT]; ///< List of a callback functions
}st_funcReg_t;

/*!
 * \struct st_eventDisc_t
 *
 * \brief Type of a structure to store particular event linked with a data
 * */
typedef struct {
    p_data_t            p_data;  ///< Pointer to a data to be transfered
    c_event_t            c_event; ///< Event type
}st_eventDisc_t;

/*==============================================================================
                            LOCAL VARIABLES
==============================================================================*/
/*! Array of functions linked with every defined event */
static    st_funcReg_t    pst_regList[EVENT_TYPES_COUNT];

/*!  Queue of events linked with a data which is associated with this event */
static st_eventDisc_t    pst_evList[EVPROC_QUEUE_SIZE];

/*! Flag to detects initialization status of a evproc library */
static uint8_t c_isInit = 0;

/*!  Queue size. Should be not more than  \ref EVPROC_QUEUE_SIZE */
static uint8_t    c_queueSize = 0;
/*==============================================================================
                             LOCAL PROTOTYPES
==============================================================================*/
static    void                 _evproc_init(void);
static    en_evprocResCode_t    _evproc_pushEvent(c_event_t c_event_type, p_data_t data);
/*==============================================================================
                             LOCAL FUNCTIONS
==============================================================================*/
/*============================================================================*/
/*!
   \brief   Initialization of the event processing lib

            This function define all the fields in pst_regList by
            zero or NULL
*/
/*============================================================================*/
void _evproc_init(void)
{
    uint8_t    i,j;
    // Initialize variable by predefined event types macro
    uint8_t pc_eventTypes[EVENT_TYPES_COUNT] = EVENT_TYPES;

    // Nullify event queue
    for (i = 0;i < EVPROC_QUEUE_SIZE; i++) {
        pst_evList[i].c_event = EVENT_TYPE_NONE;
        pst_evList[i].p_data = NULL;
    }

    // Assign every callback for every event by NULL pointer
    for(i=0; i<EVENT_TYPES_COUNT; i++)
    {
        // Set "i" event from the list by the one from eventTypes array
        pst_regList[i].c_event = pc_eventTypes[i];
        for(j=0;j<MAX_CALLBACK_COUNT;j++)
        {
            // Assign "j" callback from the list to NULL pointer
            pst_regList[i].pfn_callbList[j] = NULL;
        } /* for */
    } /* for */

    // Set init flag to prevent erasing list again
    c_isInit = 1;
} /* _evl_init() */

/*============================================================================*/
/*!
    \brief    Process input event

            Call registered on input event callback functions one-by-one

    \param    c_event_type        Type of event on which we want to register callback
    \param    callback        Pointer on the function which should be called
                            whenever c_event_type was generated

    \retval    E_UNKNOWN_TYPE        Unknown event type
    \retval    E_SUCCESS            All callbacks has been called

*/
/*============================================================================*/
en_evprocResCode_t    _evproc_pushEvent(c_event_t c_eventType, p_data_t p_data)
{
    uint8_t i,j;
    // Search for a given event type and try to add given function pointer
    // to the end of a registration list
    for(i=0; i<EVENT_TYPES_COUNT; i++)
    {
        // Find given c_eventType in registered list. If no event matches
        // then generate error code
        if(pst_regList[i].c_event == c_eventType)
        {
            // Event type has been found, now search for registered callbacks
            for (j = 0; j < MAX_CALLBACK_COUNT; j++)
            {
                // If points to a NULL pointer the end of a list has been reached
                if (pst_regList[i].pfn_callbList[j] == NULL)
                    return E_SUCCESS;
                // In every other cases callback should be called
                else {
                    pst_regList[i].pfn_callbList[j](c_eventType, p_data);
                }
            } /* for */
            return E_SUCCESS;
        } /* if */
    } /* for */

    LOG_ERR("unknown type (0x%04X)\n\r", c_eventType);
    return E_UNKNOWN_TYPE;
}

uint8_t    _evproc_lookupEvent(c_event_t c_eventType,p_data_t     p_data)
{
    uint8_t i;
    if (c_queueSize != 0) {
        for (i=0; i<c_queueSize;i++) {
            if ((pst_evList[i].c_event == c_eventType) && \
                (pst_evList[i].p_data == p_data))
                return 1;
        }
    }
    return 0;
}


/*==============================================================================
                             API FUNCTIONS
==============================================================================*/

/*============================================================================*/
/*  evproc_regCallback()                                                      */
/*============================================================================*/
en_evprocResCode_t evproc_regCallback(c_event_t c_eventType, pfn_callback_t pfn_callback)
{
    uint8_t i,j;

    // If event process wasn't initialized before do it now.
    if (!c_isInit)
        _evproc_init();

    // If no callback was given return with error code
    if (pfn_callback == NULL) {
        LOG_ERR("%s\n\r","NULL pointer function was given as a parameter.");
        return E_INVALID_PARAM;
    } /* if */

    // Search for a given event type and try to add given function pointer
    // to the end of a registration list
    for(i=0; i<EVENT_TYPES_COUNT; i++)
    {
        // Find given c_eventType in registered list. If no event matches
        // then generate error code
        if(pst_regList[i].c_event == c_eventType)
        {
            // Event type has been found, now search for registered callbacks
            for(j=0; j<MAX_CALLBACK_COUNT; j++)
            {
                // If "j" callback from registration list points to NULL
                // it means that end of a list has been reached.
                if(pst_regList[i].pfn_callbList[j] == NULL) {
                    // Given function pointer for a given event type has been added
                    LOG_INFO("new callback for (%d) event with callback %p\n\r", c_eventType, pfn_callback);
                    pst_regList[i].pfn_callbList[j] = pfn_callback;
                    return E_SUCCESS;
                } /* if */

                // If "j" callback is equal to a given function pointer
                // it means that this function for a given event has been
                // registered yet.
                if(pst_regList[i].pfn_callbList[j] == pfn_callback) {
                    LOG_ERR(" (%d) already registered\n\r",c_eventType);
                    return E_FUNC_IN_LIST;
                } /* if */

            } /* for */

            // If end of the registration list has been reached and
            // c_callbI index are equal to a maximum allowed callbacks
            // generate an error.
            LOG_ERR("limit is reached (%d)\n\r", MAX_CALLBACK_COUNT);
            return E_END_OF_LIST;
        } /* if */
    } /* for */

    LOG_ERR("unknown event type (0x%02X)\n\r", c_eventType);
    return E_UNKNOWN_TYPE;
}

/*============================================================================*/
/*  evproc_unregCallback()                                                         */
/*============================================================================*/
en_evprocResCode_t evproc_unregCallback(c_event_t c_eventType, pfn_callback_t pfn_callback)
{
    uint8_t i,j;

    // Search for a given event type and try to add given function pointer
    // to the end of a registration list
    for(i=0; i<EVENT_TYPES_COUNT; i++)
    {
        // Find given c_eventType in registered list. If no event matches
        // then generate error code
        if(pst_regList[i].c_event == c_eventType)
        {
            // Event type has been found, now search for registered callbacks
            for(j=0; j<MAX_CALLBACK_COUNT; j++)
            {
                // If "j" callback is equal to a given function pointer
                // it means that this function for a given event has been
                // found.
                if(pst_regList[i].pfn_callbList[j] == pfn_callback) {
                    pst_regList[i].pfn_callbList[j] = NULL;
                    return E_SUCCESS;
                } /* if */
            } /* for */

            LOG_ERR("%s\n\r","function wasn't registered");
            return E_NO_SUCH_FUNC;
        } /* if */
    } /* for */

    LOG_ERR(" unknown event type (0x%04X)\n\r", c_eventType);
    return E_UNKNOWN_TYPE;
}

/*============================================================================*/
/*  evproc_putEvent()                                                         */
/*============================================================================*/
en_evprocResCode_t evproc_putEvent(        en_evprocAction_t     e_actType, \
                                        c_event_t             c_eventType, \
                                        p_data_t             p_data)
{
    int8_t i;
    bsp_enterCritical();
    switch (e_actType)
    {
        case  E_EVPROC_HEAD:
            if (c_queueSize == EVPROC_QUEUE_SIZE) {
            //    printf("Not enogh space in a queue\n\r");
                bsp_exitCritical();
                return E_END_OF_LIST;
            }
            if ((c_eventType < OBLIG_EVENT_PRIOR) && (_evproc_lookupEvent(c_eventType,p_data))) {
                // Event has low priority and already in a queue
            }
            else {
                LOG_INFO("head %d : %p\n\r",c_eventType,p_data);
                pst_evList[c_queueSize].c_event = c_eventType;
                pst_evList[c_queueSize].p_data = p_data;
                c_queueSize++;
            }
            bsp_exitCritical();
            break;
        case  E_EVPROC_TAIL:
            if (c_queueSize == EVPROC_QUEUE_SIZE) {
            //    printf("Not enough space in a queue\n\r");
                bsp_exitCritical();
                return E_END_OF_LIST;
            }
            if ((c_eventType < OBLIG_EVENT_PRIOR) && (_evproc_lookupEvent(c_eventType,p_data))) {
                // Event has low priority and already in a queue
            }
            else {

                for (i=c_queueSize; i > 0; i--) {
                    pst_evList[i].c_event = pst_evList[i - 1].c_event;
                    pst_evList[i].p_data = pst_evList[i - 1].p_data;
                    //memcpy((&pst_evList + i),(&pst_evList + (i-1)),sizeof(st_eventDisc_t));
                }
                LOG_INFO("tail %d : %p\n\r",c_eventType,p_data);
                pst_evList[0].c_event = c_eventType;
                pst_evList[0].p_data = p_data;
                c_queueSize++;
            }
            bsp_exitCritical();
            break;
        case  E_EVPROC_EXEC:
            LOG_INFO("Execute event %d\n\r",c_eventType);
            bsp_exitCritical();
            if (!_evproc_pushEvent(c_eventType,p_data)) {
                return E_UNKNOWN_TYPE;
            }
            break;
        default:
            LOG_INFO("%s","Not known\n\r");
            bsp_exitCritical();
            return E_UNKNOWN_TYPE;
    }
    return E_SUCCESS;
} /* evproc_putEvent() */


/*============================================================================*/
/*  evproc_nextEvent()                                                         */
/*============================================================================*/
en_evprocResCode_t evproc_nextEvent(void)
{
    st_eventDisc_t nextEvent = {NULL,0};
    uint8_t i;
    if (c_queueSize > 0) {
        bsp_enterCritical();
        LOG_INFO("%s\n\r","Event queue");
        for (i=0; i<c_queueSize;i++)
            LOG_RAW("%d | ev = %d : %p\n\r",i,pst_evList[i].c_event,pst_evList[i].p_data);
        nextEvent.c_event = pst_evList[c_queueSize-1].c_event;
        nextEvent.p_data = pst_evList[c_queueSize-1].p_data;
        pst_evList[c_queueSize-1].c_event = 0;
        pst_evList[c_queueSize-1].p_data = NULL;
        c_queueSize--;
        bsp_exitCritical();
        if (!_evproc_pushEvent(nextEvent.c_event,nextEvent.p_data)) {
            return E_UNKNOWN_TYPE;
        }
        return E_SUCCESS;
    }
    return E_QUEUE_EMPTY;
} /* evproc_nextEvent() */

/** @} */
