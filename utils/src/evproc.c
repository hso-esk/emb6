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
 *  \file       evproc.c
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

/*
 *  --- Includes -------------------------------------------------------------*
 */
#include "emb6.h"
#include "bsp.h"
#include "evproc.h"
#include "clist.h"
#include "logger.h"

#define LOGGER_ENABLE           LOGGER_EVPROC
#include "logger.h"



/*
 *  --- Type Definitions -----------------------------------------------------*
 */

/**
 * \brief   Type of a structure to store every callback for particular event.
 */
typedef struct
{
    /** event type */
    c_event_t c_event;

    /** list of associated callback functions */
    pfn_callback_t pfn_callbList[MAX_CALLBACK_COUNT];

} st_funcReg_t;


/**
 * \brief Type of a structure to store particular data linked with an event.
 */
typedef struct
{

    /** Event type */
    c_event_t c_event;

    /** Pointer to the event data */
    p_data_t p_data;

} st_eventDisc_t;

/*
 *  --- Local Variables ---------------------------------------------------- *
 */

/** Array of functions linked with every defined event */
static st_funcReg_t pst_regList[EVENT_TYPE_MAX];

/** Queue of events linked with a data which is associated with this event */
static st_eventDisc_t pst_evList[EVPROC_QUEUE_SIZE];

/** Flag to detect initialization status of the module */
static uint8_t c_isInit = 0;

/**  Queue size. */
static uint8_t c_queueSize = 0;


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */

/* Initialize the module. For further information please refer to
 * the function definition. */
static void _evproc_init( void );

/* Put new event to the queue. For further information please refer to
 * the function definition. */
static en_evprocResCode_t _evproc_processEvent( c_event_t c_event_type,
    p_data_t data );

/* Check the queue for an event and its associated data. For further
 * Information please refer to the function definition. */
static uint8_t _evproc_lookupEvent( c_event_t c_eventType, p_data_t p_data );



/*
 *  --- Local Functions ---------------------------------------------------- *
 */

/**
 * \brief   Initialize the event processing module.
 *
 *          This function initialized the event processing module. It resets
 *          the internal queue and the registered callbacks.
 */
static void _evproc_init( void )
{
    uint8_t i,j;

    /* Nullify event queue */
    for( i = 0; i < EVPROC_QUEUE_SIZE; i++ )
    {
        pst_evList[i].c_event = EVENT_TYPE_NONE;
        pst_evList[i].p_data = NULL;
    }

    /* Assign every callback for every event by NULL pointer */
    for( i = 0; i < EVENT_TYPE_MAX; i++ )
    {
        /* Set "i" event from the list by the one from eventTypes array */
        pst_regList[i].c_event = i;
        for( j=0; j<MAX_CALLBACK_COUNT; j++ )
        {
            /* Assign "j" callback from the list to NULL pointer */
            pst_regList[i].pfn_callbList[j] = NULL;
        }
    }

    /* Set init flag to prevent erasing list again */
    c_isInit = 1;
} /* _evl_init() */


/**
 * \brief   Process a specific event.
 *
 *          This function processes a specific event. Therefore all
 *          registered callback functions will be called one-by-one with
 *          the associated data.
 *
 * \param   c_eventType   Type of the event to process.
 * \param   p_data        Pointer to the data associated to the callback.
 *
 * \return  Error code from according enumeration.
 */
en_evprocResCode_t _evproc_processEvent(c_event_t c_eventType, p_data_t p_data)
{
    uint8_t i, j;

    /*
     * Search for a given event type and try to add given function pointer
     * to the end of a registration list
     */
    for( i = 0; i < EVENT_TYPE_MAX; i++ )
    {
        /*
         * Find given c_eventType in registered list. If no event matches
         * then generate error code
         */
        if( pst_regList[i].c_event == c_eventType )
        {
            /*
             * Event type has been found, now search for registered callback(s)
             */
            for( j = 0; j < MAX_CALLBACK_COUNT; j++ )
            {
                /*
                 * If points to a NULL pointer end of the list has been reached
                 */
                if( pst_regList[i].pfn_callbList[j] == NULL )
                {
                    return E_SUCCESS;
                }
                else
                {
                    /*
                     * In every other cases callback(s) should be called
                     */
                    pst_regList[i].pfn_callbList[j]( c_eventType, p_data );
                }
            }
            return E_SUCCESS;
        }
    }

    LOG_ERR("unknown type (0x%04X)\n\r", c_eventType);
    return E_UNKNOWN_TYPE;
}


/**
 * \brief   Find an event with its associated data.
 *
 *          This function runs through the event queue and checks if it
 *          contains a specific event with it's associated data.
 *
 * \param   c_eventType   Type of the event to check for.
 * \param   p_data        Associated data to check for.
 *
 * \return  0 if element was not found or 1 if it was found.
 */
uint8_t _evproc_lookupEvent( c_event_t c_eventType, p_data_t p_data )
{
    uint8_t i;
    if( c_queueSize != 0 )
    {
        for( i=0; i<c_queueSize;i++ )
        {
            if ((pst_evList[i].c_event == c_eventType) &&
                (pst_evList[i].p_data == p_data))
                return 1;
        }
    }
    return 0;
}


/*
 * --- Global Function Definitions ----------------------------------------- *
 */


/*---------------------------------------------------------------------------*/
/*
* evproc_init()
*/
void evproc_init( void )
{
  /* reset initialization flag */
  c_isInit = 0;

  /* reinitialize */
  _evproc_init();
}

/*---------------------------------------------------------------------------*/
/*
* evproc_regCallback()
*/
en_evprocResCode_t evproc_regCallback( c_event_t c_eventType,
    pfn_callback_t pfn_callback )
{
    uint8_t i,j;

    /* If event process wasn't initialized before do it now. */
    if (!c_isInit)
        _evproc_init();

    /* If no callback was given return with error code */
    if (pfn_callback == NULL)
    {
        LOG_ERR("%s\n\r","NULL pointer function was given as a parameter.");
        return E_INVALID_PARAM;
    }

    /* Search for a given event type and try to add given function pointer
     * to the end of a registration list */
    for( i = 0; i < EVENT_TYPE_MAX; i++ )
    {
        /* Find given c_eventType in registered list. If no event matches
         * then generate error code */
        if( pst_regList[i].c_event == c_eventType )
        {
            /* Event type has been found, now search for registered callbacks */
            for( j=0; j<MAX_CALLBACK_COUNT; j++ )
            {
                /* If "j" callback from registration list points to NULL
                 * it means that end of a list has been reached. */
                if( pst_regList[i].pfn_callbList[j] == NULL )
                {
                    /* Given function pointer for a given event type has been added */
                    LOG_INFO("new callback for (%d) event with callback %p\n\r", c_eventType, (void*)pfn_callback);
                    pst_regList[i].pfn_callbList[j] = pfn_callback;
                    return E_SUCCESS;
                }

                /* If "j" callback is equal to a given function pointer
                 * it means that this function for a given event has been
                 * registered yet. */
                if( pst_regList[i].pfn_callbList[j] == pfn_callback )
                {
                    LOG_ERR(" (%d) already registered\n\r",c_eventType);
                    return E_FUNC_IN_LIST;
                }
            }

            /* If end of the registration list has been reached and
             * c_callbI index are equal to a maximum allowed callbacks
             * generate an error. */
            LOG_ERR("limit is reached (%d)\n\r", MAX_CALLBACK_COUNT);
            return E_END_OF_LIST;
        }
    }

    LOG_ERR("unknown event type (0x%02X)\n\r", c_eventType);
    return E_UNKNOWN_TYPE;

} /* evproc_regCallback() */


/*---------------------------------------------------------------------------*/
/*
* evproc_unregCallback()
*/
en_evprocResCode_t evproc_unregCallback(c_event_t c_eventType, pfn_callback_t pfn_callback)
{
    uint8_t i,j;

    /* Search for a given event type and try to add given function pointer
     * to the end of a registration list */
    for( i = 0; i < EVENT_TYPE_MAX; i++ )
    {
        /* Find given c_eventType in registered list. If no event matches
         * then generate error code */
        if( pst_regList[i].c_event == c_eventType )
        {
            /* Event type has been found, now search for registered callbacks */
            for( j=0; j<MAX_CALLBACK_COUNT; j++ )
            {
                /* If "j" callback is equal to a given function pointer
                 * it means that this function for a given event has been
                 * found. */
                if( pst_regList[i].pfn_callbList[j] == pfn_callback )
                {
                    pst_regList[i].pfn_callbList[j] = NULL;
                    return E_SUCCESS;
                }
            }

            LOG_ERR("%s\n\r","function wasn't registered");
            return E_NO_SUCH_FUNC;
        }
    }

    LOG_ERR(" unknown event type (0x%04X)\n\r", c_eventType);
    return E_UNKNOWN_TYPE;

} /* evproc_unregCallback() */


/*---------------------------------------------------------------------------*/
/*
* evproc_putEvent()
*/
en_evprocResCode_t evproc_putEvent( en_evprocAction_t e_actType,
    c_event_t c_eventType, p_data_t p_data )
{
    int8_t i;
    bsp_enterCritical();

    switch (e_actType)
    {
        case  E_EVPROC_HEAD:
            if (c_queueSize == EVPROC_QUEUE_SIZE)
            {
                bsp_exitCritical();
                return E_END_OF_LIST;
            }

            if ((c_eventType < OBLIG_EVENT_PRIOR) && (_evproc_lookupEvent(c_eventType,p_data)))
            {
                // Event has low priority and already in a queue
            }
            else
            {
                LOG_INFO("head %d : %p\n\r",c_eventType,p_data);
                pst_evList[c_queueSize].c_event = c_eventType;
                pst_evList[c_queueSize].p_data = p_data;
                c_queueSize++;
            }
            bsp_exitCritical();
            break;

        case  E_EVPROC_TAIL:
            if (c_queueSize == EVPROC_QUEUE_SIZE)
            {
                bsp_exitCritical();
                return E_END_OF_LIST;
            }
            if ((c_eventType < OBLIG_EVENT_PRIOR) && (_evproc_lookupEvent(c_eventType,p_data))) {
                // Event has low priority and already in a queue
            }
            else {

                for (i=c_queueSize; i > 0; i--)
                {
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
            if (!_evproc_processEvent(c_eventType,p_data))
            {
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


/*---------------------------------------------------------------------------*/
/*
* evproc_nextEvent()
*/
en_evprocResCode_t evproc_nextEvent(void)
{
    st_eventDisc_t nextEvent = { 0, NULL };
    uint8_t i;

    if (c_queueSize > 0)
    {
        bsp_enterCritical();

        LOG_INFO("%s\n\r", "Event queue");
        for( i = 0; i < c_queueSize; i++ )
        {
            LOG_RAW("%d | ev = %d : %p\n\r", i, pst_evList[i].c_event, pst_evList[i].p_data);
        }

        nextEvent.c_event = pst_evList[c_queueSize - 1].c_event;
        nextEvent.p_data = pst_evList[c_queueSize - 1].p_data;
        pst_evList[c_queueSize - 1].c_event = 0;
        pst_evList[c_queueSize - 1].p_data = NULL;
        c_queueSize--;

        bsp_exitCritical();

        if (!_evproc_processEvent(nextEvent.c_event, nextEvent.p_data))
        {
            return E_UNKNOWN_TYPE;
        }
        return E_SUCCESS;
    }
    return E_QUEUE_EMPTY;

} /* evproc_nextEvent() */

