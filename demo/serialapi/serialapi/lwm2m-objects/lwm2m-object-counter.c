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

/*
 * --- Module Description ---------------------------------------------------*
 */
/**
 *  \file       lwm2m-object-temp.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      LWM2M Temperature Sensor Object.
 *
*               The LWM2M Temperature Sensor Object defines a temperature
 *              sensor with several resources.
 */

/*
 * --- Includes -------------------------------------------------------------*
 */
#include <stdint.h>
#include "emb6.h"
#include "ctimer.h"
#include "lwm2m-object-temp.h"


/*
 *  --- Macros --------------------------------------------------------------*
 */
#ifndef COUNTER_MIN
#define COUNTER_MIN               (0)
#endif /* #ifndef COUNTER_MIN */

#ifndef COUNTER_MAX
#define COUNTER_MAX               (100)
#endif /* #ifndef COUNTER_MAX */

#ifndef COUNTER_INTERVAL
#define COUNTER_INTERVAL          (1)
#endif /* #ifndef COUNTER_INTERVAL */


/*
 *  --- Local Function Prototypes  ------------------------------------------*
 */

/** periodic timer handler */
static void handle_periodic_timer( void *ptr );

static int read_interval( lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize );

static int write_interval( lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
    uint8_t *outbuf, size_t outsize );

/*
 *  --- Local Variables  ----------------------------------------------------*
 */

/** current counter value */
static int32_t cur_counter;
/** minimum counter value */
static int32_t min_counter;
/** maximum counter value*/
static int32_t max_counter;

/** counter update interval*/
static int32_t interval;


/* handle for the periodic timer */
static struct ctimer periodic_timer;


LWM2M_RESOURCES(counter_resources,
                /* Temperature (Current) */
                LWM2M_RESOURCE_INTEGER_VAR(5700, &cur_counter),
                /* Units */
                LWM2M_RESOURCE_STRING(5701, "Cel"),
                /* Min Range Value */
                LWM2M_RESOURCE_INTEGER(5603, COUNTER_MIN),
                /* Max Range Value */
                LWM2M_RESOURCE_INTEGER(5604, COUNTER_MAX),
                /* Min Measured Value */
                LWM2M_RESOURCE_INTEGER_VAR(5601, &min_counter),
                /* Max Measured Value */
                LWM2M_RESOURCE_INTEGER_VAR(5602, &max_counter),
                /* Interval */
                LWM2M_RESOURCE_CALLBACK(5828, LWM2M_RESOURCE_TYPE_INT_VARIABLE,
                    { read_interval, write_interval, NULL, NULL }),
                );

LWM2M_INSTANCES(counter_instances,
                LWM2M_INSTANCE(0, counter_resources));

LWM2M_OBJECT(counter, 7001, counter_instances);


/*
 *  --- Local Functions  ----------------------------------------------------*
 */
static void handle_periodic_timer(void *ptr)
{
  /* increase counter */
  cur_counter++;

  if( min_counter > cur_counter )
  {
    /* set new value and notify observers */
    min_counter = cur_counter;
    lwm2m_object_notify_observers(&counter, "/0/5601");
  }

  if( max_counter < cur_counter )
  {
    /* set new value and notify observers */
    max_counter = cur_counter;
    lwm2m_object_notify_observers(&counter, "/0/5602");
  }

  if( cur_counter > COUNTER_MAX )
    cur_counter = COUNTER_MIN;

  /* notify observers */
  lwm2m_object_notify_observers(&counter, "/0/5700");

  /* reset timer */
  ctimer_reset(&periodic_timer);
}


static int read_interval( lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize )
{
  /* reply with current interval */
  return ctx->writer->write_int(ctx, outbuf, outsize, interval );

}

static int write_interval( lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
    uint8_t *outbuf, size_t outsize )
{
  int32_t value;
  size_t len;

  len = ctx->reader->read_int(ctx, inbuf, insize, &value);
  if(len > 0)
  {
    /* set the interval */
    interval = value;

    /* restart timer */
    ctimer_stop( &periodic_timer );
    ctimer_set( &periodic_timer, interval * bsp_getTRes(), handle_periodic_timer, NULL );
  }
  return len;
}


/*
 *  --- Global Functions  ---------------------------------------------------*
 */

/*---------------------------------------------------------------------------*/
/*
* lwm2m_object_tempInit()
*/
int8_t lwm2m_object_counterInit( void )
{
    int ret = -1;

    min_counter = COUNTER_MIN;
    max_counter = COUNTER_MIN;
    cur_counter = 0;
    interval = COUNTER_INTERVAL;


    /* register this device and its handlers - the handlers automatically
       sends in the object to handle */
    LWM2M_INIT_OBJECT((&counter));
    lwm2m_engine_register_object( &counter );

    /* start periodic timer */
    ctimer_set( &periodic_timer, interval * bsp_getTRes(),
        handle_periodic_timer, NULL );

    ret = 0;
    return ret;
} /* lwm2m_object_tempInit() */

