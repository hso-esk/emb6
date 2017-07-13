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
#ifndef TEMPERATURE_MIN
#define TEMPERATURE_MIN             (-50 * LWM2M_FLOAT32_FRAC)
#endif

#ifndef TEMPERATURE_MAX
#define TEMPERATURE_MAX             (80 * LWM2M_FLOAT32_FRAC)
#endif

#ifndef TEMPERATURE_INTERVAL
#define TEMPERATURE_INTERVAL        (1)
#endif /* #ifndef TEMPERATURE_INTERVAL */


/*
 *  --- Local Function Prototypes  ------------------------------------------*
 */

/** periodic timer handler */
static void handle_periodic_timer( void *ptr );

/*
 *  --- Local Variables  ----------------------------------------------------*
 */

/** current temperature */
static int32_t cur_temp;
/** minimum temperature captured so far */
static int32_t min_temp;
/** maximum temperature captured so far */
static int32_t max_temp;

/* handle for the periodic timer */
static struct ctimer periodic_timer;



LWM2M_RESOURCES(temperature_resources,
                /* Temperature (Current) */
                LWM2M_RESOURCE_FLOATFIX_VAR(5700, &cur_temp),
                /* Units */
                LWM2M_RESOURCE_STRING(5701, "Cel"),
                /* Min Range Value */
                LWM2M_RESOURCE_FLOATFIX(5603, TEMPERATURE_MIN),
                /* Max Range Value */
                LWM2M_RESOURCE_FLOATFIX(5604, TEMPERATURE_MAX),
                /* Min Measured Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5601, &min_temp),
                /* Max Measured Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5602, &max_temp),
                );

LWM2M_INSTANCES(temperature_instances,
                LWM2M_INSTANCE(0, temperature_resources));

LWM2M_OBJECT(temperature, 3303, temperature_instances);


/*
 *  --- Local Functions  ----------------------------------------------------*
 */
static void handle_periodic_timer(void *ptr)
{
  /* increase temperature */
  int32_t temp = cur_temp / LWM2M_FLOAT32_FRAC;
  cur_temp = temp + 1;
  cur_temp = cur_temp * LWM2M_FLOAT32_FRAC;

  if( min_temp > cur_temp )
  {
    /* set new value and notify observers */
    min_temp = cur_temp;
    lwm2m_object_notify_observers(&temperature, "/0/5601");
  }

  if( max_temp < cur_temp )
  {
    /* set new value and notify observers */
    max_temp = cur_temp;
    lwm2m_object_notify_observers(&temperature, "/0/5602");
  }

  if( cur_temp > TEMPERATURE_MAX )
    cur_temp = TEMPERATURE_MIN;

  /* notify observers */
  lwm2m_object_notify_observers(&temperature, "/0/5700");

  /* reset timer */
  ctimer_reset(&periodic_timer);
}


/*
 *  --- Global Functions  ---------------------------------------------------*
 */

/*---------------------------------------------------------------------------*/
/*
* lwm2m_object_tempInit()
*/
int8_t lwm2m_object_tempInit( void )
{
    int ret = -1;

    min_temp = 0 * LWM2M_FLOAT32_FRAC;
    max_temp = 0 * LWM2M_FLOAT32_FRAC;
    cur_temp = 0 * LWM2M_FLOAT32_FRAC;


    /* register this device and its handlers - the handlers automatically
       sends in the object to handle */
    LWM2M_INIT_OBJECT((&temperature));
    lwm2m_engine_register_object(&temperature);

    /* start periodic timer */
    ctimer_set( &periodic_timer, TEMPERATURE_INTERVAL * bsp_getTRes(),
        handle_periodic_timer, NULL );

    ret = 0;
    return ret;
} /* lwm2m_object_tempInit() */

