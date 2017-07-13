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
 *  \file       lwm2m-object-ipso-temperature.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      LWM2M IPSO Temperature Sensor Object.
 *
*               The LWM2M IPSO Temperature Sensor Object defines a temperature
 *              sensor with several resources.
 */

/*
 * --- Includes -------------------------------------------------------------*
 */
#include <stdint.h>
#include "emb6.h"
#include "ctimer.h"
#include "lwm2m-object-ipso-temperature.h"


/*
 *  --- Macros --------------------------------------------------------------*
 */
#ifndef IPSO_TEMPERATURE_UNIT_MAX
#define IPSO_TEMPERATURE_UNIT_MAX                       16
#endif /* #ifndef IPSO_TEMPERATURE_UNIT_MAX */

#ifndef IPSO_TEMPERATURE_UNIT_DEFAULT
#define IPSO_TEMPERATURE_UNIT_DEFAULT                   "Celcius"
#endif /* #ifndef IPSO_TEMPERATURE_UNIT_DEFAULT */



/*
 *  --- Local Variables  ----------------------------------------------------*
 */

static f_lwm2m_resource_access_cb _p_cb;
static void* _p_cbData;

/** current temperature */
static int32_t cur_temperature;
/** temperature unit */
static uint8_t unit_val[IPSO_TEMPERATURE_UNIT_MAX];
static uint8_t* uint_val_ptr = unit_val;
/** temperature unit length*/
static uint16_t unit_len;
/** minimum temperature captured so far */
static int32_t min_temperature;
/** maximum temperature captured so far */
static int32_t max_temperature;
/** minimum temperature range */
static int32_t range_min_temperature;
/** maximum temperature range */
static int32_t range_max_temperature;


LWM2M_RESOURCES(ipso_temperature_resources,
                /* Temperature (Current) */
                LWM2M_RESOURCE_FLOATFIX_VAR(5700, &cur_temperature),
                /* Units */
                LWM2M_RESOURCE_STRING_VAR(5701, IPSO_TEMPERATURE_UNIT_MAX, &unit_len, &uint_val_ptr),
                /* Min Measured Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5601, &min_temperature),
                /* Max Measured Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5602, &max_temperature),
                /* Min Range Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5603, &range_min_temperature),
                /* Max Range Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5604, &range_max_temperature),
                );

LWM2M_INSTANCES(ipso_temperature_instances,
                LWM2M_INSTANCE(0, ipso_temperature_resources));

LWM2M_OBJECT(ipso_temperature, 3303, ipso_temperature_instances);


/*
 *  --- Global Functions  ---------------------------------------------------*
 */

/*---------------------------------------------------------------------------*/
/*
* lwm2m_object_ipsoTemperatureInit()
*/
int8_t lwm2m_object_ipsoTemperatureInit( f_lwm2m_resource_access_cb p_cb,
    void* p_cbData )
{
    int ret = -1;

    cur_temperature = 0 * LWM2M_FLOAT32_FRAC;
    min_temperature = 0 * LWM2M_FLOAT32_FRAC;
    max_temperature = 0 * LWM2M_FLOAT32_FRAC;
    range_min_temperature = 0 * LWM2M_FLOAT32_FRAC;
    range_max_temperature = 0 * LWM2M_FLOAT32_FRAC;

    unit_len = snprintf( (char*)unit_val, IPSO_TEMPERATURE_UNIT_MAX,
        IPSO_TEMPERATURE_UNIT_DEFAULT );

    _p_cb = p_cb;
    _p_cbData = p_cbData;


    /* register this device and its handlers - the handlers automatically
       sends in the object to handle */
    LWM2M_INIT_OBJECT((&ipso_temperature));
    lwm2m_engine_register_object(&ipso_temperature);

    ret = 0;
    return ret;

} /* lwm2m_object_ipsoTemperatureInit() */

