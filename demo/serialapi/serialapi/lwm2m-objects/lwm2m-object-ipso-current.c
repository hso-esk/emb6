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
 *  \file       lwm2m-object-ipso-current.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      LWM2M IPSO Current Sensor Object.
 *
*               The LWM2M IPSO Current Sensor Object defines a current
 *              sensor with several resources.
 */

/*
 * --- Includes -------------------------------------------------------------*
 */
#include <stdint.h>
#include "emb6.h"
#include "ctimer.h"
#include "lwm2m-object-ipso-current.h"


/*
 *  --- Macros --------------------------------------------------------------*
 */
#ifndef IPSO_CURRENT_UNIT_MAX
#define IPSO_CURRENT_UNIT_MAX                       16
#endif /* #ifndef IPSO_CURRENT_UNIT_MAX */

#ifndef IPSO_CURRENT_UNIT_DEFAULT
#define IPSO_CURRENT_UNIT_DEFAULT                   "Ampere"
#endif /* #ifndef IPSO_CURRENT_UNIT_DEFAULT */

#ifndef IPSO_CURRENT_CALIB_MAX
#define IPSO_CURRENT_CALIB_MAX                      16
#endif /* #ifndef IPSO_CURRENT_CALIB_MAX */

#ifndef IPSO_CURRENT_CALIB_DEFAULT
#define IPSO_CURRENT_CALIB_DEFAULT                  "uncalibrated"
#endif /* #ifndef IPSO_CURRENT_UNIT_DEFAULT */

#ifndef IPSO_CURRENT_APPTYPE_MAX
#define IPSO_CURRENT_APPTYPE_MAX                    16
#endif /* #ifndef IPSO_CURRENT_APPTYPE_MAX */

#ifndef IPSO_CURRENT_APPTYPE_DEFAULT
#define IPSO_CURRENT_APPTYPE_DEFAULT                "Generic"
#endif /* #ifndef IPSO_CURRENT_APPTYPE_DEFAULT */


/*
 *  --- Local Function Prototypes  ------------------------------------------*
 */

/* Write calibration */
static int write_calib( lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
    uint8_t *outbuf, size_t outsize );

/* Read calibration */
static int read_calib( lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize );

/* Set calibration */
static int set_calib( void* value, size_t size );


/* Write application type */
static int write_apptype( lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
    uint8_t *outbuf, size_t outsize );

/* Read application type */
static int read_apptype( lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize );

/* Set application type */
static int set_apptype( void* value, size_t size );


/*
 *  --- Local Variables  ----------------------------------------------------*
 */

static f_lwm2m_resource_access_cb _p_cb;
static void* _p_cbData;

/** current current */
static int32_t cur_current;
/** current unit */
static uint8_t unit_val[IPSO_CURRENT_UNIT_MAX];
static uint8_t* uint_val_ptr = unit_val;
/** current unit length*/
static uint16_t unit_len;
/** minimum current captured so far */
static int32_t min_current;
/** maximum current captured so far */
static int32_t max_current;
/** minimum current range */
static int32_t range_min_current;
/** maximum current range */
static int32_t range_max_current;

/** Current Calibration */
static uint8_t curr_calib[IPSO_CURRENT_CALIB_MAX];
/** Current Calibration length*/
static uint16_t curr_calib_len;
/** Application Type */
static uint8_t apptype[IPSO_CURRENT_APPTYPE_MAX];
/** Application Type length*/
static uint16_t apptype_len;



LWM2M_RESOURCES(ipso_current_resources,
                /* Temperature (Current) */
                LWM2M_RESOURCE_FLOATFIX_VAR(5700, &cur_current),
                /* Units */
                LWM2M_RESOURCE_STRING_VAR(5701, IPSO_CURRENT_UNIT_MAX, &unit_len, &uint_val_ptr),
                /* Min Measured Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5601, &min_current),
                /* Max Measured Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5602, &max_current),
                /* Min Range Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5603, &range_min_current),
                /* Max Range Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5604, &range_max_current),
                /* Current Calibration */
                LWM2M_RESOURCE_CALLBACK(5821, LWM2M_RESOURCE_TYPE_STR_VARIABLE,
                    { read_calib, write_calib, NULL, set_calib }),
                /* Application Type */
                LWM2M_RESOURCE_CALLBACK(5750, LWM2M_RESOURCE_TYPE_STR_VARIABLE,
                    { read_apptype, write_apptype, NULL, set_apptype }),
                );

LWM2M_INSTANCES(ipso_current_instances,
                LWM2M_INSTANCE(0, ipso_current_resources));

LWM2M_OBJECT(ipso_current, 3317, ipso_current_instances);


/*
 *  --- Local Functions  ----------------------------------------------------*
 */

/**
 * \brief   Callback to write the current calibration.
 */
static int write_calib( lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
    uint8_t *outbuf, size_t outsize )
{
  size_t len;

  len = ctx->reader->read_string(ctx, inbuf, insize, curr_calib, IPSO_CURRENT_CALIB_MAX );
  if(len > 0)
  {
    curr_calib_len = len;
    /* trigger callback if registered */
    if( _p_cb != NULL )
      _p_cb( ctx->object_id, ctx->object_instance_id,
          ctx->resource_id, LWM2M_RESOURCE_TYPE_STR_VARIABLE,
          curr_calib, len, _p_cbData );
  }
  return len;
}


/**
 * \brief   Callback to read the the current calibration.
 */
static int read_calib( lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize )
{
  /* reply with current calibration */
  return ctx->writer->write_string(ctx, outbuf, outsize, (char*)curr_calib,
      curr_calib_len );
}


/**
 * \brief   Callback to set the the current calibration.
 */
static int set_calib( void* value, size_t size )
{
  EMB6_ASSERT_RET( size < sizeof(curr_calib), -1 );
  memcpy( &curr_calib, value, size );
  curr_calib_len = size;
  return curr_calib_len;
}


/**
 * \brief   Callback to write the application type.
 */
static int write_apptype( lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
  uint8_t *outbuf, size_t outsize )
{
  size_t len;

  len = ctx->reader->read_string(ctx, inbuf, insize, apptype, IPSO_CURRENT_APPTYPE_MAX );
  if(len > 0)
  {
    apptype_len = len;
    /* trigger callback if registered */
    if( _p_cb != NULL )
      _p_cb( ctx->object_id, ctx->object_instance_id,
          ctx->resource_id, LWM2M_RESOURCE_TYPE_STR_VARIABLE,
          apptype, len, _p_cbData );
  }
  return len;
}


/**
 * \brief   Callback to read the application type.
 */
static int read_apptype( lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize )
{
  /* reply with application type */
  return ctx->writer->write_string(ctx, outbuf, outsize, (char*)apptype,
      apptype_len );
}


/**
 * \brief   Callback to set application type.
 */
static int set_apptype( void* value, size_t size )
{
  EMB6_ASSERT_RET( size < sizeof(apptype_len), -1 );
  memcpy( &apptype, value, size );
  apptype_len = size;
  return apptype_len;
}


/*
 *  --- Global Functions  ---------------------------------------------------*
 */

/*---------------------------------------------------------------------------*/
/*
* lwm2m_object_ipsoCurrentInit()
*/
int8_t lwm2m_object_ipsoCurrentInit( f_lwm2m_resource_access_cb p_cb,
    void* p_cbData )
{
    int ret = -1;

    cur_current = 0 * LWM2M_FLOAT32_FRAC;
    min_current = 0 * LWM2M_FLOAT32_FRAC;
    max_current = 0 * LWM2M_FLOAT32_FRAC;
    range_min_current = 0 * LWM2M_FLOAT32_FRAC;
    range_max_current = 0 * LWM2M_FLOAT32_FRAC;

    unit_len = snprintf( (char*)unit_val, IPSO_CURRENT_UNIT_MAX,
        IPSO_CURRENT_UNIT_DEFAULT );

    curr_calib_len = snprintf( (char*)curr_calib, IPSO_CURRENT_CALIB_MAX,
        IPSO_CURRENT_CALIB_DEFAULT );

    apptype_len = snprintf( (char*)apptype, IPSO_CURRENT_APPTYPE_MAX,
        IPSO_CURRENT_APPTYPE_DEFAULT );

    _p_cb = p_cb;
    _p_cbData = p_cbData;


    /* register this device and its handlers - the handlers automatically
       sends in the object to handle */
    LWM2M_INIT_OBJECT((&ipso_current));
    lwm2m_engine_register_object(&ipso_current);

    ret = 0;
    return ret;

} /* lwm2m_object_ipsoCurrentInit() */

