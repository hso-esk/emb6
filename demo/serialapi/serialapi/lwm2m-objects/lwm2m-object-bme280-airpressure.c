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
 *  \file       lwm2m-object-airpressure.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      LWM2M BME280 Air Pressure Sensor Object.
 *
*               The LWM2M BME280 Air Pressure Sensor Object defines a air pressure
 *              sensor with several resources.
 */

/*
 * --- Includes -------------------------------------------------------------*
 */
#include <stdint.h>
#include "emb6.h"
#include "ctimer.h"
#include "lwm2m-object-bme280-airpressure.h"


/*
 *  --- Macros --------------------------------------------------------------*
 */
#ifndef AIRPRESSURE_BME280_UNIT_MAX
#define AIRPRESSURE_BME280_UNIT_MAX                 16
#endif /* #ifndef AIRPRESSURE_BME280_UNIT_MAX */

#ifndef AIRPRESSURE_BME280_UNIT_DEFAULT
#define AIRPRESSURE_BME280_UNIT_DEFAULT             "Bar"
#endif /* #ifndef AIRPRESSURE_BME280_UNIT_DEFAULT */


/*
 *  --- Local Function Prototypes  ------------------------------------------*
 */

/* Write min log airpressure */
static int write_log_min( lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
    uint8_t *outbuf, size_t outsize );

/* Read min log airpressure */
static int read_log_min( lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize );

/* Set min log airpressure */
static int set_log_min( void* value, size_t size );


/* Write max log airpressure */
static int write_log_max( lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
    uint8_t *outbuf, size_t outsize );

/* Read max log airpressure */
static int read_log_max( lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize );

/* Set max log airpressure */
static int set_log_max( void* value, size_t size );


/* Write abs log airpressure */
static int write_log_abs( lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
    uint8_t *outbuf, size_t outsize );

/* Read abs log airpressure */
static int read_log_abs( lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize );

/* Set abs log airpressure */
static int set_log_abs( void* value, size_t size );

/*
 *  --- Local Variables  ----------------------------------------------------*
 */

static f_lwm2m_resource_access_cb _p_cb;
static void* _p_cbData;

/** current airpressure */
static int32_t cur_airpressure;
/** airpressure unit */
static uint8_t unit_val[AIRPRESSURE_BME280_UNIT_MAX];
static uint8_t* uint_val_ptr = unit_val;
/** airpressure unit length*/
static uint16_t unit_len;
/** minimum airpressure captured so far */
static int32_t min_airpressure;
/** maximum airpressure captured so far */
static int32_t max_airpressure;
/** minimum airpressure range */
static int32_t range_min_airpressure;
/** maximum airpressure range */
static int32_t range_max_airpressure;

/** write log entry if airpressure gets below this value*/
static int32_t log_min_airpressure;
/** write log entry if airpressure gets above this value*/
static int32_t log_max_airpressure;
/** write log entry if airpressure changes more than this value*/
static int32_t log_abs_airpressure;



LWM2M_RESOURCES(bme280_airpressure_resources,
                /* Temperature (Current) */
                LWM2M_RESOURCE_FLOATFIX_VAR(5700, &cur_airpressure),
                /* Units */
                LWM2M_RESOURCE_STRING_VAR(5701, AIRPRESSURE_BME280_UNIT_MAX, &unit_len, &uint_val_ptr),
                /* Min Range Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5603, &range_min_airpressure),
                /* Max Range Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5604, &range_max_airpressure),
                ///* Min Measured Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5601, &min_airpressure),
                /* Max Measured Value */
                LWM2M_RESOURCE_FLOATFIX_VAR(5602, &max_airpressure),
                /* Interval */
                LWM2M_RESOURCE_CALLBACK(9980, LWM2M_RESOURCE_TYPE_FLOATFIX_VARIABLE,
                    { read_log_min, write_log_min, NULL, set_log_min }),
                /* Interval */
                LWM2M_RESOURCE_CALLBACK(9981, LWM2M_RESOURCE_TYPE_FLOATFIX_VARIABLE,
                    { read_log_max, write_log_max, NULL, set_log_max }),
                /* Interval */
                LWM2M_RESOURCE_CALLBACK(9982, LWM2M_RESOURCE_TYPE_FLOATFIX_VARIABLE,
                    { read_log_abs, write_log_abs, NULL, set_log_abs }),
                );

LWM2M_INSTANCES(bme280_airpressure_instances,
                LWM2M_INSTANCE(0, bme280_airpressure_resources));

LWM2M_OBJECT(bme280_airpressure, 13315, bme280_airpressure_instances);


/*
 *  --- Local Functions  ----------------------------------------------------*
 */

/**
 * \brief   Callback to write the minimum value to write to log file.
 */
static int write_log_min( lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
    uint8_t *outbuf, size_t outsize )
{
  int32_t value;
  size_t len;

  len = ctx->reader->read_float32fix(ctx, inbuf, insize, &value,
      LWM2M_FLOAT32_BITS);
  if(len > 0)
  {
    /* set the value*/
    log_min_airpressure = value;

    /* trigger callback if registered */
    if( _p_cb != NULL )
      _p_cb( ctx->object_id, ctx->object_instance_id,
          ctx->resource_id, LWM2M_RESOURCE_TYPE_FLOATFIX_VARIABLE,
          &value, sizeof(value), _p_cbData );
  }
  return len;
}


/**
 * \brief   Callback to read the minimum value to write to log file.
 */
static int read_log_min( lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize )
{
  /* reply with current interval */
  return ctx->writer->write_float32fix(ctx, outbuf, outsize, log_min_airpressure,
      LWM2M_FLOAT32_BITS );
}


/**
 * \brief   Callback to set the minimum value to write to log file.
 */
static int set_log_min( void* value, size_t size )
{
  EMB6_ASSERT_RET( size == sizeof(log_min_airpressure), -1 );
  memcpy( &log_min_airpressure, value, size );
  return sizeof(log_min_airpressure);
}


/**
 * \brief   Callback to write the maximum value to write to log file.
 */
static int write_log_max( lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
  uint8_t *outbuf, size_t outsize )
{
  int32_t value;
  size_t len;

  len = ctx->reader->read_float32fix(ctx, inbuf, insize, &value,
      LWM2M_FLOAT32_BITS);
  if(len > 0)
  {
    /* set the value*/
    log_max_airpressure = value;

    /* trigger callback if registered */
    if( _p_cb != NULL )
      _p_cb( ctx->object_id, ctx->object_instance_id,
          ctx->resource_id, LWM2M_RESOURCE_TYPE_FLOATFIX_VARIABLE,
          &value, sizeof(value), _p_cbData );
  }
  return len;
}


/**
 * \brief   Callback to read the maximum value to write to log file.
 */
static int read_log_max( lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize )
{
  /* reply with current interval */
  return ctx->writer->write_float32fix(ctx, outbuf, outsize, log_max_airpressure,
      LWM2M_FLOAT32_BITS);
}


/**
 * \brief   Callback to set the maximum value to write to log file.
 */
static int set_log_max( void* value, size_t size )
{
  EMB6_ASSERT_RET( size == sizeof(log_max_airpressure), -1 );
  memcpy( &log_max_airpressure, value, size );
  return sizeof(log_max_airpressure);
}


/**
 * \brief   Callback to write the absolute value change to write to log file.
 */
static int write_log_abs( lwm2m_context_t *ctx, const uint8_t *inbuf, size_t insize,
  uint8_t *outbuf, size_t outsize )
{
  int32_t value;
  size_t len;

  len = ctx->reader->read_float32fix(ctx, inbuf, insize, &value,
      LWM2M_FLOAT32_BITS);
  if(len > 0)
  {
    /* set the value*/
    log_abs_airpressure = value;

    /* trigger callback if registered */
    if( _p_cb != NULL )
      _p_cb( ctx->object_id, ctx->object_instance_id,
          ctx->resource_id, LWM2M_RESOURCE_TYPE_FLOATFIX_VARIABLE,
          &value, sizeof(value), _p_cbData );
  }
  return len;
}


/**
 * \brief   Callback to read the absolute value change to write to log file.
 */
static int read_log_abs( lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize )
{
  /* reply with current interval */
  return ctx->writer->write_float32fix(ctx, outbuf, outsize, log_abs_airpressure,
      LWM2M_FLOAT32_BITS);
}


/**
 * \brief   Callback to set the maximum value to write to log file.
 */
static int set_log_abs( void* value, size_t size )
{
  EMB6_ASSERT_RET( size == sizeof(log_abs_airpressure), -1 );
  memcpy( &log_abs_airpressure, value, size );
  return sizeof(log_abs_airpressure);
}

/*
 *  --- Global Functions  ---------------------------------------------------*
 */

/*---------------------------------------------------------------------------*/
/*
* lwm2m_object_airpressureInit()
*/
int8_t lwm2m_object_airpressureBME280Init( f_lwm2m_resource_access_cb p_cb,
    void* p_cbData )
{
    int ret = -1;

    min_airpressure = 0 * LWM2M_FLOAT32_FRAC;
    max_airpressure = 0 * LWM2M_FLOAT32_FRAC;
    cur_airpressure = 0 * LWM2M_FLOAT32_FRAC;
    range_min_airpressure = 0 * LWM2M_FLOAT32_FRAC;
    range_max_airpressure = 0 * LWM2M_FLOAT32_FRAC;

    unit_len = snprintf( (char*)unit_val, AIRPRESSURE_BME280_UNIT_MAX,
        AIRPRESSURE_BME280_UNIT_DEFAULT );

    _p_cb = p_cb;
    _p_cbData = p_cbData;

    /* register this device and its handlers - the handlers automatically
       sends in the object to handle */
    LWM2M_INIT_OBJECT((&bme280_airpressure));
    lwm2m_engine_register_object( &bme280_airpressure );

    ret = 0;
    return ret;
} /* lwm2m_object_airpressureBME280Init() */

