/*
 * Copyright (c) 2015, Yanzi Networks AB.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 *         Implementation of the Contiki OMA LWM2M engine
 * \author
 *         Joakim Eriksson <joakime@sics.se>
 *         Niclas Finne <nfi@sics.se>
 */

/*
 *  --- Includes -------------------------------------------------------------*
 */
#include "emb6.h"
#include "lwm2m-engine.h"
#include "lwm2m-object.h"
#include "lwm2m-server.h"
#include "lwm2m-device.h"
#include "lwm2m-plain-text.h"
#include "lwm2m-json.h"
#include "rest-engine.h"
#include "er-coap-constants.h"
#include "er-coap-engine.h"
#include "oma-tlv.h"
#include "oma-tlv-reader.h"
#include "oma-tlv-writer.h"
#include "uip-ds6.h"
#include "uiplib.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#if UIP_CONF_IPV6_RPL
#include "rpl.h"
#endif /* UIP_CONF_IPV6_RPL */

#define DEBUG 0
#include "uip-debug.h"

/*
 *  --- Macros ------------------------------------------------------------- *
 */
#ifndef LWM2M_ENGINE_CLIENT_ENDPOINT_PREFIX
#ifdef LWM2M_DEVICE_MODEL_NUMBER
#define LWM2M_ENGINE_CLIENT_ENDPOINT_PREFIX LWM2M_DEVICE_MODEL_NUMBER
#else /* LWM2M_DEVICE_MODEL_NUMBER */
#define LWM2M_ENGINE_CLIENT_ENDPOINT_PREFIX "emb6-"
#endif /* LWM2M_DEVICE_MODEL_NUMBER */
#endif /* LWM2M_ENGINE_CLIENT_ENDPOINT_PREFIX */

/* MAximum siz eof the location path */
#define LWM2M_LOCATION_PATH_MAX             64

/* Default ports to connect to the server */
#define REMOTE_PORT        UIP_HTONS(COAP_DEFAULT_PORT)
/* Default ports to connect to the bootstrap server */
#define BS_REMOTE_PORT     UIP_HTONS(5685)

/* Interval of the LWM2M handler */
#define LWM2M_INTERVAL_EVENT_HANDLER                    5
/* Timeout for a registration to complete */
#define LWM2M_TIMEOUT_REGISTRATION                      10
/* How many updates shall be performed during the lifetime */
#define LWM2M_INTERVAL_UPDATE                           ((lwm2m_server_getLifetime() / 2) + \
                                                         bsp_getrand(0, (lwm2m_server_getLifetime() / 4) ))
/* Timeout for a registration to complete */
#define LWM2M_TIMEOUT_UPDATE                            10
/* Maximum number of error allowed during update */
#define LWM2M_ERRMAX_UPDATE                             5


/* Flag identifying that a registration server shall be used */
#define LWM2M_STATUS_FLAG_USE_REG_SERVER                (1 << 0)
/* Flag identifying that a registration server info is available */
#define LWM2M_STATUS_FLAG_HAS_REG_SERVER_INFO           (1 << 1)
/* Flag identifying that a bootstrap server shall be used */
#define LWM2M_STATUS_FLAG_USE_BS_SERVER                 (1 << 2)
/* Flag identifying that a bootstrap server info is available */
#define LWM2M_STATUS_FLAG_HAS_BS_SERVER_INFO            (1 << 3)
/* Flag to indicate registered status */
#define LWM2M_STATUS_FLAG_REGISTERED                    (1 << 7)
/* Flag to indicate error during registration */
#define LWM2M_STATUS_FLAG_REGISTER_ERROR                (1 << 8)
/* Flag to indicate successful update */
#define LWM2M_STATUS_FLAG_UPDATED                       (1 << 15)
/* Flags to indicate error during update */
#define LWM2M_STATUS_FLAG_UPDATE_ERROR                  (1 << 16)
#define LWM2M_STATUS_FLAG_UPDATE_ERROR_POS              (17)
#define LWM2M_STATUS_FLAG_UPDATE_ERROR_MSK              (0x0000000F << LWM2M_STATUS_FLAG_UPDATE_ERROR_POS)


/*
 * --- Type Definitions -----------------------------------------------------*
 */

/**
 * Different states of the LWM2M engine.
 */
typedef enum E_LWM2M_ENGINE_STATE_T
{
  /** Engine is initialized */
  E_LWM2M_ENGINE_STATE_INIT,
  /** Engine is idle */
  E_LWM2M_ENGINE_STATE_IDLE,
  /** Random delay of the engine */
  E_LWM2M_ENGINE_STATE_DELAY,
  /** Engine is bootstrapping */
  E_LWM2M_ENGINE_STATE_BS,
  /** Engine waits for bootstrapping response */
  E_LWM2M_ENGINE_STATE_BS_WAIT,
  /** Engine is registering */
  E_LWM2M_ENGINE_STATE_REG,
  /** Engine waits for register response */
  E_LWM2M_ENGINE_STATE_REG_WAIT,
  /** Engine waits for register response */
  E_LWM2M_ENGINE_STATE_READY,
  /** Engine is updating */
  E_LWM2M_ENGINE_STATE_UPDT,
  /** Engine waits for update response */
  E_LWM2M_ENGINE_STATE_UPDT_WAIT,

} e_lwm2m_engine_state_t;

typedef struct
{
  /* IP address of the server */
  uip_ipaddr_t ipaddr;
  /* Port of the server */
  uint16_t port;

} s_lwm2m_engine_srv_t;

typedef struct
{
  /* allocate some data for the objects */
  const lwm2m_object_t* objects[MAX_OBJECTS];
  /* allocate some data for the endpint name */
  char endpoint[LWM2M_ENDPOINT_NAME_MAX];
  /* allocate some data for the update point */
  char updatepoint[LWM2M_ENDPOINT_NAME_MAX];
  /* allocate some data for the RD */
  char rd_data[LWM2M_RDDATE_LEN_MAX];

} s_lwm2m_engine_internals_t;


typedef struct
{
  /* bootstrap server information */
  s_lwm2m_engine_srv_t srv;
  /* server information */
  s_lwm2m_engine_srv_t bssrv;

  /* internal LWM2M stuff */
  s_lwm2m_engine_internals_t internal;

  /* timer for the cyclic handler */
  struct etimer event_timer;

  /* timer for handling timeouts */
  struct etimer tot_timer;

  /* Status Flags */
  uint32_t statusFlag;

  /* status of the engine */
  e_lwm2m_engine_state_t state;

  /* CoAP tranaction */
  coap_transaction_t* p_trans;

  struct
  {
      /* function pointer to the status callback */
      f_lwm2m_engine_statch_cb p_f;
      /* data for the status callback */
      void* p_data;
  } cb;

} s_lwm2m_engine_ctx_t;


/*
 *  --- Local Variables ---------------------------------------------------- *
 */


//static struct etimer et;
//static f_lwm2m_engine_statch_cb _ctx.cb.p_f;
//static void* _ctx.cb.p_data;

static s_lwm2m_engine_ctx_t _ctx;


void lwm2m_device_init(void);
void lwm2m_security_init(void);

/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */

/* Print CoAP payload. For further details see the function description */
static void _print_coap_payload( void* response );

/* Get first instance of an lwm2m object For further details see the function description */
static const lwm2m_instance_t* _lwm2m_object_first_instance( uint16_t id,
    lwm2m_context_t* context );

/* Get instance of an lwm2m object For further details see the function description */
static const lwm2m_instance_t* _lwm2m_object_get_instance(
        const lwm2m_object_t* object, lwm2m_context_t* context, int depth );

/* Get resource of an lwm2m instance For further details see the function description */
static const lwm2m_resource_t* _lwm2m_instance_get_resource(
        const lwm2m_instance_t* instance, lwm2m_context_t* context );

/* Write objects and instances in link format. For further details see the function description */
static int _lwm2m_wr_object_instances_linkformat( const lwm2m_object_t* object,
    char* buffer, size_t size );

/* Write resources in link format. For further details see the function description */
static int _lwm2m_wr_instance_resources_linkformat(const lwm2m_object_t* object,
    const lwm2m_instance_t* instance, char* buffer, size_t size );

/* Write resources in json format. For further details see the function description */
static int _lwm2m_wr_instance_resources_json( const lwm2m_context_t* context,
    const lwm2m_object_t* object, const lwm2m_instance_t* instance,
    char* buffer, size_t size );

/* Write resources in tlv format. For further details see the function description */
static int _lwm2m_wr_instance_resources_tlv( const lwm2m_context_t* context,
    const lwm2m_object_t* object, const lwm2m_instance_t* instance,
    char* buffer, size_t size );

/* Select writer format. For further details see the function description */
static unsigned int _lwm2m_engine_select_writer( lwm2m_context_t* context,
    unsigned int accept );

/* Select reader format. For further details see the function description */
static void _lwm2m_engine_select_reader( lwm2m_context_t* context,
    unsigned int content_format );

static void _lwm2m_engine_update_registration_status( void );


/* Register at a LWM2M server. For further details see the function description */
static coap_transaction_t* _lwm2m_engine_register( void );

/* Called upon registration request. For further details see the function description */
static void _lwm2m_registration_callback( void* response );

/* Update at a LWM2M server. For further details see the function description */
static coap_transaction_t* _lwm2m_engine_update( void );

/* Called upon update request. For further details see the function description */
static void _lwm2m_update_callback( void* response );

/* Callback for handling timer of the LWM2M engine.
 * For further details see the function description */
static void _lwm2m_engine_timer_handler(c_event_t c_event, p_data_t p_data);

/* Callback for handling timer of the LWM2M engine.
 * For further details see the function description */
static void _lwm2m_engine_tot_timer_handler(c_event_t c_event, p_data_t p_data);

/* Callback for handling events of the LWM2M engine.
 * For further details see the function description */
static void _lwm2m_engine_event_handler(c_event_t c_event, p_data_t p_data);


/*
 *  --- Local Functions ---------------------------------------------------- *
 */


/**
 * \brief   TODO
 *
 * \param   response  TODO
 */
static void _print_coap_payload( void *response )
{
#if (DEBUG) & DEBUG_PRINT
  const uint8_t *chunk;

  int len = coap_get_payload(response, &chunk);

  PRINTF("|%.*s\n", len, (char *)chunk);
#endif /* (DEBUG) & DEBUG_PRINT */
}


/**
 * \brief   TODO
 *
 * \param   id        TODO
 * \param   context   TODO
 *
 * \return  TODO
 */
static const lwm2m_instance_t* _lwm2m_object_first_instance( uint16_t id,
    lwm2m_context_t *context )
{
  const lwm2m_object_t *object;
  int i;

  object = lwm2m_engine_get_object(id);
  if(object == NULL) {
    /* No object with the specified id found */
    return NULL;
  }

  /* Initialize the context */
  memset(context, 0, sizeof(lwm2m_context_t));
  context->object_id = id;

  lwm2m_instance_t *p_inst = object->p_instances;
  for(i = 0; i < (object->count) && (p_inst != NULL); i++) {
    if(p_inst->flag & LWM2M_INSTANCE_FLAG_USED) {
      context->object_instance_id = p_inst->id;
      context->object_instance_index = i;
      return p_inst;
    }
    p_inst = p_inst->p_next;
  }
  return NULL;
}


/**
 * \brief   TODO
 *
 * \param   object    TODO
 * \param   context   TODO
 * \param   depth     TODO
 *
 * \return  TODO
 */
static const lwm2m_instance_t* _lwm2m_object_get_instance( const lwm2m_object_t* object,
    lwm2m_context_t* context, int depth )
{
  int i;
  if(depth > 1) {
    PRINTF("lwm2m: searching for instance %u\n", context->object_instance_id);
    lwm2m_instance_t *p_inst = object->p_instances;
    for(i = 0; i < (object->count) && (p_inst != NULL); i++) {
      PRINTF("  Instance %d -> %u (used: %d)\n", i, object->instances[i].id,
             (object->instances[i].flag & LWM2M_INSTANCE_FLAG_USED) != 0);
      if((p_inst->id == context->object_instance_id) &&
         (p_inst->flag & LWM2M_INSTANCE_FLAG_USED)) {
          context->object_instance_index = i;
          return p_inst;
      }
      p_inst = p_inst->p_next;
    }
  }
  return NULL;
}


/**
 * \brief   TODO
 *
 * \param   instance  TODO
 * \param   context   TODO
 *
 * \return  TODO
 */
static const lwm2m_resource_t* _lwm2m_instance_get_resource( const lwm2m_instance_t* instance,
    lwm2m_context_t* context )
{
  int i;
  if(instance != NULL) {
    PRINTF("lwm2m: searching for resource %u\n", context->resource_id);
    const lwm2m_resource_t *p_res = instance->p_resources;
    for(i = 0; i < (instance->count) && (p_res != NULL); i++) {
      PRINTF("  Resource %d -> %u\n", i, instance->resources[i].id);
      if(p_res->id == context->resource_id) {
        context->resource_index = i;
        return p_res;
      }
      p_res = p_res->p_next;
    }
  }
  return NULL;
}


/**
 * \brief   Write a list of object instances as a CoRE Link-format list.
 *
 * \param   object  TODO
 * \param   buffer  TODO
 * \param   size    TODO
 *
 * \return  TODO
 */
static int _lwm2m_wr_object_instances_linkformat( const lwm2m_object_t* object,
    char* buffer, size_t size )
{
  const lwm2m_instance_t *instance;
  int len, rdlen, i;

  PRINTF("</%d>", object->id);
  rdlen = snprintf(buffer, size, "</%d>",
                   object->id);
  if(rdlen < 0 || rdlen >= size) {
    return -1;
  }

  lwm2m_instance_t *p_inst = object->p_instances;
  for(i = 0; i < (object->count) && (p_inst != NULL); i++) {
    if((p_inst->flag & LWM2M_INSTANCE_FLAG_USED) == 0) {
      continue;
    }
    instance = p_inst;
    PRINTF(",</%d/%d>", object->id, instance->id);

    len = snprintf(&buffer[rdlen], size - rdlen,
                   ",<%d/%d>", object->id, instance->id);
    rdlen += len;
    if(len < 0 || rdlen >= size) {
      return -1;
    }
    p_inst = p_inst->p_next;
  }
  return rdlen;
}


/**
 * \brief   TODO
 *
 * \param   object    TODO
 * \param   instance  TODO
 * \param   buffer    TODO
 * \param   size      TODO
 *
 * \return  TODO
 */
static int _lwm2m_wr_instance_resources_linkformat( const lwm2m_object_t* object, const lwm2m_instance_t* instance,
    char* buffer, size_t size )
{
  const lwm2m_resource_t *resource;
  int len, rdlen, i;

  PRINTF("<%d/%d>", object->id, instance->id);
  rdlen = snprintf(buffer, size, "<%d/%d>",
                   object->id, instance->id);
  if(rdlen < 0 || rdlen >= size) {
    return -1;
  }

  const lwm2m_resource_t *p_res = instance->p_resources;
  for(i = 0; i < (instance->count) && (p_res != NULL); i++) {
    resource = p_res;
    PRINTF(",<%d/%d/%d>", object->id, instance->id, resource->id);

    len = snprintf(&buffer[rdlen], size - rdlen,
                   ",<%d/%d/%d>", object->id, instance->id, resource->id);
    rdlen += len;
    if(len < 0 || rdlen >= size) {
      return -1;
    }

    p_res = p_res->p_next;
  }
  return rdlen;
}


/**
 * \brief   TODO
 *
 * \param   context    TODO
 * \param   object    TODO
 * \param   instance  TODO
 * \param   buffer    TODO
 * \param   size      TODO
 *
 * \return  TODO
 */
static int _lwm2m_wr_instance_resources_json( const lwm2m_context_t* context,
    const lwm2m_object_t* object, const lwm2m_instance_t* instance,
    char* buffer, size_t size )
{
  const lwm2m_resource_t *resource;
  const char *s = "";
  int len, rdlen, i;

  PRINTF("{\"e\":[");
  rdlen = snprintf(buffer, size, "{\"e\":[");
  if(rdlen < 0 || rdlen >= size) {
    return -1;
  }

  const lwm2m_resource_t *p_res = instance->p_resources;
  for(i = 0; i < (instance->count) && (p_res != NULL); i++) {
    resource = p_res;
    len = 0;
    if(lwm2m_object_is_resource_string(resource)) {
      const uint8_t *value;
      uint16_t slen;
      value = lwm2m_object_get_resource_string(resource, context);
      slen = lwm2m_object_get_resource_strlen(resource, context);
      if(value != NULL) {
        PRINTF("%s{\"n\":\"%u\",\"sv\":\"%.*s\"}", s,
               resource->id, slen, value);
        len = snprintf(&buffer[rdlen], size - rdlen,
                       "%s{\"n\":\"%u\",\"sv\":\"%.*s\"}", s,
                       resource->id, slen, value);
      }
    } else if(lwm2m_object_is_resource_int(resource)) {
      int32_t value;
      if(lwm2m_object_get_resource_int(resource, context, &value)) {
        PRINTF("%s{\"n\":\"%u\",\"v\":%" PRId32 "}", s,
               resource->id, value);
        len = snprintf(&buffer[rdlen], size - rdlen,
                       "%s{\"n\":\"%u\",\"v\":%" PRId32 "}", s,
                       resource->id, value);
      }
    } else if(lwm2m_object_is_resource_floatfix(resource)) {
      int32_t value;
      if(lwm2m_object_get_resource_floatfix(resource, context, &value)) {
        PRINTF("%s{\"n\":\"%u\",\"v\":%" PRId32 "}", s, resource->id,
               value / LWM2M_FLOAT32_FRAC);
        len = snprintf(&buffer[rdlen], size - rdlen,
                       "%s{\"n\":\"%u\",\"v\":", s, resource->id);
        rdlen += len;
        if(len < 0 || rdlen >= size) {
          return -1;
        }

        len = lwm2m_plain_text_write_float32fix((uint8_t *)&buffer[rdlen],
                                                size - rdlen,
                                                value, LWM2M_FLOAT32_BITS);
        if(len == 0) {
          return -1;
        }
        rdlen += len;

        if(rdlen < size) {
          buffer[rdlen] = '}';
        }
        len = 1;
      }
    } else if(lwm2m_object_is_resource_boolean(resource)) {
      int value;
      if(lwm2m_object_get_resource_boolean(resource, context, &value)) {
        PRINTF("%s{\"n\":\"%u\",\"bv\":%s}", s, resource->id,
               value ? "true" : "false");
        len = snprintf(&buffer[rdlen], size - rdlen,
                       "%s{\"n\":\"%u\",\"bv\":%s}", s, resource->id,
                       value ? "true" : "false");
      }
    } else if(lwm2m_object_is_resource_callback(resource)) {

      if( resource->value.callback.read != NULL ) {
        char varBuf[32] = {0};
        lwm2m_context_t tmpContext;

        len = snprintf(&buffer[rdlen], size - rdlen,
                        "%s{\"n\":\"%u\",", s, resource->id);


        rdlen += len;
        if(len < 0 || rdlen >= size) {
          return -1;
        }

        /** read value into buffer */
        tmpContext = *context;
        tmpContext.writer = &lwm2m_plain_text_writer;
        len = resource->value.callback.read(&tmpContext, (uint8_t*)varBuf,32 );

        if(len < 0) {
          return -1;
        }

        switch( resource->subtype )
        {
          case LWM2M_RESOURCE_TYPE_BOOLEAN_VALUE:
          case LWM2M_RESOURCE_TYPE_BOOLEAN_VARIABLE:
            len = snprintf(&buffer[rdlen], size - rdlen,
                          "\"bv\":%s", varBuf );
            break;

          case LWM2M_RESOURCE_TYPE_STR_VALUE:
          case LWM2M_RESOURCE_TYPE_STR_VARIABLE:
            len = snprintf(&buffer[rdlen], size - rdlen,
                          "\"sv\":\"%s\"", varBuf );
            break;

          case LWM2M_RESOURCE_TYPE_INT_VALUE:
          case LWM2M_RESOURCE_TYPE_INT_VARIABLE:
          case LWM2M_RESOURCE_TYPE_FLOATFIX_VALUE:
          case LWM2M_RESOURCE_TYPE_FLOATFIX_VARIABLE:
            len = snprintf(&buffer[rdlen], size - rdlen,
                          "\"v\":%s", varBuf );
            break;
        }

        if(len == 0) {
          return -1;
        }
        rdlen += len;

        if(rdlen < size) {
          buffer[rdlen] = '}';
        }
        len = 1;
      }
    }
    rdlen += len;
    if(len < 0 || rdlen >= size) {
      return -1;
    }
    if(len > 0) {
      s = ",";
    }
    p_res = p_res->p_next;
  }
  PRINTF("]}\n");
  len = snprintf(&buffer[rdlen], size - rdlen, "]}");
  rdlen += len;
  if(len < 0 || rdlen >= size) {
    return -1;
  }

  return rdlen;
}


/**
 * \brief   TODO
 *
 * \param   context    TODO
 * \param   object    TODO
 * \param   instance  TODO
 * \param   buffer    TODO
 * \param   size      TODO
 *
 * \return  TODO
 */
static int _lwm2m_wr_instance_resources_tlv( const lwm2m_context_t* context,
    const lwm2m_object_t* object, const lwm2m_instance_t* instance,
    char* buffer, size_t size )
{
  const lwm2m_resource_t *resource;
  int len, rdlen, i;
  lwm2m_context_t ctx;
  ctx.writer = &oma_tlv_writer;

  rdlen = 0;
  const lwm2m_resource_t *p_res = instance->p_resources;
  for(i = 0; i < (instance->count) && (p_res != NULL); i++) {
    resource = p_res;
    len = 0;
    if(lwm2m_object_is_resource_string(resource)) {

      const uint8_t *value;
      uint16_t slen;
      value = lwm2m_object_get_resource_string(resource, context);
      slen = lwm2m_object_get_resource_strlen(resource, context);
      if(value != NULL) {
        ctx.resource_id = resource->id;
        len = ctx.writer->write_string( &ctx, (uint8_t*)&buffer[rdlen], size - rdlen,
            (const char*)value, slen);
      }


    } else if(lwm2m_object_is_resource_int(resource)) {

      int32_t value;
      if(lwm2m_object_get_resource_int(resource, context, &value)) {
        ctx.resource_id = resource->id;
        len = ctx.writer->write_int( &ctx, (uint8_t*)&buffer[rdlen], size - rdlen,
            value);
      }

    } else if(lwm2m_object_is_resource_floatfix(resource)) {

      int32_t value;
      if(lwm2m_object_get_resource_floatfix(resource, context, &value)) {
        ctx.resource_id = resource->id;
        len = ctx.writer->write_float32fix( &ctx, (uint8_t*)&buffer[rdlen], size - rdlen,
            value, LWM2M_FLOAT32_BITS );
      }


    } else if(lwm2m_object_is_resource_boolean(resource)) {

      int value;
      if(lwm2m_object_get_resource_boolean(resource, context, &value)) {
        ctx.resource_id = resource->id;
        len = ctx.writer->write_boolean( &ctx, (uint8_t*)&buffer[rdlen], size - rdlen,
            value );
      }


    } else if(lwm2m_object_is_resource_callback(resource)) {

      ctx.resource_id = resource->id;
      if( resource->value.callback.read != NULL ) {
        len = resource->value.callback.read (&ctx, (uint8_t*)&buffer[rdlen], size - rdlen );
      }
    }

    if(len < 0) {
      return -1;
    }

    rdlen += len;
    if(len < 0 || rdlen >= size) {
      return -1;
    }

    p_res = p_res->p_next;
  }

  if(len < 0 || rdlen >= size) {
    return -1;
  }

  return rdlen;
}


/**
 * \brief   Set the writer pointer to the proper writer based on the Accept: header
 *
 * \param   context   LWM2M context to operate on
 * \param   accept    Accept type number from CoAP headers
 *
 * \return  The content type of the response if the selected writer is used
 */
static unsigned int _lwm2m_engine_select_writer( lwm2m_context_t* context,
    unsigned int accept )
{
  switch(accept) {
    case LWM2M_TLV:
      context->writer = &oma_tlv_writer;
      break;
    case TEXT_PLAIN:
      context->writer = &lwm2m_plain_text_writer;
      accept = TEXT_PLAIN;
      break;
    case LWM2M_JSON:
      context->writer = &lwm2m_json_writer;
      break;
    default:
      PRINTF("Unknown Accept type %u, using LWM2M plain text\n", accept);
      context->writer = &lwm2m_plain_text_writer;
      /* Set the response type to plain text */
      accept = TEXT_PLAIN;
      break;
  }
  return accept;
}


/**
 * \brief   Set the reader pointer to the proper reader based on the Content-format: header
 *
 * \param   context           LWM2M context to operate on
 * \param   content_format    Content-type type number from CoAP headers
 *
 * \return  The content type of the response if the selected writer is used
 */
static void _lwm2m_engine_select_reader( lwm2m_context_t* context,
    unsigned int content_format)
{
  switch(content_format) {
    case LWM2M_TLV:
      context->reader = &oma_tlv_reader;
      break;
    case TEXT_PLAIN:
      context->reader = &lwm2m_plain_text_reader;
      break;
    default:
      PRINTF("Unknown content type %u, using LWM2M plain text\n",
             content_format);
      context->reader = &lwm2m_plain_text_reader;
      break;
  }
}


/**
 * \brief   Register at a LWM2M server.
 *
 * \return  Pointer to the CoAP transaction that was used or NULL on error.
 */
static coap_transaction_t* _lwm2m_engine_register( void )
{
  static coap_packet_t request[1];
  coap_transaction_t* p_trans = NULL;
  int pos;
  int len, i, j;

  /* Create the underlying CoAP packet */
  coap_init_message(request, COAP_TYPE_CON, COAP_POST, 0);
  coap_set_header_uri_path(request, "/rd");
  coap_set_header_uri_query(request, _ctx.internal.endpoint);

  /* generate the rd data from the available objects
   * instances and resources */
  pos = 0;
  for(i = 0; i < MAX_OBJECTS; i++) {
    if(_ctx.internal.objects[i] != NULL) {
      lwm2m_instance_t *p_inst = _ctx.internal.objects[i]->p_instances;
      for(j = 0; j < (_ctx.internal.objects[i]->count) && (p_inst != NULL); j++) {
        if(p_inst->flag & LWM2M_INSTANCE_FLAG_USED) {
          len = snprintf(&_ctx.internal.rd_data[pos], sizeof(_ctx.internal.rd_data) - pos,
                         "%s<%d/%d>", pos > 0 ? "," : "",
                         _ctx.internal.objects[i]->id, p_inst->id);
          if(len > 0 && len < sizeof(_ctx.internal.rd_data) - pos) {
            pos += len;
          }
        }
        p_inst = p_inst->p_next;
      }
    }
  }

  /* Set the rd data as CoAP payload */
  coap_set_payload(request, (uint8_t *)_ctx.internal.rd_data, pos);

  PRINTF("Registering with [");
  PRINT6ADDR(&server_ipaddr);
  PRINTF("]:%u lwm2m endpoint '%s': '%.*s'\n", uip_ntohs(server_port),
         _ctx.internal.endpoint, pos, _ctx.rd_data);

  /* Try to send a priorized CoAP transaction */

  coap_use_prio_transaction();
  p_trans = coap_nonblocking_request(&_ctx.srv.ipaddr, _ctx.srv.port, request,
      _lwm2m_registration_callback);
  coap_use_nonprio_transaction();

  return p_trans;
}

/*---------------------------------------------------------------------------*/
static void
_lwm2m_registration_callback(void *response)
{
  /* assume that an error occurred */
  _ctx.statusFlag |= LWM2M_STATUS_FLAG_REGISTER_ERROR;

  if( response != NULL )
  {
    volatile uint8_t code = ((coap_packet_t *)response)->code;
    if( code == CREATED_2_01)
    {
      /* store the registration path */
      if( ((coap_packet_t *)response)->location_path_len < sizeof(_ctx.internal.updatepoint) )
      {
        memset(_ctx.internal.updatepoint, 0, sizeof(_ctx.internal.updatepoint));
        memcpy( _ctx.internal.updatepoint, ((coap_packet_t *)response)->location_path,
            ((coap_packet_t *)response)->location_path_len );

        /* client is registered */
        _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_REGISTER_ERROR;
        _ctx.statusFlag |= LWM2M_STATUS_FLAG_REGISTERED;
      }
    }
  }
}


/**
 * \brief   Update at a LWM2M server.
 */
static coap_transaction_t* _lwm2m_engine_update( void )
{
  static coap_packet_t request[1];
  coap_transaction_t* p_trans = NULL;

  /* Generate the update request for the update point. This was
   * set from the registration response */
  coap_init_message(request, COAP_TYPE_CON, COAP_POST, 0);
  coap_set_header_uri_path(request, _ctx.internal.updatepoint);

  PRINTF("Registering with [");
  PRINT6ADDR(&server_ipaddr);
  PRINTF("]:%u lwm2m updatepoint '%s'\n", uip_ntohs(server_port),
         endpoint );

  /* Try to send a priorized CoAP transaction */
  coap_use_prio_transaction();
  p_trans = coap_nonblocking_request(&_ctx.srv.ipaddr, _ctx.srv.port, request,
      _lwm2m_update_callback);
  coap_use_nonprio_transaction();

  return p_trans;
}

/*---------------------------------------------------------------------------*/
static void
_lwm2m_update_callback(void *response)
{
  uint32_t errors = (_ctx.statusFlag & LWM2M_STATUS_FLAG_UPDATE_ERROR_MSK) >>
          LWM2M_STATUS_FLAG_UPDATE_ERROR_POS;

  /* assume that an error occurred */
  errors++;
  _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_UPDATE_ERROR_MSK;
  _ctx.statusFlag |= (errors << LWM2M_STATUS_FLAG_UPDATE_ERROR_POS) &
          LWM2M_STATUS_FLAG_UPDATE_ERROR_MSK;
  _ctx.statusFlag |= LWM2M_STATUS_FLAG_UPDATE_ERROR;

  if( response != NULL )
  {
    volatile uint8_t code = ((coap_packet_t *)response)->code;
    if( code == CHANGED_2_04)
    {
      /* client was updated */
      _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_UPDATE_ERROR;
      _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_UPDATE_ERROR_MSK;
      _ctx.statusFlag |= LWM2M_STATUS_FLAG_UPDATED;
    }
  }
}



/*---------------------------------------------------------------------------*/
static void
_lwm2m_engine_update_registration_status( void )
{
  if( _ctx.cb.p_f != NULL )
  {
    /* call status update callback */
    _ctx.cb.p_f( lwm2m_engine_is_registered(), _ctx.cb.p_data );
  }
}




/*---------------------------------------------------------------------------*/
static int
index_of(const uint8_t *data, int offset, int len, uint8_t c)
{
  if(offset < 0) {
    return offset;
  }
  for(; offset < len; offset++) {
    if(data[offset] == c) {
      return offset;
    }
  }
  return -1;
}
/*---------------------------------------------------------------------------*/
static int
has_network_access(void)
{
#if UIP_CONF_IPV6_RPL
  if(rpl_get_any_dag() == NULL) {
    return 0;
  }
#endif /* UIP_CONF_IPV6_RPL */
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
update_registration_server(void)
{
  if( _ctx.statusFlag & LWM2M_STATUS_FLAG_HAS_REG_SERVER_INFO ) {
    return 1;
  }

#if UIP_CONF_IPV6_RPL
  {
    rpl_dag_t *dag;

    /* Use the DAG id as server address if no other has been specified */
    dag = rpl_get_any_dag();
    if(dag != NULL) {
      uip_ipaddr_copy(&_ctx.srv.ipaddr, &dag->dag_id);
      _ctx.srv.port = REMOTE_PORT;
      return 1;
    }
  }
#endif /* UIP_CONF_IPV6_RPL */

  _ctx.statusFlag |= LWM2M_STATUS_FLAG_HAS_REG_SERVER_INFO;
  return 0;
}

/*---------------------------------------------------------------------------*/
static int
update_bootstrap_server(void)
{
  if( _ctx.statusFlag & LWM2M_STATUS_FLAG_HAS_BS_SERVER_INFO) {
    return 1;
  }

#if UIP_CONF_IPV6_RPL
  {
    rpl_dag_t *dag;

    /* Use the DAG id as server address if no other has been specified */
    dag = rpl_get_any_dag();
    if(dag != NULL) {
      uip_ipaddr_copy(&_ctx.bssrv.ipaddr, &dag->dag_id);
      _ctx.bssrv.port = REMOTE_PORT;
      return 1;
    }
  }
#endif /* UIP_CONF_IPV6_RPL */

  _ctx.statusFlag |= LWM2M_STATUS_FLAG_HAS_BS_SERVER_INFO;
  return 0;
}


/*---------------------------------------------------------------------------*/
static void
_lwm2m_engine_timer_handler(c_event_t c_event, p_data_t p_data)
{
  if( (c_event == EVENT_TYPE_TIMER_EXP) && etimer_expired(&_ctx.event_timer) )
  {
    etimer_restart( &_ctx.event_timer );
    /* call the internal event handler */
    evproc_putEvent( E_EVPROC_EXEC, EVENT_TYPE_LWM2M, NULL );
  }
}

/*---------------------------------------------------------------------------*/
static void
_lwm2m_engine_tot_timer_handler(c_event_t c_event, p_data_t p_data)
{
  if( (c_event == EVENT_TYPE_TIMER_EXP) && etimer_expired(&_ctx.tot_timer) )
  {
    etimer_stop( &_ctx.tot_timer );
    /* call the internal event handler */
    evproc_putEvent( E_EVPROC_EXEC, EVENT_TYPE_LWM2M, NULL );
  }
}

/*---------------------------------------------------------------------------*/
static void
_lwm2m_engine_event_handler(c_event_t c_event, p_data_t p_data)
{

  if( c_event == EVENT_TYPE_LWM2M )
  {

    /* update status */
      _lwm2m_engine_update_registration_status();

    /* LWM2M event triggered. Check for according action to handle
     * depending on the current state of the engine */
    switch( _ctx.state )
    {
      case E_LWM2M_ENGINE_STATE_INIT:

        /* Engine needs to be initialized. */
        _ctx.state = E_LWM2M_ENGINE_STATE_IDLE;
        evproc_putEvent( E_EVPROC_TAIL, EVENT_TYPE_LWM2M, NULL );

        /* reset Flags */
        _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_REGISTERED;
        _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_REGISTER_ERROR;
        _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_UPDATED;
        _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_UPDATE_ERROR;
        _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_UPDATE_ERROR_MSK;

        break;



      case E_LWM2M_ENGINE_STATE_IDLE:

        /* Check if we can start operating */
        if( has_network_access() )
        {
            /* we have network access. So we can check whether we
             * have all the information required for registration */
            if( (_ctx.statusFlag & LWM2M_STATUS_FLAG_USE_REG_SERVER) &&
                (_ctx.statusFlag & LWM2M_STATUS_FLAG_HAS_REG_SERVER_INFO) )
            {
                /* we can start the registration procedure */
                _ctx.state = E_LWM2M_ENGINE_STATE_DELAY;
                evproc_putEvent( E_EVPROC_TAIL, EVENT_TYPE_LWM2M, NULL );

                etimer_set( &_ctx.tot_timer, bsp_getrand(0, 5 * bsp_getTRes()),
                            _lwm2m_engine_tot_timer_handler );
            }
        }
        break;


      case E_LWM2M_ENGINE_STATE_DELAY:

        if( etimer_expired( &_ctx.tot_timer ) )
        {
            /* we only come here if the update interval was triggered */
            _ctx.state = E_LWM2M_ENGINE_STATE_REG;
            evproc_putEvent( E_EVPROC_TAIL, EVENT_TYPE_LWM2M, NULL );

            /* set the timeout timer to the according wait
             * interval */
            etimer_stop( &_ctx.tot_timer );
        }
        break;



      case E_LWM2M_ENGINE_STATE_BS:

        /* Not implemented yet */
        _ctx.state = E_LWM2M_ENGINE_STATE_BS_WAIT;
        evproc_putEvent( E_EVPROC_TAIL, EVENT_TYPE_LWM2M, NULL );
        break;



      case E_LWM2M_ENGINE_STATE_BS_WAIT:

        /* Not implemented yet */
        _ctx.state = E_LWM2M_ENGINE_STATE_REG;
        evproc_putEvent( E_EVPROC_TAIL, EVENT_TYPE_LWM2M, NULL );
        break;



      case E_LWM2M_ENGINE_STATE_REG:

        /* reset status flags */
        _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_REGISTERED;
        _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_REGISTER_ERROR;

        /* start the registration process */
        if( ( _ctx.p_trans = _lwm2m_engine_register() ) != NULL )
        {
            /* wait for the status of the registration */
            _ctx.state = E_LWM2M_ENGINE_STATE_REG_WAIT;

            /* set the timeout timer to the according wait
             * interval */
            etimer_stop( &_ctx.tot_timer );
            etimer_set( &_ctx.tot_timer, LWM2M_TIMEOUT_REGISTRATION *
                        bsp_getTRes(), NULL);
        }
        break;



      case E_LWM2M_ENGINE_STATE_REG_WAIT:

        /* Check if the registered flag was set */
        if( _ctx.statusFlag & LWM2M_STATUS_FLAG_REGISTERED )
        {
            /* the client was registered successfully */
            _ctx.state = E_LWM2M_ENGINE_STATE_READY;

            /* set the timeout timer to the according wait
             * interval */
            etimer_stop( &_ctx.tot_timer );
            etimer_set( &_ctx.tot_timer, LWM2M_INTERVAL_UPDATE *
                        bsp_getTRes(), NULL);

        }
        else if( _ctx.statusFlag & LWM2M_STATUS_FLAG_REGISTER_ERROR )
        {
            /* An error occurred during the registration */
            _ctx.state = E_LWM2M_ENGINE_STATE_REG;
            evproc_putEvent( E_EVPROC_TAIL, EVENT_TYPE_LWM2M, NULL );
        }
        else if( etimer_expired( &_ctx.tot_timer) )
        {
            /* timeout expired go back to registration. Delete pending
             * transactions */
            if( _ctx.p_trans != NULL )
                coap_clear_transaction( _ctx.p_trans );

            _ctx.state = E_LWM2M_ENGINE_STATE_REG;
            evproc_putEvent( E_EVPROC_TAIL, EVENT_TYPE_LWM2M, NULL );
        }
        break;



      case E_LWM2M_ENGINE_STATE_READY:

        if( etimer_expired( &_ctx.tot_timer ) )
        {
            /* we only come here if the update interval was triggered */
            _ctx.state = E_LWM2M_ENGINE_STATE_UPDT;
            evproc_putEvent( E_EVPROC_TAIL, EVENT_TYPE_LWM2M, NULL );

            /* Clear error counter */
            _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_UPDATE_ERROR_MSK;

            /* set the timeout timer to the according wait
             * interval */
            etimer_stop( &_ctx.tot_timer );
            etimer_set( &_ctx.tot_timer, LWM2M_TIMEOUT_UPDATE *
                        bsp_getTRes(), NULL);
        }
        break;



      case E_LWM2M_ENGINE_STATE_UPDT:

          /* reset status flags */
          _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_UPDATED;
          _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_UPDATE_ERROR;

          /* send the update packet */
          if( ( _ctx.p_trans = _lwm2m_engine_update() ) != NULL )
          {
              /* wait for the status of the registration */
              _ctx.state = E_LWM2M_ENGINE_STATE_UPDT_WAIT;
              evproc_putEvent( E_EVPROC_EXEC, EVENT_TYPE_LWM2M, NULL );
          }
          else if( etimer_expired( &_ctx.tot_timer) )
          {
              /* timeout expired go back to registration. Delete pending
               * transactions */
              if( _ctx.p_trans != NULL )
                  coap_clear_transaction( _ctx.p_trans );
              _ctx.state = E_LWM2M_ENGINE_STATE_REG;


          }
        break;



      case E_LWM2M_ENGINE_STATE_UPDT_WAIT:
      {

          uint32_t errors = (_ctx.statusFlag & LWM2M_STATUS_FLAG_UPDATE_ERROR_MSK) >>
                  LWM2M_STATUS_FLAG_UPDATE_ERROR_POS;

          /* Check if the registered flag was set */
          if( _ctx.statusFlag & LWM2M_STATUS_FLAG_UPDATED )
          {
              /* the client was registered successfully */
              _ctx.state = E_LWM2M_ENGINE_STATE_READY;
              _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_UPDATE_ERROR;

              /* set the timeout timer to the according wait
               * interval */
              etimer_stop( &_ctx.tot_timer );
              etimer_set( &_ctx.tot_timer, LWM2M_INTERVAL_UPDATE *
                          bsp_getTRes(), NULL);
          }
          else if( etimer_expired( &_ctx.tot_timer) )
          {
              /* timeout expired go back to registration. Delete pending
               * transactions */
              if( _ctx.p_trans != NULL )
                  coap_clear_transaction( _ctx.p_trans );

              errors++;
              _ctx.statusFlag &= ~LWM2M_STATUS_FLAG_UPDATE_ERROR_MSK;
              _ctx.statusFlag |= (errors << LWM2M_STATUS_FLAG_UPDATE_ERROR_POS) &
                      LWM2M_STATUS_FLAG_UPDATE_ERROR_MSK;
              _ctx.statusFlag |= LWM2M_STATUS_FLAG_UPDATE_ERROR;
          }

          if( _ctx.statusFlag & LWM2M_STATUS_FLAG_UPDATE_ERROR )
          {
              if( errors >= LWM2M_ERRMAX_UPDATE )
              {
                  /* To many errors occurred during the registration */
                  _ctx.state = E_LWM2M_ENGINE_STATE_REG;
                  evproc_putEvent( E_EVPROC_TAIL, EVENT_TYPE_LWM2M, NULL );
              }
              else
              {
                  /* try again to update */
                  _ctx.state = E_LWM2M_ENGINE_STATE_UPDT;
                  evproc_putEvent( E_EVPROC_TAIL, EVENT_TYPE_LWM2M, NULL );

                  /* set the timeout timer to the according wait
                   * interval */
                  etimer_stop( &_ctx.tot_timer );
                  etimer_set( &_ctx.tot_timer, LWM2M_TIMEOUT_UPDATE *
                              bsp_getTRes(), NULL);
              }
          }
          break;
      }
    }
  }
}

/*---------------------------------------------------------------------------*/
static int
parse_next(const char **path, int *path_len, uint16_t *value)
{
  char c;
  *value = 0;
  /* PRINTF("parse_next: %p %d\n", *path, *path_len); */
  if(*path_len == 0) {
    return 0;
  }
  while(*path_len > 0) {
    c = **path;
    (*path)++;
    *path_len = *path_len - 1;
    if(c >= '0' && c <= '9') {
      *value = *value * 10 + (c - '0');
    } else if(c == '/') {
      return 1;
    } else {
      /* error */
      return -4;
    }
  }
  return 1;
}


/*
 * --- Global Function Definitions ----------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* lwm2m_engine_use_bootstrap_server()
*/
void lwm2m_engine_use_bootstrap_server(int use)
{
  _ctx.statusFlag |= ( use ? LWM2M_STATUS_FLAG_USE_BS_SERVER : 0 );
} /* lwm2m_engine_use_bootstrap_server() */


/*---------------------------------------------------------------------------*/
/*
* lwm2m_engine_use_registration_server()
*/
void lwm2m_engine_use_registration_server(int use)
{
  _ctx.statusFlag |= ( use ? LWM2M_STATUS_FLAG_USE_REG_SERVER : 0 );
} /* lwm2m_engine_use_registration_server() */


/*---------------------------------------------------------------------------*/
/*
* lwm2m_engine_register_with_server()
*/
void lwm2m_engine_register_with_server(const uip_ipaddr_t *server, uint16_t port)
{
  uip_ipaddr_copy(&_ctx.srv.ipaddr, server);
  if(port != 0) {
      _ctx.srv.port = port;
  } else {
      _ctx.srv.port = REMOTE_PORT;
  }

  _ctx.statusFlag |= LWM2M_STATUS_FLAG_HAS_REG_SERVER_INFO;
} /* lwm2m_engine_register_with_server() */


/*---------------------------------------------------------------------------*/
/*
* lwm2m_engine_register_with_bootstrap_server()
*/
void lwm2m_engine_register_with_bootstrap_server( const uip_ipaddr_t *server,
    uint16_t port )
{
  uip_ipaddr_copy(&_ctx.bssrv.ipaddr, server);
  if(port != 0) {
    _ctx.bssrv.port = port;
  } else {
    _ctx.bssrv.port = BS_REMOTE_PORT;
  }

  _ctx.statusFlag |= LWM2M_STATUS_FLAG_HAS_BS_SERVER_INFO;
} /* lwm2m_engine_register_with_bootstrap_server() */


/*---------------------------------------------------------------------------*/
/*
* lwm2m_engine_init()
*/
void lwm2m_engine_init( char* epname, f_lwm2m_engine_statch_cb p_cb,
    void* p_data )
{
  int i;
  /* initialize REST engine */
  rest_init_engine();

  /* set callback */
  _ctx.cb.p_f = p_cb;
  _ctx.cb.p_data = p_data;

  update_bootstrap_server();
  update_registration_server();

  for(i = 0; i < MAX_OBJECTS; i++)
    _ctx.internal.objects[i] = NULL;

  /* register the default objects */
  lwm2m_engine_register_default_objects();

#ifdef LWM2M_ENGINE_CLIENT_ENDPOINT_NAME

  snprintf(_ctx.internal.endpoint, sizeof(_ctx.internal.endpoint) - 1,
           "?ep=" LWM2M_ENGINE_CLIENT_ENDPOINT_NAME);

#else /* LWM2M_ENGINE_CLIENT_ENDPOINT_NAME */

  if( epname != NULL )
  {
      snprintf(_ctx.internal.endpoint, sizeof(_ctx.internal.endpoint) - 1,
               "?ep=%s&lt=%d&b=%s", epname, lwm2m_server_getLifetime(),
               lwm2m_server_getBinding() );
  }
  else
  {
    int len, i;
    uint8_t state;
    uip_ipaddr_t *ipaddr;
    char client[sizeof(_ctx.internal.endpoint)];

    len = strlen(LWM2M_ENGINE_CLIENT_ENDPOINT_PREFIX);
    /* ensure that this fits with the hex-nums */
    if(len > sizeof(client) - 13) {
      len = sizeof(client) - 13;
    }
    memcpy(client, LWM2M_ENGINE_CLIENT_ENDPOINT_PREFIX, len);

    /* pick an IP address that is PREFERRED or TENTATIVE */
    ipaddr = NULL;
    for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
      state = uip_ds6_if.addr_list[i].state;
      if(uip_ds6_if.addr_list[i].isused &&
         (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
        ipaddr = &(uip_ds6_if.addr_list[i]).ipaddr;
        break;
      }
    }

    if(ipaddr != NULL) {
      for(i = 0; i < 6; i++) {
        /* assume IPv6 for now */
        uint8_t b = ipaddr->u8[10 + i];
        client[len++] = (b >> 4) > 9 ? 'A' - 10 + (b >> 4) : '0' + (b >> 4);
        client[len++] = (b & 0xf) > 9 ? 'A' - 10 + (b & 0xf) : '0' + (b & 0xf);
      }
    }

    /* a zero at end of string */
    client[len] = 0;
    /* create endpoint */
    snprintf(_ctx.internal.endpoint, sizeof(_ctx.internal.endpoint) - 1, "?ep=%s", client);
  }

#endif /* LWM2M_ENGINE_CLIENT_ENDPOINT_NAME */

  /* start timer for local lwm2m engine callback */
  etimer_stop( &_ctx.event_timer );
  etimer_set( &_ctx.event_timer, LWM2M_INTERVAL_EVENT_HANDLER * bsp_getTRes(),
             _lwm2m_engine_timer_handler );

  /* reset timeout timer */
  etimer_stop(&_ctx.tot_timer);

  /* register event handler */
  evproc_unregCallback( EVENT_TYPE_LWM2M, _lwm2m_engine_event_handler );
  evproc_regCallback( EVENT_TYPE_LWM2M, _lwm2m_engine_event_handler );

  _ctx.state = E_LWM2M_ENGINE_STATE_INIT;
} /* lwm2m_engine_init() */


/*---------------------------------------------------------------------------*/
/*
* lwm2m_engine_stop()
*/
void lwm2m_engine_stop( void )
{
  /* initialize REST engine */
  rest_init_engine();

  /* stop LWM2M event timer */
  etimer_stop(&_ctx.event_timer);
} /* lwm2m_engine_stop() */


/*---------------------------------------------------------------------------*/
/*
* lwm2m_engine_register_default_objects()
*/
void lwm2m_engine_register_default_objects(void)
{
  lwm2m_security_init();
  lwm2m_server_init();
  lwm2m_device_init();
} /* lwm2m_engine_register_default_objects() */


/*---------------------------------------------------------------------------*/
/*
* lwm2m_engine_parse_context()
*/
int lwm2m_engine_parse_context(const lwm2m_object_t *object,
    const char *path, int path_len, lwm2m_context_t *context)
{
  int ret;
  if(context == NULL || object == NULL || path == NULL) {
    return 0;
  }
  memset(context, 0, sizeof(lwm2m_context_t));
  /* get object id */
  ret = 0;
  ret += parse_next(&path, &path_len, &context->object_id);
  ret += parse_next(&path, &path_len, &context->object_instance_id);
  ret += parse_next(&path, &path_len, &context->resource_id);

  /* Set default reader/writer */
  context->reader = &lwm2m_plain_text_reader;
  context->writer = &oma_tlv_writer;

  return ret;
} /* lwm2m_engine_register_default_objects() */


/*---------------------------------------------------------------------------*/
/*
* lwm2m_engine_get_object()
*/
const lwm2m_object_t* lwm2m_engine_get_object(uint16_t id)
{
  int i;
  for(i = 0; i < MAX_OBJECTS; i++) {
    if(_ctx.internal.objects[i] != NULL && _ctx.internal.objects[i]->id == id) {
      return _ctx.internal.objects[i];
    }
  }
  return NULL;
} /* lwm2m_engine_get_object() */


/*---------------------------------------------------------------------------*/
/*
* lwm2m_engine_register_object()
*/
int lwm2m_engine_register_object(const lwm2m_object_t *object)
{
  int i;
  int found = 0;
  for(i = 0; i < MAX_OBJECTS; i++) {
    if(_ctx.internal.objects[i] == NULL) {
      _ctx.internal.objects[i] = object;
      found = 1;
      break;
    }
  }
  rest_activate_resource(lwm2m_object_get_coap_resource(object),
                         (char *)object->path);
  return found;
} /* lwm2m_engine_register_object() */


/*---------------------------------------------------------------------------*/
/*
* lwm2m_engine_is_registered()
*/
int lwm2m_engine_is_registered(void)
{
  int ret = 0;

  switch( _ctx.state )
  {
  case E_LWM2M_ENGINE_STATE_READY:
  case E_LWM2M_ENGINE_STATE_UPDT:
  case E_LWM2M_ENGINE_STATE_UPDT_WAIT:
      ret = 1;
      break;

  default:
      ret = 0;
      break;
  }

  return ret;
} /* lwm2m_engine_is_registered() */


/*---------------------------------------------------------------------------*/
/*
* lwm2m_engine_handler()
*/
void lwm2m_engine_handler(const lwm2m_object_t *object,
                     void *request, void *response,
                     uint8_t *buffer, uint16_t preferred_size,
                     int32_t *offset)
{
  int len;
  const char *url;
  unsigned int format;
  unsigned int accept;
  unsigned int content_type;
  int depth;
  lwm2m_context_t context;
  rest_resource_flags_t method;
  const lwm2m_instance_t *instance;
#if (DEBUG) & DEBUG_PRINT
  const char *method_str;
#endif /* (DEBUG) & DEBUG_PRINT */

  method = REST.get_method_type(request);
  len = REST.get_url(request, &url);

  if(!REST.get_header_content_type(request, &format)) {
    PRINTF("No format given. Assume text plain...\n");
    format = TEXT_PLAIN;
  } else if(format == TEXT_PLAIN) {
    /* CoAP content format text plain - assume LWM2M text plain */
    format = TEXT_PLAIN;
  }
  if(!REST.get_header_accept(request, &accept)) {
    PRINTF("No Accept header, using same as Content-format...\n");
    accept = format;
  }

  depth = lwm2m_engine_parse_context(object, url, len, &context);
  PRINTF("Context: %u/%u/%u  found: %d\n", context.object_id,
         context.object_instance_id, context.resource_id, depth);

  /* Select reader and writer based on provided Content type and Accept headers */
  _lwm2m_engine_select_reader(&context, format);
  content_type = _lwm2m_engine_select_writer(&context, accept);


#if (DEBUG) & DEBUG_PRINT
  /* for debugging */
  if(method == METHOD_GET) {
    method_str = "GET";
  } else if(method == METHOD_POST) {
    method_str = "POST";
  } else if(method == METHOD_PUT) {
    method_str = "PUT";
  } else if(method == METHOD_DELETE) {
    method_str = "DELETE";
  } else {
    method_str = "UNKNOWN";
  }
  PRINTF("%s Called Path:%.*s Format:%d ID:%d bsize:%u\n", method_str, len,
         url, format, object->id, preferred_size);
  if(format == LWM2M_TEXT_PLAIN) {
    /* a string */
    const uint8_t *data;
    int plen = REST.get_request_payload(request, &data);
    if(plen > 0) {
      PRINTF("Data: '%.*s'\n", plen, (char *)data);
    }
  }
#endif /* (DEBUG) & DEBUG_PRINT */

  instance = _lwm2m_object_get_instance(object, &context, depth);

  /* from POST */
  if(depth > 1 && instance == NULL) {
    if(method != METHOD_PUT && method != METHOD_POST) {
      PRINTF("Error - do not have instance %d\n", context.object_instance_id);
      REST.set_response_status(response, NOT_FOUND_4_04);
      return;
    } else {
      const uint8_t *data;
      int i, len, plen, pos;
      oma_tlv_t tlv;
      PRINTF(">>> CREATE ? %d/%d\n", context.object_id,
             context.object_instance_id);

      lwm2m_instance_t *p_inst = object->p_instances;
      for(i = 0; i < (object->count) && (p_inst != NULL); i++) {
        if((p_inst->flag & LWM2M_INSTANCE_FLAG_USED) == 0) {
          /* allocate this instance */
          p_inst->flag |= LWM2M_INSTANCE_FLAG_USED;
          p_inst->id = context.object_instance_id;
          context.object_instance_index = i;
          PRINTF("Created instance: %d\n", context.object_instance_id);
          REST.set_response_status(response, CREATED_2_01);
          instance = p_inst;
          break;
        }
        p_inst = p_inst->p_next;
      }

      if(instance == NULL) {
        /* could for some reason not create the instance */
        REST.set_response_status(response, NOT_ACCEPTABLE_4_06);
        return;
      }

      plen = REST.get_request_payload(request, &data);
      if(plen == 0) {
        /* do nothing more */
        return;
      }
      PRINTF("Payload: ");
      for(i = 0; i < plen; i++) {
        PRINTF("%02x", data[i]);
      }
      PRINTF("\n");

      pos = 0;
      do {
        len = oma_tlv_read(&tlv, (uint8_t *)&data[pos], plen - pos);
        PRINTF("Found TLV type=%u id=%u len=%lu\n",
               tlv.type, tlv.id, (unsigned long)tlv.length);
        /* here we need to do callbacks or write value */
        if(tlv.type == OMA_TLV_TYPE_RESOURCE) {
          context.resource_id = tlv.id;
          const lwm2m_resource_t *rsc = _lwm2m_instance_get_resource(instance, &context);
          if(rsc != NULL) {
            /* write the value to the resource */
            if(lwm2m_object_is_resource_string(rsc)) {
              PRINTF("  new string value for /%d/%d/%d = %.*s\n",
                     context.object_id, context.object_instance_id,
                     context.resource_id, (int)tlv.length, tlv.value);
              lwm2m_object_set_resource_string(rsc, &context,
                                               tlv.length, tlv.value);
            } else if(lwm2m_object_is_resource_int(rsc)) {
              PRINTF("  new int value for /%d/%d/%d = %" PRId32 "\n",
                     context.object_id, context.object_instance_id,
                     context.resource_id, oma_tlv_get_int32(&tlv));
              lwm2m_object_set_resource_int(rsc, &context,
                                            oma_tlv_get_int32(&tlv));
            } else if(lwm2m_object_is_resource_floatfix(rsc)) {
              int32_t value;
              if(oma_tlv_float32_to_fix(&tlv, &value, LWM2M_FLOAT32_BITS)) {
                PRINTF("  new float value for /%d/%d/%d = %" PRId32 "\n",
                     context.object_id, context.object_instance_id,
                     context.resource_id, value >> LWM2M_FLOAT32_BITS);
                lwm2m_object_set_resource_floatfix(rsc, &context, value);
              } else {
                PRINTF("  new float value for /%d/%d/%d: FAILED\n",
                     context.object_id, context.object_instance_id,
                     context.resource_id);
              }
            } else if(lwm2m_object_is_resource_boolean(rsc)) {
              PRINTF("  new boolean value for /%d/%d/%d = %" PRId32 "\n",
                     context.object_id, context.object_instance_id,
                     context.resource_id, oma_tlv_get_int32(&tlv));
              lwm2m_object_set_resource_boolean(rsc, &context,
                                                oma_tlv_get_int32(&tlv) != 0);
            }
          }
        }
        pos = pos + len;
      } while(len > 0 && pos < plen);
    }
    return;
  }

  if(depth == 3) {
    const lwm2m_resource_t *resource = _lwm2m_instance_get_resource(instance, &context);
    size_t content_len = 0;
    if(resource == NULL) {
      PRINTF("Error - do not have resource %d\n", context.resource_id);
      REST.set_response_status(response, NOT_FOUND_4_04);
      return;
    }
    /* HANDLE PUT */
    if(method == METHOD_PUT) {
      if(lwm2m_object_is_resource_callback(resource)) {
        if(resource->value.callback.write != NULL) {
          /* pick a reader ??? */
          if(format == TEXT_PLAIN) {
            /* a string */
            const uint8_t *data;
            int plen = REST.get_request_payload(request, &data);
            context.reader = &lwm2m_plain_text_reader;
            PRINTF("PUT Callback with data: '%.*s'\n", plen, data);
            /* no specific reader for plain text */
            content_len = resource->value.callback.write(&context, data, plen,
                                                    buffer, preferred_size);
            PRINTF("content_len:%u\n", (unsigned int)content_len);
            REST.set_response_status(response, CHANGED_2_04);
          } else {
            PRINTF("PUT callback with format %d\n", format);
            REST.set_response_status(response, NOT_ACCEPTABLE_4_06);
          }
        } else {
          PRINTF("PUT - no write callback\n");
          REST.set_response_status(response, METHOD_NOT_ALLOWED_4_05);
        }
      } else {
        PRINTF("PUT on non-callback resource!\n");
        REST.set_response_status(response, METHOD_NOT_ALLOWED_4_05);
      }
      /* HANDLE GET */
    } else if(method == METHOD_GET) {

      if(lwm2m_object_is_resource_string(resource)) {
        const uint8_t *value;
        value = lwm2m_object_get_resource_string(resource, &context);
        if(value != NULL) {
          uint16_t len = lwm2m_object_get_resource_strlen(resource, &context);
          PRINTF("Get string value: %.*s\n", (int)len, (char *)value);
          content_len = context.writer->write_string(&context, buffer,
            preferred_size, (const char *)value, len);
        }
      } else if(lwm2m_object_is_resource_int(resource)) {
        int32_t value;
        if(lwm2m_object_get_resource_int(resource, &context, &value)) {
          content_len = context.writer->write_int(&context, buffer, preferred_size, value);
        }
      } else if(lwm2m_object_is_resource_floatfix(resource)) {
        int32_t value;
        if(lwm2m_object_get_resource_floatfix(resource, &context, &value)) {
          /* export FLOATFIX */
          PRINTF("Exporting %d-bit fix as float: %" PRId32 "\n",
                 LWM2M_FLOAT32_BITS, value);

          content_len = context.writer->write_float32fix(&context, buffer,
            preferred_size, value, LWM2M_FLOAT32_BITS);

        }
      } else if(lwm2m_object_is_resource_callback(resource)) {
        if(resource->value.callback.read != NULL) {
          content_len = resource->value.callback.read(&context,
                                                 buffer, preferred_size);
        } else {
          REST.set_response_status(response, METHOD_NOT_ALLOWED_4_05);
          return;
        }
      }

      if(content_len > 0) {
        REST.set_response_payload(response, buffer, content_len);
        REST.set_header_content_type(response, content_type);
      } else {
        /* failed to produce output - it is an internal error */
        REST.set_response_status(response, INTERNAL_SERVER_ERROR_5_00);
      }
      /* Handle POST */
    } else if(method == METHOD_POST) {
      if(lwm2m_object_is_resource_callback(resource)) {
        if(resource->value.callback.exec != NULL) {
          const uint8_t *data;
          int plen = REST.get_request_payload(request, &data);
          PRINTF("Execute Callback with data: '%.*s'\n", plen, data);
          content_len = resource->value.callback.exec(&context,
                                                 data, plen,
                                                 buffer, preferred_size);
          REST.set_response_status(response, CHANGED_2_04);
        } else {
          PRINTF("Execute callback - no exec callback\n");
          REST.set_response_status(response, METHOD_NOT_ALLOWED_4_05);
        }
      } else {
        PRINTF("Resource post but no callback resource\n");
        REST.set_response_status(response, METHOD_NOT_ALLOWED_4_05);
      }
    }
  } else if(depth == 2) {
    /* produce an instance response */
    if(method != METHOD_GET) {
      REST.set_response_status(response, METHOD_NOT_ALLOWED_4_05);
    } else if(instance == NULL) {
      REST.set_response_status(response, NOT_FOUND_4_04);
    } else {
      int rdlen;
      if(accept == APPLICATION_LINK_FORMAT) {
        rdlen = _lwm2m_wr_instance_resources_linkformat(object, instance,
                                   (char *)buffer, preferred_size);
        accept = APPLICATION_LINK_FORMAT;

      } else if(accept == LWM2M_JSON) {
        rdlen = _lwm2m_wr_instance_resources_json(&context, object, instance,
                                   (char *)buffer, preferred_size);
        accept = LWM2M_JSON;
      }
      else if( accept == LWM2M_TLV )
      {
        rdlen = _lwm2m_wr_instance_resources_tlv(&context, object, instance,
            (char *)buffer, preferred_size);
        accept = LWM2M_TLV;
      }
      if(rdlen < 0) {
        PRINTF("Failed to generate instance response\n");
        REST.set_response_status(response, SERVICE_UNAVAILABLE_5_03);
        return;
      }

      REST.set_response_payload(response, buffer, rdlen);
      REST.set_header_content_type(response, accept);

    }
  } else if(depth == 1) {
    /* produce a list of instances */
    if(method != METHOD_GET) {
      REST.set_response_status(response, METHOD_NOT_ALLOWED_4_05);
    } else {
      int rdlen;
      PRINTF("Sending instance list for object %u\n", object->id);
      /* TODO: if(accept == APPLICATION_LINK_FORMAT) { */
      rdlen = _lwm2m_wr_object_instances_linkformat(object, (char *)buffer, preferred_size);
      if(rdlen < 0) {
        PRINTF("Failed to generate object response\n");
        REST.set_response_status(response, SERVICE_UNAVAILABLE_5_03);
        return;
      }
      REST.set_header_content_type(response, REST.type.APPLICATION_LINK_FORMAT);
      REST.set_response_payload(response, buffer, rdlen);
    }
  }
} /* lwm2m_engine_is_registered() */


/*---------------------------------------------------------------------------*/
/*
* lwm2m_engine_delete_handler()
*/
void
lwm2m_engine_delete_handler(const lwm2m_object_t *object, void *request,
                            void *response, uint8_t *buffer,
                            uint16_t preferred_size, int32_t *offset)
{
  int len;
  const char *url;
  lwm2m_context_t context;

  len = REST.get_url(request, &url);
  PRINTF("*** DELETE URI:'%.*s' called... - responding with DELETED.\n",
         len, url);
  len = lwm2m_engine_parse_context(object, url, len, &context);
  PRINTF("Context: %u/%u/%u  found: %d\n", context.object_id,
         context.object_instance_id, context.resource_id, len);

  REST.set_response_status(response, DELETED_2_02);
} /* lwm2m_engine_delete_handler() */
