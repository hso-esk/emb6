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
 *  \file       lwm2mapi.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      LWM2M API.
 *
 *              The LWM2M API defines a function set on top of the serial
 *              API. It allows to control the LWM2M application layer e.g.
 *              to create, delete or access resources.
 */


/*
 * --- Includes -------------------------------------------------------------*
 */
#include "emb6.h"
#include "bsp.h"
#include "evproc.h"
#include "memb.h"
#include "lwm2mapi.h"
#include "lwm2m-device.h"
#include "lwm2m-engine.h"
#include "lwm2m-object.h"


/*
 *  --- Macros --------------------------------------------------------------*
 */

/** Maximum number of LWM2M objects */
#define LWM2MAPI_OBJ_MAX                5
/** Maximum number of LWM2M instances */
#define LWM2MAPI_INST_MAX               5
/** Maximum number of LWM2M resources */
#define LWM2MAPI_RES_MAX                5

#ifdef LWM2M_USE_BOOTSTRAP
#define LWM2M_BOOTSTRAP_SERVER_IP(ipaddr)   uip_ip6addr(ipaddr,  0xbbbb, 0x0000, 0x0000, 0x0000, 0x000a, 0x0bff,0xfe0c, 0x0d0e)
#define LWM2M_BOOTSTRAP_SERVER_PORT         UIP_HTONS(5583)
#else
#define LWM2M_SERVER_IP(ipaddr)             uip_ip6addr(ipaddr, 0xbbbb, 0x0000, 0x0000, 0x0000,0x6eec, 0xebff,0xfe67, 0xea04)
#define LWM2M_SERVER_PORT                   UIP_HTONS(5683)
#endif /* #ifdef LWM2M_USE_BOOTSTRAP /* #ifdef LWM2M_USE_BOOTSTRAP */


/*
 *  --- Enumerations --------------------------------------------------------*
 */


/**
 * \brief   LWM2M API types.
 */
typedef enum
{
  /** Initialize the LWM2M communication. A host has to issue this command
    * before configuring/starting the LWM2M layer. */
  e_lwm2m_api_type_ret = 0x00,

  /** Initialize the LWM2M communication. A host has to issue this command
    * before configuring/starting the LWM2M layer. */
  e_lwm2m_api_type_init = 0x05,

  /** Set a configuration parameter. */
  e_lwm2m_api_type_cfg_set = 0x10,

  /** Get a configuration parameter. */
  e_lwm2m_api_type_cfg_get,

  /** return a configuration parameter. */
  e_lwm2m_api_type_cfg_rsp,

  /** Initiate a bootstrap procedure. The client uses the bootstrap
    * configuration to connect to the bootstrap server. */
  e_lwm2m_api_type_bs = 0x20,

  /** Start the LWM2M communication. The client tries to register
    * at the configured server. */
  e_lwm2m_api_type_start,

  /** Create a LWM2M object. */
  e_lwm2m_api_type_obj_create = 0x30,

  /** Delete a previously created object. The host must use a
    * resource ID returned by a previous create operation. */
  e_lwm2m_api_type_obj_delete,

  /** Returns an ID of a previously created L2M2M object. The
    * host uses this ID for further actions related to the
    * according resource (e.g. deletion). */
  e_lwm2m_api_type_obj_ret,

  /** Read request initiated by the device. The Device calls this
   * function whenever it receives an according request from the
   * associated LWM2M server. */
  e_lwm2m_api_type_rd_req = 0x40,

  /** The host has to answer to a LWM2M2_RD_REQ using a
    * LWM2M2_RD_RSP. */
  e_lwm2m_api_type_rd_rsp,

  /** Write request to a LWM2M resource. Both device and host use this
    * command e.g. to set configurations or stored values. */
  e_lwm2m_api_type_wr_req,

  /** The host/device has to answer to a LWM2M2_WR_REQ using a
    * LWM2M2_WR_RSP. */
  e_lwm2m_api_type_wr_rsp,

  /** Get the Status of the LWM2M module.  */
  e_lwm2m_api_type_status_get = 0x50,

  /** Returns the status of the LWM2M module. */
  e_lwm2m_api_type_status_ret,

  e_lwm2m_api_type_max

} e_lwm2m_api_type_t;


/**
 * \brief   Return and status codes.
 */
typedef enum
{
  /** Describes a positive return value e.g. after
    * a command was issued. */
  e_lwm2m_api_ret_ok = 0x00,

  /** The command is not valid or supported. */
  e_lwm2m_api_ret_error_cmd,

  /** The parameters are invalid or not supported. */
  e_lwm2m_api_ret_error_param,

  /** Describes a positive return value e.g. after
    * a command was issued. */
  e_lwm2m_api_status_ok = 0x20,

  /** LWM2M is performing bootstrapping */
  e_lwm2m_api_status_boot,

  /** LWM2M module has started. */
  e_lwm2m_api_status_started,

  /** The LWM2M module is registered at the server that was configured
    * or the server retrieved via bootstrapping. */
  e_lwm2m_api_status_registered,

} e_lwm2m_api_ret_t;


/**
 * \brief   Specific configuration IDs.
 *
 *          The CFG_GET/SET command allow setting or reading the actual
 *          configuration. Therefore specific identifiers are required.
 */
typedef enum
{
  /** IP address of the LWM2M bootstrap server. */
  e_lwm2m_api_cfgid_bssrvip,

  /** Port of the LWM2M bootstrap server. */
  e_lwm2m_api_cfgid_bssrvport,

  /** Port of the LWM2M bootstrap server. */
  e_lwm2m_api_cfgid_srvip,

  /** Port of the LWM2M bootstrap server. */
  e_lwm2m_api_cfgid_srvport,

  /** Name of the LWM2M client. The server instance will
    * use this name for identification. */
  e_lwm2m_api_cfgid_cliname

} e_serial_api_cfgid_t;

/*
 *  --- Type Definitions -----------------------------------------------------*
 */

/** frameID */
typedef uint8_t lwm2mapi_frameID_t;

/** IP address address configuration*/
typedef uip_ipaddr_t lwm2mapi_cfg_ipaddr_t;

/** Port configuration */
typedef uint16_t lwm2mapi_cfg_port_t;

/** Client Name configuration */
typedef char* lwm2mapi_cfg_cliname_t;


/**
 * \brief   Definition of a callback function for commands received.
 *
 *          This function is used to define a handler used for the reception
 *          and handling of a specific command.
 *
 * \param   p_cmd   Buffer containing the command payload.
 * \param   cmdLen  Length of the command.
 * \param   Buffer for the response.
 * \param   Length of the buffer.
 *
 * \return Length of the generated response.
 */
typedef uint16_t (*fn_serialApiHndl_t)( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */


/** Receive data.
 * For further details have a look at the function definitions. */
static int8_t _rx_data( uint8_t* p_data, uint16_t len );


/** Callback function in case a INIT command was received. For further
 * details have a look at the function definition.*/
static uint16_t _hndl_init( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );


/** Generic LWM2M handler to GET a resource. For furhther information
 * refer to the function definition. */
static void lwm2m_get(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user);

/** Generic LWM2M handler to PUT a resource. For furhther information
 * refer to the function definition. */
static void lwm2m_put(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user);

/** Generic LWM2M handler to POST a resource. For furhther information
 * refer to the function definition. */
static void lwm2m_post(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user);

/** Generic LWM2M handler to DELETE a resource. For furhther information
 * refer to the function definition. */
static void lwm2m_delete(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user);


/*
 *  --- Local Variables ---------------------------------------------------- *
 */

/** Pointer to the TX function */
static void(*_fn_tx)(uint16_t len, void* p_param) = NULL;

/** Pointer to the Tx buffer */
static uint8_t* _p_txBuf = NULL;
/** Length of the Tx buffer */
static uint16_t _txBufLen = 0;
/** Tx parameter */
static void* _p_txParam = NULL;

/** Storage for Rest Resources */
MEMB(lwm2mrestres_storage, resource_t, LWM2MAPI_OBJ_MAX );
/** Storage for LWM2M Objects */
MEMB(lwm2mobject_storage, lwm2m_object_t, LWM2MAPI_OBJ_MAX );
/** Storage for LWM2M Instances */
MEMB(lwm2minstance_storage, lwm2m_instance_t, LWM2MAPI_INST_MAX);
/** Storage for LWM2M Resources */
MEMB(lwm2mresource_storage, lwm2m_resource_t, LWM2MAPI_RES_MAX);


/** server IP address */
static uip_ipaddr_t server_ipaddr;


/*
 *  --- Local Functions ---------------------------------------------------- *
 */

/**
 * \brief   This event is raised by the serial MAC if a frame was received.
 *
 *          Everytime a full and valid frame was received by the serial
 *          MAC it raises such an event.
 *
 * \param   p_data      Payload of the frame.
 * \param   len         Length of the frame.
 */
static int8_t _rx_data( uint8_t* p_data, uint16_t len )
{
  lwm2mapi_frameID_t id;
  size_t bufLeft = len;
  uint8_t* p_dataPtr = p_data;

  fn_serialApiHndl_t f_hndl = NULL;
  uint16_t hndlRet = 0;

  /* A frame has been received */
  /* check parameters */
  EMB6_ASSERT_RET( (p_dataPtr != NULL), -1 );
  EMB6_ASSERT_RET( (bufLeft >= sizeof(lwm2mapi_frameID_t)), -1 );

  /* get ID */
  id = *p_dataPtr;
  p_dataPtr += sizeof(lwm2mapi_frameID_t);
  bufLeft -= sizeof(lwm2mapi_frameID_t);

  switch( id )
  {
    /* A ping was requested from the host. Reply with a simple
     * RET frame to indicate that the device is alive. */
    case e_lwm2m_api_type_init:
      f_hndl = _hndl_init;
      break;


    default:
      break;
  }

  /* call the according handler */
  if( f_hndl != NULL )
  {
    hndlRet = f_hndl( p_dataPtr, bufLeft, _p_txBuf, _txBufLen );
    if( hndlRet )
    {
        EMB6_ASSERT_RET( _fn_tx != NULL, -1 );
        /* Call the associated Tx function with the according
         * parameter. */
        _fn_tx( hndlRet, _p_txParam );
        return 0;
    }
  }
  else
  {
    /* The command was not found */
    uint8_t* p_txBuf = _p_txBuf;
    *((lwm2mapi_frameID_t*)p_txBuf) = e_lwm2m_api_type_ret;
    p_txBuf += sizeof(lwm2mapi_frameID_t);
    e_lwm2m_api_ret_t* p_ret = (e_lwm2m_api_ret_t*)p_txBuf;
    p_txBuf += sizeof(e_lwm2m_api_ret_t);
    *p_ret = e_lwm2m_api_ret_error_cmd;

    EMB6_ASSERT_RET( _fn_tx != NULL, -1 );
    _fn_tx( (p_txBuf - _p_txBuf), _p_txParam );
    return 0;
  }

  return -1;

}


/**
 * \brief   Callback to initialize LWM2M.
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static uint16_t _hndl_init( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
    /* Initialization was requested. Reset all the internal
     * parameters and prepare for operation. */

    EMB6_ASSERT_RET( p_cmd != NULL, 0 );
    EMB6_ASSERT_RET( p_rpl != NULL, 0 );

    uint8_t* p_txBuf = p_rpl;

    /* set the according id */
    *((lwm2mapi_frameID_t*)p_txBuf) = e_lwm2m_api_ret_ok;
    p_txBuf += sizeof(lwm2mapi_frameID_t);

    return p_txBuf - p_rpl;

  return 0;
}


/**
 * \brief   Generic LWM2M handler to GET a resource.
 *
 *          This function is called from the underlying CoAP engine
 *          if a GET access to a resource was requested. The target
 *          of the request can be seen from the parameters.
 *          The request will be forwarded to the LWM2M engine.
 *
 */
static void lwm2m_get(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user)
{
  /* forward request */
  lwm2m_engine_handler((lwm2m_object_t*)p_user, request, response, buffer,
      preferred_size, offset);
}


/**
 * \brief   Generic LWM2M handler to PUT a resource.
 *
 *          This function is called from the underlying CoAP engine
 *          if a PUT access to a resource was requested. The target
 *          of the request can be seen from the parameters.
 *          The request will be forwarded to the LWM2M engine.
 *
 */
static void lwm2m_put(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user)
{
  /* forward request */
  lwm2m_engine_handler((lwm2m_object_t*)p_user, request, response, buffer,
      preferred_size, offset);
}


/**
 * \brief   Generic LWM2M handler to POST a resource.
 *
 *          This function is called from the underlying CoAP engine
 *          if a POST access to a resource was requested. The target
 *          of the request can be seen from the parameters.
 *          The request will be forwarded to the LWM2M engine.
 *
 */
static void lwm2m_post(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user)
{
  /* forward request */
  lwm2m_engine_handler((lwm2m_object_t*)p_user, request, response, buffer,
      preferred_size, offset);
}


/**
 * \brief   Generic LWM2M handler to DELETE a resource.
 *
 *          This function is called from the underlying CoAP engine
 *          if a DELETE access to a resource was requested. The target
 *          of the request can be seen from the parameters.
 *          The request will be forwarded to the LWM2M engine.
 *
 */
static void lwm2m_delete(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user)
{
  /* forward request */
  lwm2m_engine_delete_handler((lwm2m_object_t*)p_user, request, response, buffer,
      preferred_size, offset);
}

/*
 *  --- Global Functions  ---------------------------------------------------*
 */

/*---------------------------------------------------------------------------*/
/*
* lwm2mApiInit()
*/
int8_t lwm2mApiInit( uint8_t* p_txBuf, uint16_t txBufLen,
        void(*fn_tx)(uint16_t len, void* p_param), void* p_txParam )
{
    int8_t ret = 0;

    /* set buffer description */
    _p_txBuf = p_txBuf;
    _txBufLen = txBufLen;
    _p_txParam = p_txParam;

    /* set Tx function */
    _fn_tx = fn_tx;

    /* initialize memory */
    memb_init( &lwm2mrestres_storage );
    memb_init( &lwm2mobject_storage );
    memb_init( &lwm2minstance_storage );
    memb_init( &lwm2mresource_storage );

#ifdef LWM2M_USE_BOOTSTRAP
    LWM2M_BOOTSTRAP_SERVER_IP(&server_ipaddr);
    lwm2m_engine_use_bootstrap_server(1);
    lwm2m_engine_register_with_bootstrap_server( &server_ipaddr, LWM2M_BOOTSTRAP_SERVER_PORT );
#else
    LWM2M_SERVER_IP(&server_ipaddr);
    lwm2m_engine_use_registration_server(1);
    lwm2m_engine_register_with_server( &server_ipaddr, LWM2M_SERVER_PORT );
#endif /* #ifdef LWM2M_USE_BOOTSTRAP */

    /* Initialize the OMA LWM2M engine */
    lwm2m_engine_init();

    /* Register default LWM2M objects */
    lwm2m_engine_register_default_objects();

    /* TEST */
    {
      ret = -1;

      /* allocate memory for a resource and an object */
      lwm2m_object_t* p_obj = (lwm2m_object_t* )memb_alloc( &lwm2mobject_storage );
      if( p_obj != NULL )
      {
        resource_t* p_rest = (resource_t* )memb_alloc( &lwm2mrestres_storage );
        *p_rest = (resource_t){NULL, NULL, HAS_SUB_RESOURCES | IS_OBSERVABLE, NULL,
          lwm2m_get, lwm2m_post, lwm2m_put, lwm2m_delete, {NULL}, p_obj };

        *p_obj = (lwm2m_object_t){ 3303, 0, LWM2M_OBJECT_PATH_STR(3303), p_rest, NULL};

        /* allocate memory for an instance */
        lwm2m_instance_t* p_inst = (lwm2m_instance_t*)memb_alloc( &lwm2minstance_storage );
        if( p_inst != NULL )
        {
          *p_inst = (lwm2m_instance_t) {0, 0, LWM2M_INSTANCE_FLAG_USED, NULL};


          /* allocate memory for the resources */
          lwm2m_resource_t* p_res = memb_allocm( &lwm2mresource_storage, 5 );

          if( p_res != NULL )
          {
            /* add resources to instance */
            p_inst->count = 5;
            p_inst->resources = p_res;

            /* create the resources */
            *p_res = (lwm2m_resource_t) {5700, LWM2M_RESOURCE_TYPE_FLOATFIX_VALUE,
              .value.floatfix.value = (16.3 * LWM2M_FLOAT32_FRAC)};
            p_res++;

            /* create the resources */
            *p_res = (lwm2m_resource_t) {5601, LWM2M_RESOURCE_TYPE_FLOATFIX_VALUE,
              .value.floatfix.value = (8.7 * LWM2M_FLOAT32_FRAC)};
            p_res++;

            /* create the resources */
            *p_res = (lwm2m_resource_t) {5602, LWM2M_RESOURCE_TYPE_FLOATFIX_VALUE,
              .value.floatfix.value = (26.5 * LWM2M_FLOAT32_FRAC)};
            p_res++;

            /* create the resources */
            *p_res = (lwm2m_resource_t) {5603, LWM2M_RESOURCE_TYPE_FLOATFIX_VALUE,
              .value.floatfix.value = (0.0 * LWM2M_FLOAT32_FRAC)};
            p_res++;

            /* create the resources */
            *p_res = (lwm2m_resource_t) {5604, LWM2M_RESOURCE_TYPE_FLOATFIX_VALUE,
              .value.floatfix.value = (100.0 * LWM2M_FLOAT32_FRAC)};
            p_res++;

            ret = 0;
          }


          if( ret != 0 )
            /* an error occurred and the instance must be deleted */
            free( p_inst );
          else
          {
            /* assign instance */
            p_obj->instances = p_inst;
            p_obj->count = 1;
          }

        }

        if( ret != 0 )
          /* an error occurred and the object must be deleted */
          free( p_obj );
        else
          /* register object  */
          lwm2m_engine_register_object( p_obj );
      }

    }
    /** TEST END */

    return ret;
} /* lwm2mApiInit() */


/*---------------------------------------------------------------------------*/
/*
* lwm2mApiInput()
*/
int8_t lwm2mApiInput( uint8_t* p_data, uint16_t len, uint8_t valid )
{
    int ret = -1;

    if( valid )
    {
        /* process the frame */
        ret = _rx_data( p_data, len );
    }

    return ret;
} /* lwm2mApiInput() */

