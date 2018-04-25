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
 *  \file       lwm2m-server.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      LWM2M Server Object.
 *
 */

/*
 * --- Includes -------------------------------------------------------------*
 */
#include <stdint.h>
#include "lwm2m-object.h"
#include "lwm2m-engine.h"

/*
 *  --- Macros --------------------------------------------------------------*
 */

/** Default Lifetime */
#define LWM2M_SERVER_LIFETIME_DEFAULT   86400
/** Default bindig */
#define LWM2M_SERVER_BINDING            "U"

/*
 *  --- Local Function Prototypes  ------------------------------------------*
 */

/* Request for registration update */
static int exec_regupdate( lwm2m_context_t *ctx, const uint8_t *arg, size_t len,
             uint8_t *outbuf, size_t outlen );

/*
 *  --- Local Variables  ----------------------------------------------------*
 */

/** Server ID */
static int32_t sid;
/** Lifetime */
static int32_t lifetime;
/** Default minimum period */
static int32_t pmin;
/** Default minimum period */
static int32_t pmax;


/** Create Resources fo the server object */
LWM2M_RESOURCES(server_resources,
		LWM2M_RESOURCE_INTEGER_VAR(0, &sid),
		LWM2M_RESOURCE_INTEGER_VAR(1, &lifetime),
        LWM2M_RESOURCE_INTEGER_VAR(2, &pmin),
        LWM2M_RESOURCE_INTEGER_VAR(3, &pmax),
        LWM2M_RESOURCE_STRING(7, LWM2M_SERVER_BINDING ),
        LWM2M_RESOURCE_CALLBACK(8, LWM2M_RESOURCE_TYPE_CALLBACK,
            { NULL, NULL, exec_regupdate, NULL } )
                );

/** Only one instance is upported */
LWM2M_INSTANCES(server_instances,
                LWM2M_INSTANCE(0, server_resources));

/** Creat the according server object */
LWM2M_OBJECT(server, 1, server_instances);


/*
 *  --- Local Functions  ----------------------------------------------------*
 */


/**
 * \brief   Callback function to trigger a registration update.
 *
 *          This function can be called by the server to trigger an
 *          update of the registration.
 *
 * \return  0 on success, otherwise -1
 */
static int exec_regupdate( lwm2m_context_t *ctx, const uint8_t *arg, size_t len,
    uint8_t *outbuf, size_t outlen )
{
  /* not supported yet */
  return 0;
}

/*
 *  --- Global Functions  ---------------------------------------------------*
 */

/*---------------------------------------------------------------------------*/
/*
* lwm2m_server_init()
*/
int8_t lwm2m_server_init( void )
{
  int ret = -1;

  /* set default lifetime */
  lifetime = LWM2M_SERVER_LIFETIME_DEFAULT;

  /**
   * Register this device and its handlers - the handlers
   * automatically sends in the object to handle
   */
  LWM2M_INIT_OBJECT((&server));
  lwm2m_engine_register_object(&server);

  ret = 0;
  return ret;

} /* lwm2m_server_init() */

/*---------------------------------------------------------------------------*/
/*
* lwm2m_server_getLifetime()
*/
int32_t lwm2m_server_getLifetime( void )
{
    return lifetime;
} /* lwm2m_server_getLifetime() */

/*---------------------------------------------------------------------------*/
/*
* lwm2m_server_setLifetime()
*/
void lwm2m_server_setLifetime( int32_t lt )
{
    lifetime = lt;
} /* lwm2m_server_getLifetime() */

/*---------------------------------------------------------------------------*/
/*
* lwm2m_server_getBinding()
*/
char* lwm2m_server_getBinding( void )
{
    return LWM2M_SERVER_BINDING;
} /* lwm2m_server_getBinding() */
