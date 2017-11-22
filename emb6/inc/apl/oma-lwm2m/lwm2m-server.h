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
 *  \file       lwm2m-object-counter.h
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      LWM2M Server Object.
 *
 */
#ifndef __LWM2M_SERVER_H__
#define __LWM2M_SERVER_H__

/*
 * --- Includes -------------------------------------------------------------*
 */
#include <stdint.h>
#include "lwm2m-engine.h"
#include "lwm2m-object.h"


/*
 *  --- Global Functions Definition ------------------------------------------*
 */

/**
 * \brief   Initializes the LWM2M Object(s).
 *
 *          This function initializes all the objects that have been
 *          defined statically. This means that the objects will be
 *          created and registered at the LWM2M engine.
 *
 * \return  0 on success, otherwise -1
 */
int8_t lwm2m_server_init( void );


/**
 * \brief   Get the LWM2M client lifetime.
 *
 *          LWM2M clients have a specific lifetime. After this lifetime
 *          has expired they must update the registration at the server
 *          otherwise they get deleted.
 *
 * \return  Actual lifetime.
 */
int32_t lwm2m_server_getLifetime( void );


/**
 * \brief   Set the LWM2M client lifetime.
 *
 *          LWM2M clients have a specific lifetime. After this lifetime
 *          has expired they must update the registration at the server
 *          otherwise they get deleted.
 *
 */
void lwm2m_server_setLifetime( int32_t lt );


/**
 * \brief   Get the LWM2M binding.
 *
 *          LWM2M supports different bindings This function returns
 *          the currently configured binding.
 *
 * \return  Binding as string.
 */
char* lwm2m_server_getBinding( void );


#endif /* __LWM2M_SERVER_H__ */

