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
 *  \file       lwm2mapi.h
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
#ifndef __LWM2MAPI_H__
#define __LWM2MAPI_H__

/*
 * --- Includes -------------------------------------------------------------*
 */
#include <stdint.h>
#include "emb6.h"
#include "lwm2m-object.h"


/*
 *  --- Macros --------------------------------------------------------------*
 */
#define LWM2MAPI_ACCESS_READ              0
#define LWM2MAPI_ACCESS_WRITE             1


/*
 *  --- Type Definitions -----------------------------------------------------*
 */

/**
 * \brief   Callback that is issued when ever a Resource was accessed.
 *
 * \param   objID   Object Id of the accessed resource.
 * \param   instId  Instance Id of the accessed resource.
 * \param   resID   Resource Id of the accessed resource.
 * \param   type    Type of the resource.
 * \param   val     Value that has been written.
 * \param   len     Length of the value.
 * \param   p_user  User related data.
 */
typedef void(*f_lwm2m_resource_access_cb)( uint16_t objID, uint16_t instId, uint16_t resID,
    uint8_t type, void* val, uint16_t len, void* p_user  );


/*
 *  --- Global Functions Definition ------------------------------------------*
 */

/**
 * \brief   Initializes the LWM2M API.
 *
 *          This function is used to initialize the LWM2M API. The
 *          initialization also provides pointers to the Tx buffer
 *          and the Tx function. This is required in case a reply
 *          needs to be generated or an unsolicited message has to
 *          be sent.
 *
 * \param   p_txBuf     Pointer to the Tx buffer.
 * \param   txBufLen    Lenth of the Tx buffer.
 * \patam   fn_tx       Tx function.
 * \param   p_txParam   Parameter used for Tx function.
 *
 * \return  0 on success, otherwise -1
 */
int8_t lwm2mApiInit( uint8_t* p_txBuf, uint16_t txBufLen,
        void(*fn_tx)(uint16_t len, void* p_param), void* p_txParam );

/**
 * \brief   Provide input to the LWM2M API.
 *
 *          This function can be used in case a frame was received
 *          that was directed to the LWM2M API.
 *
 * \param   p_data      Payload of the frame.
 * \param   len         Length of the frame.
 * \param   valid       Defines if a payload is valid or not (e.g. CRC error)
 *
 * \return  0 in case the frame was processed succesfully or a negative value
 *          in case of an error.
 */
int8_t lwm2mApiInput( uint8_t* p_data, uint16_t len, uint8_t valid );


#endif /* __LWM2MAPI_H__ */

