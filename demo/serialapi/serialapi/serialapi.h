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
 *  \file       serialapi.h
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Serial API.
 *
 *              The serial API is used to control the stack via an external
 *              interface such as UART or Ethernet. This module does not
 *              include the lower layer handling such as access to the physical
 *              interface or protocol handling.
 */
#ifndef __SERIALAPI_H__
#define __SERIALAPI_H__

/*
 * --- Includes -------------------------------------------------------------*
 */
#include <stdint.h>
#include "emb6.h"


/*
 *  --- Type Definitions -----------------------------------------------------*
 */

/** frameID */
typedef uint8_t serialapi_frameID_t;

/** Serial API Init function */
typedef int8_t(*fn_serial_ApiInit_t)( uint8_t* p_txBuf, uint16_t txBufLen,
        void(*fn_tx)(uint16_t len, void* p_param), void* p_txParam );

/** Serial API Input function */
typedef int8_t(*fn_serial_ApiInput_t)( uint8_t* p_data, uint16_t len,
        uint8_t valid );


/*
 *  --- Global Functions Definition ------------------------------------------*
 */

/**
 * \brief   Initializes the serial API.
 *
 *          This function is used to initialize the serial API. The
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
int8_t serialApiInit( uint8_t* p_txBuf, uint16_t txBufLen,
        void(*fn_tx)(uint16_t len, void* p_param), void* p_txParam );

/**
 * \brief   Provide input to the serial API.
 *
 *          This function can be used in case a frame was received
 *          that was directed to the serial API.
 *
 * \param   p_data      Payload of the frame.
 * \param   len         Length of the frame.
 * \param   valid       Defines if a payload is valid or not (e.g. CRC error)
 *
 * \return  0 in case the frame was processed succesfully or a negative value
 *          in case of an error.
 */
int8_t serialApiInput( uint8_t* p_data, uint16_t len, uint8_t valid );


/**
 * \brief   Register a handler for a specific type of frame ID.
 *
 *          This function can be used to register a separate module to
 *          handle a specific type of frames.
 *
 * \param   id      ID to handle.
 * \param   pf_in   Handler to register.
 */
int8_t serialApiRegister( serialapi_frameID_t id,
        fn_serial_ApiInit_t pf_init, fn_serial_ApiInput_t pf_in );


#endif /* __SERIALAPI_H__ */

