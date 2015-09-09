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
/*============================================================================*/
/**
 * 	 \addtogroup utils
 * 	 @{
 *   \addtogroup hexdump Printout data in a standard hex view
 *   @{
*/
/*============================================================================*/
/*! \file   hexdump.h

    \author Original-grapsus http://grapsus.net/blog/post/Hexadecimal-dump-in-C
    \author Updated by: Artem Yushev 
    \breif  Printout data in a standard hex view
	\version 0.1
*/
/*============================================================================*/
#ifndef HEXDUMP_H_
#define HEXDUMP_H_

/*==============================================================================
                      	  	       INCLUDES
==============================================================================*/

/*=============================================================================
                                BASIC CONSTANTS
==============================================================================*/

/*==============================================================================
                                     MACROS
 =============================================================================*/

/*==============================================================================
                                     ENUMS
==============================================================================*/

/*==============================================================================
                          SYSTEM STRUCTURES AND OTHER TYPEDEFS
 =============================================================================*/

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/
/*!
\brief   Printout data in a standard hex view

\param    p_buf        Pointer to data which should be printed out.
\param    l_len        Length of a data

\return   None
\example
0x000000: 2e 2f 68 65 78 64 75 6d ./hexdum
0x000008: 70 00 53 53 48 5f 41 47 p.SSH_AG
0x000010: 45 4e 54 5f             ENT_
*/
void hexdump(const void* p_buf, uint32_t l_len);

#endif /* HEXDUMP_H_ */
/** @} */
/** @} */
