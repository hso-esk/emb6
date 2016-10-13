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
/*!
    \file   mle_cmd.h

    \author Nidhal Mars <nidhal.mars@hs-offenburg.de>

    \brief  Functions to handle MLE command

  	\version  0.1
*/


#ifndef  __MLE_CMD_H_
#define  __MLE_CMD_H_

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#include "emb6.h"
#include "tlv.h"

/*==============================================================================
                                     MACROS
 =============================================================================*/


#define MAX_TLV_DATA_SIZE         255


/*==============================================================================
									TYPEDEFS
 =============================================================================*/

/**
* MLE COMMAND type enum
*/
typedef enum 
{
	LINK_REQUEST,
	LINK_ACCEPT,
	LINK_ACCEPT_AND_REQUEST,
	LINK_REJECT,
	ADVERTISEMENT,
	UPDATE,                    // Not used in Thread Network
	UPDATE_REQUEST,            // Not used in Thread Network
	DATA_REQUEST,
	DATA_RESPONSE,
	PARENT_REQUEST,
	PARENT_RESPONSE,
	CHILD_ID_REQUEST,
	CHILD_ID_RESPONSE,
	CHILD_UPDATE,
	CHILD_UPDATE_RESPONSE,
	CMD_NONE,
} mle_cmd_type_t;

/**
* MLE COMMAND structure
*/
typedef struct __attribute__((__packed__)) {
	/* command type */
	uint8_t    type;
	/* TLV data buffer */
	uint8_t    tlv_data[MAX_TLV_DATA_SIZE];
	/* counter for the used buffer */
	uint8_t    used_data;
}mle_cmd_t ;

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

/**
 * @brief  initialize mle command
 *
 * @param  mle_cmd	  pointer to mle_cmd structure
 * @param  type	      type of  mle command
 */

void  mle_init_cmd( mle_cmd_t* mle_cmd, const mle_cmd_type_t type );


/**
 * @brief  create  mle command from buffer
 *
 * @param  mle_cmd	  pointer to mle_cmd structure
 * @param  data	      pointer to the data
 * @param  datalen	  data length
 */
void  mle_create_cmd_from_buff( mle_cmd_t** cmd, uint8_t* data, uint16_t datalen );


/**
 * @brief  add tlv to mle command
 *
 * @param  mle_cmd	  pointer to mle_cmd structure

 * @return
         -  1 sucess
         -  0 error
 */
uint8_t mle_add_tlv_to_cmd( mle_cmd_t * mle_cmd, const tlv_type_t type, const int8_t length,  uint8_t * value);


/**
 * @brief  find tlv type in cmd
 *
 * @param  cmd	          MLE command
 * @param  type	  		  type of tlv to looking for
 *
 * @return pointer to the tlv if exist
 *         otherwise NULL
 */
tlv_t* mle_find_tlv_in_cmd( mle_cmd_t * cmd, const tlv_type_t type);



/**
 * @brief  print buffer
 *
 * @param  buffer	  pointer to buffer
 */
void print_buffer( uint8_t* buffer, uint8_t length );



/**
 * @brief  print mle command
 *
 * @param  mle_cmd	  mle_cmd structure
 *
 * @return
 *       -  1 sucess
 *       -  0 error
 */
uint8_t mle_print_cmd( mle_cmd_t mle_cmd );


/**
 * @brief  print mle command type
 *
 * @param  mle_cmd	  mle_cmd structure
 *
 * @return
 *       -  1 sucess
 *       -  0 error
 */
uint8_t mle_print_type_cmd( mle_cmd_t cmd );


#endif /* __MLE_CMD_H_ */
