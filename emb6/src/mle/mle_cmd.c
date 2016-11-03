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
    \file   mle_cmd.c

    \author Nidhal Mars <nidhal.mars@hs-offenburg.de>

    \brief  Functions to handle MLE command

  	\version  0.1
*/


/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#include "mle_cmd.h"
#include "string.h"


/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

void  mle_init_cmd( mle_cmd_t* mle_cmd, const mle_cmd_type_t type )
{
	mle_cmd->type= (uint8_t) type;
	mle_cmd->used_data=0;
}

void  mle_create_cmd_from_buff( mle_cmd_t** cmd, uint8_t* data, uint16_t datalen )
{
	*cmd=(mle_cmd_t*) data;
	(*cmd)->used_data=datalen-1;

}


tlv_t* mle_find_tlv_in_cmd(mle_cmd_t * cmd , const tlv_type_t type )
{
	tlv_t * tlv;
	uint8_t i=0;

	tlv_init(&tlv,cmd->tlv_data);
	while( tlv < (tlv_t*)&cmd->tlv_data[cmd->used_data])
	{
		if (tlv->type==type )
		{
			return  tlv;
		}
		i+=(tlv->length+2);
		tlv_init(&tlv,&cmd->tlv_data[i]);
	}
	return NULL;
}

uint8_t mle_add_tlv_to_cmd( mle_cmd_t * cmd, const tlv_type_t type, const int8_t length,   uint8_t * value )
{
	tlv_t * tlv;
	uint8_t i=0;

	/* carry about data buffer overflow*/
	if ((cmd->used_data+ length +sizeof(tlv->type) + sizeof(tlv->length)) >= MAX_TLV_DATA_SIZE)
	{
		printf("Buffer overflow \n");
		return 0;
	};

	/*
	 * an MLE message MUST NOT contain two or more TLVs of the same type
	 * With the exceptions of the Source Address TLV and Parameter TLV
	 *
	 * */

	tlv_init(&tlv,cmd->tlv_data);
	while( tlv < (tlv_t*)&cmd->tlv_data[cmd->used_data])
	{
		if (tlv->type==type && type!=TLV_SOURCE_ADDRESS)
		{
			printf("Error TLV already exist ...\n" );
			return 0 ;
		}
		i+=(tlv->length+2);
		tlv_init(&tlv,&cmd->tlv_data[i]);
	}


	/* adding the tlv */
	if(tlv_write(tlv, type,  length, value))
	{
		cmd->used_data+=(length+2);
		return 1;
	}
	else
		return 0;

}


void print_buffer( uint8_t* buffer, uint8_t length )
{
	printf("==> Buffer :  ");
	for(uint8_t i=0 ; i<length ;i++) {
		printf("%02x ", buffer[i]);
	}
	printf("\n");

}


uint8_t mle_print_type_cmd( mle_cmd_t cmd )
{
	switch (cmd.type)
	{
	case LINK_REQUEST:
		printf("LINK REQUEST");
		break;
	case LINK_ACCEPT:
		printf("LINK ACCEPT");
		break;
	case LINK_ACCEPT_AND_REQUEST:
		printf("LINK ACCEPT AND REQUEST");
		break;
	case LINK_REJECT:
		printf("LINK REJECT");
		break;
	case ADVERTISEMENT:
		printf("ADVERTISEMENT");
		break;
	case UPDATE:
		printf("UPDATE");
		break;
	case UPDATE_REQUEST:  // Not used in Thread Network
		printf("UPDATE REQUEST");
		break;
	case DATA_REQUEST:  // Not used in Thread Network
		printf("DATA REQUEST");
		break;
	case DATA_RESPONSE:
		printf("DATA RESPONSE");
		break;
	case PARENT_REQUEST:
		printf("PARENT REQUEST");
		break;
	case PARENT_RESPONSE:
		printf("PARENT RESPONSE");
		break;
	case CHILD_ID_REQUEST:
		printf("CHILD ID REQUEST");
		break;
	case CHILD_ID_RESPONSE:
		printf("CHILD ID RESPONSE");
		break;
	case CHILD_UPDATE:
		printf("CHILD UPDATE");
		break;
	case CHILD_UPDATE_RESPONSE:
		printf("CHILD UPDATE RESPONSE");
		break;
	default :
		printf("error mle command type not recognized ");
		return 0 ;
		break;
	}
	return 1 ;
}


uint8_t mle_print_cmd( mle_cmd_t cmd )
{
	tlv_t * tlv;
	uint8_t i=0;

	printf("---------------------------------------------------------------\n            MLE command : " );
	if(!mle_print_type_cmd(cmd ))
		return 0 ;
	printf("\n---------------------------------------------------------------\n" );

	tlv_init(&tlv,cmd.tlv_data);
	while( tlv < (tlv_t*)&cmd.tlv_data[cmd.used_data])
	{
		tlv_print(tlv);
		i+=(tlv->length+2);
		tlv_init(&tlv,&cmd.tlv_data[i]);
	}

	printf("---------------------------------------------------------------\n" );
	return 1 ;
}


