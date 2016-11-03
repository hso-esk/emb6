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
    \file   tlv.c

    \author Nidhal Mars <nidhal.mars@hs-offenburg.de>

    \brief  Functions to handle TLV parameters used in MLE command

    \version  0.1
*/
/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#include "tlv.h"
#include "net_tlv.h"


/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

uint8_t tlv_init( tlv_t ** tlv, uint8_t * ptr )
{
	/* if the buffer is NULL then it is an error */
	if ( ptr == NULL )
		return 0 ;
	/* init the tlv */
	*tlv= (tlv_t*)  ptr;
	return 1 ;
}


uint8_t tlv_leader_init( tlv_leader_t **tlv, uint8_t * ptr )
{
	/* if the buffer is NULL then it is an error */
	if ( ptr == NULL )
		return 0 ;

	/* init the tlv */
	*tlv= (tlv_leader_t*)  ptr;
	return 1 ;
}

uint8_t tlv_route64_init( tlv_route64_t **tlv, uint8_t * ptr )
{
	/* if the buffer is NULL then it is an error */
	if( ptr == NULL)
		return 0 ;

	/* init the tlv */
	*tlv= (tlv_route64_t*)  ptr;
	return 1 ;
}

uint8_t tlv_connectivity_init( tlv_connectivity_t **tlv, uint8_t * ptr )
{
	/* if the buffer is NULL then it is an error */
	if( ptr == NULL)
		return 0 ;

	/* init the tlv */
	*tlv= (tlv_connectivity_t*)  ptr;
	return 1 ;

}


uint8_t tlv_write( tlv_t *tlv, tlv_type_t type, int8_t length, uint8_t * value )
{
	if(tlv == NULL )
		return 0;

	/* write values */
	tlv->type = (uint8_t)type;
	tlv->length = length;
	memcpy(tlv->value, value, tlv->length);
	return 1 ;
}


tlv_t* tlv_find( uint8_t * buf, uint8_t buf_length, const tlv_type_t type )
{
	tlv_t * tlv;
	uint8_t i=0;

	/* init the local TLV pointer with the buffer */
	tlv_init(&tlv,buf);
	while( tlv < (tlv_t*) &buf[buf_length] )
	{
		if ( tlv->type==type )
		{
			printf(" TLV exist ...\n");
			return tlv;
		}
		/* check the next TLV */
		i+=(tlv->length+2);
		/* */
		tlv_init(&tlv,&buf[i]);
	}
	/* TLV type is not existing */
	return NULL;
}



uint8_t tlv_print( tlv_t* tlv )
{
	PRINTF("  + type :");
	switch (tlv->type)
	{
	case TLV_SOURCE_ADDRESS:
		PRINTF(" SOURCE_ADDRESS    ");
		break;
	case TLV_MODE:
		PRINTF(" MODE              ");
		break;
	case TLV_TIME_OUT:
		PRINTF(" TIME_OUT 	     ");
		break;
	case TLV_CHALLENGE:
		PRINTF(" CHALLENGE         ");
		break;
	case TLV_RESPONSE:
		PRINTF(" RESPONSE          ");
		break;
	case TLV_LINK_LAYER_FRAME_COUNTER:
		printf(" LINK_LAYER_FRAME_COUNTER");
		break;
	case TLV_LINK_QUALITY:  // Not used in Thread Network
		PRINTF(" LINK_QUALITY      ");
		break;
	case TLV_NETWORK_PARAMETER:  // Not used in Thread Network
		PRINTF(" NETWORK_PARAMETER ");
		break;
	case TLV_MLE_FRAME_COUNTER:
		PRINTF(" MLE_FRAME_COUNTER ");
		break;
	case TLV_ROUTE64:
		PRINTF(" ROUTE64          ");
		break;
	case TLV_ADDRESS16:
		PRINTF(" ADDRESS16         ");
		break;
	case TLV_LEADER_DATA:
		PRINTF(" LEADER_DATA       ");
		break;
	case TLV_NETWORK_DATA:
		PRINTF(" NETWORK_DATA      ");
		break;
	case TLV_TLV_REQUEST:
		PRINTF(" TLV_REQUEST       ");
		break;
	case TLV_SCAN_MASK:
		PRINTF(" SCAN_MASK         ");
		break;
	case TLV_CONNECTIVITY:
		PRINTF(" CONNECTIVITY      ");
		break;
	case TLV_LINK_MARGIN:
		PRINTF(" LINK_MARGIN       ");
		break;
	case TLV_STATUS:
		PRINTF(" STATUS            ");
		break;
	case TLV_VERSION:
		PRINTF(" VERSION           ");
		break;
	case TLV_ADDRESS_REGISTRATION:
		PRINTF(" ADDRESS_REGISTRATION");
		break;
	default:
		PRINTF("Error tlv type not recognized ");
		return 0 ;
		break;
	}
	PRINTF(", length : %i , value :  ",tlv->length);

	for(uint8_t i=0 ; i < tlv->length; i++) {
		PRINTF("%02x ", tlv->value[i]);
	}
	PRINTF("\r\n");
	return 1;

}


/* Lukas.Z: tlv  functions used on the network layer */

uint8_t tlv_target_eid_init( net_tlv_target_eid_t **tlv, uint8_t *ptr )
{
	/* if the buffer is NULL then it is an error */
	if( ptr == NULL )
		return 0 ;

	/* init the tlv */
	*tlv= (net_tlv_target_eid_t*) ptr;
	return 1 ;
}

uint8_t tlv_rloc16_init( net_tlv_rloc16_t **tlv, uint8_t *ptr )
{
	/* if the buffer is NULL then it is an error */
	if( ptr == NULL )
		return 0 ;

	/* init the tlv */
	*tlv= (net_tlv_rloc16_t*) ptr;
	return 1 ;
}

uint8_t tlv_ml_eid_init( net_tlv_ml_eid_t **tlv, uint8_t *ptr )
{
	/* if the buffer is NULL then it is an error */
	if( ptr == NULL )
		return 0 ;

	/* init the tlv */
	*tlv= (net_tlv_ml_eid_t*) ptr;
	return 1 ;
}

