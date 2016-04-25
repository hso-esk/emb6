/*
 * net_tlv.c
 *
 *  Created on: 22 Apr 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 *  Network Layer TLVs.
 */

/*
 ********************************************************************************
 *                                   INCLUDES
 ********************************************************************************
 */

#include "emb6.h"
#include "net_tlv.h"

/*
 ********************************************************************************
 *                          LOCAL FUNCTION DECLARATIONS
 ********************************************************************************
 */

/*
 ********************************************************************************
 *                               LOCAL VARIABLES
 ********************************************************************************
 */

/*
 ********************************************************************************
 *                               GLOBAL VARIABLES
 ********************************************************************************
 */

/*
 ********************************************************************************
 *                           LOCAL FUNCTION DEFINITIONS
 ********************************************************************************
 */

/**
 * Create TLV struture.
 * @param net_tlv
 * @param type				The Network Layer TLV type.
 * @param length			The length of the value field.
 * @param value				The value.
 * @param ptr_to_buffer		The buffer were the value is stored.
 * @return					If successful, a pointer to the network layer tlv.
 * 							NULL, else.
 */
net_tlv_t
*net_tlv_write(net_tlv_t *net_tlv, net_tlv_type_t type, uint8_t length,
		uint8_t * value, uint8_t *ptr_to_buffer)
{
	if ( ptr_to_buffer == NULL )
		return NULL;

	net_tlv->type = type;
	net_tlv->length = length;
	net_tlv->value = ptr_to_buffer;

	memcpy(ptr_to_buffer, value, length);

	return net_tlv;
}

/* --------------------------------------------------------------------------- */

/*
 ********************************************************************************
 *                               END OF FILE
 ********************************************************************************
 */
