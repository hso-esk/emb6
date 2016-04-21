
/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#include "tlv.h"




/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/


int8_t tlv_write(tlv_t *tlv, tlv_type_t type, int8_t length, uint8_t * value , uint8_t * ptr_to_buffer)
{
	if(tlv->type != TLV_NONE || ptr_to_buffer == NULL) // check if the command is empty
		return 0;

	tlv->type=type;
	tlv->length=length;
	tlv->value=ptr_to_buffer;
	memcpy(ptr_to_buffer, value, length);
	return 1 ;
}


int8_t tlv_print(tlv_t tlv)
{
	printf("  + ");
	switch (tlv.type)
	{
	case TLV_SOURCE_ADDRESS:
		printf("type : SOURCE_ADDRESS    ");
		break;
	case TLV_MODE:
		printf("type : MODE              ");
		break;
	case TLV_TIME_OUT:
		printf("type : TIME_OUT 	     ");
		break;
	case TLV_CHALLENGE:
		printf("type : CHALLENGE         ");
		break;
	case TLV_RESPONSE:
		printf("type : RESPONSE          ");
		break;
	case TLV_LINK_LAYER_FRAME_COUNTER:
		printf("type : LINK_LAYER_FRAME_COUNTER");
		break;
	case TLV_LINK_QUALITY:  // Not used in Thread Network
		printf("type : LINK_QUALITY      ");
		break;
	case TLV_NETWORK_PARAMETER:  // Not used in Thread Network
		printf("type : NETWORK_PARAMETER ");
		break;
	case TLV_MLE_FRAME_COUNTER:
		printf("type : MLE_FRAME_COUNTER ");
		break;
	case TLV_ROUTER64:
		printf("type : ROUTER64          ");
		break;
	case TLV_ADDRESS16:
		printf("type : ADDRESS16         ");
		break;
	case TLV_LEADER_DATA:
		printf("type : LEADER_DATA       ");
		break;
	case TLV_NETWORK_DATA:
		printf("type : NETWORK_DATA      ");
		break;
	case TLV_TLV_REQUEST:
		printf("type : TLV_REQUEST       ");
		break;
	case TLV_SCAN_MASK:
		printf("type : SCAN_MASK         ");
		break;
	case TLV_CONNECTIVITY:
		printf("type : CONNECTIVITY      ");
		break;
	case TLV_LINK_MARGIN:
		printf("type : LINK_MARGIN       ");
		break;
	case TLV_STATUS:
		printf("type : STATUS            ");
		break;
	case TLV_VERSION:
		printf("type : VERSION           ");
		break;
	case TLV_ADDRESS_REGISTRATION:
		printf("type : ADDRESS_REGISTRATION");
		break;
	case TLV_NONE:
		printf("Error tlv type not recognized ");
		return 0 ;
		break;
	}
	printf(", length : %i , value :  ",tlv.length);

	for(uint8_t i=0 ; i<tlv.length;i++) {
		printf("%02x ", tlv.value[i]);
	}
	printf("\r\n");
	return 1;

}




