
/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#include "tlv.h"




/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/


int8_t tlv_init(tlv_t ** tlv, uint8_t * ptr )
{
	if( ptr == NULL)
		return 0 ;


	*tlv= (tlv_t* )  ptr;
	return 1 ;
}


int8_t tlv_write(tlv_t *tlv, tlv_type_t type, int8_t length, uint8_t * value )
{
	if(tlv == NULL )
		return 0;

	tlv->type=(uint8_t)type;
	tlv->length=length;
	memcpy(tlv->value, value, tlv->length);
	return 1 ;
}



int8_t tlv_print(tlv_t* tlv)
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
	case TLV_ROUTER64:
		PRINTF(" ROUTER64          ");
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

	for(uint8_t i=0 ; i<tlv->length; i++) {
		PRINTF("%02x ", tlv->value[i]);
	}
	PRINTF("\r\n");
	return 1;

}




