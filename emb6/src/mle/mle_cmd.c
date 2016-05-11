
/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/
#include "mle_cmd.h"
#include "string.h"




void  mle_init_cmd(mle_cmd_t* mle_cmd , const mle_cmd_type_t type)
{
	mle_cmd->type= (uint8_t) type;
	mle_cmd->used_data=0;
}

void  mle_create_cmd_from_buff(mle_cmd_t** cmd , uint8_t* data , uint16_t datalen)
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
		//	printf(" TLV exist ...\n" );
			return  tlv;
		}
		i+=(tlv->length+2);
		tlv_init(&tlv,&cmd->tlv_data[i]);
	}
	return NULL;
	//return	tlv_find(cmd->tlv_data,cmd->used_data,type);
}

uint8_t mle_add_tlv_to_cmd(mle_cmd_t * cmd , const tlv_type_t type, const int8_t length,   uint8_t * value )
{
	tlv_t * tlv;
	uint8_t i=0;

	/* carry about data buffer overflow*/
	if ((cmd->used_data+ length +sizeof(tlv->type) + sizeof(tlv->length)) >= MAX_TLV_DATA_SIZE)
	{
		printf("buffer overflow \n");
		return 0;
	};

	/* an MLE message MUST NOT contain two or more TLVs of the same type
	 * With the exceptions of the Source Address TLV and Parameter TLV */


	/***************** use the existing function *********************/
		tlv_init(&tlv,cmd->tlv_data);
	while( tlv < (tlv_t*)&cmd->tlv_data[cmd->used_data])
	{
		if (tlv->type==type && type!=TLV_SOURCE_ADDRESS)
		{
			printf("\nError TLV already exist ...\n" );
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


void print_buffer(uint8_t* buffer ,uint8_t length)
{
	printf("==> Buffer :  ");
	for(uint8_t i=0 ; i<length ;i++) {
		printf("%02x ", buffer[i]);
	}
	printf("\n");

}

uint8_t mle_print_cmd( mle_cmd_t cmd )
{
	tlv_t * tlv;
	uint8_t i=0;

	printf("---------------------------------------------------------------\n            MLE command : " );
	switch (cmd.type)
	{
	case LINK_REQUEST:
		printf("LINK_REQUEST");
		break;
	case LINK_ACCEPT:
		printf("LINK_ACCEPT");
		break;
	case LINK_ACCEPT_AND_REQUEST:
		printf("LINK_ACCEPT_AND_REQUEST");
		break;
	case LINK_REJECT:
		printf("LINK_REJECT");
		break;
	case ADVERTISEMENT:
		printf("ADVERTISEMENT");
		break;
	case UPDATE:
		printf("UPDATE");
		break;
	case UPDATE_REQUEST:  // Not used in Thread Network
		printf("UPDATE_REQUEST");
		break;
	case DATA_REQUEST:  // Not used in Thread Network
		printf("DATA_REQUEST");
		break;
	case DATA_RESPONSE:
		printf("DATA_RESPONSE");
		break;
	case PARENT_REQUEST:
		printf("PARENT_REQUEST");
		break;
	case PARENT_RESPONSE:
		printf("PARENT_RESPONSE");
		break;
	case CHILD_ID_REQUEST:
		printf("CHILD_ID_REQUEST");
		break;
	case CHILD_ID_RESPONSE:
		printf("CHILD_ID_RESPONSE");
		break;
	case CHILD_UPDATE:
		printf("CHILD_UPDATE");
		break;
	case CHILD_UPDATE_RESPONSE:
		printf("CHILD_UPDATE_RESPONSE");
		break;
	default :
		printf("error mle command type not recognized ");
		return 0 ;
		break;
	}
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


