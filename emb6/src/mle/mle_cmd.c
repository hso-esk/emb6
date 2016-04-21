
/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/
#include "mle_cmd.h"
#include "string.h"




void  mle_init_cmd(mle_cmd_t* mle_cmd , const mle_cmd_type_t type)
{
	mle_cmd->type=type;
	for (uint8_t i=0 ; i<MAX_TLV; i++)
		mle_cmd->tlv[i].type=TLV_NONE;
	mle_cmd->used_data=0;
}

void  mle_init_empty_cmd(mle_cmd_t* mle_cmd )
{
	mle_init_cmd(mle_cmd ,CMD_NONE);
}

uint8_t mle_deserialize_cmd(const uint8_t* ptr , mle_cmd_t* mle_cmd , int16_t cmd_length )
{
	 uint8_t i=0 ; //  tlv counter
	 uint32_t index=0 ; // index : number of byte serialized

	if(mle_cmd->type != CMD_NONE || ptr == NULL) // the command should be empty
		return 0;


	/* deserialize mle cmd type  */
	mle_cmd->type= (mle_cmd_type_t) ptr[index++];

	mle_cmd->used_data=0;
	while(index < cmd_length)
	{
		// deserialize tlv type
		mle_cmd->tlv[i].type= (tlv_type_t) ptr[index++];

		// deserialize tlv length
		mle_cmd->tlv[i].length=ptr[index++];

		// deserialize data only if data is not NULL
		if(mle_cmd->tlv[i].length > 0)
		{
			mle_cmd->tlv[i].value= &mle_cmd->data[mle_cmd->used_data] ;
			memcpy(&mle_cmd->data[mle_cmd->used_data], &ptr[index], mle_cmd->tlv[i].length);

			mle_cmd->used_data+=mle_cmd->tlv[i].length;
			index += mle_cmd->tlv[i].length;
		}
		else
		{
			mle_cmd->tlv[i].value = NULL;
		}
		i++;
	}

	return 1 ;
}



uint8_t mle_serialize_cmd(const mle_cmd_t mle_cmd , uint8_t* ptr , uint16_t* result_length)
{
	 uint8_t i = 0 ; // i: tlv counter // result_length : number of byte serialized
	 uint32_t index=0;
	if(mle_cmd.type == CMD_NONE || ptr == NULL) // check if the command is empty
		return 0;

	ptr[index++]= (uint8_t) mle_cmd.type;
	while(mle_cmd.tlv[i].type!=TLV_NONE && i< MAX_TLV )
	{
		ptr[index++]=(uint8_t) mle_cmd.tlv[i].type;
		ptr[index++]=(uint8_t) mle_cmd.tlv[i].length;
		memcpy(&ptr[index], mle_cmd.tlv[i].value , mle_cmd.tlv[i].length);

		index+=(mle_cmd.tlv[i].length);
		i++;
	}
	*result_length=index;
	return 1 ;
}



uint8_t mle_add_tlv_to_cmd(mle_cmd_t * mle_cmd , const tlv_type_t type, const int8_t length,   uint8_t * value )
{
	 uint8_t i=0;
	// an MLE message MUST NOT contain two or more TLVs of the same type With the exceptions of the Source Address TLV and Parameter TLV

	while (mle_cmd->tlv[i].type!=TLV_NONE)
	{
		if (mle_cmd->tlv[i].type==type && type!=TLV_SOURCE_ADDRESS)
		{
			printf("\nError TLV already exit ...\n" );
			return 0 ;
		}
		if(i==MAX_TLV  )
		{
			printf("\nError tlv buffer is full ...\n" );
			return 0 ;
		}
		i++;
	}

	/*
	 * i should carry about data buffer overflow
	 * */

	if(tlv_write(&mle_cmd->tlv[i], type,  length, value , &mle_cmd->data[mle_cmd->used_data]))
	{
		mle_cmd->used_data+=length;
		return 1;
	}
	else
		return 0;

}


 void print_buffer(uint8_t* buffer)
{
	printf("==> Buffer :  ");
	for(uint8_t i=0 ; i<strlen((char*) buffer) ;i++) {
		printf("%02x ", buffer[i]);
	}
	printf("\n");

}

uint8_t mle_print_cmd(const mle_cmd_t mle_cmd )
{
	 uint8_t i=0;

	printf("-----------------------------------------------------\nMLE command : " );
	switch (mle_cmd.type)
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
	case CMD_NONE:
		printf("error mle command type not recognized ");
		return 0 ;
		break;
	}
	printf("\n" );

	while( mle_cmd.tlv[i].type!= TLV_NONE &&  i<MAX_TLV)
	{
		tlv_print(mle_cmd.tlv[i++]);
	}
	printf("-----------------------------------------------------\n" );
	return 1 ;
}


