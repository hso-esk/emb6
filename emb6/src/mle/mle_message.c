

#include "mle_message.h"
#include "bsp.h"

#define MLE_MSG_RADNOM()		(rand=bsp_getrand(0) | (bsp_getrand(0)<< 16))




uint16_t  get_16_MAC(void)
{
	//printf("mac addres is : %04x" ,a);
	return (mac_phy_config.mac_address[7] | (mac_phy_config.mac_address[6] << 8));

}


uint8_t  add_src_address_to_cmd(mle_cmd_t* cmd)
{
	if (mle_add_tlv_to_cmd( cmd ,TLV_SOURCE_ADDRESS , 2 ,(uint8_t*) &mac_phy_config.mac_address[6] ))
		return 1 ;
	else
		return 0 ;
}

uint8_t  add_time_out_to_cmd(mle_cmd_t* cmd , uint32_t time)
{
	if(add_tlv32_bit_to_cmd(cmd ,TLV_TIME_OUT , time))
		return 1 ;
	else
		return 0 ;
}

uint8_t  add_mode_RSDN_to_cmd(mle_cmd_t* cmd , uint8_t R , uint8_t S , uint8_t D, uint8_t N )
{
	uint8_t mode=0 ;

	/*
	 * 	+-------------------------------+
	 * 	| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
	 * 	+-------------------------------+
	 * 	|    reserved   | R | S | D | N |
	 * 	+-------------------------------+
	 */

	if(R)
		BIT_SET(mode,3);
	if(S)
		BIT_SET(mode,2);
	if(D)
		BIT_SET(mode,1);
	if(N)
		BIT_SET(mode,0);

	if (mle_add_tlv_to_cmd( cmd ,TLV_MODE , 1 , &mode ))
		return 1 ;
	else
		return 0 ;
}



uint8_t  add_scan_mask_to_cmd(mle_cmd_t* cmd , uint8_t R , uint8_t E )
{
	uint8_t mask=0 ;

	/*
	 * 	+-------------------------------+
	 * 	| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
	 * 	+-------------------------------+
	 * 	| R | E |        reserved       |
	 * 	+-------------------------------+
	 */

	if(R)
		BIT_SET(mask,7);
	if(E)
		BIT_SET(mask,6);

	if (mle_add_tlv_to_cmd( cmd ,TLV_SCAN_MASK , 1 , &mask ))
		return 1 ;
	else
		return 0 ;
}



uint8_t  add_MLEframe_counter_to_cmd(mle_cmd_t* cmd , uint32_t frame)
{
	if(add_tlv32_bit_to_cmd(cmd ,TLV_MLE_FRAME_COUNTER , frame))
		return 1 ;
	else
		return 0 ;
}

uint32_t  add_rand_challenge_to_cmd(mle_cmd_t* cmd )
{
	uint32_t rand;
	do {rand=MLE_MSG_RADNOM();}
	while(!rand);

	if(add_tlv32_bit_to_cmd(cmd ,TLV_CHALLENGE ,rand ))
		return rand ;
	else
		return 0 ;
}

uint32_t  add_response_to_cmd(mle_cmd_t* cmd , uint32_t resp )
{

	if(add_tlv32_bit_to_cmd(cmd ,TLV_RESPONSE ,resp ))
		return 1 ;
	else
		return 0 ;
}


uint8_t  add_status_to_cmd(mle_cmd_t* cmd)
{
	uint8_t status=1;

	if (mle_add_tlv_to_cmd( cmd ,TLV_STATUS , 1 , &status ))
		return 1 ;
	else
		return 0 ;
}

uint8_t  add_version_to_cmd(mle_cmd_t* cmd)
{
	uint8_t version=1;

	if (mle_add_tlv_to_cmd( cmd ,TLV_VERSION , 1 , &version ))
		return 1 ;
	else
		return 0 ;
}


uint8_t  add_tlv32_bit_to_cmd(mle_cmd_t* cmd , tlv_type_t type , uint32_t value)
{
	uint8_t buf[4];

	buf[0]= (uint8_t) (value >> 24) & 0xFF ;
	buf[1]= (uint8_t) (value >> 16) & 0xFF ;
	buf[2]= (uint8_t) (value >>  8) & 0xFF ;
	buf[3]= (uint8_t) value  & 0xFF ;

	if (mle_add_tlv_to_cmd( cmd , type , 4 , (uint8_t*) buf ))
		return 1 ;
	else
		return 0 ;
}


/* In particular, frame counters MUST NOT be reused for any given key;
 * if the outgoing MLE frame counter reaches its maximum value (0xFFFFFFFF),
 * secured MLE messages MUST NOT be sent until a new key is available,
 * at which point the outgoing MLE frame counter MAY be set back to zero.
 */
uint8_t  set_security_flag(mle_msg_t* msg, uint8_t flag)
{
	if(msg!=NULL)
		msg->secured=flag;
	else
		return 0;
	return 1 ;
}





