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
    \file   mle_message.c

    \author Nidhal Mars <nidhal.mars@hs-offenburg.de>

    \brief  Functions to handle MLE message and add some TLV parameters

  	\version  0.1
*/


/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/


#include "mle_message.h"
#include "bsp.h"

/*==============================================================================
                                     MACROS
 =============================================================================*/

/**
* Generate 32-bit Random
*/
#define MLE_MSG_RADNOM()		(bsp_getrand(0) | (bsp_getrand(0)<< 16))


/*==============================================================================
								LOCAL VARIABLES
 =============================================================================*/

uint8_t buf[8];


/*==============================================================================
								LOCAL FUNCTION
 =============================================================================*/

/**
 * \brief  add 32-bit TLV value to command
 *
 * \param  cmd	    pointer to the command
 * \param  type  	type of TLV to add
 * \param  value	value of TLV (32-bit)
 *
 * \return
 *       -  1 sucess
 *       -  0 error
 */
static uint8_t  add_tlv32_bit_to_cmd(mle_cmd_t* cmd , tlv_type_t type , uint32_t value)
{

	buf[0]= (uint8_t) (value >> 24) & 0xFF ;
	buf[1]= (uint8_t) (value >> 16) & 0xFF ;
	buf[2]= (uint8_t) (value >>  8) & 0xFF ;
	buf[3]= (uint8_t) value  & 0xFF ;

	if (mle_add_tlv_to_cmd( cmd , type , 4 , (uint8_t*) buf ))
		return 1 ;
	else
		return 0 ;
}


/**
 * \brief  add 16-bit TLV value to command
 *
 * \param  cmd	    pointer to the command
 * \param  type  	type of TLV to add
 * \param  value	value of TLV (16-bit)
 *
 * \return
 *       -  1 sucess
 *       -  0 error
 */
static uint8_t  add_tlv16_bit_to_cmd(mle_cmd_t* cmd , tlv_type_t type , uint16_t value)
{

	buf[0]= (uint8_t) (value >> 8) & 0xFF ;
	buf[1]= (uint8_t) value  & 0xFF ;

	if (mle_add_tlv_to_cmd( cmd , type , 2 , (uint8_t*) buf ))
		return 1 ;
	else
		return 0 ;
}

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/


uint8_t  add_mac_src_address_to_cmd(mle_cmd_t* cmd)
{
	if (mle_add_tlv_to_cmd( cmd ,TLV_SOURCE_ADDRESS, 2,(uint8_t*) &mac_phy_config.mac_address[6] ))
		return 1 ;
	else
		return 0 ;
}

uint8_t  add_src_address_to_cmd(mle_cmd_t* cmd, uint16_t address)
{

	if ( add_tlv16_bit_to_cmd(cmd, TLV_SOURCE_ADDRESS, address) )
		return 1 ;
	else
		return 0 ;
}

uint8_t  add_address16_to_cmd(mle_cmd_t* cmd, uint16_t address)
{
	if (add_tlv16_bit_to_cmd( cmd ,TLV_ADDRESS16 ,address ))
		return 1 ;
	else
		return 0 ;
}

uint8_t  add_time_out_to_cmd(mle_cmd_t* cmd, uint32_t time)
{
	if(add_tlv32_bit_to_cmd(cmd, TLV_TIME_OUT, time))
		return 1 ;
	else
		return 0 ;
}

uint8_t  add_mode_RSDN_to_cmd(mle_cmd_t* cmd, uint8_t R, uint8_t S, uint8_t D, uint8_t N )
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

	if (mle_add_tlv_to_cmd( cmd, TLV_MODE, 1, &mode ))
		return 1 ;
	else
		return 0 ;
}



uint8_t  add_scan_mask_to_cmd(mle_cmd_t* cmd, uint8_t R, uint8_t E )
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

	if (mle_add_tlv_to_cmd( cmd, TLV_SCAN_MASK, 1, &mask ))
		return 1 ;
	else
		return 0 ;
}



uint8_t  add_MLEframe_counter_to_cmd(mle_cmd_t* cmd, uint32_t frame)
{
	if(add_tlv32_bit_to_cmd(cmd, TLV_MLE_FRAME_COUNTER, frame))
		return 1 ;
	else
		return 0 ;
}

uint32_t  add_rand_challenge_to_cmd(mle_cmd_t* cmd )
{
	uint32_t randn;
	do {randn=MLE_MSG_RADNOM();}
	while(!randn);

	if(add_tlv32_bit_to_cmd(cmd, TLV_CHALLENGE, randn ))
		return randn ;
	else
		return 0 ;
}

uint8_t  add_response_to_cmd(mle_cmd_t* cmd, tlv_t* resp )
{

	if(mle_add_tlv_to_cmd(cmd, TLV_RESPONSE, resp->length, resp->value ) )
		return 1 ;
	else
		return 0 ;
}

uint8_t  add_response32_to_cmd(mle_cmd_t* cmd, uint32_t value )
{

	if(add_tlv32_bit_to_cmd(cmd, TLV_RESPONSE, value))
		return 1 ;
	else
		return 0 ;
}


uint8_t  add_status_to_cmd(mle_cmd_t* cmd)
{
	uint8_t status=1;

	if (mle_add_tlv_to_cmd( cmd, TLV_STATUS, 1, &status ))
		return 1 ;
	else
		return 0 ;
}

uint8_t  add_version_to_cmd(mle_cmd_t* cmd)
{
	uint8_t version=1;

	if (mle_add_tlv_to_cmd( cmd, TLV_VERSION, 1, &version ))
		return 1 ;
	else
		return 0 ;
}


uint8_t  add_Link_margin_to_cmd(mle_cmd_t* cmd, uint8_t lm)
{
	if (mle_add_tlv_to_cmd( cmd, TLV_LINK_MARGIN, 1, &lm ))
		return 1 ;
	else
		return 0 ;
}

uint8_t  add_route64_to_cmd(mle_cmd_t* cmd, tlv_route64_t* route, uint8_t len)
{
	if (mle_add_tlv_to_cmd( cmd, TLV_ROUTE64, len,(uint8_t*) route ))
		return 1 ;
	else
		return 0 ;
}


uint8_t  add_leader_to_cmd(mle_cmd_t* cmd, tlv_leader_t* lead)
{
	if (mle_add_tlv_to_cmd( cmd, TLV_LEADER_DATA, 8,(uint8_t*) lead ))
		return 1 ;
	else
		return 0 ;
}


uint8_t  add_Cnnectivity_to_cmd(mle_cmd_t* cmd, uint8_t max_child, uint8_t child_count, uint8_t LQ3, uint8_t LQ2,
		uint8_t LQ1, uint8_t Leader_cost, uint8_t id_sed)
{
	buf[0]=max_child ;
	buf[1]=child_count ;
	buf[2]=LQ3 ;
	buf[3]=LQ2 ;
	buf[4]=LQ1 ;
	buf[5]=Leader_cost ;
	buf[6]=id_sed ;
	if (mle_add_tlv_to_cmd( cmd, TLV_CONNECTIVITY, 7, (uint8_t*) buf ))
		return 1 ;
	else
		return 0 ;
}


uint8_t  comp_resp_chall(uint32_t challenge, uint8_t * buf)
{
	return (challenge == ( (buf[3]) | (buf[2] << 8) | (buf[1] << 16) | (buf[0] << 24)) );
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





