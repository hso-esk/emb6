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
    \file   mle_message.h

    \author Nidhal Mars <nidhal.mars@hs-offenburg.de>

    \brief  Functions to handle MLE message and add some TLV parameters

  	\version  0.1
*/


#ifndef  __MLE_MSG_H_
#define  __MLE_MSG_H_


/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#include "emb6.h"
#include "mle_cmd.h"
#include "framer_802154.h"

/*==============================================================================
									TYPEDEFS
 =============================================================================*/

/**
* MLE Message structure
*/
typedef struct {
	/* security bit: Thread uses an initial byte of ‘0’ indicates that the message is secured (encrypted and authenticated) */
	uint8_t 					   secured ;
	/* Aux Header security */
	frame802154_aux_hdr_t          aux_hdr;
	/* MLE command */
	mle_cmd_t           		   cmd;
	/* Message integrity code */
	uint8_t          			   mic ;
}mle_msg_t ;



/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

/**
 * \brief  add MAC source address to command
 *
 * \param  cmd	    pointer to the command
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_mac_src_address_to_cmd(mle_cmd_t* cmd);


/**
 * \brief  add 16-bit source address to command
 *
 * \param  cmd	    	pointer to the command
 * \param  address	    16-bit short address
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_src_address_to_cmd(mle_cmd_t* cmd, uint16_t address);


/**
 * \brief  add 16-bit assigned short address to command
 *
 * \param  cmd	    	pointer to the command
 * \param  address	    16-bit short address
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_address16_to_cmd(mle_cmd_t* cmd , uint16_t address);


/**
 * \brief  add time out to command
 *
 * \param  cmd	    	pointer to the command
 * \param  time		    32-bit timeout
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_time_out_to_cmd(mle_cmd_t* cmd , uint32_t time);


/**
 * \brief  add MODE TLV to command
 *
 * \param  cmd	 pointer to the command
 * \param  R     Receiver on when idle: set to '1' if the sender has its receiver on when not transmitting; otherwise, set to ‘0’
 * \param  S	 Secure data requests: Set to ‘1’ if the sender will use 802.15.4 [IEEE802154] to secure all data requests; otherwise, set to ‘0’
 * \param  D	 Device type: Set to ‘1’ if the sender is an FFD (Full Function Device); set to ‘0’ if an RFD (Reduced Function Device)
 * \param  N     Network data: Set to ‘1’ if the sender requires the full Network Data; set to ‘0’ if the sender only needs the stable Network Data
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_mode_RSDN_to_cmd(mle_cmd_t* cmd , uint8_t R , uint8_t S , uint8_t D , uint8_t N );


/**
 * \brief  add SCAN MASK TLV to command
 *
 * \param  cmd	 pointer to the command
 * \param  R     Router: Active Routers MUST respond
 * \param  E     End device: REEDs that are currently acting as Children MUST respond
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_scan_mask_to_cmd(mle_cmd_t* cmd , uint8_t R , uint8_t E );


/**
 * \brief  add MLE FRAME COUNTER TLV to command
 *
 * \param  cmd	 pointer to the command
 * \param  R     Frame counter
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_MLEframe_counter_to_cmd(mle_cmd_t* cmd, uint32_t frame) ;


/**
 * \brief  add STATUS TLV to command
 *
 * \param  cmd	 pointer to the command
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_status_to_cmd(mle_cmd_t* cmd);


/**
 * \brief  add VERSION TLV to command
 *
 * \param  cmd	 pointer to the command
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_version_to_cmd(mle_cmd_t* cmd);


/**
 * \brief  generate and add CHALLENGE TLV to command
 *
 * \param  cmd	 pointer to the command
 *
 * \return 32-bit generated challenge (0 in case of failure)
 */
uint32_t add_rand_challenge_to_cmd(mle_cmd_t* cmd );


/**
 * \brief  add RESPONSE TLV to command
 *
 * \param  cmd	 pointer to the command
 * \param  resp	 response
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_response_to_cmd(mle_cmd_t* cmd , tlv_t* resp );



/**
 * \brief  add 32-bit RESPONSE TLV to command
 *
 * \param  cmd	 pointer to the command
 * \param  resp	 32-bit response
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_response32_to_cmd(mle_cmd_t* cmd ,  uint32_t value );


/**
 * \brief  add LINK MARGIN TLV to command
 *
 * \param  cmd	 pointer to the command
 * \param  lm	 8-bit link margin
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_Link_margin_to_cmd(mle_cmd_t* cmd, uint8_t lm);


/**
 * \brief  add ROUTE64 TLV to command
 *
 * \param  cmd	    pointer to the command
 * \param  route    route64
 * \param  len      length of route64
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_route64_to_cmd(mle_cmd_t* cmd, tlv_route64_t* route , uint8_t len);


/**
 * \brief  add LEADER TLV to command
 *
 * \param  cmd	 pointer to the command
 * \param  lead	 leader tlv
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_leader_to_cmd(mle_cmd_t* cmd, tlv_leader_t* lead);


/**
 * \brief  add CONNECTIVITY TLV to command
 *
 * \param  cmd	 		 pointer to the command
 * \param  max_child	 The maximum number of Children the sender can support
 * \param  child_count	 The number of Children the sender currently supports
 * \param  LQ3	 		 number of linked neighbors with link quality 3
 * \param  LQ2	 		 number of linked neighbors with link quality 2
 * \param  LQ1	 		 number of linked neighbors with link quality 1
 * \param  Leader_cost	 leader cost
 * \param  id_sed	 	 id set
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  add_Cnnectivity_to_cmd(mle_cmd_t* cmd, uint8_t max_child, uint8_t child_count, uint8_t LQ3, uint8_t LQ2,
							uint8_t LQ1, uint8_t Leader_cost,uint8_t id_sed);


/**
 * \brief  compare challenge and response TLVs
 *
 * \param  challenge	 32-bit challenge
 * \param  buf			 32-bit response
 * \return
 *       -  1 equal
 *       -  0 different
 */
uint8_t  comp_resp_chall(uint32_t challenge , uint8_t * buf);



/**
 * \brief  set the security flag in MLE message
 *
 * \param  msg	  MLE message
 * \param  flag   1 or 0
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t  set_security_flag(mle_msg_t* msg , uint8_t flag);



#endif /* __MLE_MSG_H_ */
