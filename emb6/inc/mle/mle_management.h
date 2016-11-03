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
    \file   mle_management.h

    \author Nidhal Mars <nidhal.mars@hs-offenburg.de>

    \brief  Mesh Link Establishment protocol

  	\version  0.1
*/

#ifndef  __MLE_MANAGEMENT_H_
#define  __MLE_MANAGEMENT_H_

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/
#include "mle_table.h"
#include "mle_message.h"
#include "tcpip.h"
#include "udp-socket.h"

/*==============================================================================
                                     MACROS
 =============================================================================*/

#define		MLE_UDP_LPORT		         19788   /* MLE UDP Port  */
#define		MLE_UDP_RPORT		         19788   /* MLE UDP Port  */


/**
 * TLV parameters
 * */

#define		TIME_OUT		 		     10     /* time out   */
#define		IS_RX_ON_WHEN_IDLE		 	  1     /* Set to ‘1’ if the sender has its receiver on when not transmitting   */
#define		IS_FFD		 	     		  1
/*==============================================================================
									TYPEDEFS
 =============================================================================*/

/**
* param structure to store required
* information for a received command
*/
typedef struct {
	/* pointer to received command */
	mle_cmd_t * rec_cmd;
	/* incoming RSSI */
	uint8_t    rec_rssi;
	/* source address */
	uip_ipaddr_t  source_addr;
}mle_param_t ;


/**
* MLE Node operation mode
*/
typedef enum
{
	NOT_LINKED,
	CHILD,
	PARENT,
} mle_mode_t;


/**
* MLE join process state
*/
typedef enum {
	JP_SEND_MCAST_PR_TO_ROUTER,
	JP_SEND_MCAST_PR_TO_ROUTER_REED,
	JP_PARENT_SELECT,
	JP_SEND_CHILD_REQ,
	JP_SAVE_PARENT,
	JP_FAIL,
}jp_state_t; // join process state


/**
* MLE synchronisation process state
*/
typedef enum {
	SYN_SEND_LINK_REQUEST,
	SYN_PROCESS_LINK,
}syn_state_t; // synchronisation state


/**
* MLE keep-alive state
*/
typedef enum {
	KA_SEND_KEEP_ALIVE,
	KA_WAIT_RESPONSE,
}ka_state_t; // keep alive state


/**
* MLE Node structure
*/
typedef struct {
	uint16_t			  address;                        		/**< My node 16 bit address */
	struct udp_socket     udp_socket;							/**< UDP socket used to send and receive MLE message */
 	syn_state_t 		  syn_state;  							/**< synchronisation process current state */
	mle_mode_t     		  OpMode;                               /**< device operating mode */
	uint32_t 			  timeOut;								/**< my time out  */
	uint32_t              thrMLEFrameCounter;                   /**< Outgoing frame counter */
	uint8_t 			  NB_router_couter;						/**< nb router counter   */
	uint8_t 			  childs_counter;						/**< linked child counter  */
 	uint32_t 			  challenge ;  					        /**< current join/synchronisation process challenge  */
	uint8_t               rx_on_when_idle; 					    /**< Set to ‘1’ if the sender's receiver on when not transmitting */
}st_mle_node_t;

/**
* MLE PARENT candidate structure: used while joining process to store best candidate
*/
typedef struct {
	uint8_t  	LQ;			 /**< Link quality */
	uint8_t  	is_Router;   /**< the parent candidate is active router */
	uint8_t  	LQ3; 		 /**< LQ3 from connectivity TLV */
}mle_parent_t;

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/


/**
 * \brief  initialise MLE protocol
 *
 * \return
 *       -  1 sucess
 *       -  0 error
 */
uint8_t mle_init(void);


/**
 * \brief  send MLE advertisement message
 *
 * \param  route	  		pointer to the route64 tlv
 * \param  len	  			length of router 64
 * \param  lead	  			pointer to the leader tlv
 *
 * \return
 *       -  1 success
 *       -  0 error
 */
uint8_t send_mle_advertisement(tlv_route64_t* route, uint8_t len, tlv_leader_t* lead);


/**
 * \brief  Reply for child id request
 *
 * \param  ptr	  	Normaly it NULL, in case we are operating
 * as child the network layer have to request a new routerID and give it as parameter
 *
 */
void  reply_for_mle_childID_request(void *ptr);


/**
 * \brief  Set the node operate as parent
 */
void  mle_set_parent_mode(void);


/**
 * \brief  Set the node to operate as child
 *
 * \param  rloc16	  	16-bit address assigned by the parent
 *
 */
void  mle_set_child_mode(uint16_t rloc16);


#endif /* __MLE_MANAGEMENT_H_ */
