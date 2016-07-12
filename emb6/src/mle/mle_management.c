/*
 * mle_management.c
 *
 *      Author: Nidhal Mars
 *      manage mle protcol
 */

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#include "emb6.h"
#include "bsp.h"
#include "mle_management.h"

#include "thrd-adv.h"
#include "thrd-partition.h"

#include "etimer.h"
#include "evproc.h"
#include "tcpip.h"
#include "rpl.h"
#include "udp-socket.h"
#include "packetbuf.h"

#define		DEBUG		DEBUG_PRINT
#include "uip-debug.h"	// For debugging terminal output.

#define     LOGGER_ENABLE                 LOGGER_RADIO
#include    "logger.h"

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

static mle_node_t MyNode;
static  uip_ipaddr_t                s_destAddr;
struct  ctimer                      c_mle_Timer; // used for the join process(child) and synchro process(parent)
static 	mle_cmd_t 					cmd; // command buffer
static mle_param_t					param;
static mle_neighbor_t*				nb; // to handle parent + ...
static mle_neighbor_t* 				child; // to handle the child devices in both mode (parent/child)

/*==============================================================================
								LOCAL FUNCTION
 =============================================================================*/
static uint8_t  		mle_send_msg(mle_cmd_t* cmd,  uip_ipaddr_t *dest_addr);
uint8_t 			isActiveRouter(uint16_t srcAdd)
{
	for(uint8_t i=0; i<9;i++)
	{
		if(BIT_CHECK(srcAdd,i))
			return 0;
	}
	return 1 ;
}


#define routerMask(scanMask)			(BIT_CHECK(scanMask,7))
#define ReedMask(scanMask)				(BIT_CHECK(scanMask,6))
#define get_rssi()		             30 //sicslowpan_get_last_rssi()
#define get_routerID()		         0xff




/*
 * @brief Send multicast MLE parent request
 * @return 1    OK.
 * @return 0   FAIL.
 *
 */

uint8_t send_mle_advertisement(tlv_route64_t* route, uint8_t len, tlv_leader_t* lead)
{
	if(MyNode.OpMode == PARENT)
	{
		mle_init_cmd(&cmd,ADVERTISEMENT);
		add_src_address_to_cmd(&cmd);
		add_route64_to_cmd(&cmd,route,len);
		add_leader_to_cmd(&cmd,lead);
		uip_create_linklocal_allnodes_mcast(&s_destAddr);
		mle_send_msg(&cmd, &s_destAddr);
	}
	return 1 ;

}



/************************ link calculation functions ****************************/

static uint8_t mapRSSI(uint8_t rssi )
{
	if ( rssi > 20)
		return 3;
	else if ( (rssi > 10 )
			&& (rssi <= 20  ))
		return 2;
	else if ( (rssi > 2 )
			&& (rssi <= 10  ))
		return 1;
	else
		return 0;

}

static uint8_t calculate_two_way_LQ(uint8_t my_rssi , uint8_t rec_rssi)
{
	if (my_rssi<rec_rssi)
		return mapRSSI(my_rssi);
	else
		return mapRSSI(rec_rssi);
}

/************************ join process functions ****************************/

static uint8_t send_mle_parent_request(uint8_t R, uint8_t E)
{
	mle_init_cmd(&cmd,PARENT_REQUEST);
	add_mode_RSDN_to_cmd(&cmd, MyNode.rx_on_when_idle, 0, IS_FFD, 0);
	MyNode.challenge=add_rand_challenge_to_cmd(&cmd);
	add_scan_mask_to_cmd(&cmd,R,E);
	add_version_to_cmd(&cmd);
	uip_create_linklocal_allrouters_mcast(&s_destAddr);
	mle_send_msg(&cmd, &s_destAddr);
	return 1 ;
}

/**
 * @brief  send child id request
 *
 * @param  dest_addr	  pointer to the dest address
 */
static uint8_t send_mle_childID_request(uip_ipaddr_t *dest_addr)
{
	mle_init_cmd(&cmd,CHILD_ID_REQUEST);
	add_response32_to_cmd(&cmd, nb->challenge);
	// link-layer frame counter tlv
	// MLE Frame Counter tlv
	add_mode_RSDN_to_cmd(&cmd, MyNode.rx_on_when_idle, 0, IS_FFD, 0);
	add_time_out_to_cmd(&cmd,TIME_OUT);
	add_version_to_cmd(&cmd);
	mle_send_msg(&cmd, dest_addr);
	return 1 ;
}

/**
 * @brief  check after time out if the child still pending then remove it
 *
 * @param  ptr	  pointer to the child id
 */
static void check_child_state(void *ptr)
{
	uint8_t* child_id;
	child_id= (uint8_t*) ptr;

	if (child->state==PENDING)
	{
		child=mle_find_child(*child_id);
		mle_rm_child(child);
		PRINTFR("Time out : child removed \n"ANSI_COLOR_RESET);
	}
	else
	{
		PRINTFG("Child Linked \n"ANSI_COLOR_RESET);
	}

}

static void reply_for_mle_parent_request(void *ptr)
{

	uint8_t* child_id;
	child_id= (uint8_t*) ptr;

	if(MyNode.OpMode!= NOT_LINKED)
	{

		mle_init_cmd(&cmd,PARENT_RESPONSE);
		add_src_address_to_cmd(&cmd);
		// leader data tlv :: from Network layer
		// link-layer frame counter tlv
		// MLE Frame Counter tlv
		add_response_to_cmd(&cmd, mle_find_tlv_in_cmd(param.rec_cmd,TLV_CHALLENGE));

		/* find the child */
		child= mle_find_child(*child_id);

		/* store the challenge generated  */
		child->challenge= add_rand_challenge_to_cmd(&cmd);

		add_Link_margin_to_cmd(&cmd,param.rec_rssi);
		add_Cnnectivity_to_cmd(&cmd,MAX_CHILD,MyNode.childs_counter,count_neighbor_LQ(3),count_neighbor_LQ(2),count_neighbor_LQ(1),0,0);
		add_version_to_cmd(&cmd);


		/* to change by the child address */
		mle_send_msg( &cmd,&child->tmp);

	}

}

static uint16_t assign_address16(uint8_t id)
{
	return   (( get_routerID() << 10) |  id | 0x0000  );
}

static void reply_for_mle_childID_request(void *ptr)
{

	if(MyNode.OpMode == CHILD)
	{
		// send request to become a router and then reply
		PRINTFY("Sending request to become a Router \n"ANSI_COLOR_RESET);

		//send_req(NULL);
		/*
		void send_req(uint8_t *router_id)
		{
		thrd_request_router_id(uip_ipaddr_t *leader_addr, uint8_t *ml_eid, uint8_t *router_id)
		}
		 */

		mle_set_parent_mode();

		mle_init_cmd(&cmd,CHILD_ID_RESPONSE);
		add_src_address_to_cmd(&cmd);
		// leader data tlv :: from Network layer
		add_address16_to_cmd(&cmd , assign_address16(5));
		// router64
		mle_send_msg( &cmd,&param.source_addr);
		child= mle_find_child_byAdd(&param.source_addr);
		child->state=LINKED;


	}
	else if (MyNode.OpMode == PARENT)
	{
		mle_init_cmd(&cmd,CHILD_ID_RESPONSE);
		add_src_address_to_cmd(&cmd);
		// leader data tlv :: from Network layer
		add_address16_to_cmd(&cmd , assign_address16(5));
		// router64
		mle_send_msg( &cmd,&param.source_addr);
		child= mle_find_child_byAdd(&param.source_addr);
		child->state=LINKED;
	}
}

void mle_join_process(void *ptr)
{
	static 	jp_state_t 		jp_state;  // join process current state
	static mle_parent_t 	parent_condidate , current_parent; // parent
	tlv_connectivity_t* connectivity;
	tlv_t * tlv;

	uint8_t finish=0;
	static  uint8_t reponse_recived=0, req_sent_to_reed=0;; // flag to know if parent response received // flag to check whether the request is sent to reed or no


	if(MyNode.OpMode==NOT_LINKED)
	{
		while(!finish)
		{
			switch(jp_state)
			{
			case JP_SEND_MCAST_PR_TO_ROUTER:
				PRINTFG("[+] "ANSI_COLOR_RESET);
				PRINTF("JP Send mcast parent request to active router \n"ANSI_COLOR_RESET);

				/* Init the parent*/
				current_parent.is_Router=0;
				current_parent.LQ3=0;
				current_parent.LQ=0;

				/* Init the parent on the neighbor table*/
				nb=mle_add_nb_router(0, 0, 0, 0, 0);
				if(nb==NULL)
					nb=mle_find_nb_router(0);
				nb->state=PENDING;
				nb->LQ=0;
				nb->address16=0;
				MyNode.NB_router_couter=1;

				/* send parent request to the active router only and trigger the timer */
				jp_state=JP_PARENT_SELECT;
				send_mle_parent_request(1,0);
				PRINTFG("[+] "ANSI_COLOR_RESET);
				PRINTF("JP Waiting for incoming response from active Router\n"ANSI_COLOR_RESET);
				ctimer_set(&c_mle_Timer, 2 * bsp_get(E_BSP_GET_TRES) , mle_join_process, NULL);
				req_sent_to_reed=0;
				finish=1;
				break;
			case JP_PARENT_SELECT:
				if(ctimer_expired(&c_mle_Timer)) // that means that this function was triggered by the ctimer
				{
					if(!reponse_recived)
					{
						if(!req_sent_to_reed)
						{
							jp_state=JP_SEND_MCAST_PR_TO_ROUTER_REED;
							reponse_recived=0;
						}
						else
							jp_state=JP_FAIL;
					}
					else
					{
						if(nb->LQ == 3 || req_sent_to_reed )
						{
							PRINTFG("[+] "ANSI_COLOR_RESET);
							PRINTF("JP Parent selection \n"ANSI_COLOR_RESET);
							PRINTFG("link quality with parent : %i \n"ANSI_COLOR_RESET, nb->LQ);
							jp_state=JP_SEND_CHILD_REQ;
						}
						else
						{
							jp_state=JP_SEND_MCAST_PR_TO_ROUTER_REED;
							reponse_recived=0;
						}
					}
				}
				else
				{
					PRINTFG("[+] "ANSI_COLOR_RESET);
					PRINTF("JP received response from active Router \n"ANSI_COLOR_RESET);

					/* calculate the two-way link quality  */
					tlv=mle_find_tlv_in_cmd(param.rec_cmd,TLV_LINK_MARGIN);
					PRINTF("my rssi : %i \n", tlv->value[0]);
					PRINTF("rssi of parent : %i \n", param.rec_rssi);
					parent_condidate.LQ=calculate_two_way_LQ(tlv->value[0], param.rec_rssi);
					PRINTF("two-way link quality : %i \n", parent_condidate.LQ);

					/* identify the type of the device router/reed */
					tlv=mle_find_tlv_in_cmd(param.rec_cmd,TLV_SOURCE_ADDRESS);
					parent_condidate.is_Router=isActiveRouter(tlv->value[1] | (tlv->value[0] << 8));

					/* get number of neighbors with lq3 of the parent candidate */
					tlv=mle_find_tlv_in_cmd(param.rec_cmd,TLV_CONNECTIVITY);
					tlv_connectivity_init(&connectivity,tlv->value );
					parent_condidate.LQ3=connectivity->LQ3;

					if( current_parent.LQ < parent_condidate.LQ ||
							((current_parent.LQ == parent_condidate.LQ) && (current_parent.is_Router < parent_condidate.is_Router)) ||
							( (current_parent.LQ == parent_condidate.LQ) && (current_parent.is_Router == parent_condidate.is_Router) && (current_parent.LQ3 < parent_condidate.LQ3)))
					{

						/* update the current parent */
						current_parent.LQ=parent_condidate.LQ;
						current_parent.LQ3=parent_condidate.LQ3;
						current_parent.is_Router=parent_condidate.is_Router;

						/* save the two-way link quality */
						nb->LQ=current_parent.LQ;

						/* save the two-way link quality */
						nb->rec_rssi=param.rec_rssi;

						/* save the challenge TLV from the parent candidate */
						tlv=mle_find_tlv_in_cmd(param.rec_cmd,TLV_CHALLENGE);
						nb->challenge= (tlv->value[3]) | (tlv->value[2] << 8) | (tlv->value[1] << 16)| (tlv->value[0] << 24);

						/* save the address */
						uip_ip6addr_copy(&nb->tmp , &param.source_addr);// to verify *************
						tlv=mle_find_tlv_in_cmd(param.rec_cmd,TLV_SOURCE_ADDRESS);
						nb->address16=tlv->value[1] | (tlv->value[0] << 8);
					}

					reponse_recived=1;
					finish=1;
				}
				break;
			case JP_SEND_MCAST_PR_TO_ROUTER_REED:
				/* send parent request to the active router and reed and trigger the timer */
				PRINTFG("[+] "ANSI_COLOR_RESET);
				PRINTF("JP Send mcast parent request to active Router and REED \n"ANSI_COLOR_RESET);
				jp_state=JP_PARENT_SELECT;
				send_mle_parent_request(1,1);
				PRINTFG("[+] "ANSI_COLOR_RESET);
				PRINTF("JP Waiting for incoming response from active Router and REED \n"); PRESET();
				ctimer_set(&c_mle_Timer, 3 * bsp_get(E_BSP_GET_TRES) , mle_join_process, NULL);
				req_sent_to_reed=1;
				finish=1;
				break;
			case JP_SEND_CHILD_REQ:
				/* send child id request to the selected parent */
				PRINTFG("[+] "ANSI_COLOR_RESET);
				PRINTF("JP send Child ID Request \n"ANSI_COLOR_RESET);
				send_mle_childID_request(&nb->tmp);
				jp_state=JP_SAVE_PARENT;
				finish=1;
				break ;
			case JP_SAVE_PARENT:
				PRINTFG("[+] "ANSI_COLOR_RESET);
				PRINTF("JP Parent Stored \n"ANSI_COLOR_RESET);
				/* change the state of the parent to linked */
				nb->state=LINKED;

				/* start operating as child */
				mle_set_child_mode();
				finish=1;
				break ;
			case JP_FAIL:
				PRINTFR("Join process fail ... \n"ANSI_COLOR_RESET);
				PRINTFG("starting new partition ... \n"ANSI_COLOR_RESET);

				//thrd_partition_start();

				mle_set_parent_mode();
				PRESET();
				finish=1;
				break;
			}
		}
	}
}


/************************ MLE synchronisation process ****************************/

static uint8_t send_mle_link_request(void)
{
	mle_init_cmd(&cmd,LINK_REQUEST);
	add_src_address_to_cmd(&cmd);
	// leader data tlv :: from Network layer
	MyNode.challenge=add_rand_challenge_to_cmd(&cmd);
	add_version_to_cmd(&cmd);
	// TLV request : link Margin
	uip_create_linklocal_allrouters_mcast(&s_destAddr);
	mle_send_msg(&cmd, &s_destAddr);
	return 1 ;
}

static void reply_for_mle_link_request(void *ptr)
{

	uint8_t n;
	uint8_t* nb_id;
	nb_id= (uint8_t*) ptr;

	if(MyNode.OpMode== PARENT)
	{
		/* find the  */
		mle_init_cmd(&cmd,LINK_ACCEPT);
		add_src_address_to_cmd(&cmd);
		// leader data tlv :: from Network layer
		add_response_to_cmd(&cmd, mle_find_tlv_in_cmd(param.rec_cmd,TLV_CHALLENGE));
		// link-layer frame counter tlv
		add_version_to_cmd(&cmd);

		add_Link_margin_to_cmd(&cmd,param.rec_rssi);


		/*  If the recipient of a Link Request message has a valid frame
		 * counter for the sender, it sends a Link Accept in reply;
		 *  if not, it sends a Link Accept and Request.*/


		/* find the nb router */
		nb= mle_find_nb_router(*nb_id);
		if(nb->state==PENDING)
		{
			cmd.type=LINK_ACCEPT_AND_REQUEST;
			/* MLE Frame Counter tlv */
			// add frame counter
			/* add and store the challenge generated  */
			MyNode.challenge= add_rand_challenge_to_cmd(&cmd);

			/* we have to inform the syn process that we am waiting for link accept
			 * so we call the syn function with value of 255 */
			n=255;
			mle_synchro_process((void *) &n);
		}


		/* to change by the child address */
		mle_send_msg( &cmd,&nb->tmp);

	}

}

void mle_synchro_process(void *ptr)
{
	static 	syn_state_t 		syn_state;  // synchronisation process current state
	tlv_t * tlv;
	uint8_t finish=0;
	uint8_t * p= (uint8_t*) ptr;

	if (p) // this mean that this function was triggered when we received Link_accept_and_request
	{
		if(*p==255)
		{	/* in this case we have to be ready to receive a response */
			/* change the state and trigger the timer */
			syn_state=SYN_PROCESS_LINK;
			ctimer_set(&c_mle_Timer, 2 * bsp_get(E_BSP_GET_TRES) , mle_synchro_process, NULL);
			PRINTF("SNY process:  state changed due to link_accept_and_request command \n"ANSI_COLOR_RESET);
		}
		finish =1;
	}

	if(MyNode.OpMode==PARENT)
	{
		while(!finish)
		{
			switch(syn_state)
			{
			case SYN_SEND_LINK_REQUEST:
				PRINTFG("[+] "ANSI_COLOR_RESET);
				PRINTF("SNY process: Send Link Request to neighbors router \n"ANSI_COLOR_RESET);
				/* send MLE Link Request */
				send_mle_link_request();
				/* change the state and trigger the timer */
				syn_state=SYN_PROCESS_LINK;
				ctimer_set(&c_mle_Timer, 2 * bsp_get(E_BSP_GET_TRES) , mle_synchro_process, NULL);
				finish=1;
				break;
			case SYN_PROCESS_LINK:
				if(ctimer_expired(&c_mle_Timer)) // that means that this function was triggered by the ctimer
				{
					/* initialise the state */
					syn_state=SYN_SEND_LINK_REQUEST;
					PRINTFG("[+] "ANSI_COLOR_RESET);
					PRINTF("SNY process:  Synchronization process finished \n"ANSI_COLOR_RESET);

				}
				else
				{
					PRINTFG("[+] "ANSI_COLOR_RESET);
					PRINTF("SNY process: processing link \n"ANSI_COLOR_RESET);

					nb=mle_find_nb_router_byAdd((&param.source_addr));
					if(!nb)// the neighbor is not existing
					{
						/* store the neighbor data */
						uint8_t id=0;
						do
						{
							nb=mle_add_nb_router( id , 0 /*adress16*/ ,  0 /*frame counter*/, 0 /*modeTLV*/,  0 /*link quality */);
							id++;
						}
						while (nb==NULL && id<MAX_NB_ROUTER);

						if (nb)
						{
							/* increment the nb router counter */
							MyNode.NB_router_couter++;
							/* set the state of the nb to pending*/
							nb->state=LINKED;
							/* store the address of the nb router */
							uip_ip6addr_copy(&nb->tmp ,&param.source_addr);
							PRINTFG("[+] "ANSI_COLOR_RESET);
							PRINTF("SNY process: ID allocated for the nb router = %i \n", nb->id);
						}
						else
						{
							PRINTFR("[+] "ANSI_COLOR_RESET);
							PRINTFR("SNY process: Failed to allocate nb router id... \n"ANSI_COLOR_RESET);
							break;
						}
					}
					if(param.rec_cmd->type==LINK_ACCEPT_AND_REQUEST)
					{
						nb->state=LINKED;
						nb=mle_find_nb_router_byAdd(&param.source_addr);
						reply_for_mle_link_request(&nb->id);
					}
				}

				finish=1;
				break;
			}
		}
	}
}

/************************ process incoming MLE messages function ****************************/


static void  _mle_process_incoming_msg(struct udp_socket *c, void *ptr, const uip_ipaddr_t *source_addr, uint16_t source_port,
		const uip_ipaddr_t *dest_addr, uint16_t dest_port, const uint8_t *data, uint16_t datalen)
{

	mle_cmd_t * 	cmd;
	tlv_t* 			tlv;
	tlv_route64_t* 	route64_tlv;
	tlv_leader_t* 	leader_tlv;
	uint16_t MaxRand;
	uint8_t id=0;


	/*   To avoid the problem of the additional bytes until we fix it  ************/
	//		if( uip_is_addr_linklocal_allrouters_mcast(dest_addr) || uip_is_addr_linklocal_allnodes_mcast(dest_addr))
	mle_create_cmd_from_buff(&cmd , (uint8_t * )data , datalen - 40 );
	//		else
	//			mle_create_cmd_from_buff(&cmd , (uint8_t * )data , datalen );

	PRINTFC("<== MLE ");
	mle_print_type_cmd(*cmd);
	PRINTFC(" received from : ");
	PRINT6ADDR(source_addr); PRESET();
	mle_print_cmd(*cmd);

	switch (cmd->type)
	{
	case LINK_REQUEST:
		/* check if neighbors table is not full */
		if(MyNode.NB_router_couter<MAX_NB_ROUTER && MyNode.OpMode== PARENT)
		{

			/* check the nb already exist */
			nb=mle_find_nb_router_byAdd((uip_ipaddr_t*)source_addr);
			if(!nb)// the neighbor is not existing
			{
				/* allocate temporarily a neighbor */
				id=0;
				do
				{
					nb=mle_add_nb_router( id , 0 /*adress16*/ ,  0 /*frame counter*/, 0 /*modeTLV*/,  0 /*link quality */);
					id++;
				}
				while (nb==NULL && id<MAX_NB_ROUTER);

				if (nb)
				{
					/* increment the nb router counter */
					MyNode.NB_router_couter++;
					/* set the state of the nb to pending*/
					nb->state=PENDING;
					/* store the address of the nb router */
					uip_ip6addr_copy(&nb->tmp ,source_addr);
					PRINTF("ID allocated for the nb router = %i \n", nb->id);
				}
				else
				{
					PRINTFR("Failed to allocate nb router id...");PRESET();
					break;
				}
			}
			/* prepare param */
			param.rec_cmd=cmd;
			param.rec_rssi=get_rssi();
			uip_ip6addr_copy(&param.source_addr,source_addr);

			/* trigger the timer to reply after the random period */
			ctimer_set(&nb->timeOut, bsp_getrand(500) *  (bsp_get(E_BSP_GET_TRES) / 1000 ) , reply_for_mle_link_request, (void *) &nb->id );

		}
		break;
	case LINK_ACCEPT:
		if (MyNode.OpMode==PARENT)
		{
			/* check the response TLV before processing the parent response */
			tlv=mle_find_tlv_in_cmd(cmd,TLV_RESPONSE);
			if(comp_resp_chall(MyNode.challenge,tlv->value))
			{
				param.rec_cmd=cmd;
				param.rec_rssi=get_rssi();
				uip_ip6addr_copy(&param.source_addr,source_addr);

				mle_synchro_process(NULL);

			}
		}
		break;
	case LINK_ACCEPT_AND_REQUEST:
		if (MyNode.OpMode==PARENT)
		{
			/* check the response TLV before processing the parent response */
			tlv=mle_find_tlv_in_cmd(cmd,TLV_RESPONSE);
			if(comp_resp_chall(MyNode.challenge,tlv->value))
			{
				param.rec_cmd=cmd;
				param.rec_rssi=get_rssi();
				uip_ip6addr_copy(&param.source_addr,source_addr);

				mle_synchro_process(NULL);

			}
		}
		break;
	case LINK_REJECT:
		break;
	case ADVERTISEMENT:

		if(MyNode.OpMode == PARENT)
		{
			tlv=mle_find_tlv_in_cmd(cmd,TLV_ROUTE64);
			tlv_route64_init(&route64_tlv,tlv->value);
			tlv=mle_find_tlv_in_cmd(cmd,TLV_LEADER_DATA);
			tlv_leader_init(&leader_tlv,tlv->value);
			tlv=mle_find_tlv_in_cmd(cmd,TLV_SOURCE_ADDRESS);
			thrd_process_adv( tlv->value[1] | (tlv->value[0] << 8), route64_tlv,leader_tlv);
			PRINTFY("MLE advertisement processed ...");PRESET();
		}
		break;

	case UPDATE:
		break;
	case UPDATE_REQUEST:  // Not used in Thread Network
		break;
	case DATA_REQUEST:  // Not used in Thread Network
		break;
	case DATA_RESPONSE:
		break;
	case PARENT_REQUEST:
		/* check if child table is not full */
		if(MyNode.childs_counter<MAX_CHILD && MyNode.OpMode!= NOT_LINKED)
		{
			/* check: A Router MUST NOT send an MLE  Parent Response if:
					It is disconnected from its Partition (that is, it has not received
					an updated ID sequence number within LEADER_TIMEOUT seconds
				OR
		 			Its current routing path cost to the Leader is infinite.
			 */

			/* check the scan mask TLV to know if i should reply or not */
			tlv=mle_find_tlv_in_cmd(cmd,TLV_SCAN_MASK);
			if((MyNode.OpMode==PARENT && routerMask(tlv->value[0]) ) || (MyNode.OpMode==CHILD && ReedMask(tlv->value[0]) ) )
			{
				/* handle the random time to wait before reply */
				if(ReedMask(tlv->value[0]))
					MaxRand=1000;// max response after 1000 ms
				else
					MaxRand=500;// max response after 500 ms

				/* find the mode tlv of the sender in order to store it */
				tlv=mle_find_tlv_in_cmd(cmd,TLV_MODE);

				/* allocate temporarily a child */
				id=1; // don't use the id zero for a child : problem with the short address
				do
				{
					child=mle_add_child( id , 0 /*adress16*/ ,  0 /*frame counter*/, tlv->value[0] /*modeTLV*/,  0 /*link quality */);
					id++;
				}
				while (child==NULL && id<MAX_CHILD);

				if (child)
				{
					/* increment the child counter */
					MyNode.childs_counter++;
					/* set the state of the child to pending*/
					child->state=PENDING;
					/* store the address of the child */
					uip_ip6addr_copy(&child->tmp ,source_addr);
					PRINTF("ID allocated for the child = %i \n", child->id);
				}
				else
				{
					PRINTFR("Failed to allocate child id ...");PRESET();
					break;
				}

				/* trigger the timer to remove the child after period of pending time (if the state wasn't changed to linked) */
				ctimer_set(&child->timeOut, 5 * bsp_get(E_BSP_GET_TRES) , check_child_state, (void *) &child->id );

				/* prepare param */
				param.rec_cmd=cmd;
				param.rec_rssi=get_rssi();
				uip_ip6addr_copy(&param.source_addr,source_addr);

				/* trigger the timer to reply after the random period */
				ctimer_set(&child->timeOut, bsp_getrand(MaxRand) *  (bsp_get(E_BSP_GET_TRES) / 1000 ) , reply_for_mle_parent_request, (void *) &child->id );

			}
		}
		break;
	case PARENT_RESPONSE:

		if (MyNode.OpMode==NOT_LINKED)
		{
			/* check the response TLV before processing the parent response */
			tlv=mle_find_tlv_in_cmd(cmd,TLV_RESPONSE);
			if(comp_resp_chall(MyNode.challenge,tlv->value))
			{
				param.rec_cmd=cmd;
				param.rec_rssi=get_rssi();
				uip_ip6addr_copy(&param.source_addr,source_addr);


				mle_join_process(NULL);

			}
		}
		break;
	case CHILD_ID_REQUEST:
		// dont forget to increment the child counter

		/*
		 * If the Mode TLV indicates that the sender is an rx-off-when-idle device, it
		 * MUST begin sending IEEE 802.15.4 data requests after sending the Child ID request
		 */

		/* find child using source address */
		child= mle_find_child_byAdd( (uip_ipaddr_t *) source_addr);
		if(!child)
		{
			PRINTFY("Child not found ...");PRESET();
		}

		/* check the response TLV before processing the parent response */
		tlv=mle_find_tlv_in_cmd(cmd,TLV_RESPONSE);
		if(comp_resp_chall(child->challenge ,tlv->value))
		{
			param.rec_cmd=cmd;
			param.rec_rssi=get_rssi();
			//param.source_addr=(uip_ipaddr_t *) source_addr;
			uip_ip6addr_copy(&param.source_addr,source_addr);
			reply_for_mle_childID_request(NULL);
		}

		break;
	case CHILD_ID_RESPONSE:
		mle_join_process(NULL);
		break;
	case CHILD_UPDATE:
		break;
	case CHILD_UPDATE_RESPONSE:
		break;
	default :
		PRINTFR( "Error MLE received command not recognized \n"ANSI_COLOR_RESET);
		break;
	}

}

/************************ init MLE protocol  ****************************/

static uint8_t mle_init_udp(void)
{
	udp_socket_close(&MyNode.udp_socket);
	udp_socket_register(&MyNode.udp_socket, NULL, _mle_process_incoming_msg);
	udp_socket_bind(&MyNode.udp_socket, MLE_UDP_LPORT);
	udp_socket_connect(&MyNode.udp_socket,NULL, MLE_UDP_RPORT);

	if (&MyNode.udp_socket.udp_conn == NULL)
	{
		PRINTF("No UDP connection available, error!\n");
		return 0;
	}

	PRINTFY( "\nMLE UDP initialized : \n lport --> %u \n rport --> %u \n"ANSI_COLOR_RESET , UIP_HTONS(MyNode.udp_socket.udp_conn->lport),
			UIP_HTONS(MyNode.udp_socket.udp_conn->rport));

	return 1 ;
}

/************************  MLE send message function  ****************************/

static uint8_t mle_send_msg(mle_cmd_t* cmd,  uip_ipaddr_t *dest_addr )
{
	if (&MyNode.udp_socket.udp_conn == NULL)
	{
		PRINTF("reinitialize UPD");
		if(!mle_init_udp())
			return 0;
	}

	//	PRINTF("data sent length : %i \n", cmd->used_data+1);
	if(!udp_socket_sendto(&MyNode.udp_socket, (uint8_t*) cmd , cmd->used_data+1 , dest_addr ,MLE_UDP_RPORT))
	{
		PRINTF(ANSI_COLOR_RED "Failed to send MLE msg\n"ANSI_COLOR_RESET);
		return 0;
	}

	PRINTFC("==> MLE ");
	mle_print_type_cmd(*cmd);
	PRINTFC( " sent to : "  );
	PRINT6ADDR(dest_addr);
	PRINTF(ANSI_COLOR_RESET "\n"  );

	return 1 ;


} /* _mle_sendMsg */

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/


/**
 * @brief MLE protocol initialization
 */
uint8_t mle_init(void)
{

	/*  Inisialize  mle udp connexion */
	if(!mle_init_udp()) return 0 ;

	/* Inisialize  nb table */
	mle_nb_device_init();

	/* Inisialize  mle node structure */
	MyNode.OpMode=NOT_LINKED;
	MyNode.timeOut=TIME_OUT;
	MyNode.rx_on_when_idle=IS_RX_ON_WHEN_IDLE;
	MyNode.childs_counter=0;



	/**************  test ******************/


	/**********************/
	ctimer_set(&c_mle_Timer, 1 * bsp_get(E_BSP_GET_TRES) , mle_join_process, (void *) NULL );


	PRINTFG( "MLE protocol initialized ... ");PRESET();
	return 1;

};

uint8_t mle_set_parent_mode(void)
{
	MyNode.OpMode=PARENT; // router
	PRINTFG( "MLE : Node operate as Parent ... "ANSI_COLOR_RESET);
	PRESET();

	/* clear the neighbors table  */
	mle_rm_all_nb_router();
	/* start synchronisation with other routers */
	mle_synchro_process(NULL);

	return 1 ;
}

uint8_t mle_set_child_mode(void)
{
	MyNode.OpMode=CHILD; // router
	PRINTFG( "MLE : Node operate as Child ... "ANSI_COLOR_RESET);
	PRESET();
	/* clear the neighbors table  */
	// clear the nb table

	return 1 ;
}
