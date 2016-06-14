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
struct  ctimer                      c_mle_Timer;
static 	mle_cmd_t 					cmd; // command buffer
static mle_param_t					param;
static mle_neighbor_t*				nb;

#define get_rssi()		             30 //sicslowpan_get_last_rssi()
#define get_routerID()		         0xff
typedef enum {
	JP_SEND_MCAST_PR_TO_ROUTER,
	JP_SEND_MCAST_PR_TO_ROUTER_REED,
	JP_WAIT_1,
	JP_WAIT_2,
	JP_PARENT_SELECT,
	JP_SEND_CHILD_REQ,
	JP_SAVE_PARENT,
	JP_FAIL,
}jp_state_t; // join process state

/*==============================================================================
								LOCAL FUNCTION
 =============================================================================*/
static uint8_t  		mle_send_msg(mle_cmd_t* cmd,  uip_ipaddr_t *dest_addr);
#define routerMask(scanMask)			(BIT_CHECK(scanMask,7))
#define ReedMask(scanMask)				(BIT_CHECK(scanMask,6))

/************************ join process function ****************************/

/*
 * @brief Send multicast MLE parent request
 * @return 1    OK.
 * @return 0   FAIL.
 *
 */

static uint8_t send_mle_parent_request(uint8_t R, uint8_t E)
{
	mle_init_cmd(&cmd,PARENT_REQUEST);
	add_mode_RSDN_to_cmd(&cmd, MyNode.rx_on_when_idle, 0, IS_FFD, 0);
	MyNode.jp_challenge=add_rand_challenge_to_cmd(&cmd);
	add_scan_mask_to_cmd(&cmd,R,E);
	add_version_to_cmd(&cmd);
	uip_create_linklocal_allrouters_mcast(&s_destAddr);
	mle_send_msg(&cmd, &s_destAddr);
	return 1 ;
}

static uint8_t send_mle_childID_request(uip_ipaddr_t *dest_addr)
{
	mle_init_cmd(&cmd,CHILD_ID_REQUEST);
	// response tlv
	add_response32_to_cmd(&cmd, nb->challenge);
	// link-layer frame counter tlv
	// MLE Frame Counter tlv
	add_mode_RSDN_to_cmd(&cmd, MyNode.rx_on_when_idle, 0, IS_FFD, 0);
	add_time_out_to_cmd(&cmd,TIME_OUT);
	add_version_to_cmd(&cmd);
	mle_send_msg(&cmd, dest_addr);
	return 1 ;
}

static void reply_for_mle_parent_request(void *ptr)
{


	if(MyNode.OpMode!= NOT_LINKED)
	{
		/*********** checking the scan mask tlv before sending any response **************/

		mle_init_cmd(&cmd,PARENT_RESPONSE);
		add_src_address_to_cmd(&cmd);
		// leader data tlv :: from Network layer
		// link-layer frame counter tlv
		// MLE Frame Counter tlv
		add_response_to_cmd(&cmd, mle_find_tlv_in_cmd(param.rec_cmd,TLV_CHALLENGE));
		// i should save the challenge for the current child
		/*  child.challenge = */	add_rand_challenge_to_cmd(&cmd);
		add_Link_margin_to_cmd(&cmd,param.rec_rssi);
		// connectivity tlv
		add_Cnnectivity_to_cmd(&cmd,MAX_CHILD,MyNode.childs_counter,count_neighbor_LQ(3),count_neighbor_LQ(2),count_neighbor_LQ(1),0,0);
		add_version_to_cmd(&cmd);

		mle_send_msg( &cmd,&param.source_addr);

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
		mle_set_parent_mode();

		mle_init_cmd(&cmd,CHILD_ID_RESPONSE);
		add_src_address_to_cmd(&cmd);
		// leader data tlv :: from Network layer
		add_address16_to_cmd(&cmd , assign_address16(5));
		// router64
		mle_send_msg( &cmd,&param.source_addr);
	}
	else if (MyNode.OpMode == PARENT)
	{
		mle_init_cmd(&cmd,CHILD_ID_RESPONSE);
		add_src_address_to_cmd(&cmd);
		// leader data tlv :: from Network layer
		add_address16_to_cmd(&cmd , assign_address16(5));
		// router64
		mle_send_msg( &cmd,&param.source_addr);
	}
}

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




void mle_join_process(void *ptr)
{
	static 	jp_state_t 		jp_state ;  // join process current state
	static mle_parent_t 	parent; // parent candidate


	tlv_connectivity_t* connectivity;
	tlv_t * tlv;

	uint8_t finish=0;
	static  uint8_t 		flag=0; // parent response received


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
				//parent.two_way_LQ=0;
				//parent.jp_challenge=0;
				//parent.is_Router=0;
				//parent.LQ3=0;
				nb=mle_add_nb_router(0, 0, 0, 0, 0);
				if(nb==NULL)
					nb=mle_find_nb_router(0);
				nb->state=0;
				nb->LQ=0;
				nb->address16=0;


				jp_state=JP_WAIT_1;
				send_mle_parent_request(1,0);
				PRINTFG("[+] "ANSI_COLOR_RESET);
				PRINTF("JP Waiting for incoming response from active Router\n"ANSI_COLOR_RESET);
				ctimer_set(&c_mle_Timer, 4 * bsp_get(E_BSP_GET_TRES) , mle_join_process, NULL);

				finish=1;
				break;
			case JP_WAIT_1:
				if(ctimer_expired(&c_mle_Timer)) // that means that this function was triggered by the ctimer
				{
					if(!flag  || nb->LQ < 3)
					{
						jp_state=JP_SEND_MCAST_PR_TO_ROUTER_REED;
						flag=0;
					}
					else
						jp_state=JP_PARENT_SELECT;
				}
				else
				{
					PRINTFG("[+] "ANSI_COLOR_RESET);
					PRINTF("JP received response from active Router \n"ANSI_COLOR_RESET);
					tlv=mle_find_tlv_in_cmd(param.rec_cmd,TLV_LINK_MARGIN);

					PRINTF("rssi of parent : %i \n"ANSI_COLOR_RESET, param.rec_rssi);
					PRINTF("my rssi : %i \n"ANSI_COLOR_RESET, tlv->value[0]);

					parent.LQ=calculate_two_way_LQ(tlv->value[0], param.rec_rssi);
					PRINTF("two-way quality : %i \n"ANSI_COLOR_RESET, parent.LQ);

					if( nb->LQ < parent.LQ)
					{
						nb->LQ=parent.LQ;

						/* get the challenge tlv from the parent condidate */
						tlv=mle_find_tlv_in_cmd(param.rec_cmd,TLV_CHALLENGE);
						nb->challenge= (tlv->value[3]) | (tlv->value[2] << 8) | (tlv->value[1] << 16)| (tlv->value[0] << 24);
						//nb->challenge=0xABCDEF55;
						/* get number of neighbors with lq3 of the parent condidate */
						tlv=mle_find_tlv_in_cmd(param.rec_cmd,TLV_CONNECTIVITY);
						tlv_connectivity_init(&connectivity,tlv->value );
						parent.LQ3=connectivity->LQ3;

						/* save the address (to remplace with source address  tlv)  */
						uip_ip6addr_copy(&nb->tmp , &param.source_addr);

						/* identify the type of parent : router or reed from the sourse address */
						//parent.is_Router=....;

					}


					flag=1;
					finish=1;
				}
				break;
			case JP_SEND_MCAST_PR_TO_ROUTER_REED:
				PRINTFG("[+] "ANSI_COLOR_RESET);
				PRINTF("JP Send mcast parent request to active Router and REED \n"ANSI_COLOR_RESET);
				jp_state=JP_WAIT_2;
				send_mle_parent_request(1,1);
				PRINTFG("[+] "ANSI_COLOR_RESET);
				PRINTF("JP Waiting for incoming response from active Router and REED \n"); PRESET();
				ctimer_set(&c_mle_Timer, 6 * bsp_get(E_BSP_GET_TRES) , mle_join_process, NULL);
				finish=1;
				break;
			case JP_WAIT_2:
				if(ctimer_expired(&c_mle_Timer)) // that means that this function was triggered by the ctimer
				{
					if(!flag)
						jp_state=JP_FAIL;
					else
						jp_state=JP_PARENT_SELECT;
				}
				else
				{
					PRINTFG("[+] "ANSI_COLOR_RESET);
					PRINTF("JP received response from active Router or REED \n"ANSI_COLOR_RESET);

					tlv=mle_find_tlv_in_cmd(param.rec_cmd,TLV_LINK_MARGIN);

					PRINTF("rssi of parent : %i \n"ANSI_COLOR_RESET, param.rec_rssi);
					PRINTF("my rssi : %i \n"ANSI_COLOR_RESET, tlv->value[0]);

					parent.LQ=calculate_two_way_LQ(tlv->value[0], param.rec_rssi);
					PRINTF("two-way quality : %i \n"ANSI_COLOR_RESET, parent.LQ);

					if( nb->LQ < parent.LQ)
					{
						nb->LQ=parent.LQ;

						/* get the challenge tlv from the parent condidate */
						tlv=mle_find_tlv_in_cmd(param.rec_cmd,TLV_CHALLENGE);
						nb->challenge= (tlv->value[3]) | (tlv->value[2] << 8) | (tlv->value[1] << 16)| (tlv->value[0] << 24);
						//nb->challenge=0xABCDEF55;
						/* get number of neighbors with lq3 of the parent condidate */
						tlv=mle_find_tlv_in_cmd(param.rec_cmd,TLV_CONNECTIVITY);
						tlv_connectivity_init(&connectivity,tlv->value );
						parent.LQ3=connectivity->LQ3;

						/* save the address (to remplace with source address  tlv)  */
						uip_ip6addr_copy(&nb->tmp , &param.source_addr);

						/* identify the type of parent : router or reed from the sourse address */
						//parent.is_Router=....;

					}
					flag=1;
					finish=1;
				}
				break;
			case JP_PARENT_SELECT:
				PRINTFG("[+] "ANSI_COLOR_RESET);
				PRINTF("JP Parent selection \n"ANSI_COLOR_RESET);
				PRINTFG("link quality with parent : %i \n"ANSI_COLOR_RESET, nb->LQ);



				jp_state=JP_SEND_CHILD_REQ;;
				break ;
			case JP_SEND_CHILD_REQ:
				PRINTFG("[+] "ANSI_COLOR_RESET);
				PRINTF("JP send Child ID Request \n"ANSI_COLOR_RESET);
				send_mle_childID_request(&nb->tmp);
				jp_state=JP_SAVE_PARENT;
				finish=1;
				break ;
			case JP_SAVE_PARENT:
				PRINTFG("[+] "ANSI_COLOR_RESET);
				PRINTF("JP Parent Stored \n"ANSI_COLOR_RESET);
				mle_set_child_mode();
				finish=1;
				break ;
			case JP_FAIL:
				PRINTFR("Join process fail ... \n"ANSI_COLOR_RESET);
				PRINTFG("starting new partition ... \n"ANSI_COLOR_RESET);
				// call to Lukas start partition function
				mle_set_parent_mode();
				PRESET();
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

	mle_cmd_t * cmd;
	tlv_t * tlv;

	/***********   To avoid the problem of the additional bytes until we fix it  ************/
		if( uip_is_addr_linklocal_allrouters_mcast(dest_addr) || uip_is_addr_linklocal_allnodes_mcast(dest_addr))
			mle_create_cmd_from_buff(&cmd , (uint8_t * )data , datalen - 40 );
		else
			mle_create_cmd_from_buff(&cmd , (uint8_t * )data , datalen );

	PRINTFC("<== MLE ");
	mle_print_type_cmd(*cmd);
	PRINTFC(" received from : ");
	PRINT6ADDR(source_addr); PRESET();
	mle_print_cmd(*cmd);

	switch (cmd->type)
	{
	case LINK_REQUEST:
		break;
	case LINK_ACCEPT:
		break;
	case LINK_ACCEPT_AND_REQUEST:
		break;
	case LINK_REJECT:
		break;
	case ADVERTISEMENT:
		tlv=mle_find_tlv_in_cmd(cmd,TLV_ROUTE64);
		// Lukas_process_route64(tlv->length , tlv->value);
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
		// check if child table is not full

		if(MyNode.childs_counter<MAX_CHILD && MyNode.OpMode!= NOT_LINKED)
		{
			/* check: A Router MUST NOT send an MLE  Parent Response if:
			It is disconnected from its Partition (that is, it has not received
			an updated ID sequence number within LEADER_TIMEOUT seconds
			OR
		 	Its current routing path cost to the Leader is infinite.
			 */

			// generate random number depending on scan mask and then reply

			/* check the scan mask TLV to know if i should reply or not */
			tlv=mle_find_tlv_in_cmd(cmd,TLV_SCAN_MASK);
			if((MyNode.OpMode==PARENT && routerMask(tlv->value[0]) ) || (MyNode.OpMode==CHILD && ReedMask(tlv->value[0]) ) )
			{
				uint16_t MaxRand; //random delay before reply
				if(ReedMask(tlv->value[0]))
					MaxRand=1000;// max response after 1000 ms
				else
					MaxRand=500;// max response after 500 ms


				/*
				mle_neighbor_t * nb;
				tlv=mle_find_tlv_in_cmd(cmd,TLV_MODE);
				uint8_t id=0;
				do
				{
					id++;
					nb=mle_add_child( id ,(uip_ipaddr_t *) source_addr,  0 , tlv->value[0],  get_rssi());

				}while (nb==NULL);
				ctimer_set(&nb->timeOut, bsp_getrand(MaxRand) * bsp_get(E_BSP_GET_TRES)/1000 , reply_for_mle_parent_request, (void *) id );

				 */

				// prepare param
				param.rec_cmd=cmd;
				param.rec_rssi=get_rssi();
				//param.source_addr=(uip_ipaddr_t *) source_addr;
				uip_ip6addr_copy(&param.source_addr,source_addr);
				ctimer_set(&c_mle_Timer, /*bsp_getrand(MaxRand)* */ (bsp_get(E_BSP_GET_TRES) /*/ 1000*/ ) , reply_for_mle_parent_request, (void *) NULL );

			}
		}
		break;
	case PARENT_RESPONSE:

		if (MyNode.OpMode==NOT_LINKED)
		{
			/* check the response TLV before processing the parent response */
			tlv=mle_find_tlv_in_cmd(cmd,TLV_RESPONSE);
			if(comp_resp_chall(MyNode.jp_challenge,tlv->value))
			{
				param.rec_cmd=cmd;
				param.rec_rssi=get_rssi();
				//param.source_addr=(uip_ipaddr_t *) source_addr;
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
		/* check the response TLV before processing the parent response */

		//find child using source address

		// get child challenge


		//tlv=mle_find_tlv_in_cmd(cmd,TLV_RESPONSE); // i should save the challenge when i send the parent response
		if(1 /*comp_resp_chall(child.challenge ,tlv->value)*/)
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
 *
 */
uint8_t mle_init(void)
{

	/***********  Inisialize  mle udp connexion ***************/
	if(!mle_init_udp()) return 0 ;

	/***********  Inisialize  mle node structure **************/
	MyNode.OpMode=NOT_LINKED;
	MyNode.timeOut=TIME_OUT;
	MyNode.rx_on_when_idle=IS_RX_ON_WHEN_IDLE;
	MyNode.childs_counter=0;

/*
	 mle_add_child(1, 0x00aa,  50441 ,0,3 );
	mle_add_child(2, 0x00aa,  441 ,4,1);
	mle_add_child(3, 0x00aa,  50 ,1,3);
	mle_add_child(4, 0x00aa,  444 ,2,3);
	//mle_rm_child( mle_find_child(2));
	mle_print_child_table();

	mle_add_nb_router(1, 0x00aa,  50441 ,0,3);
	mle_add_nb_router(2, 0x00aa,  441 ,4,1);
	mle_add_nb_router(3, 0x00aa,  50 ,1,2);
	mle_add_nb_router(4, 0x00aa,  444 ,2,3);
	//mle_rm_nb_router( mle_find_nb_router(1));
	mle_print_nb_router_table();

	PRINTF("nb of lq 3  : %i \n"ANSI_COLOR_RESET, count_neighbor_LQ(3));
	PRINTF("nb of lq 2  : %i \n"ANSI_COLOR_RESET, count_neighbor_LQ(2));
	PRINTF("nb of lq 1  : %i \n"ANSI_COLOR_RESET, count_neighbor_LQ(1));
*/

	/************* just for test: to verify when the join process will be triggered  **********/
	ctimer_set(&c_mle_Timer, 1 * bsp_get(E_BSP_GET_TRES) , mle_join_process, (void *) NULL );


	PRINTFG( "MLE protocol initialized ... ");PRESET();
	return 1;

};

uint8_t mle_set_parent_mode(void)
{
	MyNode.OpMode=PARENT; // router
	PRINTFG( "MLE : Node operate as Parent ... "ANSI_COLOR_RESET);
	PRESET();
	// may be start synchronisation with other routers
	return 1 ;
}

uint8_t mle_set_child_mode(void)
{
	MyNode.OpMode=CHILD; // router
	PRINTFG( "MLE : Node operate as Child ... "ANSI_COLOR_RESET);
	PRESET();
	// may be remove neighbors etc // start join process etc
	return 1 ;
}
