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

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

static mle_node_t MyNode;
static  uip_ipaddr_t                s_destAddr;
struct  etimer                      e_mle_udp_Timer;
struct  ctimer                      c_mle_udp_Timer;




typedef enum {
	JP_SEND_MCAST_PR_TO_ROUTER,
	JP_SEND_MCAST_PR_TO_ROUTER_REED,
	JP_WAIT_1,
	JP_WAIT_2,
	JP_PARENT_SELECT,
	JP_FAIL,
}jp_state_t; // join process state

/*==============================================================================
								LOCAL FUNCTION
 =============================================================================*/
static uint8_t mle_send_msg(mle_cmd_t* cmd, const uip_ipaddr_t *dest_addr );

static void  _mle_process_incoming_msg(struct udp_socket *c, void *ptr, const uip_ipaddr_t *source_addr, uint16_t source_port,
		const uip_ipaddr_t *dest_addr, uint16_t dest_port, const uint8_t *data, uint16_t datalen)
{

	mle_cmd_t * cmd;

	/***********   To avoid the problem of the additional bytes until we fix it  ************/
	if( uip_is_addr_linklocal_allrouters_mcast(dest_addr) || uip_is_addr_linklocal_allnodes_mcast(dest_addr))
		mle_create_cmd_from_buff(&cmd , (uint8_t * )data , datalen - 40 );
	else
		mle_create_cmd_from_buff(&cmd , (uint8_t * )data , datalen );


	PRINTF(ANSI_COLOR_CYAN "MLE msg received from : ");
	PRINT6ADDR(source_addr);
	PRINTF(ANSI_COLOR_RESET "\ndata received length : %i \n", datalen);
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
		break;
	case PARENT_RESPONSE:
		break;
	case CHILD_ID_REQUEST:
		break;
	case CHILD_ID_RESPONSE:
		break;
	case CHILD_UPDATE:
		break;
	case CHILD_UPDATE_RESPONSE:
		break;
	default :
		PRINTFR( "Error MLE received command not recognized \n"ANSI_COLOR_RESET);
		break;

	}
	//mle_send_msg(cmd,  source_addr );

}





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


static uint8_t mle_send_msg(mle_cmd_t* cmd, const uip_ipaddr_t *dest_addr )
{
	if (&MyNode.udp_socket.udp_conn == NULL)
	{
		PRINTF("reinitialize UPD");
		if(!mle_init_udp())
			return 0;
	}

	PRINTF("data sent length : %i \n", cmd->used_data+1);
	if(!udp_socket_sendto(&MyNode.udp_socket, (uint8_t*) cmd , cmd->used_data+1 , dest_addr ,MLE_UDP_RPORT))
	{
		PRINTF(ANSI_COLOR_RED "Failed to send MLE msg\n"ANSI_COLOR_RESET);
		return 0;
	}
	PRINTF( ANSI_COLOR_GREEN "MLE msg  sent to : "  );
	PRINT6ADDR(dest_addr);
	PRINTF(ANSI_COLOR_RESET "\n"  );

	return 1 ;


} /* _mle_sendMsg */




static void _mle_sendMsg(c_event_t c_event,  p_data_t p_data )
{

	if (etimer_expired(&e_mle_udp_Timer))
	{

		/************************************************************************/

		mle_cmd_t cmd;
		mle_init_cmd(&cmd,LINK_ACCEPT);
		mle_add_tlv_to_cmd(&cmd , TLV_SOURCE_ADDRESS, 5,  (uint8_t*) "haaah" );
		add_version_to_cmd(&cmd);
		add_scan_mask_to_cmd(&cmd,1,1);
		add_mode_RSDN_to_cmd(&cmd, 0 , 0 , 0 , 1 );
		add_time_out_to_cmd (&cmd, MyNode.timeOut);
		add_src_address_to_cmd(&cmd);
		add_status_to_cmd(&cmd);
		add_rand_challenge_to_cmd(&cmd );

		uip_create_linklocal_allrouters_mcast(&s_destAddr);
		mle_send_msg( &cmd, &s_destAddr );

		/************************************************************************/

		etimer_restart(&e_mle_udp_Timer);
	}



}



/************************ join process ocal function ****************************/



/**
 * @brief Send multicast MLE parent request
 * @return 1    OK.
 * @return 0   FAIL.
 *
 */

static uint8_t send_mle_parent_request( uint8_t R , uint8_t E )
{
	mle_cmd_t cmd;
	mle_init_cmd(&cmd,PARENT_REQUEST);
	add_mode_RSDN_to_cmd(&cmd, MyNode.rx_on_when_idle , 0 , IS_FFD , 1 );
	MyNode.jp_challenge= add_rand_challenge_to_cmd(&cmd );
	add_scan_mask_to_cmd(&cmd,R,E);
	add_version_to_cmd(&cmd);
	uip_create_linklocal_allrouters_mcast(&s_destAddr);
	mle_send_msg( &cmd, &s_destAddr );
	return 1 ;
}

void mle_join_process( void *ptr )
{
	static 	jp_state_t 		jp_state ;  // join process current state
	uint8_t finish=0;

	while(!finish)
	{
		switch(jp_state)
		{
		case JP_SEND_MCAST_PR_TO_ROUTER:
			PRINTFG("Send mcast parent request to active router \n");PRESET();
			send_mle_parent_request(1 ,0 );
			jp_state=JP_WAIT_1;
			break;
		case JP_WAIT_1:
			PRINTFG("Waiting for incoming response \n"); PRESET();
			ctimer_set(&c_mle_udp_Timer, 5 * bsp_get(E_BSP_GET_TRES) , mle_join_process, NULL );
			finish=1;
			jp_state=JP_PARENT_SELECT;
			break;
		case JP_SEND_MCAST_PR_TO_ROUTER_REED:
			send_mle_parent_request(1 ,0 );
			jp_state=JP_WAIT_2;
			break;
		case JP_WAIT_2:
			ctimer_set(&c_mle_udp_Timer, 5 * bsp_get(E_BSP_GET_TRES) , mle_join_process, NULL );
			jp_state=JP_PARENT_SELECT;
			finish=1;
			break;
		case JP_PARENT_SELECT:
			etimer_reset(&e_mle_udp_Timer);
			PRINTFG("JP Parent selection \n");PRESET();
			finish=1;
			break;
		case JP_FAIL:
			break;
		}
	}

}

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



	//etimer_set( &e_mle_udp_Timer, 2 * bsp_get(E_BSP_GET_TRES), mle_join_process);
	ctimer_set(&c_mle_udp_Timer, 5 * bsp_get(E_BSP_GET_TRES) , mle_join_process, NULL );
	PRINTFG( "MLE protocol initialized ...  \n");PRESET();
	return 1;

};

