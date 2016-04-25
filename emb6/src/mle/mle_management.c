
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

#define		DEBUG		DEBUG_PRINT

#include "uip-debug.h"	// For debugging terminal output.

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

static mle_node_t node;

static  struct  udp_socket          st_udp_mle_socket;

static  uip_ipaddr_t                s_destAddr;
struct  etimer                      e_mle_udp_Timer;



/** set the send interval */
#define     SEND_INTERVAL                7

/*==============================================================================
								LOCAL FUNCTION
 =============================================================================*/
static void  _mle_call_back(struct udp_socket *c, void *ptr, const uip_ipaddr_t *source_addr, uint16_t source_port,
		const uip_ipaddr_t *dest_addr, uint16_t dest_port, const uint8_t *data, uint16_t datalen)
{

	mle_cmd_t * cmd;
	mle_create_cmd_from_buff(&cmd , (uint8_t * )data , datalen - 40 );

	PRINTF("data received length : %i \n", datalen);
	PRINTF("data used : %i \n", cmd->used_data);
	mle_print_cmd(*cmd);

	udp_socket_sendto(&st_udp_mle_socket, (uint8_t*) cmd , cmd->used_data+1 , source_addr , MLE_UDP_RPORT);

}


static void _mle_sendMsg(c_event_t c_event, p_data_t p_data )
{

	if (etimer_expired(&e_mle_udp_Timer))
	{

		/************************************************************************/

		mle_cmd_t cmd;
		mle_init_cmd(&cmd,LINK_ACCEPT);
		mle_add_tlv_to_cmd(&cmd , TLV_SOURCE_ADDRESS, 5,  (uint8_t*) "haaah" );
		mle_add_tlv_to_cmd(&cmd , TLV_MODE, 2,  (uint8_t*) "bbaah" );
		mle_add_tlv_to_cmd( &cmd ,TLV_VERSION , 2 ,(uint8_t*) "haaaaaa" );
		mle_add_tlv_to_cmd( &cmd ,TLV_STATUS , 4 , (uint8_t*) "hcch" );
		mle_add_tlv_to_cmd( &cmd ,TLV_RESPONSE , 1 ,(uint8_t*)  "a" );
		add_src_address_tlv_to_cmd(&cmd);
		print_buffer((uint8_t*) &cmd , cmd.used_data+1);
		//mle_print_cmd(cmd);


		PRINTF("data sent length : %i ", cmd.used_data+1);
		PRINTF("data used : %i \n", cmd.used_data);

		//udp_socket_send(&st_udp_mle_socket, (uint8_t*) &cmd , cmd.used_data+1);
		udp_socket_sendto(&st_udp_mle_socket, (uint8_t*) &cmd , cmd.used_data+1 , &s_destAddr,MLE_UDP_RPORT);


		/************************************************************************/


		PRINTF( "\nMLE command  sent to : " );
		PRINT6ADDR(&s_destAddr);
		PRINTF( "\n" );
		etimer_restart(&e_mle_udp_Timer);
	}



} /* _mle_sendMsg */


static uint8_t join_mcast_group(void)
{
	uip_ipaddr_t addr;

	uip_ip6addr(&addr, 0xff02, 0, 0, 0, 0, 0, 0x1, 0x0001);


	if(uip_ds6_maddr_add(&addr)!= NULL) {
		PRINTF("\nJoined multicast group: ");
		PRINT6ADDR(&uip_ds6_maddr_lookup(&addr)->ipaddr);
		PRINTF("\n");
		return 1;
	}
	else
	{PRINTF("unable to join the multicast group ...\n");
	return 0 ;}

}



static void print_local_addresses(void)
{
	int i;
	uint8_t state;

	PRINTF("Client IPv6 addresses: ");
	for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if(uip_ds6_if.addr_list[i].isused &&
				(state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
			PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
			PRINTF("\n");
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

	/* PS: All Thread interfaces MUST subscribe to the Link-Local All Nodes multicast address (FF02::1).
	 * All Thread interfaces operating in a Router, REED, or Border Router mode MUST subscribe to
	 * the Link-Local All Routers multicast address (FF02::2)
	 */

	//if(!join_mcast_group()) {PRINTF("Failed to join multicast group.\n");}

	/***********  Inisialize  mle udp connexion ***************/

	/* set the pointer to the udp-socket */
	uip_ip6addr(&s_destAddr, 0xff02, 0, 0, 0, 0, 0, 0, 0x0001);

	udp_socket_register(&st_udp_mle_socket, NULL, _mle_call_back);
	udp_socket_bind(&st_udp_mle_socket, MLE_UDP_LPORT);
	udp_socket_connect(&st_udp_mle_socket,NULL, MLE_UDP_RPORT);

	if (st_udp_mle_socket.udp_conn == NULL)
	{
		PRINTF("No UDP connection available, error!\n");
		return 0;
	}

	PRINTF("\nMLE UDP : \n lport --> %u \n rport --> %u \n", UIP_HTONS(st_udp_mle_socket.udp_conn->lport),
			UIP_HTONS(st_udp_mle_socket.udp_conn->rport));




	/***********  Inisialize  mle node structure ***************/
	node.mode=NOT_LINKED;


	//etimer_set( &e_mle_udp_Timer,SEND_INTERVAL * bsp_get(E_BSP_GET_TRES), _mle_sendMsg);



	PRINTF("MLE protocol initialized ...\n");

	return 1;

};

