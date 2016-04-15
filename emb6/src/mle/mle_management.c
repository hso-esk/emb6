
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
static  struct  udp_socket          *pst_udp_mle_socket;

static  uip_ipaddr_t               s_destAddr;
struct  etimer                      e_mle_udp_Timer;


static char buf[MAX_MLE_UDP_PAYLOAD_LEN];


/** set the send interval */
#define     SEND_INTERVAL                7

/*==============================================================================
								LOCAL FUNCTION
 =============================================================================*/
static void  _mle_call_back(struct udp_socket *c, void *ptr, const uip_ipaddr_t *source_addr, uint16_t source_port,
									const uip_ipaddr_t *dest_addr, uint16_t dest_port, const uint8_t *data, uint16_t datalen)
{
	//PRINTF("data lenght : %i ", datalen);
	PRINTF("MLE receive : %s from : ", (char *)data);
	PRINT6ADDR(source_addr);
	PRINTF( "\n" );
}


static void _mle_sendMsg(c_event_t c_event, p_data_t p_data )
{

static uint32_t seqId=0;
	if (etimer_expired(&e_mle_udp_Timer))
	{
           sprintf(buf, "MLE COMMAND %u", seqId++ );
           seqId%=200;

          // udp_socket_send(pst_udp_mle_socket, buf, strlen(buf));
           uip_ip6addr(&s_destAddr, 0xff02, 0, 0, 0, 0, 0, 0, 0x0001);

         //  udp_socket_sendto(pst_udp_mle_socket, buf, strlen(buf) , &s_destAddr,MLE_UDP_RPORT);
           udp_socket_sendto(pst_udp_mle_socket, "bonjour", 8 , &s_destAddr,MLE_UDP_RPORT);

           PRINTF( "MLE command  sent to : " );
          // PRINT6ADDR(&pst_udp_mle_socket->udp_conn->ripaddr); s_destAddr
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
pst_udp_mle_socket = &st_udp_mle_socket;

/*  register the udp-socket and connect  */
udp_socket_register(pst_udp_mle_socket, NULL, _mle_call_back);
udp_socket_bind(pst_udp_mle_socket , MLE_UDP_LPORT );
udp_socket_connect(pst_udp_mle_socket, NULL, MLE_UDP_RPORT);


if (pst_udp_mle_socket->udp_conn == NULL)
{
  PRINTF("No UDP connection available, error!\n");
  return 0;
}

PRINTF("\nMLE UDP : \n lport --> %u \n rport --> %u \n", UIP_HTONS(pst_udp_mle_socket->udp_conn->lport),
														 UIP_HTONS(pst_udp_mle_socket->udp_conn->rport));

/*
memcmp(&pst_udp_mle_socket->udp_conn->ripaddr, &s_destAddr, IP6ADDRSIZE));
    udp_socket_close(pst_udp_mle_socket);
 #define uip_create_linklocal_allnodes_mcast(a) uip_ip6addr(a, 0xff02UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0x0001UL)
 #define uip_create_linklocal_allrouters_mcast(a) uip_ip6addr(a, 0xff02UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0x0002UL)*/



/***********  Inisialize  mle node structure ***************/
node.mode=NOT_LINKED;

etimer_set( &e_mle_udp_Timer,SEND_INTERVAL * bsp_get(E_BSP_GET_TRES), _mle_sendMsg);






PRINTF("MLE protocol initialized ...\n");

return 1;

};

