/*
 * mle_management.h
 *
 *      Author: Nidhal Mars
 *      manage mle protcol
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


/******************* TLV parameters *****************************/

#define		TIME_OUT		 		     30     /* time out   */
#define		IS_RX_ON_WHEN_IDLE		 	  1     /* Set to ‘1’ if the sender has its receiver on when not transmitting   */
#define		IS_FFD		 	     		  1
/*==============================================================================
									TYPEDEFS
 =============================================================================*/

typedef struct {
	mle_cmd_t * rec_cmd;
	uint8_t    rec_rssi; // incoming rssi
	uip_ipaddr_t  source_addr;
}mle_param_t ;

typedef enum
{
	NOT_LINKED,
	CHILD,
	PARENT,
} mle_mode_t;

typedef enum {
	JP_SEND_MCAST_PR_TO_ROUTER,
	JP_SEND_MCAST_PR_TO_ROUTER_REED,
	JP_PARENT_SELECT,
	JP_SEND_CHILD_REQ,
	JP_SAVE_PARENT,
	JP_FAIL,
}jp_state_t; // join process state

typedef enum {
	SYN_SEND_LINK_REQUEST,
	SYN_PROCESS_LINK,
}syn_state_t; // synchronisation state

typedef struct {
	struct udp_socket     udp_socket;
	mle_mode_t     		  OpMode;                               /**< device operating mode */
	uint32_t 			  timeOut;
	uint32_t              thrMLEFrameCounter;                   /**< Outgoing frame counter */
	uint8_t 			  NB_router_couter;
	uint8_t 			  childs_counter;
 	uint32_t 			  challenge ;  					        /**< current join/synchronization process challenge      */
	uint8_t               rx_on_when_idle; 					    /**< Set to ‘1’ if the sender's receiver on when not transmitting; used for mode TLV  */
}mle_node_t ;


typedef struct {
	uint8_t  	LQ  ;
	uint8_t  	is_Router  ;  // the parent candidate is active router
	uint8_t  	LQ3  ; // LQ3 from connectivity
}mle_parent_t ;  // store best parent candidate

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

uint8_t mle_init(void);
void mle_synchro_process(void *ptr);
uint8_t send_mle_advertisement(tlv_route64_t* route, uint8_t len, tlv_leader_t* lead);
uint8_t mle_set_parent_mode(uint8_t router_id);
uint8_t mle_set_child_mode(uint16_t rloc16);
void reply_for_mle_childID_request(void *ptr);


/*

static uint8_t mle_link_request_to_routers(int8_t pan_id);
static uint8_t mle_parent_request(int8_t nwk_id , uint8_t Scan_Mask);*/

#endif /* __MLE_MANAGEMENT_H_ */
