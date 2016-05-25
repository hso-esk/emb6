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

#define		TIME_OUT		 		     10     /* time out   */
#define		IS_RX_ON_WHEN_IDLE		 	  1     /* Set to ‘1’ if the sender has its receiver on when not transmitting   */
#define		IS_FFD		 	     		  1
/*==============================================================================
									TYPEDEFS
 =============================================================================*/

typedef struct {
	mle_cmd_t * rec_cmd;
	uint8_t    rec_rssi; // incoming rssi
	uip_ipaddr_t *source_addr;
}mle_param_t ;

typedef enum
{
	NOT_LINKED,
	CHILD,
	PARENT,
} mle_mode_t;


typedef struct {
	struct udp_socket     udp_socket;
	mle_mode_t     		  OpMode;                               /**< device operating mode */
	uint32_t 			  timeOut;
	uint32_t              thrMLEFrameCounter;                   /**< Outgoing frame counter */
	uint8_t 			  NB_router_couter;
	uint8_t 			  childs_counter;
 	uint32_t 			  jp_challenge ;  					    /**< current join process challenge      */
	uint8_t               rx_on_when_idle; 					    /**< Set to ‘1’ if the sender's receiver on when not transmitting; used for mode TLV  */
}mle_node_t ;


typedef struct {
 	uint32_t 	jp_challenge ;
	uint8_t  	two_way_LQ  ;
	uint8_t  	is_Router  ;  // the parent candidate is active router
	uint8_t  	LQ3  ; // LQ3 from connectivity
}mle_parent_t ;  // store best parent candidate

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

uint8_t mle_init(void);
uint8_t mle_set_parent_mode(void);
uint8_t mle_set_child_mode(void);



/*
static uint8_t mle_build_link_request_cmd(mle_cmd_t* cmd);
static uint8_t mle_build_link_accept_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_link_accept_request_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_link_reject_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_advertisement_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_data_request_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_data_response_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_parent_request_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_parent_response_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_child_id_request_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_child_id_response_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_child_update_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_child_update_response_cmd(mle_cmd_t* mle_cmd);



static uint8_t mle_link_request_to_routers(int8_t pan_id);
static uint8_t mle_parent_request(int8_t nwk_id , uint8_t Scan_Mask);*/

#endif /* __MLE_MANAGEMENT_H_ */
