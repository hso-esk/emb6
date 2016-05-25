/*
 * mle_table.h
 *
 *  Created on: 27 Apr 2016
 *      Author: Nidhal Mars
 *      manipulate child and routers neighbors table
 */

#ifndef EMB6_INC_MLE_MLE_TABLE_H_
#define EMB6_INC_MLE_MLE_TABLE_H_

#include "ctimer.h"
#include "tcpip.h"



#define     MAX_NB_ROUTER               5
#define     MAX_CHILD                   7




typedef struct nodemle {
	struct nodemle 		*next;
	uint8_t 			       		 id;

	uint8_t 			       		 state; // pending // linked // ....
// should add challenge for the child : we need it in pending state : but length is variable for other device !!!

	uip_ipaddr_t         			 address;						      /**< link-local address */
	struct ctimer        			 timeOut;
	uint32_t            			 MLEFrameCounter;
	uint8_t 						 modeTLV;
	uint8_t 						 linkQuality;
}mle_neighbor_node_t ;




/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

void mle_nb_device_init(void);

mle_neighbor_node_t * mle_childs_head(void);
mle_neighbor_node_t * mle_nb_router_head(void);

mle_neighbor_node_t * mle_find_nb_router(uint8_t id);
mle_neighbor_node_t * mle_find_child( uint8_t id);

mle_neighbor_node_t * mle_add_child(uint8_t id, uip_ipaddr_t * address, uint32_t  MLEFrameCounter , uint8_t modeTLV, uint8_t  linkQuality);
mle_neighbor_node_t * mle_add_nb_router(uint8_t id, uip_ipaddr_t * address, uint32_t  MLEFrameCounter , uint8_t modeTLV, uint8_t  linkQuality);

uint8_t mle_rm_nb_router(mle_neighbor_node_t *nb);
uint8_t mle_rm_child( mle_neighbor_node_t *nb);

uint8_t count_neighbor_LQ(uint8_t N ); // The number of neighboring device with which the sender shares a link of quality N

void  mle_print_child_table(void);
void  mle_print_nb_router_table(void);

#endif /* EMB6_INC_MLE_MLE_TABLE_H_ */
