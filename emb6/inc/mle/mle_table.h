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



#define     MAX_NB_ROUTER               9
#define     MAX_CHILD                   5

typedef enum {PENDING, LINKED} nb_state_t;


typedef struct nodemle {
	struct nodemle 		*next;
	uint8_t 			       		 id;

	nb_state_t 			       		 state; // state of the neighbor PENDING or LINKED
	uint32_t         			     challenge;	// but length is variable for other device !!!
	uint16_t         			     address16;	// short address


   uip_ipaddr_t                tmp; // to remove because it is generated from address 16



	struct ctimer        			 timeOut;
	uint32_t            			 MLEFrameCounter;
	uint8_t 						 modeTLV;
	uint8_t 						 LQ;
	uint8_t 						 rec_rssi;
}mle_neighbor_t ;




/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

void mle_nb_device_init(void);

mle_neighbor_t * mle_childs_head(void);
mle_neighbor_t * mle_nb_router_head(void);

mle_neighbor_t * mle_find_nb_router(uint8_t id);
mle_neighbor_t * mle_find_child( uint8_t id);

mle_neighbor_t * mle_find_nb_router_byAdd(uip_ipaddr_t* address);
mle_neighbor_t * mle_find_child_byAdd( uip_ipaddr_t* address);

mle_neighbor_t * mle_add_child(uint8_t id, uint16_t  address, uint32_t  MLEFrameCounter , uint8_t modeTLV, uint8_t  linkQuality);
mle_neighbor_t * mle_add_nb_router(uint8_t id, uint16_t address, uint32_t  MLEFrameCounter , uint8_t modeTLV, uint8_t  linkQuality);

uint8_t mle_rm_nb_router(mle_neighbor_t *nb);
uint8_t mle_rm_child( mle_neighbor_t *nb);
void mle_rm_all_nb_router(void);

uint8_t count_neighbor_LQ(uint8_t N ); // The number of neighboring device with which the sender shares a link of quality N

extern bool mle_is_child(uint8_t rloc16);

void  mle_print_child_table(void);
void  mle_print_nb_router_table(void);

#endif /* EMB6_INC_MLE_MLE_TABLE_H_ */
