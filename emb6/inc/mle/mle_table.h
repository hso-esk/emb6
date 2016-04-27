/*
 * mle_table.h
 *
 *  Created on: 27 Apr 2016
 *      Author: Nidhal Mars
 *      manipulate child and routers neighbors table
 */

#ifndef EMB6_INC_MLE_MLE_TABLE_H_
#define EMB6_INC_MLE_MLE_TABLE_H_

#include "etimer.h"
#include "tcpip.h"

/** Debug print colors. */
#define ANSI_COLOR_RED     	"\x1b[31m"
#define ANSI_COLOR_YELLOW  	"\x1b[33m"
#define	ANSI_COLOR_GREEN	"\x1B[32m"
#define ANSI_COLOR_CYAN    	"\x1b[36m"
#define ANSI_COLOR_MAGENTA 	"\x1b[35m"
#define ANSI_COLOR_RESET  	"\x1b[0m"



#define     MAX_NB_ROUTER               5
#define     MAX_CHILD                   7




typedef struct {
	struct mle_neighbor_node_t 		*next;
	uint8_t 			       		 id;
	uip_ipaddr_t         			 address;						      /**< link-local address */
	struct etimer        			 timeOut;
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

mle_neighbor_node_t * mle_add_child(uint8_t id, uip_ipaddr_t  address, uint32_t  MLEFrameCounter , uint8_t modeTLV, uint8_t  linkQuality);
mle_neighbor_node_t * mle_add_nb_router(uint8_t id, uip_ipaddr_t  address, uint32_t  MLEFrameCounter , uint8_t modeTLV, uint8_t  linkQuality);

uint8_t mle_rm_nb_router(mle_neighbor_node_t *nb);
uint8_t mle_rm_child( mle_neighbor_node_t *nb);


void mle_print_table(void);

#endif /* EMB6_INC_MLE_MLE_TABLE_H_ */
