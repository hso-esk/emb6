/*
 * mle_table.c
 *
 *  Created on: 27 Apr 2016
 *      Author: Nidhal Mars
 *      manipulate child and routers neighbors table
 */

#include "mle_table.h"
#include "clist.h"
#include "memb.h"
#include "uip-debug.h"


LIST(nb_router_list);
MEMB(nb_router_memb, mle_neighbor_node_t, MAX_NB_ROUTER);


LIST(childs_list);
MEMB(childs_memb, mle_neighbor_node_t, MAX_CHILD);


/******  functions to manipulate the neighbors list   *******/

void mle_nb_device_init(void)
{
	memb_init(&nb_router_memb);
	list_init(nb_router_list);

	memb_init(&childs_memb);
	list_init(childs_list);
}


static mle_neighbor_node_t * mle_neigbhor_head(list_t* list)
{
	return list_head(*list);
}

 mle_neighbor_node_t * mle_childs_head(void)
{
	return mle_neigbhor_head(&childs_list);
}
 mle_neighbor_node_t * mle_nb_router_head(void)
{
	return mle_neigbhor_head(&nb_router_list);
}




static mle_neighbor_node_t * mle_find_neigbhor(list_t *list, uint8_t id)
{
	mle_neighbor_node_t *nb;

	for (nb = mle_neigbhor_head(list); nb != NULL; nb = list_item_next(nb))
	{
		if (nb->id == id)
			return nb;

	}
	return NULL ;
}

 mle_neighbor_node_t * mle_find_nb_router(uint8_t id)
{
	return mle_find_neigbhor(&nb_router_list,  id) ;
}
 mle_neighbor_node_t * mle_find_child( uint8_t id)
{
	return mle_find_neigbhor(&childs_list,  id) ;
}



static mle_neighbor_node_t * mle_add_neigbhor(struct memb* m , list_t *list , uint8_t id, uip_ipaddr_t  address, uint32_t  MLEFrameCounter ,
		 	 	 	 	 	 	 	 	 	 	 	 uint8_t modeTLV, uint8_t  linkQuality)
{
	mle_neighbor_node_t *nb;

	/* find the neighbor by id */

	nb = mle_find_neigbhor(list, id);

	/* Check whether the given router id already has an entry in the Router ID Set. */
	if (nb == NULL /*don't forget to check the max size */)
	{

		/* Allocate a router id entry and populate it. */
		nb = memb_alloc(m);

		if (nb == NULL) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			PRINTF("Could not allocate child id\n");
			return NULL;
		}

		nb->id = id;
		uip_ip6addr(&nb->address, 0xff02, 0, 0, 0, 0, 0, 0, 0x0001);

		/* printf("Neighbor addresss  \n\r");
		PRINTLLADDR(&(nb->address));
		PRINT6ADDR(&(nb->address));
		PRINTLLADDR(nb->address);
		PRINT6ADDR(nb->address);printf("   \n\r"); */

		//uip_ip6addr_copy(&nb->address ,&address);
		nb->MLEFrameCounter = MLEFrameCounter;
		nb->linkQuality=linkQuality;
		nb->modeTLV = modeTLV;

		/* add the neighbor */
		list_push(*list, nb);

		PRINTF("Neighbor added  : id = %d \n\r", id);

		// if i will use a counter a should increment it
	}
	else
	{
		PRINTF( ANSI_COLOR_RED "Neighbor already exist \n");
		return NULL;
	}
	return nb;
}

 mle_neighbor_node_t * mle_add_child(uint8_t id, uip_ipaddr_t  address, uint32_t  MLEFrameCounter ,
		 	 	 	 	 	 	 	 	 	 	 	 uint8_t modeTLV, uint8_t  linkQuality)
{
return mle_add_neigbhor(&childs_memb , &childs_list ,  id,   address,   MLEFrameCounter , modeTLV,   linkQuality);
}

 mle_neighbor_node_t * mle_add_nb_router(uint8_t id, uip_ipaddr_t  address, uint32_t  MLEFrameCounter ,
		 	 	 	 	 	 	 	 	 	 	 	 uint8_t modeTLV, uint8_t  linkQuality)
{
return mle_add_neigbhor(&nb_router_memb ,  &nb_router_list ,  id,   address,   MLEFrameCounter , modeTLV,   linkQuality);
}



static uint8_t mle_rm_neigbhor(struct memb* m , list_t *list ,mle_neighbor_node_t *nb)
{
	if (nb != NULL) {
		/* Remove the router id from the Router ID Set. */
		PRINTF( ANSI_COLOR_RED "removing Neighbor: id = %d \n",nb->id);
		list_remove(*list, nb);
		memb_free(m, nb);
		PRINTF("\n\r");
	}
	return 0 ;
}

 uint8_t mle_rm_nb_router(mle_neighbor_node_t *nb)
{
	return mle_rm_neigbhor(&nb_router_memb ,  &nb_router_list, nb) ;
}

 uint8_t mle_rm_child( mle_neighbor_node_t *nb)
{
	return mle_rm_neigbhor(&childs_memb , &childs_list, nb) ;
}


 void mle_print_table(void)
 {
	 mle_neighbor_node_t *i;
 	printf(ANSI_COLOR_RED
 			"|=========================== NEIGHBOR TABLE  ===========================|"
 			ANSI_COLOR_RESET "\n\r");
 	printf("| id | modeTLV | MLEFrameCounter | ipv6 addes |\n");
 	printf("------------------------------------\n\r");
 	for (i = mle_neigbhor_head(&childs_list); i != NULL; i = list_item_next(i)) {
 		printf("| " ANSI_COLOR_YELLOW "%2d" ANSI_COLOR_RESET
 				" | " ANSI_COLOR_YELLOW "%7d" ANSI_COLOR_RESET
 				" | " ANSI_COLOR_YELLOW "%15d" ANSI_COLOR_RESET
 				" |", i->id, i->modeTLV, i->MLEFrameCounter);
 		///PRINT6ADDR(&i->address);
 		//PRINTLLADDR(&i->address);
 		printf("|\n");
 	}
 	printf("------------------------------------\n\r");
 	printf(ANSI_COLOR_RED
 			"|=======================================================================|"
 			ANSI_COLOR_RESET "\n\r");
 }
